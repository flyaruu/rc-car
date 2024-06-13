#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(result_flattening)]

extern crate alloc;
use core::mem::MaybeUninit;


use alloc::boxed::Box;
use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::Timer;

use esp_backtrace as _;

use esp_hal_embassy::{init, Executor};
use esp_println::println;
use esp_wifi::{EspWifiInitFor, initialize, esp_now::EspNow};
use hal::{clock::ClockControl, gpio::{AnyInput, AnyOutput, Io, Level, Pull}, peripherals::Peripherals, prelude::*, rng::Rng, rtc_cntl::Rtc, system::SystemControl, timer::{systimer::SystemTimer, timg::TimerGroup}};

use log::info;
use protocol::{ControlMessage, BlinkerState, Headlights, MessageChannel, MessagePublisher, Message};


mod steering;
mod telemetry;
mod net;
use esp_backtrace as _;


use crate::{net::{receiver, sender}, steering::{rotary_steering, rotary_motor}, telemetry::{connection_state, telemetry_receiver}};


#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();
    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}


#[entry]
fn main() -> ! {
    init_heap();
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    // let mut delay = Delay::new(&clocks);
    let rtc = Box::leak(Box::new(Rtc::new(peripherals.LPWR,None)));

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger(log::LevelFilter::Info);
    log::info!("Logger is setup....");

    let io = Io::new(peripherals.GPIO,peripherals.IO_MUX);
    let executor = Box::leak(Box::new(Executor::new()));
    let timer_group = TimerGroup::new_async(peripherals.TIMG0, &clocks);    

    let rotary_pin_x_a = AnyInput::new(io.pins.gpio6, Pull::Up);
    let rotary_pin_x_b = AnyInput::new(io.pins.gpio4, Pull::Up);

    let rotary_pin_y_a = AnyInput::new(io.pins.gpio18, Pull::Up);
    let rotary_pin_y_b = AnyInput::new(io.pins.gpio19, Pull::Up);

    let button_pin_x = AnyInput::new(io.pins.gpio5, Pull::Up);
    let button_pin_y = AnyInput::new(io.pins.gpio9, Pull::Up);

    let button_pin_top_left = AnyInput::new(io.pins.gpio7, Pull::Up);
    let button_pin_top_right = AnyInput::new(io.pins.gpio8, Pull::Up);

    let status_pin = AnyOutput::new(io.pins.gpio3, Level::Low);
    println!("Embassy init starting");

    init(&clocks,timer_group);
    info!("Embassy init done");
    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();
    let wifi = peripherals.WIFI;
    let esp_now = EspNow::new(&init, wifi).unwrap();

    hal::interrupt::enable(hal::peripherals::Interrupt::GPIO, hal::interrupt::Priority::Priority1).unwrap();
    let command_channel: MessageChannel = MessageChannel::new();
    let command_channel = Box::leak(Box::new(command_channel));
    let (_esp_manager, esp_sender, esp_receiver) = esp_now.split();
    let heartbeat_signal: &mut Signal<NoopRawMutex,u64> = Box::leak(Box::new(Signal::new()));

    executor.run(|spawner| {
        spawner.spawn(sender(esp_sender,command_channel.subscriber().unwrap())).unwrap();
        spawner.spawn(receiver(esp_receiver,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(telemetry_receiver(command_channel.subscriber().unwrap(),rtc,heartbeat_signal)).unwrap();
        spawner.spawn(connection_state(heartbeat_signal,rtc,status_pin)).unwrap();
        spawner.spawn(rotary_steering(rotary_pin_x_a,rotary_pin_x_b,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(rotary_motor(rotary_pin_y_a,rotary_pin_y_b,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(indicator_buttons(button_pin_x,button_pin_y,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(button_top_left(button_pin_top_left,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(button_top_right(button_pin_top_right,command_channel.publisher().unwrap())).unwrap();

    })
}


#[embassy_executor::task]
async fn indicator_buttons(mut left_button_pin: AnyInput<'static>, mut right_button_pin: AnyInput<'static>, publisher: MessagePublisher) {
    let mut blinker_state  = BlinkerState::Off;
    loop {
        match select(left_button_pin.wait_for_rising_edge(),right_button_pin.wait_for_rising_edge()).await {
            Either::First(_)=>{
                blinker_state = match blinker_state {
                    BlinkerState::Left => BlinkerState::Off,
                    _ => BlinkerState::Left,
                };
            },
            Either::Second(_)=>{
                blinker_state = match blinker_state {
                    BlinkerState::Right => BlinkerState::Off,
                    _ => BlinkerState::Right,
                };
        
            }
        }
        publisher.publish(Message::Control(ControlMessage::BlinkerCommand(blinker_state))).await;
        Timer::after_millis(200).await;
    }
}

#[embassy_executor::task]
async fn button_top_left(mut button_pin: AnyInput<'static>, publisher: MessagePublisher) {
    let mut light_state: Headlights = Headlights::Off;
    loop {
        button_pin.wait_for_rising_edge().await;
        light_state = match light_state {
            Headlights::Low => Headlights::High,
            Headlights::High => Headlights::Off,
            Headlights::Off => Headlights::Low,
        };
        info!("Sending headlight command: {:?}",light_state);
        publisher.publish(Message::Control(ControlMessage::HeadlightCommand(light_state))).await;
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn button_top_right(mut button_pin: AnyInput<'static>, publisher: MessagePublisher) {
    loop {
        button_pin.wait_for_rising_edge().await;
        info!("Recalibrating motor");
        publisher.publish(Message::Control(ControlMessage::RecalibrateMotor)).await;
        Timer::after_millis(100).await;
    }
}