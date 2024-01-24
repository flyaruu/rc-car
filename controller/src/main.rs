#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(result_flattening)]

extern crate alloc;
use core::mem::MaybeUninit;
use embassy_executor::Executor;


use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal, pubsub::PubSubChannel};
use embassy_time::Timer;

use embedded_hal_async::digital::Wait;
use esp_backtrace as _;

use esp_println::println;
use esp_wifi::{EspWifiInitFor, initialize, esp_now::EspNow};
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, IO, timer::TimerGroup, embassy, systimer::SystemTimer, Rng, Rtc};

use log::info;
use protocol::{ControlMessage, BlinkerState, Headlights, MessageChannel, MessagePublisher, Message};

use static_cell::make_static;

mod steering;
mod telemetry;
mod net;
mod types;
use esp_backtrace as _;
use types::{LeftButtonPin, RightButtonPin, TopLeftButtonPin, TopRightButtonPin};


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
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    // let mut delay = Delay::new(&clocks);
    let rtc = make_static!(Rtc::new(peripherals.RTC_CNTL));

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger(log::LevelFilter::Error);
    log::info!("Logger is setup....");

    let io = IO::new(peripherals.GPIO,peripherals.IO_MUX);
    let executor = make_static!(Executor::new());
    let timer_group = TimerGroup::new(peripherals.TIMG0, &clocks);    

    let rotary_pin_x_a = io.pins.gpio6.into_pull_up_input();
    let rotary_pin_x_b = io.pins.gpio4.into_pull_up_input();

    let rotary_pin_y_a = io.pins.gpio18.into_pull_up_input();
    let rotary_pin_y_b = io.pins.gpio19.into_pull_up_input();

    let button_pin_x = io.pins.gpio5.into_pull_up_input();
    let button_pin_y = io.pins.gpio9.into_pull_up_input();

    let button_pin_top_left = io.pins.gpio7.into_pull_up_input();
    let button_pin_top_right = io.pins.gpio8.into_pull_up_input();

    let status_pin = io.pins.gpio3.into_push_pull_output();
    println!("Embassy init starting");

    embassy::init(&clocks,timer_group.timer0);
    info!("Embassy init done");
    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();
    let wifi = peripherals.WIFI;
    let esp_now = EspNow::new(&init, wifi).unwrap();

    hal::interrupt::enable(hal::peripherals::Interrupt::GPIO, hal::interrupt::Priority::Priority1).unwrap();
    let command_channel: MessageChannel = PubSubChannel::new();
    let command_channel = make_static!(command_channel);
    let (_esp_manager, esp_sender, esp_receiver) = esp_now.split();
    let heartbeat_signal: &mut Signal<NoopRawMutex,u64> = make_static!(Signal::new());

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
async fn indicator_buttons(mut left_button_pin: LeftButtonPin, mut right_button_pin: RightButtonPin, publisher: MessagePublisher) {
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
async fn button_top_left(mut button_pin: TopLeftButtonPin, publisher: MessagePublisher) {
    let mut light_state: Headlights = Headlights::Off;
    loop {
        button_pin.wait_for_rising_edge().await.unwrap();
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
async fn button_top_right(mut button_pin: TopRightButtonPin, publisher: MessagePublisher) {
    loop {
        button_pin.wait_for_rising_edge().await.unwrap();
        info!("Recalibrating motor");
        publisher.publish(Message::Control(ControlMessage::RecalibrateMotor)).await;
        Timer::after_millis(100).await;
    }
}