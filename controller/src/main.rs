#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(result_flattening)]

extern crate alloc;
use core::mem::MaybeUninit;
use embassy_executor::Executor;
use embassy_futures::select::select;
use embassy_sync::{channel::{Receiver, Sender, Channel}, blocking_mutex::raw::NoopRawMutex};
use embassy_time::Timer;
use embedded_hal::digital::InputPin;
// use embedded_hal::digital::{OutputPin, InputPin};
use embedded_hal_async::digital::Wait;
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::{EspWifiInitFor, initialize, esp_now::{EspNow, EspNowReceiver, BROADCAST_ADDRESS, EspNowSender}};
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, IO, timer::TimerGroup, embassy, systimer::SystemTimer, Rng, gpio::{Gpio4, Input, PullUp, Gpio5, Output, Gpio3, Gpio19, Gpio18, Gpio9}};

use log::{info, warn};
use protocol::{ControlMessage, TelemetryMessage, Axis};
use rotary_encoder_hal::Rotary;
use static_cell::make_static;



use esp_backtrace as _;

// use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState};


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

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup");

    let io = IO::new(peripherals.GPIO,peripherals.IO_MUX);

    let executor = make_static!(Executor::new());
    let timer_group = TimerGroup::new(peripherals.TIMG0, &clocks);    

    let rotary_pin_x_a = io.pins.gpio3.into_pull_up_input();
    let rotary_pin_x_b = io.pins.gpio4.into_pull_up_input();

    let rotary_pin_y_a = io.pins.gpio18.into_pull_up_input();
    let rotary_pin_y_b = io.pins.gpio19.into_pull_up_input();

    let button_pin_x = io.pins.gpio5.into_pull_up_input();
    let button_pin_y = io.pins.gpio9.into_pull_up_input();

    embassy::init(&clocks,timer_group.timer0);

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
    hal::interrupt::enable(hal::peripherals::Interrupt::GPIO, hal::interrupt::Priority::Priority1).unwrap();

    let command_channel: Channel<NoopRawMutex, ControlMessage, 5> = Channel::new();
    let command_channel = make_static!(command_channel);
    let esp_now = EspNow::new(&init, wifi).unwrap();
    let (_esp_manager, esp_sender, esp_receiver) = esp_now.split();

    executor.run(|spawner| {
        spawner.spawn(sender(esp_sender,command_channel.receiver())).unwrap();
        spawner.spawn(telemetry_receiver(esp_receiver)).unwrap();
        spawner.spawn(rotary_x(rotary_pin_x_a,rotary_pin_x_b,command_channel.sender())).unwrap();
        spawner.spawn(rotary_y(rotary_pin_y_a,rotary_pin_y_b,command_channel.sender())).unwrap();
        spawner.spawn(button_x(button_pin_x,command_channel.sender())).unwrap();
        spawner.spawn(button_y(button_pin_y,command_channel.sender())).unwrap();
        spawner.spawn(debug_command(command_channel.receiver())).unwrap();

    })
}

#[embassy_executor::task]
async fn debug_command(receiver: Receiver<'static, NoopRawMutex, ControlMessage,5>) {
    loop {
        let msg = receiver.receive().await;
        println!("Message received: {:?}",msg);

    }
}

#[embassy_executor::task]
async fn rotary_x(pin_a: Gpio3<Input<PullUp>>,pin_b: Gpio4<Input<PullUp>>, sender: Sender<'static, NoopRawMutex, ControlMessage,5>) {
    rotary(pin_a, pin_b, Axis::X, sender).await
}

#[embassy_executor::task]
async fn rotary_y(pin_a: Gpio18<Input<PullUp>>,pin_b: Gpio19<Input<PullUp>>, sender: Sender<'static, NoopRawMutex, ControlMessage,5>) {
    rotary(pin_a, pin_b, Axis::Y, sender).await
}


#[embassy_executor::task]
async fn button_x(button_pin: Gpio5<Input<PullUp>>, sender: Sender<'static, NoopRawMutex, ControlMessage,5>) {
    button(button_pin, Axis::X, sender).await;
}

#[embassy_executor::task]
async fn button_y(button_pin: Gpio9<Input<PullUp>>, sender: Sender<'static, NoopRawMutex, ControlMessage,5>) {
    button(button_pin, Axis::Y, sender).await;
}


async fn rotary<A: InputPin + Wait, B: InputPin + Wait>(pin_a: A, pin_b: B, axis: Axis, sender: Sender<'static, NoopRawMutex, ControlMessage,5>)->! {
    let mut rotary = Rotary::new(pin_a, pin_b);
    let mut count = 0_i32;
    loop {
        info!("Starting rotary for axis: {:?}",axis);
        let (pin_a,pin_b) = rotary.pins();
        select(pin_a.wait_for_any_edge(),pin_b.wait_for_any_edge()).await;
        let direction = rotary.update().unwrap();
        match direction {
            rotary_encoder_hal::Direction::Clockwise => {
                count+=1;
                sender.send(ControlMessage::Value(axis.clone(), count)).await;
            },
            rotary_encoder_hal::Direction::CounterClockwise => {
                count-=1;
                sender.send(ControlMessage::Value(axis.clone(), count)).await;            
            },
            rotary_encoder_hal::Direction::None => (),
        }
        println!("Count: {}",count);
    }
}

async fn button<A: InputPin + Wait>(mut button_pin: A, axis: Axis, sender: Sender<'static, NoopRawMutex, ControlMessage,5>) {
    let mut was_high = true;
    loop {
        button_pin.wait_for_any_edge().await.unwrap();
        let is_high = button_pin.is_high().unwrap();
        match (is_high,was_high) {
            (true, true) => {},
            (true, false) => {sender.send(ControlMessage::Release(axis)).await},
            (false, true) => {sender.send(ControlMessage::Press(axis)).await},
            (false, false) => {},
        }
        was_high = is_high;
    }

}


#[embassy_executor::task]
async fn telemetry_receiver(mut esp_receiver: EspNowReceiver<'static>) {
    info!("Starting receiver...");
    let msg = TelemetryMessage::from_slice(&esp_receiver.receive_async().await.data);
    if msg.missed {
        warn!("Car reports missed messages!");
    } else {
        info!("all good");        
    }

}
#[embassy_executor::task]
async fn sender( mut esp_sender: EspNowSender<'static>, receiver: Receiver<'static, NoopRawMutex, ControlMessage,5>) {
    info!("Starting sender...");
    loop {
        let message = receiver.receive().await;
        esp_sender.send_async(&BROADCAST_ADDRESS, &message.to_bytes()).await.unwrap();
    }
}