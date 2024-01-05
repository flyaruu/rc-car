#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(result_flattening)]

extern crate alloc;
use core::mem::MaybeUninit;
use embassy_executor::Executor;

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal, pubsub::{PubSubChannel, Subscriber, Publisher}};
use embassy_time::Timer;
use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;
use esp_backtrace as _;

use esp_wifi::{EspWifiInitFor, initialize, esp_now::{EspNow, BROADCAST_ADDRESS, EspNowSender}};
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, IO, timer::TimerGroup, embassy, systimer::SystemTimer, Rng, gpio::{Input, PullUp, Gpio5, Gpio9}, Rtc};

use log::info;
use protocol::{ControlMessage, Axis, BlinkerState};

use static_cell::make_static;


mod steering;
mod telemetry;

use esp_backtrace as _;


use crate::{telemetry::{connection_state, telemetry_receiver}, steering::rotary_steering_x};

// use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState};
const MAX_PUBS: usize = 5;
const MAX_SUBS: usize = 5;
const MAX_MESSAGES: usize = 10;

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
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup...");

    let io = IO::new(peripherals.GPIO,peripherals.IO_MUX);

    let executor = make_static!(Executor::new());
    let timer_group = TimerGroup::new(peripherals.TIMG0, &clocks);    

    let rotary_pin_x_a = io.pins.gpio6.into_pull_up_input();
    let rotary_pin_x_b = io.pins.gpio4.into_pull_up_input();

    // let rotary_pin_y_a = io.pins.gpio18.into_pull_up_input();
    // let rotary_pin_y_b = io.pins.gpio19.into_pull_up_input();

    let button_pin_x = io.pins.gpio5.into_pull_up_input();
    let button_pin_y = io.pins.gpio9.into_pull_up_input();

    let status_pin = io.pins.gpio3.into_push_pull_output();

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
    let command_channel: PubSubChannel<NoopRawMutex, ControlMessage, MAX_MESSAGES,MAX_SUBS,MAX_PUBS> = PubSubChannel::new();
    let command_channel = make_static!(command_channel);
    let (_esp_manager, esp_sender, esp_receiver) = esp_now.split();
    let heartbeat_signal: &mut Signal<NoopRawMutex,u64> = make_static!(Signal::new());

    executor.run(|spawner| {
        spawner.spawn(sender(esp_sender,command_channel.subscriber().unwrap())).unwrap();
        spawner.spawn(telemetry_receiver(esp_receiver,rtc,heartbeat_signal)).unwrap();

        spawner.spawn(connection_state(heartbeat_signal,rtc,status_pin)).unwrap();
        spawner.spawn(rotary_steering_x(rotary_pin_x_a,rotary_pin_x_b,command_channel.publisher().unwrap())).unwrap();
        // spawner.spawn(rotary_y(rotary_pin_y_a,rotary_pin_y_b,command_channel.sender())).unwrap();
        spawner.spawn(button_x(button_pin_x,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(button_y(button_pin_y,command_channel.publisher().unwrap())).unwrap();
        // spawner.spawn(debug_command(command_channel.receiver())).unwrap();
        spawner.spawn(blinker_test(command_channel.publisher().unwrap())).unwrap();

    })
}

#[embassy_executor::task]
async fn blinker_test(publisher: Publisher<'static, NoopRawMutex, ControlMessage,MAX_MESSAGES,MAX_PUBS,MAX_SUBS>) {
    loop {
        publisher.publish(ControlMessage::BlinkerCommand(protocol::BlinkerState::Alarm)).await;
        info!("Blinker state: {:?}",BlinkerState::Alarm);
        Timer::after_secs(2).await;
        publisher.publish(ControlMessage::BlinkerCommand(protocol::BlinkerState::Left)).await;
        info!("Blinker state: {:?}",BlinkerState::Left);
        Timer::after_secs(2).await;
        publisher.publish(ControlMessage::BlinkerCommand(protocol::BlinkerState::Right)).await;
        info!("Blinker state: {:?}",BlinkerState::Right);
        Timer::after_secs(2).await;
        publisher.publish(ControlMessage::BlinkerCommand(protocol::BlinkerState::Off)).await;
        info!("Blinker state: {:?}",BlinkerState::Off);
        Timer::after_secs(3).await;
    }
}

// #[embassy_executor::task]
// async fn rotary_y(pin_a: Gpio18<Input<PullUp>>,pin_b: Gpio19<Input<PullUp>>, sender: Sender<'static, NoopRawMutex, ControlMessage,COMMAND_QUEUE_SIZE>) {
//     rotary(pin_a, pin_b, Axis::Y, sender).await
// }


#[embassy_executor::task]
async fn button_x(button_pin: Gpio5<Input<PullUp>>, sender: Publisher<'static, NoopRawMutex, ControlMessage,MAX_MESSAGES,MAX_PUBS,MAX_SUBS>) {
    button(button_pin, Axis::X, sender).await;
}

#[embassy_executor::task]
async fn button_y(button_pin: Gpio9<Input<PullUp>>, sender: Publisher<'static, NoopRawMutex, ControlMessage,MAX_MESSAGES,MAX_PUBS,MAX_SUBS>) {
    button(button_pin, Axis::Y, sender).await;
}



async fn button<A: InputPin + Wait>(mut button_pin: A, axis: Axis, sender: Publisher<'static, NoopRawMutex, ControlMessage,MAX_MESSAGES,MAX_PUBS,MAX_SUBS>) {
    let mut was_high = true;
    loop {
        button_pin.wait_for_any_edge().await.unwrap();
        let is_high = button_pin.is_high().unwrap();
        match (is_high,was_high) {
            (true, true) => {},
            (true, false) => {sender.publish(ControlMessage::Release(axis)).await},
            (false, true) => {sender.publish(ControlMessage::Press(axis)).await},
            (false, false) => {},
        }
        was_high = is_high;
    }

}
#[embassy_executor::task]
async fn sender( mut esp_sender: EspNowSender<'static>, mut subscriber: Subscriber<'static, NoopRawMutex, ControlMessage,MAX_MESSAGES,MAX_PUBS,MAX_SUBS>) {
    info!("Starting sender...");
    loop {
        let message = subscriber.next_message_pure().await;
        esp_sender.send_async(&BROADCAST_ADDRESS, &message.to_bytes().unwrap()).await.unwrap();
    }
}