#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(result_flattening)]

extern crate alloc;
use core::mem::MaybeUninit;
use embassy_executor::Executor;


use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal, pubsub::PubSubChannel};
use embassy_time::Timer;

use embedded_hal_async::digital::Wait;
use esp_backtrace as _;

use esp_println::println;
use esp_wifi::{EspWifiInitFor, initialize, esp_now::{EspNow, BROADCAST_ADDRESS, EspNowSender}};
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, IO, timer::TimerGroup, embassy, systimer::SystemTimer, Rng, gpio::{Input, PullUp, Gpio5, Gpio9, Gpio7, Gpio8}, Rtc};

use log::info;
use protocol::{ControlMessage, BlinkerState, CommandPublisher, Lights, CommandSubscriber, CommandChannel};

use static_cell::make_static;

type LeftButtonPin = Gpio5<Input<PullUp>>;
type RightButtonPin = Gpio9<Input<PullUp>>;
type TopLeftButtonPin = Gpio7<Input<PullUp>>;
type TopRightButtonPin = Gpio8<Input<PullUp>>;

mod steering;
mod telemetry;

use esp_backtrace as _;


use crate::{telemetry::{connection_state, telemetry_receiver}, steering::{rotary_steering_x, rotary_motor_y}};


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
    let command_channel: CommandChannel = PubSubChannel::new();
    let command_channel = make_static!(command_channel);
    let (_esp_manager, esp_sender, esp_receiver) = esp_now.split();
    let heartbeat_signal: &mut Signal<NoopRawMutex,u64> = make_static!(Signal::new());

    // let left_button_signal: &mut Signal<NoopRawMutex, ButtonState> = make_static!(Signal::new());
    // let right_button_signal: &mut Signal<NoopRawMutex, ButtonState> = make_static!(Signal::new());


    executor.run(|spawner| {
        spawner.spawn(sender(esp_sender,command_channel.subscriber().unwrap())).unwrap();
        spawner.spawn(telemetry_receiver(esp_receiver,rtc,heartbeat_signal)).unwrap();

        spawner.spawn(connection_state(heartbeat_signal,rtc,status_pin)).unwrap();
        spawner.spawn(rotary_steering_x(rotary_pin_x_a,rotary_pin_x_b,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(rotary_motor_y(rotary_pin_y_a,rotary_pin_y_b,command_channel.publisher().unwrap())).unwrap();

        
        // spawner.spawn(rotary_y(rotary_pin_y_a,rotary_pin_y_b,command_channel.sender())).unwrap();
        spawner.spawn(button_left(button_pin_x,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(button_right(button_pin_y,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(button_top_left(button_pin_top_left,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(button_top_right(button_pin_top_right,command_channel.publisher().unwrap())).unwrap();
        // spawner.spawn(debug_command(command_channel.receiver())).unwrap();
        // spawner.spawn(blinker_test(command_channel.publisher().unwrap())).unwrap();

    })
}


// #[embassy_executor::task]
// async fn button_controller(left_button_signal: &'static mut Signal<NoopRawMutex,ButtonState>, right_button_signal: &'static mut Signal<NoopRawMutex,ButtonState>, mut publisher: CommandPublisher) {
//     let mut blinker_state = BlinkerState::Off;
//     loop {
//         let result = select(left_button_signal.wait(), right_button_signal.wait()).await;

//         match result {
//             Either::First(button_state) => {
//                 if button_state == ButtonState::Down {
//                     match blinker_state {
//                         BlinkerState::Left => { // left is blinking, left button was pressed, so switch off
//                             blinker_state = BlinkerState::Off;
//                             publisher.publish(ControlMessage::BlinkerCommand(blinker_state)).await;
//                         },
//                         _ => {
//                             blinker_state = BlinkerState::Left;
//                             publisher.publish(ControlMessage::BlinkerCommand(blinker_state)).await;
    
//                         },
//                     }
//                 }
//             },
//             Either::Second(right_button) => {
                
//             },
//         }
//     }    
// }

#[embassy_executor::task]
async fn button_left(mut button_pin: LeftButtonPin, publisher: CommandPublisher) {
    let mut blinker_state  = BlinkerState::Off;
    loop {
        button_pin.wait_for_rising_edge().await.unwrap();
        blinker_state = match blinker_state {
            BlinkerState::Left => BlinkerState::Off,
            _ => BlinkerState::Left,
        };
        publisher.publish(ControlMessage::BlinkerCommand(blinker_state)).await;
    }
}

#[embassy_executor::task]
async fn button_right(mut button_pin: RightButtonPin, publisher: CommandPublisher) {
    let mut blinker_state  = BlinkerState::Off;
    loop {
        button_pin.wait_for_rising_edge().await.unwrap();
        blinker_state = match blinker_state {
            BlinkerState::Right => BlinkerState::Off,
            _ => BlinkerState::Right,
        };
        publisher.publish(ControlMessage::BlinkerCommand(blinker_state)).await;
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn button_top_left(mut button_pin: TopLeftButtonPin, publisher: CommandPublisher) {
    let mut light_state: Lights = Lights::Off;
    loop {
        button_pin.wait_for_rising_edge().await.unwrap();
        light_state = match light_state {
            Lights::Low => Lights::High,
            Lights::High => Lights::Off,
            Lights::Off => Lights::Low,
        };
        publisher.publish(ControlMessage::HeadlightCommand(light_state)).await;
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn button_top_right(mut button_pin: TopRightButtonPin, _publisher: CommandPublisher) {
    loop {
        button_pin.wait_for_rising_edge().await.unwrap();
        info!("No action");
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn sender( mut esp_sender: EspNowSender<'static>, mut subscriber: CommandSubscriber) {
    info!("Starting sender...");
    loop {
        let message = subscriber.next_message_pure().await;
        esp_sender.send_async(&BROADCAST_ADDRESS, &message.to_bytes().unwrap()).await.unwrap();
    }
}