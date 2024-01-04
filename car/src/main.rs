#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(result_flattening)]

extern crate alloc;
use core::mem::MaybeUninit;
use embassy_executor::Executor;
use embassy_sync::{pubsub::{PubSubChannel, Subscriber, Publisher}, blocking_mutex::raw::NoopRawMutex};
use embassy_time::Timer;
use esp_backtrace as _;
use esp_wifi::{EspWifiInitFor, initialize, esp_now::{EspNow, EspNowReceiver, EspNowSender, BROADCAST_ADDRESS}};
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, IO, timer::TimerGroup, embassy, systimer::SystemTimer, Rng, ledc::{LEDC, LSGlobalClkSource, LowSpeed, timer, channel::{self, config::PinConfig}}, gpio::{PushPull, Output, Gpio3}};

use log::{info, error};
use protocol::{ControlMessage, TelemetryMessage};
use static_cell::make_static;

use esp_backtrace as _;

use crate::servo::Servo;

mod servo;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

const MAX_PUBS: usize = 5;
const MAX_SUBS: usize = 5;
const MAX_MESSAGES: usize = 10;

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
    let clocks = make_static!(clocks);

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup");

    let io = IO::new(peripherals.GPIO,peripherals.IO_MUX);

    let steering_pin = io.pins.gpio3.into_push_pull_output();


    let ledc = LEDC::new(peripherals.LEDC, clocks);
    let ledc = make_static!(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    
    let mut servo_timer = ledc.get_timer::<LowSpeed>(timer::Number::Timer0);
    servo_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 50_u32.Hz(),
        })
        .unwrap();
    let lstimer0 = make_static!(servo_timer);

    let mut steering_channel = ledc.get_channel(channel::Number::Channel0, steering_pin);
    steering_channel
        .configure(hal::ledc::channel::config::Config {
            timer: lstimer0,
            duty_pct: 0,
            pin_config: PinConfig::PushPull,
        })
        .unwrap();
    let steering_servo: &'static mut Servo<'_, hal::gpio::GpioPin<Output<PushPull>, 3>, 600, 2415,14,50> = make_static!(Servo::new(steering_channel));



    let executor = make_static!(Executor::new());
    let timer_group = TimerGroup::new(peripherals.TIMG0, &clocks);    
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
    let esp_now = EspNow::new(&init, wifi).unwrap();
    
    let (_esp_manager, esp_sender, esp_receiver) = esp_now.split();

    let command_channel: &PubSubChannel<NoopRawMutex, ControlMessage, MAX_MESSAGES, MAX_SUBS, MAX_PUBS> = make_static!(PubSubChannel::new());
    let telemetry_channel: &PubSubChannel<NoopRawMutex, TelemetryMessage, MAX_MESSAGES, MAX_SUBS, MAX_PUBS> = make_static!(PubSubChannel::new());

    executor.run(|spawner| {
        spawner.spawn(receiver(esp_receiver,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(steering(command_channel.subscriber().unwrap(),steering_servo)).unwrap();
        spawner.spawn(telemetry_sender(esp_sender, telemetry_channel.subscriber().unwrap())).unwrap();
        spawner.spawn(heartbeat(telemetry_channel.publisher().unwrap())).unwrap();
    })
}


#[embassy_executor::task]
async fn heartbeat(publisher: Publisher<'static, NoopRawMutex, TelemetryMessage, MAX_MESSAGES, MAX_SUBS, MAX_PUBS>)->! {
    loop {
        publisher.publish(TelemetryMessage::Heartbeat).await;
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn steering(mut subscriber: Subscriber<'static, NoopRawMutex, ControlMessage, MAX_MESSAGES, MAX_SUBS, MAX_PUBS>, steering_servo: &'static mut Servo<'_, Gpio3<Output<PushPull>>, 600, 2415, 14, 50>)-> ! {
    loop {
        match subscriber.next_message_pure().await {
            ControlMessage::SteeringPosition(value) => {
                info!("Steering value: {}",value);
                let value: u32 = (value.min(100).max(0)) as u32;
                steering_servo.set_percentage(value as u8)                
            },
            _ => {
                info!("Sometihng else");
            }
        }

    }
}

#[embassy_executor::task]
async fn telemetry_sender(mut esp_sender: EspNowSender<'static>, mut subscriber: Subscriber<'static, NoopRawMutex,TelemetryMessage,MAX_MESSAGES,MAX_SUBS,MAX_PUBS>)->! {
    loop {
        let message = subscriber.next_message_pure().await;
        match message.to_bytes() {
            Ok(msg) => {
                match esp_sender.send_async(&BROADCAST_ADDRESS, &msg).await {
                    Ok(_) => {},
                    Err(e) => {
                        error!("Error sending telemetry: {:?}",e);
                    },
                }
            },
            Err(e) => error!("Error serializing telemetry: {:?}",e),
        }
    }
}

#[embassy_executor::task]
async fn receiver(mut esp_receiver: EspNowReceiver<'static>, publisher: Publisher<'static, NoopRawMutex,ControlMessage,MAX_MESSAGES,MAX_SUBS,MAX_PUBS>)->! {
    info!("Starting receiver...");
    loop {
        let msg = esp_receiver.receive_async().await;

        let _sender = msg.info.src_address;
        let msg = ControlMessage::from_slice(&msg.data);
        match msg {
            Ok(msg) => {
                publisher.publish(msg).await;
            },
            Err(e) => {
                error!("Problem: {:?}",e);
            },
        }


    }
}