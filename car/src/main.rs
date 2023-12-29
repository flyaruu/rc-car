#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(result_flattening)]

extern crate alloc;
use core::mem::MaybeUninit;
use embassy_executor::Executor;
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::{EspWifiInitFor, initialize, esp_now::{EspNow, EspNowReceiver, EspNowSender}};
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, IO, timer::TimerGroup, embassy, systimer::SystemTimer, Rng, ledc::{LEDC, LSGlobalClkSource, LowSpeed, timer, channel::{self, config::PinConfig, Channel}}, gpio::{Gpio13, PushPull, Output, Gpio3, Gpio4}};

use log::{info, warn};
use protocol::{ControlMessage, TelemetryMessage};
use static_cell::make_static;



use esp_backtrace as _;

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
    let clocks = make_static!(clocks);
    // let mut delay = Delay::new(&clocks);

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup");

    let io = IO::new(peripherals.GPIO,peripherals.IO_MUX);

    let x_pin = io.pins.gpio3.into_push_pull_output();
    let y_pin = io.pins.gpio4.into_push_pull_output();

    let ledc = LEDC::new(peripherals.LEDC, clocks);
    let ledc = make_static!(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    
    let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 50_u32.Hz(),
        })
        .unwrap();
    let lstimer0 = make_static!(lstimer0);

    let mut x_channel = ledc.get_channel(channel::Number::Channel0, x_pin);
    x_channel
        .configure(hal::ledc::channel::config::Config {
            timer: lstimer0,
            duty_pct: 0,
            pin_config: PinConfig::PushPull,
        })
        .unwrap();

    let mut y_channel = ledc.get_channel(channel::Number::Channel1, y_pin);
    y_channel
        .configure(hal::ledc::channel::config::Config {
            timer: lstimer0,
            duty_pct: 0,
            pin_config: PinConfig::PushPull,
        })
        .unwrap();

    // hal::interrupt::enable(hal::peripherals::Interrupt::GPIO, hal::interrupt::Priority::Priority1).unwrap();
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
    executor.run(|spawner| {
        spawner.spawn(car_loop(esp_receiver,x_channel, y_channel)).unwrap();
    })
}

#[embassy_executor::task]
async fn car_loop(mut esp_receiver: EspNowReceiver<'static>, x_channel: Channel<'static, LowSpeed, Gpio3<Output<PushPull>>>, y_channel: Channel<'static, LowSpeed, Gpio4<Output<PushPull>>>) {
    info!("Starting receiver...");
    let mut previous = 0_usize;
    loop {
        let msg = esp_receiver.receive_async().await;

        info!("Received something!");
        let sender = msg.info.src_address;
        let msg = ControlMessage::from_slice(&msg.data);
        info!("Message: {:?}",msg);
        match msg {
            ControlMessage::Value(axis, value) => {
                let value = value.min(100).max(0) as u8;
                println!("Setting value to : {} on axis: {:?}",value,axis);
                match axis {
                    protocol::Axis::X => x_channel.set_duty(value).unwrap(),
                    protocol::Axis::Y => y_channel.set_duty(value).unwrap(),
                }
            },
            ControlMessage::Press(_) => {},
            ControlMessage::Release(_) => {},
        }

        // let missed = msg.count == previous + 1;
        // if missed {
        //     warn!("Missed some messages. Expected: {} got: {}",previous+1,msg.count);
        // }
        // previous = msg.count;
        // let telemetry = TelemetryMessage { missed};
        // esp_sender.send_async(&sender,&telemetry.to_bytes()).await.unwrap();
    }
}