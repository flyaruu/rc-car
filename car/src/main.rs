#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(result_flattening)]

extern crate alloc;
use core::mem::MaybeUninit;

use alloc::format;
use embassy_executor::Executor;
use embassy_sync::{pubsub::{PubSubChannel, Subscriber, Publisher}, blocking_mutex::raw::NoopRawMutex};
use embassy_time::Timer;
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::{EspWifiInitFor, initialize, esp_now::{EspNow, EspNowReceiver, EspNowSender, BROADCAST_ADDRESS}};
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, IO, timer::TimerGroup, embassy, systimer::SystemTimer, Rng, ledc::{LEDC, LSGlobalClkSource, LowSpeed, timer, channel::{self, config::PinConfig}}, gpio::{PushPull, Output, Gpio3, Gpio5, Gpio2, Gpio4, Gpio9, Gpio18, Gpio7, Gpio1, Gpio6, Gpio0}, Rtc};

use log::{info, error};
use protocol::{ControlMessage, TelemetryMessage, MAX_MESSAGES, MAX_SUBS, MAX_PUBS, MessageChannel, MessagePublisher, Message, MessageSubscriber};
use static_cell::make_static;

use esp_backtrace as _;

use crate::{servo::Servo, lights::{HeadlightController, light_controller, brakelight_controller, reverselight_controller, reverselight_motor_monitor, brakelight_motor_monitor, blink_cancellation_monitor}, blinkers::blinker};

mod servo;
mod blinkers;
mod lights;


pub type RightBlinkerPin = Gpio1<Output<PushPull>>;
pub type BrakeLightPin = Gpio2<Output<PushPull>>;
pub type TailLightPin = Gpio3<Output<PushPull>>;
pub type ReverseLightPin = Gpio4<Output<PushPull>>;
pub type LeftBlinkerPin = Gpio5<Output<PushPull>>;
pub type MotorPin = Gpio7<Output<PushPull>>;

pub type HeadlightPin = Gpio0<Output<PushPull>>;
pub type FrontLeftBlinkerPin = Gpio18<Output<PushPull>>;
pub type FrontRightBlinkerPin = Gpio9<Output<PushPull>>;
pub type SteeringPin = Gpio6<Output<PushPull>>;

pub type MotorServo = Servo<'static, MotorPin, 820, 1638, 14, MOTOR_FREQUENCY>;

pub const SERVO_TIMER_NUMBER: timer::Number = hal::ledc::timer::Number::Timer0;
pub const LED_TIMER_NUMBER: timer::Number = hal::ledc::timer::Number::Timer1;
pub const MOTOR_TIMER_NUMBER: timer::Number = hal::ledc::timer::Number::Timer2;

pub const STEERING_CHANNEL: channel::Number = channel::Number::Channel0;
pub const MOTOR_CHANNEL: channel::Number = channel::Number::Channel1;
pub const HEADLIGHT_CHANNEL: channel::Number = channel::Number::Channel2;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();


const MOTOR_FREQUENCY: u32 = 50;

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
    // let rtc = make_static!(Rtc::new(peripherals.RTC_CNTL));
    let rtc = make_static!(Rtc::new(peripherals.RTC_CNTL));

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup");

    let io = IO::new(peripherals.GPIO,peripherals.IO_MUX);

    let steering_pin = io.pins.gpio6.into_push_pull_output();
    let motor_pin = io.pins.gpio7.into_push_pull_output();

    let headlight_pin = io.pins.gpio0.into_push_pull_output();
    let taillight_pin = io.pins.gpio3.into_push_pull_output();
    let brakelight_pin = io.pins.gpio2.into_push_pull_output();
    let reverselight_pin = io.pins.gpio4.into_push_pull_output();

    let left_blinker_pin = io.pins.gpio5.into_push_pull_output();
    let right_blinker_pin = io.pins.gpio1.into_push_pull_output();



    let ledc = LEDC::new(peripherals.LEDC, clocks);
    let ledc = make_static!(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    
    let mut servo_timer = ledc.get_timer::<LowSpeed>(SERVO_TIMER_NUMBER);
    servo_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 50_u32.Hz(),
        })
        .unwrap();
    let servo_timer = make_static!(servo_timer);

    let mut motor_timer = ledc.get_timer::<LowSpeed>(MOTOR_TIMER_NUMBER );
    motor_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 50_u32.Hz(),
        })
        .unwrap();
    let motor_timer = make_static!(motor_timer);

    let mut led_timer = ledc.get_timer::<LowSpeed>(LED_TIMER_NUMBER);
    led_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 1000_u32.Hz(),
        })
        .unwrap();
    let led_timer = make_static!(led_timer);

    let mut steering_channel = ledc.get_channel(STEERING_CHANNEL, steering_pin);
    steering_channel
        .configure(hal::ledc::channel::config::Config {
            timer: servo_timer,
            duty_pct: 0,
            pin_config: PinConfig::PushPull,
        })
        .unwrap();

    let mut motor_channel = ledc.get_channel(MOTOR_CHANNEL, motor_pin);
    motor_channel
        .configure(hal::ledc::channel::config::Config {
            timer: motor_timer,
            duty_pct: 0,
            pin_config: PinConfig::PushPull,
        })
        .unwrap();

    let mut headlight_channel = ledc.get_channel(HEADLIGHT_CHANNEL, headlight_pin);
    headlight_channel
        .configure(hal::ledc::channel::config::Config {
            timer: led_timer,
            duty_pct: 0,
            pin_config: PinConfig::PushPull,
        })
        .unwrap();


    let headlight_controller = HeadlightController::new(headlight_channel,taillight_pin);

    let steering_servo: &'static mut Servo<'_, SteeringPin, 600, 2415,14,50> = make_static!(Servo::new(steering_channel));
    let motor_servo: &'static mut MotorServo = make_static!(Servo::new(motor_channel));

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

    // TODO unify?
    let command_channel: &MessageChannel = make_static!(PubSubChannel::new());
    // let telemetry_channel: &MessageChannel = make_static!(PubSubChannel::new());
    

    executor.run(|spawner| {
        spawner.spawn(receiver(esp_receiver,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(steering(command_channel.subscriber().unwrap(),steering_servo)).unwrap();
        spawner.spawn(motor(command_channel.subscriber().unwrap(),motor_servo,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(sender(esp_sender, command_channel.subscriber().unwrap())).unwrap();
        // spawner.spawn(heartbeat(command_channel.publisher().unwrap(), rtc)).unwrap();
        spawner.spawn(blinker(spawner,command_channel.subscriber().unwrap(), command_channel.publisher().unwrap(),left_blinker_pin,right_blinker_pin)).unwrap();
        spawner.spawn(light_controller(command_channel.subscriber().unwrap(),headlight_controller)).unwrap();
        spawner.spawn(brakelight_controller(command_channel.subscriber().unwrap(),brakelight_pin)).unwrap();
        spawner.spawn(reverselight_controller(command_channel.subscriber().unwrap(),reverselight_pin)).unwrap();
        spawner.spawn(reverselight_motor_monitor(command_channel.subscriber().unwrap(),command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(brakelight_motor_monitor(command_channel.subscriber().unwrap(),command_channel.publisher().unwrap(),rtc)).unwrap();
        spawner.spawn(blink_cancellation_monitor(command_channel.subscriber().unwrap(),command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(test_lights(command_channel.publisher().unwrap())).unwrap();
        // spawner.spawn(test_motor(command_channel.publisher().unwrap())).unwrap();
    })
}


#[embassy_executor::task]
async fn heartbeat(publisher: MessagePublisher, rtc: &'static Rtc<'static>)->! {
    let mut heartbeat_count = 0_u64;
    loop {
        let heartbeat = Message::Telemetry(TelemetryMessage::Heartbeat(heartbeat_count, rtc.get_time_us()));
        info!("Sending heartbeat: {:?}", heartbeat);
        publisher.publish(heartbeat).await;
        heartbeat_count+=1;
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn steering(mut subscriber: MessageSubscriber, steering_servo: &'static mut Servo<'_, SteeringPin, 600, 2415, 14, 50>)-> ! {
    steering_servo.set_percentage(50_u8); // center steering
    loop {
        match subscriber.next_message_pure().await {
            Message::Control(ControlMessage::SteeringPosition(value)) => {
                info!("Steering value: {}",value);
                // assert values min -50 max 50
                let value: u32 = ((value.min(12).max(-12)) as u32) + 50; // normalize to 0..100
                steering_servo.set_percentage(value as u8);
            },
            _ => {}
        }
    }
}

#[embassy_executor::task]
async fn motor(mut subscriber: MessageSubscriber, motor_servo: &'static mut MotorServo, publisher: MessagePublisher)-> ! {
    // recalibrate_motor(motor_servo).await;
    motor_servo.set_percentage(100);
    loop {
        match subscriber.next_message_pure().await {
            Message::Control(ControlMessage::MotorPower(value)) => {
                let v: u8 = (value + 50) as u8; 
                let duty = motor_servo.set_percentage(v);
                info!("Setting motor percentage: {}. Duty:",v);
                let msg = format!("Setting motor to percentage: {}: {} duty: {}", value, v, duty);
                publisher.publish(Message::Telemetry(TelemetryMessage::LogMessage(msg))).await;
                publisher.publish(Message::Telemetry(TelemetryMessage::MotorSetting(duty))).await;
            },
            Message::Control(ControlMessage::RecalibrateMotor) => {
                recalibrate_motor(motor_servo).await;
            }
            _ => {}
        }
    }
}

async fn recalibrate_motor(motor_servo: &mut MotorServo) {
    info!("Starting motor initialization. setting max!");
    motor_servo.set_percentage(100);
    Timer::after_secs(2).await;
    info!("Setting min...");
    motor_servo.set_percentage(0);
    Timer::after_secs(2).await;
    info!("Setting middle...");
    motor_servo.set_percentage(50);
    Timer::after_secs(2).await;
    info!("Motor initialization complete!");
}
#[embassy_executor::task]
async fn sender(mut esp_sender: EspNowSender<'static>, mut subscriber: Subscriber<'static, NoopRawMutex,Message,MAX_MESSAGES,MAX_SUBS,MAX_PUBS>)->! {
    loop {
        let message = subscriber.next_message_pure().await;
        // Only send telemetry
        match message {
            Message::Control(_) => {},
            Message::Telemetry(_) => {
                info!("Sending message from car: {:?}",message);
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
        
            },
        }
    }
}

#[embassy_executor::task]
async fn receiver(mut esp_receiver: EspNowReceiver<'static>, publisher: MessagePublisher)->! {
    info!("Starting receiver...");
    loop {
        let msg = esp_receiver.receive_async().await;

        let _sender = msg.info.src_address;
        let msg = Message::from_slice(&msg.data);
        println!("ESPNOW Reeived: {:?}",msg);
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

#[embassy_executor::task]
async fn test_lights(publisher: MessagePublisher)->! {
    info!("Starting test sequence...");

    // publisher.publish(Message::Control(ControlMessage::BlinkerCommand(protocol::BlinkerState::Alarm))).await;
    // Timer::after_millis(2500).await;
    // publisher.publish(Message::Control(ControlMessage::BlinkerCommand(protocol::BlinkerState::Off))).await;

    // publisher.publish(Message::Control(ControlMessage::ReverselightCommand(protocol::ReverseLights::On))).await;
    // publisher.publish(Message::Control(ControlMessage::BrakelightCommand(protocol::Brakelights::On))).await;
    // publisher.publish(Message::Control(ControlMessage::ReverselightCommand(protocol::ReverseLights::On))).await;

    // publisher.publish(Message::Control(ControlMessage::BlinkerCommand(protocol::BlinkerState::Off))).await;

    // for _ in 0..5 {
    //     info!("High beam");
    //     publisher.publish(Message::Control(ControlMessage::HeadlightCommand(protocol::Headlights::High))).await;
    //     Timer::after_millis(200).await;
    //     info!("Low beam");
    //     publisher.publish(Message::Control(ControlMessage::HeadlightCommand(protocol::Headlights::Low))).await;
    //     Timer::after_millis(200).await;
    //     info!("Off");
    //     publisher.publish(Message::Control(ControlMessage::HeadlightCommand(protocol::Headlights::Off))).await;
    //     Timer::after_millis(200).await;
    // }
    loop {
        // just nothing
        Timer::after_secs(100).await;
    }
}
