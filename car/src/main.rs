#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(result_flattening)]

extern crate alloc;
use core::mem::MaybeUninit;

use alloc::boxed::Box;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal_embassy::{init, Executor};
use esp_println::print;
use esp_wifi::{EspWifiInitFor, initialize, esp_now::EspNow};
use hal::{clock::ClockControl, gpio::{AnyInput, AnyOutput, Io, Level, Pull}, interrupt::enable, ledc::{channel::config::PinConfig, timer, LSGlobalClkSource, Ledc, LowSpeed}, peripherals::Peripherals, prelude::*, rng::Rng, rtc_cntl::Rtc, system::SystemControl, timer::{systimer::SystemTimer, timg::TimerGroup}};

use log::info;
use protocol::{ControlMessage, TelemetryMessage, MessageChannel, MessagePublisher, Message, MessageSubscriber};

use esp_backtrace as _;

use crate::{blinkers::blinker, lights::{HeadlightController, light_controller, brakelight_controller, reverselight_controller, reverselight_motor_monitor, brakelight_motor_monitor}, net::{receiver, sender}, servo::Servo, types::{MotorServo, SteeringServo, HEADLIGHT_CHANNEL, LED_TIMER_NUMBER, MOTOR_CHANNEL, MOTOR_TIMER_NUMBER, SERVO_TIMER_NUMBER, STEERING_CHANNEL}};

mod servo;
mod blinkers;
mod lights;

mod net;
mod types;

mod tach;

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
    let clocks = Box::leak(Box::new(clocks));
    let rtc = Box::leak(Box::new(Rtc::new(peripherals.LPWR,None)));
    esp_println::logger::init_logger(log::LevelFilter::Info);
    log::info!("Logger is setup");
    let io = Io::new(peripherals.GPIO,peripherals.IO_MUX);

    // let steering_pin = io.pins.gpio6.into_push_pull_output();
    // let steering_pin = AnyOutput::new(io.pins.gpio6, Level::Low);
    // let motor_pin = io.pins.gpio7.into_push_pull_output();
    // let motor_pin = AnyOutput::new(io.pins.gpio7, Level::Low);

    // let headlight_pin = AnyOutput::new(io.pins.gpio0, Level::Low);
    let taillight_pin = AnyOutput::new(io.pins.gpio3, Level::Low);
    let brakelight_pin = AnyOutput::new(io.pins.gpio2, Level::Low);
    let reverselight_pin = AnyOutput::new(io.pins.gpio4, Level::Low);

    let left_blinker_pin = AnyOutput::new(io.pins.gpio5, Level::Low);
    let right_blinker_pin = AnyOutput::new(io.pins.gpio1, Level::Low);

    let tach_pin = AnyInput::new(io.pins.gpio10, Pull::None);

    let ledc = Ledc::new(peripherals.LEDC, clocks);
    let ledc = Box::leak(Box::new(ledc));
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    
    enable(hal::peripherals::Interrupt::GPIO, hal::interrupt::Priority::Priority1).unwrap();

    let mut servo_timer = ledc.get_timer::<LowSpeed>(SERVO_TIMER_NUMBER);
    servo_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 50_u32.Hz(),
        })
        .unwrap();
    let servo_timer = Box::leak(Box::new(servo_timer));

    // TODO, remove motor_timer
    let mut motor_timer = ledc.get_timer::<LowSpeed>(MOTOR_TIMER_NUMBER );
    motor_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 50_u32.Hz(),
        })
        .unwrap();
    let motor_timer = Box::leak(Box::new(motor_timer));

    let mut led_timer = ledc.get_timer::<LowSpeed>(LED_TIMER_NUMBER);
    led_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 1000_u32.Hz(),
        })
        .unwrap();
    let led_timer = Box::leak(Box::new(led_timer));

    let mut steering_channel = ledc.get_channel(STEERING_CHANNEL, io.pins.gpio6);
    steering_channel
        .configure(hal::ledc::channel::config::Config {
            timer: servo_timer,
            duty_pct: 0,
            pin_config: PinConfig::PushPull,
        })
        .unwrap();

    let mut motor_channel = ledc.get_channel(MOTOR_CHANNEL, io.pins.gpio7);
    motor_channel
        .configure(hal::ledc::channel::config::Config {
            timer: motor_timer,
            duty_pct: 0,
            pin_config: PinConfig::PushPull,
        })
        .unwrap();

    let mut headlight_channel = ledc.get_channel(HEADLIGHT_CHANNEL, io.pins.gpio0);
    headlight_channel
        .configure(hal::ledc::channel::config::Config {
            timer: led_timer,
            duty_pct: 0,
            pin_config: PinConfig::PushPull,
        })
        .unwrap();

    let headlight_controller = HeadlightController::new(headlight_channel,taillight_pin);

    let steering_servo: &'static mut SteeringServo = Box::leak(Box::new(Servo::new(steering_channel)));
    let motor_servo: &'static mut MotorServo = Box::leak(Box::new(Servo::new(motor_channel)));

    let executor = Box::leak(Box::new(Executor::new()));
    let timer_group = TimerGroup::new_async(peripherals.TIMG0, &clocks);    
    init(&clocks,timer_group);

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
    
    let (_esp_manager, esp_sender, esp_receiver) = esp_now.split();

    // TODO unify?
    let command_channel: &MessageChannel = Box::leak(Box::new(MessageChannel::new()));
    hal::interrupt::enable(hal::peripherals::Interrupt::GPIO, hal::interrupt::Priority::Priority1).unwrap();

    executor.run(|spawner| {
        spawner.spawn(receiver(esp_receiver,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(steering(command_channel.subscriber().unwrap(),steering_servo)).unwrap();
        spawner.spawn(motor(command_channel.subscriber().unwrap(),motor_servo,command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(sender(esp_sender, command_channel.subscriber().unwrap())).unwrap();
        spawner.spawn(heartbeat(command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(blinker(spawner,command_channel.subscriber().unwrap(), command_channel.publisher().unwrap(),left_blinker_pin,right_blinker_pin)).unwrap();
        spawner.spawn(light_controller(command_channel.subscriber().unwrap(),headlight_controller)).unwrap();
        spawner.spawn(reverselight_controller(command_channel.subscriber().unwrap(),reverselight_pin)).unwrap();
        spawner.spawn(reverselight_motor_monitor(command_channel.subscriber().unwrap(),command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(brakelight_controller(command_channel.subscriber().unwrap(),brakelight_pin)).unwrap();
        spawner.spawn(brakelight_motor_monitor(command_channel.subscriber().unwrap(),command_channel.publisher().unwrap(),rtc)).unwrap();
        spawner.spawn(test_lights(command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(tach::tach(spawner, command_channel.publisher().unwrap(), tach_pin, rtc)).unwrap();
    })
}

#[embassy_executor::task]
async fn monitor_rpm(mut subscriber: MessageSubscriber)->! {
    let mut last_odo = 0_u64;
    let mut last_rpm = 0_u64;
    loop {
        let message = subscriber.next_message_pure().await;
        match message {
            Message::Telemetry(telemetry) => {
                match telemetry {
                    TelemetryMessage::MotorRpm(rpm) => {
                        last_rpm = rpm;
                    },
                    TelemetryMessage::MotorOdo(odo) => {
                        last_odo = odo;
                    },
                    _ => {},
                }
            },
            _ => {}
        }
        print!("RPM {: >4} ODO {: >4}\r",last_rpm,last_odo);
    }
}


#[embassy_executor::task]
async fn heartbeat(publisher: MessagePublisher)->! {
    let mut heartbeat_count = 0_u64;
    loop {
        let heartbeat = Message::Telemetry(TelemetryMessage::Heartbeat(heartbeat_count));
        publisher.publish(heartbeat).await;
        heartbeat_count+=1;
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn steering(mut subscriber: MessageSubscriber, steering_servo: &'static mut SteeringServo)-> ! {
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
async fn test_lights(publisher: MessagePublisher)->! {
    info!("Starting test sequence...");

    publisher.publish(Message::Control(ControlMessage::BlinkerCommand(protocol::BlinkerState::Alarm))).await;
    Timer::after_millis(2500).await;
    publisher.publish(Message::Control(ControlMessage::BlinkerCommand(protocol::BlinkerState::Off))).await;

    publisher.publish(Message::Control(ControlMessage::ReverselightCommand(protocol::ReverseLights::On))).await;
    publisher.publish(Message::Control(ControlMessage::BrakelightCommand(protocol::Brakelights::On))).await;
    publisher.publish(Message::Control(ControlMessage::ReverselightCommand(protocol::ReverseLights::On))).await;

    publisher.publish(Message::Control(ControlMessage::BlinkerCommand(protocol::BlinkerState::Off))).await;

    for _ in 0..5 {
        info!("High beam");
        publisher.publish(Message::Control(ControlMessage::HeadlightCommand(protocol::Headlights::High))).await;
        Timer::after_millis(200).await;
        info!("Low beam");
        publisher.publish(Message::Control(ControlMessage::HeadlightCommand(protocol::Headlights::Low))).await;
        Timer::after_millis(200).await;
        info!("Off");
        publisher.publish(Message::Control(ControlMessage::HeadlightCommand(protocol::Headlights::Off))).await;
        Timer::after_millis(200).await;
    }
    loop {
        // just nothing
        Timer::after_secs(100).await;
    }
}
