#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(result_flattening)]

extern crate alloc;
use core::mem::MaybeUninit;

use embassy_executor::Executor;

use embassy_time::Timer;
use esp_backtrace as _;
use esp_wifi::{EspWifiInitFor, initialize, esp_now::EspNow};
use hal::{clock::ClockControl, embassy, ledc::{LEDC, LSGlobalClkSource, LowSpeed, timer, channel::config::PinConfig}, peripherals::Peripherals, prelude::*, systimer::SystemTimer, timer::TimerGroup, Rng, Rtc, IO};

use log::info;
use protocol::{ControlMessage, TelemetryMessage, MessageChannel, MessagePublisher, Message, MessageSubscriber};
use static_cell::make_static;

use esp_backtrace as _;

use crate::{blinkers::blinker, lights::{HeadlightController, light_controller, brakelight_controller, reverselight_controller, reverselight_motor_monitor, brakelight_motor_monitor}, net::{receiver, sender}, servo::Servo, types::{MotorServo, SteeringServo, HEADLIGHT_CHANNEL, LED_TIMER_NUMBER, MOTOR_CHANNEL, MOTOR_TIMER_NUMBER, SERVO_TIMER_NUMBER, STEERING_CHANNEL}};

mod servo;
mod blinkers;
mod lights;

mod net;
mod types;
mod tach;
mod speedo;

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
    // let rtc = make_static!(Rtc::new(peripherals.RTC_CNTL));
    let rtc = make_static!(Rtc::new(peripherals.LPWR));
    esp_println::logger::init_logger(log::LevelFilter::Info);
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

    let tach_pin = io.pins.gpio10.into_floating_input();
    let speedo_pin = io.pins.gpio9.into_floating_input();

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

    // TODO, remove motor_timer
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

    let steering_servo: &'static mut SteeringServo = make_static!(Servo::new(steering_channel));
    let motor_servo: &'static mut MotorServo = make_static!(Servo::new(motor_channel));

    let executor = make_static!(Executor::new());
    let timer_group = TimerGroup::new(peripherals.TIMG0, &clocks);    
    embassy::init(&clocks,timer_group);

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
    let command_channel: &MessageChannel = make_static!(MessageChannel::new());
    // let telemetry_channel: &MessageChannel = make_static!(PubSubChannel::new());
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
        // spawner.spawn(blink_cancellation_monitor(command_channel.subscriber().unwrap(),command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(test_lights(command_channel.publisher().unwrap())).unwrap();
        // spawner.spawn(test_motor(command_channel.publisher().unwrap())).unwrap();
        spawner.spawn(tach::tach(spawner,tach_pin,command_channel.publisher().unwrap(),rtc)).unwrap();
        spawner.spawn(speedo::tach(spawner,speedo_pin,command_channel.publisher().unwrap(),rtc)).unwrap();
    })
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
