use embassy_futures::select::select;
use embassy_futures::select::Either;
use embassy_time::Timer;
use hal::Rtc;
use hal::gpio::OutputPin;
use hal::prelude::_embedded_hal_digital_v2_OutputPin;
use hal::ledc::{LowSpeed, channel::ChannelIFace};
use log::info;
use protocol::BlinkerState;
use protocol::{ControlMessage, Headlights, MessageSubscriber, Message, MessagePublisher, ReverseLights};

use crate::{TailLightPin, HeadlightPin, BrakeLightPin, ReverseLightPin};

pub struct HeadlightController<'a, HP: OutputPin, TP: OutputPin> {
    channel: hal::ledc::channel::Channel<'a,LowSpeed,HP>,
    taillight_pin: TP,
    current_duty: u8,
}

impl <'a, HP: OutputPin, TP: OutputPin> HeadlightController<'a, HP, TP> {

    pub fn new(channel: hal::ledc::channel::Channel<'a,LowSpeed,HP>, taillight_pin: TP)->Self {
        HeadlightController{ channel, current_duty: 0, taillight_pin }
    }

    pub fn set_duty(&mut self, percentage: u8) {
        self.channel.set_duty(percentage).unwrap();
        self.current_duty = percentage;
    }
}

#[embassy_executor::task]
pub async fn light_controller(mut subscriber: MessageSubscriber, mut light_controller: HeadlightController<'static,HeadlightPin, TailLightPin>)-> ! {
    loop {
        match subscriber.next_message_pure().await {

            Message::Control(ControlMessage::HeadlightCommand(cmd)) => {
                match cmd {
                    Headlights::High=>{
                        info!("Lights high");
                        light_controller.set_duty(99);
                        light_controller.taillight_pin.set_high().unwrap();
                    }
                    Headlights::Low => {
                        info!("Lights low");
                        light_controller.set_duty(25);
                        light_controller.taillight_pin.set_high().unwrap();
                    },
                    Headlights::Off => {
                        info!("Lights off");
                        light_controller.set_duty(0);
                        light_controller.taillight_pin.set_low().unwrap();
                    }, 
                }
            },
            _ => {},
        }
    }
}

#[embassy_executor::task]
pub async fn reverselight_motor_monitor(mut subscriber: MessageSubscriber, publisher: MessagePublisher)-> ! {
    loop {
        match subscriber.next_message_pure().await {

            Message::Control(ControlMessage::MotorPower(value)) => {
                if value < 0 {
                    publisher.publish(Message::Control(ControlMessage::ReverselightCommand(ReverseLights::On))).await
                } else {
                    publisher.publish(Message::Control(ControlMessage::ReverselightCommand(ReverseLights::Off))).await
                }
            },
            _ => {},
        }
    }
}

#[embassy_executor::task]
pub async fn blink_cancellation_monitor(mut subscriber: MessageSubscriber, publisher: MessagePublisher)-> ! {
    let mut steering_position = 0_i32;
    let mut blinker_state = BlinkerState::Off;
    loop {
        match subscriber.next_message_pure().await {

            Message::Control(ControlMessage::SteeringPosition(value)) => {
                match blinker_state {
                    BlinkerState::Left => {
                        if value > steering_position {
                            publisher.publish(Message::Control(ControlMessage::BlinkerCommand(BlinkerState::Off))).await
                        }
                    },
                    BlinkerState::Right => {
                        if value < steering_position {
                            publisher.publish(Message::Control(ControlMessage::BlinkerCommand(BlinkerState::Off))).await
                        }

                    },
                    BlinkerState::Off => {},
                    BlinkerState::Alarm => {},
                }
                steering_position = value;
            },
            Message::Control(ControlMessage::BlinkerCommand(blinker)) => {
                blinker_state = blinker;

            },

            _ => {},
        }
    }
}


#[embassy_executor::task]
pub async fn brakelight_motor_monitor(mut subscriber: MessageSubscriber, publisher: MessagePublisher, rtc: &'static Rtc<'static>)-> ! {
    let mut last_motor_setting = 0_i32;
    let mut last_update_time = rtc.get_time_ms();
    let mut brakelight_on = false;
    const BRAKELIGHT_TIMEOUT: u64 = 500;
    loop {
        let selection = select(subscriber.next_message_pure(), Timer::after_millis(500)).await;
        match selection {
            Either::First(message) => {
                match message {
                    Message::Control(ControlMessage::MotorPower(value)) => {
                        if value.abs() < last_motor_setting.abs() {
                            info!("Braking");
                            publisher.publish(Message::Control(ControlMessage::BrakelightCommand(protocol::Brakelights::On))).await;
                            brakelight_on = true;
                        } else {
                            info!("Accelerating. Brakelight: {}",brakelight_on);
                            if brakelight_on {
                                info!("Accelerating, switching on brakelight");
                            }
                            publisher.publish(Message::Control(ControlMessage::BrakelightCommand(protocol::Brakelights::Off))).await;
                            brakelight_on = false;

                        }
                        last_update_time = rtc.get_time_ms();
                        last_motor_setting = value;
                    },
                    _ => {},
                }
            }
            Either::Second(_) => {
                info!("Timeout brakelight. On: {} elapsed: {} previous: {}", brakelight_on, rtc.get_time_ms() - last_update_time, last_motor_setting);
                if brakelight_on {
                    let elapsed = rtc.get_time_ms() - last_update_time;
                    info!("Timeout brakelight: elapsed: {}",elapsed);

                    if elapsed > BRAKELIGHT_TIMEOUT {
                        info!("Switch off brakelight");
                        brakelight_on = false;
                        publisher.publish(Message::Control(ControlMessage::BrakelightCommand(protocol::Brakelights::Off))).await;
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn brakelight_controller(mut subscriber: MessageSubscriber, mut led_pin: BrakeLightPin)-> ! {
    loop {
        match subscriber.next_message_pure().await {

            Message::Control(ControlMessage::BrakelightCommand(cmd)) => {
                match cmd {
                protocol::Brakelights::On => led_pin.set_high().unwrap(),
                protocol::Brakelights::Off => led_pin.set_low().unwrap(),
                }
            },
            _ => {},
        }
    }
}

#[embassy_executor::task]
pub async fn reverselight_controller(mut subscriber: MessageSubscriber, mut led_pin: ReverseLightPin)-> ! {
    loop {
        match subscriber.next_message_pure().await {

            Message::Control(ControlMessage::ReverselightCommand(cmd)) => {
                match cmd {
                protocol::ReverseLights::On => led_pin.set_high().unwrap(),
                protocol::ReverseLights::Off => led_pin.set_low().unwrap(),
                }
            },
            _ => {},
        }
    }
}