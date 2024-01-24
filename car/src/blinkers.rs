use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};

use embassy_time::Timer;
use embedded_hal::digital::OutputPin;
use log::info;
use protocol::{BlinkState, BlinkerState, ControlMessage, Message, MessagePublisher, MessageSubscriber, TelemetryMessage};

use crate::types::{LeftBlinkerPin, RightBlinkerPin};


#[embassy_executor::task]
pub async fn blinker(spawner: Spawner, subscriber: MessageSubscriber, publisher: MessagePublisher, left_pin: LeftBlinkerPin, right_pin: RightBlinkerPin) {
    spawner.spawn(blinker_controller(subscriber, left_pin,right_pin,publisher)).unwrap();
}

#[embassy_executor::task]
async fn blinker_controller(mut subscriber: MessageSubscriber,  mut left_pin: LeftBlinkerPin, mut right_pin: RightBlinkerPin, mut publisher: MessagePublisher)-> ! {
    // let mut state = BlinkerState::Off;
    let mut blink_state = false;
    loop {
        match subscriber.next_message_pure().await {
            Message::Control(ControlMessage::BlinkerCommand(blinker)) => {
                // state = blinker;
                info!("Blinker comman received: {:?}",blinker);
                match blinker {
                    BlinkerState::Off => {
                        left_pin.set_low().unwrap();
                        right_pin.set_low().unwrap();
                        continue;
                    },
                    _ => loop {
                        info!("Starting blinker loop");
                        match select(subscriber.next_message_pure(),Timer::after_millis(400)).await {
                            Either::First(command) => {
                                info!("Blinker loop interrupted with command: {:?}",command);
                                match command {
                                    Message::Control(ControlMessage::BlinkerCommand(blinker_command)) => {
                                        // state = blinker_command;
                                        blink_state = true;
                                        set_blinker_state(blink_state, blinker_command, &mut left_pin, &mut right_pin, &mut publisher).await;
                                        match blinker_command {
                                            BlinkerState::Off => {
                                                break;
                                            },
                                            _ => {}
                                        }
                                    },
                                    _ => {}
                                }
                            },
                            Either::Second(_) => {

                                blink_state = !blink_state;
                                set_blinker_state(blink_state, blinker, &mut left_pin, &mut right_pin, &mut publisher).await;

                            },
                        }
                    }
                }
            },
            _ => {},
        }
    }
}

async fn set_blinker_state(blink_state: bool, state: BlinkerState, left_pin: &mut LeftBlinkerPin, right_pin: &mut RightBlinkerPin, publisher: &mut MessagePublisher) {
    info!("Setting blinker state to: {blink_state} and {:?}",state);
    if blink_state {
        match state {
            BlinkerState::Left => {
                left_pin.set_high().unwrap();
                publisher.publish(Message::Telemetry(TelemetryMessage::Blink(BlinkState::LeftOn))).await;
            },
            BlinkerState::Right => {
                right_pin.set_high().unwrap();
                publisher.publish(Message::Telemetry(TelemetryMessage::Blink(BlinkState::RightOn))).await;
    
            },
            BlinkerState::Off => {
                left_pin.set_low().unwrap();
                right_pin.set_low().unwrap();
                publisher.publish(Message::Telemetry(TelemetryMessage::Blink(BlinkState::AllOff))).await;
            },
            BlinkerState::Alarm => {
                left_pin.set_high().unwrap();
                right_pin.set_high().unwrap();
                publisher.publish(Message::Telemetry(TelemetryMessage::Blink(BlinkState::LeftOn))).await;
                publisher.publish(Message::Telemetry(TelemetryMessage::Blink(BlinkState::RightOn))).await;
            }        
        }    
    } else {
        left_pin.set_low().unwrap();
        right_pin.set_low().unwrap();
        publisher.publish(Message::Telemetry(TelemetryMessage::Blink(BlinkState::AllOff))).await;
    }
}