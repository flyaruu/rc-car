use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pubsub::publisher, signal::Signal};
use embassy_time::Timer;
use embedded_hal::digital::OutputPin;
use protocol::{BlinkState, BlinkerState, ControlMessage, Message, MessagePublisher, MessageSubscriber, TelemetryMessage};
use static_cell::make_static;

use crate::{LeftBlinkerPin, RightBlinkerPin};

#[embassy_executor::task]
pub async fn blinker(spawner: Spawner, subscriber: MessageSubscriber, publisher: MessagePublisher, left_pin: LeftBlinkerPin, right_pin: RightBlinkerPin) {
    // let signal: Signal<NoopRawMutex, BlinkerState> = Signal::new();
    // let signal = make_static!(signal);
    // spawner.spawn(blinker_controller(subscriber, signal)).unwrap();
    spawner.spawn(blinker_controller2(subscriber, left_pin,right_pin,publisher)).unwrap();
    // spawner.spawn(blinkers(signal, publisher ,left_pin,right_pin)).unwrap();
}

#[embassy_executor::task]
async fn blinker_controller(mut subscriber: MessageSubscriber, signal: &'static Signal<NoopRawMutex, BlinkerState>)-> ! {
    loop {
        match subscriber.next_message_pure().await {
            Message::Control(ControlMessage::BlinkerCommand(blinker)) => {
                match blinker {
                    // BlinkerState::Off => signal.reset(),
                    _ => signal.signal(blinker),
                }
            },
            _ => {},
        }
    }
}

#[embassy_executor::task]
async fn blinker_controller2(mut subscriber: MessageSubscriber,  mut left_pin: LeftBlinkerPin, mut right_pin: RightBlinkerPin, mut publisher: MessagePublisher)-> ! {
    let mut state = BlinkerState::Off;
    let mut blink_state = false;
    loop {
        match subscriber.next_message_pure().await {
            Message::Control(ControlMessage::BlinkerCommand(blinker)) => {
                state = blinker;
                match blinker {
                    BlinkerState::Off => {
                        left_pin.set_low().unwrap();
                        right_pin.set_low().unwrap();
                        continue;
                    },
                    _ => loop {
                        match select(subscriber.next_message_pure(),Timer::after_millis(400)).await {
                            Either::First(command) => {
                                match command {
                                    Message::Control(ControlMessage::BlinkerCommand(blinker_command)) => {
                                        state = blinker_command;
                                        blink_state = true;
                                        set_blinker_state(blink_state, state, &mut left_pin, &mut right_pin, &mut publisher).await;
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
                                set_blinker_state(blink_state, state, &mut left_pin, &mut right_pin, &mut publisher).await;

                            },
                        }
                    }
                }
            },
            _ => {},
        }
    }
}

#[embassy_executor::task]
async fn blinkers(signal: &'static Signal<NoopRawMutex, BlinkerState>, mut publisher: MessagePublisher,  mut left_pin: LeftBlinkerPin, mut right_pin: RightBlinkerPin)-> ! {
    let mut state = BlinkerState::Off;
    let mut blink_state = false;
    loop {
        match select(signal.wait(),Timer::after_millis(400)).await {
            Either::First(blinker_state) => {
                state = blinker_state;
                blink_state = true;
                set_blinker_state(blink_state, state, &mut left_pin, &mut right_pin, &mut publisher).await;
            },
            Either::Second(_) => {
                blink_state = !blink_state;
                set_blinker_state(blink_state, state, &mut left_pin, &mut right_pin, &mut publisher).await;
            },
        }
    }
}

async fn set_blinker_state(blink_state: bool, state: BlinkerState, left_pin: &mut LeftBlinkerPin, right_pin: &mut RightBlinkerPin, publisher: &mut MessagePublisher) {
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