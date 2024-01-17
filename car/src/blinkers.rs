use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::Timer;
use embedded_hal::digital::OutputPin;
use protocol::{ControlMessage, BlinkerState, MessageSubscriber, Message};
use static_cell::make_static;

use crate::{LeftBlinkerPin, RightBlinkerPin};

#[embassy_executor::task]
pub async fn blinker(spawner: Spawner, subscriber: MessageSubscriber, left_pin: LeftBlinkerPin, right_pin: RightBlinkerPin) {
    let signal: Signal<NoopRawMutex, BlinkerState> = Signal::new();
    let signal = make_static!(signal);
    spawner.spawn(blinker_controller(subscriber, signal)).unwrap();
    spawner.spawn(blinkers(signal, left_pin,right_pin)).unwrap();
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
async fn blinkers(signal: &'static Signal<NoopRawMutex, BlinkerState>, mut left_pin: LeftBlinkerPin, mut right_pin: RightBlinkerPin)-> ! {
    let mut state = BlinkerState::Off;
    let mut blink_state = false;
    loop {
        match select(signal.wait(),Timer::after_millis(400)).await {
            Either::First(blinker_state) => {
                state = blinker_state;
            },
            Either::Second(_) => {
                blink_state = !blink_state;
                if blink_state {
                    match state {
                        BlinkerState::Left => {
                            left_pin.set_high().unwrap();
                        },
                        BlinkerState::Right => {
                            right_pin.set_high().unwrap();
                        },
                        BlinkerState::Off => {
                            left_pin.set_low().unwrap();
                            right_pin.set_low().unwrap();
                        },
                        BlinkerState::Alarm => {
                            left_pin.set_high().unwrap();
                            right_pin.set_high().unwrap();
                        }
                   }
            
                } else {
                    left_pin.set_low().unwrap();
                    right_pin.set_low().unwrap();
                }

            },
        }
    }
}
