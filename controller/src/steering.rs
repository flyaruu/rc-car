use embassy_futures::select::select;

use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;
use esp_println::println;
use log::info;
use protocol::{ControlMessage, Message, MessagePublisher, MOTOR_CENTER_POSITION};
use rotary_encoder_hal::Rotary;

use crate::types::{MotorPinA, MotorPinB, SteeringPinA, SteeringPinB};

#[embassy_executor::task]
pub async fn rotary_steering(pin_a: SteeringPinA,pin_b: SteeringPinB, publisher: MessagePublisher) {
    let mut rotary = Rotary::new(pin_a, pin_b);
    let mut count = 0_i32;
    loop {
        let (pin_a,pin_b) = rotary.pins();
        select(pin_a.wait_for_any_edge(),pin_b.wait_for_any_edge()).await;
        let direction = rotary.update().unwrap();
        match direction {
            rotary_encoder_hal::Direction::Clockwise => {
                count+=1;
                let cmd = Message::Control(ControlMessage::SteeringPosition(count));
                info!("Sending steering command {:?}",cmd);
                publisher.publish(cmd).await;
            },
            rotary_encoder_hal::Direction::CounterClockwise => {
                count-=1;
                let cmd = Message::Control(ControlMessage::SteeringPosition(count));
                info!("Sending steering command {:?}",cmd);
                publisher.publish(cmd).await;
            },
            rotary_encoder_hal::Direction::None => (),
        }
    }
}

#[embassy_executor::task]
pub async fn rotary_motor(pin_a: MotorPinA, pin_b: MotorPinB, publisher: MessagePublisher) {
    let mut rotary = Rotary::new(pin_a, pin_b);
    let mut count = MOTOR_CENTER_POSITION;
    info!("Motor started");
    loop {
        let (pin_a,pin_b) = rotary.pins();
        select(pin_a.wait_for_any_edge(),pin_b.wait_for_any_edge()).await;
        info!("A: {} B: {}",pin_a.is_high().unwrap(),pin_b.is_high().unwrap());
        let direction = rotary.update().unwrap();
        match direction {
            rotary_encoder_hal::Direction::Clockwise => {
                info!("cw");
                if count < 50 {
                    count+=1;
                }
                let cmd = Message::Control(ControlMessage::MotorPower(count));
                publisher.publish(cmd).await;
            },
            rotary_encoder_hal::Direction::CounterClockwise => {
                info!("ccw");
                if count > -50 {
                    count-=1;
                    let cmd = Message::Control(ControlMessage::MotorPower(count));
                    publisher.publish(cmd).await;
                }
            },
            rotary_encoder_hal::Direction::None => (),
        }
        println!("Count: {}",count);
    }
}