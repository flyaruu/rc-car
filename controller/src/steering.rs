use embassy_futures::select::select;

use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;
use esp_println::println;
use hal::gpio::{Gpio6, Input, PullUp, Gpio4, Gpio19, Gpio18};
use log::info;
use protocol::{ControlMessage, MessagePublisher, Message};
use rotary_encoder_hal::Rotary;



pub async fn rotary_steering<A: InputPin + Wait, B: InputPin + Wait>(pin_a: A, pin_b: B, sender: MessagePublisher)->! {
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
                sender.publish(cmd).await;
            },
            rotary_encoder_hal::Direction::CounterClockwise => {
                count-=1;
                let cmd = Message::Control(ControlMessage::SteeringPosition(count));
                info!("Sending steering command {:?}",cmd);
                sender.publish(cmd).await;
            },
            rotary_encoder_hal::Direction::None => (),
        }
    }
}
    
    pub async fn rotary_motor<A: InputPin + Wait, B: InputPin + Wait>(pin_a: A, pin_b: B, sender: MessagePublisher)->! {
        let mut rotary = Rotary::new(pin_a, pin_b);
        let mut count = 0_i32;
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
                    sender.publish(cmd).await;
                },
                rotary_encoder_hal::Direction::CounterClockwise => {
                    info!("ccw");
                    if count > -50 {
                        count-=1;
                        let cmd = Message::Control(ControlMessage::MotorPower(count));
                        sender.publish(cmd).await;
                    }
                },
                rotary_encoder_hal::Direction::None => (),
            }
            println!("Count: {}",count);
        }
    }

#[embassy_executor::task]
pub async fn rotary_steering_x(pin_a: Gpio6<Input<PullUp>>,pin_b: Gpio4<Input<PullUp>>, publisher: MessagePublisher) {
    rotary_steering(pin_a, pin_b, publisher).await
}

#[embassy_executor::task]
pub async fn rotary_motor_y(pin_a: Gpio18<Input<PullUp>>,pin_b: Gpio19<Input<PullUp>>, publisher: MessagePublisher) {
    rotary_motor(pin_a, pin_b, publisher).await
}