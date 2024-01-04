use embassy_futures::select::select;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pubsub::Publisher};
use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;
use esp_println::println;
use hal::gpio::{Gpio6, Input, PullUp, Gpio4};
use protocol::ControlMessage;
use rotary_encoder_hal::Rotary;

use crate::{MAX_MESSAGES, MAX_PUBS, MAX_SUBS};


pub async fn rotary_steering<A: InputPin + Wait, B: InputPin + Wait>(pin_a: A, pin_b: B, sender: Publisher<'static, NoopRawMutex, ControlMessage,MAX_MESSAGES,MAX_SUBS,MAX_PUBS>)->! {
    let mut rotary = Rotary::new(pin_a, pin_b);
    let mut count = 0_i32;
    loop {
        let (pin_a,pin_b) = rotary.pins();
        select(pin_a.wait_for_any_edge(),pin_b.wait_for_any_edge()).await;
        let direction = rotary.update().unwrap();
        match direction {
            rotary_encoder_hal::Direction::Clockwise => {
                count+=1;
                sender.publish(ControlMessage::SteeringPosition(count)).await;
            },
            rotary_encoder_hal::Direction::CounterClockwise => {
                count-=1;
                sender.publish(ControlMessage::SteeringPosition(count)).await;
            },
            rotary_encoder_hal::Direction::None => (),
        }
        println!("Count: {}",count);
    }
}

#[embassy_executor::task]
pub async fn rotary_steering_x(pin_a: Gpio6<Input<PullUp>>,pin_b: Gpio4<Input<PullUp>>, publisher: Publisher<'static, NoopRawMutex, ControlMessage,MAX_MESSAGES,MAX_SUBS,MAX_PUBS>) {
    rotary_steering(pin_a, pin_b, publisher).await
}
