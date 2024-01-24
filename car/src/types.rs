use hal::{gpio::*, ledc::{channel, timer}};

use crate::servo::Servo;

pub type RightBlinkerPin = Gpio1<Output<PushPull>>;
pub type BrakeLightPin = Gpio2<Output<PushPull>>;
pub type TailLightPin = Gpio3<Output<PushPull>>;
pub type ReverseLightPin = Gpio4<Output<PushPull>>;
pub type LeftBlinkerPin = Gpio5<Output<PushPull>>;
pub type MotorPin = Gpio7<Output<PushPull>>;

pub type HeadlightPin = Gpio0<Output<PushPull>>;
pub type SteeringPin = Gpio6<Output<PushPull>>;

pub type MotorServo = Servo<'static, MotorPin, 820, 1638, 14, MOTOR_FREQUENCY>;


pub const SERVO_TIMER_NUMBER: timer::Number = hal::ledc::timer::Number::Timer0;
pub const LED_TIMER_NUMBER: timer::Number = hal::ledc::timer::Number::Timer1;
pub const MOTOR_TIMER_NUMBER: timer::Number = hal::ledc::timer::Number::Timer2;

pub const STEERING_CHANNEL: channel::Number = channel::Number::Channel0;
pub const MOTOR_CHANNEL: channel::Number = channel::Number::Channel1;
pub const HEADLIGHT_CHANNEL: channel::Number = channel::Number::Channel2;

pub const MOTOR_FREQUENCY: u32 = 50;
