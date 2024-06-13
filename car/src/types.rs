use hal::{gpio::*, ledc::{channel, timer}};

use crate::servo::Servo;

pub type MotorPin = GpioPin<7>;

pub type HeadlightPin = GpioPin<0>; //Gpio0<Output<PushPull>>;
pub type SteeringPin = GpioPin<6>; //Gpio6<Output<PushPull>>;

pub type MotorServo = Servo<'static, MotorPin, 820, 1638, 14, MOTOR_FREQUENCY>;
pub type SteeringServo = Servo<'static, SteeringPin, 600, 2415, 14, MOTOR_FREQUENCY>;

pub const SERVO_TIMER_NUMBER: timer::Number = hal::ledc::timer::Number::Timer0;
pub const LED_TIMER_NUMBER: timer::Number = hal::ledc::timer::Number::Timer1;
pub const MOTOR_TIMER_NUMBER: timer::Number = hal::ledc::timer::Number::Timer2;

pub const STEERING_CHANNEL: channel::Number = channel::Number::Channel0;
pub const MOTOR_CHANNEL: channel::Number = channel::Number::Channel1;
pub const HEADLIGHT_CHANNEL: channel::Number = channel::Number::Channel2;

pub const MOTOR_FREQUENCY: u32 = 50;
