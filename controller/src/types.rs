use hal::gpio::{Gpio18, Gpio19, Gpio4, Gpio5, Gpio6, Gpio7, Gpio8, Gpio9, Input, PullUp};

pub type LeftButtonPin = Gpio5<Input<PullUp>>;
pub type RightButtonPin = Gpio9<Input<PullUp>>;
pub type TopLeftButtonPin = Gpio7<Input<PullUp>>;
pub type TopRightButtonPin = Gpio8<Input<PullUp>>;

pub type SteeringPinA = Gpio6<Input<PullUp>>;
pub type SteeringPinB = Gpio4<Input<PullUp>>;

pub type MotorPinA = Gpio18<Input<PullUp>>;
pub type MotorPinB = Gpio19<Input<PullUp>>;

