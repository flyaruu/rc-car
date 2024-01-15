use hal::{gpio::OutputPin, ledc::{channel::{Channel, ChannelHW}, LowSpeed}};
use log::info;

pub struct Servo<'a, P: OutputPin, const MIN_DUTY: u32, const MAX_DUTY: u32, const BITS: u32, const FREQUENCY: u32> {
    channel: Channel<'a,LowSpeed,P>,
}

impl <'a, P: OutputPin,const MIN_DUTY: u32, const MAX_DUTY: u32, const BITS: u32, const FREQUENCY: u32> 
    Servo<'a, P, MIN_DUTY, MAX_DUTY, BITS,FREQUENCY> {
    const CYCLE_TIME: u32 = 1000000 / FREQUENCY;
    const DUTY_SPACE: u32 = 2_u32.pow(BITS);
    pub fn new(channel: Channel<'a,LowSpeed,P>)->Self {
        Servo { channel }
    }

    pub fn set_percentage(&mut self, percentage: u8) {
        let range: u32 = MAX_DUTY - MIN_DUTY;
        let abs_duty = MIN_DUTY + (range * percentage as u32 / 100); // in micros
        info!("Duty abs: {} duty space: {}",abs_duty, Self::DUTY_SPACE);
        let duty = abs_duty * Self::DUTY_SPACE / Self::CYCLE_TIME;
        info!("Setting duty to: {}",duty);
        self.channel.set_duty_hw(duty);
    }
}