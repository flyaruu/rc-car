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

    pub fn set_direct(&mut self, value: u32) {
        // let range: i32 = MAX_DUTY - MIN_DUTY;
        let abs_duty: u32 = MIN_DUTY + value; // in micros
        
        // info!("Duty abs: {} duty space: {}",abs_duty, Self::DUTY_SPACE);
        self.channel.set_duty_hw(abs_duty.try_into().unwrap());
    }

    pub fn set_direct2(&mut self, value: i32) {
        // let range: i32 = MAX_DUTY - MIN_DUTY;
        let abs_duty: i32 = (MIN_DUTY as i32) + value; // in micros

        info!("Setting duty abs: {}",value);
        self.channel.set_duty_hw(abs_duty as u32);
    }

    pub fn set_percentage(&mut self, percentage: u8)->u32 {
        let range: u32 = MAX_DUTY - MIN_DUTY;
        let abs_duty = MIN_DUTY + (range * percentage as u32 / 100); // in micros
        info!("Duty abs: {} duty space: {}",abs_duty, Self::DUTY_SPACE);
        let duty = abs_duty * Self::DUTY_SPACE / Self::CYCLE_TIME;
        info!("Setting duty to: {}",duty);
        self.channel.set_duty_hw(duty);
        return duty
    }
}