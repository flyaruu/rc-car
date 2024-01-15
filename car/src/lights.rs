use hal::gpio::OutputPin;
use hal::prelude::_embedded_hal_digital_v2_OutputPin;
use hal::ledc::{LowSpeed, channel::ChannelIFace};
use log::info;
use protocol::{ControlMessage, Lights};

use crate::{CommandSubscriber, TailLightPin, HeadlightPin};

pub struct HeadlightController<'a, HP: OutputPin, TP: OutputPin> {
    channel: hal::ledc::channel::Channel<'a,LowSpeed,HP>,
    taillight_pin: TP,
    current_duty: u8,
}

impl <'a, HP: OutputPin, TP: OutputPin> HeadlightController<'a, HP, TP> {

    pub fn new(channel: hal::ledc::channel::Channel<'a,LowSpeed,HP>, taillight_pin: TP)->Self {
        HeadlightController{ channel, current_duty: 0, taillight_pin }
    }

    pub fn set_duty(&mut self, percentage: u8) {
        self.channel.set_duty(percentage).unwrap();
        self.current_duty = percentage;
    }
}

#[embassy_executor::task]
pub async fn light_controller(mut subscriber: CommandSubscriber, mut light_controller: HeadlightController<'static,HeadlightPin, TailLightPin>)-> ! {
    loop {
        match subscriber.next_message_pure().await {
            ControlMessage::HeadlightCommand(cmd) => {
                match cmd {
                    Lights::High=>{
                        info!("Lights high");
                        light_controller.set_duty(99);
                        light_controller.taillight_pin.set_high().unwrap();
                    }
                    Lights::Low => {
                        info!("Lights low");
                        light_controller.set_duty(40);
                        light_controller.taillight_pin.set_high().unwrap();
                    },
                    Lights::Off => {
                        info!("Lights off");
                        light_controller.set_duty(0);
                        light_controller.taillight_pin.set_low().unwrap();
                    }, 
                }
            },
            _ => {},
        }
    }
}