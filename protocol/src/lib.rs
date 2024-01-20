#![no_std]

extern crate alloc;


use alloc::{vec::Vec, string::{ToString, String}};
use embassy_sync::{pubsub::{PubSubChannel, Subscriber, Publisher}, blocking_mutex::raw::NoopRawMutex};
use serde::{Serialize, Deserialize};

// pub type CommandChannel = PubSubChannel<NoopRawMutex, ControlMessage, MAX_MESSAGES, MAX_SUBS, MAX_PUBS>;
// pub type CommandSubscriber = Subscriber<'static, NoopRawMutex, ControlMessage, MAX_MESSAGES, MAX_SUBS, MAX_PUBS>;
// pub type CommandPublisher = Publisher<'static, NoopRawMutex, ControlMessage, MAX_MESSAGES, MAX_SUBS, MAX_PUBS>;

pub type MessageChannel = PubSubChannel<NoopRawMutex, Message, MAX_MESSAGES, MAX_SUBS, MAX_PUBS>;
pub type MessageSubscriber = Subscriber<'static, NoopRawMutex, Message, MAX_MESSAGES, MAX_SUBS, MAX_PUBS>;
pub type MessagePublisher = Publisher<'static, NoopRawMutex, Message, MAX_MESSAGES, MAX_SUBS, MAX_PUBS>;

pub const MAX_PUBS: usize = 10;
pub const MAX_SUBS: usize = 10;
pub const MAX_MESSAGES: usize = 10;


#[derive(Clone, Debug)]
pub enum ProtocolError {
    SerializationError(String),
    DeserializationError(String),
}


#[derive(Serialize,Deserialize,Clone,Debug,Copy, PartialEq, Eq)]
pub enum BlinkerState {
    Left,
    Right,
    Off,
    Alarm,
}

#[derive(Serialize,Deserialize,Clone,Debug,Copy)]
pub enum Headlights {
    Low,
    High,
    Off,
}

#[derive(Serialize,Deserialize,Clone,Debug,Copy)]
pub enum Brakelights {
    On,
    Off,
}

#[derive(Serialize,Deserialize,Clone,Debug,Copy)]
pub enum ReverseLights {
    On,
    Off,
}

// channels:
// Motor, Steering, Horn, Blinker?, Headlights, Tail lights?
// timers:
// Servo, Horn, Blinker

#[derive(Serialize,Deserialize,Clone,Debug)]
pub enum Message {
    Control(ControlMessage),
    Telemetry(TelemetryMessage),
}

#[derive(Serialize,Deserialize,Clone,Debug)]
pub enum ControlMessage {
    SteeringPosition(i32),
    MotorPower(i32),
    // Value(Axis,i32),
    BlinkerCommand(BlinkerState),
    HeadlightCommand(Headlights),
    BrakelightCommand(Brakelights),
    ReverselightCommand(ReverseLights),
}

#[derive(Serialize,Deserialize,Clone,Debug)]
pub enum TelemetryMessage {
    Heartbeat(u64, u64),
    Blink(BlinkState),
}

#[derive(Serialize,Deserialize,Clone,Debug)]
pub enum BlinkState {
    LeftOn,
    RightOn,
    AllOff,
}
impl Message {
    pub fn to_bytes(&self)->Result<Vec<u8>,ProtocolError> {
        postcard::to_allocvec(self)
            .map_err(|e| ProtocolError::SerializationError(e.to_string()))
    }
    pub fn from_slice(data: &[u8])->Result<Self,ProtocolError> {
        postcard::from_bytes(data)
            .map_err(|e| ProtocolError::DeserializationError(e.to_string()))
    }
}


impl ControlMessage {
    pub fn to_bytes(&self)->Result<Vec<u8>,ProtocolError> {
        postcard::to_allocvec(self)
            .map_err(|e| ProtocolError::SerializationError(e.to_string()))
    }
    pub fn from_slice(data: &[u8])->Result<Self,ProtocolError> {
        postcard::from_bytes(data)
            .map_err(|e| ProtocolError::DeserializationError(e.to_string()))
    }
}

impl TelemetryMessage {
    pub fn to_bytes(&self)->Result<Vec<u8>,ProtocolError> {
        postcard::to_allocvec(self)
            .map_err(|e| ProtocolError::SerializationError(e.to_string()))
    }
    pub fn from_slice(data: &[u8])->Result<Self,ProtocolError> {
        postcard::from_bytes(data)
            .map_err(|e| ProtocolError::DeserializationError(e.to_string()))
    }

}


