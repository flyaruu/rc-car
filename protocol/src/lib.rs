#![no_std]

extern crate alloc;


use alloc::{vec::Vec, string::{ToString, String}};
use embassy_sync::{pubsub::{PubSubChannel, Subscriber, Publisher}, blocking_mutex::raw::NoopRawMutex};
use serde::{Serialize, Deserialize};

pub type CommandChannel = PubSubChannel<NoopRawMutex, ControlMessage, MAX_MESSAGES, MAX_SUBS, MAX_PUBS>;
pub type CommandSubscriber = Subscriber<'static, NoopRawMutex, ControlMessage, MAX_MESSAGES, MAX_SUBS, MAX_PUBS>;
pub type CommandPublisher = Publisher<'static, NoopRawMutex, ControlMessage, MAX_MESSAGES, MAX_SUBS, MAX_PUBS>;

pub const MAX_PUBS: usize = 10;
pub const MAX_SUBS: usize = 5;
pub const MAX_MESSAGES: usize = 10;


#[derive(Clone, Debug)]
pub enum ProtocolError {
    SerializationError(String),
    DeserializationError(String),
}


#[derive(Serialize,Deserialize,Clone,Debug,Copy)]
pub enum BlinkerState {
    Left,
    Right,
    Off,
    Alarm,
}

#[derive(Serialize,Deserialize,Clone,Debug,Copy)]
pub enum Lights {
    Low,
    High,
    Off,
}

// channels:
// Motor, Steering, Horn, Blinker?, Headlights, Tail lights?
// timers:
// Servo, Horn, Blinker

#[derive(Serialize,Deserialize,Clone,Debug)]
pub enum ControlMessage {
    SteeringPosition(i32),
    MotorPower(u64),
    // Value(Axis,i32),
    BlinkerCommand(BlinkerState),
    HeadlightCommand(Lights),
}

#[derive(Serialize,Deserialize,Clone)]
pub enum TelemetryMessage {
    Heartbeat
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


