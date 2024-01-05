#![no_std]

extern crate alloc;


use alloc::{vec::Vec, string::{ToString, String}};
use serde::{Serialize, Deserialize};

#[derive(Clone, Debug)]
pub enum ProtocolError {
    SerializationError(String),
    DeserializationError(String),
}

#[derive(Serialize,Deserialize,Clone, Copy,Debug)]
pub enum Axis {
    X, Y
}

#[derive(Serialize,Deserialize,Clone,Debug)]
pub enum BlinkerState {
    Left,
    Right,
    Off,
    Alarm,
}

#[derive(Serialize,Deserialize,Clone,Debug)]
pub enum ControlMessage {
    SteeringPosition(i32),
    // Value(Axis,i32),
    Press(Axis),
    Release(Axis),
    BlinkerCommand(BlinkerState),
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


