#![no_std]

extern crate alloc;

use alloc::vec::Vec;
use serde::{Serialize, Deserialize};

#[derive(Serialize,Deserialize,Clone, Copy,Debug)]
pub enum Axis {
    X, Y
}
#[derive(Serialize,Deserialize,Clone,Debug)]
pub enum ControlMessage {
    Value(Axis,i32),
    Press(Axis),
    Release(Axis),
}

#[derive(Serialize,Deserialize,Clone)]
pub struct TelemetryMessage {
    pub missed: bool,
}

impl ControlMessage {
    pub fn to_bytes(&self)->Vec<u8> {
        postcard::to_allocvec(self).unwrap()
    }
    pub fn from_slice(data: &[u8])->Self {
        postcard::from_bytes(data).unwrap()
    }
}

impl TelemetryMessage {
    pub fn to_bytes(&self)->Vec<u8> {
        postcard::to_allocvec(self).unwrap()
    }
    pub fn from_slice(data: &[u8])->Self {
        postcard::from_bytes(data).unwrap()
    }

    pub fn new()->Self {
        Self { missed: false }
    }
}


pub fn add(left: usize, right: usize) -> usize {
    left + right
}


