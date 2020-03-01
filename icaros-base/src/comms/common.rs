use std::collections::VecDeque;
use serde::{Deserialize, Serialize};
use std::error::Error;

use crate::utils::{Point3, Quartenion};

pub trait PacketScheduler<I, O, S> {
    fn can_send(&mut self) -> bool;
    fn send_state(&mut self, state: &S) -> Result<bool, Box<dyn Error>>;
    fn recv_state(&mut self, state: &mut S) -> Result<bool, Box<dyn Error>>;
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApplicationPacket {
    app_id: u8,
    data: Vec<u8>,
}

impl ApplicationPacket {
    pub fn new(app_id: u8, message: Vec<u8>) -> Self {
        ApplicationPacket {
            app_id,
            data: message,
        }
    }

    pub fn app_id(&self) -> u8 {
        self.app_id
    }

    pub fn message(&self) -> &Vec<u8> {
        &self.data
    }

    pub fn message_iter(&self) -> ::std::slice::Iter<u8> {
        self.data.iter()
    }
}

pub trait ApplicationPacketScheduler {
    fn send_app_packet(&mut self, data: ApplicationPacket) -> Result<bool, Box<dyn Error>>;
    fn recv_app_packets(&mut self) -> Result<&mut VecDeque<ApplicationPacket>, Box<dyn Error>>;
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MotorSpeed {
    pub tl: u16,
    pub tr: u16,
    pub bl: u16,
    pub br: u16,
}

impl MotorSpeed {
    pub fn from_speeds(speeds: (u16, u16, u16, u16)) -> Self {
        MotorSpeed {
            tl: speeds.0,
            tr: speeds.1,
            bl: speeds.2,
            br: speeds.3,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Acceleration {
    vector: Point3<f32>,
}

impl Acceleration {
    pub fn from_vec3(v: Point3<f32>) -> Self {
        Acceleration { vector: v }
    }

    pub fn as_point3(&self) -> &Point3<f32> {
        &self.vector
    }

    pub fn as_point3_mut(&mut self) -> &mut Point3<f32> {
        &mut self.vector
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Orientation {
    orientation: Quartenion,
}

impl Orientation {
    pub fn from_quartenion(q: Quartenion) -> Self {
        Orientation { orientation: q }
    }

    pub fn as_quartenion(&self) -> &Quartenion {
        &self.orientation
    }

    pub fn as_quartenion_mut(&mut self) -> &mut Quartenion {
        &mut self.orientation
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PIDParameters {
    p: f32,
    i: f32,
    d: f32,
}

impl PIDParameters {
    pub fn new(p: f32, i: f32, d: f32) -> Self {
        PIDParameters { p, i, d }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PIDAxes {
    roll: PIDParameters,
    pitch: PIDParameters,
    yaw: PIDParameters,
}

impl PIDAxes {
    pub fn new(roll: PIDParameters, pitch: PIDParameters, yaw: PIDParameters) -> Self {
        PIDAxes { roll, pitch, yaw }
    }
}
