//! Common communication structures

use std::collections::VecDeque;
use serde::{Deserialize, Serialize};
use std::error::Error;

use crate::utils::{Point3, Quartenion};

///
/// A packet scheduler.
///
/// Implementors of this trait can act as packet schedulers, middleware between device drivers
/// and the core systems whose purpose is to schedule the input/output of packets.
///
/// `I` represents the type of the incoming packet (i.e. packet that updates state)
/// `O` represents the type of the outgoing packet (i.e. packet that replicates state)
/// `S` represents the type of the state being written or read to/from.
pub trait PacketScheduler<I, O, S> {
    /// Returns whether packets can be sent through this packet scheduler
    fn can_send(&mut self) -> bool;

    /// Schedule for and send the given state to the device driver
    fn send_state(&mut self, state: &S) -> Result<bool, Box<dyn Error>>;

    /// Receive data from the device driver and update the current state
    fn recv_state(&mut self, state: &mut S) -> Result<bool, Box<dyn Error>>;
}

///
/// An application-specific message payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApplicationPacket {
    app_id: u8,
    data: Vec<u8>,
}

impl ApplicationPacket {
    /// Creates a new application packet from its payload and the given application ID
    pub fn new(app_id: u8, message: Vec<u8>) -> Self {
        ApplicationPacket {
            app_id,
            data: message,
        }
    }

    /// Returns the app ID this packet is assigned for
    pub fn app_id(&self) -> u8 {
        self.app_id
    }

    /// Returns a reference to the payload
    pub fn message(&self) -> &Vec<u8> {
        &self.data
    }

    /// Returns an iterator to the payload's data
    pub fn message_iter(&self) -> ::std::slice::Iter<u8> {
        self.data.iter()
    }
}


///
/// A packet scheduler for application packets.
///
/// Implementors of this trait can act as application packet schedulers, middleware between device
/// drivers and the core systems whose purpose is to schedule the input/output
/// of application packets.
pub trait ApplicationPacketScheduler {
    /// Schedule for and send the given application packet to the device driver
    fn send_app_packet(&mut self, data: ApplicationPacket) -> Result<bool, Box<dyn Error>>;

    /// Receive application packets from the device driver
    fn recv_app_packets(&mut self) -> Result<&mut VecDeque<ApplicationPacket>, Box<dyn Error>>;
}

///
/// Represents the drone's motor speeds as raw values between 0 and 4096
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct MotorSpeed {
    /// Top-left motor
    pub tl: u16,

    /// Top-right motor
    pub tr: u16,

    /// Bottom-left motor
    pub bl: u16,

    /// Bottom-right motor
    pub br: u16,
}

impl MotorSpeed {
    /// Create a motor speeds structure from the provided motor speeds,
    /// in order: top-left, top-right, bottom-left, bottom-right.
    pub fn from_speeds(speeds: (u16, u16, u16, u16)) -> Self {
        MotorSpeed {
            tl: speeds.0,
            tr: speeds.1,
            bl: speeds.2,
            br: speeds.3,
        }
    }
}

///
/// Represents the drone's linear acceleration, in m/s^2
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Acceleration {
    vector: Point3<f32>,
}

impl Acceleration {
    /// Creates an `Acceleration` structure from the given three-dimensional vector
    pub fn from_vec3(v: Point3<f32>) -> Self {
        Acceleration { vector: v }
    }

    /// Returns the linear acceleration as a reference to a three-dimensional vector
    pub fn as_point3(&self) -> &Point3<f32> {
        &self.vector
    }

    /// Returns the linear acceleration as a mutable reference to a three-dimensional vector
    pub fn as_point3_mut(&mut self) -> &mut Point3<f32> {
        &mut self.vector
    }
}

///
/// Represents the drone's absolute orientation in space
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Orientation {
    orientation: Quartenion,
}

impl Orientation {
    /// Creates an `Orientation` structure from a quartenion
    pub fn from_quartenion(q: Quartenion) -> Self {
        Orientation { orientation: q }
    }

    /// Returns the orientation as a reference to a quartenion
    pub fn as_quartenion(&self) -> &Quartenion {
        &self.orientation
    }

    /// Returns the orientation as a mutable reference to a quartenion
    pub fn as_quartenion_mut(&mut self) -> &mut Quartenion {
        &mut self.orientation
    }
}

///
/// Represents a set of PID tuning parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PIDParameters {
    p: f32,
    i: f32,
    d: f32,
}

impl PIDParameters {
    /// Creates a set of PID tuning parameters from its individual P, I and D values.
    pub fn new(p: f32, i: f32, d: f32) -> Self {
        PIDParameters { p, i, d }
    }
}

///
/// Represents a complete set of PID tuning parameters for each rotational axes of the drone
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PIDAxes {
    roll: PIDParameters,
    pitch: PIDParameters,
    yaw: PIDParameters,
}

impl PIDAxes {
    /// Creates a complete set of PID tuning parameters from the PID parameters of each axis
    pub fn new(roll: PIDParameters, pitch: PIDParameters, yaw: PIDParameters) -> Self {
        PIDAxes { roll, pitch, yaw }
    }
}
