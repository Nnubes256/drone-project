use serde::Deserialize;
use serde::Serialize;

use crate::comms::common::Acceleration;
use crate::comms::common::MotorSpeed;
use crate::comms::common::Orientation;
use crate::comms::common::PIDAxes;
use crate::utils::quartenion::Quartenion;

// TODO move StatusData and ControlData outta here

pub struct StatusData {
    counter: u16,
    motor_speed: [u16; 4],
    orientation: Quartenion,
    accel: [f32; 3],
}

pub struct ControlData {
    roll: u16,
    pitch: u16,
    yaw: u16,
    throttle: u16,
    pid_p: f32,
    pid_i: f32,
    pid_d: f32,
}

#[derive(Serialize, Deserialize)]
pub struct A2RMessage {
    header: u8,
    counter: u16,
    motor_speed: MotorSpeed,
    orientation: Orientation,
    acceleration: Acceleration,
}

#[derive(Serialize, Deserialize)]
pub struct R2ADesiredRotationAndThrottle {
    roll: i16,
    pitch: i16,
    yaw: i16,
    throttle: u16,
}

#[derive(Serialize, Deserialize)]
pub struct R2AMessage {
    header: u8,
    rpyt: R2ADesiredRotationAndThrottle,
    pid: PIDAxes,
}

pub trait ControllerCommunicationService {
    type ControllerCommunicationOptions;

    fn setup(config: Self::ControllerCommunicationOptions) -> Self;
    fn send(&mut self, msg: R2AMessage) -> bool;
    fn recv_available(&mut self) -> usize;
    fn recv(&mut self) -> Option<A2RMessage>;
}
