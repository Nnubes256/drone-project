use core::fmt::Display;
use serde::Deserialize;
use serde::Serialize;
use snafu::Snafu;

use crate::comms::common::Acceleration;
use crate::comms::common::MotorSpeed;
use crate::comms::common::Orientation;
use crate::comms::common::PIDAxes;
use crate::utils::quartenion::Quartenion;

// TODO move StatusData and ControlData outta here

pub const CONTROLLER_HEADER_BYTE: u8 = 0x4F;

pub fn get_controller_codec() -> bincode2::Config {
    let mut config = bincode2::config();
    config.array_length(bincode2::LengthOption::U8);
    config.limit(64);
    config
}

pub struct StatusData {
    counter: u16,
    motor_speed: [u16; 4],
    orientation: Quartenion,
    accel: [f32; 3],
}

pub struct ControlData {
    roll: i16,
    pitch: i16,
    yaw: i16,
    throttle: u16,
    pid_p: f32,
    pid_i: f32,
    pid_d: f32,
}

#[derive(Debug, Snafu)]
pub enum ControllerReceiveError<T>
where
    T: Display,
{
    #[snafu(display("No packets available"))]
    NoPacketsAvailable,
    #[snafu(display("Deserialization failed: {}", inner))]
    DeserializationError { inner: bincode2::Error },
    #[snafu(display("Hardware driver error: {}", inner))]
    DriverRecvError { inner: Box<T> },
}

impl<T> From<bincode2::Error> for ControllerReceiveError<T>
where
    T: Display,
{
    fn from(value: bincode2::Error) -> Self {
        Self::DeserializationError { inner: value }
    }
}

#[derive(Debug, Snafu)]
pub enum ControllerSendError<T>
where
    T: Display,
{
    #[snafu(display("Deserialization failed: {}", inner))]
    SerializationError { inner: bincode2::Error },
    #[snafu(display("Hardware driver error: {}", inner))]
    DriverSendError { inner: Box<T> },
}

impl<T> From<bincode2::Error> for ControllerSendError<T>
where
    T: Display,
{
    fn from(value: bincode2::Error) -> Self {
        Self::SerializationError { inner: value }
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct A2RMessage {
    pub header: u8,
    pub counter: u16,
    pub motor_speed: MotorSpeed,
    pub orientation: Orientation,
    pub acceleration: Acceleration,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct R2ADesiredRotationAndThrottle {
    roll: i16,
    pitch: i16,
    yaw: i16,
    throttle: u16,
}

impl R2ADesiredRotationAndThrottle {
    pub fn new(roll: i16, pitch: i16, yaw: i16, throttle: u16) -> Self {
        R2ADesiredRotationAndThrottle { roll, pitch, yaw, throttle }
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct R2AMessage {
    pub header: u8,
    pub rpyt: R2ADesiredRotationAndThrottle,
    pub pid: PIDAxes,
}

pub trait ControllerCommunicationService : Sized {
    type ControllerCommunicationOptions;
    type HardwareDriverError: Display;

    fn setup(config: Self::ControllerCommunicationOptions) -> Result<Self, Self::HardwareDriverError>;
    fn send(&mut self, msg: R2AMessage) -> Result<bool, ControllerSendError<Self::HardwareDriverError>>;
    fn recv_available(&mut self) -> Result<usize, Self::HardwareDriverError>;
    fn recv(&mut self) -> Result<A2RMessage, ControllerReceiveError<Self::HardwareDriverError>>;
}
