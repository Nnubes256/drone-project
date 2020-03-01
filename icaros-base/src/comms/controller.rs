use crate::{comms::air::G2AControllerAxisState, utils::map_values};
use err_derive::Error;
use serde::{Deserialize, Serialize};
use std::{
    error::Error,
    fmt::{Debug, Display},
};

use crate::comms::common::{Acceleration, MotorSpeed, Orientation};

pub fn get_controller_codec() -> bincode2::Config {
    let mut config = bincode2::config();
    config.array_length(bincode2::LengthOption::U8);
    config.limit(256);
    config
}

#[derive(Debug, Error)]
pub enum ControllerReceiveError<T>
where
    T: 'static + Debug + Display + Error,
{
    #[error(display = "No packets available")]
    NoPacketsAvailable,
    #[error(display = "Deserialization failed: {}", _0)]
    DeserializationError(bincode2::Error),
    #[error(display = "Hardware driver error: {}", _0)]
    DriverRecvError(Box<T>),
}

impl<T> From<bincode2::Error> for ControllerReceiveError<T>
where
    T: 'static + Debug + Display + Error,
{
    fn from(value: bincode2::Error) -> Self {
        Self::DeserializationError(value)
    }
}

#[derive(Debug, Error)]
pub enum ControllerSendError<T>
where
    T: 'static + Debug + Display + Error,
{
    #[error(display = "Deserialization failed: {}", _0)]
    SerializationError(bincode2::Error),
    #[error(display = "Hardware driver error: {}", _0)]
    DriverSendError(Box<T>),
}

impl<T> From<bincode2::Error> for ControllerSendError<T>
where
    T: 'static + Debug + Display + Error,
{
    fn from(value: bincode2::Error) -> Self {
        Self::SerializationError(value)
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct A2RMessage {
    pub counter: u16,
    pub motor_speed: MotorSpeed,
    pub orientation: Orientation,
    pub acceleration: Acceleration,
}

impl A2RMessage {
    pub fn new(
        counter: u16,
        motor_speed: MotorSpeed,
        orientation: Orientation,
        acceleration: Acceleration,
    ) -> Self {
        A2RMessage {
            counter,
            motor_speed,
            orientation,
            acceleration,
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct R2ADesiredRotationAndThrottle {
    pub roll: i16,
    pub pitch: i16,
    pub yaw: i16,
    pub throttle: u16,
}

impl R2ADesiredRotationAndThrottle {
    pub fn new(roll: i16, pitch: i16, yaw: i16, throttle: u16) -> Self {
        R2ADesiredRotationAndThrottle {
            roll,
            pitch,
            yaw,
            throttle,
        }
    }

    pub fn from_controller(cas: &G2AControllerAxisState) -> Self {
        let corrected_throttle = {
            if cas.throttle < 0 {
                0 as u8
            } else {
                cas.throttle as u8
            }
        };
        R2ADesiredRotationAndThrottle {
            roll: map_values(cas.roll as f64, (-128.0, 127.0), (1024.0, 3072.0)) as i16,
            pitch: map_values(cas.pitch as f64, (-128.0, 127.0), (1024.0, 3072.0)) as i16,
            yaw: map_values(cas.yaw as f64, (-128.0, 127.0), (1024.0, 3072.0)) as i16,
            throttle: map_values(corrected_throttle as f64, (0.0, 127.0), (1024.0, 3072.0)) as u16,
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct R2AMessage {
    pub rpyt: R2ADesiredRotationAndThrottle,
}

impl R2AMessage {
    pub fn new(rpyt: R2ADesiredRotationAndThrottle) -> Self {
        R2AMessage { rpyt }
    }
}

pub trait ControllerCommunicationService: Sized {
    type ControllerCommunicationOptions;
    type HardwareDriverError: 'static + Error;

    fn setup(
        config: Self::ControllerCommunicationOptions,
    ) -> Result<Self, Self::HardwareDriverError>;
    fn is_tx_busy(&mut self) -> bool;
    fn send(
        &mut self,
        msg: R2AMessage,
    ) -> Result<bool, ControllerSendError<Self::HardwareDriverError>>;
    fn recv_available(&mut self) -> Result<usize, Self::HardwareDriverError>;
    fn recv(&mut self) -> Result<A2RMessage, ControllerReceiveError<Self::HardwareDriverError>>;
}
