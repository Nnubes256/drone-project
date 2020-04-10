//! Flight controller structures

use crate::{comms::air::G2AControllerAxisState, utils::map_values};
use err_derive::Error;
use serde::{Deserialize, Serialize};
use std::{
    error::Error,
    fmt::{Debug, Display},
};

use crate::comms::common::{Acceleration, MotorSpeed, Orientation};

/// Obtains the `bincode2` configuration used for the flight controller's
/// serialization/deserialization codec
pub fn get_controller_codec() -> bincode2::Config {
    let mut config = bincode2::config();
    config.array_length(bincode2::LengthOption::U8);
    config.limit(256);
    config
}

///
/// Error ocurred when receving packets from the flight controller
///
/// `T` is the type of the driver-side errors
#[derive(Debug, Error)]
pub enum ControllerReceiveError<T>
where
    T: 'static + Debug + Display + Error,
{
    /// No packets are available for receiving
    #[error(display = "No packets available")]
    NoPacketsAvailable,

    /// Packet deserialization failed
    #[error(display = "Deserialization failed: {}", _0)]
    DeserializationError(bincode2::Error),

    /// Error on the driver side
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

///
/// Error ocurred when sending packets to the flight controller
///
/// `T` is the type of the driver-side errors
#[derive(Debug, Error)]
pub enum ControllerSendError<T>
where
    T: 'static + Debug + Display + Error,
{
    /// Packet serialization failed
    #[error(display = "Deserialization failed: {}", _0)]
    SerializationError(bincode2::Error),

    /// Error on the driver side
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

///
/// A flight-controller-to-ICAROS (Arduino-to-Raspberry) message.
#[derive(Debug, Serialize, Deserialize)]
pub struct A2RMessage {
    pub counter: u16,
    pub motor_speed: MotorSpeed,
    pub orientation: Orientation,
    pub acceleration: Acceleration,
}

impl A2RMessage {
    /// Creates a new flight-controller-to-ICAROS message
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

///
/// The drone rotation and throttle inputted to the flight controller as part of the setpoint
#[derive(Debug, Serialize, Deserialize)]
pub struct R2ADesiredRotationAndThrottle {
    pub roll: i16,
    pub pitch: i16,
    pub yaw: i16,
    pub throttle: u16,
}

impl R2ADesiredRotationAndThrottle {
    /// Creates a new desired rotation and throttle from the individual measurements
    pub fn new(roll: i16, pitch: i16, yaw: i16, throttle: u16) -> Self {
        R2ADesiredRotationAndThrottle {
            roll,
            pitch,
            yaw,
            throttle,
        }
    }

    /// Creates a new desired rotation and throttle from the inputs recevied from the ground
    /// control.
    ///
    /// This involves clamping their values if applicable, and mapping them to their equivalent
    /// ranges, inverting them if necessary.
    pub fn from_controller(cas: &G2AControllerAxisState) -> Self {
        // Clamp down the throttle values in order to avoid erratic mappings from negative values
        let corrected_throttle = {
            if cas.throttle < 0 {
                0 as u8
            } else {
                cas.throttle as u8
            }
        };

        R2ADesiredRotationAndThrottle {
            roll: map_values(cas.roll as f64, (-128.0, 127.0), (3072.0, 1024.0)) as i16, // Inverted
            pitch: map_values(cas.pitch as f64, (-128.0, 127.0), (3072.0, 1024.0)) as i16, // Inverted
            yaw: map_values(cas.yaw as f64, (-128.0, 127.0), (1024.0, 3072.0)) as i16, // Normal
            throttle: map_values(corrected_throttle as f64, (0.0, 127.0), (1024.0, 3072.0)) as u16,
        }
    }
}

///
/// A ICAROS-to-flight-controller (Raspberry-to-Arduino) message.
#[derive(Debug, Serialize, Deserialize)]
pub struct R2AMessage {
    pub rpyt: R2ADesiredRotationAndThrottle,
}

impl R2AMessage {
    /// Creates a new ICAROS-to-flight-controller message from its components.
    pub fn new(rpyt: R2ADesiredRotationAndThrottle) -> Self {
        R2AMessage { rpyt }
    }
}


///
/// A flight controller communication driver
///
/// This trait, when implemented, allows the implementor to act as a device driver for
/// communications with the flight controller.
pub trait ControllerCommunicationService: Sized {
    /// The settings structure that has to be passed to the device driver
    type ControllerCommunicationOptions;

    /// The error type definining hardware-specific errors
    type HardwareDriverError: 'static + Error;


    /// Creates a new instance of the implementor with the given configuration
    fn setup(
        config: Self::ControllerCommunicationOptions,
    ) -> Result<Self, Self::HardwareDriverError>;

    /// Returns whether or not the underlying device is not yet ready for sending packets
    fn is_tx_busy(&mut self) -> bool;

    /// Sends a single message through the device
    fn send(
        &mut self,
        msg: R2AMessage,
    ) -> Result<bool, ControllerSendError<Self::HardwareDriverError>>;

    /// Returns how many (or, depending on implementation, whether any) messages are available
    fn recv_available(&mut self) -> Result<usize, Self::HardwareDriverError>;

    /// Receives a single message from the device
    fn recv(&mut self) -> Result<A2RMessage, ControllerReceiveError<Self::HardwareDriverError>>;
}
