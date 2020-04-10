//! Air-to-ground communication structures

use crate::comms::common::ApplicationPacket;
use crate::core::FullGPSData;
use core::{convert::TryFrom, fmt::Display};
use err_derive::Error;
use serde::{Deserialize, Serialize};
use std::{any::type_name, error::Error, fmt::Debug};

use crate::comms::common::{Acceleration, MotorSpeed, Orientation};

pub mod scheduler;

/// Constant value of the header byte
pub const HEADER_VALUE: u8 = 0x7F;

/// Obtains the `bincode2` configuration used for the ground control's
/// serialization/deserialization codec
pub fn get_air_codec() -> bincode2::Config {
    let mut config = bincode2::config();
    config.array_length(bincode2::LengthOption::U8);
    config.limit(32);
    config
}

///
/// Error ocurred when receving packets from ground control
///
/// `T` is the type of the driver-side errors
#[derive(Debug, Error)]
pub enum ReceiveError<T>
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

impl<T> From<bincode2::Error> for ReceiveError<T>
where
    T: 'static + Debug + Display + Error,
{
    fn from(value: bincode2::Error) -> Self {
        Self::DeserializationError(value)
    }
}

impl<T> ReceiveError<T>
where
    T: 'static + Debug + Display + Error,
{
    /// Wraps a driver-side error as a reception error
    pub fn from_driver_error(value: T) -> Self {
        Self::DriverRecvError(Box::new(value))
    }
}

///
/// Error ocurred when sending packets to ground control
///
/// `T` is the type of the driver-side errors
#[derive(Debug, Error)]
pub enum SendError<T>
where
    T: 'static + Error + Debug + Display,
{
    /// Packet serialization failed
    #[error(display = "Deserialization failed: {}", _0)]
    SerializationError(bincode2::Error),

    /// Error on the driver side
    #[error(display = "Hardware driver error: {}", _0)]
    DriverSendError(Box<T>),
}

impl<T> From<bincode2::Error> for SendError<T>
where
    T: Display,
    T: 'static + Error + Debug + Display,
{
    fn from(value: bincode2::Error) -> Self {
        Self::SerializationError(value)
    }
}

impl<T> SendError<T>
where
    T: 'static + Debug + Display + Error,
{
    /// Wraps a driver-side error as a sending error
    pub fn from_driver_error(value: T) -> Self {
        Self::DriverSendError(Box::new(value))
    }
}

///
/// A zero-sized type representing the lack of GPS data.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct GPSNoData;

///
/// The gamepad input given to the drone from ground control
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct G2AControllerAxisState {
    pub roll: i8,
    pub pitch: i8,
    pub yaw: i8,
    pub throttle: i8,
}

impl G2AControllerAxisState {
    /// Creates a gamepad input structure from its individual components
    pub fn new(roll: i8, pitch: i8, yaw: i8, throttle: i8) -> Self {
        G2AControllerAxisState {
            roll,
            pitch,
            yaw,
            throttle,
        }
    }
}

///
/// **UNUSED**
///
/// Payload for a Ground -> Drone `HEAR` packet
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct G2AHeartbeat {
    timestamp: u32,
}

impl G2AHeartbeat {
    /// Create a `HEAR` payload from its timestamp
    pub fn from_timestamp(timestamp: u32) -> Self {
        G2AHeartbeat { timestamp }
    }

    /// Returns this `HEAR` packet's timestamp
    pub fn timestamp(&self) -> u32 {
        self.timestamp
    }
}

///
/// Middle-layer internal structure involved in command serialization/deserialization.
///
/// For serialization, from its corresponding `enum`, each command type
/// (e.g. enum variant) is given a numerical identifier up to protocol specification (see
/// [`command_type!`](`command_type`)) which is represented as the
/// `command_type`, then the command's specific payload is serialized up-front
/// and inserted as a `data` buffer. The resulting structure can be then serialized
/// with the rest of the data to obtain the final serialized packet.
///
/// Similarly, for deserialization, the concerning part of the bitstream is first deserialized
/// as this type; then, depending on the deserialized value of the `command_type` field,
/// the `data` field is deserialized into the payload type corresponding to that command.
///
/// This is all handled automatically through the
/// `#[serde(try_from = "CommandTypeInternal")]`
/// and `#[serde(into = "CommandTypeInternal")]` annotations; this transformation is
/// done automatically while serializing/deserializing the entire data frame using `serde`.
#[derive(Debug, Serialize, Deserialize)]
struct CommandTypeInternal {
    command_type: u8,
    data: Vec<u8>,
}

///
/// Macro to quickly define, for a given packet kind, an enumeration of commands
/// it supports, conveying their numerical identifiers and the types
/// corresponding to those commands' specific payloads, additionally generating
/// their transformation and serialization code/annotations.
///
/// The syntax is:
///
/// ```ignore
/// command_type!(
///     /// Enum documentation
///     command CommandEnumName => {
///         /// COMMAND1 documentation
///         COMMAND1(0x00) -> Command1PayloadType,
///
///         /// COMMAND2 documentation
///         COMMAND2(0x03) -> Command2PayloadType,
///
///         /// COMMAND3 documentation
///         COMMAND3(0x05) -> Command3PayloadType,
///    }
///);
/// ```
macro_rules! command_type {
    (
        $(#[$outer:meta])* // Enum documentation
        command $cmdtypeenum: ident => { // Enum definition
            $(
                $(#[$inner:ident $($args:tt)*])* // Enum field documentation
                $commandname: ident($commandid: expr) -> $commanddatatype: ty // Enum field definition
            ),*
        }
    ) => {
        $(#[$outer])*
        #[derive(Debug, Clone, Serialize, Deserialize)]
        #[serde(try_from = "CommandTypeInternal")]
        #[serde(into = "CommandTypeInternal")]
        pub enum $cmdtypeenum {
            $(
                $(#[$inner $($args)*])*
                $commandname($commanddatatype),
            )*
        }

        impl $cmdtypeenum {
            /// Returns the numerical identifier corresponding to this command type
            fn get_command_id(&self) -> u8 {
                match self {
                    $(
                        $cmdtypeenum::$commandname(_) => $commandid,
                    )*
                }
            }
        }

        impl TryFrom<CommandTypeInternal> for $cmdtypeenum {
            type Error = Box<bincode2::ErrorKind>;

            fn try_from(internal: CommandTypeInternal) -> Result<Self, <Self as TryFrom<CommandTypeInternal>>::Error> {
                match internal.command_type {
                    $(
                        $commandid => Ok($cmdtypeenum::$commandname(get_air_codec().deserialize(&internal.data)?)),
                    )*
                    _ => Err(Box::new(bincode2::ErrorKind::Custom(format!("Unknown {}: {}", type_name::<$cmdtypeenum>(), internal.command_type))))
                }
            }
        }

        impl Into<CommandTypeInternal> for $cmdtypeenum {
            fn into(self) -> CommandTypeInternal {
                let ret = CommandTypeInternal {
                    command_type: self.get_command_id(),
                    data: match self {
                        $(
                            $cmdtypeenum::$commandname(data) => {
                                get_air_codec().serialize(&data).unwrap_or(Vec::new())
                            },
                        )*
                    }
                };

                ret
            }
        }
    }
}

command_type!(
    ///
    /// Defines the possible command types of an air-to-ground command
    command A2GCommandType => {
        /// Reports the drone's motor speed
        MOTR(0x02) -> MotorSpeed,

        /// Reports the drone's absolute orientation
        ORNT(0x03) -> Orientation,

        /// Reports the drone's linear acceleration
        ACEL(0x04) -> Acceleration,

        /// Conveys a message from an in-flight application to ground control
        APPM(0x10) -> ApplicationPacket,

        /// Reports the lack of full GPS data available
        GPSN(0x05) -> GPSNoData,

        /// Reports the drone's GPS position and altutude
        GPSD(0x06) -> FullGPSData
    }
);

command_type!(
    ///
    /// Defines the possible command types of a ground-to-air command
    command G2ACommandType => {
        /// Reports the ground control's desired inputs to the drone
        CNTA(0x02) -> G2AControllerAxisState,

        /// Conveys a message from ground control to an in-flight application
        APPM(0x10) -> ApplicationPacket,

        /// Reports that ground control is still active
        HEAR(0xFF) -> G2AHeartbeat
    }
);

///
/// Defines the full structure of a ground-to-air packet
#[derive(Debug, Serialize, Deserialize)]
pub struct G2AMessage {
    /// Message header
    pub header: u8,

    /// Packet counter
    pub counter: u16,

    /// Packet payload (i.e. the command being sent)
    pub command: G2ACommandType,
}

impl G2AMessage {
    /// Creates a new ground-to-air packet from its counter value and its payload
    pub fn new(counter: u16, command: G2ACommandType) -> Self {
        G2AMessage {
            header: HEADER_VALUE,
            counter,
            command,
        }
    }
}

///
/// Defines the full structure of a air-to-ground packet
#[derive(Debug, Serialize, Deserialize)]
pub struct A2GMessage {
    /// Message header
    pub header: u8,

    /// Packet counter
    pub counter: u16,

    /// Packet payload (i.e. the command being sent)
    pub command: A2GCommandType,
}

impl A2GMessage {
    /// Creates a new air-to-ground packet from its counter value and its payload
    pub fn new(counter: u16, command: A2GCommandType) -> Self {
        A2GMessage {
            header: HEADER_VALUE,
            counter,
            command,
        }
    }
}


///
/// An air-to-ground communication driver
///
/// This trait, when implemented, allows the implementor to act as a device driver for
/// communications with the ground control.
///
/// `Tx` is the type of the messages being transmitted
/// `Rx` is the type of the messages being received
pub trait AirCommunicationService<Tx, Rx>: Sized {
    /// The settings structure that has to be passed to the device driver
    type AirCommunicationOptions;

    /// The error type definining hardware-specific errors
    type HardwareDriverError: 'static + Error;


    /// Creates a new instance of the implementor with the given configuration
    fn setup(config: Self::AirCommunicationOptions) -> Result<Self, Self::HardwareDriverError>;

    /// Returns the maximum allowed size of application packets passing through this driver
    fn get_max_app_message_size() -> usize;

    /// Returns whether or not the underlying device is not yet ready for sending packets
    fn is_tx_busy(&mut self) -> bool;

    /// Sends a single message through the device
    fn send(&mut self, msg: Tx) -> Result<bool, SendError<Self::HardwareDriverError>>;

    /// Returns how many (or, depending on implementation, whether any) messages are available
    fn recv_available(&mut self) -> usize;

    /// Receives a single message from the device
    fn recv(&mut self) -> Result<Rx, ReceiveError<Self::HardwareDriverError>>;
}
