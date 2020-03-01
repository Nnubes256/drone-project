use crate::comms::common::ApplicationPacket;
use crate::core::FullGPSData;
use core::{convert::TryFrom, fmt::Display};
use err_derive::Error;
use serde::{Deserialize, Serialize};
use std::{any::type_name, error::Error, fmt::Debug};

use crate::comms::common::{Acceleration, MotorSpeed, Orientation};

pub mod scheduler;

pub const HEADER_VALUE: u8 = 0x7F;

pub fn get_air_codec() -> bincode2::Config {
    let mut config = bincode2::config();
    config.array_length(bincode2::LengthOption::U8);
    config.limit(32);
    config
}

#[derive(Debug, Error)]
pub enum ReceiveError<T>
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

impl<T> From<bincode2::Error> for ReceiveError<T>
where
    T: 'static + Debug + Display + Error,
{
    fn from(value: bincode2::Error) -> Self {
        Self::DeserializationError(value)
    }
}

#[derive(Debug, Error)]
pub enum SendError<T>
where
    T: 'static + Error + Debug + Display,
{
    #[error(display = "Deserialization failed: {}", _0)]
    SerializationError(bincode2::Error),
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

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct GPSNoData;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct G2AControllerAxisState {
    pub roll: i8,
    pub pitch: i8,
    pub yaw: i8,
    pub throttle: i8,
}

impl G2AControllerAxisState {
    pub fn new(roll: i8, pitch: i8, yaw: i8, throttle: i8) -> Self {
        G2AControllerAxisState {
            roll,
            pitch,
            yaw,
            throttle,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct G2AHeartbeat {
    timestamp: u32,
}

impl G2AHeartbeat {
    pub fn from_timestamp(timestamp: u32) -> Self {
        G2AHeartbeat { timestamp }
    }

    pub fn timestamp(&self) -> u32 {
        self.timestamp
    }
}

#[derive(Debug, Serialize, Deserialize)]
struct CommandTypeInternal {
    command_type: u8,
    data: Vec<u8>,
}

macro_rules! command_type {
    ($cmdtypeenum: ident => {$($commandname: ident($commandid: expr) -> $commanddatatype: ty),*}) => {
        #[derive(Debug, Clone, Serialize, Deserialize)]
        #[serde(try_from = "CommandTypeInternal")]
        #[serde(into = "CommandTypeInternal")]
        pub enum $cmdtypeenum {
            $(
                $commandname($commanddatatype),
            )*
        }

        impl $cmdtypeenum {
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

command_type!(A2GCommandType => {
    MOTR(0x02) -> MotorSpeed,
    ORNT(0x03) -> Orientation,
    ACEL(0x04) -> Acceleration,
    APPM(0x10) -> ApplicationPacket,
    GPSN(0x05) -> GPSNoData,
    GPSD(0x06) -> FullGPSData
});

command_type!(G2ACommandType => {
    CNTA(0x02) -> G2AControllerAxisState,
    APPM(0x10) -> ApplicationPacket,
    HEAR(0xFF) -> G2AHeartbeat
});

#[derive(Debug, Serialize, Deserialize)]
pub struct G2AMessage {
    pub header: u8,
    pub counter: u16,
    pub command: G2ACommandType,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct A2GMessage {
    pub header: u8,
    pub counter: u16,
    pub command: A2GCommandType,
}

impl G2AMessage {
    pub fn new(counter: u16, command: G2ACommandType) -> Self {
        G2AMessage {
            header: HEADER_VALUE,
            counter,
            command,
        }
    }
}

impl A2GMessage {
    pub fn new(counter: u16, command: A2GCommandType) -> Self {
        A2GMessage {
            header: HEADER_VALUE,
            counter,
            command,
        }
    }
}

pub trait AirCommunicationService<Tx, Rx>: Sized {
    type AirCommunicationOptions;
    type HardwareDriverError: 'static + Error;

    fn setup(config: Self::AirCommunicationOptions) -> Result<Self, Self::HardwareDriverError>;
    fn get_max_app_message_size() -> usize;
    fn is_tx_busy(&mut self) -> bool;
    fn send(&mut self, msg: Tx) -> Result<bool, SendError<Self::HardwareDriverError>>;
    fn recv_available(&mut self) -> usize;
    fn recv(&mut self) -> Result<Rx, ReceiveError<Self::HardwareDriverError>>;
}
