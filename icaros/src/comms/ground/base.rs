use core::convert::TryFrom;
use core::fmt::Display;
use serde::Deserialize;
use serde::Serialize;
use std::any::type_name;

use crate::comms::common::Acceleration;
use crate::comms::common::MotorSpeed;
use crate::comms::common::Orientation;
use snafu::Snafu;

pub fn get_ground_codec() -> bincode2::Config {
    let mut config = bincode2::config();
    config.array_length(bincode2::LengthOption::U8);
    config.limit(32);
    config
}

#[derive(Debug, Snafu)]
pub enum ReceiveError<T>
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

impl<T> From<bincode2::Error> for ReceiveError<T>
where
    T: Display,
{
    fn from(value: bincode2::Error) -> Self {
        Self::DeserializationError { inner: value }
    }
}

#[derive(Debug, Snafu)]
pub enum SendError<T>
where
    T: Display,
{
    #[snafu(display("Deserialization failed: {}", inner))]
    SerializationError { inner: bincode2::Error },
    #[snafu(display("Hardware driver error: {}", inner))]
    DriverSendError { inner: Box<T> },
}

impl<T> From<bincode2::Error> for SendError<T>
where
    T: Display,
{
    fn from(value: bincode2::Error) -> Self {
        Self::SerializationError { inner: value }
    }
}

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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApplicationMessage {
    application_id: u8,
    message: Vec<u8>,
}

impl ApplicationMessage {
    pub fn new(app_id: u8, message: Vec<u8>) -> Self {
        ApplicationMessage {
            application_id: app_id,
            message,
        }
    }

    pub fn app_id(&self) -> u8 {
        self.application_id
    }

    pub fn message(&self) -> &Vec<u8> {
        &self.message
    }

    pub fn message_iter(&self) -> ::std::slice::Iter<u8> {
        self.message.iter()
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
                        $commandid => Ok($cmdtypeenum::$commandname(get_ground_codec().deserialize(&internal.data)?)),
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
                                get_ground_codec().serialize(&data).unwrap_or(Vec::new())
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
    APPM(0x10) -> ApplicationMessage
});

command_type!(G2ACommandType => {
    CNTA(0x02) -> G2AControllerAxisState,
    APPM(0x10) -> ApplicationMessage,
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

pub trait GroundCommunicationService {
    type GroundCommunicationOptions;
    type HardwareDriverError: Display;

    fn setup(config: Self::GroundCommunicationOptions) -> Self;
    fn get_max_app_message_size() -> usize;
    fn send(&mut self, msg: A2GMessage) -> Result<bool, SendError<Self::HardwareDriverError>>;
    fn recv_available(&mut self) -> usize;
    fn recv(&mut self) -> Result<G2AMessage, ReceiveError<Self::HardwareDriverError>>;
}
