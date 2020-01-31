use crate::comms::controller::base::{
    CONTROLLER_HEADER_BYTE,
    A2RMessage, ControllerReceiveError,
    ControllerSendError, R2AMessage,
    get_controller_codec
};
use crate::utils::is_all_same;
use serialport::prelude::*;
use std::path::Path;
use serialport::posix::TTYPort;
use std::io::Read;
use std::io::Write;
use std::iter::Iterator;

use crate::comms::controller::base::ControllerCommunicationService;

pub struct SerialComunicationOptions<'a> {
    path: &'a Path,
    serial_options: &'a SerialPortSettings
}

pub struct SerialControllerCommunicationsService<'a> {
    driver: TTYPort,
    codec: bincode2::Config,
    options: SerialComunicationOptions<'a>,
    last_tx_message: Vec<u8>,
    last_rx_message: Vec<u8>
}

impl<'a> ControllerCommunicationService for SerialControllerCommunicationsService<'a> {
    type ControllerCommunicationOptions = SerialComunicationOptions<'a>;
    type HardwareDriverError = serialport::Error;

    fn setup(options: Self::ControllerCommunicationOptions) -> Result<Self, Self::HardwareDriverError> {
        Ok(SerialControllerCommunicationsService {
            driver: TTYPort::open(options.path, options.serial_options)?,
            codec: get_controller_codec(),
            options,
            last_tx_message: vec![0; 64],
            last_rx_message: vec![0; 64]
        })
    }

    fn send(&mut self, msg: R2AMessage) -> Result<bool, ControllerSendError<Self::HardwareDriverError>> {
        let message = self.codec.serialize(&msg)?;

        match self.driver.write(&message) {
            Ok(_) => Ok(true),
            Err(err) => Err(ControllerSendError::DriverSendError { inner: Box::new(serialport::Error::from(err))})
        }
    }

    fn recv_available(&mut self) -> Result<usize, Self::HardwareDriverError> {
        let bytes_available = self.driver.bytes_to_read()? as usize;
        Ok(bytes_available / 64)
    }

    fn recv(&mut self) -> Result<A2RMessage, ControllerReceiveError<Self::HardwareDriverError>> {
        let mut num_loops = 0;

        while match self.driver.read_exact(&mut self.last_rx_message[0 .. 1]) {
            Ok(_) => true,
            Err(err) => return Err(
                ControllerReceiveError::DriverRecvError {
                    inner: Box::new(serialport::Error::from(err))
                }
            )
        } {
            if self.last_rx_message[0] == CONTROLLER_HEADER_BYTE || num_loops >= 64 {
                break;
            } else {
                num_loops += 1;
            }
        }

        if self.last_rx_message[0] != CONTROLLER_HEADER_BYTE {
            return Err(ControllerReceiveError::NoPacketsAvailable);
        }

        match self.driver.read_exact(&mut self.last_rx_message[1 .. 64]) {
            Ok(_) => {},
            Err(err) => return Err(
                ControllerReceiveError::DriverRecvError {
                    inner: Box::new(serialport::Error::from(err))
                }
            )
        };

        if !self.verify_read_msg() {
            return Err(ControllerReceiveError::NoPacketsAvailable);
        }

        Ok(self.codec.deserialize(&self.last_rx_message)?)
    }
}

impl<'a> SerialControllerCommunicationsService<'a> {
    fn verify_read_msg(&self) -> bool {
        self.last_rx_message[0] == CONTROLLER_HEADER_BYTE
            && self.last_rx_message[0] == 0x00
            && is_all_same(&self.last_rx_message[44 .. 64])
    }
}
