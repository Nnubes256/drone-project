use crate::hardware::serialport::SerialComunicationOptions;
use byteorder::{ByteOrder, LittleEndian};
use err_derive::Error;
use icaros_base::{
    comms::controller::{
        get_controller_codec, A2RMessage, ControllerCommunicationService, ControllerReceiveError,
        ControllerSendError, R2AMessage,
    },
    utils::{
        crc16::{_crc16_update, crc16_calculate},
        is_all_same,
    },
};
use serialport::{posix::TTYPort, prelude::*};
use std::{
    io,
    io::{Read, Write},
    time::Instant,
};

pub const SERIAL_START_BYTE: u8 = 0x9F;
pub const SERIAL_END_BYTE: u8 = 0x8F;
pub const SERIAL_ESCAPE_BYTE: u8 = 0x7F;

pub struct SerialControllerCommunicationsService<'a> {
    driver: TTYPort,
    codec: bincode2::Config,
    _options: SerialComunicationOptions<'a>,
    last_tx_message: Vec<u8>,
    last_rx_message: Vec<u8>,
    last_msg: Instant,
    tx_priority: bool,
}

enum ParsingState {
    WaitingForHeader,
    ReadingMessage,
    ReadingEscapeByte,
}

#[derive(Debug, Error)]
pub enum SerialCommunicationError {
    #[error(display = "Serial I/O error: {}", _0)]
    IOError(#[source] ::std::io::Error),
    #[error(display = "Serial driver error: {}", _0)]
    DriverError(#[source] serialport::Error),
    #[error(
        display = "CRC mismatch (expected {:x}, obtained {:x})",
        expected,
        obtained
    )]
    CRCMismatch { expected: u16, obtained: u16 },
    #[error(display = "Bad padding")]
    BadPadding,
    #[error(
        display = "Invalid packet length (expected {}, obtained {})",
        expected,
        obtained
    )]
    InvalidLength { expected: usize, obtained: usize },
    #[error(display = "No packets available")]
    NoPacketsAvailable,
}

impl From<SerialCommunicationError> for ControllerReceiveError<SerialCommunicationError> {
    fn from(value: SerialCommunicationError) -> Self {
        ControllerReceiveError::DriverRecvError(Box::new(value))
    }
}

impl<'a> ControllerCommunicationService for SerialControllerCommunicationsService<'a> {
    type ControllerCommunicationOptions = SerialComunicationOptions<'a>;
    type HardwareDriverError = SerialCommunicationError;

    fn setup(
        options: Self::ControllerCommunicationOptions,
    ) -> Result<Self, Self::HardwareDriverError> {
        Ok(SerialControllerCommunicationsService {
            driver: TTYPort::open(options.path, options.serial_options)?,
            codec: get_controller_codec(),
            _options: options,
            last_tx_message: vec![0; 64],
            last_rx_message: vec![0; 64],
            last_msg: Instant::now(),
            tx_priority: true,
        })
    }

    fn is_tx_busy(&mut self) -> bool {
        self.last_msg.elapsed().as_millis() < 20
    }

    fn send(
        &mut self,
        msg: R2AMessage,
    ) -> Result<bool, ControllerSendError<Self::HardwareDriverError>> {
        info!("[SerialController] send: {:?}", msg);
        SerialControllerCommunicationsService::encode_message(
            &mut self.last_tx_message,
            &self.codec.serialize(&msg)?,
        );
        info!("[SerialController] send: {:x?}", self.last_tx_message);

        self.last_msg = Instant::now();
        self.tx_priority = false;

        match self.driver.write(&self.last_tx_message) {
            Ok(_) => Ok(true),
            Err(err) => Err(ControllerSendError::DriverSendError(Box::new(
                SerialCommunicationError::from(err),
            ))),
        }
    }

    fn recv_available(&mut self) -> Result<usize, Self::HardwareDriverError> {
        let bytes_available = self.driver.bytes_to_read()? as usize;
        Ok(bytes_available / 64)
    }

    fn recv(&mut self) -> Result<A2RMessage, ControllerReceiveError<Self::HardwareDriverError>> {
        let mut crc = 0;
        let bytes_read = match SerialControllerCommunicationsService::read_msg(
            &mut self.driver,
            &mut self.last_rx_message,
            &mut crc,
        ) {
            Ok(bytes_read) => bytes_read,
            Err(err) => match err {
                SerialCommunicationError::NoPacketsAvailable => {
                    return Err(ControllerReceiveError::NoPacketsAvailable)
                }
                _ => return Err(err.into()),
            },
        };
        debug!(target: "SerialControllerCommunicationsService", "rx: {:?}", self.last_rx_message);
        self.verify_read_msg(bytes_read, crc)?;
        let msg = self.codec.deserialize(&self.last_rx_message)?;
        debug!("[SerialController] recv: {:?}", msg);
        Ok(msg)
    }
}

impl<'a> SerialControllerCommunicationsService<'a> {
    fn verify_read_msg(
        &self,
        bytes_read: usize,
        obtained_crc: u16,
    ) -> Result<(), SerialCommunicationError> {
        if bytes_read != 64 {
            return Err(SerialCommunicationError::InvalidLength {
                expected: 64,
                obtained: bytes_read,
            });
        }

        if self.last_rx_message[38] != 0x01 || !is_all_same(&self.last_rx_message[38..64]) {
            return Err(SerialCommunicationError::BadPadding);
        }

        let calc_crc = crc16_calculate(&self.last_rx_message);

        if calc_crc != obtained_crc {
            return Err(SerialCommunicationError::CRCMismatch {
                expected: obtained_crc,
                obtained: calc_crc,
            });
        }

        Ok(())
    }

    fn read_msg(
        reader: &mut dyn Read,
        out: &mut Vec<u8>,
        crc: &mut u16,
    ) -> Result<usize, SerialCommunicationError> {
        let mut tries = 4;
        let mut bytes_parsed = 0;
        let mut state = ParsingState::WaitingForHeader;
        let mut read_buf = [0u8];
        let mut debug_vec = Vec::with_capacity(128);
        out.clear();
        debug!("BEGIN");

        'parseloop: while tries > 0 && bytes_parsed <= 128 {
            let byte_read = match reader.read(&mut read_buf) {
                Ok(_) => {
                    debug!("0x{:x} ", read_buf[0]);
                    read_buf[0]
                }
                Err(err) => match err.kind() {
                    io::ErrorKind::WouldBlock => {
                        tries -= 1;
                        continue;
                    }
                    _ => return Err(SerialCommunicationError::IOError(err)),
                },
            };

            match state {
                ParsingState::WaitingForHeader => {
                    debug!("WAITING");
                    if byte_read == SERIAL_START_BYTE {
                        state = ParsingState::ReadingMessage;
                    } else {
                        debug_vec.push(byte_read);
                    }
                }
                ParsingState::ReadingMessage => {
                    debug!("READING");
                    if byte_read == SERIAL_END_BYTE {
                        break 'parseloop;
                    } else if byte_read == SERIAL_ESCAPE_BYTE {
                        state = ParsingState::ReadingEscapeByte;
                    } else if bytes_parsed < 128 {
                        out.push(byte_read);
                        bytes_parsed += 1;
                    }
                }
                ParsingState::ReadingEscapeByte => {
                    debug!("ESCAPE");
                    if bytes_parsed < 128 {
                        out.push(byte_read);
                        bytes_parsed += 1;
                    }
                    state = ParsingState::ReadingMessage;
                }
            }
        }

        info!("Debug info:\n{}", String::from_utf8_lossy(&debug_vec));

        if tries <= 0 {
            return Err(SerialCommunicationError::NoPacketsAvailable);
        }

        if bytes_parsed < 3 {
            return Err(SerialCommunicationError::InvalidLength {
                expected: 64,
                obtained: bytes_parsed,
            });
        }

        *crc = LittleEndian::read_u16(&out[bytes_parsed - 2..bytes_parsed]);
        out.truncate(out.len().saturating_sub(2));

        debug!("END");

        Ok(bytes_parsed - 2)
    }

    fn encode_message(out: &mut Vec<u8>, data: &Vec<u8>) {
        let mut crc: u16 = 0;

        out.clear();

        out.push(SERIAL_START_BYTE);
        for &byte in data {
            if byte == SERIAL_END_BYTE || byte == SERIAL_ESCAPE_BYTE {
                out.push(SERIAL_ESCAPE_BYTE);
            }
            out.push(byte);
            crc = _crc16_update(crc, byte);
        }
        for &byte in crc.to_le_bytes().iter() {
            if byte == SERIAL_END_BYTE || byte == SERIAL_ESCAPE_BYTE {
                out.push(SERIAL_ESCAPE_BYTE);
            }
            out.push(byte);
        }
        out.push(SERIAL_END_BYTE);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_and_decode_message() {
        let test_message = vec![0x4A, 0x4B, 0x4C, 0x4D, 0x4E, SERIAL_START_BYTE, SERIAL_END_BYTE, 0x4F, 0x4E, SERIAL_ESCAPE_BYTE];
        let test_message_len = test_message.len();
        let mut test_message_out = Vec::new();

        SerialControllerCommunicationsService::encode_message(&mut test_message_out, &test_message);

        let expected = vec![
            SERIAL_START_BYTE,
            0x4A, 0x4B, 0x4C, 0x4D, 0x4E,
            SERIAL_START_BYTE, SERIAL_ESCAPE_BYTE, SERIAL_END_BYTE,
            0x4F, 0x4E,
            SERIAL_ESCAPE_BYTE, SERIAL_ESCAPE_BYTE,
            0x0D, 0xD7, // CRC
            SERIAL_END_BYTE,
        ];
        println!("Expected: {:?}", expected);
        println!("Obtained: {:?}", test_message_out);
        assert_eq!(
            expected
                .iter()
                .zip(test_message_out)
                .all(|(a, b)| (*a == b)),
            true
        );

        let mut test_message_back = Vec::new();
        let mut obtained_crc = 0;

        match SerialControllerCommunicationsService::read_msg(
            &mut expected.as_slice(),
            &mut test_message_back,
            &mut obtained_crc,
        ) {
            Ok(_) => {
                println!("Expected (decode): {:?}", test_message);
                println!("Obtained (decode): {:?}", test_message_back);
                assert_eq!(obtained_crc, 0xD70D);
                assert_eq!(test_message_back.len(), test_message_len);
                assert_eq!(
                    test_message_back
                        .iter()
                        .zip(test_message)
                        .all(|(a, b)| (*a == b)),
                    true
                )
            }
            Err(err) => panic!(err),
        };
    }
}
