//! Serial port implementation of a flight controller communication service (used in production)

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

/// Byte value used to signal the start of a data frame
pub const SERIAL_START_BYTE: u8 = 0x9F;

/// Byte value used to signal the end of a data frame
pub const SERIAL_END_BYTE: u8 = 0x8F;

/// Byte value used to escape the next byte value from being interpreted in a special way
pub const SERIAL_ESCAPE_BYTE: u8 = 0x7F;

///
/// Serial-port-based flight controller communication service
pub struct SerialControllerCommunicationsService<'a> {
    driver: TTYPort,
    codec: bincode2::Config,
    _options: SerialComunicationOptions<'a>,
    last_tx_message: Vec<u8>,
    last_rx_message: Vec<u8>,
    last_msg: Instant,
    tx_priority: bool,
}

/// Current frame parsing state
enum ParsingState {
    WaitingForHeader,
    ReadingMessage,
    ReadingEscapeByte,
}

///
/// An error that occurred on the serial communication
#[derive(Debug, Error)]
pub enum SerialCommunicationError {
    /// Serial Input/Output error
    /// Usually an error happening at operating system level
    #[error(display = "Serial I/O error: {}", _0)]
    IOError(#[source] ::std::io::Error),

    /// Error on the underlying library
    #[error(display = "Serial driver error: {}", _0)]
    DriverError(#[source] serialport::Error),

    /// CRC mismatch error
    #[error(
        display = "CRC mismatch (expected {:x}, obtained {:x})",
        expected,
        obtained
    )]
    CRCMismatch { expected: u16, obtained: u16 },

    /// Corrupt padding error
    #[error(display = "Bad padding")]
    BadPadding,

    /// Packet found with invalid length
    #[error(
        display = "Invalid packet length (expected {}, obtained {})",
        expected,
        obtained
    )]
    InvalidLength { expected: usize, obtained: usize },

    /// No packets were able to be found
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
        // Only allow sending packets at 50 Hz (20 miliseconds per packet)
        self.last_msg.elapsed().as_millis() < 20
    }

    fn send(
        &mut self,
        msg: R2AMessage,
    ) -> Result<bool, ControllerSendError<Self::HardwareDriverError>> {
        debug!("[SerialController] send: {:?}", msg);

        // Serialize and encode the message for transmission through the serial port
        SerialControllerCommunicationsService::encode_message(
            &mut self.last_tx_message,
            &(self.codec.serialize(&msg)?),
        );

        debug!("[SerialController] send: {:x?}", self.last_tx_message);

        self.last_msg = Instant::now();
        self.tx_priority = false;

        // Send the encoded message through the serial port
        match self.driver.write(&self.last_tx_message) {
            Ok(_) => Ok(true),
            Err(err) => Err(ControllerSendError::DriverSendError(Box::new(
                SerialCommunicationError::from(err),
            ))),
        }
    }

    fn recv_available(&mut self) -> Result<usize, Self::HardwareDriverError> {
        // We may have a packet available if more than 64 bytes are cached (messages have
        // a minimum lenght of 64 bytes, including padding)
        let bytes_available = self.driver.bytes_to_read()? as usize;
        Ok(bytes_available / 64) // Note this is an integer division, so it will round down.
    }

    fn recv(&mut self) -> Result<A2RMessage, ControllerReceiveError<Self::HardwareDriverError>> {
        let mut crc = 0;

        // Read the incoming message
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

        // Verify that the message received is valid
        self.verify_read_msg(bytes_read, crc)?;

        // Attempt to deserialize the received message
        let msg = self.codec.deserialize(&self.last_rx_message)?;
        debug!("[SerialController] recv: {:?}", msg);

        Ok(msg)
    }
}

impl<'a> SerialControllerCommunicationsService<'a> {
    /// Verifies that the message read adheres to some sanity checks.
    /// This avoids undergoing deserialization of messages on certain common scenarios where such
    /// parsing would surely fail.
    fn verify_read_msg(
        &self,
        bytes_read: usize,
        obtained_crc: u16,
    ) -> Result<(), SerialCommunicationError> {
        // Messages have a length 64 bytes when including padding
        if bytes_read != 64 {
            return Err(SerialCommunicationError::InvalidLength {
                expected: 64,
                obtained: bytes_read,
            });
        }

        // Messages include a padding from byte 38 to the end of message with constant value 0x01
        if self.last_rx_message[38] != 0x01 || !is_all_same(&self.last_rx_message[38..64]) {
            return Err(SerialCommunicationError::BadPadding);
        }

        // Messages include a CRC, calculated on the full 64-byte message, which must match
        // with the one we calculate ourselves
        let calculated_crc = crc16_calculate(&self.last_rx_message);

        if calculated_crc != obtained_crc {
            return Err(SerialCommunicationError::CRCMismatch {
                expected: obtained_crc,
                obtained: calculated_crc,
            });
        }

        Ok(())
    }

    /// Attempts to decode a message from a reader, returning its CRC and contents
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
        out.clear(); // Clear the contents of the output vector

        debug!("BEGIN");

        // While we haven't exhausted our tries...
        'parseloop: while tries > 0 && bytes_parsed <= 128 {
            // Read a single byte
            let byte_read = match reader.read(&mut read_buf) {
                Ok(_) => {
                    debug!("0x{:x} ", read_buf[0]);
                    read_buf[0]
                }
                Err(ref err) if err.kind() == io::ErrorKind::WouldBlock => { // No byte yet received
                    tries -= 1;
                    continue;
                },
                Err(err) => return Err(SerialCommunicationError::IOError(err)),
            };

            // Treat the received byte based on the state we are currently on
            match state {
                ParsingState::WaitingForHeader => { // Waiting for the start of the message
                    debug!("WAITING");

                    // Have we received the message start byte?
                    if byte_read == SERIAL_START_BYTE {
                        // Yes, we start reading
                        debug!("--> READING");
                        state = ParsingState::ReadingMessage;
                    } else {
                        debug_vec.push(byte_read);
                    }
                }
                ParsingState::ReadingMessage => { // Reading the message
                    debug!("READING");

                    // Have we received the message stop byte?
                    if byte_read == SERIAL_END_BYTE {
                        // We are done reading the message!
                        debug!("--> DONE");
                        break 'parseloop;
                    // Have we received the escape byte?
                    } else if byte_read == SERIAL_ESCAPE_BYTE {
                        // We escape the next byte
                        debug!("--> ESCAPE");
                        state = ParsingState::ReadingEscapeByte;
                    // Otherwise, as long as we haven't parsed too many bytes, we can make
                    // the assumption we have read a part of the payload
                    } else if bytes_parsed < 128 {
                        out.push(byte_read);
                        bytes_parsed += 1;
                    }
                }
                ParsingState::ReadingEscapeByte => { // Handling the escape byte
                    debug!("ESCAPE");
                    // As long as we haven't parsed too many bytes, the next byte we read
                    // is part of the payload
                    if bytes_parsed < 128 {
                        out.push(byte_read);
                        bytes_parsed += 1;
                    }

                    // The escape byte only covers a single character, so we get back
                    // to normal reading
                    state = ParsingState::ReadingMessage;
                }
            }
        }

        // We print extraneous data received before a message could be found to aid
        // flight controller debugging
        debug!("Debug info:\n{}", String::from_utf8_lossy(&debug_vec));

        // If we hung up for too long trying to read packets, we assume
        // no valid packets were read
        if tries <= 0 {
            return Err(SerialCommunicationError::NoPacketsAvailable);
        }

        // If we parsed far too little bytes to extract the CRC out of the payload,
        // bail out early
        if bytes_parsed < 3 {
            return Err(SerialCommunicationError::InvalidLength {
                expected: 64,
                obtained: bytes_parsed,
            });
        }

        // Read and extract the message CRC out of the message
        *crc = LittleEndian::read_u16(&out[bytes_parsed - 2..bytes_parsed]);
        out.truncate(out.len().saturating_sub(2));

        debug!("END");

        Ok(bytes_parsed - 2)
    }

    /// Attempts to encode a message from a reader and returns its bitstream,
    /// ready to be sent through the serial connection
    fn encode_message(out: &mut Vec<u8>, data: &Vec<u8>) {
        let mut crc: u16 = 0;

        // Clear the contents of the output vector
        out.clear();

        // We always start with the start byte
        out.push(SERIAL_START_BYTE);

        // Then, add each of the bytes we want to send
        for &byte in data {
            // If its value is equal to those of the escape or end bytes, we push the escape byte
            // before writing the actual byte, ensuring the byte is always read as data
            if byte == SERIAL_END_BYTE || byte == SERIAL_ESCAPE_BYTE {
                out.push(SERIAL_ESCAPE_BYTE);
            }
            out.push(byte);
            crc = _crc16_update(crc, byte); // We update the CRC with the added byte on each step
        }

        // The entirety of the payload has been pushed, and its CRC has been calculated;
        // we push the CRC bytes as we did with the payload
        for &byte in crc.to_le_bytes().iter() {
            // If its value is equal to those of start and end bytes, we push the escape byte
            // before writing the actual byte, ensuring the byte is always read as data
            if byte == SERIAL_END_BYTE || byte == SERIAL_ESCAPE_BYTE {
                out.push(SERIAL_ESCAPE_BYTE);
            }
            out.push(byte);
        }

        // Finally, we end with the end byte
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
