use serialport::prelude::*;
use std::path::Path;

/// The parameters required to initialize a serial port communications driver using the
/// `serialport` library.
pub struct SerialComunicationOptions<'a> {
    /// The path to the special Linux character file that represents the serial terminal
    /// we want to connect to.
    pub path: &'a Path,

    /// The settings required by the `serialport` library to estabilish a serial connection
    /// (i.e. baud rate, start bits, stop bits, etc.)
    pub serial_options: &'a SerialPortSettings,
}

impl<'a> SerialComunicationOptions<'a> {
    /// Creates a new set of options from the given serial port path and connection parameters
    pub fn from_path_and_options(
        path: &'a Path,
        serial_options: &'a serialport::SerialPortSettings,
    ) -> Self {
        SerialComunicationOptions {
            path,
            serial_options,
        }
    }
}
