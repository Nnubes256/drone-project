use serialport::prelude::*;
use std::path::Path;

pub struct SerialComunicationOptions<'a> {
    pub path: &'a Path,
    pub serial_options: &'a SerialPortSettings,
}

impl<'a> SerialComunicationOptions<'a> {
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
