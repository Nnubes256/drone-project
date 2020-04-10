use icaros_base::comms::air::scheduler::AirState;
use std::io::{BufRead, BufReader};
use serialport::{posix::TTYPort, prelude::*};
use nmea::Nmea;

use crate::hardware::serialport::SerialComunicationOptions;

pub struct GPSService {
    state: Nmea,
    driver: BufReader<TTYPort>
}

impl GPSService {
    pub fn new(options: SerialComunicationOptions) -> Result<Self, serialport::Error> {
        Ok(GPSService {
            state: Nmea::new(),
            driver: BufReader::with_capacity(1024, TTYPort::open(options.path, options.serial_options)?),
        })
    }

    pub fn update(&mut self) -> Result<bool, serialport::Error> {
        if self.driver.get_ref().bytes_to_read()? < 20 {
            return Ok(false);
        }

        let mut nmea_sentence: String = String::with_capacity(128);
        while self.driver.get_ref().bytes_to_read()? > 20 {
            self.driver.read_line(&mut nmea_sentence)?;
            for line in nmea_sentence.split("\n") {
                //info!("{}", line);
                match self.state.parse(&line) {
                    Ok(_sntc) => {
                        //info!("GPSService recv: {:?}", _sntc);
                    }, Err(_e) => {
                        //error!("Parse failure for GPS data: {}", _e);
                    }
                };
            }

        }

        Ok(true)
    }

    pub fn write_state<T>(&self, state: &mut T)
        where T: AirState
    {
        state.set_gps_coordinates((self.state.latitude(), self.state.longitude()), self.state.altitude());
    }
}
