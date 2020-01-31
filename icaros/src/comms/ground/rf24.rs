use crate::comms::ground::base::{GroundCommunicationService, get_ground_codec};
#[cfg(target = "armv7-unknown-linux-gnueabihf")]
use rf24::{RF24, SPISpeed, DataRate, PowerAmplifierLevel};

pub struct RF24CommunicationService {
    driver: RF24,
    codec: bincode2::Config
}

pub struct RF24CommunicationOptions {
    pub ce_pin: u16,
    pub csn_pin: u16,
    pub bus_speed: SPISpeed,
    pub channel: u8,
    pub data_rate: DataRate,
    pub retries: u8,
    pub retries_delay: u8,
    pub address: &[u8],
    pub tx_power: PowerAmplifierLevel
}

impl GroundCommunicationService for RF24CommunicationService {
    type GroundCommunicationOptions = RF24CommunicationOptions;
    type HardwareDriverError = rf24::RF24Error;

    fn setup(options: Self::GroundCommunicationOptions) -> Result<Self, Self::HardwareDriverError> {
        let mut driver = RF24::from_spi_device_with_speed(
            options.ce_pin,
            options.csn_pin,
            options.bus_speed);
        if driver.begin() {
            driver.set_channel(options.channel);
            driver.set_data_rate(options.data_rate);
            driver.set_power_amplifier_level(options.tx_power);
            driver.set_dynamic_payloads(true);
            driver.enable_dynamic_ack();
            driver.set_retries(options.retries, options.retries_delay);
            driver.open_writing_pipe(options.address);
            driver.open_reading_pipe(0, options.address);
            driver.listen_start();
            if !driver.chip_connected() {
                Err(rf24::RF24Error::LibraryError)
            }
            Ok(RF24CommunicationService {
                driver,
                codec: get_ground_codec()
            }
        } else {
            Err(rf24::RF24Error::LibraryError)
        }
        driver.read_dynamic_payload(buffer: &mut Vec<u8>)
    }

    fn get_max_app_message_size() -> usize {
        25
    }

    fn send(&mut self, msg: A2GMessage) -> Result<bool, SendError<Self::HardwareDriverError>> {
        if self.driver.is_rx_full() {
            // TODO move this error to library itself and properly give it its own variant
            return Err(SendError::from_driver_error(rf24::RF24Error::LibraryError));
        }

        let message = self.codec.serialize(&msg)?;

        self.driver.write_ack_payload(0, &message);

        Ok(true)
    }

    fn recv_available(&mut self) -> usize {
        if self.driver.available() {
            1
        } else {
            0
        }
    }

    fn recv(&mut self) -> Result<G2AMessage, ReceiveError<Self::HardwareDriverError>> {
        if !self.driver.available() {
            return Err(ReceiveError::NoPacketsAvailable);
        }

        let mut message_bytes: Vec<u8> = Vec::with_capacity(32);
        match self.driver.read_dynamic_payload(&mut message_bytes) {
            Ok(has_packet) => {
                if !has_packet { return Err(ReceiveError::NoPacketsAvailable); }
            },
            Err(err) => return Err(ReceiveError::from_driver_error(err))
        }

        Ok(self.codec.deserialize(&message_bytes)?)
    }
}
