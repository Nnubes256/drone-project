//! nRF24 implementation of an ground-to-air communication service (used in production)
//!
//! This implementation allows for communication with the drone through a dedicated radio
//! link between Nordic Semiconductor nRF24L01(+) transceiver modules, through interaction
//! with TMRh20's nRF24 C library.

use std::time::Instant;
use icaros_base::BincodeConfig;
use icaros_base::comms::air::{get_air_codec, AirCommunicationService, SendError, ReceiveError, G2AMessage, A2GMessage};
use rf24::{DataRate, PowerAmplifierLevel, SPISpeed, RF24};

///
/// nRF24-based ground control communication service
pub struct RF24CommunicationService {
    driver: RF24,
    codec: BincodeConfig,
    timer: Instant
}

///
/// Settings structure for [`RF24CommunicationService`](RF24CommunicationService)
pub struct RF24CommunicationOptions {
    /// Chip Enable pin number (SPI)
    pub ce_pin: u16,

    /// Chip Select Not pin number (SPI)
    pub csn_pin: u16,

    /// SPI bus speed
    pub bus_speed: SPISpeed,

    /// Radio channel to use
    pub channel: u8,

    /// Communication data rate
    pub data_rate: DataRate,

    /// Number of transmission retries
    pub retries: u8,

    /// Delay between retries
    pub retries_delay: u8,

    /// Our own 5-bit address (for receiving)
    pub rx_address: &'static [u8],

    /// Destination's 5-bit address (for transmitting)
    pub tx_address: &'static [u8],

    /// Power amplifier's strength
    pub tx_power: PowerAmplifierLevel
}

impl AirCommunicationService<G2AMessage, A2GMessage> for RF24CommunicationService {
    type AirCommunicationOptions = RF24CommunicationOptions;
    type HardwareDriverError = rf24::RF24Error;

    fn setup(options: Self::AirCommunicationOptions) -> Result<Self, Self::HardwareDriverError> {
        // Initalize the underlying device driver
        let mut driver =
            RF24::from_spi_device_with_speed(options.ce_pin, options.csn_pin, options.bus_speed);

        // Start the driver and check if the hardware itself is available
        if driver.begin() {
            // Configure the hardware with the passed options
            driver.set_channel(options.channel);
            driver.set_data_rate(options.data_rate);
            driver.set_power_amplifier_level(options.tx_power);
            driver.enable_ack_payload(); // Enable sending data through acknowledgements
                                         // (used for two-way communication)
            driver.set_dynamic_payloads(true); // Enable dynamically-sized payloads
            driver.set_auto_ack(true); // Automatic acknowledgements have to be enabled
            driver.set_retries(options.retries, options.retries_delay);

            // Open writing pipe for sending packets
            driver.open_writing_pipe(options.tx_address)?;

            // Reinitalize the hardware's listening state by quickly toggling between
            // listening states
            driver.listen_start();
            driver.listen_stop();

            // Check if the hardware is connected at this point
            if !driver.chip_connected() {
                error!("No chip connected!");
                return Err(rf24::RF24Error::LibraryError);
            }

            Ok(RF24CommunicationService {
                driver,
                codec: get_air_codec(),
                timer: Instant::now()
            })
        } else {
            Err(rf24::RF24Error::LibraryError)
        }
    }

    fn is_tx_busy(&mut self) -> bool {
        // We send packets at maximum 1000 Hz (or as fast as possible)
        self.timer.elapsed().as_micros() < 1000
    }

    fn get_max_app_message_size() -> usize {
        // (see protocol reference for details)
        25
    }

    fn send(&mut self, msg: G2AMessage) -> Result<bool, SendError<Self::HardwareDriverError>> {
        self.timer = Instant::now();

        // Serialize the message to send
        let message = self.codec.serialize(&msg)?;

        // Send the message without blocking
        match self.driver.write_fast(&message) {
            Ok(recv) => Ok(recv),
            Err(err) => Err(SendError::from_driver_error(err))
        }
    }

    fn recv_available(&mut self) -> usize {
        if self.driver.available() {
            1
        } else {
            0
        }
    }

    fn recv(&mut self) -> Result<A2GMessage, ReceiveError<Self::HardwareDriverError>> {
        // No need to receive packets if there are none available
        if !self.driver.available() {
            return Err(ReceiveError::NoPacketsAvailable);
        }

        // Read the incoming message
        let mut message_bytes: Vec<u8> = vec![0; 32];
        match self.driver.read_dynamic_payload(&mut message_bytes) {
            Ok(has_packet) => {
                if !has_packet {
                    return Err(ReceiveError::NoPacketsAvailable);
                }
            }
            Err(err) => return Err(ReceiveError::from_driver_error(err)),
        }

        // We have some data, try to deserialize it!
        Ok(self.codec.deserialize(&message_bytes)?)
    }
}
