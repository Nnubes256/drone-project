//! nRF24 implementation of an air-to-ground communication service (used in production)
//!
//! This implementation allows for communication with ground control through a dedicated radio
//! link between Nordic Semiconductor nRF24L01(+) transceiver modules, through interaction
//! with TMRh20's nRF24 C library.

use std::collections::VecDeque;
use icaros_base::comms::air::{get_air_codec, AirCommunicationService};
use icaros_base::comms::air::ReceiveError;
use icaros_base::comms::air::SendError;
use icaros_base::comms::air::A2GMessage;
use icaros_base::comms::air::G2AMessage;
use icaros_base::BincodeConfig;
use rf24::{DataRate, PowerAmplifierLevel, SPISpeed, RF24};

///
/// nRF24-based ground control communication service
pub struct RF24CommunicationService {
    driver: RF24,
    codec: BincodeConfig,
    tx_queue: VecDeque<Vec<u8>>,
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

impl AirCommunicationService<A2GMessage, G2AMessage> for RF24CommunicationService {
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
            driver.set_auto_ack(true); // Automatic acknowledgements have to be enabled
            driver.set_retries(options.retries, options.retries_delay);
            driver.set_dynamic_payloads(true); // Enable dynamically-sized payloads
            driver.enable_ack_payload(); // Enable sending data through acknowledgements
                                         // (used for two-way communication)
            // Open reading pipe 1 for listening (pipe 0 does not support ACK payloads)
            driver.open_reading_pipe(1, options.rx_address)?;

            // Start listening for messages
            driver.listen_start();

            // Check if the hardware is connected at this point
            if !driver.chip_connected() {
                error!("No chip connected!");
                return Err(rf24::RF24Error::LibraryError)
            }

            Ok(RF24CommunicationService {
                driver,
                tx_queue: VecDeque::new(),
                codec: get_air_codec(),
            })
        } else {
            Err(rf24::RF24Error::LibraryError)
        }
    }

    fn is_tx_busy(&mut self) -> bool {
        // We assume TX isn't busy, as the underlying driver doesn't allow us to
        // obtain this information
        false
    }

    fn get_max_app_message_size() -> usize {
        // (see protocol reference for details)
        25
    }

    fn send(&mut self, msg: A2GMessage) -> Result<bool, SendError<Self::HardwareDriverError>> {
        // If we attempt to send messages while the hardware's internal message queue is full,
        // the underlying driver will block until there's enough space on this queue for more
        // packets to be added in; bail out.
        if self.driver.is_rx_full() {
            return Err(SendError::DriverSendError(rf24::RF24Error::LibraryError.into()));
        }

        // Serialize the message to send
        let message = self.codec.serialize(&msg)?;

        // We'll send it through the next received packet's ACK payload.
        // Check if the sending queue is excessively filled; clear the oldest (i.e. next to be sent)
        // packet in the queue in such case, in order to avoid having a huge sending queue
        // (and therefore lagged-behind telemetry) if the ground control side suddenly stops
        // sending packets for a small period of time.
        if self.tx_queue.len() > 30 {
            self.tx_queue.pop_front();
        }

        // Push the message into the sending queue
        self.tx_queue.push_back(message);

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
        // No need to receive packets if there are none available
        if !self.driver.available() {
            return Err(ReceiveError::NoPacketsAvailable);
        }

        // If we have an message pending to be sent, we send it as part of the
        // acknowledgement packet for the message we are about to receive.
        // To do this, we queue it first into the device itself, so that once we pick the
        // packet we are receiving, the hardware itself will automatically send our message back
        // when handling message acknowledgements.
        if !self.tx_queue.is_empty() {
            // Pick up the next message
            let message = self.tx_queue.pop_front().unwrap();

            // Write it to the device
            match self.driver.write_ack_payload(1, &message) {
                Ok(()) => {},
                Err(err) => { error!("Had error while sending ack payload: {}", err) }
            };
        }

        // Read the incoming message
        let mut message_bytes: Vec<u8> = vec![0; 32];
        match self.driver.read_dynamic_payload(&mut message_bytes) {
            Ok(has_packet) => {
                if !has_packet {
                    return Err(ReceiveError::NoPacketsAvailable);
                }
            },
            Err(err) => return Err(ReceiveError::DriverRecvError(err.into())),
        };

        // We have some data, try to deserialize it!
        Ok(self.codec.deserialize(&message_bytes)?)
    }
}
