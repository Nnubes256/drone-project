use std::error::Error;

use icaros_base::comms::air::{A2GMessage, G2AMessage, ReceiveError, SendError};

pub trait AirCommunicationService: Sized {
    type AirCommunicationOptions;
    type HardwareDriverError: 'static + Error;

    fn setup(config: Self::AirCommunicationOptions) -> Result<Self, Self::HardwareDriverError>;
    fn get_max_app_message_size() -> usize;
    fn is_tx_busy(&mut self) -> bool;
    fn send(&mut self, msg: A2GMessage) -> Result<bool, SendError<Self::HardwareDriverError>>;
    fn recv_available(&mut self) -> usize;
    fn recv(&mut self) -> Result<G2AMessage, ReceiveError<Self::HardwareDriverError>>;
}
