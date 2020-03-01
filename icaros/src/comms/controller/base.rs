use std::error::Error;

use icaros_base::comms::controller::{
    A2RMessage, ControllerReceiveError, ControllerSendError, R2AMessage,
};

pub trait ControllerCommunicationService: Sized {
    type ControllerCommunicationOptions;
    type HardwareDriverError: 'static + Error;

    fn setup(
        config: Self::ControllerCommunicationOptions,
    ) -> Result<Self, Self::HardwareDriverError>;
    fn is_tx_busy(&mut self) -> bool;
    fn send(
        &mut self,
        msg: R2AMessage,
    ) -> Result<bool, ControllerSendError<Self::HardwareDriverError>>;
    fn recv_available(&mut self) -> Result<usize, Self::HardwareDriverError>;
    fn recv(&mut self) -> Result<A2RMessage, ControllerReceiveError<Self::HardwareDriverError>>;
}
