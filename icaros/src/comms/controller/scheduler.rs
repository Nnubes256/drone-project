//!
//! Packet scheduler for the flight controller and corresponding trait (interface) implementations
//!

use crate::core::SystemState;
use icaros_base::comms::{
    common::PacketScheduler,
    controller::{
        A2RMessage, ControllerCommunicationService, ControllerReceiveError,
        R2ADesiredRotationAndThrottle, R2AMessage,
    },
};
use std::{error::Error, mem};

///
/// Packet scheduler for the flight controller
///
/// `T` defines the underlying device driver to which packets are scheduled
pub struct ControllerPacketScheduler<T: ControllerCommunicationService> {
    comm_service: T,
    tx_in: Option<R2AMessage>,
    latency_recv: u32,
    counter: u16,
}

impl<T: ControllerCommunicationService> ControllerPacketScheduler<T> {
    /// Initializes a new flight controller packet scheduler, together with its underlying driver
    /// and given the latter's settings to be used
    pub fn initialize(
        config: T::ControllerCommunicationOptions,
    ) -> Result<Self, T::HardwareDriverError> {
        Ok(ControllerPacketScheduler {
            comm_service: T::setup(config)?,
            tx_in: None,
            latency_recv: 0,
            counter: 0,
        })
    }

    /// Generates a new outgoing message from the given system state and schedules them
    /// for sending
    fn generate_message_from_state(&mut self, state: &SystemState) {
        self.tx_in = Some(R2AMessage::new(
            R2ADesiredRotationAndThrottle::from_controller(&state.ground.controller_state),
        ))
    }

    /// Updates the given system state from the given incoming message
    fn update_state_from_message(&mut self, state: &mut SystemState, message: A2RMessage) {
        // Check that the packet is not out of order (i.e. within reasonable counter bounds)
        if std::u16::MAX - self.counter < 10 || (message.counter as i32 - self.counter as i32) < 10 {
            // Write data to the state
            state.drone.acceleration = message.acceleration;
            state.drone.orientation = message.orientation;
            state.drone.motor_speeds = message.motor_speed;
            self.counter = message.counter;
        } else {
            warn!(target: "ControllerPacketScheduler", "out-of-order packet {} (had {})",
                message.counter, self.counter);
        }
    }
}

impl<T> PacketScheduler<A2RMessage, R2AMessage, SystemState> for ControllerPacketScheduler<T>
where
    T: ControllerCommunicationService,
{
    fn can_send(&mut self) -> bool {
        !self.comm_service.is_tx_busy()
    }

    fn send_state(&mut self, state: &SystemState) -> Result<bool, Box<dyn Error>> {
        // Generate a new message from the given state
        self.generate_message_from_state(state);

        // If the message was successfully generated, send it
        if self.tx_in.is_some() {
            // And make sure we don't send a packet twice by moving it into
            // the temporary variable `packet`, placing a null value in its place
            let packet = mem::replace(&mut self.tx_in, None).unwrap();

            let result = self.comm_service.send(packet)?;
            Ok(result)
        } else {
            Ok(false)
        }
    }

    fn recv_state(&mut self, state: &mut SystemState) -> Result<bool, Box<dyn Error>> {
        let mut any_packet_received = false;

        // While we have packets available, receive them
        while self.comm_service.recv_available()? > 0 {
            any_packet_received = true;
            let packet = match self.comm_service.recv() {
                Ok(packet) => packet,
                Err(err) => match err {
                    ControllerReceiveError::NoPacketsAvailable => {
                        self.latency_recv += 1;
                        if self.latency_recv > 100 {
                            warn!("[ControllerPacketScheduler] Have not received anything for {} > 100 ticks!", self.latency_recv);
                        }
                        return Ok(false);
                    }
                    _ => return Err(err.into()),
                },
            };

            self.latency_recv = 0;

            // From them, update our own state
            self.update_state_from_message(state, packet);
        }

        Ok(any_packet_received)
    }
}
