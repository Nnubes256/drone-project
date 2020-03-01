use crate::core::SystemState;
use icaros_base::comms::{
    common::PacketScheduler,
    controller::{
        A2RMessage, ControllerCommunicationService, ControllerReceiveError,
        R2ADesiredRotationAndThrottle, R2AMessage,
    },
};
use std::{error::Error, mem};

pub struct ControllerPacketScheduler<T: ControllerCommunicationService> {
    comm_service: T,
    tx_in: Option<R2AMessage>,
    latency_recv: u32,
    counter: u16,
}

impl<T: ControllerCommunicationService> ControllerPacketScheduler<T> {
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

    fn generate_message_from_state(&mut self, state: &SystemState) {
        self.tx_in = Some(R2AMessage::new(
            R2ADesiredRotationAndThrottle::from_controller(&state.ground.controller_state),
        ))
    }

    fn update_state_from_message(&mut self, state: &mut SystemState, message: A2RMessage) {
        if std::u16::MAX - self.counter < 10 || (message.counter as i32 - self.counter as i32) < 10 {
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
        self.generate_message_from_state(state);
        if self.tx_in.is_some() {
            let packet = mem::replace(&mut self.tx_in, None).unwrap();
            let result = self.comm_service.send(packet)?;
            self.tx_in = None;
            Ok(result)
        } else {
            Ok(false)
        }
    }

    fn recv_state(&mut self, state: &mut SystemState) -> Result<bool, Box<dyn Error>> {
        let mut any_packet_received = false;
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

            self.update_state_from_message(state, packet);
        }

        Ok(any_packet_received)
    }
}
