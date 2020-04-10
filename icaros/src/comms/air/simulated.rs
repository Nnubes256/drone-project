//! Simulated implementation of an air-to-ground communication service (used to test data
//! sending/reception to the flight controller without a proper ground control present)

use err_derive::Error;
use icaros_base::comms::air::{
    get_air_codec, A2GMessage, AirCommunicationService, G2ACommandType, G2AControllerAxisState,
    G2AMessage, ReceiveError, SendError,
};

pub struct SimulatedGroundData {
    controller_state: G2AControllerAxisState,
    timestamp: u32,
    counter: u16,
}

impl SimulatedGroundData {
    pub fn new() -> Self {
        SimulatedGroundData {
            controller_state: G2AControllerAxisState::new(0, 0, 0, 0),
            timestamp: 0,
            counter: 0,
        }
    }
}

#[derive(Debug, Error)]
pub enum DummyError {
    #[error(display = "{}", _0)]
    Error(String),
}

impl From<String> for DummyError {
    fn from(value: String) -> Self {
        DummyError::Error(value)
    }
}

pub struct SimulatedAirController {
    pub ground_queue: Vec<Vec<u8>>,
    pub drone_queue: Vec<Vec<u8>>,
    codec: bincode2::Config,
    ground_data: SimulatedGroundData,
}

impl AirCommunicationService<A2GMessage, G2AMessage> for SimulatedAirController {
    type AirCommunicationOptions = ();
    type HardwareDriverError = DummyError;
    fn setup(_: Self::AirCommunicationOptions) -> Result<Self, Self::HardwareDriverError> {
        info!("[AirDummyController] Ready!");
        Ok(SimulatedAirController {
            ground_queue: Vec::new(),
            drone_queue: Vec::new(),
            codec: get_air_codec(),
            ground_data: SimulatedGroundData::new(),
        })
    }

    fn send(&mut self, msg: A2GMessage) -> Result<bool, SendError<Self::HardwareDriverError>> {
        let _ = match self.codec.serialize(&msg) {
            Ok(msg_bytes) => msg_bytes,
            Err(e) => {
                return Err(SendError::SerializationError(e));
            }
        };

        self.simulate_msg_sent();

        Ok(true)
    }

    fn recv_available(&mut self) -> usize {
        self.drone_queue.len()
    }

    fn recv(&mut self) -> Result<G2AMessage, ReceiveError<Self::HardwareDriverError>> {
        let new_msg = match self.drone_queue.pop() {
            Some(el) => el,
            None => return Err(ReceiveError::NoPacketsAvailable),
        };

        match self.codec.deserialize::<G2AMessage>(&new_msg) {
            Ok(message) => Ok(message),
            Err(err) => Err(ReceiveError::DeserializationError(err)),
        }
    }

    fn get_max_app_message_size() -> usize {
        26
    }

    fn is_tx_busy(&mut self) -> bool {
        false
    }
}

impl SimulatedAirController {
    /// Generates and/or updates the simulated data based on the previous state
    fn simulate_msg_sent(&mut self) {
        let timestamp_float = self.ground_data.timestamp as f64;
        if timestamp_float.sin() > 0.0 {
            self.ground_data.controller_state.roll += 1;
        } else {
            self.ground_data.controller_state.roll -= 1;
        }

        if timestamp_float.cos() > 0.0 {
            self.ground_data.controller_state.pitch += 1;
        } else {
            self.ground_data.controller_state.pitch -= 1;
        }

        self.ground_data.controller_state.yaw = 0;
        self.ground_data.controller_state.throttle = 64;

        self.send_drone_message(G2AMessage::new(
            self.ground_data.counter,
            G2ACommandType::CNTA(self.ground_data.controller_state.clone()),
        ));
        let (temp0, _) = self.ground_data.timestamp.overflowing_add(1);
        self.ground_data.timestamp = temp0;
        let (temp1, _) = self.ground_data.counter.overflowing_add(1);
        self.ground_data.counter = temp1;
    }

    fn send_drone_message(&mut self, msg: G2AMessage) {
        self.drone_queue.push(self.codec.serialize(&msg).unwrap());
    }
}
