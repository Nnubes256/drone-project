//! Simulated implementation of a flight controller communication service (used to test data
//! sending/reception to the ground control without a proper flight controller present)

use std::time::Instant;
use err_derive::Error;
use icaros_base::{
    comms::{
        common::{Acceleration, MotorSpeed, Orientation},
        controller::{
            get_controller_codec, A2RMessage, ControllerCommunicationService,
            ControllerReceiveError, ControllerSendError, R2AMessage,
        },
    },
    utils::{Point3, Quartenion},
};

pub struct SimulatedControllerData {
    motor: MotorSpeed,
    accel: Acceleration,
    orient: Orientation,
    counter: u16,
}

impl SimulatedControllerData {
    pub fn new() -> Self {
        SimulatedControllerData {
            motor: MotorSpeed::from_speeds((0, 0, 0, 0)),
            accel: Acceleration::from_vec3(Point3::from_components(0.0, 0.0, 0.0)),
            orient: Orientation::from_quartenion(Quartenion::from_values(0.0, 0.0, 0.0, 0.0)),
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

pub struct SimulatedControllerController {
    pub ground_queue: Vec<Vec<u8>>,
    pub drone_queue: Vec<Vec<u8>>,
    codec: bincode2::Config,
    controller_data: SimulatedControllerData,
    timer: Instant,
}

impl ControllerCommunicationService for SimulatedControllerController {
    type ControllerCommunicationOptions = ();
    type HardwareDriverError = DummyError;
    fn setup(_: Self::ControllerCommunicationOptions) -> Result<Self, Self::HardwareDriverError> {
        info!("[ControllerDummyController] Ready!");
        Ok(SimulatedControllerController {
            ground_queue: Vec::new(),
            drone_queue: Vec::new(),
            codec: get_controller_codec(),
            controller_data: SimulatedControllerData::new(),
            timer: Instant::now()
        })
    }

    fn send(
        &mut self,
        msg: R2AMessage,
    ) -> Result<bool, ControllerSendError<Self::HardwareDriverError>> {
        let _ = match self.codec.serialize(&msg) {
            Ok(msg_bytes) => msg_bytes,
            Err(e) => {
                return Err(ControllerSendError::SerializationError(e));
            }
        };

        //info!("[SimulatedControllerController] recv: {:?}", msg);
        self.simulate_msg_sent(msg);
        self.timer = Instant::now();

        Ok(true)
    }

    fn recv_available(&mut self) -> Result<usize, Self::HardwareDriverError> {
        Ok(self.drone_queue.len())
    }

    fn recv(&mut self) -> Result<A2RMessage, ControllerReceiveError<Self::HardwareDriverError>> {
        let new_msg = match self.drone_queue.pop() {
            Some(el) => el,
            None => return Err(ControllerReceiveError::NoPacketsAvailable),
        };

        match self.codec.deserialize::<A2RMessage>(&new_msg) {
            Ok(message) => Ok(message),
            Err(err) => Err(ControllerReceiveError::DeserializationError(err)),
        }
    }

    fn is_tx_busy(&mut self) -> bool {
        self.timer.elapsed().as_millis() < 4
    }
}

impl SimulatedControllerController {
    /// Generates and/or updates the simulated data based on the previous state
    fn simulate_msg_sent(&mut self, msg: R2AMessage) {
        let mut accel_vec = self.controller_data.accel.as_point3_mut();
        accel_vec.x = msg.rpyt.roll as f32 * 0.8;
        accel_vec.y = msg.rpyt.pitch as f32 * 0.8;
        accel_vec.z = msg.rpyt.throttle as f32 * 0.007 - 5.0;

        let mut orient_quat = self.controller_data.orient.as_quartenion_mut();
        orient_quat.w = 0.1 * f32::sin(self.controller_data.counter as f32);
        orient_quat.x = 0.1 * f32::cos(self.controller_data.counter as f32);
        orient_quat.y = 0.1 * f32::sin(-(self.controller_data.counter as f32));
        orient_quat.z = 0.1 * f32::sin(self.controller_data.counter as f32);

        let mut motor_speed = &mut self.controller_data.motor;
        let roll = msg.rpyt.roll as f32;
        let pitch = msg.rpyt.pitch as f32;
        let yaw = msg.rpyt.yaw as f32;
        let throttle = msg.rpyt.throttle as f32;
        motor_speed.tl = (throttle - pitch - roll + yaw) as u16;
        motor_speed.tr = (throttle - pitch + roll - yaw) as u16;
        motor_speed.bl = (throttle + pitch - roll - yaw) as u16;
        motor_speed.br = (throttle + pitch + roll + yaw) as u16;

        self.send_drone_message(A2RMessage::new(
            self.controller_data.counter,
            self.controller_data.motor.clone(),
            self.controller_data.orient.clone(),
            self.controller_data.accel.clone(),
        ));
        let (temp0, _) = self.controller_data.counter.overflowing_add(1);
        self.controller_data.counter = temp0;
    }

    fn send_drone_message(&mut self, msg: A2RMessage) {
        //info!("[SimulatedControllerController] send: {:?}", msg);
        self.drone_queue.push(self.codec.serialize(&msg).unwrap());
    }
}
