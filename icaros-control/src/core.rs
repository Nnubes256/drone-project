use icaros_base::comms::common::ApplicationPacket;
use std::collections::VecDeque;
use std::time::Instant;
use icaros_base::comms::air::AirCommunicationService;
use icaros_base::comms::air::G2AMessage;
use icaros_base::comms::air::A2GMessage;
use icaros_base::comms::air::scheduler::AirPacketScheduler;
use icaros_base::comms::air::G2AControllerAxisState;
use icaros_base::comms::common::{MotorSpeed, Orientation, Acceleration, ApplicationPacketScheduler};
use icaros_base::comms::air::scheduler::GroundState;
use crate::gamepad::{ControlState, GamepadService, PhysicalGamepadService};
use icaros_base::core::{GroundControlState, DroneState, FullGPSData};
use crate::web::{ControllerRXWebserverMessage, ControllerTXWebserverMessage};
use std::time::Duration;
use std::thread::{self, JoinHandle};
use std::sync::Arc;
use std::default::Default;
use std::io::Error as IOError;
use parking_lot::{RwLock, RwLockWriteGuard};
use tokio::sync::mpsc;
use serde::{Serialize};
use icaros_base::comms::common::PacketScheduler;


use crate::web::webserver_main;

#[derive(Default, Serialize)]
pub struct GroundSystemState {
    pub drone: DroneState,
    pub ground: GroundControlState,
    pub gamepad: ControlState,
    pub rx_app_messages: VecDeque<ApplicationPacket>,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub time: u64
}

impl GroundState for GroundSystemState {
    fn set_motor_speeds(&mut self, value: MotorSpeed) {
        self.drone.motor_speeds = value;
    }

    fn set_orientation(&mut self, value: Orientation) {
        self.drone.orientation = value;
    }

    fn set_acceleration(&mut self, value: Acceleration) {
        self.drone.acceleration = value;
    }

    fn get_controller_state(&self) -> G2AControllerAxisState {
        self.ground.controller_state.clone()
    }

    fn set_gps_coordinates(&mut self, value: Option<FullGPSData>) {
        match value {
            Some(gps_data) => self.drone.gps = gps_data.into(),
            None => {
                self.drone.gps.altitude = None;
                self.drone.gps.latitude = None;
                self.drone.gps.longitude = None;
            }
        }
    }
}

type DroneAirPacketScheduler<A> = AirPacketScheduler<A, GroundSystemState, G2AMessage, A2GMessage>;

pub struct ICAROSGroundController<T>
    where T: AirCommunicationService<G2AMessage, A2GMessage>
{
    state: Arc<RwLock<GroundSystemState>>,
    packet_scheduler: DroneAirPacketScheduler<T>,
    gamepad: Box<dyn GamepadService>,
    _temp_gamepad_state: ControlState,
    webserver_thread: Option<JoinHandle<()>>,
    webserver_tx: Option<mpsc::UnboundedSender<ControllerRXWebserverMessage>>,
    webserver_rx: Option<mpsc::UnboundedReceiver<ControllerTXWebserverMessage>>,
    webserver_update_time: Instant,
    gamepad_update_time: Instant,
}

impl<T> ICAROSGroundController<T>
    where T: AirCommunicationService<G2AMessage, A2GMessage> {
    pub fn new(
        air_sched: DroneAirPacketScheduler<T>
    ) -> Self {
        ICAROSGroundController {
            state: Arc::new(RwLock::new(GroundSystemState::default())),
            webserver_thread: None,
            packet_scheduler: air_sched,
            gamepad: Box::new(PhysicalGamepadService::new().unwrap()),
            _temp_gamepad_state: ControlState::default(),
            webserver_tx: None,
            webserver_rx: None,
            webserver_update_time: Instant::now(),
            gamepad_update_time: Instant::now(),
        }
    }

    pub fn start(&mut self) -> Result<(), IOError> {
        let (tx_in, rx_in) = mpsc::unbounded_channel();
        let (tx_out, rx_out) = mpsc::unbounded_channel();
        self.webserver_tx = Some(tx_in);
        self.webserver_rx = Some(rx_out);
        {
            let state = self.state.clone();
            self.webserver_thread = Some(thread::Builder::new()
                .name("Web server thread".to_string())
                .spawn(move || {
                    info!(target: "webserver", "Webserver thread init");
                    webserver_main(state, tx_out, rx_in);
                })?);
        }
        Ok(())
    }

    pub fn process(&mut self) {
        if self.gamepad_update_time.elapsed().as_millis() > 20 {
            self.gamepad_update_time = Instant::now();
            match self.gamepad.process(&mut self._temp_gamepad_state) {
                Ok(_) => {
                    let mut state_w = self.state.write();
                    state_w.gamepad = self._temp_gamepad_state.clone();
                    self.gamepad.control_state_into_message_state(&mut state_w.ground.controller_state, &self._temp_gamepad_state);
                },
                Err(err) => {
                    error!("Error while updating controller inputs: {}", err)
                }
            };
        }

        {
            let mut state_w = self.state.write();
            state_w.time += 1;
            state_w.x = (state_w.time as f64 * 0.05).sin() as f32;
            state_w.y = ((state_w.time + 2000) as f64 * 0.05).sin() as f32;
            state_w.z = ((state_w.time + 4000) as f64 * 0.05).sin() as f32;

            let state_r = RwLockWriteGuard::downgrade(state_w);
            if self.packet_scheduler.can_send() {
                match self.packet_scheduler.send_state(&state_r) {
                    Ok(_) => {},
                    Err(err) => {
                        error!("[ICAROSController] controller scheduler TX error: {}", err);
                    }
                }
            }
        }

        {
            let mut state_w = self.state.write();
            match self.packet_scheduler.recv_state(&mut state_w) {
                Ok(_) => {},
                Err(err) => {
                    error!("[ICAROSController] ground scheduler RX error: {}", err);
                }
            }

            match self.packet_scheduler.recv_app_packets() {
                Ok(pkts) => {
                    state_w.rx_app_messages.append(pkts);
                },
                Err(err) => {
                    error!("[ICAROSController] error while reading application packets: {}", err)
                }
            }
        }
        
        if self.webserver_update_time.elapsed().as_millis() > 20 {
            self.webserver_update_time = Instant::now();
            if let Some(tx) = &self.webserver_tx {
                match tx.send(ControllerRXWebserverMessage::Update) {
                    Ok(_) => {},
                    Err(e) => {
                        error!("Error sending update notif to web thread: {}", e);
                    }
                };
            }
        }

        thread::sleep(Duration::from_micros(200));
    }
}
