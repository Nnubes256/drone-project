//! Core routing facilities and base logic

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
use std::thread::{self, JoinHandle};
use std::sync::Arc;
use std::default::Default;
use std::io::Error as IOError;
use parking_lot::{RwLock, RwLockWriteGuard};
use tokio::sync::mpsc;
use serde::{Serialize};
use icaros_base::comms::common::PacketScheduler;
use crate::web::webserver_main;

/// Complete ground control system state
#[derive(Default, Serialize)]
pub struct GroundSystemState {
    /// State received from the drone (i.e. orientation, acceleration, motor speed, etc.)
    pub drone: DroneState,

    /// Self state (i.e. processed gamepad state)
    pub ground: GroundControlState,

    /// Gamepad primary (unprocessed) state
    pub gamepad: ControlState,

    /// A queue where incoming messages are read from
    /// Application messages received from the drone are pushed onto this queue
    pub rx_app_messages: VecDeque<ApplicationPacket>,

    /// Test variable: x
    pub x: f32,

    /// Test variable: y
    pub y: f32,

    /// Test variable: z
    pub z: f32,

    /// Test variable: timestamp
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
            None => { // If we don't have GPS data, we just empty it
                self.drone.gps.altitude = None;
                self.drone.gps.latitude = None;
                self.drone.gps.longitude = None;
            }
        }
    }
}

type DroneAirPacketScheduler<A> = AirPacketScheduler<A, GroundSystemState, G2AMessage, A2GMessage>;

///
/// The central controller for all the systems around the ICAROS ground control.
///
/// Takes care of updating each one of them individually, if applicable, and routing the
/// data that each of those systems gets.
pub struct ICAROSGroundController<T>
    where T: AirCommunicationService<G2AMessage, A2GMessage>
{
    /// The system state, shared between the core ICAROS control system and its webserver
    state: Arc<RwLock<GroundSystemState>>,

    /// The packet scheduler for the drone side
    packet_scheduler: DroneAirPacketScheduler<T>,

    /// The gamepad control service
    gamepad: Box<dyn GamepadService>,

    /// A handle to the webserver thread
    webserver_thread: Option<JoinHandle<()>>,

    /// Sending part of the message-passing interface with the webserver
    webserver_tx: Option<mpsc::UnboundedSender<ControllerRXWebserverMessage>>,

    /// Receiving part of the message-passing interface with the webserver
    webserver_rx: Option<mpsc::UnboundedReceiver<ControllerTXWebserverMessage>>,

    /// Webserver data update timer
    webserver_update_time: Instant,

    /// Gamepad data update timer
    gamepad_update_time: Instant,
}

impl<T> ICAROSGroundController<T>
    where T: AirCommunicationService<G2AMessage, A2GMessage>
{
    /// Initializes the central controller with the given drone packet scheduler.
    pub fn new(
        air_sched: DroneAirPacketScheduler<T>
    ) -> Self {
        info!("[ICAROSGroundController] Ready!");
        ICAROSGroundController {
            state: Arc::new(RwLock::new(GroundSystemState::default())),
            webserver_thread: None,
            packet_scheduler: air_sched,
            gamepad: Box::new(PhysicalGamepadService::new().unwrap()),
            webserver_tx: None,
            webserver_rx: None,
            webserver_update_time: Instant::now(),
            gamepad_update_time: Instant::now(),
        }
    }

    /// Intialize the core functionality and the rest of the subsystems (i.e. the webserver).
    pub fn start(&mut self) -> Result<(), IOError> {
        // Create the message-passing interface for interaction with the web server
        let (tx_in, rx_in) = mpsc::unbounded_channel();
        let (tx_out, rx_out) = mpsc::unbounded_channel();
        self.webserver_tx = Some(tx_in);
        self.webserver_rx = Some(rx_out);

        {
            // Obtain a shared reference to our state
            let state = self.state.clone();

            // Launch the thread
            self.webserver_thread = Some(thread::Builder::new()
                .name("Web server thread".to_string())
                .spawn(move || {
                    info!(target: "webserver", "Webserver thread init");
                    webserver_main(state, tx_out, rx_in);
                })?);
        }

        Ok(())
    }

    /// Processes a single tick. This will handle, in order:
    /// - Receiving new state from the gamepad
    /// - Updating the test state
    /// - Send new state to the drone
    /// - Receiving new state and incoming application-specific packets from the drone
    /// - Inform the web server that new data is available
    pub fn process(&mut self) {
        // Receive new state from the gamepad (at 50 Hz)
        if self.gamepad_update_time.elapsed().as_millis() > 20 {
            self.gamepad_update_time = Instant::now();
            let mut gamepad_state = ControlState::default();

            match self.gamepad.process(&mut gamepad_state) {
                Ok(_) => { // Lock scope begin
                    // Success in updating it! We transform the gamepad data into
                    // something we can send to the drone and store both raw and processed
                    // data into our state.
                    let mut state_w = self.state.write();
                    self.gamepad.control_state_into_message_state(&mut state_w.ground.controller_state, &gamepad_state);
                    state_w.gamepad = gamepad_state;
                }, // Lock scope end
                Err(err) => {
                    error!("Error while updating controller inputs: {}", err)
                }
            };
        }

        { // Lock scope begin
            // We update our test data
            let mut state_w = self.state.write();
            state_w.time += 1;
            state_w.x = (state_w.time as f64 * 0.05).sin() as f32;
            state_w.y = ((state_w.time + 2000) as f64 * 0.05).sin() as f32;
            state_w.z = ((state_w.time + 4000) as f64 * 0.05).sin() as f32;

            // No more writing is required from this point; downgrade to a read-only lock
            let state_r = RwLockWriteGuard::downgrade(state_w);

            // Send our own state to the drone
            if self.packet_scheduler.can_send() {
                match self.packet_scheduler.send_state(&state_r) {
                    Ok(_) => {},
                    Err(err) => {
                        error!("[ICAROSController] controller scheduler TX error: {}", err);
                    }
                }
            }
        } // Lock scope end

        { // Lock scope begin
            let mut state_w = self.state.write();

            // If packets are available from the drone, receive them and update our state
            match self.packet_scheduler.recv_state(&mut state_w) {
                Ok(_) => {},
                Err(err) => {
                    error!("[ICAROSController] ground scheduler RX error: {}", err);
                }
            }

            // Receive any incoming application packets sent from the drone too,
            // and append them to our incoming application messages queue
            match self.packet_scheduler.recv_app_packets() {
                Ok(pkts) => {
                    state_w.rx_app_messages.append(pkts);
                },
                Err(err) => {
                    error!("[ICAROSController] error while reading application packets: {}", err)
                }
            }
        } // Lock scope end

        // At 50 Hz, signal the webserver to update
        if self.webserver_update_time.elapsed().as_millis() > 20 {
            self.webserver_update_time = Instant::now();
            if let Some(tx) = &self.webserver_tx {
                // ...by sending a message to it to do so
                match tx.send(ControllerRXWebserverMessage::Update) {
                    Ok(_) => {},
                    Err(e) => {
                        error!("Error sending update notif to web thread: {}", e);
                    }
                };
            }
        }
    }
}
