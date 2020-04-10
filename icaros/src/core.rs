//! Core routing facilities and base logic


//
// Imports

use std::error::Error;
use icaros_base::comms::air::G2AMessage;
use icaros_base::comms::air::A2GMessage;
use crate::esb::ExternalServiceBroker;
use crate::hardware::gps::GPSService;
use crate::comms::{
    controller::scheduler::ControllerPacketScheduler,
};
use std::convert::TryInto;
use core::fmt::Display;
use icaros_base::{
    core::{DroneState, GroundControlState, FullGPSData},
    comms::{
        air::{AirCommunicationService, G2AControllerAxisState},
        air::scheduler::{AirPacketScheduler, AirState},
        common::{Acceleration, MotorSpeed, Orientation, PacketScheduler, ApplicationPacketScheduler},
        controller::ControllerCommunicationService,
    },
};

///
/// Holds the system statistics; essentially, the amount of events that have happened since the
/// statistics were last reset.
#[derive(Default)]
pub struct SystemStatistics {
    /// Number of driver reception updates for the ground control side (i.e. successful data received)
    pub ground_rx_updates: u32,

    /// Number of driver reception updates for the flight controller side (i.e. successful data received)
    pub controller_rx_updates: u32,

    /// Number of driver reception idle loops for the ground control side (i.e. no data received)
    pub ground_rx_idles: u32,

    /// Number of driver reception idle loops for the flight controller side (i.e. no data received)
    pub controller_rx_idles: u32,

    /// Number of driver reception errors for the ground control side (i.e. error while receiving data)
    pub ground_rx_errors: u32,

    /// Number of driver reception errors for the flight controller side (i.e. error while receiving data)
    pub controller_rx_errors: u32,

    /// Number of states sent for the ground control side
    pub ground_tx: u32,

    /// Number of states sent for the flight controller side
    pub controller_tx: u32,

    /// Number of states that failed to send for the ground control side
    pub ground_tx_errors: u32,

    /// Number of states that failed to send for the flight controller side
    pub controller_tx_errors: u32,
}

impl Display for SystemStatistics {
    fn fmt(&self, fmt: &mut std::fmt::Formatter<'_>) -> Result<(), std::fmt::Error> {
        write!(
            fmt,
            "controller tx:{}:{} rx:{}:{}:{} | ground tx:{}:{} rx:{}:{}:{}",
            self.controller_tx,
            self.controller_tx_errors,
            self.controller_rx_updates,
            self.controller_rx_idles,
            self.controller_rx_errors,
            self.ground_tx,
            self.ground_tx_errors,
            self.ground_rx_updates,
            self.ground_rx_idles,
            self.ground_rx_errors
        )
    }
}

impl SystemStatistics {

    ///
    /// Resets all the statistics to their T=0 values
    pub fn reset(&mut self) {
        self.ground_rx_updates = 0;
        self.controller_rx_updates = 0;
        self.ground_rx_idles = 0;
        self.controller_rx_idles = 0;
        self.ground_rx_errors = 0;
        self.controller_rx_errors = 0;
        self.ground_tx = 0;
        self.controller_tx = 0;
        self.ground_tx_errors = 0;
        self.controller_tx_errors = 0;
    }
}

/// Complete system state
pub struct SystemState {
    /// Self state (i.e. orientation, acceleration, motor speed, etc.)
    pub drone: DroneState,

    /// State received from ground control (i.e. gamepad state)
    pub ground: GroundControlState,

    /// Global system statistics
    pub stats: SystemStatistics,
}

impl SystemState {
    /// Create a new, initial system state
    pub fn new() -> Self {
        SystemState {
            drone: DroneState::new(),
            ground: GroundControlState::new(),
            stats: SystemStatistics::default(),
        }
    }
}

impl AirState for SystemState {
    fn get_motor_speeds(&self) -> MotorSpeed {
        self.drone.motor_speeds.clone()
    }

    fn get_orientation(&self) -> Orientation {
        self.drone.orientation.clone()
    }

    fn get_acceleration(&self) -> Acceleration {
        self.drone.acceleration.clone()
    }

    fn set_controller_state(&mut self, new: G2AControllerAxisState) {
        self.ground.controller_state = new;
    }

    fn set_gps_coordinates(&mut self, new_latlon: (Option<f64>, Option<f64>), alt: Option<f32>) {
        // Only set the latitude if we have any
        if let Some(lat) = new_latlon.0 {
            self.drone.gps.latitude = Some(lat);
        } else {
            self.drone.gps.latitude = None;
        }

        // Only set the longitude if we have any
        if let Some(lon) = new_latlon.1 {
            self.drone.gps.longitude = Some(lon);
        } else {
            self.drone.gps.longitude = None;
        }

        // Only set the altitude if we have any
        if let Some(alt) = alt {
            self.drone.gps.altitude = Some(alt);
        } else {
            self.drone.gps.longitude = None;
        }
    }

    fn get_gps_coordinates(&self) -> Option<FullGPSData> {
        match self.drone.gps.clone().try_into() {
            Ok(v) => Some(v),
            Err(_) => None
        }
    }
}

type DroneAirPacketScheduler<A> = AirPacketScheduler<A, SystemState, A2GMessage, G2AMessage>;

///
/// The central controller for all the systems around ICAROS. Takes care of updating each one
/// of them individually and of handling the state that is to be passed around to each system.
pub struct ICAROSController<A, B>
where
    A: ControllerCommunicationService,
    B: AirCommunicationService<A2GMessage, G2AMessage>,
{
    /// The system state
    sys_data: SystemState,

    /// The packet scheduler for the controller side
    controller_scheduler: ControllerPacketScheduler<A>,

    /// The packet scheduler for the ground control side
    air_scheduler: DroneAirPacketScheduler<B>,

    /// The GPS service (optional)
    gps_device: Option<GPSService>,

    /// The external service broker
    esb: ExternalServiceBroker,
}

impl<A, B> ICAROSController<A, B>
where
    A: ControllerCommunicationService,
    B: AirCommunicationService<A2GMessage, G2AMessage>,
{
    /// Initializes the central controller with the given ground control and flight controller
    /// packet schedulers and, optionally, an initialized GPS device driver. May fail if the
    /// external service broker fails to start.
    pub fn init(
        controller_sched: ControllerPacketScheduler<A>,
        air_sched: DroneAirPacketScheduler<B>,
        gps_device: Option<GPSService>
    ) -> Result<Self, Box<dyn Error>> {
        info!("[ICAROSController] Ready!");

        // Initialize the external service broker, and try to start it.
        //let mut esb = ExternalServiceBroker::new();
        let mut esb = ExternalServiceBroker::with_camera();
        esb.start()?;

        // Return the new controller
        Ok(ICAROSController {
            sys_data: SystemState::new(),
            controller_scheduler: controller_sched,
            air_scheduler: air_sched,
            gps_device,
            esb,
        })
    }

    /// Processes a single tick. This will handle, in order:
    /// - Receiving new state from the flight controller
    /// - Receiving new state from the ground control
    /// - Route incoming application-specific packets from the ground control
    ///   to the external service broker
    /// - If applicable, receiving current positional data from the GPS service
    /// - Send new state to the flight controller
    /// - Send new state to the ground control
    /// - Route outgoing application-specific messages to the ground control
    ///   to the external service broker
    pub fn process(&mut self) {
        // Receive new state from the flight controller
        match self.controller_scheduler.recv_state(&mut self.sys_data) {
            Ok(updated) => match updated {
                true => self.sys_data.stats.controller_rx_updates += 1, // New state was received
                false => self.sys_data.stats.controller_rx_idles += 1,  // No state was received
            },
            Err(err) => {
                error!("[ICAROSController] controller scheduler RX error: {}", err);
                self.sys_data.stats.controller_rx_errors += 1
            }
        }

        // Receiving new state from the ground control
        match self.air_scheduler.recv_state(&mut self.sys_data) {
            Ok(updated) => match updated {
                true => self.sys_data.stats.ground_rx_updates += 1, // New state was received
                false => self.sys_data.stats.ground_rx_idles += 1,  // No state was received
            },
            Err(err) => {
                error!("[ICAROSController] ground scheduler RX error: {}", err);
                self.sys_data.stats.ground_rx_errors += 1
            }
        }

        // Route incoming application-specific packets from the ground control
        // to the external service broker
        match self.air_scheduler.recv_app_packets() {
            Ok(pkts) => {
                // Get a hold of the incoming packets queue and append the packets we received to it
                let mut app_rx_queue = self.esb.app_messages_rx().lock();
                app_rx_queue.append(pkts);
            },
            Err(err) => {
                error!("[ICAROSController] error while reading application packets: {}", err)
            }
        }

        // If we have a GPS service avaiable...
        if let Some(gps) = &mut self.gps_device {
            // Update the current positional data from the GPS service, and copy it to our own
            // state if there's new data
            match gps.update() {
                Ok(updated) => if updated { gps.write_state(&mut self.sys_data) },
                Err(err) => {
                    error!("[ICAROSController] gps RX error: {}", err);
                }
            }
        }

        // If the flight controller device driver tells us we're clear to send more data...
        if self.controller_scheduler.can_send() {
            // Send new state to the flight controller
            match self.controller_scheduler.send_state(&self.sys_data) {
                Ok(sent) => match sent {
                    true => self.sys_data.stats.controller_tx += 1,         // Data sent successfully
                    false => self.sys_data.stats.controller_tx_errors += 1, // Data NOT sent successfully
                },
                Err(err) => {
                    // Error while sending the data
                    error!("[ICAROSController] controller scheduler TX error: {}", err);
                    self.sys_data.stats.controller_tx_errors += 1;
                }
            }
        }

        // If the ground control device driver tells us we're clear to send more data...
        if self.air_scheduler.can_send() {
            { // Lock scope begin
                // Get a hold of the outgoing application-specific messages queue
                let mut app_tx_queue = self.esb.app_messages_tx().lock();

                // Take each packet in the queue and send it to the ground control
                while !app_tx_queue.is_empty() {
                    match self.air_scheduler.send_app_packet(app_tx_queue.pop_front().unwrap()) {
                        Ok(_) => {},
                        Err(err) => {
                            error!("[ICAROSController] application scheduler TX error: {}", err);
                        }
                    }
                }
            } // Lock scope end

            // Finally, send our own state to the ground control
            match self.air_scheduler.send_state(&self.sys_data) {
                Ok(sent) => match sent {
                    true => self.sys_data.stats.ground_tx += 1,
                    false => self.sys_data.stats.ground_tx_errors += 1,
                },
                Err(err) => {
                    error!("[ICAROSController] controller scheduler TX error: {}", err);
                    self.sys_data.stats.ground_tx_errors += 1;
                }
            }
        }
    }

    /// Gets the system's current statistics
    pub fn get_stats(&self) -> &SystemStatistics {
        &self.sys_data.stats
    }

    /// Resets the system's current statistics
    pub fn reset_stats(&mut self) {
        self.sys_data.stats.reset()
    }
}
