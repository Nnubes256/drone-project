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

#[derive(Default)]
pub struct SystemStatistics {
    pub ground_rx_updates: u32,
    pub controller_rx_updates: u32,
    pub ground_rx_idles: u32,
    pub controller_rx_idles: u32,
    pub ground_rx_errors: u32,
    pub controller_rx_errors: u32,
    pub ground_tx: u32,
    pub controller_tx: u32,
    pub ground_tx_errors: u32,
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

pub struct SystemState {
    pub drone: DroneState,
    pub ground: GroundControlState,
    pub stats: SystemStatistics,
}

impl SystemState {
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
        if let Some(lat) = new_latlon.0 {
            self.drone.gps.latitude = Some(lat);
        } else {
            self.drone.gps.latitude = None;
        }

        if let Some(lon) = new_latlon.1 {
            self.drone.gps.longitude = Some(lon);
        } else {
            self.drone.gps.longitude = None;
        }

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

pub struct ICAROSController<A, B>
where
    A: ControllerCommunicationService,
    B: AirCommunicationService<A2GMessage, G2AMessage>,
{
    sys_data: SystemState,
    controller_scheduler: ControllerPacketScheduler<A>,
    air_scheduler: DroneAirPacketScheduler<B>,
    gps_device: Option<GPSService>,
    esb: ExternalServiceBroker,
}

impl<A, B> ICAROSController<A, B>
where
    A: ControllerCommunicationService,
    B: AirCommunicationService<A2GMessage, G2AMessage>,
{
    pub fn init(
        controller_sched: ControllerPacketScheduler<A>,
        air_sched: DroneAirPacketScheduler<B>,
        gps_device: Option<GPSService>
    ) -> Result<Self, Box<dyn Error>> {
        info!("[ICAROSController] Ready!");
        let mut esb = ExternalServiceBroker::new();
        esb.start()?;
        Ok(ICAROSController {
            sys_data: SystemState::new(),
            controller_scheduler: controller_sched,
            air_scheduler: air_sched,
            gps_device,
            esb,
        })
    }

    pub fn process(&mut self) {
        match self.controller_scheduler.recv_state(&mut self.sys_data) {
            Ok(updated) => match updated {
                true => self.sys_data.stats.controller_rx_updates += 1,
                false => self.sys_data.stats.controller_rx_idles += 1,
            },
            Err(err) => {
                error!("[ICAROSController] controller scheduler RX error: {}", err);
                self.sys_data.stats.controller_rx_errors += 1
            }
        }

        match self.air_scheduler.recv_state(&mut self.sys_data) {
            Ok(updated) => match updated {
                true => self.sys_data.stats.ground_rx_updates += 1,
                false => self.sys_data.stats.ground_rx_idles += 1,
            },
            Err(err) => {
                error!("[ICAROSController] ground scheduler RX error: {}", err);
                self.sys_data.stats.ground_rx_errors += 1
            }
        }

        match self.air_scheduler.recv_app_packets() {
            Ok(pkts) => {
                let mut app_rx_queue = self.esb.app_messages_rx().lock();
                app_rx_queue.append(pkts);
            },
            Err(err) => {
                error!("[ICAROSController] error while reading application packets: {}", err)
            }
        }

        if let Some(gps) = &mut self.gps_device {
            match gps.update() {
                Ok(updated) => if updated { gps.write_state(&mut self.sys_data) },
                Err(err) => {
                    error!("[ICAROSController] gps RX error: {}", err);
                }
            }
        }

        if self.controller_scheduler.can_send() {
            //info!("Can send controller!");
            match self.controller_scheduler.send_state(&self.sys_data) {
                Ok(sent) => match sent {
                    true => self.sys_data.stats.controller_tx += 1,
                    false => self.sys_data.stats.controller_tx_errors += 1,
                },
                Err(err) => {
                    error!("[ICAROSController] controller scheduler TX error: {}", err);
                    self.sys_data.stats.controller_tx_errors += 1;
                }
            }
        }

        if self.air_scheduler.can_send() {
            {
                let mut app_tx_queue = self.esb.app_messages_tx().lock();
                while !app_tx_queue.is_empty() {
                    match self.air_scheduler.send_app_packet(app_tx_queue.pop_front().unwrap()) {
                        Ok(_) => {},
                        Err(err) => {
                            error!("[ICAROSController] application scheduler TX error: {}", err);
                        }
                    }
                }
            }

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

    pub fn get_stats(&self) -> &SystemStatistics {
        &self.sys_data.stats
    }

    pub fn reset_stats(&mut self) {
        self.sys_data.stats.reset()
    }
}
