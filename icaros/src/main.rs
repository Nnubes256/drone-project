#![feature(proc_macro_hygiene)]
#![feature(test)]
extern crate bincode2;
extern crate byteorder;
extern crate err_derive;
extern crate flexi_logger;
extern crate icaros_base;
extern crate serialport;
extern crate test;
#[macro_use]
extern crate log;
#[cfg(target = "armv7-unknown-linux-gnueabihf")]
extern crate rf24;

use crate::comms::air::udp::UDPCommunicationService;
use icaros_base::comms::air::G2AMessage;
use icaros_base::comms::air::A2GMessage;
use crate::core::SystemState;
use crate::comms::air::udp::UDPCommunicationOptions;
use std::{
    thread, fs,
    path::Path,
    time::{Duration, Instant},
};
use hwloc::Topology;

mod esb;
mod comms;
mod core;
mod hardware;

use icaros_base::comms::air::scheduler::AirPacketScheduler;

use crate::{
    comms::{
        controller::{
            scheduler::ControllerPacketScheduler,
            serial::SerialControllerCommunicationsService,
        },
    },
    hardware::serialport::SerialComunicationOptions,
    core::ICAROSController,
};

pub const CONTROLLER_SERIAL_PORT_PATH: &str = "/dev/cu.usbmodem14101";

fn main() -> Result<(), ()> {
    flexi_logger::Logger::with_env().start().unwrap();

    info!("ICAROS {}", env!("CARGO_PKG_VERSION"));

    {
        let topo = Topology::new();
        info!("CPU thread binding {} supported!",
            if topo.support().cpu().set_current_thread() { "is" } else { "is NOT" }
        )
    }

    info!("Initializing peripherals");

    let opts = UDPCommunicationOptions::new(3333);

    let radio_sched: AirPacketScheduler<UDPCommunicationService, SystemState, A2GMessage, G2AMessage> =
        match AirPacketScheduler::<UDPCommunicationService, SystemState, A2GMessage, G2AMessage>::initialize(opts) {
            Ok(radio_sched) => radio_sched,
            Err(err) => {
                info!("Error while initializing air scheduler: {}", err);
                return Err(());
            }
        };

    /*let controller_sched: ControllerPacketScheduler<SimulatedControllerController> =
        match ControllerPacketScheduler::initialize(()) {
            Ok(controller) => controller,
            Err(err) => {
                error!("[SerialControllerCommunicationsService] Could not initialize controller: {}", err);
                return Err(());
            }
    };*/

    let path = Path::new(CONTROLLER_SERIAL_PORT_PATH);

    if !path.exists() {
        error!("Serial controller device path does not exist, is the Arduino connected?");
        return Err(());
    }

    let serial_path = match fs::read_link(path) {
        Ok(real_path) => {
            info!(
                "Serial controller device path resolved: {} -> {}",
                path.display(),
                real_path.display()
            );
            real_path
        }
        Err(_) => {
            info!(
                "Serial controller device path resolved: {}",
                path.display()
            );
            path.to_path_buf()
        }
    };

    /*match fs::metadata(serial_path.clone()) {
        Ok(meta) => if !meta.file_type().is_file() {
            error!("Serial controller device path is not a file, misconfiguration?");
            return Err(());
        },
        Err(err) => {
            error!("Serial controller was unable to be accessed due to error: {}", err);
            return Err(());
        }
    };*/

    let mut serial_options = serialport::SerialPortSettings::default();
    serial_options.baud_rate = 115200;
    serial_options.data_bits = serialport::DataBits::Eight;
    serial_options.parity = serialport::Parity::None;
    serial_options.stop_bits = serialport::StopBits::One;
    serial_options.flow_control = serialport::FlowControl::None;
    serial_options.timeout = Duration::from_millis(100);

    let controller_settings =
        SerialComunicationOptions::from_path_and_options(&serial_path, &serial_options);

    let controller_sched: ControllerPacketScheduler<SerialControllerCommunicationsService> =
        match ControllerPacketScheduler::initialize(controller_settings) {
            Ok(controller) => controller,
            Err(err) => {
                error!(
                    "[SerialControllerCommunicationsService] Could not initialize controller: {}",
                    err
                );
                return Err(());
            }
        };

    info!("[SerialControllerCommunicationsService] Initialized!");
    info!("All devices ready");

    info!("Initializing schedulers");

    info!("Initializing ICAROS controller");

    let mut icaros_controller = match ICAROSController::init(controller_sched, radio_sched, None) {
        Ok(controller) => controller,
        Err(err) => {
            error!("Error while initializing the controller: {}", err);
            return Err(());
        }
    };

    info!("All systems go! Begin loop!");

    let mut last_instant = Instant::now();

    loop {
        let sleep_timer = Instant::now();
        icaros_controller.process();

        let instant = Instant::now();
        let duration = last_instant.elapsed();
        if duration.as_millis() > 1000 {
            last_instant = instant;
            let stats = icaros_controller.get_stats();
            info!(target: "stats", "Stats | {}", stats);
            icaros_controller.reset_stats();
        }

        let loop_time_taken = sleep_timer.elapsed();

        if Duration::from_micros(50) > loop_time_taken {
            thread::sleep(Duration::from_micros(50) - loop_time_taken);
        }
    }
}
