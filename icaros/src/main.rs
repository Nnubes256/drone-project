#![feature(proc_macro_hygiene)]
#![feature(test)]
//! ICAROS (*In-flight Communications and Reporting Operations System*), drone side.

// Enable the proc_macro_hygiene compiler feature (required by python integration)



//
// External dependencies


extern crate bincode2;
extern crate byteorder;
extern crate err_derive;
extern crate flexi_logger;
extern crate icaros_base;
extern crate serialport;
extern crate test;
#[macro_use] extern crate log;

// Dependencies only used when compiling on the Raspberry Pi
#[cfg(target = "armv7-unknown-linux-gnueabihf")]
extern crate rf24;


//
// Submodules

mod core;
mod comms;
mod hardware;
mod esb;

// ----------------------------

use crate::comms::air::udp::UDPCommunicationService;
use icaros_base::comms::air::G2AMessage;
use icaros_base::comms::air::A2GMessage;
use icaros_base::comms::air::scheduler::AirPacketScheduler;
use crate::core::SystemState;
use crate::hardware::gps::GPSService;
use crate::comms::air::udp::UDPCommunicationOptions;
use std::{
    thread, fs,
    path::Path,
    time::{Duration, Instant},
};
use hwloc::Topology;

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

// Imports only used when compiling on the Raspberry Pi

#[cfg(target = "armv7-unknown-linux-gnueabihf")]
use crate::comms::air::rf24::{RF24CommunicationOptions, RF24CommunicationService};


//
// Constants

///
/// Path to the serial port file where the flight controller will be
/// connected to. **MODIFY AS REQUIRED**.
pub const CONTROLLER_SERIAL_PORT_PATH: &str = "/dev/ttyACM0";
//pub const CONTROLLER_SERIAL_PORT_PATH: &str = "/dev/cu.usbmodem14101";

///
/// Path to the serial port file where the GPS module will be
/// connected to. **MODIFY AS REQUIRED**.
pub const GPS_SERIAL_PORT_PATH: &str = "/dev/ttyACM0";

///
/// Entry point for the program
fn main() -> Result<(), ()> {
    // Start the logging facilities. This allows to use the `info!`, `error!` and `error!`
    // macros to debug messages to the console.
    flexi_logger::Logger::with_env().start().unwrap();

    // env!("CARGO_PKG_VERSION") will be replaced with the program's version as defined
    // in the Cargo.toml file on compile time.
    info!("ICAROS {}", env!("CARGO_PKG_VERSION"));

    // UNFINISHED FEATURE -- CPU core binding support
    {
        // Get the current topology of the processor. This tells us the number of cores, what
        // cores are we assigned to currently, and so.
        let topo = Topology::new();

        // Test whether the current platform supports binding ourselves to a specific core.
        info!("CPU thread binding {} supported!",
            if topo.support().cpu().set_current_thread() { "is" } else { "is NOT" }
        )
    }

    info!("Initializing peripherals");

    //
    // Air communication driver - UDP
    //

    // Configure the driver to listen to port 3333.
    let opts = UDPCommunicationOptions::new(3333);

    // Initialize the air communication driver.
    let radio_sched: AirPacketScheduler<UDPCommunicationService, SystemState, A2GMessage, G2AMessage> =
        match AirPacketScheduler::<UDPCommunicationService, SystemState, A2GMessage, G2AMessage>::initialize(opts) {
            Ok(radio_sched) => radio_sched,
            Err(err) => {
                // We failed to initialize the driver; exit.
                info!("Error while initializing air scheduler: {}", err);
                return Err(());
            }
        };

    //
    // Air communication driver - Radio
    //

    /*
    // Configure the driver
    let opts = RF24CommunicationOptions {
        ce_pin: 22,                             // Chip Enable pin number (SPI)
        csn_pin: 0,                             // Chip Select Not pin number (SPI)
        bus_speed: SPISpeed::SPI8MHz,           // SPI bus speed
        channel: 7,                             // Radio channel to use
        data_rate: DataRate::Radio2Mbps,        // Communication data rate
        retries: 15,                            // Number of transmission retries
        retries_delay: 15,                      // Delay between retries
        rx_address: &[90, 80, 70, 60, 50],      // Our own 5-bit address (for receiving)
        tx_address: &[50, 60, 70, 80, 90],      // Destination's 5-bit address (for transmitting)
        tx_power: PowerAmplifierLevel::High     // Power amplifier's strength
    };

    // Initialize the air communication driver.
    let radio_sched: AirPacketScheduler<RF24CommunicationService, SystemState, A2GMessage, G2AMessage> =
        match AirPacketScheduler::<RF24CommunicationService, SystemState, A2GMessage, G2AMessage>::initialize(opts) {
            Ok(controller) => controller,
            Err(err) => {
                // We failed to initialize the driver; exit.
                error!("[SerialControllerCommunicationsService] Could not initialize controller: {}", err);
                return Err(());
            }
    };*/

    //
    // Controller communication driver - Simulated (generates random data)
    //

    /*let controller_sched: ControllerPacketScheduler<SimulatedControllerController> =
        match ControllerPacketScheduler::initialize(()) {
            Ok(controller) => controller,
            Err(err) => {
                error!("[SerialControllerCommunicationsService] Could not initialize controller: {}", err);
                return Err(());
            }
    };*/

    //
    // Controller communication driver - Serial port
    //

    // Initialize a Path based on the configured path string
    let path = Path::new(CONTROLLER_SERIAL_PORT_PATH);

    // Check if a file exists on that path
    if !path.exists() {
        error!("Serial controller device path does not exist, is the Arduino connected?");
        return Err(());
    }

    // If it's a symbolic link, follow it
    let serial_path = match fs::read_link(path) {
        Ok(real_path) => {
            // It is a symbolic link, poiting to `real_path`
            info!(
                "Serial controller device path resolved: {} -> {}",
                path.display(),
                real_path.display()
            );
            real_path
        }
        Err(_) => {
            // It is NOT a symbolic link
            info!(
                "Serial controller device path resolved: {}",
                path.display()
            );
            path.to_path_buf()
        }
    };

    // Configure the driver...
    let mut serial_options = serialport::SerialPortSettings::default();
    serial_options.baud_rate = 250000;                                      // Baud rate
    serial_options.data_bits = serialport::DataBits::Eight;                 // Data bits
    serial_options.parity = serialport::Parity::None;                       // Parity bits
    serial_options.stop_bits = serialport::StopBits::One;                   // Stop bits
    serial_options.flow_control = serialport::FlowControl::None;            // Serial flow control
    serial_options.timeout = Duration::from_millis(100);                    // I/O timeout threshold

    // ...with the specified path and options
    let controller_settings =
        SerialComunicationOptions::from_path_and_options(&serial_path, &serial_options);

    // Initialize the controller communication driver.
    let controller_sched: ControllerPacketScheduler<SerialControllerCommunicationsService> =
        match ControllerPacketScheduler::initialize(controller_settings) {
            Ok(controller) => controller,
            Err(err) => {
                // We failed to initialize the driver; exit.
                error!(
                    "[SerialControllerCommunicationsService] Could not initialize controller: {}",
                    err
                );
                return Err(());
            }
        };

    info!("[SerialControllerCommunicationsService] Initialized!");

    //
    // GPS driver - Serial port
    //

    // Configure the driver...
    let mut gps_serial_options = serialport::SerialPortSettings::default();
    gps_serial_options.baud_rate = 115200;                                      // Baud rate
    gps_serial_options.data_bits = serialport::DataBits::Eight;                 // Data bits
    gps_serial_options.parity = serialport::Parity::None;                       // Parity bits
    gps_serial_options.stop_bits = serialport::StopBits::One;                   // Stop bits
    gps_serial_options.flow_control = serialport::FlowControl::None;            // Serial flow control
    gps_serial_options.timeout = Duration::from_millis(1050);                   // I/O timeout threshold

    // ...with the specified path and options
    let gps_settings =
        SerialComunicationOptions::from_path_and_options(Path::new(GPS_SERIAL_PORT_PATH), &serial_options);

    // Initialize the GPS communication driver.
    let gps_service = match GPSService::new(gps_settings) {
        Ok(gps) => gps,
        Err(err) => {
            // We failed to initialize the GPS driver; exit.
            error!("Error while initializing the controller: {}", err);
            return Err(());
        }
    };

    info!("[GPSService] Initialized!");

    info!("All devices and schedulers ready");

    info!("Initializing ICAROS controller");

    // Initialize the main system
    let mut icaros_controller = match ICAROSController::init(controller_sched, radio_sched, Some(gps_service)) {
        Ok(controller) => controller,
        Err(err) => {
            // We failed to initialize the full system; exit.
            error!("Error while initializing the controller: {}", err);
            return Err(());
        }
    };

    info!("All systems go! Begin loop!");

    // Current time
    let mut last_instant = Instant::now();

    // Forever...
    loop {
        // Take the current time
        let sleep_timer = Instant::now();

        // Process one tick on the system
        // (i.e. receive data, process data, send data)
        icaros_controller.process();

        let instant = Instant::now();              // Take the current time
        let duration = last_instant.elapsed();     // Obtain the time since the last elapsed second
                                                   // (or since the loop started)

        // If it's more that one second...
        if duration.as_millis() > 1000 {
            // The current time will be our new base time
            last_instant = instant;

            // We obtain and display statistics about the system relative to the second that
            // just elapsed
            let stats = icaros_controller.get_stats();
            info!(target: "stats", "Stats | {}", stats);

            // We then inform the controller to reset those stats, in order to have fresh new
            // ones on the next statistics readout (thus, avoids them from being accumulative).
            icaros_controller.reset_stats();
        }

        // We get the elapsed time since the current loop iteration started
        let loop_time_taken = sleep_timer.elapsed();

        // if it's less than 50 microseconds, we sleep until we reach a total iteration time of
        // 50 microseconds. This helps reduce energy consumption and gives the devices some time
        // to answer our requests.
        if Duration::from_micros(50) > loop_time_taken {
            thread::sleep(Duration::from_micros(50) - loop_time_taken);
        }
    }
}
