#![feature(async_closure)]

//! ICAROS (*In-flight Communications and Reporting Operations System*), ground control side.

#[macro_use] extern crate log;
#[macro_use] extern crate lazy_static;

mod core;
mod web;
mod gamepad;
mod comms;

use icaros_base::comms::air::A2GMessage;
use icaros_base::comms::air::G2AMessage;
use icaros_base::comms::air::scheduler::AirPacketScheduler;
use crate::core::{ICAROSGroundController, GroundSystemState};
use crate::comms::udp::{UDPCommunicationOptions, UDPCommunicationService};
use std::thread;
use std::time::Duration;

#[cfg(target = "armv7-unknown-linux-gnueabihf")] use crate::comms::rf24::{RF24CommunicationOptions, RF24CommunicationService};
#[cfg(target = "armv7-unknown-linux-gnueabihf")] use rf24::{DataRate, SPISpeed, PowerAmplifierLevel, RF24Error};
#[cfg(target = "armv7-unknown-linux-gnueabihf")]
fn get_scheduler() -> Result<AirPacketScheduler<RF24CommunicationService, GroundSystemState, G2AMessage, A2GMessage>, RF24Error> {
    // Configure the driver
    let opts = RF24CommunicationOptions {
        ce_pin: 22,
        csn_pin: 0,
        bus_speed: SPISpeed::SPI8MHz,
        channel: 7,
        data_rate: DataRate::Radio2Mbps,
        retries: 5,
        retries_delay: 15,
        rx_address: &[50, 60, 70, 80, 90],
        tx_address: &[90, 80, 70, 60, 50],
        tx_power: PowerAmplifierLevel::High
    };

    // Initialize the air communication driver.
    let scheduler = AirPacketScheduler::<RF24CommunicationService, GroundSystemState, G2AMessage, A2GMessage>::initialize(opts);

    scheduler
}

#[cfg(not(target = "armv7-unknown-linux-gnueabihf"))]
fn get_scheduler() -> Result<AirPacketScheduler<UDPCommunicationService, GroundSystemState, G2AMessage, A2GMessage>, std::io::Error> {
    // Configure the driver to listen to port 3333.
    let opts = UDPCommunicationOptions::new(format!("127.0.0.1"), 3333);

    // Initialize the air communication driver.
    let scheduler = AirPacketScheduler::<UDPCommunicationService, GroundSystemState, G2AMessage, A2GMessage>::initialize(opts);
    scheduler
}

///
/// Entry point for the program
fn main() -> Result<(), ()> {
    // Start the logging facilities. This allows to use the `info!`, `error!` and `error!`
    // macros to debug messages to the console.
    flexi_logger::Logger::with_env().start().unwrap();

    // Initialize the uplink with the drone
    let scheduler = match get_scheduler() {
        Ok(scheduler) => scheduler,
        Err(err) => {
            info!("Error while initializing air scheduler: {}", err);
            return Err(());
        }
    };

    // Initialize the main system
    let mut controller = ICAROSGroundController::new(scheduler);
    controller.start().map_err(|_| ())?;

    // Forever...
    loop {
        // Process a single tick on the main system
        controller.process();

        // Sleep for a small while in order to reduce power consumption
        thread::sleep(Duration::from_micros(200));
    }
}
