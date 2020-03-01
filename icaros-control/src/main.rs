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

fn main() -> Result<(), ()> {
    flexi_logger::Logger::with_env().start().unwrap();

    let opts = UDPCommunicationOptions::new(format!("127.0.0.1"), 3333);
    let scheduler: AirPacketScheduler<UDPCommunicationService, GroundSystemState, G2AMessage, A2GMessage> =
        match AirPacketScheduler::<UDPCommunicationService, GroundSystemState, G2AMessage, A2GMessage>::initialize(opts) {
            Ok(scheduler) => scheduler,
            Err(err) => {
                info!("Error while initializing air scheduler: {}", err);
                return Err(());
            }
        };

    let mut controller = ICAROSGroundController::new(scheduler);
    controller.start().map_err(|_| ())?;

    loop {
        controller.process();
    }
}
