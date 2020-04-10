//! Common/shared components between [`icaros`](../icaros/index.html) and [`icaros-control`](../icaros-control/index.html).

#[macro_use] extern crate log;

pub use bincode2::Config as BincodeConfig;

pub mod core;
pub mod comms;
pub mod utils;
