extern crate bincode2;
extern crate snafu;
extern crate serialport;
#[cfg(target = "armv7-unknown-linux-gnueabihf")]
extern crate rf24;

mod core;
mod comms;
mod utils;

fn main() {
    println!("Hello, world!");
}
