use rf24::RF24;

use std::thread;
use std::time::Duration;
use std::cmp::min;

fn main() {
    let mut radio: RF24 = RF24::from_spi_device(22, 0); // Create radio controller
    println!("BEGIN");
    radio.begin(); // Begin communications with radio
    println!("DONE BEGIN");
    radio.set_auto_ack(false); // Disable auto acknowledgement of messages (we are a scanner)

    radio.listen_start();
    radio.listen_stop(); // Reset the radio-frequency mode

    radio.print_details(); // Print radio controller state to console

    println!("Radio is {}", if radio.chip_connected() { "connected" } else { "DISCONNECTED" });

    // Main loop
    let num_reps = 100; // Number of measurements per sweep
    let num_channels = 126; // Number of radio channels to scan in each measurement
    let mut measurements = vec![0; num_channels]; // Measurement data

    loop {
        measurements.clear(); // Reset our measurements data
        measurements.resize(num_channels, 0);

        if radio.failure_detected() { // Has radio failed?
            println!("RADIO FAILURE!");
        }

        for _ in (0..num_reps).rev() { // For each measurement...
            for i in (0..num_channels).rev() { // For each channel...
                radio.set_channel(i as u8); // Set the radio channel to the actual channel
                radio.listen_start(); // Make the radio listen for a little while

                thread::sleep(Duration::from_micros(128)); // Sleep in the meantime
                radio.listen_stop(); // Make the radio stop listening

                if radio.carrier_available() { // Was there any perceptible signal on that channel?
                    measurements[i] += 1; // Yup! Add that to the measurements.
                }
            }
        }

        // After we're done with a sweep, print our measurements.
        for i in 0..num_channels {
            print!("{:X}", min(0xF, measurements[i]));
        }

        println!(); // Print a newline
    }
}
