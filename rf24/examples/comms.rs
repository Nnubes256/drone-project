use std::error::Error;
use rf24::RF24;

fn main() -> Result<(), Box<dyn Error>> {
    let mut radio: RF24 = RF24::from_spi_device(22, 0);
    println!("BEGIN");
    radio.begin();
    println!("DONE BEGIN");
    radio.set_retries(5, 15);
    radio.print_details();

    radio.open_writing_pipe(b"2Node")?;
    radio.open_reading_pipe(1, b"1Node")?;

    radio.listen_start();

    println!("Radio is {}", if radio.chip_connected() { "connected" } else { "DISCONNECTED" });

    println!("Listening started");

    loop {
        if radio.failure_detected() {
            println!("RADIO FAILURE!");
        }

        if radio.available() {
            let mut result = [0; 4];

            println!("Packet!");

            match radio.read(&mut result) {
                Ok(()) => { println!("Read OK"); },
                Err(error) => {
                    println!("Read error: {}", error);
                    continue;
                }
            };

            println!("Bytes left stale: {}", radio.available());

            let number = u32::from_le_bytes(result);

            println!("Received: {}", number);

            radio.listen_stop();

            let mut writeout = result.to_vec();
            writeout.append(&mut vec![0u8; 28]);

            match radio.write(&writeout) {
                Ok(res) => {println!("Write OK, ack {}", res)}
                Err(error) => {
                    println!("Write error: {}", error);
                    continue;
                }
            };

            radio.listen_start();
        }
    }
}
