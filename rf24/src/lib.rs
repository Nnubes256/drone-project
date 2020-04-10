//! Bindings for TMRh20's Optimized Fork of NRF24L01+ Driver

extern crate rf24_sys;
#[macro_use] extern crate err_derive;

use rf24_sys::root as rf24_binding;

use std::ops::Fn;
use std::ffi::c_void;
use num_enum::TryFromPrimitive;


///
/// SPI bus speeds
#[derive(Copy, Clone)]
#[repr(u32)]
pub enum SPISpeed {
    SPI64MHz = 4,
    SPI32MHz = 8,
    SPI16MHz = 16,
    SPI8MHz = 32,
    SPI4MHz = 64,
    SPI2MHz = 128,
    SPI1MHz = 256,
    SPI512KHz = 512,
    SPI256KHz = 1024,
    SPI128KHz = 2048,
    SPI64KHz = 4096,
    SPI32KHz = 8192,
    SPI16KHz = 16384,
    SPI8KHz = 32768
}

///
/// Level of transmission power amplification
/// (only supported on nRF24L01+PA+LNA models)
#[derive(Copy, Clone, TryFromPrimitive)]
#[repr(u8)]
pub enum PowerAmplifierLevel {
    Minimum = 0,
    Low = 1,
    High = 2,
    Max = 3,
    Error = 4
}

///
/// Transmission rate of data sent/received through the radio
#[derive(Copy, Clone, TryFromPrimitive)]
#[repr(u32)]
pub enum DataRate {
    Radio1Mbps = rf24_binding::rf24_datarate_e_RF24_1MBPS,
    Radio2Mbps = rf24_binding::rf24_datarate_e_RF24_2MBPS,
    Radio250Kbps = rf24_binding::rf24_datarate_e_RF24_250KBPS
}

///
/// Lenght of CRC checksum to be automatically added into packets
#[derive(Copy, Clone, TryFromPrimitive)]
#[repr(u32)]
pub enum CRCLength {
    Disabled = rf24_binding::rf24_crclength_e_RF24_CRC_DISABLED,
    CRC8 = rf24_binding::rf24_crclength_e_RF24_CRC_8,
    CRC16 = rf24_binding::rf24_crclength_e_RF24_CRC_16
}

///
/// Runtime errors defined by the library
#[derive(Copy, Clone, Debug, Error)]
pub enum RF24Error {
    /// Passed buffer is too large
    #[error(display = "buffer is too large")]
    BufferTooLarge,

    /// Passed buffer is too small
    #[error(display = "buffer is too small")]
    BufferTooSmall,

    /// Device address exceeds acceptable size
    #[error(display = "address length is too large")]
    AddressLengthTooLarge,

    /// Device address is too small
    #[error(display = "address length is too small")]
    AddressLengthTooSmall,

    /// Operation that requires dynamic payloads enabled
    /// can't be completed because the former aren't enabled
    #[error(display = "dynamic payloads are not enabled")]
    DynamicPayloadsNotEnabled,

    /// Error within the underlying nRF24 library
    #[error(display = "library error")]
    LibraryError
}

///
/// Data corresponding to an interrupt (IRQ pin driven low)
pub struct InterruptData {
    /// Packet sending was successful (TX_DS register)
    pub tx_ok: bool,

    /// Packet sending failed due to maximum attempts exceeded (MAX_RT register)
    pub tx_fail: bool,

    /// A message is waiting to be read (RX_DS register)
    pub rx_ready: bool
}

impl InterruptData {
    /// Creates an interrupt data structure from its components
    pub fn from_data(tx_ok: bool, tx_fail: bool, rx_ready: bool) -> Self {
        InterruptData {
            tx_ok: tx_ok,
            tx_fail: tx_fail,
            rx_ready: rx_ready
        }
    }
}

///
/// An nRF24 device
pub struct RF24 {
    handle: rf24_binding::RF24,
}

impl RF24 {
    /// Create a nRF24 device interface from the pin numbers where the *Chip Enable*
    /// and *Chip Select Not* pins are connected.
    pub fn from_pins(ce: u16, csn: u16) -> Self {
        unsafe {
            RF24 {
                handle: rf24_binding::RF24::new(ce, csn)
            }
        }
    }

    /// Create a nRF24 device interface from the pin number where the *Chip Enable*
    /// is connected and its SPI device index.
    pub fn from_spi_device(ce: u16, spi_device: u16) -> Self {
        unsafe {
            RF24 {
                handle: rf24_binding::RF24::new(ce, spi_device)
            }
        }
    }

    /// Create a nRF24 device interface from the pin numbers where the *Chip Enable*
    /// and *Chip Select Not* pins are connected and the bus speed to use.
    pub fn from_pins_with_speed(ce: u16, csn: u16, spi_speed: SPISpeed) -> Self {
        unsafe {
            RF24 {
                handle: rf24_binding::RF24::new1(ce, csn, spi_speed as u32)
            }
        }
    }

    /// Create a nRF24 device interface from the pin number where the *Chip Enable*
    /// is connected, its SPI device index and the bus speed to use.
    pub fn from_spi_device_with_speed(ce: u16, spi_device: u16, spi_speed: SPISpeed) -> Self {
        unsafe {
            RF24 {
                handle: rf24_binding::RF24::new1(ce, spi_device, spi_speed as u32)
            }
        }
    }

    /// Begin operating with the chip.
    ///
    /// This method has to be called before any other tasks can be done.
    pub fn begin(&mut self) -> bool {
        unsafe {
            self.handle.begin()
        }
    }

    /// Returns whether a nRF24 has been detected.
    pub fn chip_connected(&mut self) -> bool {
        unsafe {
            self.handle.isChipConnected()
        }
    }

    /// Returns whether a hardware failure has occurred. If it did, the driver will cease its
    /// operation by default.
    pub fn failure_detected(&mut self) -> bool {
        self.handle.failureDetected
    }

    /// Signals the radio to start listening for packets.
    pub fn listen_start(&mut self) {
        unsafe {
            self.handle.startListening();
        }
    }

    /// Signals the radio to stop listening for packets, allowing the radio itself
    /// to send packets.
    pub fn listen_stop(&mut self) {
        unsafe {
            self.handle.stopListening();
        }
    }

    /// Returns whether there are packets available to be read.
    pub fn available(&mut self) -> bool {
        unsafe {
            self.handle.available()
        }
    }

    /// Returns whether the hardware is an nRF24L01+ (with power amplifier) or not.
    pub fn is_p_variant(&mut self) -> bool {
        unsafe {
            self.handle.isPVariant()
        }
    }

    /// Returns the current power amplifier level
    pub fn get_power_amplifier_level(&mut self) -> Result<PowerAmplifierLevel, RF24Error> {
        unsafe {
            match std::convert::TryFrom::try_from(self.handle.getPALevel()) {
                Ok(v) => Ok(v),
                Err(_) => Err(RF24Error::LibraryError)
            }
        }
    }

    /// Sets the current power amplifier level
    pub fn set_power_amplifier_level(&mut self, pa_level: PowerAmplifierLevel) {
        unsafe {
            self.handle.setPALevel(pa_level as u8);
        }
    }

    /// Returns the device's currently configured transmission rate
    pub fn get_data_rate(&mut self) -> Result<DataRate, RF24Error> {
        unsafe {
            match std::convert::TryFrom::try_from(self.handle.getDataRate()) {
                Ok(v) => Ok(v),
                Err(_) => Err(RF24Error::LibraryError)
            }
        }
    }

    /// Sets the device's transmission rate
    pub fn set_data_rate(&mut self, data_rate: DataRate) -> bool {
        unsafe {
            self.handle.setDataRate(data_rate as rf24_binding::rf24_datarate_e)
        }
    }

    /// Returns the device's currently configured message CRC length
    pub fn get_crc_length(&mut self) -> Result<CRCLength, RF24Error> {
        unsafe {
            match std::convert::TryFrom::try_from(self.handle.getCRCLength()) {
                Ok(v) => Ok(v),
                Err(_) => Err(RF24Error::LibraryError)
            }
        }
    }

    /// Sets the device's currently configured message CRC length
    pub fn set_crc_length(&mut self, crc_length: CRCLength) {
        unsafe {
            self.handle.setCRCLength(crc_length as rf24_binding::rf24_crclength_e);
        }
    }

    /// Disables CRC validation
    pub fn disable_crc(&mut self) {
        unsafe {
            self.handle.disableCRC();
        }
    }

    /// Sets the device address width used by the device
    pub fn set_address_width(&mut self, addr_width: u8) {
        unsafe {
            self.handle.setAddressWidth(addr_width);
        }
    }

    /// Sets the number of message retransmissions that may ocurr at most if
    /// acknowledgements are not received and the delay between them.
    pub fn set_retries(&mut self, delay: u8, count: u8) {
        unsafe {
            self.handle.setRetries(delay, count);
        }
    }

    /// Sets the frequency channel where the device will listen and send packets to.
    ///
    /// This channel is represented by a number between 1 and 125 (both inclusive),
    /// channel 1 being on a radio frequency of 2400 MHz and channel 125 being on 2525 MHz,
    /// with 1 MHz of spacing between frequencies.
    pub fn set_channel(&mut self, channel: u8) {
        unsafe {
            self.handle.setChannel(channel);
        }
    }

    /// Returns the frequency channel the device is currently on.
    ///
    /// This channel is represented by a number between 1 and 125 (both inclusive),
    /// channel 1 being on a radio frequency of 2400 MHz and channel 125 being on 2525 MHz,
    /// with 1 MHz of spacing between frequencies.
    pub fn get_channel(&mut self) -> u8 {
        unsafe {
            self.handle.getChannel()
        }
    }

    /// Signals the device to enter low power mode, a deep sleep state where the device needs
    /// to be woken back up before resuming operations.
    pub fn power_down(&mut self) {
        unsafe {
            self.handle.powerDown();
        }
    }

    /// Signals the device to leave low power mode and return back to normal operation.
    pub fn power_up(&mut self) {
        unsafe {
            self.handle.powerUp();
        }
    }

    /// Reads the contents of the next packet onto the passed buffer.
    ///
    /// The buffer must be already zero-filled with the requested message length.
    pub fn read(&mut self, buffer: &mut [u8]) -> Result<(), RF24Error> {
        // Check if the buffer is too large
        if buffer.len() > self.handle.payload_size as usize {
            return Err(RF24Error::BufferTooLarge);
        }

        unsafe {
            self.handle.read(buffer.as_mut_ptr() as *mut _ as *mut c_void, buffer.len() as u8);
        }

        Ok(())
    }

    /// Reads the given amount of bytes off the contents of the next packet onto the passed buffer.
    ///
    /// The buffer must be already zero-filled with the requested message length.
    /// **WARNING:** Using the incorrect message length may overflow the buffer and break
    /// memory safety, hence why this method is marked `unsafe`.
    pub unsafe fn read_bytes(&mut self, buffer: &mut [u8], num_bytes: u8) -> Result<(), RF24Error> {
        // Check if the buffer is too large
        if buffer.len() > self.handle.payload_size as usize {
            return Err(RF24Error::BufferTooLarge);
        }

        // Just in case, check we have enough space on the buffer to fill the given amount of bytes
        if buffer.len() < num_bytes as usize {
            return Err(RF24Error::BufferTooSmall);
        }

        self.handle.read(buffer.as_mut_ptr() as *mut _ as *mut c_void, num_bytes);

        Ok(())
    }

    /// Reads off the contents of the next dynamically-sized packet onto the passed vector.
    ///
    /// The vector's contents are cleared and the vector is resized properly within the method.
    pub fn read_dynamic_payload(&mut self, buffer: &mut Vec<u8>) -> Result<bool, RF24Error> {
        match self.get_dynamic_payload_size() {
            None => Ok(false), // We don't have a dynamic payload, actually
            Some(size) => { // We have a dynamic payload, and its size
                // We clear off our output vector
                buffer.clear();

                // And fill it to the expected packet size
                buffer.resize(size as usize, 0);

                // Then read the expected amount of bytes into the vector
                unsafe {
                    return self.read_bytes(buffer, size).map(|_| true);
                }
            }
        }
    }

    /// Transmits the buffer's contents as a packet, blocking until an acknowledgement has been
    /// received for such packet (returning `true`) or the retries limits have been reached
    /// (returning `false`).
    pub fn write(&mut self, buffer: &[u8]) -> Result<bool, RF24Error> {
        // Check the buffer's length doesn't exceed the payload size
        if buffer.len() > self.handle.payload_size as usize {
            return Err(RF24Error::BufferTooLarge);
        }

        // If dynamic payloads are not enabled, the buffer size must be equal to the payload size
        if !self.handle.dynamic_payloads_enabled && buffer.len() as u8 != self.handle.payload_size {
            return Err(RF24Error::DynamicPayloadsNotEnabled);
        }

        // Send the packet
        unsafe {
            let return_type = self.handle.write(buffer.as_ptr() as *const _ as *const c_void, buffer.len() as u8);
            Ok(return_type)
        }
    }

    /// Transmits the buffer's contents as a packet while signaling that no acknowledgement
    /// is required to be sent by the receiver.
    ///
    /// [RF24::enable_dynamic_ack](crate::RF24::enable_dynamic_ack) must have been
    /// called once first.
    pub fn write_unreliable(&mut self, buffer: &[u8]) -> Result<bool, RF24Error> {
        // Check the buffer's length doesn't exceed the payload size
        if buffer.len() > self.handle.payload_size as usize {
            return Err(RF24Error::BufferTooLarge);
        }

        // If dynamic payloads are not enabled, the buffer size must be equal to the payload size
        if !self.handle.dynamic_payloads_enabled && buffer.len() as u8 != self.handle.payload_size {
            return Err(RF24Error::DynamicPayloadsNotEnabled);
        }

        // Send the packet
        unsafe {
            let return_type = self.handle.write1(buffer.as_ptr() as *const _ as *const c_void, buffer.len() as u8, true);
            Ok(return_type)
        }
    }

    /// Transmits the buffer's contents as a packet, not blocking until all the device's
    /// internal packet buffers are full.
    pub fn write_fast(&mut self, buffer: &[u8]) -> Result<bool, RF24Error> {
        // Check the buffer's length doesn't exceed the payload size
        if buffer.len() > self.handle.payload_size as usize {
            return Err(RF24Error::BufferTooLarge);
        }

        // If dynamic payloads are not enabled, the buffer size must be equal to the payload size
        if !self.handle.dynamic_payloads_enabled && buffer.len() as u8 != self.handle.payload_size {
            return Err(RF24Error::DynamicPayloadsNotEnabled);
        }

        // Send the packet
        unsafe {
            let return_type = self.handle.writeFast(buffer.as_ptr() as *const _ as *const c_void, buffer.len() as u8);
            Ok(return_type)
        }
    }

    /// Transmits the buffer's contents as a packet, not blocking until all the device's
    /// internal packet buffers are full, and also disabling acknowledgements for
    /// the given packet.
    ///
    /// [RF24::enable_dynamic_ack](crate::RF24::enable_dynamic_ack) must have been
    /// called once first.
    pub fn write_fast_unreliable(&mut self, buffer: &[u8]) -> Result<bool, RF24Error> {
        // Check the buffer's length doesn't exceed the payload size
        if buffer.len() > self.handle.payload_size as usize {
            return Err(RF24Error::BufferTooLarge);
        }

        // If dynamic payloads are not enabled, the buffer size must be equal to the payload size
        if !self.handle.dynamic_payloads_enabled && buffer.len() as u8 != self.handle.payload_size {
            return Err(RF24Error::DynamicPayloadsNotEnabled);
        }

        // Send the packet
        unsafe {
            let return_type = self.handle.writeFast1(
                buffer.as_ptr() as *const _ as *const c_void, buffer.len() as u8, true
            );
            Ok(return_type)
        }
    }

    /// Transmits the buffer's contents as a packet, not blocking until all the device's
    /// internal packet buffers are full. When it blocks, it will do so until a payload
    /// has been successfully written or the specified timeout period (in milliseconds) is reached.
    pub fn write_timeout(&mut self, buffer: &[u8], timeout: u32) -> Result<bool, RF24Error> {
        // Check the buffer's length doesn't exceed the payload size
        if buffer.len() > self.handle.payload_size as usize {
            return Err(RF24Error::BufferTooLarge);
        }

        // If dynamic payloads are not enabled, the buffer size must be equal to the payload size
        if !self.handle.dynamic_payloads_enabled && buffer.len() as u8 != self.handle.payload_size {
            return Err(RF24Error::DynamicPayloadsNotEnabled);
        }

        // Send the packet
        unsafe {
            let return_type = self.handle.writeBlocking(
                buffer.as_ptr() as *const _ as *const c_void, buffer.len() as u8, timeout
            );
            Ok(return_type)
        }
    }

    /// Toggle whether dynamically-sized payload support is enabled or not.
    ///
    /// You **must** call this function with a value of `true` before sending or receiving
    /// dynamic payloads.
    pub fn set_dynamic_payloads(&mut self, enable: bool) {
        if enable {
            unsafe {
                self.handle.enableDynamicPayloads();
            }
        } else {
            unsafe {
                self.handle.disableDynamicPayloads();
            }
        }
    }

    /// Taking a closure, executes its code, then calls [RF24::tx_standby](crate::RF24::tx_standby)
    /// after the code has executed. Intented to be used to execute multiple fast writes
    /// at once safely and by specification.
    pub fn do_fast_write_transaction<T>(&mut self, transaction: T)
        where T: Fn() -> ()
    {
        transaction();
        self.tx_standby();
    }

    /// Drop the radio from STANDBY-II mode to STANDBY-I mode, blocking until all packets
    /// are sent.
    pub fn tx_standby(&mut self) {
        unsafe {
            self.handle.txStandBy();
        }
    }

    /// Drop the radio from STANDBY-II mode to STANDBY-I mode, blocking until all packets
    /// are sent or the timeout (in miliseconds) is reached. If `start_tx` is `true`,
    /// the driver will also exit out of listening mode so the packets can be sent.
    pub fn tx_standby_timeout(&mut self, timeout: u32, start_tx: bool) -> bool {
        unsafe {
            self.handle.txStandBy1(timeout, start_tx)
        }
    }

    /// Open a pipe for writing packets out as the specified radio address.
    pub fn open_writing_pipe(&mut self, address: &[u8]) -> Result<(), RF24Error> {
        // Check that the address width is within configured bounds
        if address.len() > self.handle.addr_width as usize {
            return Err(RF24Error::BufferTooLarge);
        }

        if address.len() < self.handle.addr_width as usize {
            return Err(RF24Error::BufferTooSmall);
        }

        // Open the writing pipe
        unsafe {
            self.handle.openWritingPipe(address.as_ptr());
            Ok(())
        }
    }

    /// Open a pipe for reading packets in, listening for packets with the specified
    /// address.
    ///
    /// Up to 6 pipes, identified by their pipe numbers, can be opened for simultaneous
    /// reading before starting to listen.
    pub fn open_reading_pipe(&mut self, pipe_number: u8, address: &[u8]) -> Result<(), RF24Error> {
        // Check that the address width is within configured bounds
        if address.len() > self.handle.addr_width as usize {
            return Err(RF24Error::BufferTooLarge);
        }

        if address.len() < self.handle.addr_width as usize {
            return Err(RF24Error::BufferTooSmall);
        }

        // Open the specified reading pipe
        unsafe {
            self.handle.openReadingPipe(pipe_number, address.as_ptr());
            Ok(())
        }
    }

    /// Closes the specified reading pipe
    pub fn close_reading_pipe(&mut self, pipe_number: u8) {
        unsafe {
            self.handle.closeReadingPipe(pipe_number);
        }
    }

    /// Prints detailed debugging information about the driver and the hardware state
    /// to the standard output
    pub fn print_details(&mut self) {
        unsafe {
            self.handle.printDetails();
        }
    }

    /// Returns whether there are packets available to be read, and if so,
    /// on which reading pipe they are.
    pub fn pipe_available(&mut self) -> Option<u8> {
        let mut pipe: u8 = 0;
        let payload_available;

        unsafe {
            payload_available = self.handle.available1(&mut pipe);
        }

        if payload_available {
            return Some(pipe);
        } else {
            return None;
        }
    }

    /// Returns whether the reception queues on the device are full, meaning read
    /// request should be dispatched ASAP, as otherwise the device may lose packets.
    pub fn is_rx_full(&mut self) -> bool {
        unsafe {
            self.handle.rxFifoFull()
        }
    }

    /// Empty the transmission buffer.
    pub fn flush_tx(&mut self) -> u8 {
        unsafe {
            self.handle.flush_tx()
        }
    }

    /// Empty the reception buffer.
    pub fn flush_rx(&mut self) -> u8 {
        unsafe {
            self.handle.flush_rx()
        }
    }

    /// Signal the chip to indefinitely resend the currently-available payload
    /// in the transmission buffers until another payload is written or
    /// [RF24::flush_tx](crate::RF24::flush_tx) is called.
    pub fn reuse_tx(&mut self) {
        unsafe {
            self.handle.reUseTX();
        }
    }

    /// Toggle automatic packet acknowledgement.
    pub fn set_auto_ack(&mut self, enable: bool) {
        unsafe {
            self.handle.setAutoAck(enable);
        }
    }

    /// Toggle automatic packet acknowledgement on the specified reading pipe.
    pub fn set_auto_ack_pipe(&mut self, pipe: u8, enable: bool) {
        unsafe {
            self.handle.setAutoAck1(pipe, enable);
        }
    }

    /// Returns whether there is an acknowledgement packet payload available for reading.
    pub fn is_ack_payload_available(&mut self) -> bool {
        unsafe {
            self.handle.isAckPayloadAvailable()
        }
    }

    /// Enable writing and reading of custom payloads on acknowledgement packets.
    ///
    /// They are dynamic payloads by nature, and thus they must be enabled.
    /// ACK payloads may work better on reading pipe 1 instead of 0.
    pub fn enable_ack_payload(&mut self) {
        unsafe {
            self.handle.enableAckPayload();
        }
    }

    /// Write a custom payload for the next acknowledgement packet sent on the
    /// specified reading pipe, so that the next time a packet on this pipe,
    /// the acknowledgement sent for it will contain the custom payload.
    ///
    /// Only three custom acknowledgement payloads can be pending at any time, and
    /// dynamic payloads must be enabled before sending such a payload.
    pub fn write_ack_payload(&mut self, pipe_number: u8, buffer: &[u8]) -> Result<(), RF24Error> {
        // Checks dynamic payloads are enabled before continuing
        if !self.handle.dynamic_payloads_enabled {
            return Err(RF24Error::DynamicPayloadsNotEnabled);
        }

        unsafe {
            self.handle.writeAckPayload(pipe_number, buffer.as_ptr() as *const _ as *const c_void, buffer.len() as u8);
        }

        Ok(())
    }

    /// Enable single-write basis acknowledgement adjustements.
    pub fn enable_dynamic_ack(&mut self) {
        unsafe {
            self.handle.enableDynamicAck();
        }
    }

    /// Returns the currently set static payload size
    pub fn get_payload_size(&mut self) -> u8 {
        unsafe {
            self.handle.getPayloadSize()
        }
    }

    /// Sets the currently set static payload size
    pub fn set_payload_size(&mut self, size: u8) -> Result<(), RF24Error> {
        // Check that the chosen payload size is within reasonable bounds
        if size > 32 {
            return Err(RF24Error::BufferTooLarge);
        }

        if size < 3 {
            return Err(RF24Error::BufferTooSmall);
        }

        // Set the payload size
        unsafe {
            self.handle.setPayloadSize(size);
            Ok(())
        }
    }

    /// Get the size in bytes of the next dynamic payload. If `None`, it was a corrupt payload,
    /// and it has been automatically flushed.
    pub fn get_dynamic_payload_size(&mut self) -> Option<u8> {
        let size;

        unsafe {
            size = self.handle.getDynamicPayloadSize();
        }

        if size < 1 {
            return None;
        } else {
            return Some(size);
        }
    }

    /// When an interrupt is received, this function can be called to identify the cause
    /// of the interrupt.
    pub fn inspect_interrupt(&mut self) -> InterruptData {
        let mut tx_ok = false;
        let mut tx_fail = false;
        let mut rx_ready = false;

        unsafe {
            self.handle.whatHappened(&mut tx_ok, &mut tx_fail, &mut rx_ready);

            InterruptData::from_data(tx_ok, tx_fail, rx_ready)
        }
    }

    /// Masks the given interrupt triggers set to `true` from generating interrupts.
    pub fn mask_interrupts(&mut self, mask: InterruptData) {
        unsafe {
            self.handle.maskIRQ(mask.tx_ok, mask.tx_fail, mask.rx_ready);
        }
    }

    /// Returns whether there was a carrier on the current frequency channel for the previous
    /// listening period.
    ///
    /// Can be used to check for interference.
    pub fn carrier_available(&mut self) -> bool {
        unsafe {
            self.handle.testCarrier()
        }
    }

    /// Returns whether there was a signal greater than -64dBm on the current
    /// frequency channel for the previous listening period. Only supported on nRF24L01**P**(+) chips.
    ///
    /// Can be used to check for interference and to aid channel hopping.
    pub fn test_rpd(&mut self) -> bool {
        unsafe {
            self.handle.testRPD()
        }
    }
}
