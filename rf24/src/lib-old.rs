extern crate rf24_sys;
#[macro_use] extern crate err_derive;

use rf24_sys::root as rf24_binding;

use std::ops::Fn;
use std::ffi::c_void;

#[repr(u32)]
#[derive(Copy, Clone)]
pub enum SPISpeed {
    SPI64MHZ = 4,
    SPI32MHZ = 8,
    SPI16MHZ = 16,
    SPI8MHZ = 32,
    SPI4MHZ = 64,
    SPI2MHZ = 128,
    SPI1MHZ = 256,
    SPI512KHZ = 512,
    SPI256KHZ = 1024,
    SPI128KHZ = 2048,
    SPI64KHZ = 4096,
    SPI32KHZ = 8192,
    SPI16KHZ = 16384,
    SPI8KHZ = 32768
}

#[derive(Copy, Clone, Debug, Error)]
pub enum RF24Error {
    #[error(display = "buffer is too large")]
    BufferTooLarge,
    #[error(display = "dynamic payloads are not enabled")]
    DynamicPayloadsNotEnabled
}

pub struct InterruptData {
    pub tx_ok: bool,
    pub tx_fail: bool,
    pub rx_ready: bool
}

impl InterruptData {
    pub fn from_data(tx_ok: bool, tx_fail: bool, rx_ready: bool) -> Self {
        InterruptData {
            tx_ok: tx_ok,
            tx_fail: tx_fail,
            rx_ready: rx_ready
        }
    }
}

pub struct RF24 {
    handle: rf24_binding::RF24
}

impl RF24 {
    pub fn from_pins(ce: u16, csn: u16) -> Self {
        unsafe {
            RF24 {
                handle: rf24_binding::RF24::new(ce, csn)
            }
        }
    }
    pub fn from_spi_device(ce: u16, spi_device: u16) -> Self {
        unsafe {
            RF24 {
                handle: rf24_binding::RF24::new(ce, spi_device)
            }
        }
    }
    pub fn from_pins_with_speed(ce: u16, csn: u16, spi_speed: SPISpeed) -> Self {
        unsafe {
            RF24 {
                handle: rf24_binding::RF24::new1(ce, csn, spi_speed as u32)
            }
        }
    }
    pub fn from_spi_device_with_speed(ce: u16, spi_device: u16, spi_speed: SPISpeed) -> Self {
        unsafe {
            RF24 {
                handle: rf24_binding::RF24::new1(ce, spi_device, spi_speed as u32)
            }
        }
    }

    pub fn begin(&mut self) -> bool {
        unsafe {
            self.handle.begin()
        }
    }

    pub fn chip_connected(&mut self) -> bool {
        unsafe {
            self.handle.isChipConnected()
        }
    }

    pub fn listen_start(&mut self) {
        unsafe {
            self.handle.startListening();
        }
    }

    pub fn listen_stop(&mut self) {
        unsafe {
            self.handle.stopListening();
        }
    }

    pub fn available(&mut self) -> bool {
        unsafe {
            self.handle.available()
        }
    }

    pub fn power_up(&mut self) {
        unsafe {
            self.handle.powerUp();
        }
    }

    pub fn power_down(&mut self) {
        unsafe {
            self.handle.powerDown();
        }
    }

    pub fn read(&mut self, buffer: &mut [u8]) -> Result<(), RF24Error> {
        if buffer.len() > 32 {
            return Err(RF24Error::BufferTooLarge);
        }

        unsafe {
            self.handle.read(buffer.as_mut_ptr() as *mut _ as *mut c_void, buffer.len() as u8);
        }

        Ok(())
    }

    pub fn write(&mut self, buffer: &[u8]) -> Result<bool, RF24Error> {
        if buffer.len() > 32 {
            return Err(RF24Error::BufferTooLarge);
        }
        
        unsafe {

            if !self.handle.dynamic_payloads_enabled && buffer.len() as u8 != self.handle.payload_size {
                return Err(RF24Error::DynamicPayloadsNotEnabled);
            }

            let return_type = self.handle.write(buffer.as_ptr() as *const _ as *const c_void, buffer.len() as u8);
            Ok(return_type)
        }
    }

    pub fn write_unreliable(&mut self, buffer: &[u8]) -> Result<bool, RF24Error> {
        if buffer.len() > 32 {
            return Err(RF24Error::BufferTooLarge);
        }
        
        unsafe {
            if !self.handle.dynamic_payloads_enabled && buffer.len() as u8 != self.handle.payload_size {
                return Err(RF24Error::DynamicPayloadsNotEnabled);
            }

            let return_type = self.handle.write1(buffer.as_ptr() as *const _ as *const c_void, buffer.len() as u8, true);
            Ok(return_type)
        }
    }

    pub fn write_fast(&mut self, buffer: &[u8]) -> Result<bool, RF24Error> {
        if buffer.len() > 32 {
            return Err(RF24Error::BufferTooLarge);
        }

        unsafe {
            if !self.handle.dynamic_payloads_enabled && buffer.len() as u8 != self.handle.payload_size {
                return Err(RF24Error::DynamicPayloadsNotEnabled);
            }

            let return_type = self.handle.writeFast(buffer.as_ptr() as *const _ as *const c_void, buffer.len() as u8);
            Ok(return_type)
        }
    }

    pub fn write_fast_unreliable(&mut self, buffer: &[u8]) -> Result<bool, RF24Error> {
        if buffer.len() > 32 {
            return Err(RF24Error::BufferTooLarge);
        }

        unsafe {

            if !self.handle.dynamic_payloads_enabled && buffer.len() as u8 != self.handle.payload_size {
                return Err(RF24Error::DynamicPayloadsNotEnabled);
            }

            let return_type = self.handle.writeFast1(buffer.as_ptr() as *const _ as *const c_void, buffer.len() as u8, true);
            Ok(return_type)
        }
    }

    pub fn write_timeout(&mut self, buffer: &[u8], timeout: u32) -> Result<bool, RF24Error> {
        if buffer.len() > 32 {
            return Err(RF24Error::BufferTooLarge);
        }

        unsafe {
            
            if !self.handle.dynamic_payloads_enabled && buffer.len() as u8 != self.handle.payload_size {
                return Err(RF24Error::DynamicPayloadsNotEnabled);
            }

            let return_type = self.handle.writeBlocking(buffer.as_ptr() as *const _ as *const c_void, buffer.len() as u8, timeout);
            Ok(return_type)
        }
    }

    pub fn do_fast_write<T>(&mut self, transaction: T)
        where T: Fn() -> () 
    {
        transaction();
        self.tx_standby();
    }

    pub fn tx_standby(&mut self) {
        unsafe {
            self.handle.txStandBy();
        }
    }

    pub fn tx_standby_timeout(&mut self, timeout: u32, start_tx: bool) -> bool {
        unsafe {
            self.handle.txStandBy1(timeout, start_tx)
        }
    }

    pub fn open_writing_pipe(&mut self, address: u8) {
        unsafe {
            self.handle.openWritingPipe(&address as *const u8);
        }
    }

    pub fn open_reading_pipe(&mut self, pipe_number: u8, address: u8) {
        unsafe {
            self.handle.openReadingPipe(pipe_number, &address as *const u8);
        }
    }

    pub fn print_details(&mut self) {
        unsafe {
            self.handle.printDetails();
        }
    }

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

    pub fn is_rx_full(&mut self) -> bool {
        unsafe {
            self.handle.rxFifoFull()
        }
    }

    pub fn is_ack_payload_available(&mut self) -> bool {
        unsafe {
            self.handle.isAckPayloadAvailable()
        }
    }

    pub fn inspect_interrupt(&mut self) -> InterruptData {
        let mut tx_ok = false;
        let mut tx_fail = false;
        let mut rx_ready = false;

        unsafe {
            self.handle.whatHappened(&mut tx_ok, &mut tx_fail, &mut rx_ready);

            InterruptData::from_data(tx_ok, tx_fail, rx_ready)
        }
    }

    pub fn mask_interrupts(&mut self, mask: InterruptData) {
        unsafe {
            self.handle.maskIRQ(mask.tx_ok, mask.tx_fail, mask.rx_ready);
        }
    }
}
