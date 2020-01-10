extern crate rf24_sys;
#[macro_use] extern crate err_derive;

use rf24_sys::root as rf24_binding;

use std::ops::Fn;
use std::ffi::c_void;
use num_enum::TryFromPrimitive;
use std::convert::TryFrom;

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

#[derive(Copy, Clone, TryFromPrimitive)]
#[repr(u8)]
pub enum PowerAmplifierLevel {
    Minimum = 0,
    Low = 1,
    High = 2,
    Max = 3,
    Error = 4
}

#[derive(Copy, Clone, TryFromPrimitive)]
#[repr(u32)]
pub enum DataRate {
    Radio1Mbps = rf24_binding::rf24_datarate_e_RF24_1MBPS,
    Radio2Mbps = rf24_binding::rf24_datarate_e_RF24_2MBPS,
    Radio250Kbps = rf24_binding::rf24_datarate_e_RF24_250KBPS
}

#[derive(Copy, Clone, TryFromPrimitive)]
#[repr(u32)]
pub enum CRCLength {
    Disabled = rf24_binding::rf24_crclength_e_RF24_CRC_DISABLED,
    CRC8 = rf24_binding::rf24_crclength_e_RF24_CRC_8,
    CRC16 = rf24_binding::rf24_crclength_e_RF24_CRC_16
}

#[derive(Copy, Clone, Debug, Error)]
pub enum RF24Error {
    #[error(display = "buffer is too large")]
    BufferTooLarge,
    #[error(display = "buffer is too small")]
    BufferTooSmall,
    #[error(display = "address length is too large")]
    AddressLengthTooLarge,
    #[error(display = "address length is too small")]
    AddressLengthTooSmall,
    #[error(display = "dynamic payloads are not enabled")]
    DynamicPayloadsNotEnabled,
    #[error(display = "library error")]
    LibraryError
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

    pub fn failure_detected(&mut self) -> bool {
        self.handle.failureDetected
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

    pub fn is_p_variant(&mut self) -> bool {
        unsafe {
            self.handle.isPVariant()
        }
    }

    pub fn get_power_amplifier_level(&mut self) -> Result<PowerAmplifierLevel, RF24Error> {
        unsafe {
            match std::convert::TryFrom::try_from(self.handle.getPALevel()) {
                Ok(v) => Ok(v),
                Err(err) => Err(RF24Error::LibraryError)
            }
        }
    }

    pub fn set_power_amplifier_level(&mut self, pa_level: PowerAmplifierLevel) {
        unsafe {
            self.handle.setPALevel(pa_level as u8);
        }
    }

    pub fn get_data_rate(&mut self) -> Result<DataRate, RF24Error> {
        unsafe {
            match std::convert::TryFrom::try_from(self.handle.getDataRate()) {
                Ok(v) => Ok(v),
                Err(err) => Err(RF24Error::LibraryError)
            }
        }
    }

    pub fn set_data_rate(&mut self, data_rate: DataRate) -> bool {
        unsafe {
            self.handle.setDataRate(data_rate as rf24_binding::rf24_datarate_e)
        }
    }

    pub fn get_crc_length(&mut self) -> Result<CRCLength, RF24Error> {
        unsafe {
            match std::convert::TryFrom::try_from(self.handle.getCRCLength()) {
                Ok(v) => Ok(v),
                Err(err) => Err(RF24Error::LibraryError)
            }
        }
    }

    pub fn set_crc_length(&mut self, crc_length: CRCLength) {
        unsafe {
            self.handle.setCRCLength(crc_length as rf24_binding::rf24_crclength_e);
        }
    }

    pub fn disable_crc(&mut self) {
        unsafe {
            self.handle.disableCRC();
        }
    }

    pub fn set_address_width(&mut self, addr_width: u8) {
        unsafe {
            self.handle.setAddressWidth(addr_width);
        }
    }

    pub fn set_retries(&mut self, delay: u8, count: u8) {
        unsafe {
            self.handle.setRetries(delay, count);
        }
    }

    pub fn set_channel(&mut self, channel: u8) {
        unsafe {
            self.handle.setChannel(channel);
        }
    }

    pub fn get_channel(&mut self) -> u8 {
        unsafe {
            self.handle.getChannel()
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
        if buffer.len() > self.handle.payload_size as usize {
            return Err(RF24Error::BufferTooLarge);
        }

        unsafe {
            self.handle.read(buffer.as_mut_ptr() as *mut _ as *mut c_void, buffer.len() as u8);
        }

        Ok(())
    }

    pub unsafe fn read_bytes(&mut self, buffer: &mut [u8], num_bytes: u8) -> Result<(), RF24Error> {
        if buffer.len() > self.handle.payload_size as usize {
            return Err(RF24Error::BufferTooLarge);
        }

        if buffer.len() < num_bytes as usize {
            return Err(RF24Error::BufferTooSmall);
        }

        self.handle.read(buffer.as_mut_ptr() as *mut _ as *mut c_void, num_bytes);

        Ok(())
    }

    pub fn read_dynamic_payload(&mut self, buffer: &mut Vec<u8>) -> Result<bool, RF24Error> {
        match self.get_dynamic_payload_size() {
            None => Ok(false),
            Some(size) => {
                buffer.clear();
                buffer.reserve(size as usize);

                unsafe {
                    return self.read_bytes(buffer, size).map(|_| true);
                }
            }
        }
    }

    pub fn write(&mut self, buffer: &[u8]) -> Result<bool, RF24Error> {
        if buffer.len() > self.handle.payload_size as usize {
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
        if buffer.len() > self.handle.payload_size as usize {
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
        if buffer.len() > self.handle.payload_size as usize {
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
        if buffer.len() > self.handle.payload_size as usize {
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
        if buffer.len() > self.handle.payload_size as usize {
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

    pub fn open_writing_pipe(&mut self, address: &[u8]) -> Result<(), RF24Error> {
        if address.len() > 5 {
            return Err(RF24Error::BufferTooLarge);
        }

        if address.len() < 5 {
            return Err(RF24Error::BufferTooSmall);
        }

        unsafe {
            self.handle.openWritingPipe(address.as_ptr());
            Ok(())
        }
    }

    pub fn open_reading_pipe(&mut self, pipe_number: u8, address: &[u8]) -> Result<(), RF24Error> {
        if address.len() > 5 {
            return Err(RF24Error::BufferTooLarge);
        }

        if address.len() < 5 {
            return Err(RF24Error::BufferTooSmall);
        }

        unsafe {
            self.handle.openReadingPipe(pipe_number, address.as_ptr());
            Ok(())
        }
    }

    pub fn close_reading_pipe(&mut self, pipe_number: u8) {
        unsafe {
            self.handle.closeReadingPipe(pipe_number);
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

    pub fn flush_tx(&mut self) -> u8 {
        unsafe {
            self.handle.flush_tx()
        }
    }

    pub fn flush_rx(&mut self) -> u8 {
        unsafe {
            self.handle.flush_rx()
        }
    }

    pub fn reuse_tx(&mut self) {
        unsafe {
            self.handle.reUseTX();
        }
    }

    pub fn set_auto_ack(&mut self, enable: bool) {
        unsafe {
            self.handle.setAutoAck(enable);
        }
    }

    pub fn set_auto_ack_pipe(&mut self, pipe: u8, enable: bool) {
        unsafe {
            self.handle.setAutoAck1(pipe, enable);
        }
    }

    pub fn is_ack_payload_available(&mut self) -> bool {
        unsafe {
            self.handle.isAckPayloadAvailable()
        }
    }

    pub fn enable_ack_payload(&mut self) {
        unsafe {
            self.handle.enableAckPayload();
        }
    }

    pub fn write_ack_payload(&mut self, pipe_number: u8, buffer: &[u8]) -> Result<(), RF24Error> {
        if !self.handle.dynamic_payloads_enabled {
            return Err(RF24Error::DynamicPayloadsNotEnabled);
        }

        unsafe {
            self.handle.writeAckPayload(pipe_number, buffer.as_ptr() as *const _ as *const c_void, buffer.len() as u8);
        }

        Ok(())
    }

    pub fn enable_dynamic_ack(&mut self) {
        unsafe {
            self.handle.enableDynamicAck();
        }
    }

    pub fn get_payload_size(&mut self) -> u8 {
        unsafe {
            self.handle.getPayloadSize()
        }
    }

    pub fn set_payload_size(&mut self, size: u8) -> Result<(), RF24Error> {
        if size > 5 {
            return Err(RF24Error::AddressLengthTooLarge);
        }

        if size < 3 {
            return Err(RF24Error::AddressLengthTooSmall);
        }

        unsafe {
            self.handle.setPayloadSize(size);
            Ok(())
        }
    }

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

    pub fn carrier_available(&mut self) -> bool {
        unsafe {
            self.handle.testCarrier()
        }
    }

    pub fn test_rpd(&mut self) -> bool {
        unsafe {
            self.handle.testRPD()
        }
    }
}
