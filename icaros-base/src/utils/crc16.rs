#[inline]
pub fn _crc16_update(curr_crc: u16, data: u8) -> u16 {
    let mut crc = curr_crc;
    crc ^= data as u16;

    for _ in 0..8 {
        if (crc & 1) == 1 {
            crc = (crc >> 1) ^ 0xA001;
        } else {
            crc = crc >> 1;
        }
    }

    crc
}

pub fn crc16_calculate(data: &[u8]) -> u16 {
    let mut crc: u16 = 0; // Initial value
    for byte in data {
        crc = _crc16_update(crc, *byte);
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    fn crc16_check(data: &[u8], crc_expected: u16) -> bool {
        crc_expected == crc16_calculate(data)
    }

    #[test]
    fn checksum_crc16() {
        let data = &[0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39];
        let crc_expected: u16 = 0xBB3D;

        println!("{:x}", crc16_calculate(data));
        assert_eq!(crc16_check(data, crc_expected), true)
    }
}
