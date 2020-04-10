//! CRC calculation utilities
//!
//! Source: From [^1] which by itself cites:
//! > *See the Dallas Semiconductor app note 27 for 8051 assembler example and general CRC optimization suggestions. The table on the last page of the app note is the key to understanding these implementations.*
//! >
//! > *Jack Crenshaw's "Implementing CRCs" article in the January 1992 isue of Embedded Systems Programming. This may be difficult to find, but it explains CRC's in very clear and concise terms. Well worth the effort to obtain a copy.*
//!
//! [^1]: [https://www.nongnu.org/avr-libc/user-manual/group__util__crc.html](https://www.nongnu.org/avr-libc/user-manual/group__util__crc.html)


/// CRC calculation update function mimicking the one built into AVR processors' standard
/// libraries (i.e. Arduino)
///
/// Source: From [^1] which by itself cites:
/// > *See the Dallas Semiconductor app note 27 for 8051 assembler example and general CRC optimization suggestions. The table on the last page of the app note is the key to understanding these implementations.*
/// >
/// > *Jack Crenshaw's "Implementing CRCs" article in the January 1992 isue of Embedded Systems Programming. This may be difficult to find, but it explains CRC's in very clear and concise terms. Well worth the effort to obtain a copy.*
///
/// [^1]: [https://www.nongnu.org/avr-libc/user-manual/group__util__crc.html](https://www.nongnu.org/avr-libc/user-manual/group__util__crc.html)
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

/// Calculates the CRC checksum of a given array
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
