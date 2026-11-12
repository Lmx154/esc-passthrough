//! Reference implementations written independently of the crate: bit-serial CRCs.
#![allow(dead_code)]

/// CRC-16/ARC (polynomial 0xA001 reflected, start 0), one data bit at a time through a shift
/// register, LSB first.
pub fn ref_crc_arc(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        for bit in 0..8 {
            let data_bit = u16::from((byte >> bit) & 1);
            let feedback = (crc & 1) ^ data_bit;
            crc >>= 1;
            if feedback == 1 {
                crc ^= 0xA001;
            }
        }
    }
    crc
}

/// CRC16-XMODEM (polynomial 0x1021, start 0), one data bit at a time through a shift register,
/// MSB first.
pub fn ref_crc_xmodem(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        for bit in (0..8).rev() {
            let data_bit = u16::from((byte >> bit) & 1);
            let feedback = ((crc >> 15) & 1) ^ data_bit;
            crc <<= 1;
            if feedback == 1 {
                crc ^= 0x1021;
            }
        }
    }
    crc
}

/// Bytes biased towards the frame markers of the three formats, to reach the parser states.
pub fn marker_biased_byte() -> impl proptest::strategy::Strategy<Value = u8> {
    use proptest::prelude::*;
    prop_oneof![
        4 => any::<u8>(),
        1 => Just(b'$'),
        1 => Just(b'M'),
        1 => Just(b'<'),
        1 => Just(0x2Fu8),
        1 => Just(0u8),
    ]
}
