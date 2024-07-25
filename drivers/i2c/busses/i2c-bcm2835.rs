// SPDX-License-Identifier: GPL-2.0
/*
 * BCM2835 master mode driver
 */

use bitflags::bitflags;
use i2c::msg::I2cMsgFlags;

pub const BCM2835_I2C_C: u32 = 0x0;
pub const BCM2835_I2C_S: u32 = 0x4;
pub const BCM2835_I2C_DLEN: u32 = 0x8;
pub const BCM2835_I2C_A: u32 = 0xc;
pub const BCM2835_I2C_FIFO: u32 = 0x10;
pub const BCM2835_I2C_DIV: u32 = 0x14;
pub const BCM2835_I2C_DEL: u32 = 0x18;

/// 16-bit field for the number of SCL cycles to wait after rising SCL
/// before deciding the slave is not responding. 0 disables the
/// timeout detection.
pub const BCM2835_I2C_CLKT: u32 = 0x1c;

bitflags! {

    #[repr(transparent)]
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    pub struct BCM2835_I2C_C:u32 {
        const READ = 1 << 0;
        const CLEAR = 1 << 4;  // bits 4 and 5 both clear
        const ST = 1 << 7;
        const INTD = 1 << 8;
        const INTT = 1 << 9;
        const INTR = 1 << 10;
        const I2CEN = 1 << 15;
    }
}

bitflags! {

	#[repr(transparent)]
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
	pub struct BCM2835_I2C_S: u32 {
        const TA = 1 << 0;
        const DONE = 1 << 1;
        const TXW = 1 << 2;
        const RXR = 1 << 3;
        const TXD = 1 << 4;
        const RXD = 1 << 5;
        const TXE = 1 << 6;
        const RXF = 1 << 7;
        const ERR = 1 << 8;
        const CLKT = 1 << 9;
        const LEN = 1 << 10;  // Fake bit for SW error reporting
    }
}

pub const BCM2835_I2C_FEDL_SHIFT: u32 = 16;
pub const BCM2835_I2C_REDL_SHIFT: u32 = 0;

pub const BCM2835_I2C_CDIV_MIN: u32 = 0x0002;
pub const BCM2835_I2C_CDIV_MAX: u32 = 0xFFFE;

struct BCM2835_Debug {
    msg:I2cMsgFlags,
    msg_idx:i32,
    remian:usize,
    status:u32,
}

