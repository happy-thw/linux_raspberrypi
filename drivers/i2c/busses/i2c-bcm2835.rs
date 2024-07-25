// SPDX-License-Identifier: GPL-2.0
/*
 * BCM2835 master mode driver
 */

use bitflags::bitflags;
use i2c::msg::I2cMsgFlags;
use kernel::{
    bindings, clk::Clk, device::Device, io_mem::IoMem, macros::module, platform::Adapter,
};

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

// Wait for implemented Debug feature
struct BCM2835_Debug {
    msg: I2cMsgFlags,
    msg_idx: i32,
    remian: usize,
    status: u32,
}

// unimplemented
struct BCM2835_I2C_Dev {
    dev: Device,
    irq: i32,
    __iomem: IoMem,
    adapter: Adapter<BCM2835_I2C_Device>,
    bus_clk: Clk,
    num_msgs: i32,
    // omit debug fields for now.
    // msg_err: u32,
    // msg_buf: [u8; 16],
    // msg_buf_remaining: usize,
    // debug: [Option<BCM2835_Debug>; 4]
    // debug_num: u32,
    // debug_num_msgs: u32,
}

struct BCM2835_I2C_Device;

module! {
    type: BCM2835_I2C_Device,
    name:"BCM2835_I2C_Device",
    author:"<NAME> <<EMAIL>>",
    description:"BCM2835 I2C driver",
    license:"GPL v2",
}

// test
fn bcm2835_i2c_readl(i2c_dev: &mut BCM2835_I2C_Dev, reg: u32, val: u32) -> u32 {
    let i2c_reg = i2c_dev.__iomem.get();
    let addr = i2c_reg.wrapping_add(reg);
    unsafe { bindings::writel_relaxed(val as _, addr as _) }
}
