// SPDX-License-Identifier: GPL-2.0

//! BCM2835 master mode driver
use i2c::I2cMsg;
use kernel::{
    clk::Clk, clk_hw::ClkHw, completion::Completion, container_of, device::Device, io_mem::IoMem,
    macros::module, prelude::*,
};

use bitflags::bitflags;

/// I2C 地址预留空间
const I2C_SIZE: usize = 0x100;

/// I2C 控制寄存器地址偏移
pub const BCM2835_I2C_C: u32 = 0x0;
/// I2C 状态寄存器地址偏移
pub const BCM2835_I2C_S: u32 = 0x4;
/// I2C 数据长度寄存器地址偏移
pub const BCM2835_I2C_DLEN: u32 = 0x8;
/// I2C 从机地址寄存器地址偏移
pub const BCM2835_I2C_A: u32 = 0xc;
/// I2C 数据 FIFO 寄存器地址偏移
pub const BCM2835_I2C_FIFO: u32 = 0x10;
/// I2C 时钟分频寄存器地址偏移
pub const BCM2835_I2C_DIV: u32 = 0x14;
/// I2C 数据延迟寄存器地址偏移
pub const BCM2835_I2C_DEL: u32 = 0x18;

/// 16-bit field for the number of SCL cycles to wait after rising SCL
/// before deciding the slave is not responding. 0 disables the
/// timeout detection.
/// I2C 时钟超时寄存器地址偏移
pub const BCM2835_I2C_CLKT: u32 = 0x1c;

bitflags! {
    /// Defines the i2c control register bit fields
    #[allow(non_camel_case_types)]
    #[repr(transparent)]
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    struct I2cRegControl:u32 {
        const READ = 1 << 1;
        const CLEAR = 1 << 4;  // bits 4 and 5 both clear
        const ST = 1 << 7;
        const INTD = 1 << 8;
        const INTT = 1 << 9;
        const INTR = 1 << 10;
        const I2CEN = 1 << 15;
    }
}

bitflags! {
    /// Defines the i2c status register bit fields
    #[allow(non_camel_case_types)]
    #[repr(transparent)]
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    struct I2cRegStatus: u32 {
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

///
pub const BCM2835_I2C_FEDL_SHIFT: u32 = 16;
///
pub const BCM2835_I2C_REDL_SHIFT: u32 = 0;

///
pub const BCM2835_I2C_CDIV_MIN: u32 = 0x0002;
///
pub const BCM2835_I2C_CDIV_MAX: u32 = 0xFFFE;

// static in C code
pub const CLK_TOUT_MS: u32 = 35;
// Wait for implemented Debug feature
#[allow(unused)]
struct Bcm2835Debug {
    // msg: I2cMsgFlags,
    msg_idx: i32,
    remian: usize,
    status: u32,
}

#[allow(unused)]
struct Bcm2835I2cDev {
    dev: Device,
    irq: i32,
    regs: IoMem<I2C_SIZE>,
    // adapter: Adapter<Bcm2835I2cDev>,
    completion: Completion,
    curr_msg: I2cMsg,
    bus_clk: Clk,
    num_msgs: i32,
    // omit debug fields for now.
    // msg_err: u32,
    // msg_buf: Vec<u8>,
    // msg_buf_remaining: usize,
    // debug: [Option<BCM2835_Debug>; 4]
    // debug_num: u32,
    // debug_num_msgs: u32,
}

#[allow(unused)]
struct Bcm2835I2cData {}

struct Bcm2835I2cDevice;

module! {
    type: Bcm2835I2cDevice,
    name:"i2c_bcm2835_rust",
    author:"<NAME> <<EMAIL>>",
    description:"BCM2835 I2C driver (written in rust)",
    license:"GPL",
}

// test
#[allow(unused)]
fn bcm2835_i2c_writel(_i2c_dev: &mut Bcm2835I2cDev, _reg: usize, _val: u32) {
    // let i2c_reg = i2c_dev.regs.get();
    // let addr = i2c_reg.wrapping_add(reg);
    // unsafe { bindings::writel_relaxed(val as _, addr as _) }
}

#[allow(unused)]
fn bcm2835_i2c_readl(_i2c_dev: &mut Bcm2835I2cDev, _reg: usize) -> u32 {
    // let i2c_reg = i2c_dev.regs.get();
    // let addr = i2c_reg.wrapping_add(reg);
    // unsafe { bindings::readl_relaxed(addr as _) }
    0
}

fn to_clk_bcm2835_i2c(_hw: &mut ClkHw) -> &mut ClkBcm2835I2c {
    container_of!(_hw, ClkBcm2835I2c, hw)
}
struct ClkBcm2835I2c {
    hw: ClkHw,
    i2c_dev: Bcm2835I2cDev,
}

fn clk_bcm2835_i2c_calc_divider(rate: u64, parent_rate: u64) -> Result<u64> {
    let divider = parent_rate.div_ceil(rate);

    /*
     * Per the datasheet, the register is always interpreted as an even
     * number, by rounding down. In other words, the LSB is ignored. So,
     * if the LSB is set, increment the divider to avoid any issue.
     */
    if (divider & 0x1) != 0 {
        divider += 1;
    }
    if ((divier < BCM2835_I2C_CDIV_MIN) || (divider > BCM2835_I2C_CDIV_MAX)) {
        return Err(error::code::EINVAL);
    }

    return divider;
}

fn clk_bcm2835_i2c_set_rate(hw: &mut ClkHw, rate: u64, parent_rate: u64) -> Result<()> {
    let div = to_clk_bcm2835_i2c(&mut hw);
    let divider = clk_bcm2835_i2c_calc_divider(rate, parent_rate)?;

    bcm2835_i2c_writel(&mut div.i2c_dev, BCM2835_I2C_DIV, divider);

    /*
     * Number of core clocks to wait after falling edge before
     * outputting the next data bit.  Note that both FEDL and REDL
     * can't be greater than CDIV/2.
     */
    let fedl = max(divider / 16, 1u);

    /*
     * Number of core clocks to wait after rising edge before
     * sampling the next incoming data bit.
     */
    let redl = max(divider / 4, 1u);

    bcm2835_i2c_writel(
        &mut div.i2c_dev,
        BCM2835_I2C_DEL,
        (fedl << BCM2835_I2C_FEDL_SHIFT) | (redl << BCM2835_I2C_REDL_SHIFT),
    );

    /*
     * Set the clock stretch timeout.
     */
    let clk_tout: u32;
    if rate > 0xffff * 1000 / CLK_TOUT_MS {
        clk_tout = 0xffff;
    } else {
        clk_tout = CLK_TOUT_MS * rate / 1000;
    }

    bcm2835_i2c_writel(&mut div.i2c_dev, BCM2835_I2C_CLKT, clk_tout);

    Ok(())
}

impl kernel::Module for Bcm2835I2cDevice {
    fn init(_module: &'static ThisModule) -> Result<Self> {
        pr_info!("BCM2835 i2c bus device driver (init)\n");

        Ok(Bcm2835I2cDevice {})
    }
}

impl Drop for Bcm2835I2cDevice {
    fn drop(&mut self) {
        pr_info!("BCM2835 i2c bus device driver (exit)\n");
    }
}
