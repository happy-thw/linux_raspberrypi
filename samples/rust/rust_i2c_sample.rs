// SPDX-License-Identifier: GPL-2.0

//! Rust dw_apb_i2c
//! 
//! Refer to : 
//! (drivers/i2c/busses/i2c-designware-platdrv.c)
//! (drivers/i2c/busses/i2c-designware-master.c)
//! (drivers/i2c/busses/i2c-designware-common.c)
//! (drivers/i2c/busses/i2c-designware-core.h)
//! 

use kernel::{
    driver,
    bindings,
    c_str,
    i2c::{
        self,
        I2cAlgo,
        I2cMsg,
        I2cAdapter,
        i2c_detect_slave_mode,
        functionality::I2cFuncFlags,
        msg::{
            self, I2cMsgFlags, I2cMsgInfo, GeneralI2cMsgInfo,
        },
        timing::{
            self, I2cTiming, I2cSpeedMode,
        }
    },
    regmap::{self, BitFieldReadOps, BitFieldWriteOps, RawFieldReadOps, RawFieldWriteOps},
    device::{Device, RawDevice},
    module_platform_driver, of, platform,
    sync::{Arc, SpinLock, ArcBorrow},
    prelude::*,
    b_str,
    math,
    error,
    delay,
    irq,
    completion::Completion,
    new_completion,
    new_spinlock,

    // sync::ArcBorrow,
};

use register::*;
use core::ffi::c_void;

/// Designware Component Type number = 0x44_57_01_40. This
/// assigned unique hex value is constant and is derived from the two
/// ASCII letters “DW” followed by a 16-bit unsigned number.
pub const DW_IC_COMP_TYPE_VALUE: u32 = 0x44570140;
/// "111" = v1.11
pub const DW_IC_SDA_HOLD_MIN_VERS: u32 = 0x3131312A;
/// GENMASK(23, 16) 
pub const DW_IC_SDA_HOLD_RX_MASK: u32 = regmap::genmask(23, 16);

/// DW_IC_INTR_RX_FULL  
pub const DW_IC_INTR_RX_FULL:u32 = 1 << 2 ;
/// DW_IC_INTR_TX_ABRT
pub const DW_IC_INTR_TX_EMPTY:u32 = 1 << 4 ;
/// DW_IC_INTR_STOP_DET
pub const DW_IC_INTR_TX_ABRT:u32 = 1 << 6 ;
/// DW_IC_INTR_TX_EMPTY
pub const DW_IC_INTR_STOP_DET:u32 = 1 << 9 ;
/// DW_IC_INTR_MASTER_MASK
pub const DW_IC_INTR_MASTER_MASK:u32 = DW_IC_INTR_RX_FULL  | 
    DW_IC_INTR_TX_EMPTY | 
    DW_IC_INTR_TX_ABRT  | 
    DW_IC_INTR_STOP_DET ;

/// DW_IC_TX_ARB_LOST
pub const DW_IC_TX_ARB_LOST: u32 = 1 << 12;

///DW_IC_TX_ABRT_GCALL_READ
pub const DW_IC_TX_ABRT_GCALL_READ: u32 = 1 << 5;

/// DW_IC_TX_ABRT_NOACK
pub const DW_IC_TX_ABRT_NOACK: u32 = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) ;

/// Data for SMBus Messages
pub const I2C_SMBUS_BLOCK_MAX:u8 = 32;

///  dw-i2c-defualt functionality
pub const DW_I2C_DEFAULT_FUNCTIONALITY: u32 = I2cFuncFlags::I2C |
    I2cFuncFlags::SMBUS_BYTE       |
    I2cFuncFlags::SMBUS_BYTE_DATA  |
    I2cFuncFlags::SMBUS_WORD_DATA  |
    I2cFuncFlags::SMBUS_BLOCK_DATA |
    I2cFuncFlags::SMBUS_I2C_BLOCK  ;

static mut I2C_DW_ALGO: bindings::i2c_algorithm = bindings::i2c_algorithm {
    master_xfer: None,
    master_xfer_atomic: None,
    smbus_xfer: None,
    smbus_xfer_atomic: None,
    functionality: None,
    reg_slave: None,
    unreg_slave: None,
};

static I2C_DW_QUIRKS: bindings::i2c_adapter_quirks = bindings::i2c_adapter_quirks {
    flags: i2c::I2C_AQ_NO_ZERO_LEN,
    max_num_msgs: 0,
    max_write_len: 0,
    max_read_len: 0,
    max_comb_1st_msg_len: 0,
    max_comb_2nd_msg_len: 0,
};

const I2C_DESIGNWARE_SUPPORT_SPEED: [u32; 4] = [
    timing::I2C_MAX_STANDARD_MODE_FREQ,
    timing::I2C_MAX_FAST_MODE_FREQ,
    timing::I2C_MAX_FAST_MODE_PLUS_FREQ,
    timing::I2C_MAX_HIGH_SPEED_MODE_FREQ,
];

module_platform_driver! {
      type: DwI2cDriver,
      name: "i2c_designware",
      license: "GPL",
      initcall: "subsys",
}

// Linux Raw id table
kernel::module_of_id_table!(DW_I2C_MOD_TABLE, DW_I2C_OF_MATCH_TABLE);
// R4L IdArray table
kernel::define_of_id_table! {DW_I2C_OF_MATCH_TABLE, (), [
    (of::DeviceId::Compatible(b_str!("snps,designware-i2c")),None),
]}

#[pin_data]
struct DwI2cData {
    #[pin]
    i2c_adapter: I2cAdapter<DwI2cAlgo>,
    #[pin]
    driver: I2cDwMasterDriver,
    irq: irq::Registration::<DwI2cIrqHandler>,
}

impl driver::DeviceRemoval for DwI2cData {
    fn device_remove(&self) {
        pr_info!("unimplement DwI2cData Remove");
    }
}

impl DwI2cData {
    fn new(
        driver: I2cDwMasterDriver,
        parent: *mut bindings::device,
        of_node: *mut bindings::device_node,
        irq: u32,
    ) -> Arc<Self> {
        Arc::pin_init(pin_init!(&this in Self {
            i2c_adapter <- I2cAdapter::<DwI2cAlgo>::new(
                c_str!("Synopsys DesignWare I2C adapter"),
                &THIS_MODULE,
                parent,
                &I2C_DW_QUIRKS as *const bindings::i2c_adapter_quirks,
                //SAFETY: init by adapter
                unsafe {&mut I2C_DW_ALGO as *mut bindings::i2c_algorithm},
                of_node), 
            driver: driver,
            irq: irq::Registration::<DwI2cIrqHandler>::try_new(irq,
                //SAFETY: this is always valid
                unsafe {Arc::from_raw(this.as_ptr())},
                irq::flags::SHARED|irq::flags::COND_SUSPEND,
                fmt!("dw_i2c_irq_{irq}")).unwrap(),
        })).unwrap()
    }
}

struct DwI2cAlgo;
#[vtable]
impl I2cAlgo for DwI2cAlgo {
    type Data = Arc<DwI2cData>;
    fn master_xfer(data: ArcBorrow<'_, DwI2cData>, msgs: &I2cMsg, msg_num: usize) -> Result<i32> {
        dbg!("I2C:msg: {} \n", msg_num);
        let trans_msgs = msgs.into_array(msg_num, |x: &mut bindings::i2c_msg| {
           I2cMsgInfo::new_raw(x.addr, x.flags, x.buf, x.len as usize)
        })?;

        let master_driver = &data.driver;
        master_driver.master_transfer(trans_msgs)
    }

    fn functionality(data: ArcBorrow<'_, DwI2cData>) -> u32 {
        let master_driver = &data.driver;
        dbg!("I2C:functionality: {} \n", master_driver.get_functionality());
        master_driver.get_functionality()
    }
}

struct DwI2cIrqHandler;
impl irq::Handler for DwI2cIrqHandler {
    type Data = Arc<DwI2cData>;

    fn handle_irq(data: ArcBorrow<'_, DwI2cData>) -> irq::Return {
        let master_driver = &data.driver;
        master_driver.irq_handler()
    }
}

struct DwI2cDriver;
impl platform::Driver for DwI2cDriver {
    type Data = Arc<DwI2cData>;

    // Linux Raw id table
    kernel::driver_of_id_table!(DW_I2C_OF_MATCH_TABLE);

    fn probe(pdev: &mut platform::Device,_id_info: Option<&Self::IdInfo> ) -> Result<Self::Data> {
        let irq = pdev.irq_resource(0)?;
        let reg_base = pdev.ioremap_resource(0)?;
        let dev = Device::from_dev(pdev);
        let timing = I2cTiming::i2c_parse_fw_timings(&dev, I2cSpeedMode::StandMode, false);
        dev_info!(dev, "enter i2c platform probe func, get irq {}\n",irq);
        dev_info!(dev, "rust reg_base is {:#x}",reg_base as usize);
        dev_info!(dev, "i2c's bus_freq_hz is  {}", timing.get_bus_freq_hz());
        let config = regmap::Config::<AccessOps>::new(32, 32, 4)
            .with_max_register(0xfc)
            .with_cache_type(regmap::CacheType::RbTree);
        let regmap = Arc::try_new(regmap::Regmap::init(&dev, reg_base as *mut c_void, &config))?;
        let fields = regmap.alloc_fields(&FIELD_DESCS)?;

        if i2c_detect_slave_mode(&dev) {
            pr_err!("unimplement dw slave driver");
            return Err(ENODEV);
        }

        // clk
        let clk = dev.devm_clk_get_default_optional()?;
        clk.prepare_enable()?;
        let clk_rate_khz = (clk.get_rate() / 1000) as u32;
        dev_info!(dev, "i2c's clk_rate_khz is  {}", clk_rate_khz);
        let driver_config = I2cDwDriverConfig::new(timing, clk_rate_khz);
        let mut i2c_master_driver = I2cDwMasterDriver::new(driver_config, fields);
        i2c_master_driver.setup()?;

        // create data
        let data = DwI2cData::new(
            i2c_master_driver, 
            dev.raw_device(), 
            pdev.of_node(), 
            irq as u32,
        );

        // register adapter
        (&(data.i2c_adapter)).add_numbered_adapter(data.clone())?;
        Ok(data)
    }
}


/// I2cDwDriverConfig
#[derive(Debug, Clone)]
pub struct I2cDwDriverConfig {
    timing: I2cTiming,
    clk_rate_khz: u32,
}

impl I2cDwDriverConfig {
    /// Create  a Config
    pub fn new(timing: I2cTiming, clk_rate_khz: u32) -> Self {
        Self { timing,  clk_rate_khz  }
    }
}

/// The I2cDesignware Core Driver
pub(crate) struct I2cDwCoreDriver {
    /// I2c Registers
    pub(crate) fields: regmap::Fields<{ FIELD_DESCS.count() }>,
    /// Config From external
    pub(crate) ext_config: I2cDwDriverConfig,
    /// Corrected bus_freq_hz
    pub(crate) bus_freq_hz: u32,
    // Corrected sda_hold_time
    pub(crate) sda_hold_time: u32,
    /// I2c functionality
    pub(crate) functionality: u32,
    /// I2c SpeedMode
    speed_mode: I2cSpeedMode,
}

unsafe impl Sync for I2cDwCoreDriver {}
unsafe impl Send for I2cDwCoreDriver {}

#[allow(dead_code)]
impl I2cDwCoreDriver {
    pub(crate) fn new(config: I2cDwDriverConfig, field: regmap::Fields<{ FIELD_DESCS.count() }>) -> Self {
        Self {
            ext_config: config,
            fields: field,
            bus_freq_hz: 0,
            sda_hold_time: 0,
            functionality: DW_I2C_DEFAULT_FUNCTIONALITY,
            speed_mode: I2cSpeedMode::StandMode,
        }
    }

    pub(crate) fn speed_check(&mut self) -> Result<()> {
        let bus_freq_hz = self.ext_config.timing.get_bus_freq_hz();
        dbg!("support bus_freq_hz is {}\n", bus_freq_hz);
        if !I2C_DESIGNWARE_SUPPORT_SPEED.contains(&bus_freq_hz) {
            pr_err!("{bus_freq_hz} Hz is unsupported, only 100kHz, 400kHz, 1MHz and 3.4MHz are supported");
            return Err(EINVAL);
        }
        self.bus_freq_hz = bus_freq_hz;
        // test
        let b = ic_comp_param_1::max_speed_mode::read(&mut self.fields)? as u32  ;
        pr_info!("i2c's speed_check ic_comp_type is  {:x}",b);

        if ! (ic_comp_param_1::max_speed_mode::read(&mut self.fields)? as u32 == 0x11)
            && self.bus_freq_hz == timing::I2C_MAX_HIGH_SPEED_MODE_FREQ
        {
            pr_err!("High Speed not supported! Fall back to fast mode");
            self.bus_freq_hz = timing::I2C_MAX_FAST_MODE_FREQ;
        }

        self.speed_mode = I2cSpeedMode::from_bus_freq(self.bus_freq_hz);
        Ok(())
    }

    pub(crate) fn com_type_check(&mut self) -> Result<()> {
        let com_type= ic_comp_type::comp_type::read(&mut self.fields)?;
        if com_type == DW_IC_COMP_TYPE_VALUE {
            pr_info!("I2C: com_type check Ok");
        } else if com_type == DW_IC_COMP_TYPE_VALUE & 0x0000ffff {
            pr_err!("I2C: com_type check Failed, not support 16 bit system ");
            return Err(ENODEV);
        } else if com_type == DW_IC_COMP_TYPE_VALUE.to_be() {
            pr_err!("I2C: com_type check Failed, not support BE system ");
            return Err(ENODEV);
        } else {
            pr_err!(
                "I2C: com_type check failed, Unknown Synopsys component type: {:x}",
                com_type
            );
            return Err(ENODEV);
        }
        Ok(())
    }

    #[inline]
    pub(crate) fn functionality_add(&mut self, functionality: u32) {
        self.functionality |= functionality;
    }

    pub(crate) fn cfg_init_speed(&self) -> Result {
        match self.speed_mode {
            I2cSpeedMode::StandMode => {
                dbg!("StandMode");
                ic_con::speed::write(&self.fields, 0b01 )},
            I2cSpeedMode::HighSpeedMode => ic_con::speed::write(&self.fields, 0b11 ),
            _ => ic_con::speed::write(&self.fields, 0b10 ),
        }
    }

    #[inline]
    pub(crate) fn enable_10bitaddr(&self, enable: bool) -> Result {
        if enable {
            ic_con::ic_10bitaddr_master::set(&self.fields)
        } else {
            ic_con::ic_10bitaddr_master::clear(&self.fields)
        }
    }

    pub(crate) fn write_sda_hold_time(&self) {
        if self.sda_hold_time !=0 {
            let _ = ic_sda_hold::sda_hold::write(&self.fields, self.sda_hold_time);
        }
        pr_info!("read sda_hold_time sda_tx_hold {:#x}", ic_sda_hold::sda_hold::read(&self.fields).unwrap());
    }

    pub(crate) fn read_sda_hold_time(&mut self) -> u32 {
        let sda_hold_time = ic_sda_hold::sda_hold::read(&mut self.fields).unwrap();
        pr_info!("read sda_hold_time sda_tx_hold {:#x}", self.sda_hold_time);
        sda_hold_time
    }

    pub(crate) fn sda_hold_time_init(&mut self) -> Result<()> {
        let comp_ver:u32 = ic_comp_version::version::read(&mut self.fields)?;

        let ext_sda_hold_ns = self.ext_config.timing.get_sda_hold_ns();

        if comp_ver < DW_IC_SDA_HOLD_MIN_VERS {
            pr_warn!("Hardware too old to adjust SDA hold time.");
            self.sda_hold_time = 0;
            return Ok(());
        } else if ext_sda_hold_ns == 0 {
            self.sda_hold_time = ic_sda_hold::sda_hold::read(&mut self.fields)?;
        }else{
            let sda_hold_time = math::div_round_closest_ull(
                (self.ext_config.clk_rate_khz * ext_sda_hold_ns).into(),
                math::MICRO) as u32;
            pr_info!(" DIV_MATH:sda_hold_time is {:#x} ",sda_hold_time );
            // Workaround for avoiding TX arbitration lost in case I2C
            // slave pulls SDA down "too quickly" after falling edge of
            // SCL by enabling non-zero SDA RX hold. Specification says it
            // extends incoming SDA low to high transition while SCL is
            // high but it appears to help also above issue.  
            if (sda_hold_time & DW_IC_SDA_HOLD_RX_MASK) == 0 {
                self.sda_hold_time= sda_hold_time | (1 << 16);
            }
        }
        pr_info!(
            "sda hold time Tx:Rx = {}:{}",
            self.sda_hold_time & 0xFFFF,
            self.sda_hold_time >> 16,
        );
        pr_info!("I2C  Bus Speed: {}", self.speed_mode);
        Ok(())
    }

    pub(crate) fn write_lhcnt(&self, lhcnt: &DwI2cSclLHCnt) {
        // Write standard speed timing parameters
        let _ = ic_ss_or_ufm_scl_hcnt::value::write(&self.fields, lhcnt.ss_lcnt.into());
        let _ = ic_ss_or_ufm_scl_lcnt::value::write(&self.fields, lhcnt.ss_lcnt.into());
        pr_info!("write SCL_LCNT:HCNT  {}:{}", lhcnt.ss_lcnt, lhcnt.ss_hcnt);

        // Write fast mode/fast mode plus timing parameters
        let _ = ic_fs_scl_lcnt::value::write(&self.fields, lhcnt.fs_lcnt.into());
        let _ = ic_fs_scl_hcnt_or_ufm_tbuf_cnt::value::write(&self.fields, lhcnt.fs_hcnt.into());
        pr_info!("write FS_SCL_LCNT:HCNT  {}:{}", lhcnt.fs_lcnt, lhcnt.fs_hcnt);

        // Write high speed timing parameters if supported
        if self.speed_mode == I2cSpeedMode::HighSpeedMode {
            let _ = ic_hs_scl_lcnt::value::write(&self.fields, lhcnt.hs_lcnt.into());
            let _ = ic_hs_scl_hcnt::value::write(&self.fields, lhcnt.hs_hcnt.into());
            pr_info!("write HS_SCL_LCNT:HCNT {}:{}", lhcnt.hs_lcnt, lhcnt.hs_hcnt);
        }
    }

    #[inline]
    pub(crate) fn write_fifo(&self, ic_tx: u32, ic_rx: u32) {
        let _ = ic_tx_tl::value::write(&self.fields, ic_tx);
        let _ = ic_rx_tl::value::write(&self.fields, ic_rx);
        pr_info!("write fifo tx:rx {}:{}", ic_tx, ic_rx);
    }

    pub(crate) fn wait_bus_not_busy(&self) -> Result<bool> {
        let status: u32 = 0;
        let ret = ic_status::activity::read_poll_timeout(
            &self.fields, status, !status, 1100, 20000 );
        if ret != 0 {
            pr_err!(" while waiting for bus ready\n");
            //TODO:i2c_recover_bus
            if !ic_status::activity::is_set(&self.fields).unwrap() {
                let _ = error::to_result(ret);
            }
        }
        Ok(ret == 0 )
    }

    pub(crate) fn read_and_clear_intrbits(
        &self, 
        rx_outstanding: isize,
    ) -> u32{
        // The IC_INTR_STAT register just indicates "enabled" interrupts.
        // The unmasked raw version of interrupt status bits is available
        // in the IC_RAW_INTR_STAT register.
        //
        // That is,
        // stat = readl(IC_INTR_STAT);
        // equals to,
        // stat = readl(IC_RAW_INTR_STAT) & readl(IC_INTR_MASK);
        // The raw version might be useful for debugging purposes.
        let field = &self.fields;
        let mut abort_source = 0 ;
        // Do not use the IC_CLR_INTR register to clear interrupts, or
        // you'll miss some interrupts, triggered during the period from
        // readl(IC_INTR_STAT) to readl(IC_CLR_INTR).
        // Instead, use the separately-prepared IC_CLR_* registers.

        if ic_intr_stat::r_rx_under::is_set(field).unwrap() {
            let _ = ic_clr_rx_under::clr_rx_under::is_set(field);
        }
        if ic_intr_stat::r_rx_over::is_set(field).unwrap() {
            let _ = ic_clr_rx_over::clr_rx_over::is_set(field);
        }
        if ic_intr_stat::r_tx_over::is_set(field).unwrap() {
            let _ = ic_clr_tx_over::clr_tx_over::is_set(field);
        }
        if ic_intr_stat::r_rd_req::is_set(field).unwrap() {
            let _ = ic_clr_rd_req::clr_rd_req::is_set(field);
        }
        if ic_intr_stat::r_tx_abrt::is_set(field).unwrap() {
            // The IC_TX_ABRT_SOURCE register is cleared whenever
		    // the IC_CLR_TX_ABRT is read.  Preserve it beforehand.
            abort_source = ic_tx_abrt_source::abrt_source::read(field).unwrap();
            let _ = ic_clr_tx_abrt::clr_tx_abrt::is_set(field);
        }
        if ic_intr_stat::r_rx_done::is_set(field).unwrap() {
            let _ = ic_clr_rx_done::clr_rx_done::is_set(field);
        }
        if ic_intr_stat::r_activity::is_set(field).unwrap() {
            let _ = ic_clr_activity::clr_activity::is_set(field);
        }
        if ic_intr_stat::r_stop_det::is_set(field).unwrap() {
            if rx_outstanding == 0 || ic_intr_stat::r_rx_full::is_set(field).unwrap() {
            let _ = ic_clr_stop_det::clr_stop_det::is_set(field);
            }
        }
        if ic_intr_stat::r_start_det::is_set(field).unwrap() {
            let _ = ic_clr_start_det::clr_start_det::is_set(field);
        }
        if ic_intr_stat::r_gen_call::is_set(field).unwrap() {
            let _ = ic_clr_gen_call::clr_gen_call::is_set(field);
        }
        abort_source
    }

    #[inline]
    pub(crate) fn enable_master_intr(&self) {
        let _ = ic_intr_mask::mask_val::set_bits(
            &self.fields, 
            DW_IC_INTR_MASTER_MASK,
        );
    }

    #[inline]
    pub(crate) fn enable_tx_empty_intr(&self, enable: bool) {
        if enable {
            let _ = ic_intr_mask::mask_val::set_bits(&self.fields, DW_IC_INTR_TX_EMPTY);
        } else {
            let _ = ic_intr_mask::mask_val::clear_bits(&self.fields, DW_IC_INTR_TX_EMPTY);
        }
    }

    #[inline]
    pub(crate) fn disable_all_interrupt(&self) {
        let _ = ic_intr_mask::mask_val::write(&self.fields, 0);
    }

    #[inline]
    pub(crate) fn clear_all_interrupt(&self) {
        let _ = ic_clr_intr::clr_intr::is_set(&self.fields); // read bit(0) -> clear
    }

    pub(crate) fn disable(&mut self) {
        self.disable_controler();
        // Disable all interrupts
        self.disable_all_interrupt();
        self.clear_all_interrupt();
    }

    pub(crate) fn enable_controler(&self) {
        let _ = ic_enable::other::write(&self.fields, 0);
        let _ = ic_enable::abort::clear(&self.fields);
        let _ = ic_enable::enable::set(&self.fields);
    }

    pub(crate) fn disable_controler(&self) {
        let abort_needed = ic_raw_intr_stat::rx_mst_on_hold::is_set(&self.fields).unwrap();

        let enable: u32 = 0;
        if abort_needed {
            pr_info!("====abort_needed===");
            let _ = ic_enable::abort::set(&self.fields);
            // ic_enable.modify(IC_ENABLE::ABORT.val(1));

            let ret = ic_enable::abort::read_poll_timeout(
                &self.fields, enable, !enable, 10, 100 );
            if ret != 0 {
                pr_err!(" timeout while trying to abort current transfer\n");
            }
        }
        let mut try_cnt: i32 = 100;
        loop {
            self._disable_nowait();
            delay::usleep(100);
            // check enable_status
            if !ic_enable_status::ic_en::is_set(&self.fields).unwrap() {
                break;
            }
            try_cnt -= 1;
            if try_cnt == 0 {
                pr_err!("timeout in disabling i2c adapter");
                break;
            }
        }
    }

    fn _disable_nowait(&self) {
        let _ = ic_enable::other::write(&self.fields, 0);
        let _ = ic_enable::abort::clear(&self.fields);
        let _ = ic_enable::enable::clear(&self.fields);
    }
}

#[allow(dead_code)]
#[derive(Default, Debug, Copy, Clone)]
pub(crate) struct DwI2cSclLHCnt {
    /// standard speed HCNT value
    pub(crate) ss_hcnt: u16,
    /// standard speed LCNT value
    pub(crate) ss_lcnt: u16,
    /// Fast Speed HCNT value
    pub(crate) fs_hcnt: u16,
    /// Fast Speed LCNT value
    pub(crate) fs_lcnt: u16,
    /// Fast Speed Plus HCNT value
    pub(crate) fp_hcnt: u16,
    /// Fast Speed Plus LCNT value
    pub(crate) fp_lcnt: u16,
    /// High Speed HCNT value
    pub(crate) hs_hcnt: u16,
    /// High Speed LCNT value
    pub(crate) hs_lcnt: u16,
}

impl DwI2cSclLHCnt {
    /// Conditional expression:
    ///  
    ///  IC_[FS]S_SCL_LCNT + 1 >= IC_CLK * (tLOW + tf)
    ///
    /// DW I2C core starts counting the SCL CNTs for the LOW period
    /// of the SCL clock (tLOW) as soon as it pulls the SCL line.
    /// In order to meet the tLOW timing spec, we need to take into
    /// account the fall time of SCL signal (tf).  Default tf value
    /// should be 0.3 us, for safety.
    pub(crate) fn scl_lcnt(ic_clk: u32, tlow: u32, tf: u32, offset: u32) -> u32 {
        pr_debug!(
            "scl_lcnt: ic_clk: {} , tlow:{}  tf:{} , offset:{}",
            ic_clk,
            tlow,
            tf,
            offset
        );
        let right: u64 = ic_clk as u64 * (tlow as u64 + tf as u64);
        (math::div_round_closest_ull(right, math::MICRO) - 1 + offset as u64).try_into().unwrap()
    }

    /// DesignWare I2C core doesn't seem to have solid strategy to meet
    /// the tHD;STA timing spec.  Configuring _HCNT based on tHIGH spec
    /// will result in violation of the tHD;STA spec.
    /// Conditional expression1:
    /// IC_[FS]S_SCL_HCNT + (1+4+3) >= IC_CLK * tHIGH
    /// This is based on the DW manuals, and represents an ideal
    /// configuration.  The resulting I2C bus speed will be
    /// If your hardware is free from tHD;STA issue, try this one.
    ///
    /// Conditional expression2:
    /// IC_[FS]S_SCL_HCNT + 3 >= IC_CLK * (tHD;STA + tf)
    /// This is just experimental rule; the tHD;STA period turned
    /// out to be proportinal to (_HCNT + 3).  With this setting
    /// we could meet both tHIGH and tHD;STA timing specs.
    /// If unsure, you'd better to take this alternative.
    ///
    /// The reason why we need to take into account "tf" here,
    /// is the same as described in i2c_dw_scl_lcnt().
    pub(crate) fn scl_hcnt(ic_clk: u32, tsymbol: u32, tf: u32, cond: bool, offset: u32) -> u32 {
        if cond {
            let right: u64 = ic_clk as u64 * tsymbol as u64;
            (math::div_round_closest_ull(right, math::MICRO) - 8 + offset as u64).try_into().unwrap()
        } else {
            let right: u64 = ic_clk as u64 * (tsymbol as u64 + tf as u64);
            (math::div_round_closest_ull(right, math::MICRO) - 3 + offset as u64).try_into().unwrap()
        }
    }
}

enum TransferResult  {
    // Unexpected irq
    UnExpectedInterrupt,
    // Recive IRQ abort
    Abort,
    // All msgs are process success
    Fininsh,
    // Still need next irq
    Continue,
}

/// Master driver transfer abstract
pub struct MasterXfer {
    /// XferData
    msgs: Vec<I2cMsgInfo>,
    /// run time hadware error code
    cmd_err: u32,
    /// the element index of the current rx message in the msgs array
    msg_read_idx: usize,
    /// the element index of the current tx message in the msgs array
    msg_write_idx: usize,
    /// error status of the current transfer
    msg_err: Result<()>,
    /// copy of the TX_ABRT_SOURCE register
    abort_source: u32,
    /// current master-rx elements in tx fifo
    rx_outstanding: isize,
    // Driver Status
    status: u32,
}

impl Default for MasterXfer {
    /// Create an empty XferData
    fn default() -> Self {
        Self {
            msgs: Vec::new(),
            cmd_err: 0,
            msg_read_idx: 0,
            msg_write_idx: 0,
            msg_err: Ok(()),
            abort_source: 0,
            rx_outstanding: 0,
            status: 0,
        }
    }
}

impl MasterXfer {
    fn init(&mut self, msgs: Vec<I2cMsgInfo>) {
        self.msgs = msgs;
        self.cmd_err = 0;
        self.msg_read_idx = 0;
        self.msg_write_idx = 0;
        self.msg_err = Ok(());
        self.abort_source = 0;
        self.rx_outstanding = 0;
        self.status = 0;
    }

    /// clear active
    #[inline]
    pub fn clear_active(&mut self) {
        self.status &= !msg::STATUS_ACTIVE;
    }
    
    /// set active
    #[inline]
    pub fn set_active(&mut self) {
        self.status |= msg::STATUS_ACTIVE;
    }

    /// active status
    #[inline]
    pub fn is_active(&self) -> bool {
        (self.status & msg::STATUS_ACTIVE) != 0 
    }

    /// write_in_progress status
    #[inline]
    pub fn is_write_in_progress(&self) -> bool {
        (self.status & msg::STATUS_WRITE_IN_PROGRESS) != 0 
    }

    /// set write_in_progress
    #[inline]
    pub fn set_write_in_progress(&mut self, set: bool) {
        if set {
            self.status |= msg::STATUS_WRITE_IN_PROGRESS;
        } else {
            self.status &= !msg::STATUS_WRITE_IN_PROGRESS;
        }
    }

    /// msg prepare
    pub fn xfer_prepare(
        &mut self, 
        msgs: Vec<I2cMsgInfo>, 
        master_driver: &I2cDwMasterDriver,
    ) {
        self.init(msgs);

        let core_driver = &master_driver.driver;
        let field = &core_driver.fields;

        // disable the adapter
        master_driver.disable(false);

        let first_msg = &self.msgs[self.msg_write_idx as usize];
        pr_info!("first_msg flags is {:x}", first_msg.flags() );
        if first_msg.flags() & I2cMsgFlags::I2C_ADDR_TEN != 0 {
            let _ = core_driver.enable_10bitaddr(true);
            let _ = ic_tar::ic_10bitaddr_master::set(field);
        } else {
            let _ = core_driver.enable_10bitaddr(false);
            let _ = ic_tar::ic_10bitaddr_master::clear(field);
        }
        pr_info!("first_msg addr is {:x}", first_msg.addr() );
        let _ = ic_tar::tar::write(field, first_msg.addr().into());

        // Enforce disabled interrupts (due to HW issues) 
        core_driver.disable_all_interrupt();

        // Enable the adapter
        core_driver.enable_controler();
        self.set_active();
        // Dummy read to avoid the register getting stuck on Bay Trail
        let _ = ic_enable_status::ic_en::is_set(field);
    }

    /// Interrupt service routine.
    fn irq_process(
        &mut self, 
        master_driver: &I2cDwMasterDriver,
    ) -> TransferResult {
        let core_driver = &master_driver.driver;
        let field = &core_driver.fields;
        self.abort_source = core_driver.read_and_clear_intrbits(self.rx_outstanding);

        // Unexpected interrupt in driver point of view. State
        // variables are either unset or stale so acknowledge and
        // disable interrupts for suppressing further interrupts if
        // interrupt really came from this HW (E.g. firmware has left
        // the HW active).
        assert!(self.is_active());
        if !self.is_active() {
            return TransferResult::UnExpectedInterrupt; 
        }

        if ic_intr_stat::r_tx_abrt::is_set(field).unwrap() {
            self.cmd_err |= msg::DW_IC_ERR_TX_ABRT;
            self.status &= !msg::STATUS_MASK;
            pr_err!("I2C: recieve abort irq");
            return TransferResult::Abort; 
        }

        if ic_intr_stat::r_rx_full::is_set(field).unwrap() {
            self.read_msgs(& master_driver);
        }

        if ic_intr_stat::r_tx_empty::is_set(field).unwrap() {
            self.write_msgs(& master_driver);
        }

        if  (ic_intr_stat::r_stop_det::is_set(field).unwrap() || self.msg_err.is_err()) 
            && self.rx_outstanding == 0 {
                return TransferResult::Fininsh;
        }
        return TransferResult::Continue;
    }

    fn exit(&mut self, master_driver: &I2cDwMasterDriver) -> Result<()> {
        // We must disable the adapter before returning and signaling the end
        // of the current transfer. Otherwise the hardware might continue
        // generating interrupts which in turn causes a race condition with
        // the following transfer.  Needs some more investigation if the
        // additional interrupts are a hardware bug or this driver doesn't
        // handle them correctly yet.
        master_driver.disable(true);
        self.clear_active();

        match self.msg_err {
            Err(e) => {
                pr_err!("i2c dw transfer process msg error: {:?}",e);
                return Err(e);
            }
            Ok(_) => {},
        }

        if self.cmd_err == msg::DW_IC_ERR_TX_ABRT {
            pr_err!("i2c dw transfer recv tx_abort");
            self.handle_tx_abort()?;
        }

        if self.status != 0 {
            pr_err!("transfer terminated early - interrupt latency too high?");
            return Err(EIO);
        }
        Ok(())
    }

    fn handle_tx_abort(&mut self) -> Result<()> {
        let abort_source = self.abort_source;
        if abort_source & DW_IC_TX_ABRT_NOACK != 0 {
            return Err(EIO);
        }
        if abort_source & DW_IC_TX_ARB_LOST != 0{
            return Err(EAGAIN);
        } else if abort_source & DW_IC_TX_ABRT_GCALL_READ != 0{
            return Err(EINVAL);
        } else {
            return Err(EIO);
        }
    }

    /// Initiate (and continue) low level master read/write transaction.
    /// This function is only called from i2c_dw_isr, and pumping i2c_msg
    /// messages into the tx buffer.  Even if the size of i2c_msg data is
    /// longer than the size of the tx buffer, it handles everything.
    /// Todo: need to fix intr_mask
    fn write_msgs(
        &mut self, 
        master_driver: & I2cDwMasterDriver,
    ) {
        let msg_len = self.msgs.len();
        let core_driver = &master_driver.driver;
        let field = &core_driver.fields;
        
        let mut intr_mask = DW_IC_INTR_MASTER_MASK;
        let addr = self.msgs[self.msg_write_idx].addr();
        let mut need_restart = false;
        loop {
            let write_idx = self.msg_write_idx;
            if write_idx >= msg_len {
                break;
            }

            if !self.is_write_in_progress() {
                //If both IC_EMPTYFIFO_HOLD_MASTER_EN and
                //IC_RESTART_EN are set, we must manually
                //set restart bit between messages.
                if ic_con::ic_restart_en::is_set(field).unwrap() && write_idx > 0
                {
                    need_restart = true;           
                }
            }

            let msg = &mut self.msgs[write_idx];

            if msg.addr() != addr {
                pr_err!("invalid target address\n");
                self.msg_err = Err(EINVAL);
                break;
            }

            let flr = ic_txflr::txflr::read(field).unwrap();
            let mut tx_limit = master_driver.tx_fifo_depth - flr;

            let flr = ic_rxflr::rxflr::read(field).unwrap();
            let mut rx_limit = master_driver.rx_fifo_depth - flr;

            loop {
                if msg.send_end() || rx_limit <=0 || tx_limit <=0 {
                    break;
                }
                // let mut cmd: LocalRegisterCopy<u32, IC_DATA_CMD::Register> = LocalRegisterCopy::new(0);
                // If IC_EMPTYFIFO_HOLD_MASTER_EN is set we must
                // manually set the stop bit. However, it cannot be
                // detected from the registers so we set it always
                // when writing/reading the last byte.
                //
                // i2c-core always sets the buffer length of
                // I2C_FUNC_SMBUS_BLOCK_DATA to 1. The length will
                // be adjusted when receiving the first byte.
                // Thus we can't stop the transaction here.
                if write_idx == msg_len - 1 &&
                    (msg.flags() & I2cMsgFlags::I2C_MASTER_RECV_LEN == 0) &&
                    msg.send_left_last() 
                {
                    let _ = ic_data_cmd::stop::set(field);
                }

                if need_restart {
                    let _ = ic_data_cmd::restart::set(field);
                    need_restart = false;
                }

                if msg.flags() & I2cMsgFlags::I2C_MASTER_READ != 0 {
                    /* Avoid rx buffer overrun */
                    if self.rx_outstanding >=
                         master_driver.rx_fifo_depth.try_into().unwrap() {
                        break;
                    }
                    let _ = ic_data_cmd::cmd::set(field);
                    rx_limit -= 1;
                    self.rx_outstanding += 1;
                    msg.inc_recieve_cmd_cnt();
                } else {
                    let _ = ic_data_cmd::dat::write(field, msg.pop_front_byte() as u32);
                }
                tx_limit -=1;
            }

            // Because we don't know the buffer length in the
            // I2C_FUNC_SMBUS_BLOCK_DATA case, we can't stop the
            // transaction here. Also disable the TX_EMPTY IRQ
            // while waiting for the data length byte to avoid the
            // bogus interrupts flood.
            if msg.flags() & I2cMsgFlags::I2C_MASTER_RECV_LEN !=0 {
                self.set_write_in_progress(true);
                intr_mask &= !DW_IC_INTR_TX_EMPTY;
                break;
            } else if !msg.send_end() {
                // wait next time TX_EMPTY interrupt
                self.set_write_in_progress(true);
                break;
            } else {
                self.set_write_in_progress(false);
            }
            self.msg_write_idx +=1;
        }
        
        // If i2c_msg index search is completed, we don't need TX_EMPTY
        // interrupt any more.
        if self.msg_write_idx == msg_len {
            intr_mask &= !DW_IC_INTR_TX_EMPTY;        }

        if self.msg_err.is_err() {
            intr_mask = 0;
        }
        let _ = ic_intr_mask::mask_val::write(field, intr_mask);
    }

    fn read_msgs(
        &mut self, 
        master_driver: &I2cDwMasterDriver,
    ) {
        let msg_len = self.msgs.len();
        let core_driver = & master_driver.driver;
        let field = & core_driver.fields;

        loop {
            let read_idx = self.msg_read_idx;
            if read_idx >= msg_len {
                break;
            }

            let msg = &mut(self.msgs[read_idx]);

            if msg.flags() & I2cMsgFlags::I2C_MASTER_READ == 0 {
                self.msg_read_idx += 1;
                continue
            }

            let rx_valid = ic_rxflr::rxflr::read(field).unwrap();

            for _ in 0..rx_valid {
                // check if buf can be write
                if msg.recieve_end() {
                    break;
                }

                // let mut ic_data = core_driver.ic_data_cmd().read(IC_DATA_CMD::DAT) as u8;
                let mut ic_data = ic_data_cmd::dat::read(field).unwrap() as u8;

                // Ensure length byte is a valid value
                if msg.flags() & I2cMsgFlags::I2C_MASTER_RECV_LEN !=0 {
                    // if IC_EMPTYFIFO_HOLD_MASTER_EN is set, which cannot be
                    // detected from the registers, the controller can be
                    // disabled if the STOP bit is set. But it is only set
                    // after receiving block data response length in
                    // I2C_FUNC_SMBUS_BLOCK_DATA case. That needs to read
                    // another byte with STOP bit set when the block data
                    // response length is invalid to complete the transaction.
                    if ic_data == 0 || ic_data > I2C_SMBUS_BLOCK_MAX {
                        ic_data = 1;
                    }
                    let mut buf_len = ic_data as usize;
                    // Adjust the buffer length and mask the flag 
                    // after receiving the first byte.
                    if msg.flags() & I2cMsgFlags::I2C_CLIENT_PEC != 0 {
                        buf_len+=2;
                    } else {
                        buf_len+=1;
                    };
                    msg.modify_recieve_threshold(buf_len);
                    // cacluate read_cmd_cnt
                    msg.modify_recieve_cmd_cnt(self.rx_outstanding.min(buf_len as isize));
                    msg.remove_flag(I2cMsgFlags::I2C_MASTER_RECV_LEN);
                    
                    // Received buffer length, re-enable TX_EMPTY interrupt
                    // to resume the SMBUS transaction.
                    core_driver.enable_tx_empty_intr(true);
                }
                msg.push_byte(ic_data.try_into().unwrap());
                self.rx_outstanding -= 1;
            }
            
            if !msg.recieve_end() {
                // wait next time RX_FULL interrupt
                return
            } else {
                self.msg_read_idx +=1;
            }
        }
    }

}

/// The I2cDesignware Driver
pub struct I2cDwMasterDriver {
    /// I2c Config  register set value
    // master_cfg: u32,
    /// core Driver
    driver: I2cDwCoreDriver,
    /// I2c scl_LHCNT
    lhcnt: DwI2cSclLHCnt,
    /// Fifo
    tx_fifo_depth: u32,
    rx_fifo_depth: u32,
    
    /// Arc completion 
    cmd_complete: Arc<Completion>,

    /// Since xfer will be used in interrupt handler,
    /// the data needs a concurrent mechanism to ensure safety. 
    /// The driver will ensure that it will not be triggered
    /// by interrupts when using locks,
    /// so there is no need to use spin_noirq
    xfer: Arc<SpinLock<MasterXfer>>,
}

impl I2cDwMasterDriver {
    /// Create a new I2cDesignwarDriver
    pub fn new(config: I2cDwDriverConfig, field: regmap::Fields<{ FIELD_DESCS.count() }>) -> Self {
        Self {
            // master_cfg: 0,
            driver: I2cDwCoreDriver::new(config, field),
            lhcnt: DwI2cSclLHCnt::default(),
            tx_fifo_depth: 0,
            rx_fifo_depth: 0,
            cmd_complete: Arc::pin_init(new_completion!("Completion::completion::init")).unwrap(),
            xfer: Arc::pin_init(new_spinlock!(MasterXfer::default())).unwrap(),
        }
    }

    /// Initialize the designware I2C driver config
    pub fn setup(&mut self) -> Result<()> {
        // com and speed check must be the first step
        self.driver.com_type_check()?;
        self.driver.speed_check()?;
        // init config
        self.configure_init()?;
        self.scl_lhcnt_init()?;
        self.driver.sda_hold_time_init()?;
        self.fifo_size_init();

        // Initialize the designware I2C master hardware
        self.master_setup();
        self.driver.disable_all_interrupt();
        Ok(())
    }

    /// functionality and cfg init
    fn configure_init(&mut self) -> Result<()> {
        // init functionality
        self.driver.functionality_add(I2cFuncFlags::BIT_10_ADDR);

        // init master cfg
        let _ = ic_con::master_mode::set(&self.driver.fields);
        let _ = ic_con::ic_slave_disable::set(&self.driver.fields);
        let _ = ic_con::ic_restart_en::set(&self.driver.fields);

        // On AMD pltforms BIOS advertises the bus clear feature
        // and enables the SCL/SDA stuck low. SMU FW does the
        // bus recovery process. Driver should not ignore this BIOS
        // advertisement of bus clear feature.
        if ic_con::bus_clear_feature_ctrl::is_set(&self.driver.fields).unwrap(){
            let _ = ic_con::bus_clear_feature_ctrl::set(&self.driver.fields);
        }

        let _ = self.driver.cfg_init_speed();
        Ok(())
    }

    fn master_setup(&self) {
        // Disable the adapter
        self.disable(false);
        // Write standard speed timing parameters
        self.driver.write_lhcnt(&self.lhcnt);
        // Write SDA hold time if supported
        self.driver.write_sda_hold_time();
        // Write fifo
        self.driver.write_fifo(self.tx_fifo_depth / 2, 0);
    }

    /// return  i2c functionality
    pub fn get_functionality(&self) -> u32 {
        self.driver.functionality
    }

    fn fifo_size_init(&mut self) {
        // let com_param_1 = self.driver.ic_comp_param_1();
        self.tx_fifo_depth = ic_comp_param_1::tx_buffer_depth::read(&mut self.driver.fields).unwrap() + 1;
        self.rx_fifo_depth = ic_comp_param_1::rx_buffer_depth::read(&mut self.driver.fields).unwrap() + 1;
        pr_info!(
            "I2C fifo_depth RX:TX = {}: {}",
            self.rx_fifo_depth,
            self.tx_fifo_depth
        );
    }

    fn scl_lhcnt_init(&mut self) -> Result<()> {
        let driver = &mut self.driver;
        let ic_clk = driver.ext_config.clk_rate_khz;
        let mut scl_fall_ns = driver.ext_config.timing.get_scl_fall_ns();
        let mut sda_fall_ns = driver.ext_config.timing.get_sda_fall_ns();

        // Set standard and fast speed dividers for high/low periods
        if scl_fall_ns == 0 {
            scl_fall_ns = 300;
        }

        if sda_fall_ns == 0 {
            sda_fall_ns = 300;
        }

        // tHigh = 4 us and DW default, No offset
        self.lhcnt.ss_hcnt = DwI2cSclLHCnt::scl_hcnt(ic_clk, 4000, sda_fall_ns, false, 0) as u16;
        // tLOW = 4.7 us and No offset
        self.lhcnt.ss_lcnt = DwI2cSclLHCnt::scl_lcnt(ic_clk, 4700, scl_fall_ns, 0) as u16;
        pr_info!(
            "I2C dw Standard Mode HCNT:LCNT = {} : {}",
            self.lhcnt.ss_hcnt,
            self.lhcnt.ss_lcnt
        );

        let speed_mode = driver.speed_mode;
        if speed_mode == I2cSpeedMode::FastPlusMode {
            // tHIGH = 260 ns and DW default use `false`, No offset
            self.lhcnt.fs_hcnt = DwI2cSclLHCnt::scl_hcnt(ic_clk, 260, sda_fall_ns, false, 0) as u16;
            // tLOW = 500 ns ns and No offset 
            self.lhcnt.fs_lcnt = DwI2cSclLHCnt::scl_lcnt(ic_clk, 500, scl_fall_ns, 0) as u16;
            pr_info!(
                "I2C Fast Plus Mode HCNT:LCNT = {} : {}",
                self.lhcnt.fs_hcnt,
                self.lhcnt.fs_lcnt
            );
        } else {
            // tHD;STA = tHIGH = 0.6 us and DW default use `false`, No offset
            self.lhcnt.fs_hcnt = DwI2cSclLHCnt::scl_hcnt(ic_clk, 600, sda_fall_ns, false, 0) as u16;
            // tLOW = 1.3 us and No offset
            self.lhcnt.fs_lcnt = DwI2cSclLHCnt::scl_lcnt(ic_clk, 1300, scl_fall_ns, 0) as u16;
            pr_info!(
                "I2C Fast Mode HCNT:LCNT = {} : {}",
                self.lhcnt.fs_hcnt,
                self.lhcnt.fs_lcnt
            );
        }

        // TODO: heck is high speed possible and fall back to fast mode if not
        if speed_mode == I2cSpeedMode::HighSpeedMode {
            // tHIGH = 160 ns and DW default use `false`, No offset
            self.lhcnt.hs_hcnt = DwI2cSclLHCnt::scl_hcnt(ic_clk, 160, sda_fall_ns, false, 0) as u16;
            self.lhcnt.hs_lcnt = DwI2cSclLHCnt::scl_lcnt(ic_clk, 320, scl_fall_ns, 0) as u16;
            pr_info!(
                "I2C High Speed Mode HCNT:LCNT = {} : {}",
                self.lhcnt.hs_hcnt,
                self.lhcnt.hs_lcnt
            );
        }

        Ok(())
    }

    /// Prepare controller for a transaction and call xfer_msg
    pub fn master_transfer(&self, msgs: Vec<I2cMsgInfo>) -> Result<i32> {
        let msg_num = msgs.len();
        // reinit complete
        self.cmd_complete.reinit();
        // wait bus free
        self.driver.wait_bus_not_busy()?;
        // transfer exit make sure interrupt is disabled 
        // so here lock is safety
        let mut transfer = self.xfer.lock();
        transfer.xfer_prepare(msgs, &self);
        drop(transfer);
        // Now, could enable interrupt
        self.driver.clear_all_interrupt();
        self.driver.enable_master_intr();

        // wait transfer complete
        match self.cmd_complete.wait_for_completion_timeout_sec(1) {
            Err(e) => {
                pr_err!("I2C: wait complete timeout");
                //master_setup implicitly disables the adapter
                self.master_setup();
                self.driver.clear_all_interrupt();
                self.driver.disable_all_interrupt();
                return Err(e);
            }
            Ok(_) => (),
        }

        // complete make sure interrupt is disable 
        // so here lock is safety
        let mut transfer = self.xfer.lock();
        transfer.exit(&self)?;
        Ok(msg_num.try_into().unwrap())
    }
    
    /// Interrupt service routine. This gets called whenever an I2C master interrupt
    /// occurs
    pub fn irq_handler(&self) -> irq::Return {
        let field = &self.driver.fields;

        // check raw intr stat
        if !ic_enable::enable::is_set(field).unwrap() || 
            !ic_intr_stat::r_activity::is_set(field).unwrap()
        {
            return irq::Return::None;
        }

        // master_transfer make sure when irq hanppend(irq enable)
        // no longer lock transfer, so here lock is safety
        // pr_debug!("enter irq stat: {:x}, enable: {:x}", stat, enable.get());
        let mut transfer = self.xfer.lock();
        let result = transfer.irq_process(&self);
        drop(transfer);

        match result {
            TransferResult::UnExpectedInterrupt => {
                let _ = ic_intr_mask::mask_val::write(field, 0);
            },
            TransferResult::Abort => {
                // Anytime TX_ABRT is set, the contents of the tx/rx
                // buffers are flushed. Make sure to skip them.
                let _ = ic_intr_mask::mask_val::write(field, 0);
                self.cmd_complete.complete();
            },
            TransferResult::Fininsh => {
                self.cmd_complete.complete();
            },
            TransferResult::Continue => (),
        }
        return irq::Return::Handled;
    }
    
    /// Disable 
    pub fn disable(&self, fast: bool) {
        if fast {
            self.driver._disable_nowait();
        } else {
            self.driver.disable_controler();
        }
    }
}

regmap::define_regmap_field_descs!(FIELD_DESCS, {
    (ic_con, 0x0, RW, { 
        smb_persistant_slv_addr_en => bit(19, rw),
        smb_arp_en => bit(18, rw),
        smb_slave_quick_cmd_en => bit(17, rw),
        optional_sar_ctrl => bit(16, rw),
        bus_clear_feature_ctrl => bit(11, rw),
        stop_det_if_master_active => bit(10, rw),
        rx_fifo_full_hld_ctrl => bit(9, rw),
        tx_empty_ctrl => bit(8, rw),
        stop_det_ifaddrsed => bit(7, rw),
        ic_slave_disable => bit(6, rw),
        ic_restart_en => bit(5, rw),
        ic_10bitaddr_master => bit(4, rw),
        ic_10bitaddr_slave => bit(3, rw),
        speed => raw([2:1], rw),
        master_mode => bit(0, rw),
    }),
    (ic_tar, 0x4, RW, {
        smb_quick_cmd => bit(16, rw),
        device_id => bit(13, rw),
        ic_10bitaddr_master => bit(12, rw),
        special => bit(11, rw),
        gc_or_start => bit(10, rw),
        tar => raw([9:0], rw),
    }),
    (ic_sar, 0x8, RW, {
        sar => raw([9:0], rw),
    }),
    (ic_hs_maddr, 0xC, RW, {
        ic_hs_mar => raw([2:0], rw),
    }),
    (ic_data_cmd, 0x10, RW, {
        first_data_byte => bit(11, ro),
        restart => bit(10, wo),
        stop => bit(9, wo),
        cmd => bit(8, wo),
        dat => raw([7:0], rw),
    }),
    (ic_ss_or_ufm_scl_hcnt, 0x14, RW, {
        value => raw([15:0], rw),
    }),
    (ic_ss_or_ufm_scl_lcnt, 0x18, RW, {
        value => raw([15:0], rw),
    }),
    (ic_fs_scl_hcnt_or_ufm_tbuf_cnt, 0x1C, RW, {
        value => raw([15:0], rw),  
    }),
    (ic_fs_scl_lcnt, 0x20, RW, {
        value => raw([15:0], rw),  
    }),
    (ic_hs_scl_hcnt, 0x24, RW, {
        value => raw([15:0], rw),     
    }),
    (ic_hs_scl_lcnt, 0x28, RW, {
        value => raw([15:0], rw),     
    }),
    (ic_intr_stat, 0x2C, READ, {
        r_scl_stuck_at_low => bit(14, ro),
        r_mst_on_hold => bit(13, ro),
        r_restart_det => bit(12, ro),
        r_gen_call => bit(11, ro),
        r_start_det => bit(10, ro),
        r_stop_det => bit(9, ro),
        r_activity => bit(8, ro),
        r_rx_done => bit(7, ro),
        r_tx_abrt => bit(6, ro),
        r_rd_req => bit(5, ro),
        r_tx_empty => bit(4, ro),
        r_tx_over => bit(3, ro),
        r_rx_full => bit(2, ro),
        r_rx_over => bit(1, ro),
        r_rx_under => bit(0, ro),
    }),
    (ic_intr_mask, 0x30, RW, {
        mask_val => raw([15:0], rw), 
        // m_scl_stuck_at_low => bit(14, rw),
        // m_mst_on_hold => bit(13, rw),
        // m_restart_det => bit(12, rw),
        // m_gen_call => bit(11, rw),
        // m_start_det => bit(10, rw),
        // m_stop_det => bit(9, rw),
        // m_activity => bit(8, rw),
        // m_rx_done => bit(7, rw),
        // m_tx_abrt => bit(6, rw),
        // m_rd_req => bit(5, rw),
        // m_tx_empty => bit(4, rw),
        // m_tx_over => bit(3, rw),
        // m_rx_full => bit(2, rw),
        // m_rx_over => bit(1, rw),
        // m_rx_under => bit(0, rw),
    }),
    (ic_raw_intr_stat, 0x34, READ, {
        rx_scl_stuck_at_low => bit(14, ro),
        rx_mst_on_hold => bit(13, ro),
        rx_restart_det => bit(12, ro),
        rx_gen_call => bit(11, ro),
        rx_start_det => bit(10, ro),
        rx_stop_det => bit(9, ro),
        rx_activity => bit(8, ro),
        rx_rx_done => bit(7, ro),
        rx_tx_abrt => bit(6, ro),
        rx_rd_req => bit(5, ro),
        rx_tx_empty => bit(4, ro),
        rx_tx_over => bit(3, ro),
        rx_rx_full => bit(2, ro),
        rx_rx_over => bit(1, ro),
        rx_rx_under => bit(0, ro),
    }),
    (ic_rx_tl, 0x38, RW, {
        value => raw([7:0], rw),  
    }),
    (ic_tx_tl, 0x3C, RW, {
        value => raw([7:0], rw),      
    }),
    (ic_clr_intr, 0x40, READ, {
        clr_intr => bit(0, ro),
    }),
    (ic_clr_rx_under, 0x44, READ, {
        clr_rx_under => bit(0, ro),
    }),
    (ic_clr_rx_over, 0x48, READ, {
        clr_rx_over => bit(0, ro),
    }),
    (ic_clr_tx_over, 0x4C, READ, {
        clr_tx_over => bit(0, ro),        
    }),
    (ic_clr_rd_req, 0x50, READ, {
        clr_rd_req => bit(0, ro), 
    }),
    (ic_clr_tx_abrt, 0x54, READ, {
        clr_tx_abrt => bit(0, ro), 
    }),
    (ic_clr_rx_done, 0x58, READ, {
        clr_rx_done => bit(0, ro),       
    }),
    (ic_clr_activity, 0x5C, READ, {
        clr_activity => bit(0, ro),         
    }),
    (ic_clr_stop_det, 0x60, READ, {
        clr_stop_det => bit(0, ro),  
    }),
    (ic_clr_start_det, 0x64, READ, {
        clr_start_det => bit(0, ro),  
    }),
    (ic_clr_gen_call, 0x68, READ, {
        clr_gen_call => bit(0, ro),  
    }),
    (ic_enable, 0x6C, RW, {
        other => raw([18:2], rw),
        abort => bit(1, rw),
        enable => bit(0, rw),
    }),
    (ic_status, 0x70, READ, {
        smb_alert_status => bit(20, ro),
        smb_suspend_status => bit(19, ro),
        smb_slave_addr_resolved => bit(18, ro),
        smb_slave_addr_valid => bit(17, ro),
        smb_quick_cmd_bit => bit(16, ro),
        sda_stuck_not_recovered => bit(11, ro),
        slv_hold_rx_fifo_full => bit(10, ro),
        slv_hold_tx_fifo_empty => bit(9, ro),
        mst_hold_rx_fifo_full => bit(8, ro),
        mst_hold_tx_fifo_empty => bit(7, ro),
        slv_activity => bit(6, ro),
        mst_activity => bit(5, ro),
        rff => bit(4, ro),
        rfne => bit(3, ro),
        tfe => bit(2, ro),
        tfnf => bit(1, ro),
        activity => bit(0, ro),
    }),
    (ic_txflr, 0x74, READ, {
        txflr => raw([31:0], ro),
    }),
    (ic_rxflr, 0x78, READ, {
        rxflr => raw([31:0], ro),
    }),
    (ic_sda_hold, 0x7C, RW, {
        sda_hold => raw([23:0], rw),
    }),
    (ic_tx_abrt_source, 0x80, READ, {
        abrt_source => raw([31:0], ro),
    }),
    // (ic_slv_data_nack_only, 0x84, RW, {}),
    // (ic_dma_cr, 0x88, RW, {}),
    // (ic_dma_tdrl, 0x8C, RW, {}),
    // (ic_dma_rdrl, 0x90, RW, {}),
    // (ic_sda_setup, 0x94, RW, {}),
    // (ic_ack_general_call, 0x98, RW, {}),
    (ic_enable_status, 0x9C, READ, {
        slv_rx_data_lost => bit(2, ro),
        slv_disabled_while_busy => bit(1, ro),
        ic_en => bit(0, ro),
    }),
    // (ic_fs_or_ufm_spklen, 0xA0, RW, {}),
    // (ic_hs_spklen, 0xA4, RW, {}),
    // (ic_clr_restart_det, 0xA8, READ, {}),
    // (ic_scl_stuck_at_low_timeout, 0xAC, RW, {}),
    // (ic_sda_stuck_at_low_timeout, 0xB0, RW, {}),
    // (ic_clr_scl_stuck_det, 0xB4, READ, {}),
    // (ic_device_id, 0xB8, READ, {}),
    // (ic_smbus_clock_low_sext, 0xBC, RW, {}),
    // (ic_smbus_clock_low_mext, 0xC0, RW, {}),
    // (ic_smbus_thigh_max_idle_count, 0xC4, RW, {}),
    // (ic_smbus_intr_stat, 0xC8, READ, {}),
    // (ic_smbus_intr_mask, 0xCC, RW, {}),
    // (ic_smbus_intr_raw_status, 0xD0, READ, {}),
    // (ic_clr_smbus_intr, 0xD4, WRITE, {}),
    // (ic_optional_sar, 0xD8, RW, {}),
    // (ic_smbus_udid_lsb, 0xDC, RW, {}),
    // _reserved: [u32; 5], // e0-f0
    (ic_comp_param_1, 0xF4, READ, {
        tx_buffer_depth => raw([23:16], ro),
        rx_buffer_depth => raw([15:8], ro),
        add_encoded_params => bit(7, ro),
        has_dma => bit(6, ro),
        intr_io => bit(5, ro),
        hc_count_values => bit(4, ro),
        max_speed_mode => raw([3:2], ro), 
        apb_data_width => enum([1:0], ro, {
            apb_08bits = 0x0,
            apb_16bits = 0x1,
            apb_32bits = 0x2,
        }), 
    }),
    (ic_comp_version, 0xF8, READ, {
        version => raw([31:0], ro), 
    }),
    (ic_comp_type, 0xFC, READ, {
        comp_type => raw([31:0], ro),   
    })
});
