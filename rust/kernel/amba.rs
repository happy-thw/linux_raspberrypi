// SPDX-License-Identifier: GPL-2.0

//! Amba devices and drivers.
//!
//! C header: [`include/linux/amba/bus.h`](../../../../include/linux/amba/bus.h)

use crate::{
    bindings, device, driver, 
    error::{Result, from_result, to_result},
    io_mem::Resource, 
    str::CStr,
    types::ForeignOwnable, 
    ThisModule,
};

/// A registration of an amba driver.
pub type Registration<T> = driver::Registration<Adapter<T>>;

/// An adapter for the registration of Amba drivers.
pub struct Adapter<T: Driver>(T);

impl<T: Driver> driver::DriverOps for Adapter<T> {
    type RegType = bindings::amba_driver;

    unsafe fn register(
        reg: *mut bindings::amba_driver,
        name: &'static CStr,
        module: &'static ThisModule,
    ) -> Result {
        // SAFETY: By the safety requirements of this function (defined in the trait definition),
        // `reg` is non-null and valid.
        let amba = unsafe { &mut *reg };
        amba.drv.name = name.as_char_ptr();
        amba.drv.owner = module.0;
        amba.probe = Some(Self::probe_callback);
        amba.remove = Some(Self::remove_callback);
        if let Some(t) = T::ID_TABLE {
            amba.id_table = t.as_ref();
        }
        // SAFETY: By the safety requirements of this function, `reg` is valid and fully
        // initialised.
        to_result(unsafe { bindings::amba_driver_register(reg) })
    }

    unsafe fn unregister(reg: *mut bindings::amba_driver) {
        // SAFETY: By the safety requirements of this function (defined in the trait definition),
        // `reg` was passed (and updated) by a previous successful call to `amba_driver_register`.
        unsafe { bindings::amba_driver_unregister(reg) };
    }
}

impl<T: Driver> Adapter<T> {
    unsafe extern "C" fn probe_callback(
        adev: *mut bindings::amba_device,
        aid: *const bindings::amba_id,
    ) -> core::ffi::c_int {
        from_result( || {
            // SAFETY: `adev` is valid by the contract with the C code. `dev` is alive only for the
            // duration of this call, so it is guaranteed to remain alive for the lifetime of `dev`.
            let mut dev = unsafe { Device::from_ptr(adev) };
            // SAFETY: `aid` is valid by the requirements the contract with the C code.
            let offset = unsafe { (*aid).data };
            let info = if offset.is_null() {
                None
            } else {
                // SAFETY: The offset comes from a previous call to `offset_from` in `IdArray::new`,
                // which guarantees that the resulting pointer is within the table.
                let ptr = unsafe {
                    aid.cast::<u8>()
                        .offset(offset as _)
                        .cast::<Option<T::IdInfo>>()
                };
                // SAFETY: The id table has a static lifetime, so `ptr` is guaranteed to be valid for
                // read.
                #[allow(clippy::needless_borrow)]
                unsafe { (&*ptr).as_ref() }
            };
            let data = T::probe(&mut dev, info)?;
            let ptr = T::Data::into_foreign(data);
            // SAFETY: `adev` is valid for write by the contract with the C code.
            unsafe { bindings::amba_set_drvdata(adev, ptr as _) };
            Ok(0)
        })
    }

    unsafe extern "C" fn remove_callback(adev: *mut bindings::amba_device) {
        // SAFETY: `adev` is valid by the contract with the C code.
        let ptr = unsafe { bindings::amba_get_drvdata(adev) };
        // SAFETY: The value returned by `amba_get_drvdata` was stored by a previous call to
        // `amba_set_drvdata` in `probe_callback` above; the value comes from a call to
        // `T::Data::into_foreign`.
        let data = unsafe { T::Data::from_foreign(ptr) };
        T::remove(&data);
        <T::Data as driver::DeviceRemoval>::device_remove(&data);
    }
}

/// An amba driver.
pub trait Driver {
    /// Data stored on device by driver.
    type Data: ForeignOwnable + Send + Sync + driver::DeviceRemoval = ();

    /// The type holding information about each device id supported by the driver.
    type IdInfo: 'static = ();

    /// The table of device ids supported by the driver.
    const ID_TABLE: Option<driver::IdTable<'static, DeviceId, Self::IdInfo>> = None;

    /// Probes for the device with the given id.
    fn probe(dev: &mut Device, id_info: Option<&Self::IdInfo>) -> Result<Self::Data>;

    /// Cleans any resources up that are associated with the device.
    ///
    /// This is called when the driver is detached from the device.
    fn remove(_data: &Self::Data) {}
}

/// An Amba device.
///
/// # Invariants
///
/// The field `ptr` is non-null and valid for the lifetime of the object.
pub struct Device {
    ptr: *mut bindings::amba_device,
    res: Option<Resource>,
}

impl Device {
    /// Creates a new device from the given pointer.
    ///
    /// # Safety
    ///
    /// `ptr` must be non-null and valid. It must remain valid for the lifetime of the returned
    /// instance.
    unsafe fn from_ptr(ptr: *mut bindings::amba_device) -> Self {
        // SAFETY: The safety requirements of the function ensure that `ptr` is valid.
        let dev = unsafe { &mut *ptr };
        // INVARIANT: The safety requirements of the function ensure the lifetime invariant.
        Self {
            ptr,
            res: Resource::new(dev.res.start, dev.res.end),
        }
    }

    /// Returns the io mem resource associated with the device, if there is one.
    ///
    /// Ownership of the resource is transferred to the caller, so subsequent calls to this
    /// function will return [`None`].
    pub fn take_resource(&mut self) -> Option<Resource> {
        self.res.take()
    }

    /// Returns the index-th irq associated with the device, if one exists.
    pub fn irq(&self, index: usize) -> Option<u32> {
        // SAFETY: By the type invariants, `self.ptr` is valid for read.
        let dev = unsafe { &*self.ptr };
        if index >= dev.irq.len() || dev.irq[index] == 0 {
            None
        } else {
            Some(dev.irq[index])
        }
    }

    /// Get amba revision
    pub fn revision_get(&self) -> Option<u32> {
        // SAFETY: By the type invariants, `self.ptr` is valid for read.
        let dev = unsafe { &*self.ptr };
        let rev = dev.periphid >> 24 & 0x0f;
        return Some(rev)
    }
}

// SAFETY: The device returned by `raw_device` is the raw Amba device.
unsafe impl device::RawDevice for Device {
    fn raw_device(&self) -> *mut bindings::device {
        // SAFETY: By the type invariants, we know that `self.ptr` is non-null and valid.
        unsafe { &mut (*self.ptr).dev }
    }
}

/// Declares a kernel module that exposes a single amba driver.
///
/// # Examples
///
/// ```ignore
/// # use kernel::{amba, define_amba_id_table, module_amba_driver};
/// #
/// struct MyDriver;
/// impl amba::Driver for MyDriver {
///     // [...]
/// #   fn probe(_dev: &mut amba::Device, _id: Option<&Self::IdInfo>) -> Result {
/// #       Ok(())
/// #   }
/// #   define_amba_id_table! {(), [
/// #       ({ id: 0x00041061, mask: 0x000fffff }, None),
/// #   ]}
/// }
///
/// module_amba_driver! {
///     type: MyDriver,
///     name: "module_name",
///     author: "Author name",
///     license: "GPL",
/// }
/// ```
#[macro_export]
macro_rules! module_amba_driver {
    ($($f:tt)*) => {
        $crate::module_driver!(<T>, $crate::amba::Adapter<T>, { $($f)* });
    };
}

/// Id of an Amba device.
#[derive(Clone, Copy)]
pub struct DeviceId {
    /// Device id.
    pub id: u32,

    /// Mask that identifies which bits are valid in the device id.
    pub mask: u32,
}

// SAFETY: `ZERO` is all zeroed-out and `to_rawid` stores `offset` in `amba_id::data`.
unsafe impl driver::RawDeviceId for DeviceId {
    type RawType = bindings::amba_id;
    const ZERO: Self::RawType = bindings::amba_id {
        id: 0,
        mask: 0,
        data: core::ptr::null_mut(),
    };
}

impl DeviceId {
    #[doc(hidden)]
    pub const fn to_rawid(&self, offset: isize) -> <Self as driver::RawDeviceId>::RawType {
        bindings::amba_id {
            id: self.id,
            mask: self.mask,
            data: offset as _,
        }
    }
}

/// Defines the id table for amba devices.
///
/// # Examples
///
/// ```
/// # use kernel::{amba, define_amba_id_table, driver_amba_id_table, module_amba_id_table};
/// #
/// define_of_id_table! {MY_AMDA_ID_TABLE, (), [
///     ({ id: 0x00041061, mask: 0x000fffff }, None),
/// ]};
///
/// module_of_id_table!(MOD_TABLE, MY_AMDA_ID_TABLE);
/// # struct Sample;
/// # impl kernel::amba::Driver for Sample {
/// #   kernel::driver_amba_id_table!(MY_AMDA_ID_TABLE);
/// #   fn probe(_dev: &mut amba::Device, _id: Option<&Self::IdInfo>) -> Result {
/// #       Ok(())
/// #   }
/// ```
#[macro_export]
macro_rules! define_amba_id_table {
    ($name:ident, $data_type:ty, $($t:tt)*) => {
        $crate::define_id_array!($name, $crate::amba::DeviceId, $data_type, $($t)*);
    };
}

/// Convenience macro to declare which device ID table to use for a bus driver.
#[macro_export]
macro_rules! driver_amba_id_table {
    ($name:expr) => {
        $crate::driver_id_table!(
            ID_TABLE,
            $crate::amba::DeviceId,
            Self::IdInfo,
            $name
        );
    };
}


/// Declare a device ID table as a module-level table. This creates the necessary module alias
/// entries to enable module autoloading.
#[macro_export]
macro_rules! module_amba_id_table {
    ($item_name:ident, $table_name:ident) => {
        $crate::module_id_table!($item_name, "amba", $crate::amba::DeviceId, $table_name);
    };
}