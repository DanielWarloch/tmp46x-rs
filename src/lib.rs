//! A platform agnostic Rust driver for TMP468, based on the
//! [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.

#![no_std]
#![macro_use]
pub(crate) mod fmt;

mod error;
pub use error::{Error, Result};

#[cfg(not(any(feature = "sync", feature = "async")))]
compile_error!("You should probably choose at least one of `sync` and `async` features.");

#[cfg(feature = "sync")]
use embedded_hal::i2c::ErrorType;
#[cfg(feature = "sync")]
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::ErrorType as AsyncErrorType;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c as AsyncI2c;

/// TMP468 Possible I2C Addresses
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum TMP468Address {
    #[default]
    Ox48 = 0x48, // ADD Pin connected to GND
    Ox49 = 0x49, // ADD Pin connected to V+
    Ox4A = 0x4A, // ADD Pin connected to SDA
    Ox4B = 0x4B, // ADD Pin connected to SCL
}

const TMP468_PRODUCT_ID: u16 = 0x55;

/// Registers of the TMP468 sensor.
#[cfg(any(feature = "async", feature = "sync"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Register {
    LocalTemp = 0x00,
    RemoteTemp1 = 0x01,
    RemoteTemp2 = 0x02,
    RemoteTemp3 = 0x03,
    RemoteTemp4 = 0x04,
    RemoteTemp5 = 0x05,
    RemoteTemp6 = 0x06,
    RemoteTemp7 = 0x07,
    RemoteTemp8 = 0x08,
    SoftwareReset = 0x20,
    Therm1Status = 0x21,
    Therm2Status = 0x22,
    RemoteChannelOpenStatus = 0x23,
    Configuration = 0x30,
    ThermHysteresis = 0x38,
    LocalTempLimit1 = 0x39,
    LocalTempLimit2 = 0x3A,
    RemoteTemp1Offset = 0x40,
    RemoteTemp1Compensation = 0x41,
    RemoteTemp1Limit1 = 0x42,
    RemoteTemp1Limit2 = 0x43,
    RemoteTemp2Offset = 0x48,
    RemoteTemp2Compensation = 0x49,
    RemoteTemp2Limit1 = 0x4A,
    RemoteTemp2Limit2 = 0x4B,
    RemoteTemp3Offset = 0x50,
    RemoteTemp3Compensation = 0x51,
    RemoteTemp3Limit1 = 0x52,
    RemoteTemp3Limit2 = 0x53,
    RemoteTemp4Offset = 0x58,
    RemoteTemp4Compensation = 0x59,
    RemoteTemp4Limit1 = 0x5A,
    RemoteTemp4Limit2 = 0x5B,
    RemoteTemp5Offset = 0x60,
    RemoteTemp5Compensation = 0x61,
    RemoteTemp5Limit1 = 0x62,
    RemoteTemp5Limit2 = 0x63,
    RemoteTemp6Offset = 0x68,
    RemoteTemp6Compensation = 0x69,
    RemoteTemp6Limit1 = 0x6A,
    RemoteTemp6Limit2 = 0x6B,
    RemoteTemp7Offset = 0x70,
    RemoteTemp7Compensation = 0x71,
    RemoteTemp7Limit1 = 0x72,
    RemoteTemp7Limit2 = 0x73,
    RemoteTemp8Offset = 0x78,
    RemoteTemp8Compensation = 0x79,
    RemoteTemp8Limit1 = 0x7A,
    RemoteTemp8Limit2 = 0x7B,
    LocalTempBlockRead = 0x80,
    RemoteTemp1BlockRead = 0x81,
    RemoteTemp2BlockRead = 0x82,
    RemoteTemp3BlockRead = 0x83,
    RemoteTemp4BlockRead = 0x84,
    RemoteTemp5BlockRead = 0x85,
    RemoteTemp6BlockRead = 0x86,
    RemoteTemp7BlockRead = 0x87,
    RemoteTemp8BlockRead = 0x88,
    LockRegister = 0xC4,
    ManufacturerID = 0xFE,
    ProductID = 0xFF,
}

/// Convert ideality factor (η) to TMP468 register value for bits 15–8 of register 0x10.
pub fn eta_to_register_value(eta: f32) -> i8 {
    // TMP468 resolution is 0.000483 per LSB around base 1.008
    const BASE_ETA: f32 = 1.008;
    const LSB_STEP: f32 = 0.000483;

    // Compute signed 8-bit register value
    let val = ((BASE_ETA - eta) / LSB_STEP).round() as i16;

    // Clamp to valid i8 range
    val.clamp(-128, 127) as i8
}

/// Convert TMP468 register value (signed 8-bit from bits 15–8 of register 0x10) to ideality factor η.
pub fn register_value_to_eta(register_value: i8) -> f32 {
    const BASE_ETA: f32 = 1.008;
    const LSB_STEP: f32 = 0.000483;

    BASE_ETA - (register_value as f32 * LSB_STEP)
}

#[cfg(any(feature = "async", feature = "sync"))]
impl From<Register> for u8 {
    fn from(r: Register) -> u8 {
        r as u8
    }
}

/// Device Satuts.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Status {
    pub therm1_status: ThermStatus,
    pub therm2_status: ThermStatus,
}
/// Device Satuts.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ThermStatus {
    pub local_temp_exceeded: bool,
    pub remote_temp1_exceeded: bool,
    pub remote_temp2_exceeded: bool,
    pub remote_temp3_exceeded: bool,
    pub remote_temp4_exceeded: bool,
    pub remote_temp5_exceeded: bool,
    pub remote_temp6_exceeded: bool,
    pub remote_temp7_exceeded: bool,
    pub remote_temp8_exceeded: bool,
}

impl From<u16> for ThermStatus {
    fn from(s: u16) -> Self {
        Self {
            local_temp_exceeded: (s & (1 << 7)) != 0,
            remote_temp1_exceeded: (s & (1 << 8)) != 0,
            remote_temp2_exceeded: (s & (1 << 9)) != 0,
            remote_temp3_exceeded: (s & (1 << 10)) != 0,
            remote_temp4_exceeded: (s & (1 << 11)) != 0,
            remote_temp5_exceeded: (s & (1 << 12)) != 0,
            remote_temp6_exceeded: (s & (1 << 13)) != 0,
            remote_temp7_exceeded: (s & (1 << 14)) != 0,
            remote_temp8_exceeded: (s & (1 << 15)) != 0,
        }
    }
}

/// An TMP468 sensor on the I2C bus `I`.
///
/// The address of the sensor will be `DEFAULT_ADDRESS` from this package,
/// unless there is some kind of special address translating hardware in use.
#[maybe_async_cfg::maybe(
    sync(feature = "sync", self = "TMP468"),
    async(feature = "async", keep_self)
)]
pub struct AsyncTMP468<I> {
    i2c: I,
    address: u8,
}

#[maybe_async_cfg::maybe(
    sync(
        feature = "sync",
        self = "TMP468",
        idents(AsyncI2c(sync = "I2c"), AsyncErrorType(sync = "ErrorType"))
    ),
    async(feature = "async", keep_self)
)]
impl<I: AsyncI2c + AsyncErrorType> AsyncTMP468<I> {
    /// Check the TMP468 its Product ID.
    async fn check_id(&mut self) -> Result<&mut Self, I::Error> {
        trace!("check_id");
        match self.read_reg(Register::ProductID).await? {
            TMP468_PRODUCT_ID => Ok(self),
            _ => Err(Error::InvalidID),
        }
    }

    /// Get the curent status.
    pub async fn status(&mut self) -> Result<Status, I::Error> {
        trace!("status");
        let t1 = self.read_reg(Register::Therm1Status).await?;
        let t2 = self.read_reg(Register::Therm2Status).await?;
        debug!(
            "status: Therm1Status=0x{:02x}, Therm2Status=0x{:02x}",
            t1, t2
        );
        Ok(Status {
            therm1_status: ThermStatus::from(t1 as u16),
            therm2_status: ThermStatus::from(t2 as u16),
        })
    }

    /// read_reg read a register value.
    async fn read_reg<REG: Into<u8>>(&mut self, reg: REG) -> Result<u16, I::Error> {
        trace!("read_reg");
        let mut buf = [0x00; 2];
        let reg = reg.into();
        self.i2c
            .write_read(self.address, &[reg], &mut buf)
            .await
            .map_err(Error::I2c)?;
        info!("R @0x{:x}={:x?}", reg, buf);
        Ok(u16::from_be_bytes(buf))
    }

    /// write_reg blindly write a single register with a fixed value.
    async fn write_reg<REG: Into<u8>>(&mut self, reg: REG, value: u16) -> Result<(), I::Error> {
        trace!("write_reg");
        let reg = reg.into();
        info!("W @0x{:x}={:x}", reg, value);
        self.i2c
            .write(
                self.address,
                &[reg, (value >> 8) as u8, (value & 0xFF) as u8],
            )
            .await
            .map_err(Error::I2c)
    }

    /// Return the underlying I2C device
    pub fn release(self) -> I {
        self.i2c
    }

    /// Destroys this driver and releases the I2C bus `I`.
    pub fn destroy(self) -> Self {
        self
    }
}

#[maybe_async_cfg::maybe(
    sync(
        feature = "sync",
        self = "TMP468",
        idents(AsyncI2c(sync = "I2c"), AsyncErrorType(sync = "ErrorType"))
    ),
    async(feature = "async", keep_self)
)]
impl<I: AsyncI2c + AsyncErrorType> AsyncTMP468<I> {
    /// Initializes the TMP468 driver.
    ///
    /// This consumes the I2C bus `I`. The address will almost always
    /// be `DEFAULT_ADDRESS` from this crate.
    pub async fn with_address(i2c: I, address: u8) -> Result<Self, I::Error> {
        let tmp468 = AsyncTMP468 { i2c, address };
        trace!("new");
        // tmp468.check_id().await?;
        Ok(tmp468)
    }
    pub async fn new(i2c: I) -> Result<Self, I::Error> {
        AsyncTMP468::with_address(i2c, TMP468Address::default() as u8).await
    }

    /// Read the current local temperature value in degree Celsius.
    ///
    /// Range is -255~255.9375°C with 0.0625°C resolution
    pub async fn local_temp(&mut self) -> Result<f32, I::Error> {
        trace!("local_temp");
        let val = self.read_reg(Register::LocalTemp).await?;
        Ok(val as f32 * 0.0625)
    }

    /// Read the current remote temperature value in degree Celsius.
    ///
    /// Range is -255~255.9375°C with 0.0625°C resolution
    /// Channel is 0~8.
    /// Channel 0 is the local temperature.
    pub async fn remote_temp(&mut self, channel: u8) -> Result<f32, I::Error> {
        trace!("remote_temp");
        if !(0..=8).contains(&channel) {
            return Err(Error::InvalidValue);
        }
        let val = self.read_reg(channel).await?;
        Ok(val as f32 * 0.0625)
    }

    /// Read all temperature values in degree Celsius.
    pub async fn all_temps(&mut self) -> Result<[f32; 9], I::Error> {
        trace!("all_temps");
        let mut temps = [0; 18]; // 9 temperatures, each 2 bytes
        self.i2c
            .write_read(
                self.address,
                &[Register::LocalTempBlockRead as u8],
                &mut temps,
            )
            .await
            .map_err(Error::I2c)?;
        debug!("all_temps: {:?}", temps);
        let mut parsed_temps = [0.0; 9];
        for (i, raw) in temps.chunks(2).enumerate() {
            let temp = i16::from_be_bytes([raw[0], raw[1]]) >> 3;
            parsed_temps[i] = temp as f32 * 0.0625;
        }
        Ok(parsed_temps)
    }
    pub async fn set_n_factor(&mut self, channel: u8, factor: f32) -> Result<(), I::Error> {
        trace!("calibrate_remote_temp");
        if !(1..=8).contains(&channel) {
            return Err(Error::InvalidValue);
        }
        let reg_offset = Register::RemoteTemp1Compensation as u8 + (channel - 1) * 8;
        debug!(
            "calibrate_remote_temp: channel={}, factor={}",
            channel, factor
        );
        self.write_reg(
            reg_offset,
            (eta_to_register_value(factor) as u16).swap_bytes(),
        )
        .await
        .map_err(Error::I2c);
        Ok(())
    }
    pub async fn read_n_factor(&mut self, channel: u8) -> Result<f32, I::Error> {
        trace!("read_n_factor");
        if !(1..=8).contains(&channel) {
            return Err(Error::InvalidValue);
        }
        let reg_offset = Register::RemoteTemp1Compensation as u8 + (channel - 1) * 8;
        let resp = self.read_reg(reg_offset).await.map_err(Error::I2c).unwrap();
        let factor = register_value_to_eta((resp >> 8) as i8);
        debug!("read_n_factor: channel={}, factor={}", channel, factor);

        Ok(factor)
    }
    pub async fn set_temp_offset(&mut self, channel: u8, offset_c: f32) -> Result<(), I::Error> {
        trace!("calibrate_remote_temp");
        if !(1..=8).contains(&channel) {
            return Err(Error::InvalidValue);
        }
        let reg_offset = Register::RemoteTemp1Offset as u8 + (channel - 1) * 8;

        if offset_c < -128.0 || offset_c > 127.9375 {
            error!("Offset out of range");
            return Err(Error::InvalidValue);
        }

        // Convert °C to register steps (0.0625 °C per LSB)
        let mut raw = (offset_c / 0.0625).round() as i16;

        // Ensure proper two's complement for 12 bits
        if raw < -2048 || raw > 2047 {
            error!("Offset out of 12-bit range");
            return Err(Error::InvalidValue);
        }
        // info!("Raw offset value: {}", raw);
        // info!("Binary representation: {:b}", raw);
        // if raw < 0 {
        //     raw = (1 << 12) + raw; // two's complement
        // }
        // info!("Two's complement raw value: {}", raw);
        //
        // // Align to upper 12 bits (lower 3 bits always zero)
        // let reg_value: u16 = (raw as u16) << 3;
        // info!("Register value to write: 0x{:04X}", reg_value);
        //
        let reg_value: u16 = (raw << 3) as u16;
        info!("Register value to write: 0x{:04X}", reg_value);
        info!(
            "set_temp_offset: channel={}, offset_c={}",
            channel, offset_c
        );
        self.write_reg(reg_offset, reg_value)
            .await
            .map_err(Error::I2c);
        Ok(())
    }
    pub async fn read_temp_offset(&mut self, channel: u8) -> Result<f32, I::Error> {
        trace!("read_n_factor");
        if !(1..=8).contains(&channel) {
            return Err(Error::InvalidValue);
        }
        let reg_offset = Register::RemoteTemp1Offset as u8 + (channel - 1) * 8;
        let resp = self.read_reg(reg_offset).await.map_err(Error::I2c).unwrap();
        // Extract the 12-bit signed value (upper 12 bits)
        let raw = (resp >> 3);
        let offset = if raw & 0x800 != 0 {
            // Negative value in two's complement
            -(((!raw & 0xFFF) + 1) as f32 * 0.0625)
        } else {
            // Positive value
            (raw as f32) * 0.0625
        };
        debug!("read_temp_offset: channel={}, offset_c={}", channel, offset);

        Ok(offset)
    }

    pub async fn set_lock(&mut self, locked: bool) -> Result<(), I::Error> {
        let value = if locked { 0x5CA6 } else { 0xEB19 };
        self.write_reg(Register::LockRegister, value)
            .await
            .map_err(Error::I2c)
            .unwrap();
        Ok(())
    }
    pub async fn read_lock(&mut self) -> Result<bool, I::Error> {
        let resp = self
            .read_reg(Register::LockRegister)
            .await
            .map_err(Error::I2c)
            .unwrap();
        if resp == 0x8000 {
            return Ok(true);
        } else if resp == 0x0000 {
            return Ok(false);
        }
        Err(Error::InvalidValue)
    }
}

#[cfg(test)]
mod test {
    // extern crate alloc;
    extern crate std;

    use super::*;
    use embedded_hal_mock::{common::Generic, eh1::i2c};
    use std::vec;

    #[test]
    fn new_tmp468() {
        let expectations = [i2c::Transaction::write_read(
            TMP468Address::Ox48 as u8,
            vec![Register::ProductID as u8],
            vec![TMP468_PRODUCT_ID as u8, (TMP468_PRODUCT_ID >> 8) as u8],
        )];
        let mock = i2c::Mock::new(&expectations);
        let tmp468: TMP468<Generic<embedded_hal_mock::eh1::i2c::Transaction>> =
            TMP468::new(mock).unwrap();

        let mut mock = tmp468.release();
        mock.done();
    }
}
