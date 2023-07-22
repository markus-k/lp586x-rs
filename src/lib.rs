//! Driver for the Texas Instruments LP586x LED matrix driver. Supports the LP5860,
//! LP5861, LP5862, LP5864 and LP5868 subvariants.
//!
//! Datasheet: <https://www.ti.com/lit/ds/symlink/lp5864.pdf>
//!
//! Register map: <https://www.ti.com/lit/ug/snvu786/snvu786.pdf>

#![cfg_attr(not(test), no_std)]

pub mod configuration;
pub mod interface;
mod register;

use configuration::Configuration;
use interface::RegisterAccess;
use register::{BitFlags, Register};

/// Error enum for the LP586x driver
#[derive(Debug)]
pub enum Error<BusE> {
    /// A bus related error has occured
    Bus(BusE),

    /// Temporary buffer too small
    BufferOverrun,
}

/// Output PWM frequency setting
#[derive(Debug)]
pub enum PwmFrequency {
    /// 125 kHz
    Pwm125kHz,
    /// 62.5 kHz
    Pwm62_5kHz,
}

/// Line switch blanking time setting
#[derive(Debug)]
pub enum LineBlankingTime {
    /// 1µs
    Blank1us,
    /// 0.5µs
    Blank0_5us,
}

/// Dimming scale setting of final PWM generator
#[derive(Debug)]
pub enum PwmScaleMode {
    /// Linear scale dimming curve
    Linear,
    /// Exponential scale dimming curve
    Exponential,
}

/// Downside deghosting level selection
#[derive(Debug)]
pub enum DownDeghost {
    None,
    Weak,
    Medium,
    Strong,
}

impl DownDeghost {
    pub const fn register_value(&self) -> u8 {
        match self {
            DownDeghost::None => 0,
            DownDeghost::Weak => 1,
            DownDeghost::Medium => 2,
            DownDeghost::Strong => 3,
        }
    }
}

/// Scan line clamp voltage of upside deghosting
#[derive(Debug)]
pub enum UpDeghost {
    /// VLED - 2V
    VledMinus2V,
    /// VLED - 2.5V
    VledMinus2_5V,
    /// VLED - 3V
    VledMinus3V,
    /// GND
    Gnd,
}

impl UpDeghost {
    pub const fn register_value(&self) -> u8 {
        match self {
            UpDeghost::VledMinus2V => 0,
            UpDeghost::VledMinus2_5V => 1,
            UpDeghost::VledMinus3V => 2,
            UpDeghost::Gnd => 3,
        }
    }
}

/// Data refresh mode selection
#[derive(Debug)]
pub enum DataRefMode {
    /// 8 bit PWM, update instantly, no external VSYNC
    Mode1,
    /// 8 bit PWM, update by frame, external VSYNC
    Mode2,
    /// 16 bit PWM, update by frame, external VSYNC
    Mode3,
}

impl DataRefMode {
    pub const fn register_value(&self) -> u8 {
        match self {
            DataRefMode::Mode1 => 0,
            DataRefMode::Mode2 => 1,
            DataRefMode::Mode3 => 2,
        }
    }
}

/// Maximum current cetting
#[derive(Debug)]
pub enum CurrentSetting {
    Max3mA,
    Max5mA,
    Max10mA,
    Max15mA,
    Max20mA,
    Max30mA,
    Max40mA,
    Max50mA,
}

impl CurrentSetting {
    pub const fn register_value(&self) -> u8 {
        match self {
            CurrentSetting::Max3mA => 0,
            CurrentSetting::Max5mA => 1,
            CurrentSetting::Max10mA => 2,
            CurrentSetting::Max15mA => 3,
            CurrentSetting::Max20mA => 4,
            CurrentSetting::Max30mA => 5,
            CurrentSetting::Max40mA => 6,
            CurrentSetting::Max50mA => 7,
        }
    }
}

/// Fixed color groups for current sinks
#[derive(Debug)]
pub enum Group {
    /// CS0, CS3, CS6, CS9, CS12, CS15
    Group0,
    /// CS1, CS4, CS7, CS10, CS13, CS16
    Group1,
    /// CS2, CS5, CS8, CS11, CS14, CS17
    Group2,
}

#[derive(Debug)]
pub struct GlobalFaultState {
    led_open_detected: bool,
    led_short_detected: bool,
}

impl GlobalFaultState {
    /// True, if any LED is detected open.
    ///
    /// LED open detection is only performed when PWM ≥ 25 (Mode 1 and Mode 2) or
    /// PWM ≥ 6400 (Mode 3) and voltage on CSn is detected lower than open threshold
    /// for continuously 4 sub-periods.
    pub fn led_open_detected(&self) -> bool {
        self.led_open_detected
    }

    /// True, if any LED is detected shorted.
    ///
    /// LED short detection only performed when PWM ≥ 25 (Mode 1 and Mode 2) or
    /// PWM ≥ 6400 (Mode 3) and voltage on CSn is detected higher than short threshold
    // for continuously 4 sub-periods.
    pub fn led_short_detected(&self) -> bool {
        self.led_short_detected
    }
}

/// Represents a safe way to address a dot in the matrix.
pub struct Dot<DV>(u16, core::marker::PhantomData<DV>);

impl<DV: DeviceVariant> Dot<DV> {
    /// Create [`Dot`] at `index`. Panics if given `index` is outside the device
    /// variants capabilites.
    pub fn with_index(index: u16) -> Self {
        if index / 18 > DV::NUM_LINES as u16 {
            panic!("Device variant does not support dot {index}");
        }

        Self(index, core::marker::PhantomData::default())
    }

    pub fn index(&self) -> u16 {
        self.0
    }

    pub fn line(&self) -> u16 {
        self.0 / 18
    }

    pub fn current_sink(&self) -> u16 {
        self.0 % 18
    }
}

mod seal {
    pub trait Sealed {}
}

/// Marker trait for a device variant.
pub trait DeviceVariant: seal::Sealed {
    /// Number of scan lines of this device variant.
    const NUM_LINES: u8;

    /// Number of current sinks of this device variant.
    const NUM_CURRENT_SINKS: u8 = 18;

    /// Total number of LED dots of this device variant.
    const NUM_DOTS: u16 = Self::NUM_LINES as u16 * Self::NUM_CURRENT_SINKS as u16;
}

#[doc(hidden)]
pub struct Variant0;
impl DeviceVariant for Variant0 {
    const NUM_LINES: u8 = 11;
}
impl seal::Sealed for Variant0 {}

#[doc(hidden)]
pub struct Variant1;
impl DeviceVariant for Variant1 {
    const NUM_LINES: u8 = 1;
}
impl seal::Sealed for Variant1 {}

#[doc(hidden)]
pub struct Variant2;
impl DeviceVariant for Variant2 {
    const NUM_LINES: u8 = 2;
}
impl seal::Sealed for Variant2 {}

#[doc(hidden)]
pub struct Variant4;
impl DeviceVariant for Variant4 {
    const NUM_LINES: u8 = 4;
}
impl seal::Sealed for Variant4 {}

#[doc(hidden)]
pub struct Variant8;
impl DeviceVariant for Variant8 {
    const NUM_LINES: u8 = 8;
}
impl seal::Sealed for Variant8 {}

pub trait DataModeMarker: seal::Sealed {}

pub struct DataModeUnconfigured;
impl DataModeMarker for DataModeUnconfigured {}
impl seal::Sealed for DataModeUnconfigured {}

pub struct DataMode8Bit;
impl DataModeMarker for DataMode8Bit {}
impl seal::Sealed for DataMode8Bit {}

pub struct DataMode16Bit;
impl DataModeMarker for DataMode16Bit {}
impl seal::Sealed for DataMode16Bit {}

/// Generic driver for all LP586x variants.
pub struct Lp586x<DV, I, DM> {
    interface: I,
    _data_mode: DM,
    _phantom_data: core::marker::PhantomData<DV>,
}

#[cfg(feature = "eh1_0")]
impl<DV: DeviceVariant, DM: DataModeMarker, BusE, D> Lp586x<DV, interface::I2cInterface<D>, DM>
where
    D: eh1_0::i2c::I2c<Error = BusE>,
{
    pub fn new_with_i2c(
        i2c: D,
        address: u8,
    ) -> Result<Lp586x<DV, interface::I2cInterface<D>, DataModeUnconfigured>, Error<BusE>> {
        Lp586x::<DV, _, DataModeUnconfigured>::new(interface::I2cInterface::new(i2c, address))
    }
}

#[cfg(feature = "eh1_0")]
impl<DV: DeviceVariant, DM: DataModeMarker, BusE, D>
    Lp586x<DV, interface::SpiDeviceInterface<D>, DM>
where
    D: eh1_0::spi::SpiDevice<Error = BusE>,
{
    pub fn new_with_spi_device(
        spi_device: D,
    ) -> Result<Lp586x<DV, interface::SpiDeviceInterface<D>, DataModeUnconfigured>, Error<BusE>>
    {
        Lp586x::<DV, _, DataModeUnconfigured>::new(interface::SpiDeviceInterface::new(spi_device))
    }
}

#[cfg(not(feature = "eh1_0"))]
mod for_eh02 {
    use super::*;

    impl<DV: DeviceVariant, DM: DataModeMarker, BusE, SPI, CS>
        Lp586x<DV, interface::SpiInterface<SPI, CS>, DM>
    where
        SPI: embedded_hal::blocking::spi::Transfer<u8, Error = BusE>
            + embedded_hal::blocking::spi::Write<u8, Error = BusE>,
        CS: embedded_hal::digital::v2::OutputPin<Error = BusE>,
    {
        pub fn new_with_spi_cs(
            spi: SPI,
            cs: CS,
        ) -> Result<Lp586x<DV, interface::SpiInterface<SPI, CS>, DataModeUnconfigured>, Error<BusE>>
        {
            Lp586x::<DV, _, DataModeUnconfigured>::new(interface::SpiInterface::new(spi, cs))
        }
    }
}

impl<DV: DeviceVariant, I, DM, BusE> Lp586x<DV, I, DM>
where
    I: RegisterAccess<Error = Error<BusE>>,
    DM: DataModeMarker,
{
    /// Number of current sinks of the LP586x
    pub const NUM_CURRENT_SINKS: usize = DV::NUM_CURRENT_SINKS as usize;

    /// Total number of LEDs supported by this driver
    pub const NUM_DOTS: usize = DV::NUM_DOTS as usize;

    /// Time to wait after enabling the chip (t_chip_en)
    pub const T_CHIP_EN_US: u32 = 100;

    /// Create a new LP586x driver instance with the given `interface`.
    ///
    /// The returned driver has the chip enabled
    pub fn new(interface: I) -> Result<Lp586x<DV, I, DataModeUnconfigured>, Error<BusE>> {
        let mut driver = Lp586x {
            interface,
            _data_mode: DataModeUnconfigured,
            _phantom_data: core::marker::PhantomData::default(),
        };
        driver.chip_enable(true)?;

        Ok(driver)
    }

    pub fn num_lines(&self) -> u8 {
        DV::NUM_LINES
    }

    pub fn num_dots(&self) -> u16 {
        DV::NUM_DOTS
    }

    /// Enable or disable the chip.
    ///
    /// After enabling the chip, wait t_chip_en (100µs) for the chip to enter normal mode.
    pub fn chip_enable(&mut self, enable: bool) -> Result<(), Error<BusE>> {
        self.interface.write_register(
            Register::CHIP_EN,
            if enable { BitFlags::CHIP_EN_CHIP_EN } else { 0 },
        )
    }

    pub fn configure(&mut self, configuration: &Configuration) -> Result<(), Error<BusE>> {
        let dev_initial = match configuration.pwm_frequency {
            PwmFrequency::Pwm62_5kHz => 0,
            PwmFrequency::Pwm125kHz => BitFlags::DEV_INITIAL_PWM_FREQ,
        } | configuration.data_ref_mode.register_value()
            << BitFlags::DEV_INITIAL_DATA_REF_MODE_SHIFT
            | (configuration.max_line_num & BitFlags::DEV_INITIAL_MAX_LINE_NUM_MASK)
                << BitFlags::DEV_INITIAL_MAX_LINE_NUM_SHIFT;

        let dev_config1 = configuration
            .cs_turn_on_delay
            .then_some(BitFlags::DEV_CONFIG1_CS_ON_SHIFT)
            .unwrap_or(0)
            | configuration
                .pwm_phase_shift
                .then_some(BitFlags::DEV_CONFIG1_PWM_PHASE_SHIFT)
                .unwrap_or(0)
            | match configuration.pwm_scale_mode {
                PwmScaleMode::Linear => 0,
                PwmScaleMode::Exponential => BitFlags::DEV_CONFIG1_PWM_SCALE_MODE,
            }
            | match configuration.switch_blanking_time {
                LineBlankingTime::Blank1us => 0,
                LineBlankingTime::Blank0_5us => BitFlags::DEV_CONFIG1_SW_BLK,
            };

        let dev_config2 = configuration
            .lsd_removal
            .then_some(BitFlags::DEV_CONFIG2_LSD_REMOVAL)
            .unwrap_or(0)
            | configuration
                .lod_removal
                .then_some(BitFlags::DEV_CONFIG2_LOD_REMOVAL)
                .unwrap_or(0)
            | configuration.comp_group1.clamp(0, 3) << BitFlags::DEV_CONFIG2_COMP_GROUP1_SHIFT
            | configuration.comp_group2.clamp(0, 3) << BitFlags::DEV_CONFIG2_COMP_GROUP2_SHIFT
            | configuration.comp_group3.clamp(0, 3) << BitFlags::DEV_CONFIG2_COMP_GROUP3_SHIFT;

        let dev_config3 = configuration
            .up_deghost_enable
            .then_some(BitFlags::DEV_CONFIG3_UP_DEGHOST_ENABLE)
            .unwrap_or(0)
            | configuration.maximum_current.register_value()
                << BitFlags::DEV_CONFIG3_MAXIMUM_CURRENT_SHIFT
            | configuration.up_deghost.register_value() << BitFlags::DEV_CONFIG3_UP_DEGHOST_SHIFT
            | configuration.down_deghost.register_value()
                << BitFlags::DEV_CONFIG3_DOWN_DEGHOST_SHIFT;

        self.interface
            .write_register(Register::DEV_INITIAL, dev_initial)?;
        self.interface
            .write_register(Register::DEV_CONFIG1, dev_config1)?;
        self.interface
            .write_register(Register::DEV_CONFIG2, dev_config2)?;
        self.interface
            .write_register(Register::DEV_CONFIG3, dev_config3)?;

        Ok(())
    }

    /// Resets the chip.
    pub fn reset(&mut self) -> Result<(), Error<BusE>> {
        self.interface.write_register(Register::RESET, 0xff)
    }

    /// Sets the global brightness across all LEDs.
    pub fn set_global_brightness(&mut self, brightness: u8) -> Result<(), Error<BusE>> {
        self.interface
            .write_register(Register::GLOBAL_BRIGHTNESS, brightness)?;

        Ok(())
    }

    /// Sets the brightness across all LEDs in the given [`Group`].
    pub fn set_group_brightness(
        &mut self,
        group: Group,
        brightness: u8,
    ) -> Result<(), Error<BusE>> {
        let register = match group {
            Group::Group0 => Register::GROUP0_BRIGHTNESS,
            Group::Group1 => Register::GROUP1_BRIGHTNESS,
            Group::Group2 => Register::GROUP2_BRIGHTNESS,
        };

        self.interface.write_register(register, brightness)?;

        Ok(())
    }

    /// Set group current scaling (0..127).
    pub fn set_group_current(&mut self, group: Group, current: u8) -> Result<(), Error<BusE>> {
        let register = match group {
            Group::Group0 => Register::GROUP0_CURRENT,
            Group::Group1 => Register::GROUP1_CURRENT,
            Group::Group2 => Register::GROUP2_CURRENT,
        };

        self.interface.write_register(register, current)?;

        Ok(())
    }

    /// Get global fault state, indicating if any LEDs in the matrix have a
    /// open or short failure.
    pub fn get_global_fault_state(&mut self) -> Result<GlobalFaultState, Error<BusE>> {
        let fault_register = self.interface.read_register(Register::FAULT_STATE)?;

        Ok(GlobalFaultState {
            led_open_detected: fault_register & BitFlags::FAULT_STATE_GLOBAL_LOD > 0,
            led_short_detected: fault_register & BitFlags::FAULT_STATE_GLOBAL_LSD > 0,
        })
    }

    pub fn into_16bit_data_mode(self) -> Result<Lp586x<DV, I, DataMode16Bit>, Error<BusE>> {
        Ok(Lp586x {
            interface: self.interface,
            _data_mode: DataMode16Bit,
            _phantom_data: core::marker::PhantomData::default(),
        })
    }

    pub fn into_8bit_data_mode(self) -> Result<Lp586x<DV, I, DataMode8Bit>, Error<BusE>> {
        Ok(Lp586x {
            interface: self.interface,
            _data_mode: DataMode8Bit,
            _phantom_data: core::marker::PhantomData::default(),
        })
    }
}

/// Trait for accessing PWM data in the correct data format.
pub trait PwmAccess<T> {
    type Error;

    /// Set PWM values of `values.len()` dots, starting from dot `start`.
    fn set_pwm(&mut self, start: u16, values: &[T]) -> Result<(), Self::Error>;

    /// Get PWM value of a single dot.
    fn get_pwm(&mut self, dot: u16) -> Result<T, Self::Error>;
}

impl<DV: DeviceVariant, I: RegisterAccess> PwmAccess<u16> for Lp586x<DV, I, DataMode16Bit> {
    type Error = I::Error;

    fn set_pwm(&mut self, start_dot: u16, values: &[u16]) -> Result<(), Self::Error> {
        let mut buffer = [0; Variant0::NUM_DOTS as usize * 2];

        if values.len() + start_dot as usize > (DV::NUM_DOTS as usize) {
            // TODO: probably we don't want to panic in an embedded system...
            panic!("Too many values supplied for given start and device variant.");
        }

        // map u16 values to a u8 buffer (little endian)
        values.iter().enumerate().for_each(|(idx, value)| {
            let register_offset = idx * 2;
            [buffer[register_offset], buffer[register_offset + 1]] = value.to_le_bytes();
        });

        self.interface.write_registers(
            Register::PWM_BRIGHTNESS_START + start_dot * 2,
            &buffer[..values.len() * 2],
        )?;

        Ok(())
    }

    fn get_pwm(&mut self, dot: u16) -> Result<u16, Self::Error> {
        self.interface
            .read_register_wide(Register::PWM_BRIGHTNESS_START + (dot * 2))
    }
}

#[cfg(feature = "eh1_0")]
impl<DV, SPID: eh1_0::spi::SpiDevice, DM> Lp586x<DV, interface::SpiDeviceInterface<SPID>, DM> {
    /// Destroys the driver and releases the owned [`SpiDevice`].
    pub fn release(self) -> SPID {
        self.interface.release()
    }
}

#[cfg(feature = "eh1_0")]
impl<DV, I2C: eh1_0::i2c::I2c, DM> Lp586x<DV, interface::I2cInterface<I2C>, DM> {
    /// Destorys the driver and releases the owned [`I2c`].
    pub fn release(self) -> I2C {
        self.interface.release()
    }
}

#[cfg(test)]
impl<DV, DM> Lp586x<DV, interface::mock::MockInterface, DM> {
    /// Destroys the drivers and returns the owned [`MockInterface`].
    pub fn release(self) -> interface::mock::MockInterface {
        self.interface
    }
}

/// LP5860 driver with 11 lines
pub type Lp5860<I> = Lp586x<Variant0, I, DataModeUnconfigured>;

/// LP5861 driver with 1 line
pub type Lp5861<I> = Lp586x<Variant1, I, DataModeUnconfigured>;

/// LP5862 driver with 2 lines
pub type Lp5862<I> = Lp586x<Variant2, I, DataModeUnconfigured>;

/// LP5864 driver with 4 lines
pub type Lp5864<I> = Lp586x<Variant4, I, DataModeUnconfigured>;

/// LP5868 driver with 8 lines
pub type Lp5868<I> = Lp586x<Variant8, I, DataModeUnconfigured>;

#[cfg(test)]
mod tests {
    use super::*;
    use interface::mock::{Access, MockInterface};

    #[test]
    fn test_create_new() {
        let interface = MockInterface::new(vec![Access::WriteRegister(0x0a9, 0xff)]);

        let mut ledmatrix = Lp5860::new(interface).unwrap();
        ledmatrix.reset().unwrap();

        ledmatrix.release().done();
    }
}
