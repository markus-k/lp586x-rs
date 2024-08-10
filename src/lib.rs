//! Driver for the Texas Instruments LP586x LED matrix driver. Supports the LP5860,
//! LP5861, LP5862, LP5864 and LP5868 subvariants.
//!
//! Datasheet: <https://www.ti.com/lit/ds/symlink/lp5864.pdf>
//!
//! Register map: <https://www.ti.com/lit/ug/snvu786/snvu786.pdf>

#![cfg_attr(not(test), no_std)]

mod configuration;
pub mod interface;
mod register;

pub use configuration::ConfigBuilder;
use configuration::{ConfigBuilderDeviceSpecific, Configuration};
use embedded_hal::delay::DelayNs;
use interface::RegisterAccess;
use register::{BitFlags, Register};

/// Error enum for the LP586x driver
#[derive(Debug)]
pub enum Error<IE> {
    /// An interface related error has occured
    Interface(IE),

    /// Temporary buffer too small
    BufferOverrun,
}

/// Time to wait after enabling the chip (t_chip_en)
pub const T_CHIP_EN_US: u32 = 100;

pub trait ToRegisterValue<T> {
    fn register_value(&self) -> T;
}

/// Output PWM frequency setting
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmFrequency {
    /// 125 kHz
    Pwm125kHz,
    /// 62.5 kHz
    Pwm62_5kHz,
}

/// Line switch blanking time setting
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineBlankingTime {
    /// 1µs
    Blank1us,
    /// 0.5µs
    Blank0_5us,
}

/// Dimming scale setting of final PWM generator
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmScaleMode {
    /// Linear scale dimming curve
    Linear,
    /// Exponential scale dimming curve
    Exponential,
}

/// Downside deghosting level selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DownDeghost {
    None,
    Weak,
    Medium,
    Strong,
}

impl ToRegisterValue<u8> for DownDeghost {
    fn register_value(&self) -> u8 {
        match self {
            DownDeghost::None => 0,
            DownDeghost::Weak => 1,
            DownDeghost::Medium => 2,
            DownDeghost::Strong => 3,
        }
    }
}

/// Scan line clamp voltage of upside deghosting
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

impl ToRegisterValue<u8> for UpDeghost {
    fn register_value(&self) -> u8 {
        match self {
            UpDeghost::VledMinus2V => 0,
            UpDeghost::VledMinus2_5V => 1,
            UpDeghost::VledMinus3V => 2,
            UpDeghost::Gnd => 3,
        }
    }
}

/// Data refresh mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataRefMode {
    /// 8 bit PWM, update instantly, no external VSYNC
    Mode1,
    /// 8 bit PWM, update by frame, external VSYNC
    Mode2,
    /// 16 bit PWM, update by frame, external VSYNC
    Mode3,
}

impl ToRegisterValue<u8> for DataRefMode {
    fn register_value(&self) -> u8 {
        match self {
            DataRefMode::Mode1 => 0,
            DataRefMode::Mode2 => 1,
            DataRefMode::Mode3 => 3,
        }
    }
}

/// Maximum current setting
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum CurrentSettingNonT {
    Max3mA,
    Max5mA,
    Max10mA,
    #[default]
    Max15mA,
    Max20mA,
    Max30mA,
    Max40mA,
    Max50mA,
}

impl ToRegisterValue<u8> for CurrentSettingNonT {
    fn register_value(&self) -> u8 {
        match self {
            CurrentSettingNonT::Max3mA => 0,
            CurrentSettingNonT::Max5mA => 1,
            CurrentSettingNonT::Max10mA => 2,
            CurrentSettingNonT::Max15mA => 3,
            CurrentSettingNonT::Max20mA => 4,
            CurrentSettingNonT::Max30mA => 5,
            CurrentSettingNonT::Max40mA => 6,
            CurrentSettingNonT::Max50mA => 7,
        }
    }
}

/// Maximum current setting for T-devices
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum CurrentSettingT {
    Max7_5mA,
    Max12_5mA,
    Max25mA,
    #[default]
    Max37_5mA,
    Max50mA,
    Max75mA,
    Max100mA,
}

impl ToRegisterValue<u8> for CurrentSettingT {
    fn register_value(&self) -> u8 {
        match self {
            CurrentSettingT::Max7_5mA => 0,
            CurrentSettingT::Max12_5mA => 1,
            CurrentSettingT::Max25mA => 2,
            CurrentSettingT::Max37_5mA => 3,
            CurrentSettingT::Max50mA => 4,
            CurrentSettingT::Max75mA => 5,
            CurrentSettingT::Max100mA => 6,
        }
    }
}

/// Maximum current setting for LP5861T
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum CurrentSettingLP5861T {
    Max7_5mA,
    Max12_5mA,
    Max25mA,
    #[default]
    Max37_5mA,
    Max50mA,
    Max75mA,
    Max100mA,
    Max125mA,
}

impl ToRegisterValue<u8> for CurrentSettingLP5861T {
    fn register_value(&self) -> u8 {
        match self {
            CurrentSettingLP5861T::Max7_5mA => 0,
            CurrentSettingLP5861T::Max12_5mA => 1,
            CurrentSettingLP5861T::Max25mA => 2,
            CurrentSettingLP5861T::Max37_5mA => 3,
            CurrentSettingLP5861T::Max50mA => 4,
            CurrentSettingLP5861T::Max75mA => 5,
            CurrentSettingLP5861T::Max100mA => 6,
            CurrentSettingLP5861T::Max125mA => 7,
        }
    }
}

/// Fixed color groups for current sinks
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Group {
    /// CS0, CS3, CS6, CS9, CS12, CS15
    Group0,
    /// CS1, CS4, CS7, CS10, CS13, CS16
    Group1,
    /// CS2, CS5, CS8, CS11, CS14, CS17
    Group2,
}

impl Group {
    pub fn brightness_reg_addr(&self) -> u16 {
        match self {
            Group::Group0 => Register::GROUP0_BRIGHTNESS,
            Group::Group1 => Register::GROUP1_BRIGHTNESS,
            Group::Group2 => Register::GROUP2_BRIGHTNESS,
        }
    }

    pub fn current_reg_addr(&self) -> u16 {
        match self {
            Group::Group0 => Register::GROUP0_CURRENT,
            Group::Group1 => Register::GROUP1_CURRENT,
            Group::Group2 => Register::GROUP2_CURRENT,
        }
    }
}

/// Configurable group for each dot
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DotGroup {
    None,
    Group0,
    Group1,
    Group2,
}

impl ToRegisterValue<u8> for DotGroup {
    fn register_value(&self) -> u8 {
        match self {
            DotGroup::None => 0,
            DotGroup::Group0 => 0b01,
            DotGroup::Group1 => 0b10,
            DotGroup::Group2 => 0b11,
        }
    }
}

/// Holds information about global faults
#[derive(Debug)]
pub struct GlobalFaultState {
    led_open_detected: bool,
    led_short_detected: bool,
}

impl GlobalFaultState {
    pub(crate) fn from_reg_value(fault_state_value: u8) -> Self {
        GlobalFaultState {
            led_open_detected: fault_state_value & BitFlags::FAULT_STATE_GLOBAL_LOD > 0,
            led_short_detected: fault_state_value & BitFlags::FAULT_STATE_GLOBAL_LSD > 0,
        }
    }

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

    /// Current setting type associated with this device variant.
    type CurrentSetting: ToRegisterValue<u8> + Clone + core::fmt::Debug + Default;
}

macro_rules! device_variant {
    ($name:ident, $num_lines:literal, $current_setting_type:path) => {
        #[doc(hidden)]
        #[derive(Debug, Clone, Copy)]
        pub struct $name;
        impl DeviceVariant for $name {
            const NUM_LINES: u8 = $num_lines;
            type CurrentSetting = $current_setting_type;
        }
        impl seal::Sealed for $name {}
    };
}

device_variant!(Variant0, 11, CurrentSettingNonT);
device_variant!(Variant1, 1, CurrentSettingNonT);
device_variant!(Variant2, 2, CurrentSettingNonT);
device_variant!(Variant4, 4, CurrentSettingNonT);
device_variant!(Variant6, 6, CurrentSettingNonT);
device_variant!(Variant8, 8, CurrentSettingNonT);

device_variant!(Variant0T, 11, CurrentSettingT);
device_variant!(Variant1T, 1, CurrentSettingLP5861T);
device_variant!(Variant6T, 6, CurrentSettingT);
device_variant!(Variant8T, 8, CurrentSettingT);

/// Marker trait for configured data mode
pub trait DataModeMarker: seal::Sealed {}

#[doc(hidden)]
#[derive(Debug, Clone, Copy)]
pub struct DataMode8Bit;
impl DataModeMarker for DataMode8Bit {}
impl seal::Sealed for DataMode8Bit {}

#[doc(hidden)]
#[derive(Debug, Clone, Copy)]
pub struct DataMode16Bit;
impl DataModeMarker for DataMode16Bit {}
impl seal::Sealed for DataMode16Bit {}

/// Generic driver for all LP586x variants.
pub struct Lp586x<DV, I, DM> {
    interface: I,
    _phantom_data: core::marker::PhantomData<(DV, DM)>,
}

impl<DV: DeviceVariant, DM: DataModeMarker, IE, I2C> Lp586x<DV, interface::I2cInterface<I2C>, DM>
where
    I2C: embedded_hal::i2c::I2c<Error = IE>,
{
    pub fn new_with_i2c(
        config: &ConfigBuilderDeviceSpecific<DV, DM>,
        i2c: I2C,
        address: u8,
        delay: &mut D,
    ) -> Result<Lp586x<DV, interface::I2cInterface<I2C>, DM>, Error<IE>> {
        Lp586x::<DV, _, DM>::new(config, interface::I2cInterface::new(i2c, address), delay)
    }
}

impl<DV: DeviceVariant, DM: DataModeMarker, IE, SPI>
    Lp586x<DV, interface::SpiDeviceInterface<SPI>, DM>
where
    SPI: embedded_hal::spi::SpiDevice<Error = IE>,
{
    pub fn new_with_spi_device(
        config: &ConfigBuilderDeviceSpecific<DV, DM>,
        spi_device: SPI,
        delay: &mut D,
    ) -> Result<Lp586x<DV, interface::SpiDeviceInterface<SPI>, DM>, Error<IE>> {
        Lp586x::<DV, _, DM>::new(
            config,
            interface::SpiDeviceInterface::new(spi_device),
            delay,
        )
    }
}

macro_rules! fault_per_dot_fn {
    ($name:ident, $reg:expr, $doc:literal) => {
        #[doc=$doc]
        pub fn $name(&mut self, dots: &mut [bool]) -> Result<(), Error<IE>> {
            let mut buffer = [0u8; 33];

            self.interface.read_registers($reg, &mut buffer)?;

            dots[..DV::NUM_DOTS as usize]
                .iter_mut()
                .enumerate()
                .map(|(i, dot)| {
                    (
                        i / DV::NUM_CURRENT_SINKS as usize,
                        i % DV::NUM_CURRENT_SINKS as usize,
                        dot,
                    )
                })
                .for_each(|(line, cs, led_is_open)| {
                    *led_is_open = buffer[line * 3 + cs / 8] & (1 << (cs % 8)) > 0;
                });

            Ok(())
        }
    };
}

impl<DV: DeviceVariant, I, DM, IE> Lp586x<DV, I, DM>
where
    I: RegisterAccess<Error = Error<IE>>,
    DM: DataModeMarker,
{
    /// Number of current sinks of the LP586x
    pub const NUM_CURRENT_SINKS: usize = DV::NUM_CURRENT_SINKS as usize;

    /// Total number of LEDs supported by this driver
    pub const NUM_DOTS: usize = DV::NUM_DOTS as usize;

    /// Create a new LP586x driver instance with the given `config` and `interface`.
    /// A `delay` is required to meet the chip-enable timings.
    ///
    /// The returned driver has been configured and enabled.
    pub fn new(
        config: &ConfigBuilderDeviceSpecific<DV, DM>,
        interface: I,
        delay: &mut D,
    ) -> Result<Lp586x<DV, I, DM>, Error<IE>> {
        let mut driver = Lp586x {
            interface,
            _phantom_data: core::marker::PhantomData,
        };
        driver.reset()?;
        driver.chip_enable(true)?;
        delay.delay_us(T_CHIP_EN_US);
        driver.configure(&config.configuration)?;
        driver.chip_enable(false)?;
        delay.delay_us(T_CHIP_EN_US);
        driver.chip_enable(true)?;

        Ok(driver)
    }

    /// Number of lines (switches) supported by this driver
    pub const fn num_lines(&self) -> u8 {
        DV::NUM_LINES
    }

    /// Total number of dots supported by this driver
    pub const fn num_dots(&self) -> u16 {
        DV::NUM_DOTS
    }

    /// Enable or disable the chip.
    ///
    /// After enabling the chip, wait t_chip_en (100µs) for the chip to enter normal mode.
    pub fn chip_enable(&mut self, enable: bool) -> Result<(), Error<IE>> {
        self.interface.write_register(
            Register::CHIP_EN,
            if enable { BitFlags::CHIP_EN_CHIP_EN } else { 0 },
        )
    }

    pub(crate) fn configure(&mut self, configuration: &Configuration<DV>) -> Result<(), Error<IE>> {
        self.interface.write_registers(
            Register::DEV_INITIAL,
            &[
                configuration.dev_initial_reg_value(),
                configuration.dev_config1_reg_value(),
                configuration.dev_config2_reg_value(),
                configuration.dev_config3_reg_value(),
            ],
        )?;

        Ok(())
    }

    /// Resets the chip.
    pub fn reset(&mut self) -> Result<(), Error<IE>> {
        self.interface.write_register(Register::RESET, 0xff)
    }

    /// Configures dot groups, starting at dot L0-CS0. At least the first dot group has
    /// to be specified, and at most `self.num_dots()`.
    pub fn set_dot_groups(&mut self, dot_groups: &[DotGroup]) -> Result<(), Error<IE>> {
        let mut buffer = [0u8; 54];

        assert!(dot_groups.len() <= self.num_dots() as usize);
        assert!(!dot_groups.is_empty());

        dot_groups
            .iter()
            .enumerate()
            .map(|(i, dot_group)| {
                (
                    i / Self::NUM_CURRENT_SINKS,
                    i % Self::NUM_CURRENT_SINKS,
                    dot_group,
                )
            })
            .for_each(|(line, cs, dot_group)| {
                buffer[line * 5 + cs / 4] |= dot_group.register_value() << (cs % 4 * 2)
            });

        let last_group = (dot_groups.len() - 1) / Self::NUM_CURRENT_SINKS * 5
            + (dot_groups.len() - 1) % Self::NUM_CURRENT_SINKS / 4;

        self.interface
            .write_registers(Register::DOT_GROUP_SELECT_START, &buffer[..=last_group])?;

        Ok(())
    }

    /// Set dot current, starting from `start_dot`.
    pub fn set_dot_current(&mut self, start_dot: u16, current: &[u8]) -> Result<(), Error<IE>> {
        assert!(current.len() <= self.num_dots() as usize);
        assert!(!current.is_empty());

        self.interface
            .write_registers(Register::DOT_CURRENT_START + start_dot, current)?;

        Ok(())
    }

    /// Sets the global brightness across all LEDs.
    pub fn set_global_brightness(&mut self, brightness: u8) -> Result<(), Error<IE>> {
        self.interface
            .write_register(Register::GLOBAL_BRIGHTNESS, brightness)?;

        Ok(())
    }

    /// Sets the brightness across all LEDs in the given [`Group`].
    /// Note that individual LEDS/dots need to be assigned to a `LED_DOT_GROUP`
    /// for this setting to have effect. By default dots ar not assigned to any group.
    pub fn set_group_brightness(&mut self, group: Group, brightness: u8) -> Result<(), Error<IE>> {
        self.interface
            .write_register(group.brightness_reg_addr(), brightness)?;

        Ok(())
    }

    /// Set color group current scaling (0..127).
    pub fn set_color_group_current(&mut self, group: Group, current: u8) -> Result<(), Error<IE>> {
        self.interface
            .write_register(group.current_reg_addr(), current.min(0x7f))?;

        Ok(())
    }

    /// Get global fault state, indicating if any LEDs in the matrix have a
    /// open or short failure.
    pub fn get_global_fault_state(&mut self) -> Result<GlobalFaultState, Error<IE>> {
        let fault_state_value = self.interface.read_register(Register::FAULT_STATE)?;
        Ok(GlobalFaultState::from_reg_value(fault_state_value))
    }

    fault_per_dot_fn!(
        get_led_open_states,
        Register::DOT_LOD_START,
        "Get LED open states, starting from the first dot."
    );

    fault_per_dot_fn!(
        get_led_short_states,
        Register::DOT_LSD_START,
        "Get LED short states, starting from the first dot."
    );

    /// Clear all led open detection (LOD) indication bits
    pub fn clear_led_open_fault(&mut self) -> Result<(), Error<IE>> {
        self.interface.write_register(Register::LOD_CLEAR, 0xF)
    }

    /// Clear all led short detection (LSD) indication bits
    pub fn clear_led_short_fault(&mut self) -> Result<(), Error<IE>> {
        self.interface.write_register(Register::LSD_CLEAR, 0xF)
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

impl<DV: DeviceVariant, I, IE> PwmAccess<u8> for Lp586x<DV, I, DataMode8Bit>
where
    I: RegisterAccess<Error = Error<IE>>,
{
    type Error = Error<IE>;

    fn set_pwm(&mut self, start_dot: u16, values: &[u8]) -> Result<(), Self::Error> {
        if values.len() + start_dot as usize > (DV::NUM_DOTS as usize) {
            // TODO: probably we don't want to panic in an embedded system...
            panic!("Too many values supplied for given start and device variant.");
        }

        self.interface
            .write_registers(Register::PWM_BRIGHTNESS_START + start_dot, values)?;

        Ok(())
    }

    fn get_pwm(&mut self, dot: u16) -> Result<u8, Self::Error> {
        self.interface
            .read_register(Register::PWM_BRIGHTNESS_START + dot)
    }
}

impl<DV: DeviceVariant, I, IE> PwmAccess<u16> for Lp586x<DV, I, DataMode16Bit>
where
    I: RegisterAccess<Error = Error<IE>>,
{
    type Error = Error<IE>;

    fn set_pwm(&mut self, start_dot: u16, values: &[u16]) -> Result<(), Self::Error> {
        // very inefficient to "allocate" 304 bytes on the stack, but `DV::NUM_DOTS`
        // can't be used here (yet)
        let mut buffer = [0; Variant0::NUM_DOTS as usize * 2];

        if values.len() + start_dot as usize > (DV::NUM_DOTS as usize) {
            // TODO: probably we don't want to panic in an embedded system...
            panic!("Too many values supplied for given start and device variant.");
        }

        // map u16 values to a u8 buffer (little endian)
        buffer
            .chunks_exact_mut(2)
            .zip(values.iter())
            .for_each(|(dest, src)| dest.copy_from_slice(&src.to_le_bytes()));

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

impl<DV, SPID: embedded_hal::spi::SpiDevice, DM>
    Lp586x<DV, interface::SpiDeviceInterface<SPID>, DM>
{
    /// Destroys the driver and releases the owned `SpiDevice`.
    pub fn release(self) -> SPID {
        self.interface.release()
    }
}

impl<DV, I2C: embedded_hal::i2c::I2c, DM> Lp586x<DV, interface::I2cInterface<I2C>, DM> {
    /// Destorys the driver and releases the owned `I2c`-interface.
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

#[cfg(test)]
mod tests {
    use super::*;
    use interface::mock::{Access, MockInterface};

    #[test]
    fn test_create_new() {
        let interface = MockInterface::new(vec![
            Access::WriteRegister(0x0a9, 0xff),
            Access::WriteRegister(0x000, 1),
            Access::WriteRegisters(0x001, vec![0x5E, 0x00, 0x00, 0x57]),
            Access::WriteRegister(0x000, 0),
            Access::WriteRegister(0x000, 1),
        ]);
        let mut delay = embedded_hal_mock::eh1::delay::NoopDelay::new();

        let config = ConfigBuilder::new_lp5860();
        let ledmatrix = Lp586x::new(&config, interface, &mut delay).unwrap();

        ledmatrix.release().done();
    }

    #[test]
    fn test_set_dot_groups() {
        #[rustfmt::skip]
        let interface = MockInterface::new(vec![
            Access::WriteRegister(0x0a9, 0xff),
            Access::WriteRegister(0x000, 1),
            Access::WriteRegisters(0x001, vec![0x5E, 0x00, 0x00, 0x57]),
            Access::WriteRegister(0x000, 0),
            Access::WriteRegister(0x000, 1),
            Access::WriteRegisters(
                0x00c,
                vec![
                    // L0
                    0b01111001, 0b10011110, 0b11100111, 0b01111001, 0b1110,
                    // L1
                    0b01111001, 0b00111110,
                ],
            ),
            Access::WriteRegisters(
                0x00c,
                vec![0b00]
            ),
        ]);
        let mut delay = embedded_hal_mock::eh1::delay::NoopDelay::new();

        let config = ConfigBuilder::new_lp5860();
        let mut ledmatrix = Lp586x::new(&config, interface, &mut delay).unwrap();

        ledmatrix
            .set_dot_groups(&[
                // L0
                DotGroup::Group0,
                DotGroup::Group1,
                DotGroup::Group2,
                DotGroup::Group0,
                DotGroup::Group1,
                DotGroup::Group2,
                DotGroup::Group0,
                DotGroup::Group1,
                DotGroup::Group2,
                DotGroup::Group0,
                DotGroup::Group1,
                DotGroup::Group2,
                DotGroup::Group0,
                DotGroup::Group1,
                DotGroup::Group2,
                DotGroup::Group0,
                DotGroup::Group1,
                DotGroup::Group2,
                // L1
                DotGroup::Group0,
                DotGroup::Group1,
                DotGroup::Group2,
                DotGroup::Group0,
                DotGroup::Group1,
                DotGroup::Group2,
                DotGroup::Group2,
            ])
            .unwrap();

        ledmatrix.set_dot_groups(&[DotGroup::None]).unwrap();

        ledmatrix.release().done();
    }
}
