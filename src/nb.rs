use embedded_hal_async::delay::DelayNs;

use crate::{
    configuration::{ConfigBuilderDeviceSpecific, Configuration},
    interface,
    register::Register,
    DataMode16Bit, DataMode8Bit, DataModeMarker, DataRefMode, DeviceVariant, Error, Group,
    Variant0, Variant0T, Variant1, Variant1T, Variant2, Variant4, Variant6, Variant6T, Variant8,
    Variant8T, T_CHIP_EN_US,
};

pub struct Lp586x<DV, I, DM> {
    interface: I,
    _phantom_data: core::marker::PhantomData<(DV, DM)>,
}

impl<DV: DeviceVariant, DM: DataModeMarker, IE, SPI>
    Lp586x<DV, interface::nb::SpiDeviceInterface<SPI>, DM>
where
    SPI: embedded_hal_async::spi::SpiDevice<Error = IE>,
{
    pub async fn init_with_spi_device<D: DelayNs>(
        config: &ConfigBuilderDeviceSpecific<DV, DM>,
        spi_device: SPI,
        delay: &mut D,
    ) -> Result<Lp586x<DV, interface::nb::SpiDeviceInterface<SPI>, DM>, Error<IE>> {
        Lp586x::<DV, _, DM>::init(
            config,
            interface::nb::SpiDeviceInterface::new(spi_device),
            delay,
        )
        .await
    }
}

impl<DV: DeviceVariant, I, DM: DataModeMarker, IE> Lp586x<DV, I, DM>
where
    I: interface::nb::RegisterAccess<Error = Error<IE>>,
{
    /// Number of current sinks of the LP586x
    pub const NUM_CURRENT_SINKS: usize = DV::NUM_CURRENT_SINKS as usize;

    /// Total number of LEDs supported by this driver
    pub const NUM_DOTS: usize = DV::NUM_DOTS as usize;

    /// Number of lines (switches) supported by this driver
    pub const fn num_lines(&self) -> u8 {
        DV::NUM_LINES
    }

    /// Total number of dots supported by this driver
    pub const fn num_dots(&self) -> u16 {
        DV::NUM_DOTS
    }

    pub async fn init<D: DelayNs>(
        config: &ConfigBuilderDeviceSpecific<DV, DM>,
        interface: I,
        delay: &mut D,
    ) -> Result<Self, Error<IE>> {
        let mut this = Self {
            interface,
            _phantom_data: Default::default(),
        };

        this.reset().await?;
        this.enable(delay).await?;
        this.configure(&config.configuration).await?;

        match config.configuration.data_ref_mode {
            DataRefMode::Mode2 | DataRefMode::Mode3 => {
                // chip needs to be toggled on/off after setting max current in Mode 2 and 3
                this.disable().await?;
                this.enable(delay).await?;
            }
            DataRefMode::Mode1 => {}
        };

        Ok(this)
    }

    pub(crate) async fn configure(
        &mut self,
        configuration: &Configuration<DV>,
    ) -> Result<(), Error<IE>> {
        self.interface
            .write_registers(
                Register::DEV_INITIAL,
                &[
                    configuration.dev_initial_reg_value(),
                    configuration.dev_config1_reg_value(),
                    configuration.dev_config2_reg_value(),
                    configuration.dev_config3_reg_value(),
                ],
            )
            .await?;

        Ok(())
    }

    /// Resets the chip.
    pub async fn reset(&mut self) -> Result<(), Error<IE>> {
        self.interface.write_register(Register::RESET, 0xff).await?;

        Ok(())
    }

    /// Enables the chip and waits the required delay t_chip_en.
    pub async fn enable<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<IE>> {
        self.chip_enable(true).await?;
        delay.delay_us(T_CHIP_EN_US).await;

        Ok(())
    }

    /// Disables the chip. All data is retained.
    pub async fn disable(&mut self) -> Result<(), Error<IE>> {
        self.chip_enable(false).await?;

        Ok(())
    }

    pub async fn chip_enable(&mut self, enable: bool) -> Result<(), Error<IE>> {
        self.interface
            .write_register(Register::CHIP_EN, if enable { 0xff } else { 0x00 })
            .await?;

        Ok(())
    }

    /// Set dot current, starting from `start_dot`.
    pub async fn set_dot_current(
        &mut self,
        start_dot: u16,
        current: &[u8],
    ) -> Result<(), Error<IE>> {
        assert!(current.len() <= self.num_dots() as usize);
        assert!(!current.is_empty());

        self.interface
            .write_registers(Register::DOT_CURRENT_START + start_dot, current)
            .await?;

        Ok(())
    }

    /// Sets the brightness across all LEDs in the given [`Group`].
    /// Note that individual LEDS/dots need to be assigned to a `LED_DOT_GROUP`
    /// for this setting to have effect. By default dots ar not assigned to any group.
    pub async fn set_group_brightness(
        &mut self,
        group: Group,
        brightness: u8,
    ) -> Result<(), Error<IE>> {
        self.interface
            .write_register(group.brightness_reg_addr(), brightness)
            .await?;

        Ok(())
    }

    /// Set color group current scaling (0..127).
    pub async fn set_color_group_current(
        &mut self,
        group: Group,
        current: u8,
    ) -> Result<(), Error<IE>> {
        self.interface
            .write_register(group.current_reg_addr(), current.min(0x7f))
            .await?;

        Ok(())
    }

    /// Sets the global brightness across all LEDs.
    pub async fn set_global_brightness(&mut self, brightness: u8) -> Result<(), Error<IE>> {
        self.interface
            .write_register(Register::GLOBAL_BRIGHTNESS, brightness)
            .await?;

        Ok(())
    }
}
impl<DV, I, IE> Lp586x<DV, I, DataMode8Bit>
where
    DV: DeviceVariant,
    I: interface::nb::RegisterAccess<Error = Error<IE>>,
{
    pub async fn set_brightness(&mut self, start_dot: u16, values: &[u8]) -> Result<(), Error<IE>> {
        assert!(start_dot as usize + values.len() <= self.num_dots() as usize);

        self.interface
            .write_registers(Register::PWM_BRIGHTNESS_START + start_dot, values)
            .await?;

        Ok(())
    }
}

macro_rules! brightness_impl {
    ($variant:ident) => {
        impl<I, IE> Lp586x<$variant, I, DataMode16Bit>
        where
            I: interface::nb::RegisterAccess<Error = Error<IE>>,
        {
            pub async fn set_brightness(
                &mut self,
                start_dot: u16,
                values: &[u16],
            ) -> Result<(), Error<IE>> {
                assert!(start_dot as usize + values.len() <= self.num_dots() as usize);

                // we can't do something like [u8; N*2] with stable rust here, so we
                // have to use macro to create a specialized impl for every variant...
                let mut buffer = [0; $variant::NUM_DOTS as usize * 2];

                buffer
                    .chunks_exact_mut(2)
                    .zip(values.iter())
                    .for_each(|(dest, src)| {
                        dest.copy_from_slice(&src.to_le_bytes());
                    });

                self.interface
                    .write_registers(Register::PWM_BRIGHTNESS_START + start_dot, &buffer)
                    .await?;

                Ok(())
            }
        }
    };
}

// this is kind of stupid/crazy, but otherwise this can only be implemented
// by wasting tons of stack memory in stable rust at the moment
brightness_impl!(Variant0);
brightness_impl!(Variant1);
brightness_impl!(Variant2);
brightness_impl!(Variant4);
brightness_impl!(Variant6);
brightness_impl!(Variant8);

brightness_impl!(Variant0T);
brightness_impl!(Variant1T);
brightness_impl!(Variant6T);
brightness_impl!(Variant8T);
