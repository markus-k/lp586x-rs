use core::marker::PhantomData;

use crate::{
    register::BitFlags, DataMode16Bit, DataMode8Bit, DataModeMarker, DataRefMode, DeviceVariant,
    DownDeghost, LineBlankingTime, PwmFrequency, PwmScaleMode, ToRegisterValue, UpDeghost,
    Variant0, Variant0T, Variant1, Variant1T, Variant2, Variant4, Variant6, Variant6T, Variant8,
    Variant8T,
};

#[derive(Debug, Clone)]
pub(crate) struct Configuration<DV: DeviceVariant> {
    // dev_initial
    pub(crate) max_line_num: u8,
    pub(crate) data_ref_mode: DataRefMode,
    pub(crate) pwm_frequency: PwmFrequency,

    // dev_config1
    pub(crate) switch_blanking_time: LineBlankingTime,
    pub(crate) pwm_scale_mode: PwmScaleMode,
    pub(crate) pwm_phase_shift: bool,
    pub(crate) cs_turn_on_delay: bool,

    // dev_config2
    pub(crate) comp_group3: u8,
    pub(crate) comp_group2: u8,
    pub(crate) comp_group1: u8,
    pub(crate) lod_removal: bool,
    pub(crate) lsd_removal: bool,

    // dev_config3
    pub(crate) down_deghost: DownDeghost,
    pub(crate) up_deghost: UpDeghost,
    pub(crate) maximum_current: DV::CurrentSetting,
    pub(crate) up_deghost_enable: bool,
}

impl<DV> Configuration<DV>
where
    DV: DeviceVariant,
{
    pub fn dev_initial_reg_value(&self) -> u8 {
        // wtf is going on here? when I remove the return [...]; there are loads
        // of syntax errors
        return match self.pwm_frequency {
            PwmFrequency::Pwm125kHz => 0,
            PwmFrequency::Pwm62_5kHz => BitFlags::DEV_INITIAL_PWM_FREQ,
        } | self.data_ref_mode.register_value()
            << BitFlags::DEV_INITIAL_DATA_REF_MODE_SHIFT
            | (self.max_line_num & BitFlags::DEV_INITIAL_MAX_LINE_NUM_MASK)
                << BitFlags::DEV_INITIAL_MAX_LINE_NUM_SHIFT;
    }

    pub fn dev_config1_reg_value(&self) -> u8 {
        self.cs_turn_on_delay
            .then_some(BitFlags::DEV_CONFIG1_CS_ON_SHIFT)
            .unwrap_or(0)
            | self
                .pwm_phase_shift
                .then_some(BitFlags::DEV_CONFIG1_PWM_PHASE_SHIFT)
                .unwrap_or(0)
            | match self.pwm_scale_mode {
                PwmScaleMode::Linear => 0,
                PwmScaleMode::Exponential => BitFlags::DEV_CONFIG1_PWM_SCALE_MODE,
            }
            | match self.switch_blanking_time {
                LineBlankingTime::Blank1us => 0,
                LineBlankingTime::Blank0_5us => BitFlags::DEV_CONFIG1_SW_BLK,
            }
    }

    pub fn dev_config2_reg_value(&self) -> u8 {
        self.lsd_removal
            .then_some(BitFlags::DEV_CONFIG2_LSD_REMOVAL)
            .unwrap_or(0)
            | self
                .lod_removal
                .then_some(BitFlags::DEV_CONFIG2_LOD_REMOVAL)
                .unwrap_or(0)
            | self.comp_group1.clamp(0, 3) << BitFlags::DEV_CONFIG2_COMP_GROUP1_SHIFT
            | self.comp_group2.clamp(0, 3) << BitFlags::DEV_CONFIG2_COMP_GROUP2_SHIFT
            | self.comp_group3.clamp(0, 3) << BitFlags::DEV_CONFIG2_COMP_GROUP3_SHIFT
    }

    pub fn dev_config3_reg_value(&self) -> u8 {
        self.up_deghost_enable
            .then_some(BitFlags::DEV_CONFIG3_UP_DEGHOST_ENABLE)
            .unwrap_or(0)
            | self.maximum_current.register_value() << BitFlags::DEV_CONFIG3_MAXIMUM_CURRENT_SHIFT
            | self.up_deghost.register_value() << BitFlags::DEV_CONFIG3_UP_DEGHOST_SHIFT
            | self.down_deghost.register_value() << BitFlags::DEV_CONFIG3_DOWN_DEGHOST_SHIFT
    }
}

pub struct ConfigBuilder {}

macro_rules! new_device_config {
    ($name:ident, $marker:path, $doc_name:literal) => {
        #[doc = concat!("Create a new configuration for the ", $doc_name, " variant.")]
        pub fn $name() -> ConfigBuilderDeviceSpecific<$marker, DataMode16Bit> {
            Self::new()
        }
    };
}

impl ConfigBuilder {
    fn new<DV: DeviceVariant>() -> ConfigBuilderDeviceSpecific<DV, DataMode16Bit> {
        ConfigBuilderDeviceSpecific {
            configuration: Configuration {
                max_line_num: DV::NUM_LINES,
                data_ref_mode: DataRefMode::Mode3,
                pwm_frequency: PwmFrequency::Pwm125kHz,

                switch_blanking_time: LineBlankingTime::Blank1us,
                pwm_scale_mode: PwmScaleMode::Linear,
                pwm_phase_shift: false,
                cs_turn_on_delay: false,

                comp_group1: 0,
                comp_group2: 0,
                comp_group3: 0,
                lod_removal: false,
                lsd_removal: false,

                down_deghost: DownDeghost::Weak,
                up_deghost: UpDeghost::VledMinus2_5V,
                maximum_current: DV::CurrentSetting::default(),
                up_deghost_enable: true,
            },
            _marker: Default::default(),
        }
    }

    new_device_config!(new_lp5860, Variant0, "LP5860");
    new_device_config!(new_lp5861, Variant1, "LP5861");
    new_device_config!(new_lp5862, Variant2, "LP5862");
    new_device_config!(new_lp5864, Variant4, "LP5864");
    new_device_config!(new_lp5866, Variant6, "LP5866");
    new_device_config!(new_lp5868, Variant8, "LP5868");

    new_device_config!(new_lp5860t, Variant0T, "LP5860T");
    new_device_config!(new_lp5861t, Variant1T, "LP5861T");
    new_device_config!(new_lp5866t, Variant6T, "LP5866T");
    new_device_config!(new_lp5868t, Variant8T, "LP5868T");
}

/// Builder for creating the device configuration.
#[derive(Debug, Clone)]
pub struct ConfigBuilderDeviceSpecific<DV: DeviceVariant, DM> {
    pub(crate) configuration: Configuration<DV>,
    _marker: PhantomData<DM>,
}

macro_rules! builder_property {
    ($field:ident, $field_type:path, $doc:literal) => {
        #[doc = $doc]
        pub fn $field(mut self, $field: $field_type) -> Self {
            self.configuration.$field = $field;
            self
        }
    };
}

impl<DV: DeviceVariant, DM: DataModeMarker> ConfigBuilderDeviceSpecific<DV, DM> {
    /// Set data mode to use 8 bit PWM
    ///
    /// # Arguments
    /// * `use_vsync`: enable vsync for display refresh
    pub fn data_mode_8bit(
        mut self,
        use_vsync: bool,
    ) -> ConfigBuilderDeviceSpecific<DV, DataMode8Bit> {
        self.configuration.data_ref_mode = if use_vsync {
            DataRefMode::Mode2
        } else {
            DataRefMode::Mode1
        };
        ConfigBuilderDeviceSpecific {
            configuration: self.configuration,
            _marker: Default::default(),
        }
    }

    /// Set data mode to 16 bit PWM. Vsync is always enabled in this mode
    pub fn data_mode_16bit(mut self) -> ConfigBuilderDeviceSpecific<DV, DataMode16Bit> {
        self.configuration.data_ref_mode = DataRefMode::Mode3;
        ConfigBuilderDeviceSpecific {
            configuration: self.configuration,
            _marker: Default::default(),
        }
    }

    builder_property!(
        max_line_num,
        u8,
        "Maximum scan line number selection. Can be greater than supported by the device"
    );
    builder_property!(pwm_frequency, PwmFrequency, "Output PWM frequency setting");

    builder_property!(
        switch_blanking_time,
        LineBlankingTime,
        "Line switch blanking time setting"
    );
    builder_property!(
        pwm_scale_mode,
        PwmScaleMode,
        "Dimming scale setting of final PWM generator"
    );
    builder_property!(pwm_phase_shift, bool, "PWM phase shift selection");
    builder_property!(cs_turn_on_delay, bool, "Current sink turn on delay setting");

    builder_property!(
        comp_group3,
        u8,
        "Low brightness compensation clock shift number setting for group3"
    );
    builder_property!(
        comp_group2,
        u8,
        "Low brightness compensation clock shift number setting for group2"
    );
    builder_property!(
        comp_group1,
        u8,
        "Low brightness compensation clock shift number setting for group1"
    );
    builder_property!(lod_removal, bool, "LOD removal function enable");
    builder_property!(lsd_removal, bool, "LSD removal function enable");

    builder_property!(
        down_deghost,
        DownDeghost,
        "Downside deghosting level selection"
    );
    builder_property!(
        up_deghost,
        UpDeghost,
        "Scan line clamp voltage of upside deghosting"
    );
    builder_property!(
        maximum_current,
        DV::CurrentSetting,
        "Maximum current setting"
    );
    builder_property!(up_deghost_enable, bool, "Current sink turn on delay enable");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_builder() {
        let builder = ConfigBuilder::new_lp5864()
            .cs_turn_on_delay(true)
            .pwm_phase_shift(true)
            .pwm_scale_mode(PwmScaleMode::Linear)
            .data_mode_8bit(false);

        assert_eq!(builder.configuration.cs_turn_on_delay, true);
        assert_eq!(builder.configuration.pwm_scale_mode, PwmScaleMode::Linear);
        assert_eq!(builder.configuration.data_ref_mode, DataRefMode::Mode1);
    }
}
