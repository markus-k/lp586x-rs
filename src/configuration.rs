use core::marker::PhantomData;

use crate::{
    register::BitFlags, CurrentSetting, DataMode16Bit, DataMode8Bit, DataModeMarker, DataRefMode,
    DeviceVariant, DownDeghost, LineBlankingTime, PwmFrequency, PwmScaleMode, UpDeghost, Variant0,
    Variant1, Variant2, Variant4, Variant8,
};

#[derive(Debug, Clone)]
pub(crate) struct Configuration {
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
    pub(crate) maximum_current: CurrentSetting,
    pub(crate) up_deghost_enable: bool,
}

impl Configuration {
    pub fn dev_initial_reg_value(&self) -> u8 {
        // wtf is going on here? when I remove the return [...]; there are loads
        // of syntax errors
        return match self.pwm_frequency {
            PwmFrequency::Pwm62_5kHz => 0,
            PwmFrequency::Pwm125kHz => BitFlags::DEV_INITIAL_PWM_FREQ,
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

/// Just a helper marker to create a new [`ConfigBuilder`]
#[doc(hidden)]
pub struct VariantUnspecified;

#[derive(Debug, Clone)]
pub struct ConfigBuilder<DV, DM> {
    pub(crate) configuration: Configuration,
    _marker: PhantomData<(DV, DM)>,
}

macro_rules! new_device_config {
    ($name:ident, $marker:ident) => {
        pub fn $name() -> ConfigBuilder<$marker, DataMode16Bit> {
            Self::new()
        }
    };
}

impl ConfigBuilder<VariantUnspecified, DataMode16Bit> {
    fn new<DV: DeviceVariant>() -> ConfigBuilder<DV, DataMode16Bit> {
        ConfigBuilder {
            configuration: Configuration {
                max_line_num: DV::NUM_LINES,
                data_ref_mode: DataRefMode::Mode3,
                pwm_frequency: PwmFrequency::Pwm62_5kHz,

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
                maximum_current: CurrentSetting::Max15mA,
                up_deghost_enable: true,
            },
            _marker: Default::default(),
        }
    }

    new_device_config!(new_lp5860, Variant0);
    new_device_config!(new_lp5861, Variant1);
    new_device_config!(new_lp5862, Variant2);
    new_device_config!(new_lp5864, Variant4);
    new_device_config!(new_lp5868, Variant8);
}

macro_rules! builder_property {
    ($field:ident, $field_type:ident) => {
        pub fn $field(mut self, $field: $field_type) -> Self {
            self.configuration.$field = $field;
            self
        }
    };
}

impl<DV: DeviceVariant, DM: DataModeMarker> ConfigBuilder<DV, DM> {
    pub fn data_mode_8bit(mut self, use_vsync: bool) -> ConfigBuilder<DV, DataMode8Bit> {
        self.configuration.data_ref_mode = if use_vsync {
            DataRefMode::Mode2
        } else {
            DataRefMode::Mode1
        };
        ConfigBuilder {
            configuration: self.configuration,
            _marker: Default::default(),
        }
    }

    pub fn data_mode_16bit(mut self) -> ConfigBuilder<DV, DataMode16Bit> {
        self.configuration.data_ref_mode = DataRefMode::Mode3;
        ConfigBuilder {
            configuration: self.configuration,
            _marker: Default::default(),
        }
    }

    builder_property!(pwm_frequency, PwmFrequency);

    builder_property!(switch_blanking_time, LineBlankingTime);
    builder_property!(pwm_scale_mode, PwmScaleMode);
    builder_property!(pwm_phase_shift, bool);
    builder_property!(cs_turn_on_delay, bool);

    builder_property!(comp_group3, u8);
    builder_property!(comp_group2, u8);
    builder_property!(comp_group1, u8);
    builder_property!(lod_removal, bool);
    builder_property!(lsd_removal, bool);

    builder_property!(down_deghost, DownDeghost);
    builder_property!(up_deghost, UpDeghost);
    builder_property!(maximum_current, CurrentSetting);
    builder_property!(up_deghost_enable, bool);
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
