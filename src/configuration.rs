use crate::{
    register::BitFlags, CurrentSetting, DataRefMode, DeviceVariant, DownDeghost, LineBlankingTime,
    PwmFrequency, PwmScaleMode, UpDeghost,
};

#[derive(Debug)]
pub struct Configuration {
    // dev_initial
    pub max_line_num: u8,
    pub data_ref_mode: DataRefMode,
    pub pwm_frequency: PwmFrequency,

    // dev_config1
    pub switch_blanking_time: LineBlankingTime,
    pub pwm_scale_mode: PwmScaleMode,
    pub pwm_phase_shift: bool,
    pub cs_turn_on_delay: bool,

    // dev_config2
    pub comp_group3: u8,
    pub comp_group2: u8,
    pub comp_group1: u8,
    pub lod_removal: bool,
    pub lsd_removal: bool,

    // dev_config3
    pub down_deghost: DownDeghost,
    pub up_deghost: UpDeghost,
    pub maximum_current: CurrentSetting,
    pub up_deghost_enable: bool,
}

impl Configuration {
    pub fn new<DV: DeviceVariant>() -> Self {
        Self {
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
        }
    }

    // TODO: implement builder pattern

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
