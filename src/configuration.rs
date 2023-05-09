use crate::{
    CurrentSetting, DataRefMode, DeviceVariant, DownDeghost, LineBlankingTime, PwmFrequency,
    PwmScaleMode, UpDeghost,
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

            down_deghost: DownDeghost::None,
            up_deghost: UpDeghost::VledMinus2V,
            maximum_current: CurrentSetting::Max15mA,
            up_deghost_enable: false,
        }
    }

    // TODO: implement builder pattern
}
