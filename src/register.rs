/// LP586x registers
///
/// Register table: <https://www.ti.com/lit/ug/snvu786/snvu786.pdf>
pub struct Register;
#[allow(dead_code)]
impl Register {
    pub const CHIP_EN: u16 = 0x000;
    pub const DEV_INITIAL: u16 = 0x001;
    pub const DEV_CONFIG1: u16 = 0x002;
    pub const DEV_CONFIG2: u16 = 0x003;
    pub const DEV_CONFIG3: u16 = 0x004;
    pub const GLOBAL_BRIGHTNESS: u16 = 0x005;
    pub const GROUP0_BRIGHTNESS: u16 = 0x006;
    pub const GROUP1_BRIGHTNESS: u16 = 0x007;
    pub const GROUP2_BRIGHTNESS: u16 = 0x008;
    pub const GROUP0_CURRENT: u16 = 0x009;
    pub const GROUP1_CURRENT: u16 = 0x00A;
    pub const GROUP2_CURRENT: u16 = 0x00B;

    pub const DOT_GROUP_SELECT_START: u16 = 0x00c;
    pub const fn dot_group_select(group: u16) -> u16 {
        Self::DOT_GROUP_SELECT_START + group
    }

    pub const DOT_ONOFF_START: u16 = 0x044;
    pub const fn dot_onoff(dot: u16) -> u16 {
        Self::DOT_ONOFF_START + dot
    }

    pub const FAULT_STATE: u16 = 0x064;

    pub const DOT_LOD_START: u16 = 0x65;
    pub const fn dot_lod(dot: u16) -> u16 {
        Self::DOT_LOD_START + dot
    }

    pub const DOT_LSD_START: u16 = 0x86;
    pub const fn dot_lsd(dot: u16) -> u16 {
        Self::DOT_LSD_START + dot
    }

    pub const LOD_CLEAR: u16 = 0x0A7;
    pub const LSD_CLEAR: u16 = 0x0A8;
    pub const RESET: u16 = 0x0A9;

    pub const DOT_CURRENT_START: u16 = 0x100;
    pub const fn dot_current(dot: u16) -> u16 {
        Self::DOT_CURRENT_START + dot
    }

    // Order of PWM registers:
    // 8-bit:
    //  - L0-CS0, L0-CS1, .., L1-CS0, ...
    // 16 bit:
    //  - [7:0] L0-CS0, [15:8] L0-CS0, [7:0] L0-CS1, [15:8] L0-CS1, ...
    pub const PWM_BRIGHTNESS_START: u16 = 0x20;
    pub const fn pwm_brightness(dot: u16) -> u16 {
        Self::PWM_BRIGHTNESS_START + dot
    }
}

/// Bitflags for registers
pub struct BitFlags;
#[allow(dead_code)]
impl BitFlags {
    pub const CHIP_EN_CHIP_EN: u8 = (1 << 0);

    pub const DEV_INITIAL_PWM_FREQ: u8 = (1 << 0);
    pub const DEV_INITIAL_DATA_REF_MODE_SHIFT: u8 = 1;
    pub const DEV_INITIAL_DATA_REF_MODE_MASK: u8 = 0b11;
    pub const DEV_INITIAL_MAX_LINE_NUM_SHIFT: u8 = 3;
    pub const DEV_INITIAL_MAX_LINE_NUM_MASK: u8 = 0b1111;

    pub const DEV_CONFIG1_CS_ON_SHIFT: u8 = (1 << 0);
    pub const DEV_CONFIG1_PWM_PHASE_SHIFT: u8 = (1 << 1);
    pub const DEV_CONFIG1_PWM_SCALE_MODE: u8 = (1 << 2);
    pub const DEV_CONFIG1_SW_BLK: u8 = (1 << 3);

    pub const DEV_CONFIG2_LSD_REMOVAL: u8 = (1 << 0);
    pub const DEV_CONFIG2_LOD_REMOVAL: u8 = (1 << 1);
    pub const DEV_CONFIG2_COMP_GROUP1_SHIFT: u8 = 2;
    pub const DEV_CONFIG2_COMP_GROUP1_MASK: u8 = 0b11;
    pub const DEV_CONFIG2_COMP_GROUP2_SHIFT: u8 = 4;
    pub const DEV_CONFIG2_COMP_GROUP2_MASK: u8 = 0b11;
    pub const DEV_CONFIG2_COMP_GROUP3_SHIFT: u8 = 6;
    pub const DEV_CONFIG2_COMP_GROUP3_MASK: u8 = 0b11;

    pub const DEV_CONFIG3_UP_DEGHOST_ENABLE: u8 = (1 << 0);
    pub const DEV_CONFIG3_MAXIMUM_CURRENT_SHIFT: u8 = 1;
    pub const DEV_CONFIG3_MAXUMUM_CURRENT_MASK: u8 = 0b111;
    pub const DEV_CONFIG3_UP_DEGHOST_SHIFT: u8 = 4;
    pub const DEV_CONFIG3_UP_DEGHOST_MASK: u8 = 0b11;
    pub const DEV_CONFIG3_DOWN_DEGHOST_SHIFT: u8 = 6;
    pub const DEV_CONFIG3_DOWN_DEGHOST_MASK: u8 = 0b11;

    pub const FAULT_STATE_GLOBAL_LSD: u8 = (1 << 0);
    pub const FAULT_STATE_GLOBAL_LOD: u8 = (1 << 1);
}
