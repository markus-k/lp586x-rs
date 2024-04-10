use embedded_hal::{delay::DelayNs, digital::OutputPin};
use lp586x::{configuration::ConfigBuilder, Lp586x, PwmAccess};

fn main() {
    // placeholders, replace with instances from your HAL
    let i2c_bus = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    let mut delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
    let mut vsync_pin = embedded_hal_mock::eh1::pin::Mock::new(&[]);

    let config = ConfigBuilder::new_lp5864()
        .pwm_frequency(lp586x::PwmFrequency::Pwm62_5kHz)
        .maximum_current(lp586x::CurrentSetting::Max40mA)
        .data_mode_16bit();
    let mut led_driver = Lp586x::new_with_i2c(&config, i2c_bus, 0x00, &mut delay).unwrap();
    let mut framebuffer = [0u16; 72];
    framebuffer[0] = u16::MAX;

    loop {
        // display current framebuffer
        led_driver.set_pwm(0, &framebuffer).unwrap();

        // do vsync
        vsync_pin.set_high().unwrap();
        delay.delay_us(lp586x::T_CHIP_EN_US);
        vsync_pin.set_low().unwrap();

        // roll framebuffer around to have some kind of animation
        for i in 0..framebuffer.len() {
            framebuffer[if i == 0 { framebuffer.len() - 1 } else { i - 1 }] = framebuffer[i];
        }

        // update at roughly 50Hz
        delay.delay_ms(20);
    }
}
