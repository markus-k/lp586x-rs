# LP586x matrix LED driver
[![CI](https://github.com/markus-k/lp586x-rs/actions/workflows/rust.yml/badge.svg)](https://github.com/markus-k/lp586x-rs/actions/workflows/rust.yml)

LP586x matrix LED driver written in Rust (supports Texas Instruments
[LP5860](https://www.ti.com/product/LP5860), LP5861, LP5862, LP5864, 
LP5868).

This driver is still work-in-progress and not stable. Pull requests welcome!

## Examples

For examples, have a look at the [examples/](examples/) directory.

A very simple example:

```rust
use lp586x::{ConfigBuilder, Lp586x, PwmAccess};

let config = ConfigBuilder::new_lp5864()
    .pwm_frequency(lp586x::PwmFrequency::Pwm62_5kHz)
    .maximum_current(lp586x::CurrentSetting::Max40mA)
    .data_mode_8bit(false); // disable vsync for simplicity

let mut led_driver = Lp586x::new_with_i2c(&config, i2c_bus, 0x00, &mut delay).unwrap();
let mut framebuffer = [0u8; 72];
framebuffer[0] = u8::MAX;

// display current framebuffer
led_driver.set_pwm(0, &framebuffer).unwrap();
```

## License
Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
