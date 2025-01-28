# LP586x matrix LED driver
[![CI](https://github.com/markus-k/lp586x-rs/actions/workflows/rust.yml/badge.svg)](https://github.com/markus-k/lp586x-rs/actions/workflows/rust.yml)
[![crates.io page](https://img.shields.io/crates/v/lp586x.svg)](https://crates.io/crates/lp586x)
[![docs.rs page](https://docs.rs/lp586x/badge.svg)](https://docs.rs/lp586x)
![no_std](https://img.shields.io/badge/no__std-yes-blue)

LP586x matrix LED driver written in Rust.

Supported devices:
* [LP5860](https://www.ti.com/product/LP5860)
* [LP5861](https://www.ti.com/product/LP5861)
* [LP5862](https://www.ti.com/product/LP5862)
* [LP5864](https://www.ti.com/product/LP5864)
* [LP5866](https://www.ti.com/product/LP5866)
* [LP5868](https://www.ti.com/product/LP5868)
* [LP5860T](https://www.ti.com/product/LP5860T)
* [LP5861T](https://www.ti.com/product/LP5861T)
* [LP5866T](https://www.ti.com/product/LP5866T)
* [LP5868T](https://www.ti.com/product/LP5868T)

This driver is still work-in-progress and it's API may not be stable. Pull requests welcome!

## Examples

For examples, have a look at the [examples/](examples/) directory.

A very simple example:

```rust
use lp586x::{ConfigBuilder, Lp586x, PwmAccess};

let config = ConfigBuilder::new_lp5864()
    .pwm_frequency(lp586x::PwmFrequency::Pwm62_5kHz)
    .maximum_current(lp586x::CurrentSettingNonT::Max40mA)
    .data_mode_8bit(false); // disable vsync for simplicity

let mut led_driver = Lp586x::new_with_i2c(&config, i2c_bus, 0x40, &mut delay).unwrap();
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
