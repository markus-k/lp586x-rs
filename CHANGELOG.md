# ChangeLog

All notable changes for this crate are documented in this file.

## 0.2.0

* Support for T variants: LP5860T, LP5861T, LP5866T, LP5868T
* Support for LP5866
* `ConfigBuilder` is now an empty, generic-free struct that provides device-specific 
  `new_*()` methods. These return a `ConfigBuilderDeviceSpecific`.
* `LP586x::new_*()` methods now take a `impl DelayNs` instead of `&mut impl DelayNs`.
  This usually shouldn't require code changes.

## 0.1.0

Initial release
