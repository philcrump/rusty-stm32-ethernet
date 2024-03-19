# Rust STM32 Ethernet

This repository is currently used for education and experimentation of STM32 Embedded Rust, using the [Embassy Rust Framework](https://embassy.dev/).

Other notable libraries used include:
* [picoserve HTTP server](https://github.com/sammhicks/picoserve)

The current hardware target is an [ST NUCLEO-F767ZI](https://www.st.com/en/evaluation-tools/nucleo-f767zi.html) development board. 

## Dependencies

### Install Rust + cargo

`curl https://sh.rustup.rs -sSf | sh`

Nightly is currently required for make_static trait macros in picoserve

```bash
rustup default nightly
rustup +nightly target add thumbv7em-none-eabihf
```

### probe-rs

`sudo apt install -y libusb-1.0-0-dev libftdi1-dev libudev-dev libssl-dev`

`cargo install probe-rs --features cli`

### libdeflate

Approx. 10% better compression of HTTP assets.

`sudo apt install libdeflate-tools`

## Configuration

### `.cargo/config.toml`

Chip and Target architecture

### `Cargo.toml`

Rust library dependencies

## Tools

`./build` - Compile the application.

`./flash` - Compile and flash the application with probe-rs

`./debug` - Compile, flash, and connnect semihosting to the application with probe-rs

## Authors

* Phil Crump <phil@philcrump.co.uk>
