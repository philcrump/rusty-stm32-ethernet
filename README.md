# Rust STM32 Ethernet

This repository is currently used for education and experimentation of STM32 Embedded Rust, using the [Embassy Rust Framework](https://embassy.dev/).

Other notable libraries used include:
* [picoserve HTTP server](https://github.com/sammhicks/picoserve)

The current hardware target is an [ST NUCLEO-H753ZI](https://www.st.com/en/evaluation-tools/nucleo-h753zi.html) development board. 
* 480MHz clock with VS0 state is used, requiring later silicon revision of V or X.

## Dependencies

### Install Rust + cargo

`curl https://sh.rustup.rs -sSf | sh`

Nightly is currently required for make_static trait macros in picoserve

```bash
rustup install nightly-2024-06-01
cargo +nightly-2024-06-01
rustup +nightly-2024-06-01 target add thumbv7em-none-eabihf
rustup default nightly
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
