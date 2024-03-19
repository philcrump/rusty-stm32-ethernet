# Rust STM32 Ethernet

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

### `build`

## Build

```bash
clear && cargo build --bin eth3-pserve1 --target thumbv7em-none-eabihf --release
clear && cargo flash --bin eth3-pserve1 --target thumbv7em-none-eabihf --chip STM32F767ZITx --release
```


```bash
clear && cargo +nightly build --bin eth3-pserve1 --target thumbv7em-none-eabihf --release
clear && cargo +nightly flash --bin eth3-pserve1 --target thumbv7em-none-eabihf --chip STM32F767ZITx --release
clear && cargo +nightly run --bin eth3-pserve1 --target thumbv7em-none-eabihf --release



clear && cargo +nightly flash --bin main --target thumbv7em-none-eabihf --chip STM32F767ZITx --release
```

## Authors

* Phil Crump <phil@philcrump.co.uk>
