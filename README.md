# Drone project

Drone project code

Repository code @ https://github.com/Nnubes256/drone-project

# Project structure

- `.github`: Continuous Integration workflows.
- `icaros`: The ICAROS (*In-flight Communications And Reporting Operations System*) drone-side code.
- `icaros-control`: The ICAROS Control (*In-flight Communications And Reporting Operations System*) ground-control-side code.
- `icaros-base`: ICAROS common code.
- `icaros-docs`: Generated HTML documentation for all Rust code (can be generated from source by executing `cargo doc`).
- `flightControllingAlgorithm`: Arduino flight controller code.
- `ComputerVision`: Computer vision in-flight application.
- `rf24`: Safe Rust interface to TMRh20's RF24 library.
- `rf24-sys`: Raw Rust bindings to TMRh20's RF24 library.
- `bashScript`: Supplementary scripts for Linux integration.

# Building

## Dependencies

- `sudo apt-get install build-essential gcc libclang-dev hwloc`
- Install Rust Nightly on the target platform through Rustup: https://rustup.rs/
- Build and compile WifiBroadacast using a C toolchain and Make: https://bitbucket.org/befi/wifibroadcast/.
  This should provide you with two executables: `tx` and `rx`.

## Steps

- Pull the repository if you haven't already
- Navigate to the folder you have pulled the repository on
- Initialize the git submodules
```
git submodule update --init --recursive
```

- Build the project on debug mode
```
cargo build
```

- If you want to build the project in release mode (e.g. with optimizations), then do.
```
cargo build --release
```

# Deployment

This assumes you have done the steps shown on Building on a Raspberry Pi.

## Drone

- Create two new folders on `/opt`: `computerVision` and `icaros`.
- Copy the contents of `bashScript/opt/computerVision/` and `ComputerVision/` into `/opt/computerVision`.
- Copy `icaros/start.sh` into `/opt/icaros/start.sh`.
- Copy the WifiBroadcast executable `tx` as `/opt/icaros/tx`
- Go to `target/debug` or `target/release` (whether you have built the project on debug mode or release mode)
  and copy the built executable `icaros` to `/opt/icaros/icaros`.
- Install and enable the *systemd* services on `bashScript/etc/systemd/system` into `/etc/systemd/system`.

The ICAROS system and the object detection application should now start at boot.

## Ground control

- Create a new folder on `/opt` called `icaros_control`.
- Copy the contents of `icaros-control/www` into `/opt/icaros_control/www`.
- Copy the WifiBroadcast executable `rx` as `/opt/icaros/rx`
- Go to `/opt/icaros_control`.
- Execute the program: `sudo ./icaros_control`.
