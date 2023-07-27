PINBALL CONTROLLER
==================

Pinball controller made for Pinball FX.  
Supports tilting through a ICM-20948 IMU (accelerometer).

## Setup

```bash
cmake -B build -S . && cd build && make ccls
```

## Building

```bash
cd build
```

```bash
make
```

## Flashing from command line

Install [picotool](https://github.com/raspberrypi/picotool).  
The most convenient way on macOS is through homebrew:

```bash
brew install picotool
```

Now you can flash the firmware like so when the RP2040 is in boot select mode (see below):

```bash
cd build
```

```bash
make flash
```

## Manual flashing

Set the RP2040 in bootloader mode by holding the boot select button while powering on.  
The RP2040 will apear as a mass storage device on the host computer.  
Drag and drop the `uf2` file to the RP2040.
