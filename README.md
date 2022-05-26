# MicroTVM ESP-IDF Template

Project API files to deploy MicroTVM Models to ESP-IDF compatible devices (ESP32 & ESP32-C3)

## Components

## Usage

### Prerequisites

First, make sure to have an up-to-date version of TVM installed, either by
- Installing a nightly build from TLCPack (See https://tlcpack.ai)
- Compiling it from soruce (See https://tvm.apache.org/docs/install/from_source.html)

If compiing from source the following options have to be set in your `config.cmake` file:
- Set `USE_MICRO` to `ON`
- Set `?` to `ON`
- Point `USE_LLVM` to the `llvm-config` path of your LLVM installation 

Additionally a supported version of the ESP-IDF Toolchain should be installed manually. Please follow the official documentation:
- Windows (untested): https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html
- Linux and macOS: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html

### Deploying MicroTVM applications to an ESP-IDF device

There are two approaches supported by the MicroTVm Project API.

#### Using the TVMC command line tool


#### Writing a custom Python script

## Configuration

The Project API templates supports the following config options:

- TODO

## Compatibility

The codebase has been tested on the following environments:

- Operating Systems: MacOS ???, Ubuntu 20.04, ???
- Python Versions: v3.8
- TVM Version: Commit ???
- ESP-IDF Version: ???