# Multi-Protocol Hardware Bridge Library (CH347T & Beyond)

A modern, high-performance C++ library designed to interface with hardware protocols like **SPI** and **I2C** using USB bridge chips on **Linux**. 

The library leverages **`libusb-1.0`** for user-space USB communication with the **CH347T** high-speed bridge chip. The architecture is explicitly decoupled, allowing seamless integration of future bridge chips (e.g., FTDI) without changing your application code.

## 🎯 Purpose

The primary goal of this library is to simplify and accelerate the testing of I2C and SPI hardware devices directly from a Linux host.

### The problem

Traditionally, testing an I2C or SPI peripheral, requires a lot of overhead.
* **Single-Board computers (e.g. Raspberry Pi)**: Flashing an OS, configuring device trees, dealing with proprietary vendor SDKs, or writing complex kernel-space drivers just to toggle a few registers.
* **Embedded Microcontrollers (e.g. Arduino, ESP32)**: Writing test firmware, setting up toolchains, flashing the board and implementing a custom serial/UART protocol just to pipe test data back to a PC.

Both methods slow down rapid prototyping

### The solution

This library transforms your Linux PC into a powerful hardware testing station using affordable, high-speed USB bridge chips (like the CH347T).
* **Plug-and-Play Testing:** Write and run hardware tests instantly from your laptop or PC using user-space comminucation - no custom kernel modules required.
* **Rapid Prototyping:** Ideal for hardware verification, sensor calibration, production line testing, and firmware debugging.

## 🚀 Key Features

*   **Multi-Protocol Support:** Unified abstract interfaces for SPI and I2C operations.
*   **Pure User-Space Driver:** Powered by `libusb-1.0`, eliminating the need for custom kernel modules.
*   **Extensible Architecture:** Interface-driven design (`I2CController`, `SPIController`) to support multiple bridge chips in the future.
*   **Linux Native & Modern C++:** Tailored for Linux environments using clean, efficient, and type-safe C++17/C++20.

## 🛠️ Project Structure

```text
├── CMakeLists.txt                  # CMake build configuration
├── CMakePresets.json               # CMake Presets configuration file
├── examples/                       # Usage demonstrations & test tools
├── include/
│   └── hardware_protocols/
│       ├── i2c/
│       │   ├── i2c.hpp             # Abstract I2C interface
│       │   └── i2c_ch347t.hpp      # CH347T I2C implementation header
│       └── spi/
│           ├── spi.hpp             # Abstract SPI interface
│           └── spi_ch347t.hpp      # CH347T SPI implementation header
└── src/
    └── hardware_protocols/
        ├── i2c/
        │   └── i2c_ch347t.cpp      # CH347T I2C implementation source file
        └── spi/
            └── spi_ch347t.cpp      # CH347T SPI implementation source file
```

## 📦 Prerequisites & Dependencies

### 1. Required Packages
You need a C++17 compatible compiler, CMake, and the `libusb-1.0` development headers. Install them via your package manager:

**Ubuntu / Debian:**
```bash
sudo apt update
sudo apt install build-essential cmake libusb-1.0-0-dev
```

**Fedora / RHEL:**
```bash
sudo dnf groupinstall "Development Tools"
sudo dnf install cmake libusb1-devel
```

### 2. USB Permissions (Udev Rules)
By default, Linux restricts direct USB access for non-root users. To run your application without `sudo`, create a udev rule for the CH347T:

1. Create a new rule file:
   ```bash
   sudo nano /etc/udev/rules.d/99-ch347.rules
   ```
2. Add the following line (Set mode switch to Mode 1 on CH347T board):
   ```text
   SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="55db", MODE="0666", GROUP="plugdev"
   ```
3. Reload the udev rules:
   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

## ⚙️ Building the Library

Build the project using standard CMake workflow:

```bash
# 1. Clone the repository
git clone https://github.com/tvlinh0com/hardware_protocols.git
cd hardware_protocols

# 2. Configure and build (use default preset and build examples)
cmake --preset default -DHWPLIB_BUILD_EXAMPLE=ON
cmake --build build
```

## 💡 Quick Start & Usage

Below is a quick example showing how to initialize the CH347T via libusb and perform basic SPI/I2C transfers.

```cpp
// I2C usage template
#include <hardware_protocols/i2c/i2c_ch347t.hpp>
#include <iostream>
#include <memory>
#include <vector>

using namespace tvlinh::hardware_protocols;

int main() {
    // 1. Initialize the CH347T's I2C Controller (internally manages libusb context)
    auto i2c = std::make_unique<CH347TI2CController>();
    if (!i2c) {
        std::cout << "Cannot create I2C Controller instance\n";
        return 1;
    }

    if (!i2c->Init()) {
        std::cout << "Cannot initialize the I2C Controller\n";
        return 1;
    }

    // 2. Configure I2C clock speed
    if (!i2c->ConfigureClockSpeed(I2CController::ClockSpeed::kSpeed100K)) {
        std::cout << "Cannot set I2C clock speed\n";
        return 1;
    }

    uint32_t device_address = 0x38;
    std::vector<uint8_t> write_data = {0x01, 0x71};
    std::vector<uint8_t> read_data(1);

    // 3. I2C write operation with 2 bytes
    if (i2c->Write(device_address, write_data)) {
        std::cout << "Wrote data to device\n";
    } else {
        std::cout << "Failed to write data\n";
    }

    // 4. I2C read operation with 1 byte
    if (i2c->Read(device_address, read_data)) {
        std::cout << "Read data from device\n";
    } else {
        std::cout << "Failed to read data\n";
    }

    return 0;
}
```

## 🗺️ Roadmap & Future Scope

- [x] Abstract API interfaces (`I2CController`, `SPIController`).
- [x] Native Linux `libusb-1.0` driver backend for CH347T.
- [ ] Add support for FTDI chips (e.g., FT2232H / FT232H) using the same abstraction.
- [ ] Support Windows system

## 📄 License

Distributed under the MIT License. See `LICENSE` for more information.
