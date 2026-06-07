#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace tvlinh {
    namespace hardware_protocols {
        class I2CController {
            public:
                enum class ClockSpeed : uint8_t {
                    kSpeed20K,
                    kSpeed100K,
                    kSpeed400K,
                    kSpeed750K,
                };

                I2CController() = default;
                virtual ~I2CController() = default;

                virtual bool Init() = 0;
                virtual bool ConfigureClockSpeed(ClockSpeed speed) = 0;
                virtual bool Write(uint8_t address, std::vector<uint8_t>& data) = 0;
                virtual bool Read(uint8_t address, std::vector<uint8_t>& data) = 0;
        };
    }  // namespace hardware_protocols
}  // namespace tvlinh
