#pragma once

#include <libusb.h>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>
#include "i2c.hpp"

namespace tvlinh {
    namespace hardware_protocols {
        class CH347TI2CController : public I2CController {
            public:
                CH347TI2CController();
                ~CH347TI2CController();

                bool Init();
                bool ConfigureClockSpeed(ClockSpeed speed);
                bool Write(uint8_t address, std::vector<uint8_t>& data);
                bool Read(uint8_t address, std::vector<uint8_t>& data);

            private:
                static constexpr uint16_t kCh347tUsbVid = 0x1A86;
                static constexpr uint16_t kCh347tUsbPid = 0x55DB;
                static constexpr int kCh347tInterfaceNumber = 2;
                static constexpr int kCh347tTimeout = 1000;
                static constexpr int kCh347tOutEp = 0x06;
                static constexpr int kCh347tInEp = 0x86;
                static constexpr int kMaxPacketSize = 512;

                static constexpr uint8_t kCh347tCmdI2cStream = 0xAA;
                static constexpr uint8_t kCh347tCmdI2cStmStart = 0x74;
                static constexpr uint8_t kCh347tCmdI2cStmStop = 0x75;
                static constexpr uint8_t kCh347tCmdI2cStmOut = 0x80;
                static constexpr uint8_t kCh347tCmdI2cStmIn = 0xC0;
                static constexpr uint8_t kCh347tCmdI2cStmSet = 0x60;  // bit 2: SPI with two data pairs D5,D4=out, D7,D6=in
                static constexpr uint8_t kCh347tCmdI2cStmEnd = 0x00;

                static constexpr uint8_t kCh347tStmI2c20K = 0x00;
                static constexpr uint8_t kCh347tStmI2c100K = 0x01;
                static constexpr uint8_t kCh347tStmI2c400K = 0x02;
                static constexpr uint8_t kCh347tStmI2c750K = 0x03;

                libusb_context* context_;
                libusb_device_handle* device_handle_;
                bool is_init_done_;
        };
    }  // namespace hardware_protocols
}  // namespace tvlinh
