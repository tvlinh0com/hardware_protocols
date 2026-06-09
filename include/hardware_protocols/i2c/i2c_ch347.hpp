#pragma once

#include <libusb.h>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>
#include "i2c.hpp"

namespace tvlinh {
    namespace hardware_protocols {
        class CH347I2CController : public I2CController {
            public:
                CH347I2CController();
                ~CH347I2CController();

                bool Init();
                bool ConfigureClockSpeed(ClockSpeed speed);
                bool Write(uint8_t address, std::vector<uint8_t>& data);
                bool Read(uint8_t address, std::vector<uint8_t>& data);

            private:
                static constexpr uint16_t kCh347UsbVid = 0x1A86;
                static constexpr uint16_t kCh347UsbPid = 0x55DB;
                static constexpr int kCh347InterfaceNumber = 2;
                static constexpr int kCh347Timeout = 1000;
                static constexpr int kCh347OutEp = 0x06;
                static constexpr int kCh347InEp = 0x86;
                static constexpr int kMaxPacketSize = 512;

                static constexpr uint8_t kCh347CmdI2cStream = 0xAA;
                static constexpr uint8_t kCh347CmdI2cStmStart = 0x74;
                static constexpr uint8_t kCh347CmdI2cStmStop = 0x75;
                static constexpr uint8_t kCh347CmdI2cStmOut = 0x80;
                static constexpr uint8_t kCh347CmdI2cStmIn = 0xC0;
                static constexpr uint8_t kCh347CmdI2cStmSet = 0x60;  // bit 2: SPI with two data pairs D5,D4=out, D7,D6=in
                static constexpr uint8_t kCh347CmdI2cStmEnd = 0x00;

                static constexpr uint8_t kCh347StmI2c20K = 0x00;
                static constexpr uint8_t kCh347StmI2c100K = 0x01;
                static constexpr uint8_t kCh347StmI2c400K = 0x02;
                static constexpr uint8_t kCh347StmI2c750K = 0x03;

                libusb_context* context_;
                libusb_device_handle* device_handle_;
                bool is_init_done_;
        };
    }  // namespace hardware_protocols
}  // namespace tvlinh
