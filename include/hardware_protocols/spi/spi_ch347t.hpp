#pragma once

#include <libusb.h>
#include <cstdint>
#include <vector>
#include "spi.hpp"

namespace tvlinh {
    namespace hardware_protocols {
        class CH347TSPIController : public SPIController {
            public:
                CH347TSPIController();
                CH347TSPIController(uint8_t bus, uint8_t dev_num);
                virtual ~CH347TSPIController();

                virtual bool Init();
                virtual bool SetConfiguration(SPIConfiguration& spi_configuration);
                virtual bool Send(std::vector<uint8_t>& out_data);
                virtual bool Receive(std::vector<uint8_t>& in_data);
                virtual bool Transfer(std::vector<uint8_t>& out_data, std::vector<uint8_t>& in_data);

            private:
                bool UsbTransfer(std::vector<uint8_t>& output, std::vector<uint8_t>& input);
                bool SetCSPin(uint32_t pin_num, bool enable);

#pragma pack(push, 1)
                struct CH347TSPIConfiguration {
                    public:
                        // 2 Lines FullDuplex (0x0000)
                        // 2 Lines RX (0x0400)
                        // 1 Line RX (0x8000)
                        // 1 Line TX (0xC000)
                        uint16_t direction;
                        // Master (0x0104)
                        // Slave (0x0000)
                        uint16_t mode;
                        uint16_t bpw;       // Bits per word, 0x0000: 8 bits, 0x0800: 16 bits
                        uint16_t polarity;  // Clock polarity (bit 1)
                        uint16_t phase;     // Clock phase (bit 0)
                        // Software NSS (0x0200)
                        // Hardware NSS (0x0000)
                        uint16_t nss;  // Negative Slave Select
                        uint16_t baud_prescaler;
                        uint16_t firstbit;  // Bit order, 0x00: MSB, 0x80: LSB
                        uint16_t crc_polynomial;
                        uint16_t write_read_interval;  // microseconds
                        uint8_t out_default_data;      // the default data sent on MOSI while the device is reading
                        // CS0 polarity (bit 7) CS1 polarity (bit 6)
                        // 0: low active, 1: high active
                        uint8_t cs_config;
                        uint8_t reserved[4];
                };
#pragma pack(pop)

                static constexpr uint16_t kCh347tUsbVid = 0x1A86;
                static constexpr uint16_t kCh347tUsbPid = 0x55DB;
                static constexpr int kCh347tInterfaceNumber = 2;
                static constexpr int kCh347tTimeout = 1000;
                static constexpr int kCh347tOutEp = 0x06;
                static constexpr int kCh347tInEp = 0x86;
                /* The USB descriptor says the max transfer size is 512 bytes, but the
                 * vendor driver only seems to transfer a maximum of 510 bytes at once
                 */
                static constexpr uint32_t kMaxUsbPacketSize = 510;

                static constexpr uint8_t kCh347tCmdSpiGetCfg = 0xCA;
                static constexpr uint8_t kCh347tCmdSpiSetCfg = 0xC0;
                static constexpr uint8_t kCh347tCmdSpiCsCtrl = 0xC1;
                static constexpr uint8_t kCh347tCmdSpiOutIn = 0xC2;
                static constexpr uint8_t kCh347tCmdSpiIn = 0xC3;
                static constexpr uint8_t kCh347tCmdSpiOut = 0xC4;
                static constexpr uint8_t kCh347tCmdSpiClockInit = 0xE1;

                static constexpr uint8_t kCh347tClockPolarityHigh = 0x02;
                static constexpr uint8_t kCh347tClockPolarityLow = 0x00;
                static constexpr uint16_t kCh347tClockPhaseFirstEdge = 0x00;
                static constexpr uint16_t kCh347tClockPhaseSecondEdge = 0x01;
                static constexpr uint8_t kCh347tLsbFirst = 0x08;
                static constexpr uint8_t kCh347tMsbFirst = 0x00;
                static constexpr uint8_t kCh347tCSActiveHigh = 0xC0;
                static constexpr uint8_t kCh347tCSActiveLow = 0x00;

                static constexpr uint8_t kCh347tCsAssert = 0x00;
                static constexpr uint8_t kCh347tCsDeassert = 0x40;
                static constexpr uint8_t kCh347tCsChange = 0x80;

                static constexpr uint32_t kCh347tMaxSpiFreq = 60 * 1000 * 1000;
                static constexpr uint32_t kCh347tMinSpiFreq = kCh347tMaxSpiFreq >> 7;
                static constexpr uint32_t kCh347tMaxCSPin = 2;
                static constexpr uint8_t kCh347tCS0Pin = 0;
                static constexpr uint8_t kCh347tCS1Pin = 1;
                // Minus 3 (1 byte CH347T command + 2 bytes length overhead)
                static constexpr uint32_t kCh347tMaxSpiPacketSize = kMaxUsbPacketSize - 3;

                libusb_context* context_;
                libusb_device_handle* device_handle_;
                bool is_init_done_;
                CH347TSPIConfiguration ch347t_spi_configuration_;
                uint16_t firmware_version_;
                uint8_t bus_, dev_num_;
                bool is_specific_device_;
        };
    }  // namespace hardware_protocols
}  // namespace tvlinh
