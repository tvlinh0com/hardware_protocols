#include <libusb.h>
#include <cstdio>
#include <cstring>
#include <hardware_protocols/spi/spi_ch347t.hpp>
#include <iostream>

tvlinh::hardware_protocols::CH347TSPIController::CH347TSPIController() {
    this->context_ = nullptr;
    this->device_handle_ = nullptr;
    this->is_init_done_ = false;
    this->spi_configuration_ = {};
    this->ch347t_spi_configuration_ = {};
    this->firmware_version_ = 0;
}

tvlinh::hardware_protocols::CH347TSPIController::~CH347TSPIController() {
    if (this->is_init_done_) {
        libusb_release_interface(this->device_handle_, kCh347tInterfaceNumber);
        this->is_init_done_ = false;
    }

    if (this->device_handle_) {
        libusb_close(this->device_handle_);
        this->device_handle_ = nullptr;
    }

    if (this->context_) {
        libusb_exit(this->context_);
        this->context_ = nullptr;
    }
}

bool tvlinh::hardware_protocols::CH347TSPIController::UsbTransfer(std::vector<uint8_t>& output, std::vector<uint8_t>& input) {
    if (output.size() > kMaxUsbPacketSize || input.size() > kMaxUsbPacketSize) {
        return false;
    }

    int transferred = 0;

    if (output.size() != 0) {
        auto result = libusb_bulk_transfer(this->device_handle_, kCh347tOutEp, output.data(), output.size(), &transferred, kCh347tTimeout);

        if (result < 0 || transferred != output.size()) {
            return false;
        }
    }

    if (input.size() != 0) {
        auto result = libusb_bulk_transfer(this->device_handle_, kCh347tInEp, input.data(), input.size(), &transferred, kCh347tTimeout);

        if (result < 0 || transferred != input.size()) {
            return false;
        }
    }

    return true;
}

bool tvlinh::hardware_protocols::CH347TSPIController::Init() {
    auto result = libusb_init(&this->context_);

    if (result < 0) {
        return false;
    }

    this->device_handle_ = libusb_open_device_with_vid_pid(this->context_, kCh347tUsbVid, kCh347tUsbPid);

    if (!this->device_handle_) {
        return false;
    }

    result = libusb_reset_device(this->device_handle_);

    if (result == LIBUSB_ERROR_NOT_FOUND) {
        // Need to close the old device handle and reopen a new one
        libusb_close(this->device_handle_);

        this->device_handle_ = libusb_open_device_with_vid_pid(this->context_, kCh347tUsbVid, kCh347tUsbPid);

        if (!this->device_handle_) {
            return false;
        }
    } else if (result != 0) {
        return false;
    }

    result = libusb_set_auto_detach_kernel_driver(this->device_handle_, 1);

    if (result < 0) {
        return false;
    }

    result = libusb_claim_interface(this->device_handle_, kCh347tInterfaceNumber);

    if (result < 0) {
        return false;
    }

    this->is_init_done_ = true;

    // Get CH347 firmware version
    std::vector<uint8_t> out_firmware_version{kCh347tCmdSpiGetCfg, 0x01, 0x00, 0};
    std::vector<uint8_t> in_firmware_version(3 + 4);

    if (!this->UsbTransfer(out_firmware_version, in_firmware_version) || in_firmware_version[0] != kCh347tCmdSpiGetCfg) {
        return false;
    }

    this->firmware_version_ = (in_firmware_version[4] << 8 | in_firmware_version[3]);

    // Load SPI configuration during initialization time
    std::vector<uint8_t> out_get_config{kCh347tCmdSpiGetCfg, 0x01, 0x00, 1};
    std::vector<uint8_t> in_get_config(3 + sizeof this->ch347t_spi_configuration_);

    if (!this->UsbTransfer(out_get_config, in_get_config)) {
        return false;
    }

    std::memcpy(&this->ch347t_spi_configuration_, in_get_config.data() + 3, sizeof this->ch347t_spi_configuration_);

    return true;
}

bool tvlinh::hardware_protocols::CH347TSPIController::SetConfiguration(SPIConfiguration& spi_configuration) {
    // Reject SPI configuration in case of CS pin greater or equal MaxCS
    if (spi_configuration.cs_pin >= kCh347tMaxCSPin) {
        return false;
    }

    if (!spi_configuration.IsValid()) {
        return false;
    }

    // Find clock prescaler
    uint32_t clock_table0[] = {
        60000000, 48000000, 36000000, 30000000, 28000000, 24000000, 18000000, 15000000,
        14000000, 12000000, 9000000, 7500000, 7000000, 6000000, 4500000, 3750000,
        3500000, 3000000, 2250000, 1875000, 1750000, 1500000, 1125000, 937500,
        875000, 750000, 562500, 468750, 437500, 375000, 281250, 218750};

    uint32_t clock_table1[] = {
        28000000, 14000000, 7000000, 3500000, 1750000, 875000, 437500, 218750,
        72000000, 36000000, 18000000, 9000000, 4500000, 2250000, 1125000, 562500,
        48000000, 24000000, 12000000, 6000000, 3000000, 1500000, 750000, 375000,
        60000000, 30000000, 15000000, 7500000, 3750000, 1875000, 937500, 468750};

    uint32_t clock_table2[] = {
        60000000, 30000000, 15000000, 7500000, 3750000, 1875000, 937500, 468750};

    uint8_t clock_index = 1, scale = 0;
    this->ch347t_spi_configuration_.baud_prescaler = 0;
    int i, j;

    if (this->firmware_version_ >= 0x0341) {
        for (i = 0; i < sizeof clock_table0 / sizeof clock_table0[0]; i++) {
            if (clock_table0[i] <= spi_configuration.frequency_hz) {
                for (j = 0; j < sizeof clock_table1 / sizeof clock_table1[0]; j++) {
                    if (clock_table0[i] == clock_table1[j]) {
                        break;
                    }
                }

                clock_index = j / 8 + 1;
                scale = clock_table1[j] / clock_table1[clock_index * 8 - 1];

                if (clock_index == 2) {
                    clock_index = 5;
                }

                this->ch347t_spi_configuration_.baud_prescaler = 7;

                while (scale / 2) {
                    this->ch347t_spi_configuration_.baud_prescaler--;
                    scale /= 2;
                }

                break;
            }
        }
    } else {
        for (i = 0; i < sizeof clock_table2 / sizeof clock_table2[0]; i++) {
            if (clock_table2[i] <= spi_configuration.frequency_hz) {
                scale = clock_table2[i] / clock_table2[7];
                this->ch347t_spi_configuration_.baud_prescaler = 7;

                while (scale / 2) {
                    this->ch347t_spi_configuration_.baud_prescaler--;
                    scale /= 2;
                }

                break;
            }
        }
    }

    // Set prescaler
    this->ch347t_spi_configuration_.baud_prescaler <<= 3;

    // Set clock polarity
    this->ch347t_spi_configuration_.polarity = spi_configuration.polarity == SPIConfiguration::SPIPolarity::kLow ? kCh347tClockPolarityLow : kCh347tClockPolarityHigh;

    // Set clock phase
    this->ch347t_spi_configuration_.phase = spi_configuration.phase == SPIConfiguration::SPIPhase::kFirstEdge ? kCh347tClockPhaseFirstEdge : kCh347tClockPhaseSecondEdge;

    // Set clock active level
    this->ch347t_spi_configuration_.cs_config = spi_configuration.cs_active == SPIConfiguration::SPICSActive::kHigh ? kCh347tCSActiveHigh : kCh347tCSActiveLow;

    // Set bit order
    this->ch347t_spi_configuration_.firstbit = spi_configuration.bit_order == SPIConfiguration::SPIBitOrder::kMSBFirst ? kCh347tMsbFirst : kCh347tLsbFirst;

    // Init SPI system clock
    if (this->firmware_version_ >= 0x0341) {
        std::vector<uint8_t> out_system_clock{kCh347tCmdSpiClockInit, 0x01, 0x00, clock_index};

        if (!this->UsbTransfer(out_system_clock, out_system_clock)) {
            return false;
        }
    }

    std::vector<uint8_t> in_set_config(3 + 1);
    std::vector<uint8_t> out_set_config{kCh347tCmdSpiSetCfg, sizeof this->ch347t_spi_configuration_, 0x00};

    uint8_t* p_spi_config = reinterpret_cast<uint8_t*>(&this->ch347t_spi_configuration_);

    out_set_config.insert(out_set_config.end(), p_spi_config, p_spi_config + sizeof this->ch347t_spi_configuration_);

    if (!this->UsbTransfer(out_set_config, in_set_config)) {
        return false;
    }

    if (in_set_config[0] != kCh347tCmdSpiSetCfg || in_set_config[3] != 0) {
        return false;
    }

    // Save new SPI configuration and we will use CS pin later
    this->spi_configuration_ = spi_configuration;

    return true;
}

bool tvlinh::hardware_protocols::CH347TSPIController::SetCSPin(uint32_t pin_num, bool enable) {
    std::vector<uint8_t> out_cs_ctrl(13);
    std::vector<uint8_t> in_cs_ctrl(0);

    out_cs_ctrl[0] = kCh347tCmdSpiCsCtrl;
    out_cs_ctrl[1] = 10;
    out_cs_ctrl[2] = 0;
    // Data start from offset 3 of payload
    out_cs_ctrl[pin_num ? 8 : 3] = enable ? (kCh347tCsChange | kCh347tCsAssert) : (kCh347tCsChange | kCh347tCsDeassert);

    if (!this->UsbTransfer(out_cs_ctrl, in_cs_ctrl)) {
        return false;
    }

    return true;
}

bool tvlinh::hardware_protocols::CH347TSPIController::Send(std::vector<uint8_t>& out_data) {
    if (out_data.size() == 0) {
        return true;
    }

    if (!this->SetCSPin(this->spi_configuration_.cs_pin, true)) {
        return false;
    }

    uint32_t remaining_len = out_data.size();
    uint32_t offset = 0;

    while (remaining_len) {
        uint32_t chunk_len = remaining_len > kCh347tMaxSpiPacketSize ? kCh347tMaxSpiPacketSize : remaining_len;
        std::vector<uint8_t> out(3 + chunk_len);
        std::vector<uint8_t> in(4, 0);
        out[0] = kCh347tCmdSpiOut;
        out[1] = chunk_len & 0xff;
        out[2] = (chunk_len >> 8) & 0xff;
        std::copy(out_data.begin() + offset, out_data.begin() + offset + chunk_len, out.begin() + 3);

        if (!this->UsbTransfer(out, in) || in[3] != 0) {
            return false;
        }

        remaining_len -= chunk_len;
        offset += chunk_len;
    }

    if (!this->SetCSPin(this->spi_configuration_.cs_pin, false)) {
        return false;
    }

    return true;
}

bool tvlinh::hardware_protocols::CH347TSPIController::Receive(std::vector<uint8_t>& in_data) {
    if (in_data.size() == 0) {
        return true;
    }

    if (!this->SetCSPin(this->spi_configuration_.cs_pin, true)) {
        return false;
    }

    uint32_t total_len = in_data.size();
    // Construct payload for CH347T
    std::vector<uint8_t> out_command{kCh347tCmdSpiIn, 0x04, 0x00,             // Command + payload length
                                     static_cast<uint8_t>(total_len),         // Data length
                                     static_cast<uint8_t>(total_len >> 8),    // Data length
                                     static_cast<uint8_t>(total_len >> 16),   // Data length
                                     static_cast<uint8_t>(total_len >> 24)};  // Data length
    std::vector<uint8_t> dummy(0);

    if (!this->UsbTransfer(out_command, dummy)) {
        return false;
    }

    uint32_t remaining_len = total_len;
    uint32_t offset = 0;
    while (remaining_len) {
        uint32_t chunk_len = remaining_len > kCh347tMaxSpiPacketSize ? kCh347tMaxSpiPacketSize : remaining_len;
        std::vector<uint8_t> in_command(3 + chunk_len, 0x00);

        if (!this->UsbTransfer(dummy, in_command) || in_command[0] != kCh347tCmdSpiIn) {
            return false;
        }

        std::copy(in_command.begin() + 3, in_command.begin() + 3 + chunk_len, in_data.begin() + offset);

        remaining_len -= chunk_len;
        offset += chunk_len;
    }

    if (!this->SetCSPin(this->spi_configuration_.cs_pin, false)) {
        return false;
    }

    return true;
}

bool tvlinh::hardware_protocols::CH347TSPIController::Transfer(std::vector<uint8_t>& out_data, std::vector<uint8_t>& in_data) {
    if (out_data.size() == 0 || in_data.size() == 0 || out_data.size() != in_data.size()) {
        return false;
    }

    if (!this->SetCSPin(this->spi_configuration_.cs_pin, true)) {
        return false;
    }

    uint32_t remaining_len = out_data.size();
    uint32_t offset = 0;

    while (remaining_len) {
        uint32_t chunk_len = remaining_len > kCh347tMaxSpiPacketSize ? kCh347tMaxSpiPacketSize : remaining_len;

        std::vector<uint8_t> out_command(3 + chunk_len);
        std::vector<uint8_t> in_command(3 + chunk_len);
        out_command[0] = kCh347tCmdSpiOutIn;
        out_command[1] = static_cast<uint8_t>(chunk_len);
        out_command[2] = static_cast<uint8_t>(chunk_len >> 8);
        std::copy(out_data.begin() + offset, out_data.begin() + offset + chunk_len, out_command.begin() + 3);

        if (!this->UsbTransfer(out_command, in_command) || in_command[0] != kCh347tCmdSpiOutIn) {
            return false;
        }

        std::copy(in_command.begin() + 3, in_command.begin() + 3 + chunk_len, in_data.begin() + offset);

        remaining_len -= chunk_len;
        offset += chunk_len;
    }

    if (!this->SetCSPin(this->spi_configuration_.cs_pin, false)) {
        return false;
    }

    return true;
}
