#include "hardware_protocols/i2c/i2c_ch347.hpp"
#include <libusb.h>
#include <cstddef>
#include <cstdint>
#include <vector>

tvlinh::hardware_protocols::CH347I2CController::CH347I2CController() {
    this->context_ = nullptr;
    this->device_handle_ = nullptr;
    this->is_init_done_ = false;
}

tvlinh::hardware_protocols::CH347I2CController::~CH347I2CController() {
    if (this->is_init_done_) {
        libusb_release_interface(this->device_handle_, kCh347InterfaceNumber);
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

bool tvlinh::hardware_protocols::CH347I2CController::Init() {
    auto result = libusb_init_context(&this->context_, NULL, 0);

    if (result < 0) {
        return false;
    }

    this->device_handle_ = libusb_open_device_with_vid_pid(this->context_, kCh347UsbVid, kCh347UsbPid);

    if (!this->device_handle_) {
        libusb_exit(this->context_);
        return false;
    }

    result = libusb_set_auto_detach_kernel_driver(this->device_handle_, 1);

    if (result < 0) {
        libusb_close(this->device_handle_);
        libusb_exit(this->context_);
        return false;
    }

    result = libusb_claim_interface(this->device_handle_, kCh347InterfaceNumber);

    if (result < 0) {
        libusb_close(this->device_handle_);
        libusb_exit(this->context_);
        return false;
    }

    this->is_init_done_ = true;

    return true;
}

bool tvlinh::hardware_protocols::CH347I2CController::ConfigureClockSpeed(ClockSpeed speed) {
    uint8_t speed_value = kCh347StmI2c20K;

    switch (speed) {
        case ClockSpeed::kSpeed20K:
            speed_value = kCh347StmI2c20K;
            break;
        case ClockSpeed::kSpeed100K:
            speed_value = kCh347StmI2c100K;
            break;
        case ClockSpeed::kSpeed400K:
            speed_value = kCh347StmI2c400K;
            break;
        case ClockSpeed::kSpeed750K:
            speed_value = kCh347StmI2c750K;
            break;
    };

    std::vector<uint8_t> packet;

    packet.push_back(kCh347CmdI2cStream);
    packet.push_back(kCh347CmdI2cStmSet | (speed_value & 0b111));
    packet.push_back(kCh347CmdI2cStmEnd);

    int transferred_out = 0;

    auto result = libusb_bulk_transfer(this->device_handle_, kCh347OutEp, packet.data(), packet.size(), &transferred_out, kCh347Timeout);

    if (result < 0 || transferred_out != packet.size()) {
        return false;
    }

    return true;
}

bool tvlinh::hardware_protocols::CH347I2CController::Write(uint8_t address, std::vector<uint8_t>& data) {
    std::vector<uint8_t> packet;

    packet.push_back(kCh347CmdI2cStream);
    packet.push_back(kCh347CmdI2cStmStart);
    packet.push_back(kCh347CmdI2cStmOut | (data.size() + 1));
    packet.push_back(address << 1);
    for (auto value : data) {
        packet.push_back(value);
    }
    packet.push_back(kCh347CmdI2cStmStop);
    packet.push_back(kCh347CmdI2cStmEnd);

    int transferred = 0;

    auto result = libusb_bulk_transfer(this->device_handle_, kCh347OutEp, packet.data(), packet.size(), &transferred, kCh347Timeout);

    if (result < 0 || transferred != packet.size()) {
        return false;
    }

    std::vector<uint8_t> ack(kMaxPacketSize);
    result = libusb_bulk_transfer(this->device_handle_, kCh347InEp, ack.data(), ack.size(), &transferred, kCh347Timeout);

    if (result < 0) {
        return false;
    }

    if (transferred != (data.size() + 1)) {
        return false;
    }

    // Verify ACK for each sent byte (ADDRESS + DATA)
    for (int i = 0; i < transferred; i++) {
        if (ack[i] != 1) {
            return false;
        }
    }

    return true;
}

bool tvlinh::hardware_protocols::CH347I2CController::Read(uint8_t address, std::vector<uint8_t>& data) {
    if (data.size() == 0) {
        return true;
    }

    std::vector<uint8_t> packet;

    packet.push_back(kCh347CmdI2cStream);
    packet.push_back(kCh347CmdI2cStmStart);
    packet.push_back(kCh347CmdI2cStmOut | 1);
    packet.push_back((address << 1) | 1);

    for (int i = 1; i < data.size(); i++) {
        // Read back and emit ACK
        packet.push_back(kCh347CmdI2cStmIn | 1);
    }

    // Read back and emit NACK
    packet.push_back(kCh347CmdI2cStmIn);
    packet.push_back(kCh347CmdI2cStmStop);
    packet.push_back(kCh347CmdI2cStmEnd);

    int transferred = 0;

    auto result = libusb_bulk_transfer(this->device_handle_, kCh347OutEp, packet.data(), packet.size(), &transferred, kCh347Timeout);

    if (result < 0 || transferred != packet.size()) {
        return false;
    }

    // Resize to contain ACK status
    data.resize(data.size() + 1);

    result = libusb_bulk_transfer(this->device_handle_, kCh347InEp, data.data(), data.size(), &transferred, kCh347Timeout);

    if (result < 0 || transferred != data.size() || data[0] != 1) {
        return false;
    }

    // Remove ACK
    if (!data.empty()) {
        data.erase(data.begin());
    }

    return true;
}
