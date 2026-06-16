#include <cstdint>
#include <hardware_protocols/i2c/i2c_ch347t.hpp>
#include <iostream>
#include <vector>

using namespace tvlinh::hardware_protocols;

int main() {
    I2CController* i2c = new CH347TI2CController();

    if (!i2c) {
        std::cout << "Cannot create new I2C Controller instance\n";
        return 1;
    }

    if (!i2c->Init()) {
        std::cout << "Cannot initialize the I2C Controller\n";
        delete i2c;
        return 1;
    }

    if (!i2c->ConfigureClockSpeed(I2CController::ClockSpeed::kSpeed100K)) {
        std::cout << "Cannot set I2C clock speed\n";
        delete i2c;
        return 1;
    }

    std::cout << "Start probing I2C addresses...\n";

    for (uint8_t address = 0; address <= 0x7f; address++) {
        std::vector<uint8_t> dummy(0);

        if (i2c->Write(address, dummy)) {
            std::cout << "Address 0x" << std::hex << (int)address << " is present\n";
        }
    }

    std::cout << "Complete the probing.\n";
    delete i2c;

    return 0;
}
