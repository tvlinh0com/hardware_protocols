#include <chrono>
#include <cmath>
#include <cstdint>
#include <hardware_protocols/i2c/i2c_ch347.hpp>
#include <iostream>
#include <thread>
#include <vector>

class AHT20 {
    public:
        AHT20() : i2c(nullptr) {};

        ~AHT20() {
            if (this->i2c != nullptr) {
                delete this->i2c;
            }
        }

        bool Initialize() {
            if (this->i2c == nullptr) {
                this->i2c = new tvlinh::hardware_protocols::CH347I2CController();

                if (this->i2c) {
                    if (!this->i2c->Init()) {
                        return false;
                    }

                    if (!this->i2c->ConfigureClockSpeed(tvlinh::hardware_protocols::I2CController::ClockSpeed::kSpeed100K)) {
                        return false;
                    }
                } else {
                    return false;
                }
            }

            return true;
        }

        bool IsPresent() {
            std::vector<uint8_t> data(0);

            if (!this->i2c->Write(kAht20Address, data)) {
                return false;
            }

            return true;
        }

        bool InitializeSensor() {
            std::vector<uint8_t> init_command;

            init_command.push_back(kAht20InitCommand);
            init_command.push_back(kAht20InitData1);
            init_command.push_back(kAht20InitData2);

            if (!this->i2c->Write(kAht20Address, init_command)) {
                return false;
            }

            return true;
        }

        bool GetStatus(uint8_t& status) {
            std::vector<uint8_t> status_command(1, kAht20GetStatusCommand);

            if (!this->i2c->Write(kAht20Address, status_command)) {
                return false;
            }

            std::vector<uint8_t> status_value(1);

            if (!this->i2c->Read(kAht20Address, status_value)) {
                return false;
            }

            status = status_value[0];

            return true;
        }

        bool TriggerMeasurement() {
            std::vector<uint8_t> measure_command;

            measure_command.push_back(kAht20MeasureCommand);
            measure_command.push_back(kAht20MeasureData1);
            measure_command.push_back(kAht20MeasureData2);

            if (!this->i2c->Write(kAht20Address, measure_command)) {
                return false;
            }

            return true;
        }

        bool ReadSensor(double& temperature, double& humidity) {
            std::vector<uint8_t> data(7);

            if (!this->i2c->Read(kAht20Address, data)) {
                return false;
            }

            humidity = ((data[1]) << 12) | (data[2] << 4) | ((data[3] & 0xf0) >> 4);
            humidity = humidity / std::pow(2, 20) * 100;

            temperature = ((data[3] & 0xf) << 16) | (data[4] << 8) | (data[5]);
            temperature = temperature / std::pow(2, 20) * 200 - 50;

            return true;
        }

        static constexpr uint8_t kAht20Address = 0x38;

        static constexpr uint8_t kAht20GetStatusCommand = 0x71;
        static constexpr uint32_t kStatusBusyIndicator = 0b10000000;
        static constexpr uint32_t kStatusCalEnabled = 0b00001000;

        static constexpr uint8_t kAht20InitCommand = 0xbe;
        static constexpr uint8_t kAht20InitData1 = 0x08;
        static constexpr uint8_t kAht20InitData2 = 0x00;

        static constexpr uint8_t kAht20MeasureCommand = 0xac;
        static constexpr uint8_t kAht20MeasureData1 = 0x33;
        static constexpr uint8_t kAht20MeasureData2 = 0x00;

    private:
        tvlinh::hardware_protocols::I2CController* i2c;
};

int main() {
    AHT20 aht20;

    if (!aht20.Initialize()) {
        std::cout << "Cannot initialize AHT20 sensor\n";
        return 1;
    }

    if (!aht20.IsPresent()) {
        std::cout << "AHT20 sensor is not present on I2C bus\n";
        return 1;
    }

    std::cout << "AHT20 sensor is present on I2C bus\n";

    // Wait 40ms after power-on
    std::this_thread::sleep_for(std::chrono::milliseconds(40));

    // Wait for ready to read
    while (true) {
        uint8_t status = 0;

        if (!aht20.GetStatus(status)) {
            std::cout << "Cannot get STATUS\n";
            return 1;
        }

        if ((status & AHT20::kStatusCalEnabled)) {
            // Ready to read
            break;
        } else {
            // Need to initialize
            if (!aht20.InitializeSensor()) {
                std::cout << "Cannot initialize AHT20 sensor\n";
                return 1;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    std::cout << "AHT20 sensor is ready to read now\n";

    // Loop to read temperature and humidity each 1 second
    while (true) {
        // Trigger measurement
        if (!aht20.TriggerMeasurement()) {
            std::cout << "Cannot trigger measurement\n";
            return 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(80));

        uint8_t status = 0;

        do {
            if (!aht20.GetStatus(status)) {
                std::cout << "Cannot get STATUS\n";
                return 1;
            }

            if (status & AHT20::kStatusBusyIndicator) {
                // Wait for ready
                std::this_thread::sleep_for(std::chrono::milliseconds(80));
            }
        } while (status & AHT20::kStatusBusyIndicator);

        double temp, humi;

        if (!aht20.ReadSensor(temp, humi)) {
            std::cout << "Cannot get Temperature and Humidity value\n";
            return 1;
        }

        std::cout << std::fixed << "Temperature = " << temp << " *C\n";
        std::cout << std::fixed << "Humidity = " << humi << " %RH\n";

        // Sleep 1 second each reading
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
