#pragma once

#include <cstdint>

namespace tvlinh {
    namespace hardware_protocols {
        class SPIController {
            public:
                struct SPIConfiguration {
                    public:
                        enum class SPIPolarity {
                            kLow,
                            kHigh
                        };

                        enum class SPIPhase {
                            kFirstEdge,
                            kSecondEdge
                        };

                        enum class SPICSActive {
                            kLow,
                            kHigh
                        };

                        enum class SPIBitOrder {
                            kMSBFirst,
                            kLSBFirst
                        };

                        uint32_t frequency_hz = 60000000;
                        SPIPolarity polarity = SPIPolarity::kLow;
                        SPIPhase phase = SPIPhase::kFirstEdge;
                        SPIBitOrder bit_order = SPIBitOrder::kMSBFirst;
                        SPICSActive cs_active = SPICSActive::kLow;
                        uint32_t cs_pin = 0;

                        bool IsValid() {
                            if (this->polarity != SPIPolarity::kLow && this->polarity != SPIPolarity::kHigh) {
                                return false;
                            }

                            if (this->phase != SPIPhase::kFirstEdge && this->phase != SPIPhase::kSecondEdge) {
                                return false;
                            }

                            if (this->cs_active != SPICSActive::kLow && this->cs_active != SPICSActive::kHigh) {
                                return false;
                            }

                            if (this->bit_order != SPIBitOrder::kMSBFirst && this->bit_order != SPIBitOrder::kLSBFirst) {
                                return false;
                            }

                            return true;
                        }
                };

                SPIController() = default;
                virtual ~SPIController() = default;

                virtual bool Init() = 0;
                virtual bool SetConfiguration(SPIConfiguration& spi_configuration) = 0;
                virtual bool Send(std::vector<uint8_t>& out_data) = 0;
                virtual bool Receive(std::vector<uint8_t>& in_data) = 0;
                virtual bool Transfer(std::vector<uint8_t>& out_data, std::vector<uint8_t>& in_data) = 0;

            protected:
                SPIConfiguration spi_configuration_;
        };
    }  // namespace hardware_protocols
}  // namespace tvlinh
