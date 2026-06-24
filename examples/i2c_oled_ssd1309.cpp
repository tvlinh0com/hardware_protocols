#include <chrono>
#include <cmath>
#include <cstdint>
#include <hardware_protocols/i2c/i2c_ch347t.hpp>
#include <iostream>
#include <thread>

using namespace tvlinh::hardware_protocols;

class Point {
};

class Line {
    private:
        Point x, y;
};

class OledDisplay {
    private:
        class PixelsMAP {
            private:
                friend class OledDisplay;
                class BitProxy {
                    public:
                        BitProxy(uint8_t& byte, int bit_pos, PixelsMAP& pixels_ram) : byte_(byte), bit_pos_(bit_pos), pixels_ram_(pixels_ram) {}

                        ~BitProxy() {}

                        operator bool() const {
                            return (this->byte_ & (1 << this->bit_pos_)) != 0;
                        }

                        PixelsMAP& operator=(bool value) {
                            if (value) {
                                this->byte_ |= (1 << this->bit_pos_);
                            } else {
                                this->byte_ &= ~(1 << this->bit_pos_);
                            }

                            return this->pixels_ram_;
                        }

                        PixelsMAP& operator=(const BitProxy& bit_proxy) {
                            return this->operator=(static_cast<bool>(bit_proxy));
                        }

                    private:
                        uint8_t& byte_;
                        int bit_pos_;
                        PixelsMAP& pixels_ram_;
                };

            public:
                PixelsMAP(uint32_t width, uint32_t height) {
                    this->width_ = width;
                    this->height_ = height;

                    uint32_t row = std::ceil(height / 8.0);

                    this->pixels_ = new uint8_t*[row];

                    for (uint32_t i = 0; i < row; i++) {
                        this->pixels_[i] = new uint8_t[width];
                    }
                }

                PixelsMAP(uint32_t width, uint32_t height, const std::vector<uint8_t>& bytes) : PixelsMAP(width, height) {
                    this->Fill(bytes);
                }

                ~PixelsMAP() {
                    uint32_t row = std::ceil(this->height_ / 8.0);

                    if (this->pixels_) {
                        for (uint32_t i = 0; i < row; i++) {
                            if (this->pixels_[i]) {
                                delete[] this->pixels_[i];
                            }
                        }

                        delete[] this->pixels_;
                        this->pixels_ = nullptr;
                    }
                }

                BitProxy operator()(int row, int col) & {
                    return BitProxy(this->pixels_[row / 8][col], row % 8, *this);
                }

                PixelsMAP& operator<<(PixelsMAP& map) {
                    for (uint32_t row = 0; row < map.height_; row++) {
                        for (uint32_t col = 0; col < map.width_; col++) {
                            this->operator()(this->current_row_ + row, this->current_col_ + col) = map(row, col);
                        }
                    }

                    this->current_col_ += 8;

                    // Column overflow
                    if (this->current_col_ >= this->width_) {
                        this->current_col_ = 0;
                        this->current_row_ += 8;
                    }

                    // Row overflow
                    if (this->current_row_ >= this->height_) {
                        this->current_row_ = 0;
                    }

                    return *this;
                }

                void Clear() {
                    uint32_t row = std::ceil(this->height_ / 8.0);

                    if (this->pixels_) {
                        for (uint32_t i = 0; i < row; i++) {
                            for (uint32_t j = 0; j < this->width_; j++) {
                                this->pixels_[i][j] = 0;
                            }
                        }
                    }
                }

                PixelsMAP& Fill(const std::vector<uint8_t>& bytes) {
                    uint32_t col = 0;

                    for (auto col_value : bytes) {
                        this->pixels_[0][col++] = col_value;
                    }

                    return *this;
                }

                bool Render(OledDisplay* display) {
                    for (uint32_t i = 0; i < 8; i++) {
                        for (uint32_t j = 0; j < 4; j++) {
                            std::vector<uint8_t> data(&this->pixels_[i][j * 32], &this->pixels_[i][j * 32 + 32]);
                            if (!display->SendData(data)) {
                                return false;
                            }
                        }
                    }

                    return true;
                }

            private:
                uint8_t** pixels_ = nullptr;
                uint32_t width_ = 0;
                uint32_t height_ = 0;
                uint32_t current_row_ = 0;
                uint32_t current_col_ = 0;
        };

    public:
        enum class MemoryAddressingMode : uint8_t {
            kHorizontalAddressingMode = 0x00,
            kVerticalAddressingMode = 0x01,
            kPageAddressingMode = 0x02
        };

        enum class VCOMLevel : uint8_t {
            kLowVoltage = 0x00,
            kMidVoltage = 0x34,
            kHighVoltage = 0x3c
        };

        OledDisplay() : frame_buffer_(128, 64) {
            this->i2c_ = nullptr;
            this->InitializeFonts();
        }

        ~OledDisplay() {
            if (this->i2c_) {
                delete this->i2c_;
                this->i2c_ = nullptr;
            }
        }

        bool Init() {
            if (this->i2c_) {
                return true;
            }

            this->i2c_ = new CH347TI2CController();

            if (!this->i2c_) {
                return false;
            }

            if (!this->i2c_->Init()) {
                return false;
            }

            if (!this->i2c_->ConfigureClockSpeed(I2CController::ClockSpeed::kSpeed750K)) {
                return false;
            }

            std::vector<uint8_t> dummy(0);

            if (!this->i2c_->Write(kOledDisplayAddress, dummy)) {
                return false;
            }

            std::vector<uint8_t> commands{
                // Display OFF (sleep mode)
                kOledDisplayCommandTurnOff,
                // Set Clock Divide Ratio/Oscillator Frequency (0x80 = standard)
                kOledDisplayCommandSetDisplayClock, 0x80,
                // Multiplex Ratio = 63 (For 64-line displays)
                kOledDisplayCommandSetMultiplexRatio, 63,
                // Set Display Offset to 0 (No vertical shift)
                kOledDisplayCommandSetDisplayOffset, 0x00,
                // Set Display Start Line to RAM row 0
                kOledDisplayMaskStartLine | 0,
                // Set Memory Addressing Mode to Horizontal (Auto-wrap)
                kOledDisplayCommandSetMemoryAddressingMode, static_cast<uint8_t>(MemoryAddressingMode::kHorizontalAddressingMode),
                // Set Segment Re-map (COL0 mapped to SEG127) - Normal width orientation
                kOledDisplayCommandSetSegmentRemapOff,
                // Set COM Output Scan Direction - Remapped (Flips vertically)
                kOledDisplayCommandSetCOMRemapOff,
                // Set COM Pins Hardware Configuration (Alternative layout, Left/Right remap off)
                kOledDisplayCommandSetCOMHWConfiguration, 0x12,
                // Set Contrast Control to 255 (Maximum brightness brightness)
                kOledDisplayCommandContrast, 255,
                // Set Pre-charge Period (Phase 1: 2 clocks, Phase 2: 2 clocks)
                kOledDisplayCommandSetPreChargePeriod, 0x22,
                // Set VCOMH Deselect Voltage Level (~0.77 x VCC)
                kOledDisplayCommandSetVCOMLevel, static_cast<uint8_t>(VCOMLevel::kMidVoltage),
                // Entire Display ON (Resume to GDDRAM content, not forced all-pixels-white)
                kOledDisplayCommandEntireRAM,
                // Set Normal Display Mode (1 = Pixel Illuminated, 0 = Pixel Dark)
                kOledDisplayCommandNormalDisplay,
                // Display ON (Wake screen up out of sleep mode)
                kOledDisplayCommandTurnOn};

            return this->SendCommands(commands);

            return true;
        }

        bool TurnOn() {
            std::vector<uint8_t> commands{kOledDisplayCommandTurnOn};
            return this->SendCommands(commands);
        }

        bool TurnOff() {
            std::vector<uint8_t> commands{kOledDisplayCommandTurnOff};
            return this->SendCommands(commands);
        }

        bool SetContrastLevel(uint8_t contrast_level) {
            std::vector<uint8_t> commands{kOledDisplayCommandContrast, contrast_level};
            return this->SendCommands(commands);
        }

        bool SetRAMDisplay(bool ram_display) {
            std::vector<uint8_t> commands{ram_display ? kOledDisplayCommandEntireRAM : kOledDisplayCommandEntireON};
            return this->SendCommands(commands);
        }

        bool SetNormalDisplay(bool normal) {
            std::vector<uint8_t> commands{normal ? kOledDisplayCommandNormalDisplay : kOledDisplayCommandInverseDisplay};
            return this->SendCommands(commands);
        }

        bool SetMUXRatio(uint8_t mux) {
            if (mux < 15 || mux > 63) {
                return false;
            }

            std::vector<uint8_t> commands{kOledDisplayCommandSetMultiplexRatio, mux};
            return this->SendCommands(commands);
        }

        bool SetDisplayStartLine(uint8_t line) {
            if (line > 63) {
                return false;
            }

            std::vector<uint8_t> commands{static_cast<uint8_t>(kOledDisplayMaskStartLine | line)};
            return this->SendCommands(commands);
        }

        bool SetDisplayOffset(uint8_t offset) {
            if (offset > 63) {
                return false;
            }

            std::vector<uint8_t> commands{kOledDisplayCommandSetDisplayOffset, offset};
            return this->SendCommands(commands);
        }

        bool SetMemoryAddressingMode(MemoryAddressingMode mode) {
            if (mode > MemoryAddressingMode::kPageAddressingMode) {
                return false;
            }

            std::vector<uint8_t> commands{kOledDisplayCommandSetMemoryAddressingMode, static_cast<uint8_t>(mode)};
            return this->SendCommands(commands);
        }

        bool SetPAMStartPage(uint8_t row) {
            if (row > 7) {
                return false;
            }

            std::vector<uint8_t> commands{static_cast<uint8_t>(0xb0 | row)};
            return this->SendCommands(commands);
        }

        bool SetPAMStartColumn(uint8_t col) {
            if (col > 128) {
                return false;
            }

            std::vector<uint8_t> commands{static_cast<uint8_t>(0x10 | ((col >> 4) & 0xf)), static_cast<uint8_t>(col & 0xf)};
            return this->SendCommands(commands);
        }

        bool SetHAMPageRange(uint8_t start_row, uint8_t end_row) {
            if (start_row > 7 || end_row > 7) {
                return false;
            }

            std::vector<uint8_t> commands{kOledDisplayCommandSetPageAddress, start_row, end_row};
            return this->SendCommands(commands);
        }

        bool SetHAMColumnRange(uint8_t start_col, uint8_t end_col) {
            if (start_col > 127 || end_col > 127) {
                return false;
            }

            std::vector<uint8_t> commands{kOledDisplayCommandSetColumnAddress, start_col, end_col};
            return this->SendCommands(commands);
        }

        bool SetSegmentRemap(bool enable) {
            std::vector<uint8_t> commands{enable ? kOledDisplayCommandSetSegmentRemapOn : kOledDisplayCommandSetSegmentRemapOff};
            return this->SendCommands(commands);
        }

        bool SetCOMRemap(bool enable) {
            std::vector<uint8_t> commands{enable ? kOledDisplayCommandSetCOMRemapOn : kOledDisplayCommandSetCOMRemapOff};
            return this->SendCommands(commands);
        }

        void DrawPixel(int row, int col, bool value) {
            this->frame_buffer_(row, col) = value;
        }

        void DrawChar(char c) {
            int8_t code = c;

            switch (c) {
                case '\n':
                    this->frame_buffer_.current_row_ += 8;
                    if (this->frame_buffer_.current_row_ >= this->frame_buffer_.height_) {
                        this->frame_buffer_.current_row_ = 0;
                    }
                    return;
                case '\r':
                    this->frame_buffer_.current_col_ = 0;
                    return;

                default:
                    // Because the font MAP start from 0x20(32) SPACE char, we have to move the char code to first element of font MAP
                    code -= 0x20;

                    // Draw the font to main frame buffer
                    this->frame_buffer_ << *(this->fonts8x8[code]);
            }
        }

        void DrawString(std::string str) {
            for (uint32_t i = 0; i < str.size(); i++) {
                this->DrawChar(str.at(i));
            }
        }

        bool Render() {
            this->SetMemoryAddressingMode(MemoryAddressingMode::kHorizontalAddressingMode);
            this->SetHAMPageRange(0, 7);
            this->SetHAMColumnRange(0, 127);

            return this->frame_buffer_.Render(this);
        }

    private:
        void InitializeFonts() {
            // Font MAP start from 0x20 SPACE char
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x23, 0x13, 0x08, 0x64, 0x62, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x14, 0x08, 0x3E, 0x08, 0x14, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x50, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x42, 0x61, 0x51, 0x49, 0x46, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x21, 0x41, 0x45, 0x4B, 0x31, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x27, 0x45, 0x45, 0x45, 0x39, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x01, 0x71, 0x09, 0x05, 0x03, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x36, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x56, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x08, 0x14, 0x22, 0x41, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x41, 0x22, 0x14, 0x08, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x02, 0x01, 0x51, 0x09, 0x06, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x32, 0x49, 0x79, 0x41, 0x3E, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7F, 0x49, 0x49, 0x49, 0x41, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7F, 0x09, 0x09, 0x09, 0x01, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x3E, 0x41, 0x49, 0x49, 0x7A, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x20, 0x40, 0x41, 0x3F, 0x01, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7F, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7F, 0x09, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7F, 0x09, 0x19, 0x29, 0x46, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x46, 0x49, 0x49, 0x49, 0x31, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x01, 0x01, 0x7F, 0x01, 0x01, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x63, 0x14, 0x08, 0x14, 0x63, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x07, 0x08, 0x70, 0x08, 0x07, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x7F, 0x41, 0x41, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x41, 0x41, 0x7F, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x04, 0x02, 0x01, 0x02, 0x04, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x01, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x20, 0x54, 0x54, 0x54, 0x78, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7F, 0x48, 0x44, 0x44, 0x38, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x38, 0x44, 0x44, 0x44, 0x20, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x38, 0x44, 0x44, 0x48, 0x7F, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x38, 0x54, 0x54, 0x54, 0x18, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x08, 0x7E, 0x09, 0x01, 0x02, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x0C, 0x52, 0x52, 0x52, 0x3E, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x44, 0x7D, 0x40, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x20, 0x40, 0x44, 0x3D, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x41, 0x7F, 0x40, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7C, 0x04, 0x18, 0x04, 0x78, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7C, 0x08, 0x04, 0x04, 0x78, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x38, 0x44, 0x44, 0x44, 0x38, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7C, 0x14, 0x14, 0x14, 0x08, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x08, 0x14, 0x14, 0x18, 0x7C, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x7C, 0x08, 0x04, 0x04, 0x08, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x48, 0x54, 0x54, 0x54, 0x24, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x04, 0x3F, 0x44, 0x40, 0x20, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x44, 0x28, 0x10, 0x28, 0x44, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x0C, 0x50, 0x50, 0x50, 0x3C, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x44, 0x64, 0x54, 0x4C, 0x44, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x08, 0x36, 0x41, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x00, 0x77, 0x00, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x00, 0x41, 0x36, 0x08, 0x00, 0x00, 0x00, 0x00}));
            fonts8x8.push_back(std::make_unique<PixelsMAP>(8, 8, std::vector<uint8_t>{0x08, 0x08, 0x2A, 0x1C, 0x08, 0x00, 0x00, 0x00}));
        }

        bool SendCommands(std::vector<uint8_t>& commands) {
            commands.insert(commands.begin(), 0x00);
            return this->i2c_->Write(kOledDisplayAddress, commands);
        }

        bool SendData(std::vector<uint8_t>& data) {
            data.insert(data.begin(), 0x40);
            return this->i2c_->Write(kOledDisplayAddress, data);
        }

    private:
        I2CController* i2c_;
        PixelsMAP frame_buffer_;
        std::vector<std::unique_ptr<PixelsMAP>> fonts8x8;

        static constexpr uint8_t kOledDisplayAddress = 0x3c;
        static constexpr uint8_t kOledDisplayCommandContrast = 0x81;
        static constexpr uint8_t kOledDisplayCommandEntireRAM = 0xa4;
        static constexpr uint8_t kOledDisplayCommandEntireON = 0xa5;
        static constexpr uint8_t kOledDisplayCommandNormalDisplay = 0xa6;
        static constexpr uint8_t kOledDisplayCommandInverseDisplay = 0xa7;
        static constexpr uint8_t kOledDisplayCommandTurnOff = 0xae;
        static constexpr uint8_t kOledDisplayCommandTurnOn = 0xaf;
        static constexpr uint8_t kOledDisplayCommandRightHorizontalScroll = 0x26;
        static constexpr uint8_t kOledDisplayCommandLeftHorizontalScroll = 0x27;
        static constexpr uint8_t kOledDisplayCommandDeactivateScroll = 0x2e;
        static constexpr uint8_t kOledDisplayCommandActivateScroll = 0x2f;
        static constexpr uint8_t kOledDisplayMaskStartLine = 0x40;
        static constexpr uint8_t kOledDisplayCommandSetMultiplexRatio = 0xa8;
        static constexpr uint8_t kOledDisplayCommandSetDisplayClock = 0xd5;
        static constexpr uint8_t kOledDisplayCommandSetMemoryAddressingMode = 0x20;
        static constexpr uint8_t kOledDisplayCommandSetColumnAddress = 0x21;
        static constexpr uint8_t kOledDisplayCommandSetPageAddress = 0x22;
        static constexpr uint8_t kOledDisplayCommandSetDisplayOffset = 0xd3;
        static constexpr uint8_t kOledDisplayCommandSetSegmentRemapOff = 0xa0;
        static constexpr uint8_t kOledDisplayCommandSetSegmentRemapOn = 0xa1;
        static constexpr uint8_t kOledDisplayCommandSetCOMRemapOff = 0xc0;
        static constexpr uint8_t kOledDisplayCommandSetCOMRemapOn = 0xc8;
        static constexpr uint8_t kOledDisplayCommandSetCOMHWConfiguration = 0xda;
        static constexpr uint8_t kOledDisplayCommandSetPreChargePeriod = 0xd9;
        static constexpr uint8_t kOledDisplayCommandSetVCOMLevel = 0xdb;
};

int main() {
    OledDisplay oled;

    if (!oled.Init()) {
        std::cout << "Cannot initialize the OLED display\n";
        return 1;
    }

    oled.DrawString(std::string("I2C PROTOCOL!"));
    oled.Render();

    return 0;
}
