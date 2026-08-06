#include <hardware_protocols/spi/spi_ch347t.hpp>
#include <iostream>
#include <vector>

using namespace tvlinh::hardware_protocols;

#define CAN_MAX_LEN 8

struct CanFrame {
    public:
        uint32_t can_id_;
        uint8_t can_dlc_;
        uint8_t data[CAN_MAX_LEN];
};

class Mcp2515 {
    public:
        /*
         * Speed 8M
         */
        static constexpr uint8_t MCP_8MHz_1000kBPS_CFG1 = 0x00;
        static constexpr uint8_t MCP_8MHz_1000kBPS_CFG2 = 0x80;
        static constexpr uint8_t MCP_8MHz_1000kBPS_CFG3 = 0x80;

        static constexpr uint8_t MCP_8MHz_500kBPS_CFG1 = 0x00;
        static constexpr uint8_t MCP_8MHz_500kBPS_CFG2 = 0x90;
        static constexpr uint8_t MCP_8MHz_500kBPS_CFG3 = 0x82;

        static constexpr uint8_t MCP_8MHz_250kBPS_CFG1 = 0x00;
        static constexpr uint8_t MCP_8MHz_250kBPS_CFG2 = 0xB1;
        static constexpr uint8_t MCP_8MHz_250kBPS_CFG3 = 0x85;

        static constexpr uint8_t MCP_8MHz_200kBPS_CFG1 = 0x00;
        static constexpr uint8_t MCP_8MHz_200kBPS_CFG2 = 0xB4;
        static constexpr uint8_t MCP_8MHz_200kBPS_CFG3 = 0x86;

        static constexpr uint8_t MCP_8MHz_125kBPS_CFG1 = 0x01;
        static constexpr uint8_t MCP_8MHz_125kBPS_CFG2 = 0xB1;
        static constexpr uint8_t MCP_8MHz_125kBPS_CFG3 = 0x85;

        static constexpr uint8_t MCP_8MHz_100kBPS_CFG1 = 0x01;
        static constexpr uint8_t MCP_8MHz_100kBPS_CFG2 = 0xB4;
        static constexpr uint8_t MCP_8MHz_100kBPS_CFG3 = 0x86;

        static constexpr uint8_t MCP_8MHz_80kBPS_CFG1 = 0x01;
        static constexpr uint8_t MCP_8MHz_80kBPS_CFG2 = 0xBF;
        static constexpr uint8_t MCP_8MHz_80kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_8MHz_50kBPS_CFG1 = 0x03;
        static constexpr uint8_t MCP_8MHz_50kBPS_CFG2 = 0xB4;
        static constexpr uint8_t MCP_8MHz_50kBPS_CFG3 = 0x86;

        static constexpr uint8_t MCP_8MHz_40kBPS_CFG1 = 0x03;
        static constexpr uint8_t MCP_8MHz_40kBPS_CFG2 = 0xBF;
        static constexpr uint8_t MCP_8MHz_40kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_8MHz_33k3BPS_CFG1 = 0x47;
        static constexpr uint8_t MCP_8MHz_33k3BPS_CFG2 = 0xE2;
        static constexpr uint8_t MCP_8MHz_33k3BPS_CFG3 = 0x85;

        static constexpr uint8_t MCP_8MHz_31k25BPS_CFG1 = 0x07;
        static constexpr uint8_t MCP_8MHz_31k25BPS_CFG2 = 0xA4;
        static constexpr uint8_t MCP_8MHz_31k25BPS_CFG3 = 0x84;

        static constexpr uint8_t MCP_8MHz_20kBPS_CFG1 = 0x07;
        static constexpr uint8_t MCP_8MHz_20kBPS_CFG2 = 0xBF;
        static constexpr uint8_t MCP_8MHz_20kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_8MHz_10kBPS_CFG1 = 0x0F;
        static constexpr uint8_t MCP_8MHz_10kBPS_CFG2 = 0xBF;
        static constexpr uint8_t MCP_8MHz_10kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_8MHz_5kBPS_CFG1 = 0x1F;
        static constexpr uint8_t MCP_8MHz_5kBPS_CFG2 = 0xBF;
        static constexpr uint8_t MCP_8MHz_5kBPS_CFG3 = 0x87;

        /*
         * Speed 16M
         */
        static constexpr uint8_t MCP_16MHz_1000kBPS_CFG1 = 0x00;
        static constexpr uint8_t MCP_16MHz_1000kBPS_CFG2 = 0xD0;
        static constexpr uint8_t MCP_16MHz_1000kBPS_CFG3 = 0x82;

        static constexpr uint8_t MCP_16MHz_500kBPS_CFG1 = 0x00;
        static constexpr uint8_t MCP_16MHz_500kBPS_CFG2 = 0xF0;
        static constexpr uint8_t MCP_16MHz_500kBPS_CFG3 = 0x86;

        static constexpr uint8_t MCP_16MHz_250kBPS_CFG1 = 0x41;
        static constexpr uint8_t MCP_16MHz_250kBPS_CFG2 = 0xF1;
        static constexpr uint8_t MCP_16MHz_250kBPS_CFG3 = 0x85;

        static constexpr uint8_t MCP_16MHz_200kBPS_CFG1 = 0x01;
        static constexpr uint8_t MCP_16MHz_200kBPS_CFG2 = 0xFA;
        static constexpr uint8_t MCP_16MHz_200kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_16MHz_125kBPS_CFG1 = 0x03;
        static constexpr uint8_t MCP_16MHz_125kBPS_CFG2 = 0xF0;
        static constexpr uint8_t MCP_16MHz_125kBPS_CFG3 = 0x86;

        static constexpr uint8_t MCP_16MHz_100kBPS_CFG1 = 0x03;
        static constexpr uint8_t MCP_16MHz_100kBPS_CFG2 = 0xFA;
        static constexpr uint8_t MCP_16MHz_100kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_16MHz_95kBPS_CFG1 = 0x03;
        static constexpr uint8_t MCP_16MHz_95kBPS_CFG2 = 0xAD;
        static constexpr uint8_t MCP_16MHz_95kBPS_CFG3 = 0x07;

        static constexpr uint8_t MCP_16MHz_83k3BPS_CFG1 = 0x03;
        static constexpr uint8_t MCP_16MHz_83k3BPS_CFG2 = 0xBE;
        static constexpr uint8_t MCP_16MHz_83k3BPS_CFG3 = 0x07;

        static constexpr uint8_t MCP_16MHz_80kBPS_CFG1 = 0x03;
        static constexpr uint8_t MCP_16MHz_80kBPS_CFG2 = 0xFF;
        static constexpr uint8_t MCP_16MHz_80kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_16MHz_50kBPS_CFG1 = 0x07;
        static constexpr uint8_t MCP_16MHz_50kBPS_CFG2 = 0xFA;
        static constexpr uint8_t MCP_16MHz_50kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_16MHz_40kBPS_CFG1 = 0x07;
        static constexpr uint8_t MCP_16MHz_40kBPS_CFG2 = 0xFF;
        static constexpr uint8_t MCP_16MHz_40kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_16MHz_33k3BPS_CFG1 = 0x4E;
        static constexpr uint8_t MCP_16MHz_33k3BPS_CFG2 = 0xF1;
        static constexpr uint8_t MCP_16MHz_33k3BPS_CFG3 = 0x85;

        static constexpr uint8_t MCP_16MHz_20kBPS_CFG1 = 0x0F;
        static constexpr uint8_t MCP_16MHz_20kBPS_CFG2 = 0xFF;
        static constexpr uint8_t MCP_16MHz_20kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_16MHz_10kBPS_CFG1 = 0x1F;
        static constexpr uint8_t MCP_16MHz_10kBPS_CFG2 = 0xFF;
        static constexpr uint8_t MCP_16MHz_10kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_16MHz_5kBPS_CFG1 = 0x3F;
        static constexpr uint8_t MCP_16MHz_5kBPS_CFG2 = 0xFF;
        static constexpr uint8_t MCP_16MHz_5kBPS_CFG3 = 0x87;

        /*
         * Speed 20M
         */
        static constexpr uint8_t MCP_20MHz_1000kBPS_CFG1 = 0x00;
        static constexpr uint8_t MCP_20MHz_1000kBPS_CFG2 = 0xD9;
        static constexpr uint8_t MCP_20MHz_1000kBPS_CFG3 = 0x82;

        static constexpr uint8_t MCP_20MHz_500kBPS_CFG1 = 0x00;
        static constexpr uint8_t MCP_20MHz_500kBPS_CFG2 = 0xFA;
        static constexpr uint8_t MCP_20MHz_500kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_20MHz_250kBPS_CFG1 = 0x41;
        static constexpr uint8_t MCP_20MHz_250kBPS_CFG2 = 0xFB;
        static constexpr uint8_t MCP_20MHz_250kBPS_CFG3 = 0x86;

        static constexpr uint8_t MCP_20MHz_200kBPS_CFG1 = 0x01;
        static constexpr uint8_t MCP_20MHz_200kBPS_CFG2 = 0xFF;
        static constexpr uint8_t MCP_20MHz_200kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_20MHz_125kBPS_CFG1 = 0x03;
        static constexpr uint8_t MCP_20MHz_125kBPS_CFG2 = 0xFA;
        static constexpr uint8_t MCP_20MHz_125kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_20MHz_100kBPS_CFG1 = 0x04;
        static constexpr uint8_t MCP_20MHz_100kBPS_CFG2 = 0xFA;
        static constexpr uint8_t MCP_20MHz_100kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_20MHz_83k3BPS_CFG1 = 0x04;
        static constexpr uint8_t MCP_20MHz_83k3BPS_CFG2 = 0xFE;
        static constexpr uint8_t MCP_20MHz_83k3BPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_20MHz_80kBPS_CFG1 = 0x04;
        static constexpr uint8_t MCP_20MHz_80kBPS_CFG2 = 0xFF;
        static constexpr uint8_t MCP_20MHz_80kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_20MHz_50kBPS_CFG1 = 0x09;
        static constexpr uint8_t MCP_20MHz_50kBPS_CFG2 = 0xFA;
        static constexpr uint8_t MCP_20MHz_50kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_20MHz_40kBPS_CFG1 = 0x09;
        static constexpr uint8_t MCP_20MHz_40kBPS_CFG2 = 0xFF;
        static constexpr uint8_t MCP_20MHz_40kBPS_CFG3 = 0x87;

        static constexpr uint8_t MCP_20MHz_33k3BPS_CFG1 = 0x0B;
        static constexpr uint8_t MCP_20MHz_33k3BPS_CFG2 = 0xFF;
        static constexpr uint8_t MCP_20MHz_33k3BPS_CFG3 = 0x87;

        enum class CanClock {
            k20Mhz,
            k16Mhz,
            k8Mhz
        };

        enum class CanSpeed {
            k5Kbps,
            k10Kbps,
            k20Kbps,
            k31K25Bps,
            k33Kbps,
            k40Kbps,
            k50Kbps,
            k80Kbps,
            k83K3Bps,
            k95Kbps,
            k100Kbps,
            k125Kbps,
            k200Kbps,
            k250Kbps,
            k500Kbps,
            k1000Kbps
        };

        enum class CanClkout {
            kDisable = -1,
            kDiv1 = 0x0,
            kDiv2 = 0x1,
            kDiv4 = 0x2,
            kDiv8 = 0x3,
        };

        enum class Error {
            kOk = 0,
            kFail = 1,
            kAllTxBusy = 2,
            kFailInit = 3,
            kFailTx = 4,
            kNoMsg = 5
        };

        enum class Mask {
            kMask0,
            kMask1
        };

        enum class Rxf {
            kRxf0 = 0,
            kRxf1 = 1,
            kRxf2 = 2,
            kRxf3 = 3,
            kRxf4 = 4,
            kRxf5 = 5
        };

        enum class Rxbn {
            kRxb0 = 0,
            kRxb1 = 1
        };

        enum class Txbn {
            kTxb0 = 0,
            kTxb1 = 1,
            kTxb2 = 2
        };

        enum class CanIntf : uint8_t {
            kRx0If = 0x01,
            kRx1If = 0x02,
            kTx0If = 0x04,
            kTx1If = 0x08,
            kTx2If = 0x10,
            kErrIf = 0x20,
            kWakIf = 0x40,
            kMerrf = 0x80
        };

        enum class Eflg : uint8_t {
            kRx1Ovr = (1 << 7),
            kRx0Ovr = (1 << 6),
            kTxBo = (1 << 5),
            kTxEp = (1 << 4),
            kRxEp = (1 << 3),
            kTxWar = (1 << 2),
            kRxWar = (1 << 1),
            kEwarn = (1 << 0)
        };

    private:
        static constexpr uint8_t kCanCtrlReqop = 0xE0;
        static constexpr uint8_t kCanCtrlAbat = 0x10;
        static constexpr uint8_t kCanCtrlOsm = 0x08;
        static constexpr uint8_t kCanCtrlClken = 0x04;
        static constexpr uint8_t kCanCtrlClkpre = 0x03;

        enum class CanCtrlReqopMode : uint8_t {
            kNormal = 0x00,
            kOsm = 0x08,
            kSleep = 0x20,
            kLoopback = 0x40,
            kListenOnly = 0x60,
            kConfig = 0x80,
            kPowerup = 0xE0
        };

        static constexpr uint8_t kCanStatOpmod = 0xE0;
        static constexpr uint8_t kCanStatIcod = 0x0E;

        static constexpr uint8_t kCnf3Sof = 0x80;

        static constexpr uint8_t kTxbExideMask = 0x08;
        static constexpr uint8_t kDlcMask = 0x0F;
        static constexpr uint8_t kRtrMask = 0x40;

        static constexpr uint8_t kRxbnCtrlRxmStd = 0x20;
        static constexpr uint8_t kRxbnCtrlRxmExt = 0x40;
        static constexpr uint8_t kRxbnCtrlRxmStdExt = 0x00;
        static constexpr uint8_t kRxbnCtrlRxmMask = 0x60;
        static constexpr uint8_t kRxbnCtrlRtr = 0x08;
        static constexpr uint8_t kRxb0CtrlBukt = 0x04;
        static constexpr uint8_t kRxb0CtrlFilhitMask = 0x03;
        static constexpr uint8_t kRxb1CtrlFilhitMask = 0x07;
        static constexpr uint8_t kRxb0CtrlFilhit = 0x00;
        static constexpr uint8_t kRxb1CtrlFilhit = 0x01;

        static constexpr uint8_t kMcpSidh = 0;
        static constexpr uint8_t kMcpSidl = 1;
        static constexpr uint8_t kMcpEid8 = 2;
        static constexpr uint8_t kMcpEid0 = 3;
        static constexpr uint8_t kMcpDlc = 4;
        static constexpr uint8_t kMcpData = 5;

        enum class Stat : uint8_t {
            kRx0If = (1 << 0),
            kRx1If = (1 << 1)
        };

        static const uint8_t kStatRxifMask = static_cast<uint8_t>(Stat::kRx0If) | static_cast<uint8_t>(Stat::kRx1If);

        enum class TxbnCtrl : uint8_t {
            kAbtf = 0x40,
            kMloa = 0x20,
            kTxerr = 0x10,
            kTxreq = 0x08,
            kTxie = 0x04,
            kTxp = 0x03
        };

        static const uint8_t kEflgErrorMask = static_cast<uint8_t>(Eflg::kRx1Ovr) |
                                              static_cast<uint8_t>(Eflg::kRx0Ovr) |
                                              static_cast<uint8_t>(Eflg::kTxBo) |
                                              static_cast<uint8_t>(Eflg::kTxEp) |
                                              static_cast<uint8_t>(Eflg::kRxEp);

        enum class Instruction : uint8_t {
            kWrite = 0x02,
            kRead = 0x03,
            kBitmod = 0x05,
            kLoadTx0 = 0x40,
            kLoadTx1 = 0x42,
            kLoadTx2 = 0x44,
            kRtsTx0 = 0x81,
            kRtsTx1 = 0x82,
            kRtsTx2 = 0x84,
            kRtsAll = 0x87,
            kReadRx0 = 0x90,
            kReadRx1 = 0x94,
            kReadStatus = 0xA0,
            kRxStatus = 0xB0,
            kReset = 0xC0
        };

        enum class Register : uint8_t {
            kRxf0Sidh = 0x00,
            kRxf0Sidl = 0x01,
            kRxf0Eid8 = 0x02,
            kRxf0Eid0 = 0x03,
            kRxf1Sidh = 0x04,
            kRxf1Sidl = 0x05,
            kRxf1Eid8 = 0x06,
            kRxf1Eid0 = 0x07,
            kRxf2Sidh = 0x08,
            kRxf2Sidl = 0x09,
            kRxf2Eid8 = 0x0A,
            kRxf2Eid0 = 0x0B,
            kCanStat = 0x0E,
            kCanCtrl = 0x0F,
            kRxf3Sidh = 0x10,
            kRxf3Sidl = 0x11,
            kRxf3Eid8 = 0x12,
            kRxf3Eid0 = 0x13,
            kRxf4Sidh = 0x14,
            kRxf4Sidl = 0x15,
            kRxf4Eid8 = 0x16,
            kRxf4Eid0 = 0x17,
            kRxf5Sidh = 0x18,
            kRxf5Sidl = 0x19,
            kRxf5Eid8 = 0x1A,
            kRxf5Eid0 = 0x1B,
            kTec = 0x1C,
            kRec = 0x1D,
            kRxm0Sidh = 0x20,
            kRxm0Sidl = 0x21,
            kRxm0Eid8 = 0x22,
            kRxm0Eid0 = 0x23,
            kRxm1Sidh = 0x24,
            kRxm1Sidl = 0x25,
            kRxm1Eid8 = 0x26,
            kRxm1Eid0 = 0x27,
            kCnf3 = 0x28,
            kCnf2 = 0x29,
            kCnf1 = 0x2A,
            kCanInte = 0x2B,
            kCanIntf = 0x2C,
            kEflg = 0x2D,
            kTxb0Ctrl = 0x30,
            kTxb0Sidh = 0x31,
            kTxb0Sidl = 0x32,
            kTxb0Eid8 = 0x33,
            kTxb0Eid0 = 0x34,
            kTxb0Dlc = 0x35,
            kTxb0Data = 0x36,
            kTxb1Ctrl = 0x40,
            kTxb1Sidh = 0x41,
            kTxb1Sidl = 0x42,
            kTxb1Eid8 = 0x43,
            kTxb1Eid0 = 0x44,
            kTxb1Dlc = 0x45,
            kTxb1Data = 0x46,
            kTxb2Ctrl = 0x50,
            kTxb2Sidh = 0x51,
            kTxb2Sidl = 0x52,
            kTxb2Eid8 = 0x53,
            kTxb2Eid0 = 0x54,
            kTxb2Dlc = 0x55,
            kTxb2Data = 0x56,
            kRxb0Ctrl = 0x60,
            kRxb0Sidh = 0x61,
            kRxb0Sidl = 0x62,
            kRxb0Eid8 = 0x63,
            kRxb0Eid0 = 0x64,
            kRxb0Dlc = 0x65,
            kRxb0Data = 0x66,
            kRxb1Ctrl = 0x70,
            kRxb1Sidh = 0x71,
            kRxb1Sidl = 0x72,
            kRxb1Eid8 = 0x73,
            kRxb1Eid0 = 0x74,
            kRxb1Dlc = 0x75,
            kRxb1Data = 0x76
        };

        static constexpr uint32_t kDefaultSpiClock = 10000000;  // 10MHz

        static constexpr int kNumTxBuffers = 3;
        static constexpr int kNumRxBuffers = 2;

        struct TxbnRegs {
                Register ctrl;
                Register sidh;
                Register data;
        };

        struct RxbnRegs {
                Register ctrl;
                Register sidh;
                Register data;
                CanIntf can_intf_rxn_if;
        };

        static const struct TxbnRegs txb[kNumTxBuffers];
        static const struct RxbnRegs rxb[kNumRxBuffers];

        uint8_t spi_cs_;
        uint32_t spi_clock_;
        SPIController* spi;

    private:
        void StartSpi();
        void EndSpi();

        Error SetMode(const CanCtrlReqopMode mode);

        uint8_t ReadRegister(const Register reg);
        void ReadRegisters(const Register reg, uint8_t values[], const uint8_t n);
        void SetRegister(const Register reg, const uint8_t value);
        void SetRegisters(const Register reg, const uint8_t values[], const uint8_t n);
        void ModifyRegister(const Register reg, const uint8_t mask, const uint8_t data);

        void PrepareId(uint8_t* buffer, const bool ext, const uint32_t id);

    public:
        Mcp2515();
        Error Reset(void);
        Error SetConfigMode();
        Error SetListenOnlyMode();
        Error SetSleepMode();
        Error SetLoopbackMode();
        Error SetNormalMode();
        Error SetNormalOneShotMode();
        Error SetClkOut(const CanClkout divisor);
        Error SetBitrate(const CanSpeed can_speed);
        Error SetBitrate(const CanSpeed can_speed, const CanClock can_clock);
        Error SetFilterMask(const Mask num, const bool ext, const uint32_t ul_data);
        Error SetFilter(const Rxf num, const bool ext, const uint32_t ul_data);
        Error SendMessage(const Txbn txbn, const struct CanFrame* frame);
        Error SendMessage(const struct CanFrame* frame);
        Error ReadMessage(const Rxbn rxbn, struct CanFrame* frame);
        Error ReadMessage(struct CanFrame* frame);
        bool CheckReceive(void);
        bool CheckError(void);
        uint8_t GetErrorFlags(void);
        uint8_t GetControlRegister(void);
        void ClearRxnOvrFlags(void);
        uint8_t GetInterrupts(void);
        uint8_t GetInterruptMask(void);
        void ClearInterrupts(void);
        void ClearTxInterrupts(void);
        uint8_t GetStatus(void);
        void ClearRxnOvr(void);
        void ClearMerr();
        void ClearErrIf();
        uint8_t ErrorCountRx(void);
        uint8_t ErrorCountTx(void);
};

const struct Mcp2515::TxbnRegs Mcp2515::txb[Mcp2515::kNumTxBuffers] = {
    {Mcp2515::Register::kTxb0Ctrl, Mcp2515::Register::kTxb0Sidh, Mcp2515::Register::kTxb0Data},
    {Mcp2515::Register::kTxb1Ctrl, Mcp2515::Register::kTxb1Sidh, Mcp2515::Register::kTxb1Data},
    {Mcp2515::Register::kTxb2Ctrl, Mcp2515::Register::kTxb2Sidh, Mcp2515::Register::kTxb2Data}};

const struct Mcp2515::RxbnRegs Mcp2515::rxb[Mcp2515::kNumRxBuffers] = {
    {Mcp2515::Register::kRxb0Ctrl, Mcp2515::Register::kRxb0Sidh, Mcp2515::Register::kRxb0Data, Mcp2515::CanIntf::kRx0If},
    {Mcp2515::Register::kRxb1Ctrl, Mcp2515::Register::kRxb1Sidh, Mcp2515::Register::kRxb1Data, Mcp2515::CanIntf::kRx1If}};

Mcp2515::Mcp2515() {
}

int main() {
    SPIController* spi = new CH347TSPIController();

    if (!spi) {
        std::cout << "memory error\n";
        return 1;
    }

    if (!spi->Init()) {
        std::cout << "cannot initialize SPI controller\n";
        return 1;
    }

    SPIController::SPIConfiguration spi_config;
    spi_config.frequency_hz = 875000;
    spi_config.polarity = SPIController::SPIConfiguration::SPIPolarity::kLow;
    spi_config.phase = SPIController::SPIConfiguration::SPIPhase::kFirstEdge;
    spi_config.bit_order = SPIController::SPIConfiguration::SPIBitOrder::kMSBFirst;
    spi_config.cs_active = SPIController::SPIConfiguration::SPICSActive::kLow;
    spi_config.cs_pin = 1;

    if (!spi->SetConfiguration(spi_config)) {
        return false;
    }

    std::cout << "init done\n";

    std::vector<uint8_t> out_data{0x03, 0x0e, 0x00, 0x00, 0x00};
    std::vector<uint8_t> in_data(5);

    if (!spi->Transfer(out_data, in_data)) {
        std::cout << "cannot received data from SPI line\n";
        return 1;
    }

    for (auto e : in_data) {
        std::cout << std::hex << (int)e << std::endl;
    }

    std::cout << "done!\n";

    return 0;
}
