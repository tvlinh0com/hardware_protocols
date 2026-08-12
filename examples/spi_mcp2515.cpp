#include <chrono>
#include <cstring>
#include <format>
#include <hardware_protocols/spi/spi_ch347t.hpp>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

using namespace tvlinh::hardware_protocols;

/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000UL /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000UL /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000UL /* error message frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFUL /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFUL /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFUL /* omit EFF, RTR, ERR flags */
/* CAN payload length and DLC definitions according to ISO 11898-1 */
#define CAN_MAX_DLC 8
#define CAN_MAX_DLEN 8

#define CAN_SFF_ID_BITS 11
#define CAN_EFF_ID_BITS 29

typedef uint32_t canid_t;

#pragma pack(push, 1)
struct CanFrame {
    public:
        canid_t can_id;
        uint8_t can_dlc;
        uint8_t data[CAN_MAX_DLEN];
};
#pragma pack(pop)

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

        static constexpr uint32_t kDefaultSpiClock = 10000000;  // 10MHz

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

        static const TxbnRegs txb[kNumTxBuffers];
        static const RxbnRegs rxb[kNumRxBuffers];

        std::unique_ptr<SPIController> spi_;

    private:
        unsigned long millis() {
            return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        }

        Error SetMode(const CanCtrlReqopMode mode);
        uint8_t ReadRegister(const Register reg);
        void ReadRegisters(const Register reg, uint8_t values[], const uint8_t n);
        void SetRegister(const Register reg, const uint8_t value);
        void SetRegisters(const Register reg, const uint8_t values[], const uint8_t n);
        void ModifyRegister(const Register reg, const uint8_t mask, const uint8_t data);
        void PrepareId(uint8_t* buffer, const bool ext, const uint32_t id);

    public:
        Mcp2515(std::unique_ptr<SPIController>& spi);
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
        Error SetFilterMask(const Mask mask, const bool ext, const uint32_t ul_data);
        Error SetFilter(const Rxf num, const bool ext, const uint32_t ul_data);
        Error SendMessage(const Txbn txbn, const CanFrame* frame);
        Error SendMessage(const CanFrame* frame);
        Error ReadMessage(const Rxbn rxbn, CanFrame* frame);
        Error ReadMessage(CanFrame* frame);
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

Mcp2515::Mcp2515(std::unique_ptr<SPIController>& spi) {
    this->spi_ = std::move(spi);
}

Mcp2515::Error Mcp2515::SetMode(const CanCtrlReqopMode mode) {
    this->ModifyRegister(Register::kCanCtrl, kCanCtrlReqop | kCanCtrlOsm, static_cast<uint8_t>(mode));

    unsigned long end_time = millis() + 10;
    bool mode_match = false;

    while (millis() < end_time) {
        uint8_t new_mode = this->ReadRegister(Register::kCanStat);
        new_mode &= kCanStatOpmod;

        mode_match = new_mode == static_cast<uint8_t>(mode);

        if (mode_match) {
            break;
        }
    }

    return mode_match ? Error::kOk : Error::kFail;
}

uint8_t Mcp2515::ReadRegister(const Register reg) {
    std::vector<uint8_t> out_data{static_cast<uint8_t>(Instruction::kRead), static_cast<uint8_t>(reg), 0x00};
    std::vector<uint8_t> in_data(3, 0x00);

    if (this->spi_->Transfer(out_data, in_data)) {
        return in_data[2];
    } else {
        return 0;
    }
}

void Mcp2515::ReadRegisters(const Register reg, uint8_t values[], const uint8_t n) {
    std::vector<uint8_t> out_data(n + 2, 0x00);
    std::vector<uint8_t> in_data(n + 2, 0x00);

    out_data[0] = static_cast<uint8_t>(Instruction::kRead);
    out_data[1] = static_cast<uint8_t>(reg);

    if (this->spi_->Transfer(out_data, in_data)) {
        std::copy(in_data.begin() + 2, in_data.end(), values);
    }
}

void Mcp2515::SetRegister(const Register reg, const uint8_t value) {
    std::vector<uint8_t> out_data{static_cast<uint8_t>(Instruction::kWrite), static_cast<uint8_t>(reg), value};
    this->spi_->Send(out_data);
}

void Mcp2515::SetRegisters(const Register reg, const uint8_t values[], const uint8_t n) {
    std::vector<uint8_t> out_data(2 + n, 0x00);
    out_data[0] = static_cast<uint8_t>(Instruction::kWrite);
    out_data[1] = static_cast<uint8_t>(reg);

    for (uint8_t i = 0; i < n; i++) {
        out_data[2 + i] = values[i];
    }

    this->spi_->Send(out_data);
}

void Mcp2515::ModifyRegister(const Register reg, const uint8_t mask, const uint8_t data) {
    std::vector<uint8_t> out_data{static_cast<uint8_t>(Instruction::kBitmod), static_cast<uint8_t>(reg), mask, data};
    this->spi_->Send(out_data);
}

void Mcp2515::PrepareId(uint8_t* buffer, const bool ext, const uint32_t id) {
    uint16_t can_id = (uint16_t)(id & 0x0ffff);

    if (ext) {
        buffer[kMcpEid0] = static_cast<uint8_t>(can_id & 0xff);
        buffer[kMcpEid8] = static_cast<uint8_t>(can_id >> 8);
        can_id = static_cast<uint16_t>(id >> 16);
        buffer[kMcpSidl] = static_cast<uint8_t>(can_id & 0x03);
        buffer[kMcpSidl] += static_cast<uint8_t>((can_id & 0x1c) << 3);
        buffer[kMcpSidl] |= kTxbExideMask;
        buffer[kMcpSidh] = static_cast<uint8_t>(can_id >> 5);
    } else {
        buffer[kMcpSidh] = static_cast<uint8_t>(can_id >> 3);
        buffer[kMcpSidl] = static_cast<uint8_t>((can_id & 0x07) << 5);
        buffer[kMcpEid0] = 0;
        buffer[kMcpEid8] = 0;
    }
}

Mcp2515::Error Mcp2515::Reset(void) {
    std::vector<uint8_t> out_data{static_cast<uint8_t>(Instruction::kReset)};
    this->spi_->Send(out_data);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    uint8_t zeros[14];
    std::memset(zeros, 0, sizeof zeros);

    this->SetRegisters(Register::kTxb0Ctrl, zeros, sizeof zeros);
    this->SetRegisters(Register::kTxb1Ctrl, zeros, sizeof zeros);
    this->SetRegisters(Register::kTxb2Ctrl, zeros, sizeof zeros);

    this->SetRegister(Register::kRxb0Ctrl, 0);
    this->SetRegister(Register::kRxb1Ctrl, 0);

    this->SetRegister(Register::kCanInte, static_cast<uint8_t>(CanIntf::kRx0If) |
                                              static_cast<uint8_t>(CanIntf::kRx1If) |
                                              static_cast<uint8_t>(CanIntf::kErrIf) |
                                              static_cast<uint8_t>(CanIntf::kMerrf));

    // Receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
    this->ModifyRegister(Register::kRxb0Ctrl, kRxbnCtrlRxmMask | kRxb0CtrlBukt | kRxb0CtrlFilhitMask,
                         kRxbnCtrlRxmStdExt | kRxb0CtrlBukt | kRxb0CtrlFilhit);

    // Clear filters and masks
    // Do not filter any standard frames for RXF0 used by RXB0
    // Do not filter any standard frames for RXF1 used by RXB1
    Rxf filters[] = {Rxf::kRxf0, Rxf::kRxf1, Rxf::kRxf2, Rxf::kRxf3, Rxf::kRxf4, Rxf::kRxf5};

    for (int i = 0; i < 6; i++) {
        bool ext = (i == 1);
        Error result = this->SetFilter(filters[i], ext, 0);

        if (result != Error::kOk) {
            return result;
        }
    }

    Mask masks[] = {Mask::kMask0, Mask::kMask1};
    for (int i = 0; i < 2; i++) {
        Error result = this->SetFilterMask(masks[i], true, 0);

        if (result != Error::kOk) {
            return result;
        }
    }

    return Error::kOk;
}

Mcp2515::Error Mcp2515::SetConfigMode() {
    return this->SetMode(CanCtrlReqopMode::kConfig);
}

Mcp2515::Error Mcp2515::SetListenOnlyMode() {
    return this->SetMode(CanCtrlReqopMode::kListenOnly);
}

Mcp2515::Error Mcp2515::SetSleepMode() {
    return this->SetMode(CanCtrlReqopMode::kSleep);
}

Mcp2515::Error Mcp2515::SetLoopbackMode() {
    return this->SetMode(CanCtrlReqopMode::kLoopback);
}

Mcp2515::Error Mcp2515::SetNormalMode() {
    return this->SetMode(CanCtrlReqopMode::kNormal);
}

Mcp2515::Error Mcp2515::SetNormalOneShotMode() {
    return this->SetMode(CanCtrlReqopMode::kOsm);
}

Mcp2515::Error Mcp2515::SetClkOut(const CanClkout divisor) {
    if (divisor == CanClkout::kDisable) {
        // Turn off CLKEN
        this->ModifyRegister(Register::kCanCtrl, kCanCtrlClken, 0x00);

        // Turn on CLKOUT for SOF
        this->ModifyRegister(Register::kCnf3, kCnf3Sof, kCnf3Sof);

        return Error::kOk;
    }

    // Set the prescaler (CLKPRE)
    this->ModifyRegister(Register::kCanCtrl, kCanCtrlClkpre, static_cast<uint8_t>(divisor));

    // Turn on CLKEN
    this->ModifyRegister(Register::kCanCtrl, kCanCtrlClken, kCanCtrlClken);

    // Turn off CLKOUT for SOF
    this->ModifyRegister(Register::kCnf3, kCnf3Sof, 0x00);

    return Error::kOk;
}

Mcp2515::Error Mcp2515::SetBitrate(const CanSpeed can_speed) {
    return this->SetBitrate(can_speed, CanClock::k16Mhz);
}

Mcp2515::Error Mcp2515::SetBitrate(const CanSpeed can_speed, const CanClock can_clock) {
    Error error = this->SetConfigMode();

    if (error != Error::kOk) {
        return error;
    }

    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;

    switch (can_clock) {
        case CanClock::k8Mhz:
            switch (can_speed) {
                case CanSpeed::k5Kbps:  // 5Kbps
                    cfg1 = MCP_8MHz_5kBPS_CFG1;
                    cfg2 = MCP_8MHz_5kBPS_CFG2;
                    cfg3 = MCP_8MHz_5kBPS_CFG3;
                    break;

                case CanSpeed::k10Kbps:  // 10Kbps
                    cfg1 = MCP_8MHz_10kBPS_CFG1;
                    cfg2 = MCP_8MHz_10kBPS_CFG2;
                    cfg3 = MCP_8MHz_10kBPS_CFG3;
                    break;

                case CanSpeed::k20Kbps:  // 20Kbps
                    cfg1 = MCP_8MHz_20kBPS_CFG1;
                    cfg2 = MCP_8MHz_20kBPS_CFG2;
                    cfg3 = MCP_8MHz_20kBPS_CFG3;
                    break;

                case CanSpeed::k31K25Bps:  // 31.25Kbps
                    cfg1 = MCP_8MHz_31k25BPS_CFG1;
                    cfg2 = MCP_8MHz_31k25BPS_CFG2;
                    cfg3 = MCP_8MHz_31k25BPS_CFG3;
                    break;

                case CanSpeed::k33Kbps:  // 33.333Kbps
                    cfg1 = MCP_8MHz_33k3BPS_CFG1;
                    cfg2 = MCP_8MHz_33k3BPS_CFG2;
                    cfg3 = MCP_8MHz_33k3BPS_CFG3;
                    break;

                case CanSpeed::k40Kbps:  // 40Kbps
                    cfg1 = MCP_8MHz_40kBPS_CFG1;
                    cfg2 = MCP_8MHz_40kBPS_CFG2;
                    cfg3 = MCP_8MHz_40kBPS_CFG3;
                    break;

                case CanSpeed::k50Kbps:  // 50Kbps
                    cfg1 = MCP_8MHz_50kBPS_CFG1;
                    cfg2 = MCP_8MHz_50kBPS_CFG2;
                    cfg3 = MCP_8MHz_50kBPS_CFG3;
                    break;

                case CanSpeed::k80Kbps:  // 80Kbps
                    cfg1 = MCP_8MHz_80kBPS_CFG1;
                    cfg2 = MCP_8MHz_80kBPS_CFG2;
                    cfg3 = MCP_8MHz_80kBPS_CFG3;
                    break;

                case CanSpeed::k100Kbps:  // 100Kbps
                    cfg1 = MCP_8MHz_100kBPS_CFG1;
                    cfg2 = MCP_8MHz_100kBPS_CFG2;
                    cfg3 = MCP_8MHz_100kBPS_CFG3;
                    break;

                case CanSpeed::k125Kbps:  // 125Kbps
                    cfg1 = MCP_8MHz_125kBPS_CFG1;
                    cfg2 = MCP_8MHz_125kBPS_CFG2;
                    cfg3 = MCP_8MHz_125kBPS_CFG3;
                    break;

                case CanSpeed::k200Kbps:  // 200Kbps
                    cfg1 = MCP_8MHz_200kBPS_CFG1;
                    cfg2 = MCP_8MHz_200kBPS_CFG2;
                    cfg3 = MCP_8MHz_200kBPS_CFG3;
                    break;

                case CanSpeed::k250Kbps:  // 250Kbps
                    cfg1 = MCP_8MHz_250kBPS_CFG1;
                    cfg2 = MCP_8MHz_250kBPS_CFG2;
                    cfg3 = MCP_8MHz_250kBPS_CFG3;
                    break;

                case CanSpeed::k500Kbps:  // 500Kbps
                    cfg1 = MCP_8MHz_500kBPS_CFG1;
                    cfg2 = MCP_8MHz_500kBPS_CFG2;
                    cfg3 = MCP_8MHz_500kBPS_CFG3;
                    break;

                case CanSpeed::k1000Kbps:  // 1000Kbps
                    cfg1 = MCP_8MHz_1000kBPS_CFG1;
                    cfg2 = MCP_8MHz_1000kBPS_CFG2;
                    cfg3 = MCP_8MHz_1000kBPS_CFG3;
                    break;

                default:
                    set = 0;
                    break;
            }
            break;

        case CanClock::k16Mhz:
            switch (can_speed) {
                case CanSpeed::k5Kbps:  // 5Kbps
                    cfg1 = MCP_16MHz_5kBPS_CFG1;
                    cfg2 = MCP_16MHz_5kBPS_CFG2;
                    cfg3 = MCP_16MHz_5kBPS_CFG3;
                    break;

                case CanSpeed::k10Kbps:  // 10Kbps
                    cfg1 = MCP_16MHz_10kBPS_CFG1;
                    cfg2 = MCP_16MHz_10kBPS_CFG2;
                    cfg3 = MCP_16MHz_10kBPS_CFG3;
                    break;

                case CanSpeed::k20Kbps:  // 20Kbps
                    cfg1 = MCP_16MHz_20kBPS_CFG1;
                    cfg2 = MCP_16MHz_20kBPS_CFG2;
                    cfg3 = MCP_16MHz_20kBPS_CFG3;
                    break;

                case CanSpeed::k33Kbps:  // 33.333Kbps
                    cfg1 = MCP_16MHz_33k3BPS_CFG1;
                    cfg2 = MCP_16MHz_33k3BPS_CFG2;
                    cfg3 = MCP_16MHz_33k3BPS_CFG3;
                    break;

                case CanSpeed::k40Kbps:  // 40Kbps
                    cfg1 = MCP_16MHz_40kBPS_CFG1;
                    cfg2 = MCP_16MHz_40kBPS_CFG2;
                    cfg3 = MCP_16MHz_40kBPS_CFG3;
                    break;

                case CanSpeed::k50Kbps:  // 50Kbps
                    cfg1 = MCP_16MHz_50kBPS_CFG1;
                    cfg2 = MCP_16MHz_50kBPS_CFG2;
                    cfg3 = MCP_16MHz_50kBPS_CFG3;
                    break;

                case CanSpeed::k80Kbps:  // 80Kbps
                    cfg1 = MCP_16MHz_80kBPS_CFG1;
                    cfg2 = MCP_16MHz_80kBPS_CFG2;
                    cfg3 = MCP_16MHz_80kBPS_CFG3;
                    break;

                case CanSpeed::k83K3Bps:  // 83.333Kbps
                    cfg1 = MCP_16MHz_83k3BPS_CFG1;
                    cfg2 = MCP_16MHz_83k3BPS_CFG2;
                    cfg3 = MCP_16MHz_83k3BPS_CFG3;
                    break;

                case CanSpeed::k95Kbps:  // 95Kbps
                    cfg1 = MCP_16MHz_95kBPS_CFG1;
                    cfg2 = MCP_16MHz_95kBPS_CFG2;
                    cfg3 = MCP_16MHz_95kBPS_CFG3;
                    break;

                case CanSpeed::k100Kbps:  // 100Kbps
                    cfg1 = MCP_16MHz_100kBPS_CFG1;
                    cfg2 = MCP_16MHz_100kBPS_CFG2;
                    cfg3 = MCP_16MHz_100kBPS_CFG3;
                    break;

                case CanSpeed::k125Kbps:  // 125Kbps
                    cfg1 = MCP_16MHz_125kBPS_CFG1;
                    cfg2 = MCP_16MHz_125kBPS_CFG2;
                    cfg3 = MCP_16MHz_125kBPS_CFG3;
                    break;

                case CanSpeed::k200Kbps:  // 200Kbps
                    cfg1 = MCP_16MHz_200kBPS_CFG1;
                    cfg2 = MCP_16MHz_200kBPS_CFG2;
                    cfg3 = MCP_16MHz_200kBPS_CFG3;
                    break;

                case CanSpeed::k250Kbps:  // 250Kbps
                    cfg1 = MCP_16MHz_250kBPS_CFG1;
                    cfg2 = MCP_16MHz_250kBPS_CFG2;
                    cfg3 = MCP_16MHz_250kBPS_CFG3;
                    break;

                case CanSpeed::k500Kbps:  // 500Kbps
                    cfg1 = MCP_16MHz_500kBPS_CFG1;
                    cfg2 = MCP_16MHz_500kBPS_CFG2;
                    cfg3 = MCP_16MHz_500kBPS_CFG3;
                    break;

                case CanSpeed::k1000Kbps:  // 1000Kbps
                    cfg1 = MCP_16MHz_1000kBPS_CFG1;
                    cfg2 = MCP_16MHz_1000kBPS_CFG2;
                    cfg3 = MCP_16MHz_1000kBPS_CFG3;
                    break;

                default:
                    set = 0;
                    break;
            }
            break;

        case CanClock::k20Mhz:
            switch (can_speed) {
                case CanSpeed::k33Kbps:  // 33.333Kbps
                    cfg1 = MCP_20MHz_33k3BPS_CFG1;
                    cfg2 = MCP_20MHz_33k3BPS_CFG2;
                    cfg3 = MCP_20MHz_33k3BPS_CFG3;
                    break;

                case CanSpeed::k40Kbps:  // 40Kbps
                    cfg1 = MCP_20MHz_40kBPS_CFG1;
                    cfg2 = MCP_20MHz_40kBPS_CFG2;
                    cfg3 = MCP_20MHz_40kBPS_CFG3;
                    break;

                case CanSpeed::k50Kbps:  // 50Kbps
                    cfg1 = MCP_20MHz_50kBPS_CFG1;
                    cfg2 = MCP_20MHz_50kBPS_CFG2;
                    cfg3 = MCP_20MHz_50kBPS_CFG3;
                    break;

                case CanSpeed::k80Kbps:  // 80Kbps
                    cfg1 = MCP_20MHz_80kBPS_CFG1;
                    cfg2 = MCP_20MHz_80kBPS_CFG2;
                    cfg3 = MCP_20MHz_80kBPS_CFG3;
                    break;

                case CanSpeed::k83K3Bps:  // 83.333Kbps
                    cfg1 = MCP_20MHz_83k3BPS_CFG1;
                    cfg2 = MCP_20MHz_83k3BPS_CFG2;
                    cfg3 = MCP_20MHz_83k3BPS_CFG3;
                    break;

                case CanSpeed::k100Kbps:  // 100Kbps
                    cfg1 = MCP_20MHz_100kBPS_CFG1;
                    cfg2 = MCP_20MHz_100kBPS_CFG2;
                    cfg3 = MCP_20MHz_100kBPS_CFG3;
                    break;

                case CanSpeed::k125Kbps:  // 125Kbps
                    cfg1 = MCP_20MHz_125kBPS_CFG1;
                    cfg2 = MCP_20MHz_125kBPS_CFG2;
                    cfg3 = MCP_20MHz_125kBPS_CFG3;
                    break;

                case CanSpeed::k200Kbps:  // 200Kbps
                    cfg1 = MCP_20MHz_200kBPS_CFG1;
                    cfg2 = MCP_20MHz_200kBPS_CFG2;
                    cfg3 = MCP_20MHz_200kBPS_CFG3;
                    break;

                case CanSpeed::k250Kbps:  // 250Kbps
                    cfg1 = MCP_20MHz_250kBPS_CFG1;
                    cfg2 = MCP_20MHz_250kBPS_CFG2;
                    cfg3 = MCP_20MHz_250kBPS_CFG3;
                    break;

                case CanSpeed::k500Kbps:  // 500Kbps
                    cfg1 = MCP_20MHz_500kBPS_CFG1;
                    cfg2 = MCP_20MHz_500kBPS_CFG2;
                    cfg3 = MCP_20MHz_500kBPS_CFG3;
                    break;

                case CanSpeed::k1000Kbps:  // 1000Kbps
                    cfg1 = MCP_20MHz_1000kBPS_CFG1;
                    cfg2 = MCP_20MHz_1000kBPS_CFG2;
                    cfg3 = MCP_20MHz_1000kBPS_CFG3;
                    break;

                default:
                    set = 0;
                    break;
            }
            break;

        default:
            set = 0;
            break;
    }

    if (set) {
        this->SetRegister(Register::kCnf1, cfg1);
        this->SetRegister(Register::kCnf2, cfg2);
        this->SetRegister(Register::kCnf3, cfg3);
        return Error::kOk;
    } else {
        return Error::kFail;
    }
}

Mcp2515::Error Mcp2515::SetFilterMask(const Mask mask, const bool ext, const uint32_t ul_data) {
    Error res = this->SetConfigMode();
    if (res == Error::kOk) {
        return res;
    }

    uint8_t tbuf_data[4];
    this->PrepareId(tbuf_data, ext, ul_data);

    Register reg;
    switch (mask) {
        case Mask::kMask0:
            reg = Register::kRxm0Sidh;
            break;

        case Mask::kMask1:
            reg = Register::kRxm1Sidh;
            break;

        default:
            return Error::kFail;
    }

    this->SetRegisters(reg, tbuf_data, sizeof tbuf_data / sizeof tbuf_data[0]);
    return Error::kOk;
}

Mcp2515::Error Mcp2515::SetFilter(const Rxf num, const bool ext, const uint32_t ul_data) {
    Error res = this->SetConfigMode();

    if (res != Error::kOk) {
        return res;
    }

    Register reg;

    switch (num) {
        case Rxf::kRxf0:
            reg = Register::kRxf0Sidh;
            break;
        case Rxf::kRxf1:
            reg = Register::kRxf1Sidh;
            break;
        case Rxf::kRxf2:
            reg = Register::kRxf2Sidh;
            break;
        case Rxf::kRxf3:
            reg = Register::kRxf3Sidh;
            break;
        case Rxf::kRxf4:
            reg = Register::kRxf4Sidh;
            break;
        case Rxf::kRxf5:
            reg = Register::kRxf5Sidh;
            break;
        default:
            return Error::kFail;
    }

    uint8_t tbuf_data[4];
    this->PrepareId(tbuf_data, ext, ul_data);
    this->SetRegisters(reg, tbuf_data, sizeof tbuf_data / sizeof tbuf_data[0]);

    return Error::kOk;
}

Mcp2515::Error Mcp2515::SendMessage(const Txbn txbn, const CanFrame* frame) {
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return Error::kFailTx;
    }

    const TxbnRegs* txb_regs = &txb[static_cast<uint8_t>(txbn)];
    uint8_t data[13];

    bool ext = (frame->can_id & CAN_EFF_FLAG);
    bool rtr = (frame->can_id & CAN_RTR_FLAG);
    uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

    this->PrepareId(data, ext, id);
    data[kMcpDlc] = rtr ? (frame->can_dlc | kRtrMask) : frame->can_dlc;
    std::memcpy(&data[kMcpData], frame->data, frame->can_dlc);
    this->SetRegisters(txb_regs->sidh, data, 5 + frame->can_dlc);
    this->ModifyRegister(txb_regs->ctrl, static_cast<uint8_t>(TxbnCtrl::kTxreq), static_cast<uint8_t>(TxbnCtrl::kTxreq));

    uint8_t ctrl = this->ReadRegister(txb_regs->ctrl);

    if ((ctrl & (static_cast<uint8_t>(TxbnCtrl::kAbtf) | static_cast<uint8_t>(TxbnCtrl::kMloa) | static_cast<uint8_t>(TxbnCtrl::kTxerr))) != 0) {
        return Error::kFailTx;
    }

    return Error::kOk;
}

Mcp2515::Error Mcp2515::SendMessage(const CanFrame* frame) {
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return Error::kFailTx;
    }

    Txbn tx_buffers[kNumTxBuffers] = {Txbn::kTxb0, Txbn::kTxb1, Txbn::kTxb2};

    for (int i = 0; i < kNumTxBuffers; i++) {
        const TxbnRegs* txb_regs = &txb[static_cast<uint8_t>(tx_buffers[i])];
        uint8_t ctrlval = this->ReadRegister(txb_regs->ctrl);

        if ((ctrlval & static_cast<uint8_t>(TxbnCtrl::kTxreq)) == 0) {
            return this->SendMessage(tx_buffers[i], frame);
        }
    }

    return Error::kAllTxBusy;
}

Mcp2515::Error Mcp2515::ReadMessage(const Rxbn rxbn, CanFrame* frame) {
    const RxbnRegs* rxb_regs = &rxb[static_cast<uint8_t>(rxbn)];
    uint8_t tbuf_data[5];

    this->ReadRegisters(rxb_regs->sidh, tbuf_data, 5);
    uint32_t id = (tbuf_data[kMcpSidh] << 3) + (tbuf_data[kMcpSidl] >> 5);

    if ((tbuf_data[kMcpSidl] & kTxbExideMask) == kTxbExideMask) {
        id = (id << 2) + (tbuf_data[kMcpSidl] & 0x03);
        id = (id << 8) + tbuf_data[kMcpEid8];
        id = (id << 8) + tbuf_data[kMcpEid0];
        id |= CAN_EFF_FLAG;
    }

    uint8_t dlc = (tbuf_data[kMcpDlc] & kDlcMask);

    if (dlc > CAN_MAX_DLEN) {
        return Error::kFail;
    }

    uint8_t ctrl = this->ReadRegister(rxb_regs->ctrl);

    if (ctrl & kRxbnCtrlRtr) {
        id |= CAN_RTR_FLAG;
    }

    frame->can_id = id;
    frame->can_dlc = dlc;

    this->ReadRegisters(rxb_regs->data, frame->data, dlc);
    this->ModifyRegister(Register::kCanIntf, static_cast<uint8_t>(rxb_regs->can_intf_rxn_if), 0);

    return Error::kOk;
}

Mcp2515::Error Mcp2515::ReadMessage(CanFrame* frame) {
    Error rc;
    uint8_t stat = this->GetStatus();

    if (stat & static_cast<uint8_t>(Stat::kRx0If)) {
        rc = this->ReadMessage(Rxbn::kRxb0, frame);
    } else if (stat & static_cast<uint8_t>(Stat::kRx1If)) {
        rc = this->ReadMessage(Rxbn::kRxb1, frame);
    } else {
        rc = Error::kNoMsg;
    }

    return rc;
}

bool Mcp2515::CheckReceive(void) {
    uint8_t res = this->GetStatus();

    if (res & kStatRxifMask) {
        return true;
    } else {
        return false;
    }
}

bool Mcp2515::CheckError(void) {
    uint8_t eflg = this->GetErrorFlags();

    if (eflg & kEflgErrorMask) {
        return true;
    } else {
        return false;
    }
}

uint8_t Mcp2515::GetErrorFlags(void) {
    return this->ReadRegister(Register::kEflg);
}

uint8_t Mcp2515::GetControlRegister(void) {
    return this->ReadRegister(Register::kCanCtrl);
}

void Mcp2515::ClearRxnOvrFlags(void) {
    this->ModifyRegister(Register::kEflg,
                         static_cast<uint8_t>(Eflg::kRx0Ovr) | static_cast<uint8_t>(Eflg::kRx1Ovr),
                         0);
}

uint8_t Mcp2515::GetInterrupts(void) {
    return this->ReadRegister(Register::kCanIntf);
}

uint8_t Mcp2515::GetInterruptMask(void) {
    return this->ReadRegister(Register::kCanInte);
}

void Mcp2515::ClearInterrupts(void) {
    this->SetRegister(Register::kCanIntf, 0);
}

void Mcp2515::ClearTxInterrupts(void) {
    this->ModifyRegister(Register::kCanIntf,
                         static_cast<uint8_t>(CanIntf::kTx0If) | static_cast<uint8_t>(CanIntf::kTx1If) | static_cast<uint8_t>(CanIntf::kTx2If),
                         0);
}

uint8_t Mcp2515::GetStatus(void) {
    std::vector<uint8_t> out_data{static_cast<uint8_t>(Instruction::kReadStatus), 0x00};
    std::vector<uint8_t> in_data(out_data.size(), 0x00);

    if (this->spi_->Transfer(out_data, in_data)) {
        return in_data[1];
    } else {
        return 0;
    }
}

void Mcp2515::ClearRxnOvr(void) {
    uint8_t eflg = this->GetErrorFlags();

    if (eflg != 0) {
        this->ClearRxnOvrFlags();
        this->ClearInterrupts();
    }
}

void Mcp2515::ClearMerr() {
    this->ModifyRegister(Register::kCanIntf, static_cast<uint8_t>(CanIntf::kMerrf), 0);
}

void Mcp2515::ClearErrIf() {
    this->ModifyRegister(Register::kCanIntf, static_cast<uint8_t>(CanIntf::kErrIf), 0);
}

uint8_t Mcp2515::ErrorCountRx(void) {
    return this->ReadRegister(Register::kRec);
}

uint8_t Mcp2515::ErrorCountTx(void) {
    return this->ReadRegister(Register::kTec);
}

int main() {
    std::unique_ptr<SPIController> spi = std::make_unique<CH347TSPIController>();
    if (!spi) {
        std::cout << "memory error\n";
        return 1;
    }

    if (!spi->Init()) {
        std::cout << "cannot initialize SPI controller\n";
        return 1;
    }

    SPIController::SPIConfiguration spi_config;
    spi_config.frequency_hz = Mcp2515::kDefaultSpiClock;
    spi_config.polarity = SPIController::SPIConfiguration::SPIPolarity::kLow;
    spi_config.phase = SPIController::SPIConfiguration::SPIPhase::kFirstEdge;
    spi_config.bit_order = SPIController::SPIConfiguration::SPIBitOrder::kMSBFirst;
    spi_config.cs_active = SPIController::SPIConfiguration::SPICSActive::kLow;
    spi_config.cs_pin = 1;

    if (!spi->SetConfiguration(spi_config)) {
        return 1;
    }

    std::cout << "Initialized SPI controller successfully\n";

    // This program only performs the monitoring CAN bus and write data to terminal
    Mcp2515 mcp2515(spi);

    mcp2515.Reset();

    Mcp2515::Error result = mcp2515.SetBitrate(Mcp2515::CanSpeed::k250Kbps, Mcp2515::CanClock::k8Mhz);

    if (result != Mcp2515::Error::kOk) {
        std::cout << "Failed to configure MCP2515 Bit rate\n";
        return 1;
    }

    std::cout << "Configured MCP2515 Bit rate successfully\n";

    result = mcp2515.SetNormalMode();

    if (result != Mcp2515::Error::kOk) {
        std::cout << "Failed to set MCP2515 to normal mode\n";
        return 1;
    }

    CanFrame can_msg;

    while (true) {
        if (mcp2515.ReadMessage(&can_msg) == Mcp2515::Error::kOk) {
            std::cout << std::format("ID: {:X} | DLC: {} | DATA: ", can_msg.can_id, can_msg.can_dlc);

            for (int i = 0; i < can_msg.can_dlc; ++i) {
                std::cout << std::format("0x{:X} ", can_msg.data[i]);
            }

            std::cout << std::endl;
        }
    }

    return 0;
}
