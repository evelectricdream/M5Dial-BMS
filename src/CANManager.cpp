// =============================================================================
// CANManager.cpp  v8
// Changes from v7:
//   - Added VW BMS protocol support (Type 5 modules)
//   - VW modules respond on 0x1CC+N, 3 frames per module with 4 cells each
//   - New accessors: getVWSlaveData(), getVWLastSeen()
// =============================================================================
#include "CANManager.h"
#include "bms_config.h"
#include "BMSModuleManager.h"
#include "Logger.h"
#include "WiFiManager.h"

extern BMSModuleManager bms;
extern EEPROMSettings   settings;

// PHEV CRC finalxor table — declared extern in bms_config.h
const uint8_t BMW_PHEV_FINAL_XOR[12] = {
    0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C
};

CANManager::CANManager()
    : running(false), rxTaskHandle(nullptr), phevCmdTaskHandle(nullptr),
      i3BusCmdTaskHandle(nullptr),
      lastChargerSeen(0), canCurrentA(0.0f), externalDeviceSeen(false),
      phevNextMod(0), phevMesCycle(0), phevTestCycle(0), phevBalCells(false),
      bmwI3Bus_counter(0), i3BusLastReplySeenMs(0), i3BusTxStartMs(0)
{
    memset(i3data,   0, sizeof(i3data));
    memset(i3acc,    0, sizeof(i3acc));
    memset(phevdata, 0, sizeof(phevdata));
    memset(phevAcc,  0, sizeof(phevAcc));
    memset(vwdata,   0, sizeof(vwdata));
    memset(vwacc,    0, sizeof(vwacc));
    phevCrc8.begin();
}
