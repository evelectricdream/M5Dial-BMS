#pragma once
// =============================================================================
// VWProtocol.h - VW BMS CAN frame decoding
// 
// Decodes CAN frames from VW type 5 modules and extracts cell voltages.
// Protocol details from VW-bms repository (Tom-evnut).
// 
// Frame structure:
//   Control ID: 0x0BA (used to poll modules)
//   Module ID start: 0x1CC (module 0 = 0x1CC, module 1 = 0x1CD, etc.)
//   Cell voltage frames: 3 frames per module, 4 cells per frame
//   Total: 12 cells per module
// =============================================================================

#include <Arduino.h>

// VW CAN protocol constants
#define VW_CONTROL_ID           0x0BA      // Master command ID
#define VW_MODULE_ID_START      0x1CC      // First module ID (0x1CC for module 0)
#define VW_MAX_MODULES          30         // Hardware max: 30 modules in theory
#define VW_CELLS_PER_MODULE     12         // 12 cells per module
#define VW_FRAMES_PER_MODULE    3          // 3 CAN frames per module (4 cells each)
#define VW_CMD_INTERVAL_MS      100        // Poll interval per module
#define VW_TIMEOUT_MS           3000       // Module offline timeout

// Extended VW IDs used by VWBMSV2 reference
#define VW_BAL_STATUS_ID_MIN    0x16A95471UL
#define VW_BAL_STATUS_ID_MAX    0x16A9547FUL
#define VW_TEMP_TYPE1_ID_MIN    0x1A555401UL
#define VW_TEMP_TYPE1_ID_MAX    0x1A55543FUL
#define VW_TEMP_TYPE2_ID_MIN    0x1A5555F0UL
#define VW_TEMP_TYPE2_ID_MAX    0x1A5555FEUL

// Voltage encoding: uint16_t little-endian cell values
// Actual voltage (mV) = raw + 1000

// VW slave staging struct
struct VWSlaveData {
    float    cellV[VW_CELLS_PER_MODULE];   // 12 cell voltages in V
    float    temp[2];                       // 2 temperatures exposed in M5Dial UI
    float    minCellV;                      // Minimum cell voltage
    float    maxCellV;                      // Maximum cell voltage
    uint32_t balStat;                       // Balance state/status bits from CAN
    bool     fresh;                         // True if data updated this cycle
    uint32_t lastSeenMs;                    // Last update timestamp
};

// Internal accumulator for multi-frame reconstruction
struct VWCellAccumulator {
    float   cells[VW_CELLS_PER_MODULE];
    uint8_t framesRx;                       // Bits 0-2 track which frames received
                                            // Bit pattern: if all 3 frames = 0x07
};
