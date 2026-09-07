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
//   Total: 12 cells per module (min/max 4-14)
// =============================================================================

#include <Arduino.h>

// VW CAN protocol constants
#define VW_CONTROL_ID           0x0BA      // Master command ID
#define VW_MODULE_ID_START      0x1CC      // First module ID (0x1CC for module 0)
#define VW_MAX_MODULES          30         // Hardware max: 30 modules in theory
#define VW_CELLS_PER_MODULE     12         // 12 cells per module
#define VW_FRAMES_PER_MODULE    3          // 3 CAN frames per module (4 cells each)

// Voltage encoding: stored as 12-bit value in 8 bits
// Actual voltage = (decoded_value + 1000) mV
// This gives us ~1V offset and 4mV resolution per LSB

// VW slave staging struct
struct VWSlaveData {
    float    cellV[VW_CELLS_PER_MODULE];   // 12 cell voltages in V
    float    minCellV;                      // Minimum cell voltage
    float    maxCellV;                      // Maximum cell voltage
    bool     fresh;                         // True if data updated this cycle
    uint32_t lastSeenMs;                    // Last update timestamp
};

// Internal accumulator for multi-frame reconstruction
struct VWCellAccumulator {
    float   cells[VW_CELLS_PER_MODULE];
    uint8_t framesRx;                       // Bits 0-2 track which frames received
                                            // Bit pattern: if all 3 frames = 0x07
};

#endif
