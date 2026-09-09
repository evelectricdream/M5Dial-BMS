#pragma once
// =============================================================================
// BMSModuleManager.h
// v5: added setBalanceInhibit(), sendCANFrames(), auto-balance scheduling
// =============================================================================
#include "bms_config.h"
#include "BMSModule.h"
#include "CANManager.h"

class BMSModuleManager
{
public:
    BMSModuleManager();

    // Board enumeration
    void setupBoards();
    void findBoards();
    void renumberBoardIDs();

    // Runtime operations
    void clearFaults();
    void sleepBoards();
    void wakeBoards();
    void getAllVoltTemp();
    void balanceCells();

    // Configuration
    void readSetpoints();
    void setBatteryID(int id);
    void setPstrings(int Pstrings);
    void setSensors(int sensor, float Ignore);

    // v5: balancing inhibit (drive mode)
    void setBalanceInhibit(bool inhibit);
    bool getBalanceInhibit();

    // v5: enable/disable auto-balancing
    void setAutoBalance(bool enabled);
    bool getAutoBalance();

    // Pack-level data
    float getPackVoltage();
    float getAvgTemperature();
    float getHighTemperature();
    float getLowTemperature();
    float getAvgCellVolt();
    float getLowCellVolt();
    float getHighCellVolt();
    float getLowVoltage();
    float getHighVoltage();
    int   getNumModules();
    bool  isFaultedState();
    // Returns the active cell count for a specific module address.
    int   getModuleCells(int addr);
    // BMW i3 CAN-sourced data path (called instead of UART getAllVoltTemp)
    void  getAllVoltTempFromCAN();
    // BMW PHEV CAN-sourced data path
    void  getAllVoltTempFromPHEV();
    // VW CAN-sourced data path
    void  getAllVoltTempFromVW();

    // Per-module accessor
    BMSModule& getModule(int addr);

    // Serial console debug
    void printPackSummary();
    void printPackDetails();

private:
    float     packVolt;
    int       Pstring;
    float     LowCellVolt;
    float     HighCellVolt;
    float     lowestPackVolt;
    float     highestPackVolt;
    float     lowestPackTemp;
    float     highestPackTemp;
    BMSModule modules[MAX_MODULE_ADDR + 1];
    int       batteryID;
    int       numFoundModules;
    bool      isFaulted;
    bool      balanceInhibit;   // true = drive mode, skip balancing
    bool      autoBalance;      // true = balance automatically each cycle
};
