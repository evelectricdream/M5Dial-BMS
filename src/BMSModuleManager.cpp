// =============================================================================
// BMSModuleManager.cpp - Tesla BMS pack-level manager
// Ported for M5Dial ESP32-S3:
//   - #include "config.h" -> #include "bms_config.h"
//   - Removed #include "pin_config.h" (T-Display-S3 specific)
//   - getAllVoltTemp(): replaced digitalRead(11) hardware fault pin with
//     software fault detection from UART register data (getFaults())
//   - printPackSummary/Details: no SerialUSB direct calls, all via SERIALCONSOLE
//   - CAN functions remain commented out (unchanged from ronaegis port)
//   - Added isFaultedState() and getNumModules() for display layer
//   - Added getModule() accessor for display layer
//   - bms_status and bms_modules_text globals populated for M5Dial display
// =============================================================================
#include "bms_config.h"
#include "BMSModuleManager.h"
#include "BMSUtil.h"
#include "Logger.h"
#include "CANManager.h"
#include <EEPROM.h>

extern EEPROMSettings settings;
extern CANManager     can;

// Global strings read by the display layer in the main .ino
String bms_status;
String bms_modules_text;

// ---------------------------------------------------------------------------
// SoC estimation - simple linear interpolation on pack voltage
// Divides by BMS_NUM_SERIES to get per-module voltage, scales 18.0-25.2V range
// ---------------------------------------------------------------------------
static float getSoC(float packV)
{
    extern EEPROMSettings settings;
    int   ns  = (settings.numSeries > 0)  ? settings.numSeries  : BMS_NUM_SERIES;
    float lo  = (settings.socLo    > 0.0f) ? settings.socLo     : 18.0f;
    float hi  = (settings.socHi    > lo)   ? settings.socHi     : 25.2f;
    float v   = packV / (float)ns;
    v = (v - lo) * 100.0f / (hi - lo);
    if (v > 100.0f) v = 100.0f;
    if (v < 0.0f)   v = 0.0f;
    return v;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
BMSModuleManager::BMSModuleManager()
{
    for (int i = 1; i <= MAX_MODULE_ADDR; i++) {
        modules[i].setExists(false);
        modules[i].setAddress(i);
    }
    lowestPackVolt  = 1000.0f;
    highestPackVolt = 0.0f;
    lowestPackTemp  = 200.0f;
    highestPackTemp = -100.0f;
    LowCellVolt     = 5.0f;
    HighCellVolt    = 0.0f;
    packVolt        = 0.0f;
    Pstring         = BMS_NUM_PARALLEL;
    batteryID       = 1;
    numFoundModules = 0;
    isFaulted       = false;
    balanceInhibit  = false;
    autoBalance     = false;
}

// ---------------------------------------------------------------------------
// balanceCells - send balance commands to each module that needs it
// Skipped entirely when balanceInhibit is true (drive mode active).
// ---------------------------------------------------------------------------
void BMSModuleManager::balanceCells()
{
    if (balanceInhibit) {
        Logger::info("Balancing inhibited (drive mode active)");
        return;
    }

    if (settings.cmuType == CMU_VW_BMS) {
        float packLow = getLowCellVolt();
        for (int address = 1; address <= MAX_MODULE_ADDR && address <= 12; address++) {
            if (!modules[address].isExisting()) continue;
            uint16_t balanceMask = 0;
            int modCells = getModuleCells(address);
            for (int i = 0; i < modCells && i < VW_CELLS_PER_MODULE; i++) {
                float cv = modules[address].getCellVoltage(i);
                if (cv > settings.IgnoreVolt && cv > (packLow + settings.balanceHyst))
                    balanceMask |= (uint16_t)(1U << i);
            }
            can.sendVWBalanceCommand((uint8_t)address, balanceMask);
        }
        return;
    }

    uint8_t payload[4];
    uint8_t buff[30];
    uint8_t balance = 0;

    for (int address = 1; address <= MAX_MODULE_ADDR; address++)
    {
        if (modules[address].isExisting())
        {
            balance = 0;
            float packLow = getLowCellVolt(); // already excludes ignored cells
            int modCells = getModuleCells(address); // per-module or global default
            for (int i = 0; i < modCells; i++)
            {
                float cv = modules[address].getCellVoltage(i);
                // Skip cells below IgnoreCell threshold (unpopulated/bonded cells)
                // IgnoreCell is stored per-module; use same threshold as getLowCellV()
                if (cv > settings.IgnoreVolt && cv > packLow)
                    balance |= (1 << i);
            }

            if (balance != 0)
            {
                payload[0] = address << 1;
                payload[1] = REG_BAL_TIME;
                payload[2] = 60; // 60-second balance limit
                BMSUtil::sendData(payload, 3, true);
                delay(2);
                BMSUtil::getReply(buff, 30);

                payload[0] = address << 1;
                payload[1] = REG_BAL_CTRL;
                payload[2] = balance;
                BMSUtil::sendData(payload, 3, true);
                delay(2);
                BMSUtil::getReply(buff, 30);

                if (Logger::isDebug())
                {
                    delay(50);
                    payload[0] = address << 1;
                    payload[1] = REG_BAL_TIME;
                    payload[2] = 1;
                    BMSUtil::sendData(payload, 3, false);
                    delay(2);
                    BMSUtil::getReply(buff, 30);

                    payload[0] = address << 1;
                    payload[1] = REG_BAL_CTRL;
                    payload[2] = 1;
                    BMSUtil::sendData(payload, 3, false);
                    delay(2);
                    BMSUtil::getReply(buff, 30);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// setupBoards - assign addresses to any un-addressed module on the bus
// Called by renumberBoardIDs() after broadcast reset.
// ---------------------------------------------------------------------------
void BMSModuleManager::setupBoards()
{
    uint8_t payload[3];
    uint8_t buff[10];
    int retLen;

    while (1 == 1)
    {
        payload[0] = 0;
        payload[1] = 0;
        payload[2] = 1;
        retLen = BMSUtil::sendDataWithReply(payload, 3, false, buff, 4);
        if (retLen == 4)
        {
            if (buff[0] == 0x80 && buff[1] == 0 && buff[2] == 1)
            {
                Logger::debug("Unaddressed module found");
                for (int y = 1; y <= MAX_MODULE_ADDR; y++)
                {
                    if (!modules[y].isExisting())
                    {
                        payload[0] = 0;
                        payload[1] = REG_ADDR_CTRL;
                        payload[2] = y | 0x80;
                        BMSUtil::sendData(payload, 3, true);
                        delay(3);
                        if (BMSUtil::getReply(buff, 10) > 2)
                        {
                            if (buff[0] == 0x81 && buff[1] == REG_ADDR_CTRL
                                && buff[2] == (y + 0x80))
                            {
                                modules[y].setExists(true);
                                numFoundModules++;
                                Logger::debug("Address %i assigned", y);
                            }
                        }
                        break;
                    }
                }
            }
            else break;
        }
        else break;
    }
}

// ---------------------------------------------------------------------------
// findBoards - poll all 62 possible addresses, mark which ones respond
// ---------------------------------------------------------------------------
void BMSModuleManager::findBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];

    numFoundModules = 0;
    payload[1] = 0;
    payload[2] = 1;

    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        modules[x].setExists(false);
        payload[0] = x << 1;
        BMSUtil::sendData(payload, 3, false);
        delay(20);
        if (BMSUtil::getReply(buff, 8) > 4)
        {
            if (buff[0] == (x << 1) && buff[1] == 0 && buff[2] == 1 && buff[4] > 0)
            {
                modules[x].setExists(true);
                numFoundModules++;
                Logger::debug("Found module at address: %X", x);
            }
        }
        delay(5);
    }
    Logger::info("findBoards complete: %i modules found", numFoundModules);
}

// ---------------------------------------------------------------------------
// renumberBoardIDs - broadcast reset then re-enumerate from address 1
// ---------------------------------------------------------------------------
void BMSModuleManager::renumberBoardIDs()
{
    uint8_t payload[3];
    uint8_t buff[8];
    int attempts = 1;

    for (int y = 1; y <= MAX_MODULE_ADDR; y++) {
        modules[y].setExists(false);
        numFoundModules = 0;
    }

    while (attempts < 3)
    {
        payload[0] = 0x3F << 1; // broadcast
        payload[1] = 0x3C;       // reset command
        payload[2] = 0xA5;       // reset magic byte
        BMSUtil::sendData(payload, 3, true);
        delay(100);
        BMSUtil::getReply(buff, 8);
        if (buff[0] == 0x7F && buff[1] == 0x3C
            && buff[2] == 0xA5 && buff[3] == 0x57) break;
        attempts++;
    }

    setupBoards();
}

// ---------------------------------------------------------------------------
// clearFaults - write then clear both alert and fault status registers
// ---------------------------------------------------------------------------
void BMSModuleManager::clearFaults()
{
    uint8_t payload[3];
    uint8_t buff[8];

    payload[0] = 0x7F;
    payload[1] = REG_ALERT_STATUS;
    payload[2] = 0xFF;
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);
    payload[2] = 0x00;
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[1] = REG_FAULT_STATUS;
    payload[2] = 0xFF;
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);
    payload[2] = 0x00;
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    isFaulted = false;
}

// ---------------------------------------------------------------------------
// sleepBoards / wakeBoards
// ---------------------------------------------------------------------------
void BMSModuleManager::sleepBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F;
    payload[1] = REG_IO_CTRL;
    payload[2] = 0x04; // set sleep bit
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
}

void BMSModuleManager::wakeBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];

    payload[0] = 0x7F;
    payload[1] = REG_IO_CTRL;
    payload[2] = 0x00; // clear sleep bit
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);

    payload[1] = REG_ALERT_STATUS;
    payload[2] = 0x04;
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);

    payload[2] = 0x00;
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
}

// ---------------------------------------------------------------------------
// getAllVoltTemp - main polling loop called once per second from loop()
//
// KEY CHANGE from original:
//   Original:  digitalRead(11) == LOW -> isFaulted = true
//   M5Dial:    No hardware fault pin. GPIO1/GPIO2 both occupied by UART.
//              Instead, derive isFaulted from UART register data that is
//              already read inside readModuleValues() -> readStatus().
//              This is actually more informative: we know *which* module
//              faulted and *what* the fault code is.
// ---------------------------------------------------------------------------
void BMSModuleManager::getAllVoltTemp()
{
    bms_modules_text = "";
    packVolt = 0.0f;
    float lowCell  = 1000.0f;
    float highCell = -1000.0f;

    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            Logger::debug("Module %i: reading voltage and temperature", x);
            modules[x].readModuleValues();

            float low  = modules[x].getLowCellV();
            float high = modules[x].getHighCellV();
            Logger::debug("Module %i voltage: %fV  low: %fV  high: %fV",
                          x, modules[x].getModuleVoltage(), low, high);

            if (low  < lowCell)  lowCell  = low;
            if (high > highCell) highCell = high;

            packVolt += modules[x].getModuleVoltage();

            // Only record temperatures from sensors that are actually connected
            // Unpopulated thermistors read ~-89C; ignore anything below -70C
            if (modules[x].getLowTemp()  > settings.IgnoreTempThresh && modules[x].getLowTemp()  < lowestPackTemp)
                lowestPackTemp  = modules[x].getLowTemp();
            if (modules[x].getHighTemp() > settings.IgnoreTempThresh && modules[x].getHighTemp() > highestPackTemp)
                highestPackTemp = modules[x].getHighTemp();

            // Build display string for this module
            bms_modules_text += String("Mod #") + (int)x
                              + ": " + modules[x].getModuleVoltage() + "v"
                              + " lo:" + low + "v hi:" + high + "v"
                              + " d="  + (high - low) + "v\n";
        }
    }

    // Divide by parallel string count for true pack voltage
    int ps = (Pstring > 0) ? Pstring : 1;
    packVolt = packVolt / (float)ps;
    if (packVolt > highestPackVolt) highestPackVolt = packVolt;
    if (packVolt < lowestPackVolt)  lowestPackVolt  = packVolt;

    float delta = (highCell > -1000.0f && lowCell < 1000.0f) ? (highCell - lowCell) : 0.0f;

    // Update pack summary string for display
    bms_status  = String("SoC: ") + (int)getSoC(packVolt) + " %\n\n";
    bms_status += String("Pack: ") + packVolt + "v\n";
    bms_status += String("Lo: ")   + lowCell  + "v  Hi: " + highCell + "v\n";
    bms_status += String("Δ: ")    + delta    + "v";

    // -------------------------------------------------------------------------
    // Software fault detection (replaces digitalRead hardware fault pin)
    // Scan all module fault registers - already read by readStatus() above.
    // If any module reports a non-zero fault register, mark pack as faulted.
    // -------------------------------------------------------------------------
    bool anyFault = false;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting() && (modules[x].getFaults() & ~0x02) > 0)
        {
            if (!isFaulted)
                Logger::error("Module %i entered fault state! Faults=0x%X",
                              x, modules[x].getFaults());
            anyFault = true;
        }
    }

    if (!anyFault && isFaulted)
        Logger::info("All modules have exited faulted state");

    isFaulted = anyFault;
}

// ---------------------------------------------------------------------------
// Setters for configuration from SerialConsole / EEPROM
// ---------------------------------------------------------------------------
void BMSModuleManager::setBatteryID(int id) { batteryID = id; }
void BMSModuleManager::setPstrings(int Pstrings) { Pstring = Pstrings; }

void BMSModuleManager::setSensors(int sensor, float Ignore)
{
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            modules[x].settempsensor(sensor);
            modules[x].setIgnoreCell(Ignore);
        }
    }
}

// ---------------------------------------------------------------------------
// Pack-level data accessors
// ---------------------------------------------------------------------------
float BMSModuleManager::getLowCellVolt()
{
    LowCellVolt = 5.0f;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
        if (modules[x].isExisting() && modules[x].getLowCellV() < LowCellVolt)
            LowCellVolt = modules[x].getLowCellV();
    return LowCellVolt;
}

float BMSModuleManager::getHighCellVolt()
{
    HighCellVolt = 0.0f;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
        if (modules[x].isExisting() && modules[x].getHighCellV() > HighCellVolt)
            HighCellVolt = modules[x].getHighCellV();
    return HighCellVolt;
}

float BMSModuleManager::getPackVoltage()    { return packVolt;        }
float BMSModuleManager::getLowVoltage()     { return lowestPackVolt;  }
float BMSModuleManager::getHighVoltage()    { return highestPackVolt; }
int   BMSModuleManager::getNumModules()     { return numFoundModules; }
bool  BMSModuleManager::isFaultedState()    { return isFaulted;       }

BMSModule& BMSModuleManager::getModule(int addr)
{
    // addr clamped to valid range; caller should check isExisting()
    if (addr < 1 || addr > MAX_MODULE_ADDR) addr = 1;
    return modules[addr];
}

float BMSModuleManager::getAvgTemperature()
{
    float avg = 0.0f;
    int   validModules = 0;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting() && modules[x].getAvgTemp() > settings.IgnoreTempThresh)
        {
            avg += modules[x].getAvgTemp();
            validModules++;
        }
    }
    if (validModules == 0) return 0.0f;
    return avg / (float)validModules;
}

float BMSModuleManager::getHighTemperature()
{
    return highestPackTemp;
}

float BMSModuleManager::getLowTemperature()
{
    return lowestPackTemp;
}

float BMSModuleManager::getAvgCellVolt()
{
    float avg = 0.0f;
    if (numFoundModules == 0) return 0.0f;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
        if (modules[x].isExisting()) avg += modules[x].getAverageV();
    return avg / (float)numFoundModules;
}

// ---------------------------------------------------------------------------
// printPackSummary - USB console debug output
// All direct serial calls use SERIALCONSOLE (no hardcoded SerialUSB)
// ---------------------------------------------------------------------------
void BMSModuleManager::printPackSummary()
{
    Logger::console("");
    Logger::console("=================== Pack Status ===================");
    if (isFaulted) Logger::console("  *** FAULTED ***");
    else            Logger::console("  All systems go!");
    Logger::console("Modules: %i  Pack: %fV  AvgCell: %fV  AvgTemp: %fC",
                    numFoundModules, getPackVoltage(), getAvgCellVolt(), getAvgTemperature());
    Logger::console("");

    for (int y = 1; y <= MAX_MODULE_ADDR; y++)
    {
        if (modules[y].isExisting())
        {
            uint8_t faults = modules[y].getFaults();
            uint8_t alerts = modules[y].getAlerts();
            uint8_t COV    = modules[y].getCOVCells();
            uint8_t CUV    = modules[y].getCUVCells();

            Logger::console("Module #%i", y);
            Logger::console("  Voltage: %fV  (%fV-%fV)  Temps: (%fC-%fC)",
                            modules[y].getModuleVoltage(),
                            modules[y].getLowCellV(), modules[y].getHighCellV(),
                            modules[y].getLowTemp(),  modules[y].getHighTemp());

            if (faults > 0)
            {
                Logger::console("  MODULE IS FAULTED:");
                if (faults & 1) {
                    SERIALCONSOLE.print("    OV cells (1-6): ");
                    for (int i = 0; i < 6; i++)
                        if (COV & (1 << i)) { SERIALCONSOLE.print(i+1); SERIALCONSOLE.print(" "); }
                    SERIALCONSOLE.println();
                }
                if (faults & 2) {
                    SERIALCONSOLE.print("    UV cells (1-6): ");
                    for (int i = 0; i < 6; i++)
                        if (CUV & (1 << i)) { SERIALCONSOLE.print(i+1); SERIALCONSOLE.print(" "); }
                    SERIALCONSOLE.println();
                }
                if (faults & 4)  Logger::console("    CRC error in received packet");
                if (faults & 8)  Logger::console("    Power on reset occurred");
                if (faults & 0x10) Logger::console("    Test fault active");
                if (faults & 0x20) Logger::console("    Internal registers inconsistent");
            }
            if (alerts > 0)
            {
                Logger::console("  MODULE HAS ALERTS:");
                if (alerts & 1)    Logger::console("    Over temperature TS1");
                if (alerts & 2)    Logger::console("    Over temperature TS2");
                if (alerts & 4)    Logger::console("    Sleep mode active");
                if (alerts & 8)    Logger::console("    Thermal shutdown active");
                if (alerts & 0x10) Logger::console("    Test alert");
                if (alerts & 0x20) Logger::console("    OTP EPROM uncorrectable error");
                if (alerts & 0x40) Logger::console("    GROUP3 regs invalid");
                if (alerts & 0x80) Logger::console("    Address not registered");
            }
            if (faults > 0 || alerts > 0) SERIALCONSOLE.println();
        }
    }
}

// ---------------------------------------------------------------------------
// printPackDetails - per-cell voltage + temperature dump to USB console
// ---------------------------------------------------------------------------
void BMSModuleManager::printPackDetails()
{
    Logger::console("");
    Logger::console("=================== Pack Details ==================");
    if (isFaulted) Logger::console("  *** FAULTED ***");
    else            Logger::console("  All systems go!");
    Logger::console("Modules: %i  Pack: %fV  AvgCell: %fV  Lo: %fV  Hi: %fV  AvgTemp: %fC",
                    numFoundModules, getPackVoltage(), getAvgCellVolt(),
                    LowCellVolt, HighCellVolt, getAvgTemperature());
    Logger::console("");

    int cellNum = 0;
    for (int y = 1; y <= MAX_MODULE_ADDR; y++)
    {
        if (modules[y].isExisting())
        {
            SERIALCONSOLE.print("Module #");
            if (y < 10) SERIALCONSOLE.print(" ");
            SERIALCONSOLE.print(y);
            SERIALCONSOLE.print("  ");
            SERIALCONSOLE.print(modules[y].getModuleVoltage());
            SERIALCONSOLE.print("V");

            for (int i = 0; i < 6; i++)
            {
                SERIALCONSOLE.print("  Cell");
                if (cellNum < 10) SERIALCONSOLE.print(" ");
                SERIALCONSOLE.print(cellNum++);
                SERIALCONSOLE.print(": ");
                SERIALCONSOLE.print(modules[y].getCellVoltage(i));
                SERIALCONSOLE.print("V");
            }
            SERIALCONSOLE.print("  NegTemp: ");
            SERIALCONSOLE.print(modules[y].getTemperature(0));
            SERIALCONSOLE.print("C  PosTemp: ");
            SERIALCONSOLE.print(modules[y].getTemperature(1));
            SERIALCONSOLE.println("C");
        }
    }
}

// ---------------------------------------------------------------------------
// v5: Balance inhibit / auto-balance control
// ---------------------------------------------------------------------------
void BMSModuleManager::setBalanceInhibit(bool inhibit) { balanceInhibit = inhibit; }
bool BMSModuleManager::getBalanceInhibit()             { return balanceInhibit;   }
void BMSModuleManager::setAutoBalance(bool enabled)    { autoBalance = enabled;   }
bool BMSModuleManager::getAutoBalance()                { return autoBalance;      }

// ---------------------------------------------------------------------------
// getModuleCells - returns active cell count for a module address.
// Per-module override (settings.moduleCells[addr]) takes priority;
// falls back to the global settings.numCells default.
// Also syncs the resolved count into the BMSModule object.
// ---------------------------------------------------------------------------
int BMSModuleManager::getModuleCells(int addr)
{
    if (addr < 1 || addr > MAX_MODULE_ADDR) return 6;
    // BMW i3 / Mini-E variants: always 12 cells per module regardless of settings
    if (settings.cmuType == CMU_BMW_I3 || settings.cmuType == CMU_BMW_I3_BUS || settings.cmuType == CMU_BMW_MINIE) {
        modules[addr].setNumCells(BMW_I3_CELLS_PER_MOD);
        return BMW_I3_CELLS_PER_MOD;
    }
    // BMW PHEV: always 16 cells per module (SP44 populates only 0-7, rest zero)
    if (settings.cmuType == CMU_BMW_PHEV) {
        modules[addr].setNumCells(BMW_PHEV_CELLS_PER_MOD);
        return BMW_PHEV_CELLS_PER_MOD;
    }
    // VW Type 5: fixed 12 cells per module
    if (settings.cmuType == CMU_VW_BMS) {
        modules[addr].setNumCells(VW_CELLS_PER_MODULE);
        return VW_CELLS_PER_MODULE;
    }
    uint8_t ov = settings.moduleCells[addr];
    int n = (ov >= 1 && ov <= 6) ? (int)ov
          : ((settings.numCells >= 1 && settings.numCells <= 6) ? (int)settings.numCells : 6);
    modules[addr].setNumCells(n);
    return n;
}

// ---------------------------------------------------------------------------
// getAllVoltTempFromCAN - populate BMSModule objects from BMW i3 CAN data
// Called instead of UART getAllVoltTemp() when cmuType == CMU_BMW_I3
// ---------------------------------------------------------------------------
void BMSModuleManager::getAllVoltTempFromCAN()
{
    packVolt = 0.0f;
    numFoundModules = 0;

    for (int x = 1; x <= BMW_I3_MAX_MODS; x++) {
        I3SlaveData d;
        if (!can.getI3SlaveData(x, d)) {
            // No data yet - if module was previously existing, check timeout
            if (modules[x].isExisting()) {
                // Keep existing until we confirm gone (handled by lastSeenMs check below)
            }
            continue;
        }

        // Mark module as existing only if seen within 3x the normal CAN cycle
        bool alive = (millis() - d.lastSeenMs) < 3000;
        modules[x].setExists(alive);
        if (!alive) continue;

        numFoundModules++;

        // Push cell voltages
        modules[x].setNumCells(BMW_I3_CELLS_PER_MOD);
        float modV = 0.0f;
        for (int c = 0; c < BMW_I3_CELLS_PER_MOD; c++) {
            modules[x].setCellVoltage(c, d.cellV[c]);
            modV += d.cellV[c];
        }
        modules[x].setModuleVoltage(modV);
        modules[x].setTemperature(0, d.temp[0]);
        modules[x].setTemperature(1, d.temp[1]);
        modules[x].setFaults(0);
        modules[x].setAlerts(0);

        packVolt += modV;
    }

    int np = (settings.numParallel > 0) ? settings.numParallel : BMS_NUM_PARALLEL;
    if (np > 1) packVolt /= np;
}

// ---------------------------------------------------------------------------
// getAllVoltTempFromPHEV - populate BMSModule objects from BMW PHEV CAN data
// Called instead of UART getAllVoltTemp() when cmuType == CMU_BMW_PHEV.
// Reads PhevSlaveData staging buffers populated by CANManager's RX task.
// Handles up to BMW_PHEV_MAX_MODS (12) modules with 16 cells and 4 temps each.
// ---------------------------------------------------------------------------
void BMSModuleManager::getAllVoltTempFromPHEV()
{
    packVolt = 0.0f;
    numFoundModules = 0;

    for (int x = 1; x <= BMW_PHEV_MAX_MODS; x++) {
        PhevSlaveData d;
        if (!can.getPhevSlaveData(x, d)) {
            // No data ever received for this slot
            continue;
        }

        bool alive = (millis() - d.lastSeenMs) < (uint32_t)BMW_PHEV_TIMEOUT_MS;
        modules[x].setExists(alive);
        if (!alive) continue;

        numFoundModules++;
        modules[x].setNumCells(BMW_PHEV_CELLS_PER_MOD);

        float modV = 0.0f;
        for (int c = 0; c < BMW_PHEV_CELLS_PER_MOD; c++) {
            modules[x].setCellVoltage(c, d.cellV[c]);
            if (d.cellV[c] > settings.IgnoreVolt && d.cellV[c] < 5.0f)
                modV += d.cellV[c];
        }
        modules[x].setModuleVoltage(modV);

        // 4 temperature sensors — store all 4; display uses T1/T2
        for (int t = 0; t < BMW_PHEV_TEMPS_PER_MOD; t++) {
            modules[x].setTemperature(t, d.temp[t]);
        }

        // Map 32-bit error word to 8-bit faults field (non-zero = fault)
        uint8_t faultByte = (d.error != 0) ? 0x01 : 0x00;
        modules[x].setFaults(faultByte);
        modules[x].setAlerts(0);

        packVolt += modV;
    }

    int np = (settings.numParallel > 0) ? settings.numParallel : BMS_NUM_PARALLEL;
    if (np > 1) packVolt /= np;
}

// ---------------------------------------------------------------------------
// getAllVoltTempFromVW - populate BMSModule objects from VW Type 5 CAN data
// ---------------------------------------------------------------------------
void BMSModuleManager::getAllVoltTempFromVW()
{
    packVolt = 0.0f;
    numFoundModules = 0;

    const uint32_t now = millis();
    const int maxMods = (VW_MAX_MODULES < MAX_MODULE_ADDR) ? VW_MAX_MODULES : MAX_MODULE_ADDR;

    for (int i = 0; i < maxMods; i++) {
        const int addr = i + 1;
        VWSlaveData d;
        bool hasData = can.getVWSlaveData(i, d);
        uint32_t lastSeen = hasData ? d.lastSeenMs : can.getVWLastSeen(i);
        bool alive = (lastSeen != 0) && ((now - lastSeen) < VW_TIMEOUT_MS);
        modules[addr].setExists(alive);
        if (!alive) continue;

        modules[addr].setNumCells(VW_CELLS_PER_MODULE);
        if (d.fresh) {
            float modV = 0.0f;
            for (int c = 0; c < VW_CELLS_PER_MODULE; c++) {
                modules[addr].setCellVoltage(c, d.cellV[c]);
                if (d.cellV[c] > settings.IgnoreVolt && d.cellV[c] < 5.0f)
                    modV += d.cellV[c];
            }
            modules[addr].setModuleVoltage(modV);
            modules[addr].setTemperature(0, d.temp[0]);
            modules[addr].setTemperature(1, d.temp[1]);
            modules[addr].setFaults(0);
            modules[addr].setAlerts(0);
        }

        numFoundModules++;
        packVolt += modules[addr].getModuleVoltage();
    }

    int np = (settings.numParallel > 0) ? settings.numParallel : BMS_NUM_PARALLEL;
    if (np > 1) packVolt /= np;
}
