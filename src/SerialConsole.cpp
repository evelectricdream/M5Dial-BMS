// =============================================================================
// SerialConsole.cpp - USB serial debug interface  v5.1
// Press 'h' or '?' to see menu with current values for all settings.
// All KEY=value commands save to flash immediately.
// =============================================================================
#include "SerialConsole.h"
#include "Logger.h"
#include "BMSModuleManager.h"
#include <EEPROM.h>

template<class T> inline Print &operator<<(Print &obj, T arg) { obj.print(arg); return obj; }

extern EEPROMSettings   settings;
extern BMSModuleManager bms;

bool     printPrettyDisplay;
uint32_t prettyCounter;
int      whichDisplay;

SerialConsole::SerialConsole() { init(); }

void SerialConsole::init() {
    ptrBuffer          = 0;
    state              = STATE_ROOT_MENU;
    loopcount          = 0;
    cancel             = false;
    printPrettyDisplay = false;
    prettyCounter      = 0;
    whichDisplay       = 0;
}

void SerialConsole::loop() {
    if (SERIALCONSOLE.available()) serialEvent();
    if (printPrettyDisplay && (millis() > (prettyCounter + 3000))) {
        prettyCounter = millis();
        if (whichDisplay == 0) bms.printPackSummary();
        if (whichDisplay == 1) bms.printPackDetails();
    }
}

// ---------------------------------------------------------------------------
// printMenu - shows all commands WITH their current values
// ---------------------------------------------------------------------------
void SerialConsole::printMenu() {
    Logger::console("\n========== M5DialBMS v7 - MENU ==========");
    Logger::console("Line ending required (LF, CR, or CRLF)\n");

    Logger::console("--- Pack Actions ---");
    Logger::console("  h / ? / H  This menu");
    Logger::console("  S          Sleep all boards");
    Logger::console("  W          Wake all boards");
    Logger::console("  C          Clear all faults");
    Logger::console("  F          Find connected boards");
    Logger::console("  R          Renumber boards from 1");
    Logger::console("  B          Manual balance cycle");
    Logger::console("  p          Toggle pack summary (3s)");
    Logger::console("  d          Toggle pack details (3s)");
    Logger::console("  X          Stop WiFi and CAN\n");

    Logger::console("--- Pack Configuration ---");
    Logger::console("  NUMCELLS=N       Cells per module [%d]  (4-16)",
                    settings.numCells);
    Logger::console("  NUMSERIES=N      Modules in series [%d]",
                    settings.numSeries);
    Logger::console("  NUMPARALLEL=N    Modules in parallel [%d]",
                    settings.numParallel);
    Logger::console("  SOCLO=N.NN       V/module at 0%% SoC [%.2f]",
                    settings.socLo);
    Logger::console("  SOCHI=N.NN       V/module at 100%% SoC [%.2f]\n",
                    settings.socHi);

    Logger::console("--- Cell Thresholds ---");
    Logger::console("  VOLTLIMHI=N.NN   OV limit per cell [%.2fV]",
                    settings.OverVSetpoint);
    Logger::console("  VOLTLIMLO=N.NN   UV limit per cell [%.2fV]",
                    settings.UnderVSetpoint);
    Logger::console("  IGNORECELL=N.NN  Ignore cells below this V [%.2fV]",
                    settings.IgnoreVolt);
    Logger::console("  TEMPLIMHI=N.NN   Over-temp limit [%.1fC]",
                    settings.OverTSetpoint);
    Logger::console("  TEMPLIMLO=N.NN   Under-temp limit [%.1fC]",
                    settings.UnderTSetpoint);
    Logger::console("  IGNORETEMP=N.NN  Ignore temps below this C [%.1fC]\n",
                    settings.IgnoreTempThresh);

    Logger::console("--- Balancing ---");
    Logger::console("  AUTOBAL=0/1      Auto-balance [%s]",
                    settings.balancingEnabled ? "ON" : "OFF");
    Logger::console("  BALVOLT=N.NN     Balance target V [%.2fV]",
                    settings.balanceVoltage);
    Logger::console("  BALHYST=N.NN     Balance hysteresis [%.3fV]",
                    settings.balanceHyst);
    Logger::console("  DRIVEPIN=0/1     Manual drive inhibit [%s]\n",
                    bms.getBalanceInhibit() ? "INHIBITED" : "off");

    Logger::console("--- Connectivity ---");
    Logger::console("  WIFI=0/1         WiFi at boot [%s]",
                    settings.wifiEnabled ? "ON" : "OFF");
    Logger::console("  CAN=0/1          CAN TX/RX [running=use X to stop]");
    Logger::console("  BATTERYID=N      CAN battery ID [%d]",
                    settings.batteryID);
    Logger::console("  LOGLEVEL=N       0=debug 1=info 2=warn 3=error 4=off [%d]",
                    settings.logLevel);
    Logger::console("\n--- v7: CMU Type & CAN Inhibit ---");
    Logger::console("  CMUTYPE=0-5      0=Tesla UART, 1=BMW i3, 2=BMW i3 Bus,");
    Logger::console("                   3=BMW Mini-E, 4=BMW PHEV, 5=VW BMS [%d] (reboot)",
                    settings.cmuType);
    Logger::console("  CANINHIBIT=0/1   CAN-based balance inhibit [%s]",
                    settings.canInhibitEnabled ? "ON" : "OFF");
    Logger::console("  CHGID=0xNNN      Charger heartbeat CAN ID [0x%03X]",
                    settings.chargerHeartbeatID);
    Logger::console("================================================\n");
}

// ---------------------------------------------------------------------------
// serialEvent
// ---------------------------------------------------------------------------
void SerialConsole::serialEvent() {
    int incoming = SERIALCONSOLE.read();
    if (incoming == -1) return;
    if (incoming == 10 || incoming == 13) {
        handleConsoleCmd();
        ptrBuffer = 0;
    } else {
        cmdBuffer[ptrBuffer++] = (unsigned char)incoming;
        if (ptrBuffer > 79) ptrBuffer = 79;
    }
}

void SerialConsole::handleConsoleCmd() {
    if (state == STATE_ROOT_MENU) {
        if (ptrBuffer == 1) handleShortCmd();
        else                handleConfigCmd();
    }
}

// ---------------------------------------------------------------------------
// handleShortCmd
// ---------------------------------------------------------------------------
void SerialConsole::handleShortCmd() {
    switch (cmdBuffer[0]) {
    case 'h': case '?': case 'H':
        printMenu();
        break;
    case 'S':
        Logger::console("Sleeping all boards");
        bms.sleepBoards();
        break;
    case 'W':
        Logger::console("Waking all boards");
        bms.wakeBoards();
        break;
    case 'C':
        Logger::console("Clearing all faults");
        bms.clearFaults();
        break;
    case 'F':
        bms.findBoards();
        break;
    case 'R':
        Logger::console("Renumbering boards");
        bms.renumberBoardIDs();
        break;
    case 'B':
        Logger::console("Manual balance cycle");
        bms.balanceCells();
        break;
    case 'p':
        if (whichDisplay == 1 && printPrettyDisplay) { whichDisplay = 0; }
        else {
            printPrettyDisplay = !printPrettyDisplay;
            Logger::console(printPrettyDisplay ? "Pack summary ON (3s)" : "Pack summary OFF");
        }
        break;
    case 'd':
        if (whichDisplay == 0 && printPrettyDisplay) { whichDisplay = 1; }
        else {
            printPrettyDisplay = !printPrettyDisplay;
            whichDisplay = 1;
            Logger::console(printPrettyDisplay ? "Pack details ON (3s)" : "Pack details OFF");
        }
        break;
    case 'X':
        Logger::console("Stopping WiFi and CAN...");
        extern void stopWifiAndCan();
        stopWifiAndCan();
        break;
    }
}

// ---------------------------------------------------------------------------
// handleConfigCmd - KEY=value commands, all saved to flash
// ---------------------------------------------------------------------------
void SerialConsole::handleConfigCmd() {
    int   newValue;
    float newFloat;
    bool  needEEPROMWrite = false;

    if (ptrBuffer < 3) return;
    cmdBuffer[ptrBuffer] = 0;

    String cmdString;
    int i = 0;
    while (cmdBuffer[i] != '=' && i < ptrBuffer) cmdString.concat(String(cmdBuffer[i++]));
    i++; // skip '='
    if (i >= ptrBuffer) { Logger::console("Format: KEY=value  (e.g. VOLTLIMHI=4.20)"); return; }

    newValue = strtol((char *)(cmdBuffer + i), NULL, 0);
    newFloat = strtof((char *)(cmdBuffer + i), NULL);
    cmdString.toUpperCase();

    // --- Pack topology ---
    if (cmdString == "NUMCELLS") {
        if (newValue >= 1 && newValue <= 16) {
            settings.numCells = (uint8_t)newValue;
            needEEPROMWrite = true;
            Logger::console("Cells per module: %d  (display + web will show %d cells)", newValue, newValue);
        } else Logger::console("Invalid (1-16)");
    }
    else if (cmdString == "NUMSERIES") {
        if (newValue >= 1 && newValue <= 62) {
            settings.numSeries = (uint8_t)newValue;
            bms.setPstrings(settings.numParallel);
            needEEPROMWrite = true;
            Logger::console("Modules in series: %d", newValue);
        } else Logger::console("Invalid (1-62)");
    }
    else if (cmdString == "NUMPARALLEL") {
        if (newValue >= 1 && newValue <= 10) {
            settings.numParallel = (uint8_t)newValue;
            bms.setPstrings(newValue);
            needEEPROMWrite = true;
            Logger::console("Modules in parallel: %d", newValue);
        } else Logger::console("Invalid (1-10)");
    }
    else if (cmdString == "SOCLO") {
        if (newFloat >= 10.0f && newFloat < settings.socHi) {
            settings.socLo = newFloat;
            needEEPROMWrite = true;
            Logger::console("SoC 0%% calibration: %.2fV/module", newFloat);
        } else Logger::console("Invalid (10.0 to SOCHI)");
    }
    else if (cmdString == "SOCHI") {
        if (newFloat > settings.socLo && newFloat <= 30.0f) {
            settings.socHi = newFloat;
            needEEPROMWrite = true;
            Logger::console("SoC 100%% calibration: %.2fV/module", newFloat);
        } else Logger::console("Invalid (SOCLO to 30.0)");
    }
    // --- Cell thresholds ---
    else if (cmdString == "VOLTLIMHI") {
        if (newFloat >= 0.0f && newFloat <= 6.0f) {
            settings.OverVSetpoint = newFloat;
            needEEPROMWrite = true;
            Logger::console("OV limit: %.2fV", newFloat);
        } else Logger::console("Invalid (0.0-6.0)");
    }
    else if (cmdString == "VOLTLIMLO") {
        if (newFloat >= 0.0f && newFloat <= 6.0f) {
            settings.UnderVSetpoint = newFloat;
            needEEPROMWrite = true;
            Logger::console("UV limit: %.2fV", newFloat);
        } else Logger::console("Invalid (0.0-6.0)");
    }
    else if (cmdString == "IGNORECELL") {
        if (newFloat >= 0.0f && newFloat <= 3.0f) {
            settings.IgnoreVolt = newFloat;
            bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
            needEEPROMWrite = true;
            Logger::console("Ignore cells below: %.2fV", newFloat);
        } else Logger::console("Invalid (0.0-3.0)");
    }
    else if (cmdString == "TEMPLIMHI") {
        if (newFloat >= 0.0f && newFloat <= 100.0f) {
            settings.OverTSetpoint = newFloat;
            needEEPROMWrite = true;
            Logger::console("OT limit: %.1fC", newFloat);
        } else Logger::console("Invalid (0.0-100.0)");
    }
    else if (cmdString == "TEMPLIMLO") {
        if (newFloat >= -60.0f && newFloat <= 50.0f) {
            settings.UnderTSetpoint = newFloat;
            needEEPROMWrite = true;
            Logger::console("UT limit: %.1fC", newFloat);
        } else Logger::console("Invalid (-60.0 to 50.0)");
    }
    else if (cmdString == "IGNORETEMP") {
        if (newFloat >= -100.0f && newFloat <= 0.0f) {
            settings.IgnoreTempThresh = newFloat;
            needEEPROMWrite = true;
            Logger::console("Ignore temps below: %.1fC", newFloat);
        } else Logger::console("Invalid (-100.0 to 0.0)");
    }
    // --- Balancing ---
    else if (cmdString == "AUTOBAL") {
        bool en = (newValue != 0);
        settings.balancingEnabled = en ? 1 : 0;
        bms.setAutoBalance(en);
        needEEPROMWrite = true;
        Logger::console("Auto-balancing: %s", en ? "ON" : "OFF");
    }
    else if (cmdString == "BALVOLT") {
        if (newFloat >= 0.0f && newFloat <= 6.0f) {
            settings.balanceVoltage = newFloat;
            needEEPROMWrite = true;
            Logger::console("Balance target: %.2fV", newFloat);
        } else Logger::console("Invalid (0.0-6.0)");
    }
    else if (cmdString == "BALHYST") {
        if (newFloat >= 0.0f && newFloat <= 1.0f) {
            settings.balanceHyst = newFloat;
            needEEPROMWrite = true;
            Logger::console("Balance hysteresis: %.3fV", newFloat);
        } else Logger::console("Invalid (0.0-1.0)");
    }
    else if (cmdString == "DRIVEPIN") {
        bool inhibit = (newValue != 0);
        bms.setBalanceInhibit(inhibit);
        Logger::console("Balance inhibit: %s", inhibit ? "INHIBITED" : "cleared");
        // not saved to flash - runtime only
    }
    // --- Connectivity ---
    else if (cmdString == "WIFI") {
        settings.wifiEnabled = (newValue != 0) ? 1 : 0;
        needEEPROMWrite = true;
        Logger::console("WiFi at boot: %s (reboot to apply)", settings.wifiEnabled ? "ON" : "OFF");
    }
    else if (cmdString == "CAN") {
        extern void setCANEnabled(bool en);
        setCANEnabled(newValue != 0);
        Logger::console("CAN: %s", (newValue != 0) ? "starting" : "stopping");
    }
    else if (cmdString == "BATTERYID") {
        if (newValue > 0 && newValue < 15) {
            settings.batteryID = (uint8_t)newValue;
            bms.setBatteryID(newValue);
            needEEPROMWrite = true;
            Logger::console("Battery CAN ID: %d", newValue);
        } else Logger::console("Invalid (1-14)");
    }
    else if (cmdString == "LOGLEVEL") {
        if (newValue >= 0 && newValue <= 4) {
            Logger::setLoglevel((Logger::LogLevel)newValue);
            settings.logLevel = (uint8_t)newValue;
            needEEPROMWrite = true;
            const char *names[] = {"DEBUG","INFO","WARN","ERROR","OFF"};
            Logger::console("Log level: %s", names[newValue]);
        } else Logger::console("Invalid (0-4)");
    }
    // --- v6: CMU type ---
    else if (cmdString == "CMUTYPE") {
        if (newValue >= 0 && newValue <= 5) {
            if (settings.cmuType != (uint8_t)newValue) {
                settings.cmuType = (uint8_t)newValue;
                needEEPROMWrite = true;
                const char *cmuNames[] = {
                    "Tesla UART",
                    "BMW i3 CAN",
                    "BMW i3 Bus Pack",
                    "BMW Mini-E",
                    "BMW PHEV SP06/SP41",
                    "VW BMS"
                };
                Logger::console("CMU type: %s  (Rebooting in 1s to apply...)", cmuNames[newValue]);
                xTaskCreate([](void *param) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    ESP.restart();
                }, "restart_task", 1024, nullptr, 1, nullptr);
            } else {
                Logger::console("CMU type already set to that value.");
            }
        } else Logger::console("Invalid (0=Tesla, 1=BMW i3, 2=BMW i3 Bus, 3=BMW Mini-E, 4=BMW PHEV, 5=VW BMS)");
    }
    // --- v6: CAN-based balance inhibit ---
    else if (cmdString == "CANINHIBIT") {
        settings.canInhibitEnabled = (newValue != 0) ? 1 : 0;
        needEEPROMWrite = true;
        Logger::console("CAN inhibit: %s", settings.canInhibitEnabled ? "ON" : "OFF");
    }
    else if (cmdString == "CHGID") {
        // Accepts decimal or 0x hex (e.g. CHGID=0x305 or CHGID=773)
        uint32_t id = (uint32_t)strtoul((const char *)(cmdBuffer + i), NULL, 0);
        if (id > 0 && id <= 0x7FF) {
            settings.chargerHeartbeatID = id;
            needEEPROMWrite = true;
            Logger::console("Charger heartbeat ID: 0x%03X", id);
        } else Logger::console("Invalid CAN ID (0x001-0x7FF)");
    }
    else {
        Logger::console("Unknown: '%s'  Press H for menu.", cmdString.c_str());
    }

    if (needEEPROMWrite) {
        EEPROM.put(EEPROM_PAGE, settings);
        EEPROM.commit();
        Logger::console("Saved.");
    }
}
