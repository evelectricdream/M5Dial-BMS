// =============================================================================
// main.cpp - M5DialBMS v7
//
// New in v7:
//   - 5 CMU types: Tesla UART / BMW i3 / BMW i3 Bus Pack / BMW Mini-E / BMW PHEV
//   - BMW PHEV SP06/SP41/SP44 support (polled 50ms, 16 cells, 4 temps)
//   - GVRET/SavvyCAN TCP server on port 23
//   - Web installer via GitHub Pages CI/CD
//   - M5DialBMS branding, WiFi SSID M5DialBMS/M5DialBMS
//
// Hardware:
//   CMU UART  : Grove A  - GPIO13(RX) / GPIO15(TX)  [Tesla mode only]
//   CAN XCVR  : Grove B  - GPIO2(TX)  / GPIO1(RX)
//   Drive inh : GPIO38   - HIGH = drive, inhibit balancing
//   Encoder   : GPIO40(A) / GPIO41(B) / GPIO42(BTN)
//   Power latch: GPIO46
// =============================================================================
#include <Arduino.h>
#include <M5Unified.h>
#include <EEPROM.h>
#include "bms_config.h"
#include "BMSModuleManager.h"
#include "SerialConsole.h"
#include "Logger.h"
#include "Display.h"
#include "CANManager.h"
#include "WiFiManager.h"

HardwareSerial SERIALBMS(1);

BMSModuleManager bms;
SerialConsole    console;
EEPROMSettings   settings;
Display          display;
CANManager       can;
WiFiManager      wifi;

String wifiDisplayIP = "";

// ---------------------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------------------
#define ENC_A_PIN   40
#define ENC_B_PIN   41
#define ENC_BTN_PIN 42

static volatile int  encoderCount    = 0;
static volatile bool encoderBtnShort = false;  // short press
static volatile bool encoderBtnLong  = false;  // long press (WiFi toggle)
static int     lastEncA      = HIGH;
static bool    lastBtnState  = HIGH;
static uint32_t btnPressStart = 0;
static bool    btnHeld        = false;

static void pollEncoder()
{
    int a = digitalRead(ENC_A_PIN);
    if (a != lastEncA) {
        lastEncA = a;
        if (a == LOW) {
            if (digitalRead(ENC_B_PIN) == HIGH) encoderCount++;
            else                                 encoderCount--;
        }
    }

    bool btn = (digitalRead(ENC_BTN_PIN) == LOW);

    if (btn && !lastBtnState) {
        // Button just pressed
        btnPressStart = millis();
        btnHeld = false;
    }

    if (btn && !btnHeld && (millis() - btnPressStart > 1000)) {
        // Long press threshold crossed
        btnHeld = true;
        encoderBtnLong = true;
    }

    if (!btn && lastBtnState) {
        // Button released
        if (!btnHeld) {
            encoderBtnShort = true;  // short press
        }
        btnHeld = false;
    }

    lastBtnState = btn;
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
static uint32_t lastBmsRead     = 0;
static uint32_t lastCANSend     = 0;
static uint32_t lastEncoderPoll = 0;
static int      lastEncoderCount = 0;

#define BMS_READ_INTERVAL_MS    1000
#define CAN_SEND_INTERVAL_MS    1000
#define ENCODER_POLL_MS            5
#define FAULT_BEEP_INTERVAL_MS  2000
static bool faultAlertActive = false;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void loadSettings();
void applySettings();
int  nextExistingPage(int current, int direction);

// ---------------------------------------------------------------------------
// Helper: human-readable CMU type label
// ---------------------------------------------------------------------------
static const char *cmuTypeName()
{
    switch (settings.cmuType) {
        case CMU_TESLA:      return "Tesla UART";
        case CMU_BMW_I3:     return "BMW i3";
        case CMU_BMW_I3_BUS: return "BMW i3 Bus";
        case CMU_BMW_MINIE:  return "BMW Mini-E";
        case CMU_BMW_PHEV:   return "BMW PHEV";
        case CMU_VW_BMS:     return "VW BMS";
        default:             return "Unknown";
    }
}

// ---------------------------------------------------------------------------
// Helpers exposed to SerialConsole
// ---------------------------------------------------------------------------
void stopWifiAndCan()
{
    if (can.isRunning())  can.end();
    if (wifi.isRunning()) wifi.end();
}

void setCANEnabled(bool en)
{
    if (en && !can.isRunning())  can.begin();
    if (!en && can.isRunning())  can.end();
}

// ---------------------------------------------------------------------------
// WiFi runtime toggle (long-press)
// ---------------------------------------------------------------------------
static void toggleWifi()
{
    if (wifi.isRunning()) {
        Logger::console("WiFi: toggled OFF by button long-press");
        wifi.end();
        wifiDisplayIP = "";
    } else {
        Logger::console("WiFi: toggled ON by button long-press");
        display.showStartup("Starting WiFi...");
        wifi.begin();
        wifiDisplayIP = wifi.getIP();
        Logger::console("WiFi IP: %s", wifiDisplayIP.c_str());
    }
    display.markDirty();
}

// =============================================================================
// setup()
// =============================================================================
void setup()
{
    // 0. Serial mutex — must be first, before any Logger calls or tasks
    Logger::init();

    // 1. Power hold MUST be first
    pinMode(PIN_POWER_HOLD, OUTPUT);
    digitalWrite(PIN_POWER_HOLD, HIGH);

    // 2. M5Dial init
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(0);
    M5.Display.setBrightness(180);

    // 3. LVGL display
    display.begin();
    display.showStartup("Booting...");

    // 4. Encoder
    pinMode(ENC_A_PIN,   INPUT_PULLUP);
    pinMode(ENC_B_PIN,   INPUT_PULLUP);
    pinMode(ENC_BTN_PIN, INPUT_PULLUP);
    lastEncA     = digitalRead(ENC_A_PIN);
    lastBtnState = (digitalRead(ENC_BTN_PIN) == LOW);

    // 5. Drive inhibit GPIO
    pinMode(PIN_DRIVE_INHIBIT, INPUT_PULLDOWN);

    // 6. USB console
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000) delay(10);
    Logger::console("M5DialBMS v7 booting...");

    // 7. CMU UART — always init; harmless in CAN-only modes
    SERIALBMS.begin(BMS_BAUD_RATE, SERIAL_8N1, BMS_SERIAL_RX_PIN, BMS_SERIAL_TX_PIN);
    delay(50);
    while (SERIALBMS.available()) SERIALBMS.read();
    Logger::console("CMU UART: %d baud GPIO%d/GPIO%d",
                    BMS_BAUD_RATE, BMS_SERIAL_RX_PIN, BMS_SERIAL_TX_PIN);

    // 8. EEPROM / settings
    EEPROM.begin(sizeof(EEPROMSettings));
    loadSettings();
    applySettings();

    // 9. BMS enumeration — path depends on CMU type
    if (settings.cmuType == CMU_TESLA) {
        display.showStartup("Waking CMUs...");
        Logger::console("Tesla UART mode: waking and numbering CMUs...");
        bms.wakeBoards();
        delay(500);
        display.showStartup("Numbering...");
        bms.renumberBoardIDs();
        delay(500);
        display.showStartup("Finding...");
        bms.findBoards();
        bms.clearFaults();
    } else if (settings.cmuType == CMU_BMW_PHEV) {
        Logger::console("BMW PHEV CAN mode: skipping UART enumeration");
        display.showStartup("PHEV CAN mode...");
        // Module discovery via 0x080 polls — starts automatically once CAN begins
    } else if (settings.cmuType == CMU_VW_BMS) {
        Logger::console("VW CAN mode: skipping UART enumeration");
        display.showStartup("VW CAN mode...");
        // Module discovery via 0x0BA polls — starts automatically once CAN begins
    } else {
        Logger::console("%s CAN mode: skipping UART enumeration", cmuTypeName());
        display.showStartup("BMW CAN mode...");
        // Modules appear once CAN RX task sees their frames
    }

    int n = bms.getNumModules();
    Logger::console("Ready. CMU=%s (%d), %d module(s) found.", cmuTypeName(), settings.cmuType, n);

    // 10. WiFi (if enabled at boot)
    if (settings.wifiEnabled) {
        display.showStartup("Starting WiFi...");
        wifi.begin();
        wifiDisplayIP = wifi.getIP();
    }

    // 11. CAN (always attempt - RX task starts inside begin())
    display.showStartup("Starting CAN...");
    if (can.begin()) {
        Logger::console("CAN ready on GPIO%d/GPIO%d (TX+RX)", PIN_CAN_TX, PIN_CAN_RX);
    } else {
        Logger::console("CAN not available");
    }

    // 12. Auto-balance
    bms.setAutoBalance(settings.balancingEnabled != 0);

    display.setPage(0);
    display.markDirty();
    Logger::console("Boot complete.");
}

// =============================================================================
// loop()
// =============================================================================
void loop()
{
    uint32_t now = millis();
    M5.update();

    // Encoder poll
    if (now - lastEncoderPoll >= ENCODER_POLL_MS) {
        lastEncoderPoll = now;
        pollEncoder();
    }

    // Balance inhibit: GPIO + optional CAN charger heartbeat
    bool gpioInhibit = (digitalRead(PIN_DRIVE_INHIBIT) == BALANCE_INHIBIT_ACTIVE);
    bool canInhibit  = (settings.canInhibitEnabled && can.isRunning() && !can.getChargerActive());
    bms.setBalanceInhibit(gpioInhibit || canInhibit);

    // BMS read — dispatch by CMU type
    if (now - lastBmsRead >= BMS_READ_INTERVAL_MS) {
        lastBmsRead = now;

        switch (settings.cmuType) {
            case CMU_TESLA:
                bms.getAllVoltTemp();
                // Auto-balance: Tesla UART only — CAN modules handle balancing internally
                if (bms.getAutoBalance() && !bms.getBalanceInhibit() && !bms.isFaultedState())
                    bms.balanceCells();
                break;
            case CMU_BMW_PHEV:
                bms.getAllVoltTempFromPHEV();
                break;
            case CMU_VW_BMS:
                bms.getAllVoltTempFromVW();
                break;
            default:  // CMU_BMW_I3, CMU_BMW_I3_BUS, CMU_BMW_MINIE
                bms.getAllVoltTempFromCAN();
                break;
        }

        display.markDirty();

        // Fault buzzer
        if (bms.isFaultedState()) {
            if (!faultAlertActive ||
                (now % FAULT_BEEP_INTERVAL_MS < BMS_READ_INTERVAL_MS)) {
                M5.Speaker.tone(1200, 300);
                faultAlertActive = true;
            }
        } else {
            faultAlertActive = false;
        }
    }

    // CAN TX summary frames
    if (can.isRunning() && (now - lastCANSend >= CAN_SEND_INTERVAL_MS)) {
        lastCANSend = now;
        can.sendBatterySummary();
    }

    // Encoder navigation
    int delta = encoderCount - lastEncoderCount;
    if (delta != 0) {
        lastEncoderCount = encoderCount;
        int dir = (delta > 0) ? +1 : -1;
        int newPage = nextExistingPage(display.getPage(), dir);
        display.setPage(newPage);
    }

    // Short press - return to pack summary
    if (encoderBtnShort) {
        encoderBtnShort = false;
        display.setPage(0);
    }

    // Long press - WiFi toggle
    if (encoderBtnLong) {
        encoderBtnLong = false;
        toggleWifi();
    }

    // LVGL tick + render
    display.tick();

    // Serial console
    console.loop();

    // WiFi housekeeping
    if (wifi.isRunning()) wifi.loop();
}

// =============================================================================
// loadSettings
// =============================================================================
void loadSettings()
{
    EEPROM.get(EEPROM_PAGE, settings);
    if (settings.version != EEPROM_VERSION) {
        Logger::console("EEPROM mismatch (0x%X vs 0x%X). Loading defaults.",
                        settings.version, EEPROM_VERSION);
        settings.version          = EEPROM_VERSION;
        settings.checksum         = 0;
        settings.canSpeed         = CAN_BAUD_RATE;
        settings.batteryID        = 1;
        settings.logLevel         = Logger::Debug;
        settings.OverVSetpoint    = DEFAULT_OVER_V;
        settings.UnderVSetpoint   = DEFAULT_UNDER_V;
        settings.OverTSetpoint    = DEFAULT_OVER_T;
        settings.UnderTSetpoint   = DEFAULT_UNDER_T;
        settings.ChargeTSetpoint  = DEFAULT_CHARGE_T;
        settings.DisTSetpoint     = DEFAULT_DIS_T;
        settings.IgnoreTemp       = DEFAULT_IGNORE_TEMP;
        settings.IgnoreVolt       = DEFAULT_IGNORE_VOLT;
        settings.IgnoreTempThresh = DEFAULT_IGNORE_TEMP_THRESH;
        settings.balanceVoltage   = DEFAULT_BALANCE_V;
        settings.balanceHyst      = DEFAULT_BALANCE_HYST;
        settings.wifiEnabled      = 0;
        settings.balancingEnabled = 0;
        strncpy(settings.wifiSSID, WIFI_SSID_DEFAULT, 31);
        strncpy(settings.wifiPass, WIFI_PASS_DEFAULT, 31);
        settings.numCells    = DEFAULT_NUM_CELLS;
        settings.numSeries   = BMS_NUM_SERIES;
        settings.numParallel = BMS_NUM_PARALLEL;
        settings.socLo       = DEFAULT_SOC_LO;
        settings.socHi       = DEFAULT_SOC_HI;
        memset(settings.moduleCells, 0, sizeof(settings.moduleCells));
        // v6 new defaults
        settings.cmuType            = CMU_TESLA;
        settings.canInhibitEnabled  = DEFAULT_CAN_INHIBIT;
        settings.chargerHeartbeatID = DEFAULT_CHARGER_HB_ID;
        EEPROM.put(EEPROM_PAGE, settings);
        EEPROM.commit();
        Logger::console("Defaults written.");
    } else {
        Logger::console("Settings loaded: OV=%.2fV UV=%.2fV CMU=%s WiFi=%d CanInh=%d",
                        settings.OverVSetpoint, settings.UnderVSetpoint,
                        cmuTypeName(),
                        settings.wifiEnabled, settings.canInhibitEnabled);
    }
}

// =============================================================================
// applySettings
// =============================================================================
void applySettings()
{
    Logger::setLoglevel((Logger::LogLevel)settings.logLevel);
    bms.setBatteryID(settings.batteryID);
    bms.setPstrings(settings.numParallel > 0 ? settings.numParallel : BMS_NUM_PARALLEL);
    bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
    bms.setAutoBalance(settings.balancingEnabled != 0);
}

// =============================================================================
// nextExistingPage
// =============================================================================
int nextExistingPage(int current, int direction)
{
    if (direction > 0) {
        for (int i = current + 1; i <= MAX_MODULE_ADDR; i++)
            if (bms.getModule(i).isExisting()) return i;
        return 0;
    } else {
        if (current == 0) {
            for (int i = MAX_MODULE_ADDR; i >= 1; i--)
                if (bms.getModule(i).isExisting()) return i;
            return 0;
        }
        for (int i = current - 1; i >= 1; i--)
            if (bms.getModule(i).isExisting()) return i;
        return 0;
    }
}
