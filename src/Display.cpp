// =============================================================================
// Display.cpp  v6 - LVGL display manager for TeslaBMS M5Dial
//
// Changes from v5:
//   - Module page: larger fonts (montserrat_20 cell voltages, montserrat_16 labels)
//     2-column layout with 40px row spacing, 18px bars, supports up to 12 cells (i3)
//   - Pack page: "dV:" instead of broken Δ unicode glyph; WiFi long-press hint
//   - Settings page: shows CMU type, CAN inhibit status, charger heartbeat ID
//   - updatePackPage: WiFi line shows IP when up, "Btn-hold=WiFi" when off
// =============================================================================
#include "Display.h"
#include "bms_config.h"
#include "BMSModuleManager.h"
#include "CANManager.h"
#include "Logger.h"
#include <M5Unified.h>
#include <lvgl.h>

extern BMSModuleManager bms;
extern EEPROMSettings   settings;
extern String wifiDisplayIP;
extern CANManager       can;

// LVGL draw buffer
static lv_disp_draw_buf_t drawBuf;
static lv_color_t buf1[240 * 20];
static lv_color_t buf2[240 * 20];
static lv_disp_drv_t  dispDrv;
static lv_disp_t     *disp = nullptr;

// ---------------------------------------------------------------------------
// LVGL flush callback
// ---------------------------------------------------------------------------
static void lvglFlush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *colors)
{
    int32_t w = area->x2 - area->x1 + 1;
    int32_t h = area->y2 - area->y1 + 1;
    M5.Display.startWrite();
    M5.Display.setAddrWindow(area->x1, area->y1, w, h);
    M5.Display.writePixels((lgfx::rgb565_t *)colors, w * h);
    M5.Display.endWrite();
    lv_disp_flush_ready(drv);
}

// ---------------------------------------------------------------------------
// Pack page objects
// ---------------------------------------------------------------------------
static struct {
    lv_obj_t *screen;
    lv_obj_t *arc;
    lv_obj_t *voltLabel;
    lv_obj_t *socLabel;
    lv_obj_t *loLabel;
    lv_obj_t *hiLabel;
    lv_obj_t *deltaLabel;
    lv_obj_t *tempLabel;
    lv_obj_t *modLabel;
    lv_obj_t *ipLabel;
    lv_obj_t *faultLabel;
} packPage;

// ---------------------------------------------------------------------------
// Module page — up to 12 cells in 2 columns × 6 rows
// ---------------------------------------------------------------------------
#define MAX_CELLS_DISPLAY 16
static struct {
    lv_obj_t *screen;
    lv_obj_t *titleLabel;
    lv_obj_t *bar[MAX_CELLS_DISPLAY];
    lv_obj_t *cellLabel[MAX_CELLS_DISPLAY];    // "C1"
    lv_obj_t *cellValLabel[MAX_CELLS_DISPLAY]; // "3.852V"
    lv_obj_t *tempLabel;
    lv_obj_t *faultBadge;
    lv_obj_t *navLabel;
    int       addr;
} modPage;

// ---------------------------------------------------------------------------
// Settings page
// ---------------------------------------------------------------------------
static struct {
    lv_obj_t *screen;
    lv_obj_t *textLabel;
} settPage;

static lv_obj_t *activeScreen = nullptr;

// ---------------------------------------------------------------------------
// Styles
// ---------------------------------------------------------------------------
static lv_style_t styleBg;
static lv_style_t styleSmall;    // montserrat_12, slate-400
static lv_style_t styleMid;      // montserrat_16, slate-100
static lv_style_t styleBig;      // montserrat_28, white
static lv_style_t styleLarge;    // montserrat_20, white  [new v6]
static lv_style_t styleBarBg;
static lv_style_t styleBarOk;
static lv_style_t styleBarWarn;
static lv_style_t styleBarErr;
static bool stylesCreated = false;

static void createStyles()
{
    if (stylesCreated) return;
    stylesCreated = true;

    lv_style_init(&styleBg);
    lv_style_set_bg_color(&styleBg, lv_color_hex(0x0F172A));
    lv_style_set_text_color(&styleBg, lv_color_hex(0xF1F5F9));

    lv_style_init(&styleSmall);
    lv_style_set_text_font(&styleSmall, &lv_font_montserrat_12);
    lv_style_set_text_color(&styleSmall, lv_color_hex(0x94A3B8));

    lv_style_init(&styleMid);
    lv_style_set_text_font(&styleMid, &lv_font_montserrat_16);
    lv_style_set_text_color(&styleMid, lv_color_hex(0xF1F5F9));

    lv_style_init(&styleLarge);
    lv_style_set_text_font(&styleLarge, &lv_font_montserrat_20);
    lv_style_set_text_color(&styleLarge, lv_color_hex(0xFFFFFF));

    lv_style_init(&styleBig);
    lv_style_set_text_font(&styleBig, &lv_font_montserrat_28);
    lv_style_set_text_color(&styleBig, lv_color_hex(0xFFFFFF));

    lv_style_init(&styleBarBg);
    lv_style_set_bg_color(&styleBarBg, lv_color_hex(0x1E293B));
    lv_style_set_radius(&styleBarBg, 4);

    lv_style_init(&styleBarOk);
    lv_style_set_bg_color(&styleBarOk, lv_color_hex(0x22C55E));

    lv_style_init(&styleBarWarn);
    lv_style_set_bg_color(&styleBarWarn, lv_color_hex(0xF59E0B));

    lv_style_init(&styleBarErr);
    lv_style_set_bg_color(&styleBarErr, lv_color_hex(0xEF4444));
}

// ---------------------------------------------------------------------------
// begin()
// ---------------------------------------------------------------------------
Display::Display() : currentPage(0), dirty(true), initialized(false), lastLvglTick(0), showSecondHalf(false) {}

bool Display::begin()
{
    lv_init();

    lv_disp_draw_buf_init(&drawBuf, buf1, buf2, 240 * 20);

    lv_disp_drv_init(&dispDrv);
    dispDrv.hor_res  = 240;
    dispDrv.ver_res  = 240;
    dispDrv.flush_cb = lvglFlush;
    dispDrv.draw_buf = &drawBuf;
    disp = lv_disp_drv_register(&dispDrv);

    if (!disp) {
        Logger::error("LVGL display driver registration failed");
        return false;
    }

    createStyles();

    lv_obj_t *defScr = lv_scr_act();
    lv_obj_add_style(defScr, &styleBg, 0);
    lv_obj_set_style_bg_color(defScr, lv_color_hex(0x0F172A), 0);

    buildPackPage();
    buildModulePage(1);
    buildSettingsPage();

    lv_scr_load(packPage.screen);
    activeScreen = packPage.screen;

    initialized = true;
    Logger::info("LVGL display initialized (240x240)");
    return true;
}

// ---------------------------------------------------------------------------
// tick()
// ---------------------------------------------------------------------------
void Display::tick()
{
    if (!initialized) return;

    uint32_t now     = millis();
    uint32_t elapsed = now - lastLvglTick;
    if (elapsed > 0) {
        lv_tick_inc(elapsed);
        lastLvglTick = now;
    }

    // Alternate cells 1-6 / 7-12 every 1000ms on module pages if module has > 6 cells
    static uint32_t lastToggleTime = 0;
    if (currentPage >= 1 && currentPage <= MAX_MODULE_ADDR && bms.getModule(currentPage).isExisting()) {
        int activeCells = bms.getModuleCells(currentPage);
        if (activeCells > 6) {
            if (now - lastToggleTime >= 1000) {
                lastToggleTime = now;
                showSecondHalf = !showSecondHalf;
                dirty = true;
            }
        } else {
            showSecondHalf = false;
            lastToggleTime = now;
        }
    } else {
        showSecondHalf = false;
        lastToggleTime = now;
    }

    if (dirty) {
        dirty = false;
        int settPageNum = MAX_MODULE_ADDR + 1;
        if (currentPage == 0) {
            updatePackPage();
        } else if (currentPage == settPageNum) {
            updateSettingsPage();
        } else if (currentPage >= 1 && currentPage <= MAX_MODULE_ADDR
                   && bms.getModule(currentPage).isExisting()) {
            updateModulePage(currentPage);
        } else {
            currentPage = 0;
            updatePackPage();
        }
    }

    lv_task_handler();
}

void Display::setPage(int page)  { currentPage = page; dirty = true; }
int  Display::getPage()          { return currentPage; }
void Display::markDirty()        { dirty = true; }

void Display::showStartup(const char *msg)
{
    if (!initialized) return;
    lv_label_set_text(packPage.voltLabel, msg);
    lv_task_handler();
}

// ---------------------------------------------------------------------------
// buildPackPage
// ---------------------------------------------------------------------------
void Display::buildPackPage()
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0F172A), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // SoC arc
    lv_obj_t *arc = lv_arc_create(scr);
    lv_obj_set_size(arc, 220, 220);
    lv_obj_align(arc, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_rotation(arc, 135);
    lv_arc_set_bg_angles(arc, 0, 270);
    lv_arc_set_value(arc, 0);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x22C55E), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 8, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x1E293B), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 8, LV_PART_MAIN);
    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_arc_set_mode(arc, LV_ARC_MODE_NORMAL);
    packPage.arc = arc;

    lv_obj_t *vl = lv_label_create(scr);
    lv_obj_add_style(vl, &styleBig, 0);
    lv_obj_align(vl, LV_ALIGN_CENTER, 0, -28);
    lv_label_set_text(vl, "--V");
    packPage.voltLabel = vl;

    lv_obj_t *sl = lv_label_create(scr);
    lv_obj_add_style(sl, &styleMid, 0);
    lv_obj_set_style_text_color(sl, lv_color_hex(0x38BDF8), 0);
    lv_obj_align(sl, LV_ALIGN_CENTER, 0, 10);
    lv_label_set_text(sl, "SoC --%");
    packPage.socLabel = sl;

    lv_obj_t *lo = lv_label_create(scr);
    lv_obj_add_style(lo, &styleSmall, 0);
    lv_obj_align(lo, LV_ALIGN_CENTER, -45, 38);
    lv_label_set_text(lo, "Lo: --V");
    packPage.loLabel = lo;

    lv_obj_t *hi = lv_label_create(scr);
    lv_obj_add_style(hi, &styleSmall, 0);
    lv_obj_align(hi, LV_ALIGN_CENTER, 45, 38);
    lv_label_set_text(hi, "Hi: --V");
    packPage.hiLabel = hi;

    // dV: label - plain ASCII, renders in all fonts
    lv_obj_t *dl = lv_label_create(scr);
    lv_obj_add_style(dl, &styleSmall, 0);
    lv_obj_align(dl, LV_ALIGN_CENTER, 0, 54);
    lv_label_set_text(dl, "dV: ---V");
    packPage.deltaLabel = dl;

    lv_obj_t *tl = lv_label_create(scr);
    lv_obj_add_style(tl, &styleSmall, 0);
    lv_obj_set_style_text_color(tl, lv_color_hex(0xFB923C), 0);
    lv_obj_align(tl, LV_ALIGN_CENTER, 0, 70);
    lv_label_set_text(tl, "Temp: --C");
    packPage.tempLabel = tl;

    lv_obj_t *ml = lv_label_create(scr);
    lv_obj_add_style(ml, &styleSmall, 0);
    lv_obj_align(ml, LV_ALIGN_CENTER, 0, 85);
    lv_label_set_text(ml, "Mods: 0");
    packPage.modLabel = ml;

    // WiFi / long-press hint
    lv_obj_t *ip = lv_label_create(scr);
    lv_obj_add_style(ip, &styleSmall, 0);
    lv_obj_set_style_text_color(ip, lv_color_hex(0x64748B), 0);
    lv_obj_align(ip, LV_ALIGN_BOTTOM_MID, 0, -12);
    lv_label_set_text(ip, "Hold=WiFi");
    packPage.ipLabel = ip;

    lv_obj_t *fl = lv_label_create(scr);
    lv_obj_set_style_text_color(fl, lv_color_hex(0xEF4444), 0);
    lv_obj_add_style(fl, &styleMid, 0);
    lv_obj_align(fl, LV_ALIGN_TOP_MID, 0, 10);
    lv_label_set_text(fl, "");
    packPage.faultLabel = fl;

    packPage.screen = scr;
}

// ---------------------------------------------------------------------------
// buildModulePage  v7 - larger fonts, adaptive cell layout
//
// Layout by cell count:
//   <= 6 cells (Tesla):         2 columns × 3 rows, 40px row pitch
//   <= 8 cells (PHEV SP44):     2 columns × 4 rows, 33px row pitch
//   <= 12 cells (BMW i3):       2 columns × 6 rows, 26px row pitch
//   <= 16 cells (PHEV SP06/41): 4 columns × 4 rows, 33px row pitch, no bars
// Cells beyond resolved numCells are hidden.
// ---------------------------------------------------------------------------
void Display::buildModulePage(int addr)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0F172A), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t *tl = lv_label_create(scr);
    lv_obj_add_style(tl, &styleMid, 0);
    lv_obj_align(tl, LV_ALIGN_TOP_MID, 0, 8);
    lv_label_set_text(tl, "Module # --");
    modPage.titleLabel = tl;

    int activeCells = bms.getModuleCells(addr);

    // Select layout mode
    // mode 0: <=6  cells — 2 col × 3 row, pitch=36
    // mode 1: <=8  cells — 2 col × 4 row, pitch=33
    // mode 2: <=12 cells — 2 col × 6 row, pitch=26 (we force mode 0 to alternate 6 cells at a time)
    // mode 3: <=16 cells — 4 col × 4 row, pitch=33, no bars (too narrow)
    int mode;
    if      (activeCells <= 6)  mode = 0;
    else if (activeCells <= 8)  mode = 1;
    else if (activeCells <= 12) mode = 0; // force 6-cell layout to alternate 1-6 / 7-12
    else                        mode = 3;

    // Column x positions per mode
    const int COL_X_2[2] = {30, 128};          // 2-column layouts
    const int COL_X_4[4] = {14, 72, 130, 188}; // 4-column layout

    // Row y positions per mode
    const int ROW_Y_3[3] = {63, 99, 135};
    const int ROW_Y_4[4] = {45, 78, 111, 144};
    const int ROW_Y_6[6] = {42, 68, 94, 120, 146, 172};

    for (int i = 0; i < MAX_CELLS_DISPLAY; i++) {
        int col, row, x, y;
        bool showBar = true;

        switch (mode) {
            case 0:  // 2 col × 3 row
                col = i % 2; row = i / 2;
                x = COL_X_2[col]; y = ROW_Y_3[row % 3];
                break;
            case 1:  // 2 col × 4 row
                col = i % 2; row = i / 2;
                x = COL_X_2[col]; y = ROW_Y_4[row % 4];
                break;
            case 2:  // 2 col × 6 row
                col = i % 2; row = i / 2;
                x = COL_X_2[col]; y = ROW_Y_6[row % 6];
                break;
            default: // 4 col × 4 row — no bars (columns too narrow)
                col = i % 4; row = i / 4;
                x = COL_X_4[col]; y = ROW_Y_4[row % 4];
                showBar = false;
                break;
        }

        // Cell index label "C1"
        lv_obj_t *cl = lv_label_create(scr);
        lv_obj_add_style(cl, &styleSmall, 0);
        lv_obj_set_style_text_color(cl, lv_color_hex(0x94A3B8), 0);
        lv_obj_set_pos(cl, x, y);
        char clbuf[6]; snprintf(clbuf, sizeof(clbuf), "C%d", i + 1);
        lv_label_set_text(cl, clbuf);
        modPage.cellLabel[i] = cl;

        // Cell voltage value label
        lv_obj_t *vl = lv_label_create(scr);
        lv_obj_add_style(vl, showBar ? &styleLarge : &styleMid, 0);
        lv_obj_set_pos(vl, x, y + 12);
        lv_label_set_text(vl, "-.---");
        modPage.cellValLabel[i] = vl;

        // Bar chart — hidden in 4-column mode (too narrow)
        lv_obj_t *bar = lv_bar_create(scr);
        lv_obj_set_size(bar, 100, 6);
        lv_obj_set_pos(bar, x, y + 27);
        lv_bar_set_range(bar, 0, 1000);
        lv_bar_set_value(bar, 500, LV_ANIM_OFF);
        lv_obj_add_style(bar, &styleBarBg, LV_PART_MAIN);
        lv_obj_add_style(bar, &styleBarOk, LV_PART_INDICATOR);
        modPage.bar[i] = bar;

        // Hide cells beyond activeCells, or bars in 4-col mode
        if (i >= activeCells) {
            lv_obj_add_flag(cl,  LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(vl,  LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(bar, LV_OBJ_FLAG_HIDDEN);
        } else if (!showBar) {
            lv_obj_add_flag(bar, LV_OBJ_FLAG_HIDDEN);
        }
    }

    // Temp label — bottom strip
    lv_obj_t *tmt = lv_label_create(scr);
    lv_obj_add_style(tmt, &styleSmall, 0);
    lv_obj_set_style_text_color(tmt, lv_color_hex(0xFB923C), 0);
    lv_obj_align(tmt, LV_ALIGN_BOTTOM_MID, 0, -28);
    lv_label_set_text(tmt, "T1:--C  T2:--C");
    modPage.tempLabel = tmt;

    // Fault badge
    lv_obj_t *fb = lv_label_create(scr);
    lv_obj_add_style(fb, &styleSmall, 0);
    lv_obj_align(fb, LV_ALIGN_BOTTOM_MID, 0, -16);
    lv_label_set_text(fb, "");
    modPage.faultBadge = fb;

    // Nav hint
    lv_obj_t *nav = lv_label_create(scr);
    lv_obj_add_style(nav, &styleSmall, 0);
    lv_obj_set_style_text_color(nav, lv_color_hex(0x475569), 0);
    lv_obj_align(nav, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_label_set_text(nav, "Rot=nav  Btn=pack");
    modPage.navLabel = nav;

    modPage.screen = scr;
    modPage.addr   = addr;
}

// ---------------------------------------------------------------------------
// buildSettingsPage
// ---------------------------------------------------------------------------
void Display::buildSettingsPage()
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0F172A), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_obj_t *lbl = lv_label_create(scr);
    lv_obj_add_style(lbl, &styleSmall, 0);
    lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 20, 30);
    lv_label_set_text(lbl, "Settings\n...");
    lv_obj_set_style_text_line_space(lbl, 5, 0);
    settPage.textLabel = lbl;
    settPage.screen    = scr;
}

// ---------------------------------------------------------------------------
// updatePackPage
// ---------------------------------------------------------------------------
void Display::updatePackPage()
{
    float packV  = bms.getPackVoltage();
    float lowC   = bms.getLowCellVolt();
    float highC  = bms.getHighCellVolt();
    float avgT   = bms.getAvgTemperature();
    bool  fault  = bms.isFaultedState();
    int   nMods  = bms.getNumModules();

    int   ns     = settings.numSeries > 0 ? settings.numSeries : BMS_NUM_SERIES;
    float vPerMod = (ns > 0) ? (packV / (float)ns) : packV;
    float socRange = settings.socHi - settings.socLo;
    float socPct   = (socRange > 0.01f)
        ? (vPerMod - settings.socLo) / socRange * 100.0f : 0.0f;
    if (socPct > 100.0f) socPct = 100.0f;
    if (socPct < 0.0f)   socPct = 0.0f;

    char buf[48];

    lv_arc_set_value(packPage.arc, (int)socPct);
    lv_obj_set_style_arc_color(packPage.arc,
        fault ? lv_color_hex(0xEF4444) : lv_color_hex(0x22C55E),
        LV_PART_INDICATOR);

    snprintf(buf, sizeof(buf), "%.1fV", packV);
    lv_label_set_text(packPage.voltLabel, buf);
    lv_obj_set_style_text_color(packPage.voltLabel,
        fault ? lv_color_hex(0xEF4444) : lv_color_hex(0xFFFFFF), 0);

    snprintf(buf, sizeof(buf), "SoC %d%%", (int)socPct);
    lv_label_set_text(packPage.socLabel, buf);

    snprintf(buf, sizeof(buf), "Lo:%.3fV", lowC);
    lv_label_set_text(packPage.loLabel, buf);
    snprintf(buf, sizeof(buf), "Hi:%.3fV", highC);
    lv_label_set_text(packPage.hiLabel, buf);

    // Plain ASCII "dV:" — works in every font, no Unicode glyph needed
    float delta = highC - lowC;
    snprintf(buf, sizeof(buf), "dV:%.3fV", delta);
    lv_label_set_text(packPage.deltaLabel, buf);
    lv_obj_set_style_text_color(packPage.deltaLabel,
        delta > 0.05f ? lv_color_hex(0xF59E0B) : lv_color_hex(0x94A3B8), 0);

    snprintf(buf, sizeof(buf), "%.1fC", avgT);
    lv_label_set_text(packPage.tempLabel, buf);

    snprintf(buf, sizeof(buf), "Mods: %d", nMods);
    lv_label_set_text(packPage.modLabel, buf);

    lv_label_set_text(packPage.faultLabel, fault ? "! FAULT" : "");

    // Bottom line: show IP if WiFi up, "Hold=WiFi" hint if off
    if (wifiDisplayIP.length() > 0) {
        snprintf(buf, sizeof(buf), "%s", wifiDisplayIP.c_str());
    } else {
        snprintf(buf, sizeof(buf), "Hold=WiFi toggle");
    }
    lv_label_set_text(packPage.ipLabel, buf);
    lv_obj_set_style_text_color(packPage.ipLabel,
        wifiDisplayIP.length() > 0 ? lv_color_hex(0x38BDF8) : lv_color_hex(0x475569), 0);

    if (lv_scr_act() != packPage.screen)
        lv_scr_load(packPage.screen);
}

// ---------------------------------------------------------------------------
// updateModulePage  v6 — larger voltage labels, 12-cell aware
// ---------------------------------------------------------------------------
void Display::updateModulePage(int addr)
{
    BMSModule &mod = bms.getModule(addr);
    if (!mod.isExisting()) { setPage(0); return; }

    modPage.addr = addr;

    int activeCells = bms.getModuleCells(addr);

    char buf[48];
    if (activeCells == 12) {
        snprintf(buf, sizeof(buf), "Mod#%d (%s) %.2fV", addr, showSecondHalf ? "7-12" : "1-6", mod.getModuleVoltage());
    } else {
        snprintf(buf, sizeof(buf), "Mod#%d  %.2fV", addr, mod.getModuleVoltage());
    }
    lv_label_set_text(modPage.titleLabel, buf);

    for (int i = 0; i < MAX_CELLS_DISPLAY; i++) {
        bool shouldHide = false;
        if (i >= activeCells) {
            shouldHide = true;
        } else if (activeCells == 12) {
            if (showSecondHalf) {
                if (i < 6) shouldHide = true;
            } else {
                if (i >= 6) shouldHide = true;
            }
        }

        if (shouldHide) {
            lv_obj_add_flag(modPage.cellLabel[i],   LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(modPage.cellValLabel[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(modPage.bar[i],          LV_OBJ_FLAG_HIDDEN);
            continue;
        }

        lv_obj_clear_flag(modPage.cellLabel[i],   LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(modPage.cellValLabel[i], LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(modPage.bar[i],          LV_OBJ_FLAG_HIDDEN);

        float cv = mod.getCellVoltage(i);

        // Voltage value in large font
        snprintf(buf, sizeof(buf), "%.3f", cv);
        lv_label_set_text(modPage.cellValLabel[i], buf);

        // Bar 0-1000 from 3.0-4.25V
        int barVal = (int)(((cv - 3.0f) / (4.25f - 3.0f)) * 1000.0f);
        if (barVal < 0) barVal = 0;
        if (barVal > 1000) barVal = 1000;
        lv_bar_set_value(modPage.bar[i], barVal, LV_ANIM_OFF);

        // Bar colour
        lv_style_t *bs = &styleBarOk;
        if      (cv < settings.UnderVSetpoint) bs = &styleBarErr;
        else if (cv > settings.OverVSetpoint)  bs = &styleBarWarn;
        lv_obj_remove_style(modPage.bar[i], NULL, LV_PART_INDICATOR);
        lv_obj_add_style(modPage.bar[i], bs, LV_PART_INDICATOR);

        // Voltage label colour
        lv_obj_set_style_text_color(modPage.cellValLabel[i],
            (cv < settings.UnderVSetpoint) ? lv_color_hex(0xEF4444) :
            (cv > settings.OverVSetpoint)  ? lv_color_hex(0xF59E0B) :
                                              lv_color_hex(0xFFFFFF), 0);
    }

    // Suppress disconnected thermistors (reads ~-89°C on BQ76PL536A)
    float t1 = mod.getTemperature(0);
    float t2 = mod.getTemperature(1);
    char t1s[10], t2s[10];
    if (t1 > settings.IgnoreTempThresh) snprintf(t1s, sizeof(t1s), "%.1fC", t1);
    else                                 snprintf(t1s, sizeof(t1s), "--");
    if (t2 > settings.IgnoreTempThresh) snprintf(t2s, sizeof(t2s), "%.1fC", t2);
    else                                 snprintf(t2s, sizeof(t2s), "--");
    snprintf(buf, sizeof(buf), "T1:%s  T2:%s", t1s, t2s);
    lv_label_set_text(modPage.tempLabel, buf);

    uint8_t faults = mod.getFaults() & ~0x02;  // mask CUV on unused inputs
    if (faults > 0) {
        snprintf(buf, sizeof(buf), "FAULT:0x%02X", mod.getFaults());
        lv_label_set_text(modPage.faultBadge, buf);
        lv_obj_set_style_text_color(modPage.faultBadge, lv_color_hex(0xEF4444), 0);
    } else {
        lv_label_set_text(modPage.faultBadge, "OK");
        lv_obj_set_style_text_color(modPage.faultBadge, lv_color_hex(0x22C55E), 0);
    }

    snprintf(buf, sizeof(buf), "Mod%d/%d  Btn=pack", addr, bms.getNumModules());
    lv_label_set_text(modPage.navLabel, buf);

    if (lv_scr_act() != modPage.screen)
        lv_scr_load(modPage.screen);
}

// ---------------------------------------------------------------------------
// updateSettingsPage  v6 — includes CMU type and CAN inhibit status
// ---------------------------------------------------------------------------
void Display::updateSettingsPage()
{
    char buf[320];
    static const char *cmuNames[] = {
        "Tesla UART", "BMW i3 CAN", "BMW i3 Bus", "BMW Mini-E", "BMW PHEV", "VW BMS"
    };
    snprintf(buf, sizeof(buf),
        "M5DialBMS v7\n"
        "CMU: %s\n"
        "Cells: %d  Ser: %d  Par: %d\n"
        "OV:%.2fV  UV:%.2fV\n"
        "BalV:%.2fV +/-%.3fV\n"
        "BalInh: GPIO=%s CAN=%s\n"
        "ChgHB: 0x%03X  %s\n"
        "AutoBal: %s\n"
        "Mods: %d / %d max",
        cmuNames[settings.cmuType <= 5 ? settings.cmuType : 0],
        settings.numCells, settings.numSeries, settings.numParallel,
        settings.OverVSetpoint, settings.UnderVSetpoint,
        settings.balanceVoltage, settings.balanceHyst,
        bms.getBalanceInhibit() ? "ON" : "off",
        settings.canInhibitEnabled ? (can.getChargerActive() ? "CHG-OK" : "TIMEOUT") : "off",
        settings.chargerHeartbeatID,
        settings.canInhibitEnabled ? "en" : "dis",
        bms.getAutoBalance() ? "ON" : "OFF",
        bms.getNumModules(), MAX_MODULE_ADDR
    );
    lv_label_set_text(settPage.textLabel, buf);

    if (lv_scr_act() != settPage.screen)
        lv_scr_load(settPage.screen);
}
