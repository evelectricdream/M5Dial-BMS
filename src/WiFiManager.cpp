// =============================================================================
// WiFiManager.cpp  v5.1
// Routes:
//   GET /            -> HTML dashboard (3 tabs: Pack/Modules, CAN Feed, Settings)
//   GET /api/data    -> JSON pack + module data (respects numCells)
//   GET /api/can     -> JSON array of last 50 CAN frames (rolling buffer)
//   GET /api/settings -> current settings JSON
// =============================================================================
#include "WiFiManager.h"
#include "CANManager.h"
#include "bms_config.h"
#include "BMSModuleManager.h"
#include "Logger.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <driver/twai.h>

extern BMSModuleManager bms;
extern EEPROMSettings   settings;
extern CANManager       can;
extern WiFiManager      wifi;   // for gvretPushFrame in wifiLogCAN()

static AsyncWebServer *server = nullptr;

// ---------------------------------------------------------------------------
// CAN frame rolling buffer - populated by CANManager::sendFrame()
// ---------------------------------------------------------------------------
#define CAN_LOG_SIZE 50
struct CANLogEntry {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  len;
    uint32_t ts;   // millis()
};
static CANLogEntry canLog[CAN_LOG_SIZE];
static int canLogHead = 0;
static int canLogCount = 0;

void wifiLogCAN(uint32_t id, uint8_t *data, uint8_t len)
{
    canLog[canLogHead] = { id, {0}, len, millis() };
    memcpy(canLog[canLogHead].data, data, len < 8 ? len : 8);
    canLogHead = (canLogHead + 1) % CAN_LOG_SIZE;
    if (canLogCount < CAN_LOG_SIZE) canLogCount++;
    // Push to any connected SavvyCAN/GVRET client
    wifi.gvretPushFrame(id, data, len, false);
}

// SimpBMS frame decoder - returns human-readable description
static String decodeCAN(uint32_t id, uint8_t *d)
{
    char buf[80];
    switch (id) {
        case 0x351: {
            float cvh = ((int16_t)(d[1]<<8|d[0])) * 0.1f;
            float ccl = ((int16_t)(d[3]<<8|d[2])) * 0.1f;
            float dcl = ((int16_t)(d[5]<<8|d[4])) * 0.1f;
            float cvl = ((int16_t)(d[7]<<8|d[6])) * 0.1f;
            snprintf(buf,sizeof(buf),"CVH:%.1fV CCL:%.1fA DCL:%.1fA CVL:%.1fV",cvh,ccl,dcl,cvl);
            return buf;
        }
        case 0x355: {
            int soc = d[1]<<8|d[0];
            int soh = d[3]<<8|d[2];
            snprintf(buf,sizeof(buf),"SoC:%d%% SoH:%d%%",soc,soh);
            return buf;
        }
        case 0x356: {
            float v = ((int16_t)(d[1]<<8|d[0])) * 0.01f;
            float a = ((int16_t)(d[3]<<8|d[2])) * 0.1f;
            float t = ((int16_t)(d[5]<<8|d[4])) * 0.1f;
            snprintf(buf,sizeof(buf),"V:%.2f A:%.1f T:%.1fC",v,a,t);
            return buf;
        }
        case 0x35A: {
            snprintf(buf,sizeof(buf),"Alarms:0x%02X Faults:0x%02X",d[0],d[1]);
            return buf;
        }
        case 0x35E:
            snprintf(buf,sizeof(buf),"Mfr: %.8s",(char*)d);
            return buf;
        case 0x35F:
            snprintf(buf,sizeof(buf),"Chem:%.2s HW:%d.%d FW:%d.%d",(char*)d,d[3],d[2],d[5],d[4]);
            return buf;
        default:
            return "";
    }
}

WiFiManager::WiFiManager()
    : running(false), ipAddr(""),
      gvretServer(23), gvretClientConnected(false) {}

// ---------------------------------------------------------------------------
// HTML - single file, 3-tab layout
// ---------------------------------------------------------------------------
static const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>M5Dial BMS</title>
<style>
:root{--ok:#22c55e;--warn:#f59e0b;--err:#ef4444;--bg:#0f172a;--card:#1e293b;--text:#f1f5f9;--sub:#94a3b8;--border:#334155}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--text);min-height:100vh}
h1{text-align:center;font-size:1.3rem;padding:1rem;color:var(--ok)}
.tabs{display:flex;border-bottom:2px solid var(--border);padding:0 1rem}
.tab{padding:.6rem 1.2rem;cursor:pointer;font-size:.85rem;color:var(--sub);border-bottom:2px solid transparent;margin-bottom:-2px;transition:color .2s}
.tab.active{color:var(--ok);border-bottom-color:var(--ok)}
.tab:hover{color:var(--text)}
.panel{display:none;padding:1rem}
.panel.active{display:block}
/* Pack */
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:.6rem;margin-bottom:1.2rem}
.stat{background:var(--card);border-radius:.6rem;padding:.8rem;text-align:center}
.stat .val{font-size:1.6rem;font-weight:700}
.stat .lbl{font-size:.7rem;color:var(--sub);margin-top:.2rem}
.ok{color:var(--ok)} .warn{color:var(--warn)} .err{color:var(--err)}
.modules{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));gap:.8rem}
.module{background:var(--card);border-radius:.6rem;padding:.8rem;border:2px solid var(--border)}
.module.faulted{border-color:var(--err)}
.module h3{font-size:.85rem;margin-bottom:.6rem;display:flex;justify-content:space-between;align-items:center}
.cells{display:grid;gap:.3rem}
.cell{background:#0f172a;border-radius:.3rem;padding:.3rem .5rem;display:flex;align-items:center;gap:.4rem;font-size:.78rem}
.cell .cn{width:1.8rem;color:var(--sub)}
.cell .cv{width:3.5rem;font-weight:600}
.cell .bar{flex:1;height:6px;background:var(--border);border-radius:3px;overflow:hidden}
.cell .bf{height:100%;background:var(--ok);transition:width .4s}
.temps{margin-top:.5rem;font-size:.75rem;color:var(--sub)}
.badge{padding:.15rem .4rem;border-radius:.25rem;font-size:.65rem;font-weight:700}
.badge.ok{background:#14532d;color:var(--ok)}
.badge.err{background:#7f1d1d;color:var(--err)}
/* CAN */
#canTable{width:100%;border-collapse:collapse;font-size:.78rem;font-family:monospace}
#canTable th{text-align:left;padding:.4rem .6rem;color:var(--sub);border-bottom:1px solid var(--border);font-size:.7rem}
#canTable td{padding:.3rem .6rem;border-bottom:1px solid #1e293b}
#canTable tr:hover td{background:#1e293b}
.id351{color:#38bdf8}.id355{color:#a78bfa}.id356{color:#34d399}
.id35a{color:#f87171}.id35e{color:#94a3b8}.id35f{color:#94a3b8}
.can-send{background:var(--card);border-radius:.6rem;padding:.8rem;margin-bottom:1rem}
.can-send h4{font-size:.8rem;color:var(--sub);margin-bottom:.6rem;text-transform:uppercase;letter-spacing:.05em}
.can-row{display:flex;gap:.5rem;flex-wrap:wrap;align-items:flex-end}
.can-row label{font-size:.75rem;color:var(--sub);display:block;margin-bottom:.2rem}
.can-row input{background:#0f172a;border:1px solid var(--border);color:var(--text);padding:.35rem .5rem;border-radius:.3rem;font-family:monospace;font-size:.82rem;width:100%}
.fi-id{width:80px} .fi-data{flex:1;min-width:180px}
/* Settings */
.sform{display:grid;grid-template-columns:1fr 1fr;gap:.5rem 1rem;margin-bottom:1rem}
@media(max-width:500px){.sform{grid-template-columns:1fr}}
.sf-group{background:var(--card);border-radius:.5rem;padding:.7rem;grid-column:span 2}
.sf-group h4{font-size:.75rem;color:var(--sub);text-transform:uppercase;letter-spacing:.05em;margin-bottom:.6rem}
.sf-group .sform{grid-template-columns:1fr 1fr;margin:0}
.sf-field{display:flex;flex-direction:column;gap:.2rem}
.sf-field label{font-size:.72rem;color:var(--sub)}
.sf-field input{background:#0f172a;border:1px solid var(--border);color:var(--text);padding:.3rem .5rem;border-radius:.3rem;font-size:.82rem;width:100%}
.sf-field input:focus{outline:none;border-color:var(--ok)}
.sf-field .sf-cur{font-size:.68rem;color:#475569;margin-top:.1rem}
.btn{padding:.45rem 1.1rem;border:none;border-radius:.4rem;cursor:pointer;font-size:.82rem;font-weight:600}
.btn-ok{background:#15803d;color:#fff}
.btn-ok:hover{background:#16a34a}
.btn-send{background:#1d4ed8;color:#fff}
.btn-send:hover{background:#2563eb}
.btn-warn{background:#92400e;color:#fff}
.btn-warn:hover{background:#b45309}
.mod-cell-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(90px,1fr));gap:.4rem}
.mcell-slot{background:#0f172a;border:1px solid var(--border);border-radius:.3rem;padding:.4rem .5rem}
.mcell-slot.mcell-active{border-color:var(--warn);background:#1c1407}
.mcell-lbl{font-size:.68rem;color:var(--sub);margin-bottom:.25rem}
.mcell-sel{width:100%;background:#0f172a;border:1px solid var(--border);color:var(--text);padding:.2rem .3rem;border-radius:.25rem;font-size:.75rem}
.mcell-slot.mcell-active .mcell-sel{border-color:var(--warn)}
.toast{position:fixed;bottom:1.5rem;right:1.5rem;background:#1e293b;border:1px solid var(--border);padding:.6rem 1rem;border-radius:.5rem;font-size:.8rem;opacity:0;transition:opacity .3s;pointer-events:none}
.toast.show{opacity:1}
#status{text-align:center;font-size:.7rem;color:var(--sub);padding:.5rem}
</style>
</head>
<body>
<h1>&#9889; M5Dial BMS</h1>
<div class="tabs">
  <div class="tab active" onclick="switchTab('pack')">Pack &amp; Modules</div>
  <div class="tab" onclick="switchTab('can')">CAN Feed</div>
  <div class="tab" onclick="switchTab('settings')">Settings</div>
</div>

<div id="pack" class="panel active">
  <div class="grid" id="packStats"></div>
  <div class="modules" id="modules"></div>
</div>

<div id="can" class="panel">
  <div class="can-send">
    <h4>Transmit CAN Frame</h4>
    <div class="can-row">
      <div class="fi-id">
        <label>ID (hex)</label>
        <input id="txId" placeholder="0x123" value="0x123">
      </div>
      <div class="fi-data">
        <label>Data bytes (space-separated hex)</label>
        <input id="txData" placeholder="01 02 03 04 05 06 07 08">
      </div>
      <button class="btn btn-send" onclick="sendCAN()">Send</button>
    </div>
    <div id="txStatus" style="font-size:.75rem;color:var(--sub);margin-top:.4rem"></div>
  </div>
  <table id="canTable">
    <thead><tr><th>Time</th><th>ID</th><th>Data</th><th>Decoded</th></tr></thead>
    <tbody id="canBody"></tbody>
  </table>
</div>

<div id="settings" class="panel">
  <div class="sform" id="settForm"></div>
  <div style="display:flex;gap:.5rem;flex-wrap:wrap">
    <button class="btn btn-ok" onclick="applySettings()">&#10003; Apply &amp; Save</button>
    <button class="btn btn-warn" onclick="fetchSettings()">&#8635; Reload</button>
  </div>
  <div id="settStatus" style="font-size:.75rem;color:var(--sub);margin-top:.5rem"></div>
</div>

<div id="status">Connecting...</div>
<div class="toast" id="toast"></div>

<script>
let activeTab='pack';

// Settings field schema: [key, label, type, min, max, step, group]
const FIELDS=[
  // group header acts as separator
  ['__g','Pack Topology'],
  ['numCells',   'Cells per module',    'int',   1,  6, 1],
  ['numSeries',  'Modules in series',   'int',   1, 62, 1],
  ['numParallel','Modules in parallel', 'int',   1, 10, 1],
  ['socLo_V',    'SoC 0% V/module',     'float', 10,30,0.1],
  ['socHi_V',    'SoC 100% V/module',   'float', 10,30,0.1],
  ['__g','Cell Thresholds'],
  ['ovSetpoint', 'OV limit (V)',        'float', 3.0,5.0,0.01],
  ['uvSetpoint', 'UV limit (V)',        'float', 2.0,4.0,0.01],
  ['ignoreVolt', 'Ignore cell below (V)','float',0,3.5,0.1],
  ['otSetpoint', 'Over-temp (°C)',      'float', 20,100,1],
  ['utSetpoint', 'Under-temp (°C)',     'float',-40, 30,1],
  ['ignoreTempC','Ignore temp below (°C)','float',-100,0,1],
  ['__g','Balancing'],
  ['balVoltage', 'Balance target (V)',  'float', 3.0,4.5,0.01],
  ['balHyst',    'Balance hyst (V)',    'float', 0.001,0.5,0.001],
  ['autoBalance','Auto-balance (0/1)',  'int',   0,1,1],
  ['__g','Connectivity'],
  ['wifiEnabled','WiFi at boot (0/1)',  'int',   0,1,1],
  ['batteryID',  'CAN Battery ID',      'int',   1,14,1],
  ['logLevel',   'Log level (0-4)',     'int',   0,4,1],
];

let currentSettings={};

function toast(msg,ok=true){
  const el=document.getElementById('toast');
  el.textContent=msg;
  el.style.borderColor=ok?'var(--ok)':'var(--err)';
  el.classList.add('show');
  setTimeout(()=>el.classList.remove('show'),3000);
}

function switchTab(t){
  activeTab=t;
  document.querySelectorAll('.tab').forEach((el,i)=>
    el.classList.toggle('active',['pack','can','settings'][i]===t));
  document.querySelectorAll('.panel').forEach(el=>
    el.classList.toggle('active',el.id===t));
  if(t==='settings') fetchSettings();
}

// ---- Pack tab ----
function colorV(v,lo,hi){
  if(v<=0)return'sub';if(v<lo)return'err';if(v>hi)return'warn';return'ok';
}
function barW(v){return Math.max(0,Math.min(100,((v-3.0)/(4.25-3.0))*100));}
function barCol(v){return v<3.0?'#ef4444':v>4.2?'#f59e0b':'#22c55e';}

async function fetchData(){
  try{
    const r=await fetch('/api/data');
    const d=await r.json();
    renderPack(d);renderModules(d);
    document.getElementById('status').textContent='Updated: '+new Date().toLocaleTimeString();
  }catch(e){document.getElementById('status').textContent='Error: '+e.message;}
}

function renderPack(d){
  const p=d.pack,fc=p.faulted?'err':'ok';
  document.getElementById('packStats').innerHTML=`
    <div class="stat"><div class="val ${fc}">${Number(p.voltage).toFixed(1)}V</div><div class="lbl">Pack Voltage</div></div>
    <div class="stat"><div class="val ${fc}">${p.soc}%</div><div class="lbl">SoC</div></div>
    <div class="stat"><div class="val">${Number(p.cellLow).toFixed(3)}V</div><div class="lbl">Cell Min</div></div>
    <div class="stat"><div class="val">${Number(p.cellHigh).toFixed(3)}V</div><div class="lbl">Cell Max</div></div>
    <div class="stat"><div class="val ${Number(p.delta)>0.05?'warn':'ok'}">${Number(p.delta).toFixed(3)}V</div><div class="lbl">Delta</div></div>
    <div class="stat"><div class="val">${Number(p.temp).toFixed(1)}&deg;C</div><div class="lbl">Avg Temp</div></div>
    <div class="stat"><div class="val">${p.modules}</div><div class="lbl">Modules</div></div>
    <div class="stat"><div class="val ${p.balancing?'warn':'ok'}">${p.balancing?'BAL':'idle'}</div><div class="lbl">Balancing</div></div>`;
}

function renderModules(d){
  const uvLo=d.pack.uvSetpoint||3.0,ovHi=d.pack.ovSetpoint||4.2;
  let html='';
  for(const m of d.modules){
    const rf=m.faulted&&(m.faultCode&~0x02)>0;
    const badge=rf?`<span class="badge err">FAULT 0x${m.faultCode.toString(16).toUpperCase()}</span>`
                  :`<span class="badge ok">OK</span>`;
    let cells='';
    const nc=m.numCells||d.pack.numCells||6;
    for(let i=0;i<nc;i++){
      const v=m.cells[i],cc=colorV(v,uvLo,ovHi);
      cells+=`<div class="cell"><div class="cn">C${i+1}</div><div class="cv ${cc}">${v.toFixed(3)}</div>
        <div class="bar"><div class="bf" style="width:${barW(v)}%;background:${barCol(v)}"></div></div></div>`;
    }
    const ignT=d.pack.ignoreTempThresh||-70;
    let temps='';
    if(m.t1>ignT) temps+=`T1:${Number(m.t1).toFixed(1)}&deg;C&nbsp;&nbsp;`;
    if(m.t2>ignT) temps+=`T2:${Number(m.t2).toFixed(1)}&deg;C`;
    if(!temps) temps='<span style="color:#475569">No temp sensors</span>';
    html+=`<div class="module ${rf?'faulted':''}">
      <h3><span>Module ${m.addr} &mdash; ${Number(m.voltage).toFixed(2)}V</span>${badge}</h3>
      <div class="cells">${cells}</div><div class="temps">${temps}</div></div>`;
  }
  document.getElementById('modules').innerHTML=html||'<p style="color:var(--sub);padding:1rem">No modules found</p>';
}

// ---- CAN tab ----
async function sendCAN(){
  const idStr=document.getElementById('txId').value.trim();
  const dataStr=document.getElementById('txData').value.trim();
  const id=parseInt(idStr,16);
  if(isNaN(id)||id<0||id>0x7FF){
    document.getElementById('txStatus').textContent='Invalid ID (0x000-0x7FF)';return;
  }
  const bytes=dataStr.split(/\s+/).filter(Boolean).map(s=>parseInt(s,16));
  if(bytes.some(isNaN)||bytes.length<1||bytes.length>8){
    document.getElementById('txStatus').textContent='Invalid data (1-8 hex bytes)';return;
  }
  try{
    const r=await fetch('/api/cantx',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({id,data:bytes})
    });
    const j=await r.json();
    if(j.ok){
      document.getElementById('txStatus').textContent='Sent OK';
      toast('CAN frame sent');
    } else {
      document.getElementById('txStatus').textContent='TX error: '+j.error;
      toast('TX failed: '+j.error,false);
    }
  }catch(e){document.getElementById('txStatus').textContent='Network error';}
}

async function fetchCAN(){
  if(activeTab!=='can')return;
  try{
    const r=await fetch('/api/can');
    const frames=await r.json();
    let rows='';
    for(let i=frames.length-1;i>=0;i--){
      const f=frames[i];
      const idHex='0x'+f.id.toString(16).toUpperCase().padStart(3,'0');
      const cls='id'+f.id.toString(16).toLowerCase();
      const dataHex=f.data.map(b=>b.toString(16).toUpperCase().padStart(2,'0')).join(' ');
      rows+=`<tr><td>${(f.ts/1000).toFixed(1)}s</td><td class="${cls}">${idHex}</td><td>${dataHex}</td><td>${f.decoded}</td></tr>`;
    }
    document.getElementById('canBody').innerHTML=rows;
  }catch(e){}
}

// ---- Settings tab ----
async function fetchSettings(){
  try{
    const r=await fetch('/api/settings');
    currentSettings=await r.json();
    renderSettingsForm();
    document.getElementById('settStatus').textContent='';
    document.getElementById('settStatus').style.color='var(--sub)';
  }catch(e){
    document.getElementById('settStatus').textContent=
      'Connection lost (device rebooting?) — retrying...';
    document.getElementById('settStatus').style.color='var(--warn)';
    setTimeout(fetchSettings, 1500);
  }
}

// pollForReboot - called after a cmuType (or other reboot-requiring) save.
// Waits for the device to actually drop offline, then come back, then
// forces one fresh fetch. This avoids two failure modes of naive polling:
//   - refreshing too early, while the old firmware is still running and
//     would just show the pre-reboot value again
//   - never refreshing at all if the user reboots well after the save
//     (e.g. walks away, comes back later) since a plain setTimeout would
//     have long since stopped retrying
async function pollForReboot(){
  let sawOffline = false;
  const maxWaitMs = 120000;   // give up after 2 minutes
  const intervalMs = 1000;
  const startedAt = Date.now();

  while (Date.now() - startedAt < maxWaitMs) {
    await new Promise(res => setTimeout(res, intervalMs));
    try {
      const r = await fetch('/api/settings');
      if (!r.ok) throw new Error('bad status');
      const data = await r.json();
      if (sawOffline) {
        // Device was down, now it's back - this is the post-reboot state
        currentSettings = data;
        renderSettingsForm();
        document.getElementById('settStatus').textContent =
          'Device rebooted — settings refreshed.';
        document.getElementById('settStatus').style.color='var(--ok)';
        toast('Device back online, settings refreshed');
        return;
      }
      // Still online and hasn't gone down yet - keep waiting for the
      // actual reboot, don't refresh on stale pre-reboot data.
    } catch (e) {
      sawOffline = true;
      document.getElementById('settStatus').textContent =
        'Device offline, waiting for reboot to complete...';
    }
  }
  document.getElementById('settStatus').textContent =
    'Gave up waiting for reboot. Reload the page once the device is back.';
  document.getElementById('settStatus').style.color='var(--err)';
}

function renderSettingsForm(){
  const s=currentSettings;
  let html='';
  let inGroup=false;
  for(const f of FIELDS){
    if(f[0]==='__g'){
      if(inGroup) html+='</div></div>';
      html+=`<div class="sf-group" style="grid-column:span 2"><h4>${f[1]}</h4><div class="sform">`;
      inGroup=true;
      continue;
    }
    const [key,lbl,type,min,max,step]=f;
    const cur=s[key]!==undefined?s[key]:'';
    const stepAttr=type==='float'?`step="${step}"`:`step="1"`;
    html+=`<div class="sf-field">
      <label for="sf_${key}">${lbl}</label>
      <input id="sf_${key}" type="number" min="${min}" max="${max}" ${stepAttr} value="${cur}">
      <div class="sf-cur">Current: <strong>${cur}</strong></div>
    </div>`;
  }
  if(inGroup) html+='</div></div>';

  // Per-module cell count section - only show modules that exist (have been found)
  // v6/v7 CMU / inhibit fields - values are applied AFTER innerHTML below,
  // since these elements don't exist in the DOM until that assignment runs.
  const mc=s.moduleCells||[];
  const globalCells=s.numCells||6;
  // Collect which module slots are nonzero-override OR are found modules
  // We show all slots 1..maxModules that have an override set, plus allow setting any 1..20
  html+=`<div class="sf-group" style="grid-column:span 2">
  <h3>CMU Type &amp; Balance Inhibit (v7)</h3>
  <div class=\"row\"><label>CMU Type (reboot)</label><select id=\"cmuType\"><option value=\"0\">Tesla UART</option><option value=\"1\">BMW i3 CAN</option><option value=\"2\">BMW i3 Bus Pack</option><option value=\"3\">BMW Mini-E</option><option value=\"4\">BMW PHEV SP06/SP41/SP44</option><option value=\"5\">VW BMS Type 5</option></select></div>
  <div class=\"row\"><label>Balance Inhibit</label><select id=\"canInhibitEnabled\"><option value=\"0\">GPIO only</option><option value=\"1\">GPIO+CAN charger</option></select></div>
  <div class=\"row\"><label>Charger HB ID</label><input type=\"text\" id=\"chargerHeartbeatID\" style=\"width:80px\" placeholder=\"0x305\"></div>
  <div class=\"row\"><label>Charger active</label><span id=\"chargerStatus\" style=\"font-weight:bold\">--</span></div>
    <h4>Per-Module Cell Count Override</h4>
    <p style="font-size:.72rem;color:var(--sub);margin-bottom:.7rem">
      Global default: <strong>${globalCells} cells</strong>
      (change via &#8220;Cells per module&#8221; field above).
      Set a module to <strong>0</strong> to inherit the global default.
      Only modules with an active override are highlighted.
    </p>
    <div class="mod-cell-grid" id="modCellGrid">`;
  const maxMod=s.maxModules||20;
  for(let i=1;i<=maxMod;i++){
    const ov=mc[i]||0;
    const active=ov>0;
    html+=`<div class="mcell-slot ${active?'mcell-active':''}">
      <div class="mcell-lbl">Mod ${i}</div>
      <select class="mcell-sel" id="mcs_${i}">
        <option value="0" ${ov===0?'selected':''}>default (${globalCells})</option>
        <option value="4" ${ov===4?'selected':''}>4S</option>
        <option value="5" ${ov===5?'selected':''}>5S</option>
        <option value="6" ${ov===6?'selected':''}>6S</option>
        <option value="8" ${ov===8?'selected':''}>8S</option>
        <option value="12" ${ov===12?'selected':''}>12S</option>
        <option value="16" ${ov===16?'selected':''}>16S</option>
      </select>
    </div>`;
  }
  html+='</div></div>';

  document.getElementById('settForm').innerHTML=html;

  // Now that the select/input elements actually exist in the DOM, set
  // their values from the loaded settings. Doing this before the innerHTML
  // assignment above was a no-op (getElementById returned null), which is
  // why cmuType always silently fell back to its first <option> (Tesla).
  const ct=document.getElementById('cmuType');
  if(ct) ct.value=String(s.cmuType||0);
  const ci=document.getElementById('canInhibitEnabled');
  if(ci) ci.value=String(s.canInhibitEnabled?1:0);
  const chgId=document.getElementById('chargerHeartbeatID');
  if(chgId) chgId.value='0x'+(s.chargerHeartbeatID||0x305).toString(16).toUpperCase();
}

async function applySettings(){
  const payload={};
  for(const f of FIELDS){
    if(f[0]==='__g') continue;
    const [key,lbl,type]=f;
    const el=document.getElementById('sf_'+key);
    if(!el) continue;
    payload[key]=type==='float'?parseFloat(el.value):parseInt(el.value);
  }
  // Collect per-module cell count overrides
  const maxMod=(currentSettings.maxModules||20);
  const mc=[0]; // index 0 unused, pad with 0
  for(let i=1;i<=maxMod;i++){
    const sel=document.getElementById('mcs_'+i);
    mc.push(sel?parseInt(sel.value):0);
  }
  payload.moduleCells=mc;
  // v6
  const ct2=document.getElementById('cmuType');
  if(ct2) payload.cmuType=parseInt(ct2.value);
  const ci2=document.getElementById('canInhibitEnabled');
  if(ci2) payload.canInhibitEnabled=parseInt(ci2.value);
  const chgId2=document.getElementById('chargerHeartbeatID');
  if(chgId2) payload.chargerHeartbeatID=parseInt(chgId2.value,16);
  try{
    const r=await fetch('/api/settings',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify(payload)
    });
    const j=await r.json();
    if(j.ok){
      document.getElementById('settStatus').textContent='Saved at '+new Date().toLocaleTimeString();
      toast('Settings saved!');
      const ctEl=document.getElementById('cmuType');
      const cmuChanged = ctEl && parseInt(ctEl.value) !== (currentSettings.cmuType||0);
      if (cmuChanged) {
        toast('CMU type changed — power-cycle the device to apply, then this page will refresh automatically');
        document.getElementById('settStatus').textContent =
          'CMU type saved. Reboot the device, then wait — this page will detect it and refresh.';
        document.getElementById('settStatus').style.color='var(--warn)';
        pollForReboot();
      } else {
        // Reload to confirm round-trip
        setTimeout(fetchSettings,400);
      }
    } else {
      document.getElementById('settStatus').textContent='Error: '+j.error;
      toast('Save failed',false);
    }
  }catch(e){document.getElementById('settStatus').textContent='Network error: '+e.message;}
}

// ---- Poll ----
fetchData();
setInterval(fetchData,2000);
setInterval(fetchCAN,1000);
</script>
</body>
</html>
)rawhtml";

// ---------------------------------------------------------------------------
// begin()
// ---------------------------------------------------------------------------
bool WiFiManager::begin()
{
    const char *ssid = (strlen(settings.wifiSSID) > 0) ? settings.wifiSSID : WIFI_SSID_DEFAULT;
    const char *pass = (strlen(settings.wifiPass) > 0) ? settings.wifiPass : WIFI_PASS_DEFAULT;

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, pass);
    delay(500);

    ipAddr = WiFi.softAPIP().toString();
    Logger::console("WiFi AP: SSID=%s  IP=%s", ssid, ipAddr.c_str());

    if (server) { delete server; server = nullptr; }
    server = new AsyncWebServer(80);

    // --- Root ---
    server->on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send_P(200, "text/html", INDEX_HTML);
    });

    // --- /api/data ---
    server->on("/api/data", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        JsonObject pack = doc["pack"].to<JsonObject>();

        float packV  = bms.getPackVoltage();
        float lowC   = bms.getLowCellVolt();
        float highC  = bms.getHighCellVolt();
        float avgT   = bms.getAvgTemperature();
        int   ns     = settings.numSeries  > 0 ? settings.numSeries  : BMS_NUM_SERIES;
        float lo     = settings.socLo > 0.0f   ? settings.socLo      : 18.0f;
        float hi     = settings.socHi > lo     ? settings.socHi      : 25.2f;
        float socPct = ((packV / (float)ns) - lo) * 100.0f / (hi - lo);
        if (socPct > 100.0f) socPct = 100.0f;
        if (socPct < 0.0f)   socPct = 0.0f;

        pack["voltage"]         = serialized(String(packV, 2));
        pack["soc"]             = (int)socPct;
        pack["cellLow"]         = serialized(String(lowC,  3));
        pack["cellHigh"]        = serialized(String(highC, 3));
        pack["delta"]           = serialized(String(highC - lowC, 3));
        pack["temp"]            = serialized(String(avgT,  1));
        pack["modules"]         = bms.getNumModules();
        pack["faulted"]         = bms.isFaultedState();
        pack["balancing"]       = bms.getAutoBalance() && !bms.getBalanceInhibit();
        int defaultCells = 6;
        if (settings.cmuType == CMU_BMW_I3 || settings.cmuType == CMU_BMW_I3_BUS || settings.cmuType == CMU_BMW_MINIE) {
            defaultCells = 12;
        } else if (settings.cmuType == CMU_BMW_PHEV) {
            defaultCells = 16;
        } else if (settings.cmuType == CMU_VW_BMS) {
            defaultCells = 12;
        } else if (settings.numCells > 0) {
            defaultCells = settings.numCells;
        }
        pack["numCells"]        = defaultCells;
        // v6: Balance inhibit status
        pack["chargerActive"]   = can.getChargerActive();
        pack["canInhibit"]      = (bool)settings.canInhibitEnabled;
        pack["canCurrentA"]     = can.getCanCurrentA();
        pack["cmuType"]         = (int)settings.cmuType;
        pack["uvSetpoint"]      = settings.UnderVSetpoint;
        pack["ovSetpoint"]      = settings.OverVSetpoint;
        pack["ignoreTempThresh"]= settings.IgnoreTempThresh;

        JsonArray mods = doc["modules"].to<JsonArray>();
        for (int i = 1; i <= MAX_MODULE_ADDR; i++) {
            BMSModule &m = bms.getModule(i);
            if (!m.isExisting()) continue;
            JsonObject mo = mods.add<JsonObject>();
            mo["addr"]      = i;
            mo["voltage"]   = serialized(String(m.getModuleVoltage(), 2));
            // Only flag faulted for real faults (mask CUV bit 0x02)
            uint8_t realFaults = m.getFaults() & ~0x02;
            mo["faulted"]   = (realFaults > 0);
            mo["faultCode"] = m.getFaults();
            mo["alertCode"] = m.getAlerts();
            int mCells = bms.getModuleCells(i);
            mo["numCells"]  = mCells;  // per-module override or global default
            mo["t1"]        = serialized(String(m.getTemperature(0), 1));
            mo["t2"]        = serialized(String(m.getTemperature(1), 1));
            JsonArray cells = mo["cells"].to<JsonArray>();
            for (int c = 0; c < mCells; c++)
                cells.add(serialized(String(m.getCellVoltage(c), 3)));
        }

        String json;
        serializeJson(doc, json);
        req->send(200, "application/json", json);
    });

    // --- /api/can - rolling CAN frame log ---
    server->on("/api/can", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
        // Return in chronological order
        int count = canLogCount < CAN_LOG_SIZE ? canLogCount : CAN_LOG_SIZE;
        int start = canLogCount < CAN_LOG_SIZE ? 0 : canLogHead;
        for (int i = 0; i < count; i++) {
            int idx = (start + i) % CAN_LOG_SIZE;
            CANLogEntry &e = canLog[idx];
            JsonObject f = arr.add<JsonObject>();
            f["id"]  = e.id;
            f["ts"]  = e.ts;
            JsonArray data = f["data"].to<JsonArray>();
            for (int b = 0; b < e.len; b++) data.add(e.data[b]);
            f["decoded"] = decodeCAN(e.id, e.data);
        }
        String json;
        serializeJson(doc, json);
        req->send(200, "application/json", json);
    });

    // --- /api/settings ---
    server->on("/api/settings", HTTP_GET, [](AsyncWebServerRequest *req) {
        JsonDocument doc;
        doc["numCells"]        = (int)settings.numCells;
        doc["numSeries"]       = (int)settings.numSeries;
        doc["numParallel"]     = (int)settings.numParallel;
        doc["socLo_V"]         = settings.socLo;
        doc["socHi_V"]         = settings.socHi;
        doc["ovSetpoint"]      = settings.OverVSetpoint;
        doc["uvSetpoint"]      = settings.UnderVSetpoint;
        doc["otSetpoint"]      = settings.OverTSetpoint;
        doc["utSetpoint"]      = settings.UnderTSetpoint;
        doc["ignoreVolt"]      = settings.IgnoreVolt;
        doc["ignoreTempC"]     = settings.IgnoreTempThresh;
        doc["balVoltage"]      = settings.balanceVoltage;
        doc["balHyst"]         = settings.balanceHyst;
        doc["autoBalance"]     = (bool)settings.balancingEnabled;
        doc["wifiEnabled"]     = (bool)settings.wifiEnabled;
        doc["batteryID"]       = (int)settings.batteryID;
        doc["logLevel"]        = (int)settings.logLevel;
        doc["maxModules"]      = MAX_MODULE_ADDR;
        // v6: CMU type and balance inhibit
        doc["cmuType"]            = (int)settings.cmuType;
        doc["canInhibitEnabled"]  = (bool)settings.canInhibitEnabled;
        doc["chargerHeartbeatID"] = (int)settings.chargerHeartbeatID;
        // Per-module cell count overrides: index 1..MAX_MODULE_ADDR, 0 = use global
        JsonArray mc = doc["moduleCells"].to<JsonArray>();
        for (int i = 0; i <= MAX_MODULE_ADDR; i++)
            mc.add((int)settings.moduleCells[i]);
        String json;
        serializeJson(doc, json);
        req->send(200, "application/json", json);
    });

    // --- POST /api/cantx - transmit a single CAN frame from web UI ---
    // Uses body handler since AsyncCallbackJsonWebHandler requires AsyncJson.h which
    // may not be present in all forks; raw body approach is universally compatible.
    server->on("/api/cantx", HTTP_POST,
        [](AsyncWebServerRequest *req) {},  // empty finalizer
        nullptr,
        [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
            JsonDocument jdoc;
            DeserializationError err = deserializeJson(jdoc, data, len);
            if (err) { req->send(400, "application/json", "{\"ok\":false,\"error\":\"json parse\"}"); return; }
            JsonObject j = jdoc.as<JsonObject>();
            uint32_t id  = j["id"] | 0;
            JsonArray arr = j["data"].as<JsonArray>();
            if (!arr || arr.size() < 1 || arr.size() > 8 || id > 0x7FF) {
                req->send(400, "application/json", "{\"ok\":false,\"error\":\"bad params\"}"); return;
            }
            uint8_t txdata[8] = {};
            uint8_t txlen = 0;
            for (JsonVariant b : arr) txdata[txlen++] = b.as<uint8_t>();
            twai_message_t msg = {};
            msg.identifier       = id;
            msg.extd             = 0;
            msg.data_length_code = txlen;
            memcpy(msg.data, txdata, txlen);
            esp_err_t res = twai_transmit(&msg, pdMS_TO_TICKS(10));
            if (res == ESP_OK) {
                wifiLogCAN(id, txdata, txlen);
                req->send(200, "application/json", "{\"ok\":true}");
            } else {
                char buf[64];
                snprintf(buf, sizeof(buf), "{\"ok\":false,\"error\":\"TWAI 0x%02X\"}", res);
                req->send(200, "application/json", buf);
            }
        }
    );

    // --- POST /api/settings - apply + save settings from web UI ---
    server->on("/api/settings", HTTP_POST,
        [](AsyncWebServerRequest *req) {},
        nullptr,
        [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
            JsonDocument jdoc;
            DeserializationError err = deserializeJson(jdoc, data, len);
            if (err) { req->send(400, "application/json", "{\"ok\":false,\"error\":\"json parse\"}"); return; }
            JsonObject j = jdoc.as<JsonObject>();
            bool changed = false;
            // Pack topology
            if (!j["numCells"].isNull())    { int v=j["numCells"];    if(v>=1&&v<=6)   { settings.numCells=(uint8_t)v;      changed=true; } }
            if (!j["numSeries"].isNull())   { int v=j["numSeries"];   if(v>=1&&v<=62)  { settings.numSeries=(uint8_t)v;     changed=true; } }
            if (!j["numParallel"].isNull()) { int v=j["numParallel"]; if(v>=1&&v<=10)  { settings.numParallel=(uint8_t)v;   changed=true; } }
            if (!j["socLo_V"].isNull())     { float v=j["socLo_V"];   if(v>=10&&v<30)  { settings.socLo=v;                  changed=true; } }
            if (!j["socHi_V"].isNull())     { float v=j["socHi_V"];   if(v>10&&v<=30)  { settings.socHi=v;                  changed=true; } }
            // Thresholds
            if (!j["ovSetpoint"].isNull())  { float v=j["ovSetpoint"];  if(v>=3&&v<=5)    { settings.OverVSetpoint=v;   changed=true; } }
            if (!j["uvSetpoint"].isNull())  { float v=j["uvSetpoint"];  if(v>=2&&v<=4)    { settings.UnderVSetpoint=v;  changed=true; } }
            if (!j["otSetpoint"].isNull())  { float v=j["otSetpoint"];  if(v>=20&&v<=100) { settings.OverTSetpoint=v;   changed=true; } }
            if (!j["utSetpoint"].isNull())  { float v=j["utSetpoint"];  if(v>=-40&&v<=30) { settings.UnderTSetpoint=v;  changed=true; } }
            if (!j["ignoreVolt"].isNull())  { float v=j["ignoreVolt"];  if(v>=0&&v<=3.5f) { settings.IgnoreVolt=v;      changed=true; } }
            if (!j["ignoreTempC"].isNull()) { float v=j["ignoreTempC"]; if(v>=-100&&v<=0) { settings.IgnoreTempThresh=v; changed=true; } }
            // Balance
            if (!j["balVoltage"].isNull())  { float v=j["balVoltage"]; if(v>=3&&v<=4.5f) { settings.balanceVoltage=v;  changed=true; } }
            if (!j["balHyst"].isNull())     { float v=j["balHyst"];    if(v>=0&&v<=0.5f) { settings.balanceHyst=v;     changed=true; } }
            if (!j["autoBalance"].isNull()) { settings.balancingEnabled=(j["autoBalance"].as<int>()!=0)?1:0; changed=true; }
            // Connectivity
            if (!j["wifiEnabled"].isNull()) { settings.wifiEnabled=(j["wifiEnabled"].as<int>()!=0)?1:0; changed=true; }
            if (!j["batteryID"].isNull())   { int v=j["batteryID"]; if(v>=1&&v<=14) { settings.batteryID=(uint8_t)v; changed=true; } }
            if (!j["logLevel"].isNull())    { int v=j["logLevel"];  if(v>=0&&v<=4)  { settings.logLevel=(uint8_t)v;  changed=true; } }
            // v7: CMU type and balance inhibit
            bool cmuChanged = false;
            if (!j["cmuType"].isNull()) {
                int v = j["cmuType"];
                if (v >= 0 && v <= 5) {
                    if (settings.cmuType != (uint8_t)v) {
                        settings.cmuType = (uint8_t)v;
                        changed = true;
                        cmuChanged = true;
                    }
                }
            }
            if (!j["canInhibitEnabled"].isNull()) { settings.canInhibitEnabled=(j["canInhibitEnabled"].as<int>()!=0)?1:0; changed=true; }
            if (!j["chargerHeartbeatID"].isNull()) { int v=j["chargerHeartbeatID"]; if(v>0&&v<=0x7FF) { settings.chargerHeartbeatID=(uint32_t)v; changed=true; } }
            // Per-module cell count overrides: array index 1..MAX_MODULE_ADDR
            if (!j["moduleCells"].isNull() && j["moduleCells"].is<JsonArray>()) {
                JsonArray mc = j["moduleCells"].as<JsonArray>();
                int idx = 0;
                for (JsonVariant v : mc) {
                    if (idx <= MAX_MODULE_ADDR) {
                        int cv = v.as<int>();
                        // 0 = inherit global, 4/5/6 = Tesla, 8 = PHEV SP44, 12 = i3/Mini-E/i3-Bus, 16 = PHEV SP06/SP41
                        if (cv == 0 || cv == 4 || cv == 5 || cv == 6 || cv == 8 || cv == 12 || cv == 16) {
                            settings.moduleCells[idx] = (uint8_t)cv;
                            changed = true;
                        }
                    }
                    idx++;
                }
            }

            if (changed) {
                EEPROM.put(EEPROM_PAGE, settings);
                EEPROM.commit();
                Logger::console("Settings updated from web UI");
                req->send(200, "application/json", "{\"ok\":true}");
                if (cmuChanged) {
                    xTaskCreate([](void *param) {
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        ESP.restart();
                    }, "restart_task", 1024, nullptr, 1, nullptr);
                }
            } else {
                req->send(200, "application/json", "{\"ok\":false,\"error\":\"no valid fields\"}");
            }
        }
    );

    server->onNotFound([](AsyncWebServerRequest *req) {
        req->send(404, "text/plain", "Not found");
    });

    server->begin();
    running = true;
    Logger::console("HTTP server started: http://%s", ipAddr.c_str());

    // GVRET/SavvyCAN TCP server on port 23
    gvretServer.begin();
    gvretServer.setNoDelay(true);
    gvretClientConnected = false;
    Logger::console("GVRET server started: %s:23 (SavvyCAN)", ipAddr.c_str());

    return true;
}

void WiFiManager::end()
{
    if (gvretClientConnected) { gvretClient.stop(); gvretClientConnected = false; }
    gvretServer.stop();
    if (server) { server->end(); delete server; server = nullptr; }
    WiFi.softAPdisconnect(true);
    running = false;
    Logger::info("WiFi stopped");
}

bool WiFiManager::isRunning() { return running; }
String WiFiManager::getIP()   { return ipAddr;  }

void WiFiManager::loop()
{
    if (!running) return;
    gvretLoop();
}

// ---------------------------------------------------------------------------
// gvretLoop — accept new client, read and discard any inbound bytes.
// SavvyCAN sends a version request on connect; we respond with GVRET header.
// ---------------------------------------------------------------------------
void WiFiManager::gvretLoop()
{
    // Accept new client if none connected
    if (!gvretClientConnected) {
        WiFiClient newClient = gvretServer.accept();
        if (newClient) {
            gvretClient = newClient;
            gvretClientConnected = true;
            Logger::info("GVRET: SavvyCAN connected from %s",
                         gvretClient.remoteIP().toString().c_str());
            // Send GVRET device description string that SavvyCAN expects
            // Format: 0xF1 0x01 <description string> 0x00
            const char *desc = "M5DialBMS";
            gvretClient.write((uint8_t)0xF1);
            gvretClient.write((uint8_t)0x01);
            gvretClient.print(desc);
            gvretClient.write((uint8_t)0x00);
        }
        return;
    }

    // Check existing client still alive
    if (!gvretClient.connected()) {
        gvretClient.stop();
        gvretClientConnected = false;
        Logger::info("GVRET: SavvyCAN disconnected");
        return;
    }

    // Drain any inbound bytes (SavvyCAN sends keepalive/config frames)
    while (gvretClient.available()) gvretClient.read();
}

// ---------------------------------------------------------------------------
// gvretPushFrame — called by wifiLogCAN() for every TX/RX frame.
// Encodes one CAN frame in GVRET binary format and sends to SavvyCAN.
//
// GVRET frame format (from Collin Kidder's GVRET spec):
//   Byte 0:    0xF1         (start of frame)
//   Byte 1:    0x00         (CAN frame command)
//   Bytes 2-5: timestamp    (little-endian uint32, microseconds)
//   Bytes 6-9: CAN ID       (little-endian uint32, bit 31 set = extended)
//   Byte 10:   DLC          (data length 0-8)
//   Byte 11:   bus number   (0 = CAN0)
//   Bytes 12+: data bytes   (DLC bytes)
// ---------------------------------------------------------------------------
void WiFiManager::gvretPushFrame(uint32_t id, uint8_t *data, uint8_t len,
                                  bool extended)
{
    if (!gvretClientConnected || !gvretClient.connected()) return;

    uint32_t ts  = (uint32_t)(micros());
    uint32_t eid = extended ? (id | 0x80000000U) : id;
    uint8_t  dlc = (len <= 8) ? len : 8;

    uint8_t buf[12 + 8];
    buf[0]  = 0xF1;
    buf[1]  = 0x00;
    buf[2]  = (uint8_t)(ts & 0xFF);
    buf[3]  = (uint8_t)((ts >>  8) & 0xFF);
    buf[4]  = (uint8_t)((ts >> 16) & 0xFF);
    buf[5]  = (uint8_t)((ts >> 24) & 0xFF);
    buf[6]  = (uint8_t)(eid & 0xFF);
    buf[7]  = (uint8_t)((eid >>  8) & 0xFF);
    buf[8]  = (uint8_t)((eid >> 16) & 0xFF);
    buf[9]  = (uint8_t)((eid >> 24) & 0xFF);
    buf[10] = dlc;
    buf[11] = 0x00;   // bus 0
    memcpy(buf + 12, data, dlc);

    gvretClient.write(buf, 12 + dlc);
}
