#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <math.h>
#include <stdlib.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif
#ifndef AP_SSID
#define AP_SSID "SprinklerController"
#endif
#ifndef AP_PASSWORD
#define AP_PASSWORD "ChangeMe123"
#endif
#ifndef OTA_PASSWORD
#define OTA_PASSWORD "change-this-unique-ota-password"
#endif

namespace Cfg {
constexpr uint8_t DIR_PIN=25, PWM_PIN=26, PRESSURE_PIN=34;
constexpr uint8_t DIR_OPEN_LEVEL=HIGH;
constexpr float SENSOR_MIN_V=0.5f, SENSOR_MAX_V=4.5f, SENSOR_FS_PSI=150.0f;
constexpr float DIVIDER_TOP=10000.0f, DIVIDER_BOTTOM=20000.0f;
constexpr float DIVIDER_RATIO=DIVIDER_BOTTOM/(DIVIDER_TOP+DIVIDER_BOTTOM);
constexpr float PRESSURE_GAIN=1.0f, PRESSURE_OFFSET_PSI=0.0f;
constexpr float SENSOR_FAULT_LOW_V=0.20f, SENSOR_FAULT_HIGH_V=4.80f;
constexpr uint8_t ADC_SAMPLES=16;
constexpr uint8_t ZERO_HISTORY_SAMPLES=20;
constexpr float ZERO_HISTORY_MAX_SPREAD_PSI=1.0f;
constexpr float MAX_ZERO_OFFSET_PSI=30.0f;
constexpr float FILTER_ALPHA=0.20f;
constexpr float DEFAULT_TARGET_PSI=40.0f, DEADBAND_PSI=0.75f;
constexpr float MAX_SETPOINT_PSI=100.0f, HARD_MAX_PRESSURE_PSI=120.0f;
constexpr float OVERPRESSURE_CLEAR_PSI=115.0f;
constexpr uint32_t CONTROL_INTERVAL_MS=100, SETTLE_MS=400;
constexpr uint32_t MIN_PULSE_MS=20, MAX_PULSE_MS=1000, VALVE_FULL_TRAVEL_MS=4500;
}

WebServer server(80);
Preferences prefs;
enum class Mode:uint8_t { MANUAL, AUTO };
enum class Motion:uint8_t { STOPPED, OPENING, CLOSING };
enum class Fault:uint8_t { NONE, SENSOR, OVERPRESSURE };
Mode mode=Mode::MANUAL;
Motion motion=Motion::STOPPED;
Fault fault=Fault::NONE;
float targetPsi=Cfg::DEFAULT_TARGET_PSI, adcMv=0, adcV=0, sensorV=0;
float rawPsi=0, filteredPsi=NAN, estimatedPosition=50.0f, pressureOffsetPsi=Cfg::PRESSURE_OFFSET_PSI;
float zeroHistory[Cfg::ZERO_HISTORY_SAMPLES]{};
uint8_t zeroHistoryCount=0, zeroHistoryNext=0;
uint32_t pulseEndMs=0, pulseStartMs=0, settleUntilMs=0, lastControlMs=0, lastReadMs=0, lastLogMs=0;
uint8_t goodSamples=0;
bool sensorValid=false, pressureReady=false, stationConnected=false, pulseIsOpen=false;
String lastAction="boot_safe", lastError="";

const char PAGE[] PROGMEM=R"HTML(
<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Sprinkler pressure</title><style>
:root{color-scheme:dark;--bg:#101820;--panel:#1c2a35;--ink:#edf4f7;--muted:#a9bbc6;--accent:#47c6a5;--warn:#ffbf55;--bad:#ff6868}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:16px system-ui,sans-serif}.wrap{max-width:900px;margin:auto;padding:18px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:12px}.card{background:var(--panel);border-radius:14px;padding:16px}.hero{font-size:clamp(2.3rem,8vw,4rem);font-weight:750;line-height:1.1}.label{color:var(--muted);font-size:.82rem;text-transform:uppercase;letter-spacing:.08em}.value{font-size:1.25rem;margin-top:6px}.fault{font-weight:800;color:var(--accent)}.fault.bad{color:var(--bad)}.fault.warn{color:var(--warn)}input,button{font:inherit;border:0;border-radius:9px;padding:11px}input{width:120px;background:#0e171e;color:var(--ink);border:1px solid #50616c}button{background:#304550;color:var(--ink);cursor:pointer;margin:4px 4px 4px 0}button.primary{background:#168d76}button.stop{background:#9d343c}.row{display:flex;align-items:center;flex-wrap:wrap;gap:6px;margin-top:10px}.small{color:var(--muted);font-size:.9rem}.notice{padding:12px;border-left:4px solid var(--warn);background:#2b2a22;border-radius:6px;margin:14px 0}details{margin-top:14px}pre{white-space:pre-wrap;color:var(--muted)}
</style></head><body><main class="wrap"><h1>Sprinkler pressure</h1><div id="notice" class="notice">AUTO is off until explicitly enabled. Keep it off until commissioning is complete.</div><section class="grid"><article class="card"><div class="label">Current pressure</div><div class="hero"><span id="pressure">--</span> <small>PSI</small></div></article><article class="card"><div class="label">Target pressure</div><div class="value"><span id="target">--</span> PSI</div><div class="row"><input id="setpoint" type="number" min="0" max="100" step="0.1" value="40" aria-label="Target PSI"><button class="primary" onclick="setTarget()">Set target</button></div></article><article class="card"><div class="label">System</div><div id="fault" class="value fault">--</div><div class="small">Mode: <span id="mode">--</span> · Valve: <span id="valve">--</span></div></article></section><section class="card" style="margin-top:12px"><div class="label">Controls</div><div class="row"><button class="primary" onclick="auto(true)">AUTO ON</button><button onclick="auto(false)">AUTO OFF</button><button class="stop" onclick="post('/api/stop')">STOP</button></div><div class="row"><button onclick="pulse('open',100)">OPEN 100 ms</button><button onclick="pulse('close',100)">CLOSE 100 ms</button><button onclick="pulse('open',300)">OPEN 300 ms</button><button onclick="pulse('close',300)">CLOSE 300 ms</button></div><div id="message" class="small" role="status"></div></section><details class="card"><summary>Diagnostics and zero calibration</summary><p class="small">To zero a gauge sensor, vent its pressure port to atmosphere, keep the valve stopped in MANUAL, and confirm the installed divider is correct. Zeroing only adjusts offset; it cannot fix wiring or verify pressure span.</p><button onclick="zeroSensor()">Zero sensor at atmosphere</button><pre id="diag">Loading…</pre></details><p class="small">Connection: <span id="wifi">--</span> · Uptime: <span id="uptime">--</span></p></main><script>
async function post(path,body=''){try{let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body}),j=await r.json();document.querySelector('#message').textContent=j.error||j.message||'OK';if(!r.ok)throw Error(j.error||'Request failed');await refresh()}catch(e){document.querySelector('#message').textContent=e.message}}
function setTarget(){post('/api/setpoint','psi='+encodeURIComponent(document.querySelector('#setpoint').value))}function auto(v){post('/api/auto','enabled='+(v?1:0))}function pulse(d,ms){post('/api/pulse','direction='+d+'&ms='+ms)}function zeroSensor(){if(confirm('Confirm the pressure port is open to atmosphere and the divider is wired correctly. This stores a zero offset only; it does not verify pressure span. Keep AUTO off until checked against a known gauge. Continue?'))post('/api/zero','confirmed=1')}
async function refresh(){try{let r=await fetch('/api/status',{cache:'no-store'}),s=await r.json();document.querySelector('#pressure').textContent=s.pressure===null?'--':Number(s.pressure).toFixed(1);document.querySelector('#target').textContent=Number(s.target).toFixed(1);document.querySelector('#setpoint').value=s.target;document.querySelector('#mode').textContent=s.mode;document.querySelector('#valve').textContent=s.valve;let f=document.querySelector('#fault');f.textContent=s.system.toUpperCase();f.className='value fault'+(s.fault==='sensor'?' bad':s.fault==='overpressure'?' warn':'');document.querySelector('#wifi').textContent=s.wifi+' · '+s.ip;document.querySelector('#uptime').textContent=Math.floor(s.uptimeMs/1000)+' s';document.querySelector('#diag').textContent=`ADC: ${s.adcMillivolts} mV (${s.adcVoltage.toFixed(3)} V)\nSensor: ${s.sensorVoltage.toFixed(3)} V\nRaw: ${s.rawPressure.toFixed(2)} PSI\nZero offset: ${s.pressureOffsetPsi.toFixed(2)} PSI\nFiltered: ${s.pressure===null?'invalid':s.pressure.toFixed(2)+' PSI'}\nLast action: ${s.lastAction}\nEstimated position: ${s.valvePosition.toFixed(1)}% (diagnostic only)\nWi-Fi RSSI: ${s.wifiRssi} dBm\nLast error: ${s.lastError||'none'}`;document.querySelector('#notice').textContent=s.warning}catch(e){document.querySelector('#message').textContent='Status unavailable'}}setInterval(refresh,500);refresh();
</script></body></html>
)HTML";

const char* modeName(){return mode==Mode::AUTO?"auto":"manual";}
const char* motionName(){return motion==Motion::OPENING?"opening":motion==Motion::CLOSING?"closing":"stopped";}
const char* faultName(){return fault==Fault::SENSOR?"sensor":fault==Fault::OVERPRESSURE?"overpressure":"none";}
bool reached(uint32_t now,uint32_t deadline){return static_cast<int32_t>(now-deadline)>=0;}
void clearZeroHistory(){zeroHistoryCount=0;zeroHistoryNext=0;}
void addZeroSample(float psi){zeroHistory[zeroHistoryNext]=psi;zeroHistoryNext=(zeroHistoryNext+1)%Cfg::ZERO_HISTORY_SAMPLES;if(zeroHistoryCount<Cfg::ZERO_HISTORY_SAMPLES)++zeroHistoryCount;}
bool zeroHistoryStats(float& center,float& spread){
  if(zeroHistoryCount<Cfg::ZERO_HISTORY_SAMPLES)return false;
  float sorted[Cfg::ZERO_HISTORY_SAMPLES];
  for(uint8_t i=0;i<zeroHistoryCount;i++){
    float value=zeroHistory[i];uint8_t j=i;
    while(j>0&&sorted[j-1]>value){sorted[j]=sorted[j-1];--j;}
    sorted[j]=value;
  }
  uint8_t lowIndex=zeroHistoryCount/10,highIndex=(zeroHistoryCount*9)/10;
  center=(sorted[(zeroHistoryCount-1)/2]+sorted[zeroHistoryCount/2])*0.5f;
  spread=sorted[highIndex]-sorted[lowIndex];
  return isfinite(center)&&isfinite(spread);
}
void logEvent(const char* what){Serial.printf("[%lu] %s PSI=%.2f target=%.2f ADC=%.3fV sensor=%.3fV mode=%s valve=%s fault=%s\n",(unsigned long)millis(),what,pressureReady?filteredPsi:NAN,targetPsi,adcV,sensorV,modeName(),motionName(),faultName());}

void stopValve(const char* why="stop"){
  digitalWrite(Cfg::PWM_PIN,LOW);
  if(motion!=Motion::STOPPED){uint32_t now=millis(),elapsed=now-pulseStartMs;float delta=100.0f*elapsed/Cfg::VALVE_FULL_TRAVEL_MS;estimatedPosition=constrain(estimatedPosition+(pulseIsOpen?delta:-delta),0.0f,100.0f);motion=Motion::STOPPED;settleUntilMs=now+Cfg::SETTLE_MS;lastAction=why;logEvent(why);}
}
bool startPulse(bool open,uint32_t ms,const char* origin){
  uint32_t now=millis();if(motion!=Motion::STOPPED||!reached(now,settleUntilMs))return false;
  ms=constrain(ms,Cfg::MIN_PULSE_MS,Cfg::MAX_PULSE_MS);
  digitalWrite(Cfg::PWM_PIN,LOW);digitalWrite(Cfg::DIR_PIN,open?Cfg::DIR_OPEN_LEVEL:!Cfg::DIR_OPEN_LEVEL);delayMicroseconds(20);digitalWrite(Cfg::PWM_PIN,HIGH);
  pulseStartMs=now;pulseEndMs=now+ms;pulseIsOpen=open;motion=open?Motion::OPENING:Motion::CLOSING;lastAction=String(origin)+(open?"_open_":"_close_")+String(ms)+"ms";
  Serial.printf("[%lu] PSI=%.2f target=%.2f err=%+.2f %s %lums ADC=%.3fV sensor=%.3fV mode=%s fault=%s\n",(unsigned long)now,pressureReady?filteredPsi:NAN,targetPsi,pressureReady?targetPsi-filteredPsi:NAN,open?"OPEN":"CLOSE",(unsigned long)ms,adcV,sensorV,modeName(),faultName());return true;
}
void servicePulse(){if(motion!=Motion::STOPPED&&reached(millis(),pulseEndMs))stopValve("pulse_complete");}

void updatePressure(){
  uint32_t now=millis();if(now-lastReadMs<50)return;lastReadMs=now;uint32_t total=0;
  for(uint8_t i=0;i<Cfg::ADC_SAMPLES;i++){total+=analogReadMilliVolts(Cfg::PRESSURE_PIN);delayMicroseconds(100);}
  adcMv=(float)total/Cfg::ADC_SAMPLES;adcV=adcMv/1000.0f;sensorV=adcV/Cfg::DIVIDER_RATIO;
  rawPsi=((sensorV-Cfg::SENSOR_MIN_V)/(Cfg::SENSOR_MAX_V-Cfg::SENSOR_MIN_V))*Cfg::SENSOR_FS_PSI;
  sensorValid=isfinite(sensorV)&&sensorV>=Cfg::SENSOR_FAULT_LOW_V&&sensorV<=Cfg::SENSOR_FAULT_HIGH_V;
  if(!sensorValid){clearZeroHistory();goodSamples=0;fault=Fault::SENSOR;mode=Mode::MANUAL;stopValve("sensor_fault_stop");lastError="Pressure sensor outside 0.20-4.80 V";return;}
  uint32_t nowMs=millis();if(mode==Mode::MANUAL&&motion==Motion::STOPPED&&reached(nowMs,settleUntilMs))addZeroSample(rawPsi);else clearZeroHistory();
  float calibratedPsi=rawPsi*Cfg::PRESSURE_GAIN+pressureOffsetPsi;
  if(goodSamples<5)++goodSamples;
  if(goodSamples>=5&&fault==Fault::SENSOR){fault=Fault::NONE;pressureReady=false;lastError="";logEvent("sensor_recovered_auto_remains_off");}
  if(!pressureReady){filteredPsi=calibratedPsi;pressureReady=true;}else filteredPsi=Cfg::FILTER_ALPHA*calibratedPsi+(1-Cfg::FILTER_ALPHA)*filteredPsi;
}
uint32_t pulseFor(float e){return e<=1.5f?70:e<=3.0f?110:e<=6.0f?200:325;}
void serviceControl(){
  if(!sensorValid||!pressureReady||motion!=Motion::STOPPED)return;uint32_t now=millis();
  if(!reached(now,settleUntilMs)||now-lastControlMs<Cfg::CONTROL_INTERVAL_MS)return;lastControlMs=now;
  // A measured overpressure is a close-only safety override, including in MANUAL.
  if(filteredPsi>Cfg::HARD_MAX_PRESSURE_PSI){if(fault!=Fault::OVERPRESSURE)logEvent("OVERPRESSURE_close_override");fault=Fault::OVERPRESSURE;startPulse(false,300,"overpressure");return;}
  if(fault==Fault::OVERPRESSURE){if(filteredPsi<Cfg::OVERPRESSURE_CLEAR_PSI){fault=Fault::NONE;logEvent("overpressure_clear");}else{startPulse(false,250,"overpressure");return;}}
  if(mode!=Mode::AUTO)return;float error=targetPsi-filteredPsi;if(fabsf(error)<=Cfg::DEADBAND_PSI)return;startPulse(error>0,pulseFor(fabsf(error)),"auto");
}

bool badOrigin(){if(!server.hasHeader("Origin"))return false;String o=server.header("Origin"),h="http://"+server.hostHeader();if(o==h)return false;return o!="https://"+server.hostHeader();}
void errorJson(int code,const char* msg){server.send(code,"application/json",String("{\"ok\":false,\"error\":\"")+msg+"\"}");}
bool numberArg(const char* key,float& v){if(!server.hasArg(key))return false;String s=server.arg(key);s.trim();if(!s.length()||s.length()>24)return false;char* end=nullptr;v=strtof(s.c_str(),&end);return end!=s.c_str()&&*end=='\0'&&isfinite(v);}
void status(){char out[1200];int rssi=WiFi.status()==WL_CONNECTED?WiFi.RSSI():0;String ip=stationConnected?WiFi.localIP().toString():WiFi.softAPIP().toString();const char* sys=fault==Fault::NONE?(sensorValid?"ok":"sensor_fault"):faultName();const char* p=pressureReady?"%.2f":"null";char pressure[24];if(pressureReady)snprintf(pressure,sizeof(pressure),p,filteredPsi);else strcpy(pressure,"null");
  snprintf(out,sizeof(out),"{\"pressure\":%s,\"rawPressure\":%.3f,\"pressureOffsetPsi\":%.3f,\"target\":%.2f,\"adcMillivolts\":%.1f,\"adcVoltage\":%.4f,\"sensorVoltage\":%.4f,\"mode\":\"%s\",\"valve\":\"%s\",\"fault\":\"%s\",\"system\":\"%s\",\"wifiRssi\":%d,\"wifi\":\"%s\",\"ip\":\"%s\",\"uptimeMs\":%lu,\"lastAction\":\"%s\",\"lastError\":\"%s\",\"valvePosition\":%.1f,\"warning\":\"Throttling is not a static pressure regulator; install rated mechanical relief hardware.\"}",pressure,rawPsi,pressureOffsetPsi,targetPsi,adcMv,adcV,sensorV,modeName(),motionName(),faultName(),sys,rssi,stationConnected?"station+ap":"access_point",ip.c_str(),(unsigned long)millis(),lastAction.c_str(),lastError.c_str(),estimatedPosition);
  server.send(200,"application/json",out);
}
void setpoint(){if(badOrigin())return errorJson(403,"Cross-origin request rejected");float v;if(!numberArg("psi",v)||v<0||v>Cfg::MAX_SETPOINT_PSI)return errorJson(400,"psi must be a finite number from 0 to 100");targetPsi=v;prefs.putFloat("target",v);lastError="";server.send(200,"application/json","{\"ok\":true,\"message\":\"Setpoint saved\"}");}
void automatic(){if(badOrigin())return errorJson(403,"Cross-origin request rejected");if(!server.hasArg("enabled")||(server.arg("enabled")!="0"&&server.arg("enabled")!="1"))return errorJson(400,"enabled must be 0 or 1");bool on=server.arg("enabled")=="1";if(on&&(!sensorValid||fault==Fault::SENSOR||!pressureReady))return errorJson(409,"AUTO blocked until sensor is valid");if(on&&fault==Fault::OVERPRESSURE)return errorJson(409,"AUTO blocked during overpressure");if(on&&motion!=Motion::STOPPED)return errorJson(409,"Wait for current pulse to finish");mode=on?Mode::AUTO:Mode::MANUAL;lastAction=on?"auto_enabled_by_user":"auto_disabled_by_user";logEvent(on?"AUTO enabled":"AUTO disabled");server.send(200,"application/json",on?"{\"ok\":true,\"message\":\"AUTO enabled\"}":"{\"ok\":true,\"message\":\"AUTO off\"}");}
void manualPulse(){if(badOrigin())return errorJson(403,"Cross-origin request rejected");if(!sensorValid||fault==Fault::SENSOR)return errorJson(409,"Manual pulse blocked while sensor is faulted");String d=server.arg("direction");if(d!="open"&&d!="close")return errorJson(400,"direction must be open or close");float ms;if(!numberArg("ms",ms)||ms<Cfg::MIN_PULSE_MS||ms>Cfg::MAX_PULSE_MS||floorf(ms)!=ms)return errorJson(400,"ms must be an integer from 20 to 1000");bool open=d=="open";if(open&&(fault==Fault::OVERPRESSURE||filteredPsi>Cfg::HARD_MAX_PRESSURE_PSI))return errorJson(409,"OPEN blocked during overpressure");mode=Mode::MANUAL;if(!startPulse(open,(uint32_t)ms,"manual"))return errorJson(409,"Valve is moving or settling; wait before another pulse");server.send(200,"application/json","{\"ok\":true,\"message\":\"Pulse started\"}");}
void stopRequest(){if(badOrigin())return errorJson(403,"Cross-origin request rejected");mode=Mode::MANUAL;stopValve("manual_stop_auto_off");lastAction="manual_stop_auto_off";server.send(200,"application/json","{\"ok\":true,\"message\":\"Stopped; AUTO is off\"}");}
void zeroPressure(){
  if(badOrigin())return errorJson(403,"Cross-origin request rejected");
  if(!server.hasArg("confirmed")||server.arg("confirmed")!="1")return errorJson(400,"Explicit confirmation required");
  if(mode!=Mode::MANUAL)return errorJson(409,"Turn AUTO off before zero calibration");
  if(motion!=Motion::STOPPED||!reached(millis(),settleUntilMs))return errorJson(409,"Wait until the valve is stopped and settled");
  if(!sensorValid||fault!=Fault::NONE||!pressureReady)return errorJson(409,"Sensor must be valid with no active fault");
  float medianRaw=0,spread=0;if(!zeroHistoryStats(medianRaw,spread)||spread>Cfg::ZERO_HISTORY_MAX_SPREAD_PSI)return errorJson(409,"Need 20 stable manual readings before zeroing");
  float newOffset=-medianRaw*Cfg::PRESSURE_GAIN;if(!isfinite(newOffset)||fabsf(newOffset)>Cfg::MAX_ZERO_OFFSET_PSI)return errorJson(409,"Zero correction exceeds the safe 30 PSI limit; inspect sensor and divider");
  if(prefs.putFloat("zero_offset",newOffset)!=sizeof(float))return errorJson(500,"Could not save zero calibration");
  pressureOffsetPsi=newOffset;pressureReady=false;filteredPsi=NAN;clearZeroHistory();lastError="";lastAction="zero_calibrated";logEvent("zero_calibrated");
  char out[160];snprintf(out,sizeof(out),"{\"ok\":true,\"message\":\"Zero saved at %.2f PSI raw; offset %.2f PSI\",\"rawAtZero\":%.3f,\"offsetPsi\":%.3f}",medianRaw,newOffset,medianRaw,newOffset);server.send(200,"application/json",out);
}

void setupRoutes(){const char* h[]={"Origin"};server.collectHeaders(h,1);server.on("/",HTTP_GET,[]{server.send_P(200,"text/html",PAGE);});server.on("/api/status",HTTP_GET,status);server.on("/api/setpoint",HTTP_POST,setpoint);server.on("/api/auto",HTTP_POST,automatic);server.on("/api/pulse",HTTP_POST,manualPulse);server.on("/api/stop",HTTP_POST,stopRequest);server.on("/api/zero",HTTP_POST,zeroPressure);server.onNotFound([]{server.send(404,"application/json","{\"ok\":false,\"error\":\"Not found\"}");});}
void network(){WiFi.mode(WIFI_AP_STA);WiFi.setSleep(false);bool ap=WiFi.softAP(AP_SSID,AP_PASSWORD);Serial.printf("Fallback AP %s (%s), IP %s\n",AP_SSID,ap?"started":"FAILED",WiFi.softAPIP().toString().c_str());if(strlen(WIFI_SSID)){WiFi.begin(WIFI_SSID,WIFI_PASSWORD);uint32_t start=millis();while(WiFi.status()!=WL_CONNECTED&&millis()-start<9000){server.handleClient();delay(25);}}stationConnected=WiFi.status()==WL_CONNECTED;if(stationConnected)Serial.printf("Station IP %s RSSI %d dBm\n",WiFi.localIP().toString().c_str(),WiFi.RSSI());else Serial.println("Station unavailable; AP remains active.");}

void setupOta(){
  ArduinoOTA.setHostname("sprinkler-pressure");ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([](){mode=Mode::MANUAL;stopValve("ota_update_stop");Serial.println("Authenticated OTA update started; AUTO off and valve stopped.");});
  ArduinoOTA.onEnd([](){Serial.println("OTA update complete; rebooting.");});
  ArduinoOTA.onError([](ota_error_t error){Serial.printf("OTA error %u\n",(unsigned)error);});
  ArduinoOTA.begin();Serial.println("Authenticated OTA ready (sprinkler-pressure.local, port 3232).");
}

void setup(){
  // PWM LOW before Wi-Fi or any lengthy initialization; carrier also has a 100k pull-down.
  pinMode(Cfg::PWM_PIN,OUTPUT);digitalWrite(Cfg::PWM_PIN,LOW);pinMode(Cfg::DIR_PIN,OUTPUT);digitalWrite(Cfg::DIR_PIN,!Cfg::DIR_OPEN_LEVEL);pinMode(Cfg::PRESSURE_PIN,INPUT);
  Serial.begin(115200);delay(10);Serial.println("\nBoot SAFE; AUTO remains off.");analogReadResolution(12);analogSetPinAttenuation(Cfg::PRESSURE_PIN,ADC_11db);
  prefs.begin("pressure",false);targetPsi=prefs.getFloat("target",Cfg::DEFAULT_TARGET_PSI);if(!isfinite(targetPsi)||targetPsi<0||targetPsi>Cfg::MAX_SETPOINT_PSI)targetPsi=Cfg::DEFAULT_TARGET_PSI;
  pressureOffsetPsi=prefs.getFloat("zero_offset",Cfg::PRESSURE_OFFSET_PSI);if(!isfinite(pressureOffsetPsi)||fabsf(pressureOffsetPsi)>Cfg::MAX_ZERO_OFFSET_PSI)pressureOffsetPsi=Cfg::PRESSURE_OFFSET_PSI;
  network();setupRoutes();server.begin();setupOta();Serial.printf("Web AP http://%s/\n",WiFi.softAPIP().toString().c_str());if(stationConnected)Serial.printf("Web STA http://%s/\n",WiFi.localIP().toString().c_str());logEvent("boot_safe_auto_off");
}
void loop(){server.handleClient();ArduinoOTA.handle();updatePressure();if(!sensorValid&&motion!=Motion::STOPPED)stopValve("sensor_fault_stop");servicePulse();serviceControl();uint32_t now=millis();if(now-lastLogMs>=5000){lastLogMs=now;logEvent("periodic_status");}bool connected=WiFi.status()==WL_CONNECTED;if(stationConnected&&!connected)Serial.printf("[%lu] Station disconnected; fallback AP remains active.\n",(unsigned long)now);if(!stationConnected&&connected)Serial.printf("[%lu] Station reconnected, IP %s.\n",(unsigned long)now,WiFi.localIP().toString().c_str());stationConnected=connected;}
