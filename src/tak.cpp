#include "tak.h"
#include "logging.h"
#include "types.h"
#include "cot_format.h"
#include <terseCRSF.h>

#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>

#if defined(ESP32)
  #include <WiFi.h>
  #include <WiFiUdp.h>
  #include <WebServer.h>
  #include <DNSServer.h>
#else
  #include <ESP8266WiFi.h>
  #include <WiFiUdp.h>
  #include <ESP8266WebServer.h>
  #include <DNSServer.h>
#endif

// ======================================================
// Telemetry sources (defined in main.cpp / application.cpp)
// ======================================================
extern CRSF crsf;          // last decoded telemetry (battery, link, gps speed/heading)
extern Location cur;       // current position: lat / lon / alt / hdg
extern bool motArmed;

// ======================================================
// TAK / CoT configuration
// ======================================================
static const char*    COT_UID         = "DRONE-01";    // stable UID -> TAK auto-draws the track
static const char*    COT_CALLSIGN    = "Drone-01";
static const char*    COT_TYPE        = "a-f-A-M-F-Q";  // friendly UAS (air track)
static const uint16_t COT_PORT        = 4242;           // UDP port the TAK app listens on
static const uint32_t COT_INTERVAL_MS = 1000;           // ~1 Hz position updates
static const uint32_t COT_STALE_SEC   = 10;             // marker greys out after this w/o updates

// Editable roster of drones that share this binding phrase. The phone picks one
// from the captive-portal dropdown; the choice becomes the CoT uid + callsign.
// (craft_name isn't reachable over the backpack, so identity is chosen manually.)
static const char* DRONE_NAMES[] = { "Drone-1", "Drone-2", "Drone-3" };
static const uint8_t DRONE_COUNT = sizeof(DRONE_NAMES) / sizeof(DRONE_NAMES[0]);

// SoftAP subnet broadcast (192.168.4.x is the ESP SoftAP default).
// Broadcast reaches the single phone client without needing its lease IP.
static const IPAddress COT_DEST(192, 168, 4, 255);

// ======================================================
// State
// ======================================================
static WiFiUDP udp;
#if defined(ESP32)
static WebServer server(80);
#else
static ESP8266WebServer server(80);
#endif
static DNSServer dns;

static bool     timeIsSet = false;
static uint32_t lastSend  = 0;
static uint32_t lastHint  = 0;

// Which drone this backpack is currently reporting (chosen on the portal page).
static char selectedDrone[24] = {0};

static void applyClockMs(uint64_t ms);   // defined below; used by handleSetDrone

static void sanitizeInto(const String& in, char* out, size_t cap)
{
    // Keep only XML/uid-safe characters; everything else becomes '_'.
    size_t w = 0;
    for (size_t i = 0; i < in.length() && w < cap - 1; ++i)
    {
        char ch = in[i];
        bool ok = (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
                  (ch >= '0' && ch <= '9') || ch == '-' || ch == '_';
        out[w++] = ok ? ch : '_';
    }
    out[w] = '\0';
}

// ======================================================
// Captive portal: sync the phone's UTC clock + pick which drone this is
// ======================================================
static void handleRoot()
{
    String html = F(
        "<!DOCTYPE html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>Drone TAK</title></head>"
        "<body style='font-family:sans-serif;text-align:center;padding:2em'>"
        "<h2>Drone TAK Bridge</h2><p id=s>Syncing clock&hellip;</p>"
        "<p>Drone: <select id=d>");
    for (uint8_t i = 0; i < DRONE_COUNT; ++i)
    {
        html += "<option";
        if (selectedDrone[0] && strcmp(selectedDrone, DRONE_NAMES[i]) == 0) html += " selected";
        html += '>'; html += DRONE_NAMES[i]; html += "</option>";
    }
    html += F("</select> <button onclick=setd()>Set</button></p><p id=c>Current: ");
    html += (selectedDrone[0] ? selectedDrone : "(none)");
    html += F("</p><script>"
        "fetch('/settime?ms='+Date.now()).then(r=>r.text())"
        ".then(t=>{s.innerText='Clock set: '+t;}).catch(e=>{s.innerText='Sync failed';});"
        "function setd(){fetch('/setdrone?name='+encodeURIComponent(d.value)+'&ms='+Date.now())"
        ".then(r=>r.text()).then(t=>{c.innerText='Current: '+t+' (clock re-synced)';});}"
        "</script></body></html>");
    server.send(200, "text/html", html);
}

static void handleSetDrone()
{
    if (!server.hasArg("name"))
    {
        server.send(400, "text/plain", "missing name");
        return;
    }
    sanitizeInto(server.arg("name"), selectedDrone, sizeof(selectedDrone));

    // Re-sync the clock on every drone selection so time stays fresh on switch.
    if (server.hasArg("ms"))
        applyClockMs(strtoull(server.arg("ms").c_str(), nullptr, 10));

    LOG_INFO("TAK: drone selected -> '%s' (clock %ssynced)",
             selectedDrone, server.hasArg("ms") ? "re-" : "not ");
    server.send(200, "text/plain", selectedDrone);
}

// Apply a UTC-milliseconds-since-epoch value (from the phone's Date.now(),
// which is timezone-independent) to the system clock.
static void applyClockMs(uint64_t ms)
{
    struct timeval tv;
    tv.tv_sec  = (time_t)(ms / 1000ULL);
    tv.tv_usec = (suseconds_t)((ms % 1000ULL) * 1000ULL);
    settimeofday(&tv, nullptr);
    timeIsSet = true;
}

static void handleSetTime()
{
    if (!server.hasArg("ms"))
    {
        server.send(400, "text/plain", "missing ms");
        return;
    }

    applyClockMs(strtoull(server.arg("ms").c_str(), nullptr, 10));

    char buf[32];
    time_t now = time(nullptr);
    struct tm tmv;
    gmtime_r(&now, &tmv);
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tmv);

    LOG_INFO("TAK: clock synced from phone: %s", buf);
    server.send(200, "text/plain", buf);
}

// ======================================================
// CoT event generation
// ======================================================
static void sendCoT()
{
    // Per-drone identity is chosen on the portal (drones share a binding phrase).
    // Until one is picked, fall back to the defaults.
    const char* callsign = selectedDrone[0] ? selectedDrone : COT_CALLSIGN;
    char uid[40];
    if (selectedDrone[0]) snprintf(uid, sizeof(uid), "DRONE-%s", selectedDrone);
    else                  snprintf(uid, sizeof(uid), "%s", COT_UID);

    char xml[768];
    int n = cot_format_event(
        xml, sizeof(xml),
        time(nullptr), COT_STALE_SEC,
        uid, COT_TYPE, callsign,
        cur.lat, cur.lon, cur.alt,
        crsf.gpsF_heading,              // ground course, deg
        crsf.gpsF_groundspeed / 3.6f,   // CRSF groundspeed is km/h -> m/s
        crsf.batF_voltage, crsf.batF_current, (unsigned long)crsf.bat_fuel_drawn,
        (int)crsf.link_up_rssi_ant_1, (int)crsf.link_up_quality,
        crsf.flightMode.c_str());

    if (n <= 0 || n >= (int)sizeof(xml))
        return;

    udp.beginPacket(COT_DEST, COT_PORT);
    udp.write((const uint8_t*)xml, n);
    udp.endPacket();
}

// ======================================================
// Public API
// ======================================================
void takInit(const char* ssid, const char* password)
{
    // SoftAP on channel 1 == the ESP-NOW channel, so telemetry RX and
    // IP traffic share the single radio without a channel conflict.
    WiFi.softAP(ssid, password, 1);
    IPAddress ip = WiFi.softAPIP();
    LOG_INFO("TAK SoftAP '%s' up at %s", ssid, ip.toString().c_str());
    LOG_INFO("TAK: join the AP, a browser page will pop to sync the clock");

    // Captive portal: send every DNS lookup to us so iOS auto-opens the page.
    dns.start(53, "*", ip);

    server.on("/", handleRoot);
    server.on("/settime", handleSetTime);
    server.on("/setdrone", handleSetDrone);
    // iOS/Android connectivity-check URLs land here -> serve our page so the
    // captive sign-in sheet appears automatically.
    server.onNotFound(handleRoot);
    server.begin();

    udp.begin(COT_PORT);
}

void takLoop()
{
    dns.processNextRequest();
    server.handleClient();

    uint32_t nowms = millis();

    if (!timeIsSet)
    {
        if (nowms - lastHint > 5000)
        {
            lastHint = nowms;
            LOG_INFO("TAK: waiting for clock sync - open http://192.168.4.1 on the phone");
        }
        return;
    }

    if (nowms - lastSend >= COT_INTERVAL_MS)
    {
        lastSend = nowms;

        // No GPS fix yet -> nothing meaningful to plot (avoids a marker at 0,0).
        if (cur.lat == 0.0f && cur.lon == 0.0f)
            return;

        sendCoT();
    }
}
