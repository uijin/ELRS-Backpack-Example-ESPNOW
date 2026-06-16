#include "tak.h"
#include "logging.h"
#include "types.h"
#include "cot_format.h"
#include <terseCRSF.h>

#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(ESP32)
  #include <WiFi.h>
  #include <WebServer.h>
  #include "esp_tls.h"            // TLS CoT server for iOS TAK (OmniTAK/iTAK/TAK Aware)
  #include <lwip/sockets.h>
  #include <fcntl.h>
  #include "tak_cert.h"           // baked-in self-signed server cert + key
#else
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
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
static const char*    COT_TYPE        = "a-f-A-M-H-Q";  // friendly rotary-wing UAS (helicopter icon)
static const uint16_t COT_TLS_PORT    = 8089;           // TLS server (iOS TAK: OmniTAK / iTAK / TAK Aware)
static const uint32_t COT_INTERVAL_MS = 1000;           // ~1 Hz position updates
static const uint32_t COT_STALE_SEC   = 10;             // marker greys out after this w/o updates

// TEST: when true, ignore real GPS and (once a drone is selected) simulate the
// drone orbiting a fixed circle so you get a moving track in TAK indoors.
// Runtime-toggleable from the captive portal (Simulate button); defaults OFF
// for field use (real GPS).
static bool         takTestSimulate     = false;
static const double SIM_CENTER_LAT      = 22.66862578822146;   // circle centre
static const double SIM_CENTER_LON      = 120.30209741628654;
static const double SIM_RADIUS_M        = 100.0;               // metres
static const double SIM_PERIOD_S        = 60.0;                // seconds per lap (~10.5 m/s)
static const float  SIM_ALT_M           = 50.0f;               // reported hae

#define MAX_TLS_CLIENTS 4   // OmniTAK / iTAK / TAK Aware can stream at once (+1 headroom)

// Editable roster of drones that share this binding phrase. The phone picks one
// from the captive-portal dropdown; the choice becomes the CoT uid + callsign.
// (craft_name isn't reachable over the backpack, so identity is chosen manually.)
static const char* DRONE_NAMES[] = { "AT35", "M75", "M75P1" };
static const uint8_t DRONE_COUNT = sizeof(DRONE_NAMES) / sizeof(DRONE_NAMES[0]);

// ======================================================
// State
// ======================================================
// TLS is the only CoT transport (iOS TAK apps require an SSL TAK server).
#if defined(ESP32)
static WebServer server(80);
#else
static ESP8266WebServer server(80);
#endif

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
    html += F("</p><p>Simulate: <button id=sim onclick=tsim()>");
    html += (takTestSimulate ? "ON" : "OFF");
    html += F("</button></p><script>"
        "fetch('/settime?ms='+Date.now()).then(r=>r.text())"
        ".then(t=>{s.innerText='Clock set: '+t;}).catch(e=>{s.innerText='Sync failed';});"
        "function setd(){fetch('/setdrone?name='+encodeURIComponent(d.value)+'&ms='+Date.now())"
        ".then(r=>r.text()).then(t=>{c.innerText='Current: '+t+' (clock re-synced)';});}"
        "function tsim(){var on=sim.innerText=='ON'?0:1;"
        "fetch('/setsim?on='+on).then(r=>r.text()).then(t=>{sim.innerText=t;});}"
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

// Toggle the simulated-orbit test mode at runtime. ?on=1 -> simulate, ?on=0 ->
// real GPS. Replies with the new state ("ON"/"OFF") so the page can update.
static void handleSetSim()
{
    if (server.hasArg("on"))
        takTestSimulate = (server.arg("on").toInt() != 0);
    else
        takTestSimulate = !takTestSimulate;   // no arg -> plain toggle

    LOG_INFO("TAK: simulate mode -> %s", takTestSimulate ? "ON (orbit)" : "OFF (real GPS)");
    server.send(200, "text/plain", takTestSimulate ? "ON" : "OFF");
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
// TLS CoT server (iOS TAK clients connect here as an SSL "server")
// One client at a time; raw CoT XML over TLS (TAK protocol version 0).
// ======================================================
#if defined(ESP32)
// Multiple iOS TAK apps stream at once; each gets its own slot. App identity is
// not tracked (all share one client cert) — the count is what we report.
static int        tlsListenFd                = -1;
static esp_tls_t* tlsConn[MAX_TLS_CLIENTS]   = { nullptr };
static int        tlsConnFd[MAX_TLS_CLIENTS];

static int tlsActiveCount()
{
    int n = 0;
    for (int i = 0; i < MAX_TLS_CLIENTS; ++i) if (tlsConn[i]) ++n;
    return n;
}

static void tlsServerBegin(uint16_t port)
{
    for (int i = 0; i < MAX_TLS_CLIENTS; ++i) { tlsConn[i] = nullptr; tlsConnFd[i] = -1; }

    tlsListenFd = socket(AF_INET, SOCK_STREAM, 0);
    if (tlsListenFd < 0) { LOG_ERROR("TAK TLS: socket() failed"); return; }
    int opt = 1;
    setsockopt(tlsListenFd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in a;
    memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port = htons(port);
    if (bind(tlsListenFd, (struct sockaddr*)&a, sizeof(a)) < 0)
    {
        LOG_ERROR("TAK TLS: bind :%u failed", port);
        close(tlsListenFd); tlsListenFd = -1; return;
    }
    listen(tlsListenFd, MAX_TLS_CLIENTS);
    fcntl(tlsListenFd, F_SETFL, O_NONBLOCK);
    LOG_INFO("TAK: TLS CoT server on :%u (up to %d clients)", port, MAX_TLS_CLIENTS);
}

static void tlsDrop(int i)
{
    if (tlsConn[i]) { esp_tls_server_session_delete(tlsConn[i]); tlsConn[i] = nullptr; }
    if (tlsConnFd[i] >= 0) { close(tlsConnFd[i]); tlsConnFd[i] = -1; }
}

static void tlsServerService()
{
    if (tlsListenFd < 0) return;

    // Accept at most one new client per pass, into the first free slot.
    int slot = -1;
    for (int i = 0; i < MAX_TLS_CLIENTS; ++i) if (!tlsConn[i]) { slot = i; break; }
    if (slot >= 0)
    {
        int cfd = accept(tlsListenFd, nullptr, nullptr);
        if (cfd >= 0)
        {
            esp_tls_cfg_server_t cfg;
            memset(&cfg, 0, sizeof(cfg));
            cfg.servercert_buf   = (const unsigned char*)TAK_SERVER_CERT;
            cfg.servercert_bytes = strlen(TAK_SERVER_CERT) + 1;
            cfg.serverkey_buf    = (const unsigned char*)TAK_SERVER_KEY;
            cfg.serverkey_bytes  = strlen(TAK_SERVER_KEY) + 1;   // no cacert -> client cert not verified

            esp_tls_t* tls = esp_tls_init();
            // Blocking handshake (~1-3s for RSA2048, one-time per connect).
            int r = esp_tls_server_session_create(&cfg, cfd, tls);
            if (r != 0)
            {
                LOG_ERROR("TAK: TLS handshake failed (%d)", r);
                esp_tls_server_session_delete(tls);
                close(cfd);
            }
            else
            {
                fcntl(cfd, F_SETFL, O_NONBLOCK);       // non-blocking for steady-state
                tlsConn[slot] = tls; tlsConnFd[slot] = cfd;
                LOG_INFO("TAK: TLS client connected (slot %d, %d active)", slot, tlsActiveCount());
            }
        }
    }

    // Drain inbound chatter on every active client; detect close.
    char buf[128];
    for (int i = 0; i < MAX_TLS_CLIENTS; ++i)
    {
        if (!tlsConn[i]) continue;
        int r = esp_tls_conn_read(tlsConn[i], buf, sizeof(buf));
        if (r == 0 ||
            (r < 0 && r != ESP_TLS_ERR_SSL_WANT_READ && r != ESP_TLS_ERR_SSL_WANT_WRITE))
        {
            tlsDrop(i);
            LOG_INFO("TAK: TLS client disconnected (slot %d, %d active)", i, tlsActiveCount());
        }
    }
}

// iOS TAK clients (iTAK/OmniTAK/TAK Aware) probe the TAK API (8443) and cert
// enrollment (8446) ports for reachability before they commit to the 8089 CoT
// stream. We don't implement those services, but simply accepting and closing
// the probe satisfies the client so it proceeds to the streaming connection.
static int apiFd8443 = -1, apiFd8446 = -1;
static int openApiPort(uint16_t port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int opt = 1; setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in a; memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET; a.sin_addr.s_addr = INADDR_ANY; a.sin_port = htons(port);
    if (bind(fd, (struct sockaddr*)&a, sizeof(a)) < 0) { close(fd); return -1; }
    listen(fd, 1); fcntl(fd, F_SETFL, O_NONBLOCK);
    return fd;
}
static void serviceApiPort(int fd)
{
    if (fd < 0) return;
    int c = accept(fd, nullptr, nullptr);
    if (c >= 0) close(c);   // accept-and-close satisfies the reachability probe
}

static void tlsServerWrite(const char* data, int len)
{
    for (int i = 0; i < MAX_TLS_CLIENTS; ++i)
    {
        if (!tlsConn[i]) continue;
        int off = 0;
        while (off < len)
        {
            int w = esp_tls_conn_write(tlsConn[i], data + off, len - off);
            if (w > 0) { off += w; continue; }
            if (w == ESP_TLS_ERR_SSL_WANT_WRITE || w == ESP_TLS_ERR_SSL_WANT_READ) continue;
            LOG_INFO("TAK: TLS write failed (%d), dropping slot %d", w, i);
            tlsDrop(i);
            break;
        }
    }
}
#endif  // ESP32

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

    // Position/track: real telemetry, or a simulated orbit in test mode.
    double lat    = cur.lat, lon = cur.lon;
    float  alt    = cur.alt;
    float  course = crsf.gpsF_heading;            // deg
    float  speed  = crsf.gpsF_groundspeed / 3.6f; // km/h -> m/s

    if (takTestSimulate)
    {
        const double R = 6378137.0;               // earth radius, m
        double theta = 2.0 * M_PI * (millis() / 1000.0) / SIM_PERIOD_S;
        double north = SIM_RADIUS_M * cos(theta); // metre offsets from centre
        double east  = SIM_RADIUS_M * sin(theta);
        lat = SIM_CENTER_LAT + (north / R) * (180.0 / M_PI);
        lon = SIM_CENTER_LON + (east / (R * cos(SIM_CENTER_LAT * M_PI / 180.0))) * (180.0 / M_PI);
        alt = SIM_ALT_M;
        // course = tangent heading (bearing CW from north); speed = orbit speed
        course = atan2(cos(theta), -sin(theta)) * 180.0 / M_PI;
        if (course < 0) course += 360.0f;
        speed = (float)(2.0 * M_PI * SIM_RADIUS_M / SIM_PERIOD_S);
    }

    char xml[768];
    int n = cot_format_event(
        xml, sizeof(xml),
        time(nullptr), COT_STALE_SEC,
        uid, COT_TYPE, callsign,
        lat, lon, alt, course, speed,
        crsf.batF_voltage, crsf.batF_current, (unsigned long)crsf.bat_fuel_drawn,
        (int)crsf.link_up_rssi_ant_1, (int)crsf.link_up_quality,
        crsf.flightMode.c_str());

    if (n <= 0 || n >= (int)sizeof(xml))
        return;

    // TLS is the only transport -> iOS TAK apps (OmniTAK / iTAK / TAK Aware)
#if defined(ESP32)
    tlsServerWrite(xml, n);
#endif
}

// Answer OS connectivity probes with 404 = "no internet" (NOT the success page,
// NOT captive HTML/redirect). This is the sweet spot: the phone keeps the WiFi
// associated for local traffic (UDP CoT) while routing INTERNET over cellular/LTE,
// and it avoids the captive-portal sheet that disconnects WiFi when backgrounded.
static void handleNoInternet()
{
    server.send(404, "text/plain", "no internet");
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
    LOG_INFO("TAK: open http://192.168.4.1 on the phone to sync clock + pick drone");

    // No DNS hijack on purpose: a catch-all DNS would poison the phone's lookups
    // and break its cellular internet. Leaving DNS alone lets the phone treat this
    // as a plain no-internet AP (WiFi kept for local UDP, internet via cellular).
    server.on("/", handleRoot);
    server.on("/settime", handleSetTime);
    server.on("/setdrone", handleSetDrone);
    server.on("/setsim", handleSetSim);
    // Everything else (incl. OS connectivity probes like /hotspot-detect.html)
    // -> 404 "no internet": phone keeps WiFi for local UDP, uses LTE for internet.
    // Reach the config page directly at http://192.168.4.1
    server.onNotFound(handleNoInternet);
    server.begin();

#if defined(ESP32)
    tlsServerBegin(COT_TLS_PORT);        // TLS only -> iOS TAK (OmniTAK / iTAK / TAK Aware)
    apiFd8443 = openApiPort(8443);       // accept iOS clients' API reachability probe
    apiFd8446 = openApiPort(8446);       // accept iOS clients' enrollment reachability probe
    LOG_INFO("TAK: CoT on TLS :%u only (OmniTAK / iTAK / TAK Aware), up to %d clients",
             COT_TLS_PORT, MAX_TLS_CLIENTS);
#else
    LOG_INFO("TAK: TLS CoT server requires ESP32; no transport on this target");
#endif
}

bool takClockSynced()
{
    return timeIsSet;
}

void takLoop()
{
    server.handleClient();
#if defined(ESP32)
    tlsServerService();
    serviceApiPort(apiFd8443);
    serviceApiPort(apiFd8446);
#endif

    uint32_t nowms = millis();

    // SoftAP association check: ground truth for "is the phone still on the AP".
    // iOS may show LTE for internet while staying associated here -> the TLS CoT
    // stream still works. Logged on change and every 10s.
    static uint8_t  lastSta    = 255;
    static uint32_t lastStaLog = 0;
    uint8_t sta = WiFi.softAPgetStationNum();
    if (sta != lastSta || nowms - lastStaLog > 10000)
    {
        lastSta    = sta;
        lastStaLog = nowms;
        LOG_INFO("TAK: SoftAP clients=%u (%s)", sta,
                 sta ? "phone associated - TLS CoT reachable" : "NO client connected");
    }

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

        if (takTestSimulate)
        {
            // Simulated orbit: only fly once a drone has been selected.
            if (!selectedDrone[0]) return;
        }
        else
        {
            // Field: wait for a real GPS fix (avoids a marker at 0,0).
            if (cur.lat == 0.0f && cur.lon == 0.0f) return;
        }

        sendCoT();
    }
}
