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
  #include <WiFiUdp.h>
  #include <WebServer.h>
  #include "esp_tls.h"            // TLS CoT server for iOS TAK (iTAK/OmniTAK)
  #include <lwip/sockets.h>
  #include <fcntl.h>
  #include "tak_cert.h"           // baked-in self-signed server cert + key
#else
  #include <ESP8266WiFi.h>
  #include <WiFiUdp.h>
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
static const char*    COT_TYPE        = "a-f-A-M-F-Q";  // friendly UAS (air track)
static const uint16_t COT_PORT        = 4242;           // UDP port (ATAK/WinTAK custom input)
static const uint16_t COT_TCP_PORT    = 8087;           // plain TCP server (desktop / legacy tools)
static const uint16_t COT_TLS_PORT    = 8089;           // TLS server (iOS TAK: iTAK / OmniTAK)
static const uint32_t COT_INTERVAL_MS = 1000;           // ~1 Hz position updates
static const uint32_t COT_STALE_SEC   = 10;             // marker greys out after this w/o updates

// TEST: when true, ignore real GPS and (once a drone is selected) simulate the
// drone orbiting a fixed circle so you get a moving track in TAK indoors.
// Set false for field use.
static const bool   TAK_TEST_SIMULATE   = true;
static const double SIM_CENTER_LAT      = 0.0;   // circle centre (set your test location)
static const double SIM_CENTER_LON      = 0.0;
static const double SIM_RADIUS_M        = 100.0;               // metres
static const double SIM_PERIOD_S        = 60.0;                // seconds per lap (~10.5 m/s)
static const float  SIM_ALT_M           = 50.0f;               // reported hae

#define MAX_TCP_CLIENTS 4

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
static WiFiServer cotServer(COT_TCP_PORT);   // streams CoT to connected TAK clients
static WiFiClient tcpClients[MAX_TCP_CLIENTS];
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
// TLS CoT server (iOS TAK clients connect here as an SSL "server")
// One client at a time; raw CoT XML over TLS (TAK protocol version 0).
// ======================================================
#if defined(ESP32)
static int        tlsListenFd = -1;
static esp_tls_t* tlsConn     = nullptr;
static int        tlsConnFd   = -1;

static void tlsServerBegin(uint16_t port)
{
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
    listen(tlsListenFd, 1);
    fcntl(tlsListenFd, F_SETFL, O_NONBLOCK);
    LOG_INFO("TAK: TLS CoT server on :%u", port);
}

static void tlsDrop()
{
    if (tlsConn) { esp_tls_server_session_delete(tlsConn); tlsConn = nullptr; }
    if (tlsConnFd >= 0) { close(tlsConnFd); tlsConnFd = -1; }
}

static void tlsServerService()
{
    if (tlsListenFd < 0) return;

    if (!tlsConn)
    {
        int cfd = accept(tlsListenFd, nullptr, nullptr);
        if (cfd < 0) return;                       // no pending client

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
            return;
        }
        fcntl(cfd, F_SETFL, O_NONBLOCK);           // non-blocking for steady-state
        tlsConn = tls; tlsConnFd = cfd;
        LOG_INFO("TAK: TLS client connected");
        return;
    }

    // Drain inbound chatter; detect close.
    char buf[128];
    int r = esp_tls_conn_read(tlsConn, buf, sizeof(buf));
    if (r == 0 ||
        (r < 0 && r != ESP_TLS_ERR_SSL_WANT_READ && r != ESP_TLS_ERR_SSL_WANT_WRITE))
    {
        LOG_INFO("TAK: TLS client disconnected");
        tlsDrop();
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
    if (!tlsConn) return;
    int off = 0;
    while (off < len)
    {
        int w = esp_tls_conn_write(tlsConn, data + off, len - off);
        if (w > 0) { off += w; continue; }
        if (w == ESP_TLS_ERR_SSL_WANT_WRITE || w == ESP_TLS_ERR_SSL_WANT_READ) continue;
        LOG_INFO("TAK: TLS write failed (%d), dropping client", w);
        tlsDrop();
        return;
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

    if (TAK_TEST_SIMULATE)
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

    // UDP broadcast -> ATAK / WinTAK custom inputs
    udp.beginPacket(COT_DEST, COT_PORT);
    udp.write((const uint8_t*)xml, n);
    udp.endPacket();

    // Plain-TCP stream -> desktop / legacy tools
    for (int i = 0; i < MAX_TCP_CLIENTS; ++i)
        if (tcpClients[i] && tcpClients[i].connected())
            tcpClients[i].write((const uint8_t*)xml, n);

    // TLS stream -> iOS TAK apps (iTAK / OmniTAK)
#if defined(ESP32)
    tlsServerWrite(xml, n);
#endif
}

// Accept new TCP CoT clients, drain their inbound chatter, drop dead sockets.
static void tcpServiceClients()
{
#if defined(ESP32)
    WiFiClient nc = cotServer.accept();
#else
    WiFiClient nc = cotServer.available();
#endif
    if (nc)
    {
        int slot = -1;
        for (int i = 0; i < MAX_TCP_CLIENTS; ++i)
            if (!tcpClients[i] || !tcpClients[i].connected()) { slot = i; break; }
        if (slot >= 0)
        {
            tcpClients[slot].stop();
            tcpClients[slot] = nc;
            LOG_INFO("TAK: TCP client connected (slot %d)", slot);
        }
        else
        {
            nc.stop();   // all slots busy
        }
    }

    for (int i = 0; i < MAX_TCP_CLIENTS; ++i)
        if (tcpClients[i] && tcpClients[i].connected())
            while (tcpClients[i].available()) tcpClients[i].read();   // discard client CoT
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
    // Everything else (incl. OS connectivity probes like /hotspot-detect.html)
    // -> 404 "no internet": phone keeps WiFi for local UDP, uses LTE for internet.
    // Reach the config page directly at http://192.168.4.1
    server.onNotFound(handleNoInternet);
    server.begin();

    udp.begin(COT_PORT);                 // UDP broadcast -> ATAK / WinTAK
    cotServer.begin();                   // plain TCP     -> desktop / legacy tools
    cotServer.setNoDelay(true);
#if defined(ESP32)
    tlsServerBegin(COT_TLS_PORT);        // TLS           -> iOS TAK (iTAK / OmniTAK)
    apiFd8443 = openApiPort(8443);       // accept iOS clients' API reachability probe
    apiFd8446 = openApiPort(8446);       // accept iOS clients' enrollment reachability probe
#endif
    LOG_INFO("TAK: CoT on UDP :%u (ATAK/WinTAK), TCP :%u, TLS :%u (iTAK/OmniTAK)",
             COT_PORT, COT_TCP_PORT, COT_TLS_PORT);
}

bool takClockSynced()
{
    return timeIsSet;
}

void takLoop()
{
    server.handleClient();
    tcpServiceClients();
#if defined(ESP32)
    tlsServerService();
    serviceApiPort(apiFd8443);
    serviceApiPort(apiFd8446);
#endif

    uint32_t nowms = millis();

    // SoftAP association check: ground truth for "is the phone still on the AP".
    // iOS may show LTE for internet while staying associated here -> UDP still
    // arrives. Logged on change and every 10s.
    static uint8_t  lastSta    = 255;
    static uint32_t lastStaLog = 0;
    uint8_t sta = WiFi.softAPgetStationNum();
    if (sta != lastSta || nowms - lastStaLog > 10000)
    {
        lastSta    = sta;
        lastStaLog = nowms;
        LOG_INFO("TAK: SoftAP clients=%u (%s)", sta,
                 sta ? "phone associated - UDP will arrive" : "NO client connected");
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

        if (TAK_TEST_SIMULATE)
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
