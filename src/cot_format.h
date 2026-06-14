#pragma once

// ======================================================
// Pure CoT (Cursor-on-Target) event formatting.
//
// Deliberately free of Arduino / WiFi / CRSF dependencies so it can be
// unit-tested on the host. tak.cpp passes in a telemetry snapshot;
// this just renders the XML.
// ======================================================

#include <time.h>
#include <stdio.h>
#include <stddef.h>

// Render a CoT <event> into `out`. Returns the number of bytes written
// (excluding the NUL), or a negative / >= cap value on truncation, matching
// snprintf semantics so the caller can detect overflow.
static inline int cot_format_event(
    char* out, size_t cap,
    time_t now, unsigned long staleSec,
    const char* uid, const char* type, const char* callsign,
    double lat, double lon, float hae,
    float courseDeg, float speedMs,
    float volts, float amps, unsigned long mah,
    int rssi, int lq, const char* mode)
{
    char tNow[28], tStale[28];
    struct tm tmv;

    time_t t = now;
    gmtime_r(&t, &tmv);
    strftime(tNow, sizeof(tNow), "%Y-%m-%dT%H:%M:%S.000Z", &tmv);

    t = now + (time_t)staleSec;
    gmtime_r(&t, &tmv);
    strftime(tStale, sizeof(tStale), "%Y-%m-%dT%H:%M:%S.000Z", &tmv);

    return snprintf(out, cap,
        "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
        "<event version=\"2.0\" uid=\"%s\" type=\"%s\" how=\"m-g\" "
        "time=\"%s\" start=\"%s\" stale=\"%s\">"
        "<point lat=\"%.7f\" lon=\"%.7f\" hae=\"%.1f\" ce=\"9999999.0\" le=\"9999999.0\"/>"
        "<detail>"
        "<track course=\"%.1f\" speed=\"%.2f\"/>"
        "<contact callsign=\"%s\"/>"
        "<remarks>%.1fV %.1fA %lumAh | RSSI %d LQ %d%% | %s</remarks>"
        "</detail></event>",
        uid, type, tNow, tNow, tStale,
        lat, lon, hae,
        courseDeg, speedMs,
        callsign,
        volts, amps, mah,
        rssi, lq, mode);
}
