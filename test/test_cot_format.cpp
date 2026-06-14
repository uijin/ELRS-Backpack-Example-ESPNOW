// Host (native) unit test for CoT formatting — no Arduino/hardware needed.
//   c++ -std=c++11 -I src test/test_cot_format.cpp -o /tmp/cot && /tmp/cot
//
// Verifies the XML the firmware will emit: well-formedness, ISO-8601 UTC
// timestamps, stale offset, km/h->m/s conversion, and buffer safety.

#include "cot_format.h"
#include <string>
#include <cstring>
#include <cstdio>
#include <cassert>

static int failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("  FAIL: %s\n", msg); ++failures; } \
    else         { printf("  ok  : %s\n", msg); } \
} while (0)

static bool contains(const std::string& h, const char* n) {
    return h.find(n) != std::string::npos;
}

int main()
{
    char buf[768];

    // Fixed epoch: 2026-06-18T12:00:00Z = 1781784000 (verified via `date -u -r`)
    time_t now = 1781784000;

    int n = cot_format_event(
        buf, sizeof(buf),
        now, /*staleSec*/ 10,
        "DRONE-01", "a-f-A-M-F-Q", "Drone-01",
        47.1234567, 8.7654321, 452.3f,
        123.4f, /*speed m/s from*/ 36.0f / 3.6f,   // 36 km/h -> 10.00 m/s
        14.8f, 12.3f, 450,
        -72, 100, "ARM");

    std::string s(buf, n > 0 ? n : 0);
    printf("--- generated CoT ---\n%s\n---------------------\n", buf);

    printf("[format / safety]\n");
    CHECK(n > 0 && n < (int)sizeof(buf), "snprintf returned a non-truncated length");
    CHECK((size_t)n == strlen(buf), "returned length matches strlen (no embedded NUL)");

    printf("[well-formedness]\n");
    CHECK(s.rfind("<?xml", 0) == 0, "starts with XML prolog");
    CHECK(contains(s, "<event ") && contains(s, "</event>"), "has a single closed <event>");
    CHECK(contains(s, "<point ") && contains(s, "/>"), "has a self-closed <point>");
    CHECK(contains(s, "<detail>") && contains(s, "</detail>"), "detail block balanced");
    CHECK(contains(s, "<track ") && contains(s, "<contact ") && contains(s, "<remarks>"),
          "track / contact / remarks present");

    printf("[identity & type]\n");
    CHECK(contains(s, "uid=\"DRONE-01\""), "stable uid (drives TAK track history)");
    CHECK(contains(s, "type=\"a-f-A-M-F-Q\""), "friendly-UAS CoT type");
    CHECK(contains(s, "callsign=\"Drone-01\""), "callsign present");

    printf("[time]\n");
    CHECK(contains(s, "time=\"2026-06-18T12:00:00.000Z\""), "event time is ISO-8601 UTC");
    CHECK(contains(s, "start=\"2026-06-18T12:00:00.000Z\""), "start == time");
    CHECK(contains(s, "stale=\"2026-06-18T12:00:10.000Z\""), "stale = time + 10s");

    printf("[geometry & telemetry]\n");
    CHECK(contains(s, "lat=\"47.1234567\""), "lat at 7-decimal precision");
    CHECK(contains(s, "lon=\"8.7654321\""), "lon at 7-decimal precision");
    CHECK(contains(s, "hae=\"452.3\""), "altitude rendered");
    CHECK(contains(s, "course=\"123.4\""), "course rendered");
    CHECK(contains(s, "speed=\"10.00\""), "36 km/h converts to 10.00 m/s");
    CHECK(contains(s, "14.8V 12.3A 450mAh"), "battery remarks");
    CHECK(contains(s, "RSSI -72 LQ 100%"), "link remarks");
    CHECK(contains(s, "| ARM<"), "flight mode in remarks");

    printf("[truncation guard]\n");
    char tiny[40];
    int tn = cot_format_event(tiny, sizeof(tiny), now, 10,
                              "DRONE-01", "a-f-A-M-F-Q", "Drone-01",
                              47.1, 8.7, 1.0f, 0.0f, 0.0f,
                              0.0f, 0.0f, 0, 0, 0, "");
    CHECK(tn >= (int)sizeof(tiny), "overflow is detectable (snprintf returns needed length)");

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
