#pragma once

// ======================================================
// TAK / CoT Bridge
// ------------------------------------------------------
// Streams the decoded drone telemetry to an iOS TAK app
// (TAK Aware / iTAK) as Cursor-on-Target (CoT) UDP events.
//
// Topology: the ESP runs its own WiFi SoftAP on the SAME
// channel as the ELRS ESP-NOW link (channel 1). The phone
// joins that AP, so ESP-NOW telemetry reception and IP
// traffic coexist on one radio without a channel conflict.
//
// Time: the ESP has no RTC and no internet on the SoftAP,
// so a tiny captive-portal page lets the phone's browser
// hand over its (LTE-accurate) UTC clock. CoT events are
// only emitted once the clock has been set.
// ======================================================

// Bring up the SoftAP, UDP sender and captive-portal web server.
// Call once in setup() AFTER WiFi + ESP-NOW are initialised.
void takInit(const char* ssid, const char* password);

// Pump the DNS/web server and emit CoT on a timer.
// Call every loop() iteration (non-blocking).
void takLoop();
