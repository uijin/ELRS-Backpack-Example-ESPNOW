#if defined(ESP32)
#pragma message "ESP32 stuff happening!"
#else
#pragma message "ESP8266 stuff happening!"
#endif

// ===== Logging Level =====
//#define LOG_LEVEL LOG_LEVEL_INFO   // oder INFO
//#define LOG_LEVEL LOG_LEVEL_INFO
#define LOG_LEVEL LOG_LEVEL_DEBUG
#include "logging.h"

// ======================================================
// 1. Includes
// ======================================================

#include <terseCRSF.h>  // https://github.com/zs6buj/terseCRSF   use v 0.0.6 or later
#include "msp.h"   // MSP Paket Handling
#include "msptypes.h"
#include "fake_vrx_fake_trainer.h"
#include <math.h>
#include "application.h"
#include "types.h"
#include "tak.h"

// ======================================================
// 2. Platform Selection (ESP32 / ESP8266)
// ======================================================

#if defined(ESP32)
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_timer.h>
#else
#include <ESP8266WiFi.h>
#include <espnow.h>
#endif

enum RadioMode
{
    MODE_RX_ONLY = 0,
    MODE_TX_ONLY = 1,
    MODE_BOTH    = 2
};

// ======================================================
// 3. Configuration (Defines, UID, WiFi, Logging)
// ======================================================

// Please use ExpressLRS Configurator Runtime Options to obtain your UID (unique MAC hashed
// from binding_phrase) Insert the six numbers between the curly brackets below
//For ESP NOW / ELRS Backpack you need to enter the UID resulting from hashing your
//secret binding phrase. This must be obtained by launching https://expresslrs.github.io/web-flasher/,
//enter your binding phrase, then make a note of the UID. Enter the 6 numbers between the commas
// Config Elrs Binding
//uint8_t UID[6] = {0,0,0,0,0,0}; // this is my UID. You have to change it to your once, should look
//uint8_t UID[6] = {106,19,19,206,193,30};
uint8_t UID[6] = {0,0,0,0,0,0};   // <-- enter YOUR UID (hash of your binding phrase)

// ======================================================
// RadioMode Configuration
// ======================================================
//
// MODE_RX_ONLY  : Only receive ESP-NOW data (telemetry).
//                 Sending (trainer/VTX channels) is disabled.
//
// MODE_TX_ONLY  : Only send ESP-NOW data (trainer/VTX channels).
//                 Receiving telemetry is disabled.
//
// MODE_BOTH     : Send and receive ESP-NOW data.
//                 Full bidirectional operation.
//
// You can change the default mode below.
// It can also be changed later via Serial commands.
// ======================================================
RadioMode radioMode = MODE_BOTH;   // Default startup mode

// Time window (in milliseconds) after boot during which
// the RadioMode can be changed via Serial commands.
// After this time, the mode becomes fixed.
const unsigned long CONFIG_WINDOW_MS = 5000;  // 5 seconds

// ===== AP Wifi Config =====
const char* ssid = "Backpack_TAK";              // SSID of the SoftAP (change me)
const char* password = "changeme123";                 // AP password, min 8 chars (change me)

// ===== Status LED (Seeed Studio XIAO ESP32-C6 user LED) =====
// The XIAO ESP32-C6 user LED sits on GPIO15 and is active-LOW (LOW = on).
// Driven by an esp_timer (independent of loop()), so the cadence stays
// stable even when loop() stalls in server.handleClient() / blocking I/O.
// Cadence: waiting for phone time sync -> 0.3s on / 0.7s off (1s period).
//          after time sync            -> 0.5s on / 2.5s off (3s heartbeat).
#define STATUS_LED_PIN        15
#define STATUS_LED_ON         LOW
#define STATUS_LED_OFF        HIGH
#define LED_ON_MS_UNSYNCED    300
#define LED_OFF_MS_UNSYNCED   700
#define LED_ON_MS_SYNCED      500
#define LED_OFF_MS_SYNCED     2500

// ===== Config for Channel output =====
const uint8_t NUM_CHANNELS = 16;
#define CHANNEL_MIN 172
#define CHANNEL_MAX 1811
// ===== in the Example there is a wave for the output channels that you can see the channes are moving
#define WAVE_SPEED 0.05
float wavePhase = 0.0;
#define STEP_INTERVAL 100   // 100ms Offset for each channel when the wave starts to the channel bevor

// ======================================================
// 4. Global Objects
// ======================================================


unsigned long configWindowStart = 0;
uint16_t rampChannels[NUM_CHANNELS];
uint32_t lastStepTime = 0;
uint8_t activeChannel = 0;
bool rampUp = true;

volatile bool espnow_received = false;
volatile uint16_t espnow_len = 0;
uint8_t espnow_buffer[250];
volatile uint16_t crsf_len = 0;

#if defined(ESP32)
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#endif

// ======================================================
// Platform Specific
// ======================================================
#if defined(ESP32)
QueueHandle_t rxqueue;
#endif

// ======================================================
// Module Instances
// ======================================================
FakeVRXFakeTrainer vrxModule;
MSP recv_msp;
CRSF crsf;

// ===============================
// RC Channels
// ===============================
uint16_t rc_channels[NUM_CHANNELS] = {0};

// ===============================
// Telemetry Values
// ===============================
int16_t hud_bat1_volts = 0;
int16_t hud_bat1_amps  = 0;
uint16_t hud_bat1_mAh  = 0;

bool motArmed = false;

// ===============================
// GPS Struct
// ===============================

Location hom = {0,0,0,0,0};
Location cur = {0,0,0,0,0};

// ===============================
bool finalHomeStored = false;

// ======================================================
// ESP-NOW Receive Callback
// ======================================================
#if defined(ESP32)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
// Arduino-ESP32 3.x / ESP-IDF 5.x changed the recv callback signature
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
#else
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
#endif

    if (len <= 8 || incomingData == NULL) return;

    if (incomingData[8] == 0 && incomingData[9] == 0 && len > 10) {
        return;
    }

    crsf_len = len - 8;

    if (crsf_len > sizeof(espnow_buffer)) {
        crsf_len = sizeof(espnow_buffer);
    }

    memcpy(espnow_buffer, incomingData + 8, crsf_len);

    espnow_received = true;
}

// ======================================================
// ESP-NOW Send Callback
// ======================================================

#if defined(ESP32)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
// Arduino-ESP32 3.x / ESP-IDF 5.x changed the send callback signature
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    // Beim ESP32 ist status 0 (ESP_NOW_SEND_SUCCESS)
    bool success = (status == ESP_NOW_SEND_SUCCESS);
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Beim ESP32 ist status 0 (ESP_NOW_SEND_SUCCESS)
    bool success = (status == ESP_NOW_SEND_SUCCESS);
#endif
#else
void OnDataSent(uint8_t *mac_addr, uint8_t status) {
    // Beim ESP8266 ist status 0 meist Erfolg
    bool success = (status == 0);
#endif

    if (success) {
        LOG_DEBUG_INLINE("ESP-NOW Send OK          ");
    } else {
        LOG_ERROR_INLINE("ESP-NOW Send FAIL (Status: %d) t=%lu ms      ", status, millis());
    }
}

// ======================================================
// 7. Setup()
// ======================================================

void initSerial()
{
    Serial.begin(115200);
    LOG_INFO("Start");
}


void initWiFi()
{
    UID[0] &= ~0x01;   // unicast fix
    // AP_STA: STA interface keeps the UID MAC for ESP-NOW telemetry RX,
    // while the SoftAP (brought up later in takInit) serves the phone.
    WiFi.mode(WIFI_AP_STA);
    WiFi.disconnect();

    #if defined(ESP32)
        // Deine echte Identität bleibt die UID! (Wichtig für den RX-Empfang)
        esp_wifi_set_mac(WIFI_IF_STA, UID);

        uint8_t ap_mac[6];
        memcpy(ap_mac, UID, 6);
        ap_mac[5] ^= 0x01;
        esp_wifi_set_mac(WIFI_IF_AP, ap_mac);

        esp_wifi_start();
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    #else
        wifi_set_macaddr(STATION_IF, UID);
        wifi_set_channel(1);
    #endif
}

void initESPNow()
{
    #if defined(ESP32)
        if (esp_now_init() != ESP_OK)
    #else
        if (esp_now_init() != 0)
    #endif
    {
        LOG_ERROR("ESP-NOW init failed");
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    #if defined(ESP32)
        // Für den ESP32 müssen wir hier keine Sende-Peers registrieren.
        // Der RX-Empfang geht automatisch.
        // Das TX-Senden (Senden an sich selbst) wird über den L2-Raw-Bypass
        // in der fake_vrx_fake_trainer.cpp abgewickelt!
    #else
        // FÜR ESP8266: Bleibt komplett unverändert auf der Original-Logik
        esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
        esp_now_add_peer(UID, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
    #endif
}

void initRamp()
{
    for(int i = 0; i < NUM_CHANNELS; i++)
        rampChannels[i] = CHANNEL_MIN;
}

void initESP32Queue()
{
#if defined(ESP32)
    rxqueue = xQueueCreate(20, sizeof(mspPacket_t));

    if (rxqueue == NULL)
    {
        LOG_ERROR("Queue creation failed");
    }
#endif
}

void initInfo()
{
    LOG_INFO("==================================================");
    LOG_INFO("Radio Mode Configuration");
    LOG_INFO("Viewpoint: ESP Module Role (not the RC transmitter)");
    LOG_INFO("--------------------------------------------------");
    LOG_INFO("RX_ONLY : ESP receives ESP-NOW data and decodes it.");
    LOG_INFO("          Sending is disabled.");
    LOG_INFO("TX_ONLY : ESP sends ESP-NOW data.");
    LOG_INFO("          Receiving/decoding is disabled.");
    LOG_INFO("BOTH    : Full bidirectional operation.");
    LOG_INFO("--------------------------------------------------");
    LOG_INFO("Config window active for %lu ms", CONFIG_WINDOW_MS);
    LOG_INFO("Change mode via Serial during this window:");
    LOG_INFO("  1 = RX_ONLY");
    LOG_INFO("  2 = TX_ONLY");
    LOG_INFO("  3 = BOTH");
    LOG_INFO("==================================================");

    configWindowStart = millis();
}

static esp_timer_handle_t statusLedTimer = nullptr;
static volatile bool      statusLedOn     = false;

// Fires from the esp_timer task (independent of loop()), so blink timing is
// immune to loop() stalls in server.handleClient() / blocking serial I/O.
// One-shot, self-rearming: toggles the LED then schedules the next edge using
// the current on/off duration for the current sync state.
static void statusLedTimerCb(void* /*arg*/)
{
    statusLedOn = !statusLedOn;
    digitalWrite(STATUS_LED_PIN, statusLedOn ? STATUS_LED_ON : STATUS_LED_OFF);

    bool synced = takClockSynced();
    uint32_t nextMs = statusLedOn
        ? (synced ? LED_ON_MS_SYNCED  : LED_ON_MS_UNSYNCED)
        : (synced ? LED_OFF_MS_SYNCED : LED_OFF_MS_UNSYNCED);

    esp_timer_start_once(statusLedTimer, (uint64_t)nextMs * 1000ULL);
}

void initStatusLed()
{
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, STATUS_LED_OFF);
    statusLedOn = false;

    const esp_timer_create_args_t args = {
        .callback             = &statusLedTimerCb,
        .arg                  = nullptr,
        .dispatch_method      = ESP_TIMER_TASK,
        .name                 = "status_led",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&args, &statusLedTimer);

    // LED starts off; fire after the off-duration to begin the first on-phase.
    esp_timer_start_once(statusLedTimer, (uint64_t)LED_OFF_MS_UNSYNCED * 1000ULL);
}

void setup() {
    initSerial();
    initStatusLed();
    delay(3000);
    initWiFi();
    initESPNow();
    initESP32Queue();
    vrxModule.init(UID);
    initRamp();
    takInit(ssid, password);   // SoftAP + CoT/UDP bridge for iOS TAK
    initInfo();
}

// ======================================================
// 8. Loop()
// ======================================================

void loop()
{

    // ======================================================
    // Serial Mode Switch (only during config window)
    // ======================================================
    if (millis() - configWindowStart < CONFIG_WINDOW_MS)
    {
        if (Serial.available())
        {
            char c = Serial.read();

            // Nur echte Ziffern akzeptieren
            if (c >= '1' && c <= '3')
            {
                radioMode = (RadioMode)(c - '1');

                LOG_INFO("RadioMode changed to %d", radioMode + 1);
            }
        }
    }

    // ======================================================
    // ESP-NOW Receive Handling (CRSF)
    // ======================================================
   if (radioMode != MODE_TX_ONLY)
    {
        if (espnow_received)
        {
            uint16_t len_to_process = 0;

            #if defined(ESP32)
            portENTER_CRITICAL(&mux);
            #else
            noInterrupts();
            #endif

            len_to_process = crsf_len;
            memcpy(crsf.crsf_buf, espnow_buffer, len_to_process);
            espnow_received = false;

            #if defined(ESP32)
            portEXIT_CRITICAL(&mux);
            #else
            interrupts();
            #endif

            processCRSFFrame(crsf.crsf_buf, len_to_process);
        }
    }

    // ======================================================
    // Regular Mode
    // ======================================================
    if (radioMode != MODE_RX_ONLY)
    {
        vrxModule.updateChannelRamp();   // ESP-NOW senden
    }

    // ======================================================
    // TAK / CoT Bridge (SoftAP web server + UDP CoT emit)
    // ======================================================
    takLoop();
    // Status LED is driven by esp_timer (see initStatusLed), not from loop().
}
