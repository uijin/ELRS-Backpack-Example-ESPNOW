#include "application.h"
#include "terseCRSF.h"
#include "logging.h"
#include "types.h"
#include "crsf_defines.h"
#include <math.h>
#include <string>

extern CRSF crsf;

extern int16_t hud_bat1_volts;
extern int16_t hud_bat1_amps;
extern uint16_t hud_bat1_mAh;

extern bool motArmed;

extern Location hom;
extern Location cur;

extern bool finalHomeStored;

/* ===================== */
/*        STATES         */
/* ===================== */

static bool prevGpsGood = false;
static std::string lastMode = "";

uint16_t channel[16];

/* ===================== */
/*     CRSF PROCESSING   */
/* ===================== */

void processCRSFFrame(uint8_t* buffer, uint16_t len)
{

    // ================= RC CHANNELS =================
    if (buffer[2] == CRSF_FRAMETYPE_RC_CHANNELS)   // CRSF_FRAMETYPE_RC_CHANNELS
    {
        LOG_INFO("RC Frame received (%d bytes)", len);
        crsf.decodeRC();
        return;
    }

    uint8_t crsf_id = crsf.decodeTelemetry(buffer, len);

    switch (crsf_id)
    {



        /* ================= GPS ================= */
        case GPS_ID:
        {
            cur.lat = crsf.gpsF_lat;
            cur.lon = crsf.gpsF_lon;
            cur.alt = crsf.gps_altitude;

            bool gpsfixGood = (crsf.gps_sats >= 5);
            bool lonGood    = (crsf.gpsF_lon != 0.0f);
            bool latGood    = (crsf.gpsF_lat != 0.0f);
            bool altGood    = (crsf.gps_altitude != 0);

            bool gpsGood = gpsfixGood && lonGood && latGood && altGood;

            if (finalHomeStored)
                cur.alt_ag = cur.alt - hom.alt;
            else
                cur.alt_ag = 0;

            /* ---- GPS LOG ---- */
            // change-gating temporarily removed: log every frame
            LOG_INFO("GPS: %.7f, %.7f alt=%d sats=%d",
                     cur.lat,
                     cur.lon,
                     crsf.gps_altitude,
                     crsf.gps_sats);

            /* ---- GPS STATUS CHANGE ---- */
            if (gpsGood != prevGpsGood)
            {
                LOG_INFO("GPS Status changed: gpsGood=%d", gpsGood);
                prevGpsGood = gpsGood;
            }

        } break;

        /* ================= VARIO ================= */
        case CF_VARIO_ID:
        {
            // crsf.decodeTelemetry(buffer, len) hat bereits crsf.vario / crsf.varioF gesetzt

            // rohdaten in cm/s
            int16_t climb_cm = crsf.vario;

            // in m/s für Berechnungen / Logging
            float climb_m_s = crsf.varioF;

            // change-gating temporarily removed: log every frame
            LOG_INFO("VARIO: %+.2f m/s (%d cm/s)", climb_m_s, climb_cm);

        } break;

        /* ================= BATTERY ================= */
        case BATTERY_ID:
        {
            hud_bat1_volts = crsf.batF_voltage;
            hud_bat1_amps  = crsf.batF_current;
            hud_bat1_mAh   = crsf.bat_fuel_drawn;   // mAh drawn (capacity used)

            // change-gating temporarily removed: log every frame
            LOG_INFO("BAT: %.1fV %.1fA %u%% %lumAh",
                     crsf.batF_voltage,
                     crsf.batF_current,
                     crsf.bat_remaining,
                     (unsigned long)crsf.bat_fuel_drawn);

        } break;

        /* ================= BAROMETER ================= */
        case BARO_ALT_ID:
        {
            // Rohwert aus CRSF-Klasse
            uint16_t alt_raw = crsf.baro_altitude;
            LOG_INFO("BARO: raw=%u", alt_raw);

        } break;

        /* ================= ATTITUDE ================= */
        case ATTITUDE_ID:
        {
            cur.hdg = crsf.attiF_yaw;

            // change-gating temporarily removed: log every frame
            LOG_INFO("ATT: p=%.1f r=%.1f y=%.1f",
                     crsf.attiF_pitch,
                     crsf.attiF_roll,
                     crsf.attiF_yaw);

        } break;

                /* ================= LINK STATISTICS (0x14) ================= */
        case LINK_ID:   // 0x14
        {
            LOG_INFO("LINK: RSSI1=%d RSSI2=%d Q=%d SNR=%d TX=%d RF=%d",
                     crsf.link_up_rssi_ant_1,
                     crsf.link_up_rssi_ant_2,
                     crsf.link_up_quality,
                     crsf.link_up_snr,
                     crsf.link_up_tx_power,
                     crsf.link_rf_mode);
        } break;

        /* ================= FLIGHT MODE ================= */
        case FLIGHT_MODE_ID:
        {
            motArmed = (crsf.flightMode.compare("ARM") == 0);

            // change-gating temporarily removed: log every frame
            LOG_INFO("MODE: %s Armed:%d",
                     crsf.flightMode.c_str(),
                     motArmed);

        } break;

        /* ================= DEVICE INFO (0x29) ================= */
        case DEVICE_INFO_ID:
        {
            // Extended CRSF frame:
            //   [0]=addr [1]=len [2]=0x29 [3]=dest [4]=origin
            //   [5..]=display name (NUL-terminated ASCII)
            //   then serial(u32) hw(u32) sw(u32) param_count(u8) param_proto(u8) [CRC]
            uint8_t origin = (len > 4) ? buffer[4] : 0;

            char name[40] = {0};
            uint16_t i = 5, j = 0;
            while (i < len && buffer[i] != 0 && j < sizeof(name) - 1)
                name[j++] = (char)buffer[i++];
            name[j] = '\0';

            uint16_t after = i + 1;   // step past the NUL terminator
            uint32_t serial = 0, hw = 0, sw = 0;
            uint8_t  param_count = 0, param_proto = 0;
            if (after + 14 <= len)
            {
                serial = ((uint32_t)buffer[after]   << 24) | ((uint32_t)buffer[after+1] << 16) |
                         ((uint32_t)buffer[after+2]  << 8)  |  (uint32_t)buffer[after+3];
                hw     = ((uint32_t)buffer[after+4]  << 24) | ((uint32_t)buffer[after+5] << 16) |
                         ((uint32_t)buffer[after+6]  << 8)  |  (uint32_t)buffer[after+7];
                sw     = ((uint32_t)buffer[after+8]  << 24) | ((uint32_t)buffer[after+9] << 16) |
                         ((uint32_t)buffer[after+10] << 8)  |  (uint32_t)buffer[after+11];
                param_count = buffer[after+12];
                param_proto = buffer[after+13];
            }

            const char* who = "?";
            switch (origin)
            {
                case 0xEA: who = "Radio";     break;
                case 0xEE: who = "TX-Module"; break;
                case 0xEC: who = "RX";        break;
                case 0xC8: who = "FC";        break;
            }

            LOG_INFO("DEVICE_INFO from 0x%02X (%s): name='%s' serial=0x%08lX hw=0x%08lX sw=0x%08lX params=%u proto=%u",
                     origin, who, name,
                     (unsigned long)serial, (unsigned long)hw, (unsigned long)sw,
                     param_count, param_proto);

            char hexString[256] = {0};
            char* ptr = hexString;
            for (uint16_t k = 0; k < len && (ptr - hexString) < (int)sizeof(hexString) - 4; k++)
                ptr += sprintf(ptr, "%02X ", buffer[k]);
            LOG_INFO("DEVICE_INFO raw: %s", hexString);

        } break;

        default:
        {
            // decodeTelemetry() returns 0 for any unrecognized type, masking the
            // real frame type. Log buffer[2] (the actual CRSF type byte) instead.
            LOG_INFO("Unknown CRSF ID: 0x%02X (len=%d)", buffer[2], len);

            // komplettes Paket als HEX dump ausgeben
            char hexString[512] = {0};
            char* ptr = hexString;

            for (uint16_t i = 0; i < len; i++)
            {
                ptr += sprintf(ptr, "%02X ", buffer[i]);
            }

            LOG_INFO("Frame Data: %s", hexString);
        }
        break;
    }
}
