/*

pio run -t nobuild -t factory_flash -e tasmota

*/
/*
  xsns_107_victron.ino - Victron VE.Direct protocol parser

  Copyright (C) 2023  MichaM

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
// #ifdef AAAA_BBBB_CCCC
#ifdef USE_VICTRON

#define HERE() AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s():%d HERE\n"), __FILE__, __func__, __LINE__)

/*********************************************************************************************\
 * Victron VE.Diret UART protocol parser
 *
 * To use Tasmota the user needs to add an ESP8266 or ESP32
\*********************************************************************************************/

#include <vector>
#include <string>
#include <utility>
#include <queue>
#include <map>

#include <TasmotaSerial.h>

#define HTTP_VICTRON_4S_FORTMATSTR "<tr>  <th>%s</th> <td></td> <td>%s</td> <td></td> <td>%s</td> <td></td> <td>%s</td>  </tr>"

#define XSNS_107                 107
#define VICTRON_BAUTRATE 19200
#define VICTRON_MSG_SIZE (512) /* must be at least 19200 / 4 == 480 bytes! */

#define MIN(a,b)  ((a) < (b) ? (a) : (b))

typedef std::pair<std::string, std::string> PSS;
typedef std::vector<PSS> VPSS;

typedef struct {
  const char *product_name;
  const char *pid;
} Pair;

typedef struct {
  const char *label;
  const char *unit;
  const char *description;
  const char *alt_name;
  float multiplication_factor;
  const char *res_unit;
} Tuple;


static const Pair pid_mapping[] PROGMEM = {
  {"BMV-700", "0x203"},
  {"BMV-702", "0x204"},
  {"BMV-700H", "0x205"},
  {"BlueSolar MPPT 70|15", "0x0300"},
  {"BlueSolar MPPT 75|50", "0xA040"},
  {"BlueSolar MPPT 150|35", "0xA041"},
  {"BlueSolar MPPT 75|15", "0xA042"},
  {"BlueSolar MPPT 100|15", "0xA043"},
  {"BlueSolar MPPT 100|30", "0xA044"},
  {"BlueSolar MPPT 100|50", "0xA045"},
  {"BlueSolar MPPT 150|70", "0xA046"},
  {"BlueSolar MPPT 150|100", "0xA047"},
  {"BlueSolar MPPT 100|50 rev2", "0xA049"},
  {"BlueSolar MPPT 100|30 rev2", "0xA04A"},
  {"BlueSolar MPPT 150|35 rev2", "0xA04B"},
  {"BlueSolar MPPT 75|10", "0xA04C"},
  {"BlueSolar MPPT 150|45", "0xA04D"},
  {"BlueSolar MPPT 150|60", "0xA04E"},
  {"BlueSolar MPPT 150|85", "0xA04F"},
  {"SmartSolar MPPT 250|100", "0xA050"},
  {"SmartSolar MPPT 150|100", "0xA051"},
  {"SmartSolar MPPT 150|85", "0xA052"},
  {"SmartSolar MPPT 75|15", "0xA053"},
  {"SmartSolar MPPT 75|10", "0xA054"},
  {"SmartSolar MPPT 100|15", "0xA055"},
  {"SmartSolar MPPT 100|30", "0xA056"},
  {"SmartSolar MPPT 100|50", "0xA057"},
  {"SmartSolar MPPT 150|35", "0xA058"},
  {"SmartSolar MPPT 150|100 rev2", "0xA059"},
  {"SmartSolar MPPT 150|85 rev2", "0xA05A"},
  {"SmartSolar MPPT 250|70", "0xA05B"},
  {"SmartSolar MPPT 250|85", "0xA05C"},
  {"SmartSolar MPPT 250|60", "0xA05D"},
  {"SmartSolar MPPT 250|45", "0xA05E"},
  {"SmartSolar MPPT 100|20", "0xA05F"},
  {"SmartSolar MPPT 100|20 48V", "0xA060"},
  {"SmartSolar MPPT 150|45", "0xA061"},
  {"SmartSolar MPPT 150|60", "0xA062"},
  {"SmartSolar MPPT 150|70", "0xA063"},
  {"SmartSolar MPPT 250|85 rev2", "0xA064"},
  {"SmartSolar MPPT 250|100 rev2", "0xA065"},
  {"BlueSolar MPPT 100|20", "0xA066"},
  {"BlueSolar MPPT 100|20 48V", "0xA067"},
  {"SmartSolar MPPT 250|60 rev2", "0xA068"},
  {"SmartSolar MPPT 250|70 rev2", "0xA069"},
  {"SmartSolar MPPT 150|45 rev2", "0xA06A"},
  {"SmartSolar MPPT 150|60 rev2", "0xA06B"},
  {"SmartSolar MPPT 150|70 rev2", "0xA06C"},
  {"SmartSolar MPPT 150|85 rev3", "0xA06D"},
  {"SmartSolar MPPT 150|100 rev3", "0xA06E"},
  {"BlueSolar MPPT 150|45 rev2", "0xA06F"},
  {"BlueSolar MPPT 150|60 rev2", "0xA070"},
  {"BlueSolar MPPT 150|70 rev2", "0xA071"},
  {"SmartSolar MPPT VE.Can 150/70", "0xA102"},
  {"SmartSolar MPPT VE.Can 150/45", "0xA103"},
  {"SmartSolar MPPT VE.Can 150/60", "0xA104"},
  {"SmartSolar MPPT VE.Can 150/85", "0xA105"},
  {"SmartSolar MPPT VE.Can 150/100", "0xA106"},
  {"SmartSolar MPPT VE.Can 250/45", "0xA107"},
  {"SmartSolar MPPT VE.Can 250/60", "0xA108"},
  {"SmartSolar MPPT VE.Can 250/70", "0xA109"},
  {"SmartSolar MPPT VE.Can 250/85", "0xA10A"},
  {"SmartSolar MPPT VE.Can 250/100", "0xA10B"},
  {"SmartSolar MPPT VE.Can 150/70 rev2", "0xA10C"},
  {"SmartSolar MPPT VE.Can 150/85 rev2", "0xA10D"},
  {"SmartSolar MPPT VE.Can 150/100 rev2", "0xA10E"},
  {"BlueSolar MPPT VE.Can 150/100", "0xA10F"},
  {"BlueSolar MPPT VE.Can 250/70", "0xA112"},
  {"BlueSolar MPPT VE.Can 250/100", "0xA113"},
  {"SmartSolar MPPT VE.Can 250/70 rev2", "0xA114"},
  {"SmartSolar MPPT VE.Can 250/100 rev2", "0xA115"},
  {"SmartSolar MPPT VE.Can 250/85 rev2", "0xA116"},
  {"Phoenix Inverter 12V 250VA 230V", "0xA201"},
  {"Phoenix Inverter 24V 250VA 230V", "0xA202"},
  {"Phoenix Inverter 48V 250VA 230V", "0xA204"},
  {"Phoenix Inverter 12V 375VA 230V", "0xA211"},
  {"Phoenix Inverter 24V 375VA 230V", "0xA212"},
  {"Phoenix Inverter 48V 375VA 230V", "0xA214"},
  {"Phoenix Inverter 12V 500VA 230V", "0xA221"},
  {"Phoenix Inverter 24V 500VA 230V", "0xA222"},
  {"Phoenix Inverter 48V 500VA 230V", "0xA224"},
  {"Phoenix Inverter 12V 250VA 230V", "0xA231"},
  {"Phoenix Inverter 24V 250VA 230V", "0xA232"},
  {"Phoenix Inverter 48V 250VA 230V", "0xA234"},
  {"Phoenix Inverter 12V 250VA 120V", "0xA239"},
  {"Phoenix Inverter 24V 250VA 120V", "0xA23A"},
  {"Phoenix Inverter 48V 250VA 120V", "0xA23C"},
  {"Phoenix Inverter 12V 375VA 230V", "0xA241"},
  {"Phoenix Inverter 24V 375VA 230V", "0xA242"},
  {"Phoenix Inverter 48V 375VA 230V", "0xA244"},
  {"Phoenix Inverter 12V 375VA 120V", "0xA249"},
  {"Phoenix Inverter 24V 375VA 120V", "0xA24A"},
  {"Phoenix Inverter 48V 375VA 120V", "0xA24C"},
  {"Phoenix Inverter 12V 500VA 230V", "0xA251"},
  {"Phoenix Inverter 24V 500VA 230V", "0xA252"},
  {"Phoenix Inverter 48V 500VA 230V", "0xA254"},
  {"Phoenix Inverter 12V 500VA 120V", "0xA259"},
  {"Phoenix Inverter 24V 500VA 120V", "0xA25A"},
  {"Phoenix Inverter 48V 500VA 120V", "0xA25C"},
  {"Phoenix Inverter 12V 800VA 230V", "0xA261"},
  {"Phoenix Inverter 24V 800VA 230V", "0xA262"},
  {"Phoenix Inverter 48V 800VA 230V", "0xA264"},
  {"Phoenix Inverter 12V 800VA 120V", "0xA269"},
  {"Phoenix Inverter 24V 800VA 120V", "0xA26A"},
  {"Phoenix Inverter 48V 800VA 120V", "0xA26C"},
  {"Phoenix Inverter 12V 1200VA 230V", "0xA271"},
  {"Phoenix Inverter 24V 1200VA 230V", "0xA272"},
  {"Phoenix Inverter 48V 1200VA 230V", "0xA274"},
  {"Phoenix Inverter 12V 1200VA 120V", "0xA279"},
  {"Phoenix Inverter 24V 1200VA 120V", "0xA27A"},
  {"Phoenix Inverter 48V 1200VA 120V", "0xA27C"},
  {"Phoenix Inverter 12V 1600VA 230V", "0xA281"},
  {"Phoenix Inverter 24V 1600VA 230V", "0xA282"},
  {"Phoenix Inverter 48V 1600VA 230V", "0xA284"},
  {"Phoenix Inverter 12V 2000VA 230V", "0xA291"},
  {"Phoenix Inverter 24V 2000VA 230V", "0xA292"},
  {"Phoenix Inverter 48V 2000VA 230V", "0xA294"},
  {"Phoenix Inverter 12V 3000VA 230V", "0xA2A1"},
  {"Phoenix Inverter 24V 3000VA 230V", "0xA2A2"},
  {"Phoenix Inverter 48V 3000VA 230V", "0xA2A4"},
  {"Phoenix Smart IP43 Charger 12|50 (1+1)", "0xA340"},
  {"Phoenix Smart IP43 Charger 12|50 (3)", "0xA341"},
  {"Phoenix Smart IP43 Charger 24|25 (1+1)", "0xA342"},
  {"Phoenix Smart IP43 Charger 24|25 (3)", "0xA343"},
  {"Phoenix Smart IP43 Charger 12|30 (1+1)", "0xA344"},
  {"Phoenix Smart IP43 Charger 12|30 (3)", "0xA345"},
  {"Phoenix Smart IP43 Charger 24|16 (1+1)", "0xA346"},
  {"Phoenix Smart IP43 Charger 24|16 (3)", "0xA347"},
  {"BMV-712 Smart", "0xA381"},
  {"BMV-710H Smart", "0xA382"},
  {"BMV-712 Smart Rev2", "0xA383"},
  {"SmartShunt 500A/50mV", "0xA389"},
  {"SmartShunt 1000A/50mV", "0xA38A"},
  {"SmartShunt 2000A/50mV", "0xA38B"}
};

static const Tuple field_metadata[] PROGMEM = {
// label       unit        description                          alt_name
  {"V",        "mV",       "Main or channel 1 (battery) voltage", "Chan 1 bat voltage", 0.001, "V"},
  {"V2",       "mV",       "Channel 2 (battery) voltage", "Chan 2 bat voltage", 0.001, "V"},
  {"V3",       "mV",       "Channel 3 (battery) voltage", "Chan 3 bat voltage", 0.001, "V"},
  {"VS",       "mV",       "Auxiliary (starter) voltage", "Aux voltage", 0.001, "V"},
  {"VM",       "mV",       "Mid-point voltage of the battery bank", NULL, 0.001, "V"},
  {"DM",       "%%",       "Mid-point deviation of the battery bank", NULL, -1.0, NULL},
  {"VPV",      "mV",       "Panel voltage", NULL, 0.001, "V"},
  {"PPV",      "W",        "Panel power", NULL, -1.0, NULL},
  {"I",        "mA",       "Main or channel 1 battery current", "Chan 1 bat current", 0.001, "A"},
  {"I2",       "mA",       "Channel 2 battery current", "Chan 2 bat current", 0.001, "A"},
  {"I3",       "mA",       "Channel 3 battery current", "Chan 3 bat current", 0.001, "A"},
  {"IL",       "mA",       "Load current",                              NULL, 0.001, "A"},
  {"LOAD",     "",         "Load output state (ON/OFF)",                NULL, -1.0, NULL},
  {"T",        "°C",       "Battery temperature",                       NULL, -1.0, NULL},
  {"P",        "W",        "Instantaneous power",                       NULL, -1.0, NULL},
  {"CE",       "mAh",      "Consumed Amp Hours",                        NULL, 0.001, "Ah"},
  {"SOC",      "%%",       "State-of-charge",                           NULL, -1.0, NULL},
  {"TTG",      "Minutes",  "Time-to-go",                                NULL, -1.0, NULL},
  {"Alarm",    "",         "Alarm condition active",                    NULL, -1.0, NULL},
  {"Relay",    "",         "Relay state",                               NULL, -1.0, NULL},
  {"AR",       "",         "Alarm reason",                              NULL, -1.0, NULL},
  {"OR",       "",         "Off reason",                                NULL, -1.0, NULL},
  {"H1",       "mAh",      "Depth of the deepest discharge",            NULL, 0.001, "Ah"},
  {"H2",       "mAh",      "Depth of the last discharge",               NULL, 0.001, "Ah"},
  {"H3",       "mAh",      "Depth of the average discharge",            NULL, 0.001, "Ah"},
  {"H4",       "",         "Number of charge cycles",                   NULL, -1.0, NULL},
  {"H5",       "",         "Number of full discharges",                 NULL, -1.0, NULL},
  {"H6",       "mAh",      "Cumulative Amp Hours drawn",                NULL, 0.001, "Ah"},
  {"H7",       "mV",       "Minimum main (battery) voltage",            NULL, 0.001, "V"},
  {"H8",       "mV",       "Maximum main (battery) voltage",            NULL, 0.001, "V"},
  {"H9",       "Seconds",  "Number of seconds since last full charge",  NULL, -1.0, NULL},
  {"H10",      "",         "Number of automatic synchronizations",          NULL, -1.0, NULL},
  {"H11",      "",         "Number of low main voltage alarms",             NULL, -1.0, NULL},
  {"H12",      "",         "Number of high main voltage alarms",            NULL, -1.0, NULL},
  {"H13",      "",         "Number of low auxiliary voltage alarms",        NULL, -1.0, NULL},
  {"H14",      "",         "Number of high auxiliary voltage alarms",       NULL, -1.0, NULL},
  {"H15",      "mV",       "Minimum auxiliary (battery) voltage",           NULL, 0.001, "V"},
  {"H16",      "mV",       "Maximum auxiliary (battery) voltage",           NULL, 0.001, "V"},
  {"H17",      "0.01 kWh", "Amount of discharged energy (BMV) / Amount of produced energy (DC monitor)", NULL, 0.01f, "kWh"},
  {"H18",      "0.01 kWh", "Amount of charged energy (BMV) / Amount of consumed energy (DC monitor)", NULL, 0.01f, "kWh"},
  {"H19",      "0.01 kWh", "Yield total (user resettable counter)",         "Total yield", 0.01f, "kWh"},
  {"H20",      "0.01 kWh", "Yield today", NULL, 0.01f, "kWh"},
  {"H21",      "W",        "Maximum power today", NULL, -1.0, NULL},
  {"H22",      "0.01 kWh", "Yield yesterday", NULL, 0.01f, "kWh"},
  {"H23",      "W",        "Maximum power yesterday", NULL, -1.0, NULL},
  {"ERR",      "",         "Error code", NULL, -1.0, NULL},
  {"CS",       "",         "State of operation", NULL, -1.0, NULL},
  {"BMV",      "",         "Model description (deprecated)", NULL, -1.0, NULL},
  {"FW",       "",         "Firmware version (16 bit)", NULL, -1.0, NULL},
  {"FWE",      "",         "Firmware version (24 bit)", NULL, -1.0, NULL},
  {"PID",      "",         "Product ID", "Product", -1.0, NULL},
  {"SER#",     "",         "Serial number", NULL, -1.0, NULL},
  {"HSDS",     "",         "Day sequence number (0..364)", NULL, -1.0, NULL},
  {"MODE",     "",         "Device mode", NULL, -1.0, NULL},
  {"AC_OUT_V", "0.01 V",   "AC output voltage", NULL, 0.01f, "V"},
  {"AC_OUT_I", "0.1 A",    "AC output current", NULL, 0.01f, "A"},
  {"AC_OUT_S", "VA",       "AC output apparent power", NULL, -1.0, NULL},
  {"WARN",     "",         "Warning reason", NULL, -1.0, NULL},
  {"MPPT",     "",         "Tracker operation mode", NULL, -1.0, NULL},
  {"MON",      "",         "DC monitor mode", NULL, -1.0, NULL}
};


/* array of the following struct */
typedef struct {
    uint64_t nbytes = 0; // counts every byte that this serial instance has received
    bool last_250ms_had_data = false;
    TasmotaSerial *serial = NULL;
    uint32_t pin; // which GPIO pin is assigned to this serial instance
    std::string backlog;
    VPSS last_msg;
} Victron_Serial;


struct VICTRON {
  uint64_t nbytes = 0; // at 168 bytes/S  32 bit would overflow after 295.89 days.
  uint32_t _50ms = 0;
  uint32_t _250ms = 0;
  uint32_t _xsns107 = 0;
  bool last_250ms_had_data = false;
  TasmotaSerial *VictronSerial = NULL;
  std::string backlog;
  VPSS last_msg;

  std::vector<Victron_Serial*> serial_lst; // new
} g_Victron;


const Tuple *get_et(const char *key) {
  for (size_t i = 0; i < sizeof(field_metadata)/sizeof(field_metadata[0]); i++) {
    if (0 == strcmp(key, field_metadata[i].label))
      return &field_metadata[i];
  }
  return NULL;
}


static bool read_uart(std::string & rv, void *vp) {
  VICTRON *v = (VICTRON*)vp;
  rv = ""; // empty string
  bool do_loop = true; //
  do {
    uint8_t buf[128] = {0};
    int s_available = v->VictronSerial->available();
    size_t toread = MIN(sizeof(buf), s_available);
    size_t  got = v->VictronSerial->read(buf, toread);
    if (got == (size_t)s_available) do_loop = false; // no need for a loop
    rv += std::string(buf, buf+got);
    v->nbytes += got;
  } while (do_loop);

  return true; // success
}

static bool read_uart2(std::string & rv, void *vp) {
  Victron_Serial *vs = (Victron_Serial*)vp;
  if (!vs) return false;
  rv = "";
  bool do_loop = true; //
  do {
    uint8_t buf[128] = {0};
    int s_available = vs->serial->available();
    size_t toread = MIN(sizeof(buf), s_available);
    size_t  got = vs->serial->read(buf, toread);
    if (got == (size_t)s_available) do_loop = false; // no need for a loop
    rv += std::string(buf, buf+got);
    vs->nbytes += got;
  } while (do_loop);

  return true; // success
}


static bool VictronVerifyChecksum(const std::string & s) {
  if (s.size() < 12) return false; // 12 because of   >\r\nChecksum\t?<
  uint8_t qsum = 0;
  for (size_t i = 0; i < s.size(); i++)
    qsum = (qsum + (uint8_t)s[i]) & 0xff;

  return qsum == 0; // 0 is the expected sum
}


static uint8_t get_checksum(const std::string & s) {
  if (s.size() < 12) return 0; // 12 because of   >\r\nChecksum\t?<
  return (uint8_t)s[s.size()-1];
}

/*
 * ## Message format ##
 *
 * The device transmits blocks of data at 1 second intervals. Each field
 * is sent using the following format:
 *
 * <Newline><Field-Lable><Tab><Field-Value>
 *
 * +---------------+--------------------------------------------------------+
 * | Identifier    | Meaning                                                |
 * +---------------|--------------------------------------------------------+
 * | <Newline>     | A carriage return followed by a line feed \r\n         |
 * |               | (0x0d, 0x0a).                                          |
 * +---------------|--------------------------------------------------------+
 * | <Field-Label> | An arbitary length label that identifies the field.    |
 * |               | Where applicable, this will be the same as the label   |
 * |               | that is used on the LCD.                               |
 * +---------------|--------------------------------------------------------+
 * | <Tab>         | A horizontal tab \t (0x09).                            |
 * +---------------|--------------------------------------------------------+
 * | <Field-Value> | The ASCII formatted value of this field. The number of |
 * |               | characters transmitted depends on the magnitude and    |
 * |               | sign of the value.                                     |
 * +---------------|--------------------------------------------------------+
 *
 *
 * ## Data integrity ##
 *
 * The statistics are grouped in blocks with a checksum appended. The last
 * field in a block will always be "Checksum". The value is a single byte, and
 * will not necessarily be a printable ASCII character. The modulo 256 sum of
 * all bytes in a block will equal 0 if there were no transmission errors.
 * Multiple blocks are sent containing different fields.
 * (Source: https://www.victronenergy.com/upload/documents/VE.Direct-Protocol-3.32.pdf)
**/

static void Victron250ms(void *vp) {                // Every 250ms
  if (!vp) return;
  VICTRON *v = (VICTRON*)vp;
  if (v->VictronSerial == NULL)
    return;
  int s_available = v->VictronSerial->available();
  if (s_available == 0) { // No new data available on UART
    if (v->last_250ms_had_data && v->_250ms >= 4 ) { // we need to process the queued data
      size_t len = v->backlog.size();
      if (VictronVerifyChecksum(v->backlog) == true) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s():%d Checksum_ok: 0x%02x\n"), __FILE__, __func__, __LINE__, (unsigned)get_checksum(v->backlog));
        size_t pos = v->backlog.find("Checksum"); // this should be right at the end at position len - 10
        if (pos + 10 == len) { // syntactical correct ending
          char prev_char = 0;
          size_t start_idx = 0;
          // \r\nKEY_LABEL1\tVALUE1\r\nKEY_LABEL2\tVALUE2\r\nKEY_LABEL3\t\VALUE3...
          std::string key, value;
          VPSS vpss;
          for (size_t i = 0; i < len; i++) {
            char c = v->backlog[i];
            if (i > 0 && c == '\t') { // key has ended...
              key = v->backlog.substr(start_idx, i - start_idx);
            } else if (i > 0 && (c == '\r' || (i+1)==len)) { // value has ended...
              value = v->backlog.substr(start_idx, i - start_idx);
              vpss.push_back(PSS(key, value));
            }
            if (prev_char == '\t' || prev_char == '\n') start_idx = i;

            prev_char = c;
          }
          if (key == "Checksum") {
            value = v->backlog.substr(start_idx, len - start_idx);
            if (value.size() == 1) {
              vpss.push_back(PSS(key, value));
            }
          }
          v->last_msg = vpss;
        }
      } else {
        AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s():%d Checksum_FAIL: 0x%02x\n"), __FILE__, __func__, __LINE__, (unsigned)get_checksum(v->backlog));
      }
      v->backlog = ""; // empty backlog data
    }
    v->last_250ms_had_data = false;
  } else { // s_available == 0
    std::string s = "";
    bool rv = read_uart(s, vp);
    if (rv) {
      v->backlog += s;
      v->last_250ms_had_data = true;
    } else {
      v->last_250ms_had_data = false;
    }
  }
}

static void Victron250msV2(void *vp) {
  if (!vp) return;
  VICTRON *v = (VICTRON*)vp;
  // AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s() v->serial_lst.size(): %d\n"), __FILE__, __func__, (int)v->serial_lst.size());
  for (size_t i = 0; i < v->serial_lst.size(); i++) {
    Victron_Serial *cur = (Victron_Serial*)v->serial_lst[i];
    int s_available = cur->serial->available();
    AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s() loop %d, available: %d\n"), __FILE__, __func__, (int)i, s_available);
    if (s_available == 0) {
      size_t len = cur->backlog.size();
      if (cur->last_250ms_had_data && v->_250ms >= 4 && len > 12) { // we need to process the queued data
        if (VictronVerifyChecksum(cur->backlog) == true) {
          AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s():%d (%d) Checksum_OK: 0x%02x, bl: %d\n"), __FILE__, __func__, __LINE__, (int)i, (unsigned)get_checksum(v->backlog), (int)len);
          size_t pos = cur->backlog.find("Checksum"); // this should be right at the end at position len - 10
          if (pos + 10 == len) { // syntactical correct ending
            HERE();
            char prev_char = 0;
            size_t start_idx = 0;
            // \r\nKEY_LABEL1\tVALUE1\r\nKEY_LABEL2\tVALUE2\r\nKEY_LABEL3\t\VALUE3...
            std::string key, value;
            VPSS vpss;
            for (size_t i = 0; i < len; i++) { // tokenize
              char c = cur->backlog[i];
              if (i > 0 && c == '\t') { // key has ended...
                key = cur->backlog.substr(start_idx, i - start_idx);
              } else if (i > 0 && (c == '\r' || (i+1)==len)) { // value has ended...
                value = cur->backlog.substr(start_idx, i - start_idx);
                vpss.push_back(PSS(key, value));
              }
              if (prev_char == '\t' || prev_char == '\n') start_idx = i;
              prev_char = c;
            }
            if (key == "Checksum") {
              value = cur->backlog.substr(start_idx, len - start_idx);
              if (value.size() == 1) {
                vpss.push_back(PSS(key, value));
              }
            }
            HERE();
            cur->last_msg = vpss;
          } else {
            AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s():%d pos(%d) + 10 != len(%d)"), __FILE__, __func__, __LINE__, (int)pos, (int)len);
          }
        } else {
          AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s():%d (%d) Checksum_FAIL: 0x%02x, bl: %d\n"), __FILE__, __func__, __LINE__, (int)i, (unsigned)get_checksum(cur->backlog), (int)len);
        }
        cur->backlog = ""; // empty backlock
      }
    } else { // s_available == 0
      HERE();
      std::string s = "";
      bool rv = read_uart2(s, cur);
      if (rv) {
        cur->backlog += s;
        v->nbytes += s.size();
        cur->last_250ms_had_data = true;
      } else {
        cur->last_250ms_had_data = false;
      }
    }
  }
}


static void VictronInit(void *vp) {
  if (!vp) return;
#if 0
  VICTRON *v = (VICTRON*)vp;
  if (PinUsed(GPIO_VICTRON_VEDIRECT_RX)) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s(void):%d Pin(GPIO_VICTRON_VEDIRECT_RX): %d, GPIO_VICTRON_VEDIRECT_RX:(%d)\n"), __FILE__, __func__, __LINE__,
                            (int)Pin(GPIO_VICTRON_VEDIRECT_RX), (int)GPIO_VICTRON_VEDIRECT_RX);
    v->VictronSerial = new TasmotaSerial(Pin(GPIO_VICTRON_VEDIRECT_RX), -1, 1 /* hw fallback */, 0 /* nwmode, 0 default */, VICTRON_MSG_SIZE /* buf size */);
    if (v->VictronSerial == NULL) return;
    if (v->VictronSerial->begin(VICTRON_BAUTRATE)) { // 480 bytes / 250ms
      if (v->VictronSerial->hardwareSerial()) {
        ClaimSerial();
      }
    }
  }
#else
  VICTRON *v = (VICTRON*)vp;
  int gpio_lst[] = {GPIO_VICTRON_VEDIRECT_RX0, GPIO_VICTRON_VEDIRECT_RX1, GPIO_VICTRON_VEDIRECT_RX2, GPIO_VICTRON_VEDIRECT_RX3};
  for (int i = 0; i < sizeof(gpio_lst)/sizeof(gpio_lst[0]); i++) {
    int gpio = gpio_lst[i];
    AddLog(LOG_LEVEL_INFO, PSTR("%s:%s() gpio_lst[%d] = %d (%d)\n"), __FILE__, __func__, i, gpio, (int)PinUsed(gpio));
    if (PinUsed(gpio)) {
      Victron_Serial *vs = (Victron_Serial*) new Victron_Serial;
      if (vs) {
        vs->nbytes = 0;
        vs->last_250ms_had_data = false;
        vs->serial = new TasmotaSerial(Pin(gpio), -1, 1, 0, VICTRON_MSG_SIZE);
        if (!vs->serial) {
          AddLog(LOG_LEVEL_INFO, PSTR("%s:%s() vs->serial == NULL\n"), __FILE__, __func__);
          delete vs;
          return;
        }
        vs->serial->begin(VICTRON_BAUTRATE);
        vs->backlog = "";
        v->serial_lst.push_back(vs);
      }
    }
  }
#endif
}


static void process_PID(const Tuple *tuple, const char *value, std::string & rv) {
  if (!tuple || !value) return;
  for (size_t i = 0; i < sizeof(pid_mapping)/sizeof(pid_mapping[0]); i++) {
    if (0 == strcasecmp(value, pid_mapping[i].pid)) {
      rv = std::string(pid_mapping[i].product_name) + " (" + std::string(value) + ")";
      return;
    }
  }
  rv = "Unknown (" + std::string(value) + ")";
}


static void my_fmt(const Tuple *tuple, const char *value, std::string & rv) {
  if (tuple == NULL || value == NULL) return;
  if (0 == strcmp("PID", tuple->label)) {
    process_PID(tuple, value, rv);
    return;
  }
  char tmp[128] = {0};
  float factor = tuple->multiplication_factor;
  const char *unit = tuple->res_unit;
  if (unit == NULL) unit = tuple->unit; // fallback to default
  if (factor > 0.0) { // if some sort of multiplication shall happen
    int v = 0;
    if (1 == sscanf(value, "%i", &v)) {
      float x = factor * v;
      ext_snprintf_P(tmp, sizeof(tmp), PSTR("%*_f %s"), 3, &x, unit);
      // AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s():%d v: %d, value: >%s< unit: >%s< tmp:>%s< %d\n"), __FILE__, __func__, __LINE__,
      //                                                    v, value, unit, tmp, (int)x);
    }
  } else {
    snprintf(tmp, sizeof(tmp), "%s %s", value, unit);
  }
  rv = std::string(tmp);
  // rv += " (" + std::string(tuple->label) + ")";
}


static void VictronShow(bool json, void *vp) {
  if (!vp) return;
  VICTRON *v = (VICTRON*)vp;
  if (json) {
    auto t0 = millis();
    ResponseAppend_P(PSTR(",\"VictronVE.Direct\":[" ));
    if (v->last_msg.size() >= 1 && v->last_msg[v->last_msg.size()-1].first == "Checksum") {
      for (size_t i = 0; i < v->last_msg.size()-1; i++) {
        auto p = v->last_msg[i];
        std::string key = p.first;
        std::string value = p.second;
        std::string unit = "n/a";
        std::string desc = "n/a";
        if (key == "Checksum") continue; // skip the Checksum entry as that will be handled below
        const Tuple *node = get_et(key.c_str());
        if (node != NULL) {
          unit = std::string(node->unit);
          desc = std::string(node->description);
        }
        ResponseAppend_P(PSTR("{"));
          ResponseAppend_P(PSTR("\"field\": \"%s\","), key.c_str());
          ResponseAppend_P(PSTR("\"unit\": \"%s\","), unit.c_str());
          ResponseAppend_P(PSTR("\"value\": \"%s\","), value.c_str());
          ResponseAppend_P(PSTR("\"description\": \"%s\""), desc.c_str());
        ResponseAppend_P(PSTR("},"));
      }
      char cs[16] = {0};
      snprintf(cs, sizeof(cs), "0x%02x", (unsigned)v->last_msg[v->last_msg.size()-1].second[0]);
      ResponseAppend_P(PSTR("{\"Checksum\": \"%s\"}]"), cs);
      ResponseAppend_P(PSTR(",\"bytes_read\": %lld"), (long long)v->nbytes);
    }
    auto t1 = millis();
    auto delta = t1 - t0;
    ResponseAppend_P(PSTR(",\"MetaTS\": {\"Start_TS\": %lld, \"End_TS\": %lld, \"Delta\": %lld}"), (long long)t0, (long long)t1, (long long)delta);
#ifdef USE_WEBSERVER
  } else { // if json
    char tmpbuf[20] = {0};
    for (int idx = 0; idx < v->serial_lst.size(); idx++) {
      if (idx > 0) { // not before first
        WSContentSend_P(PSTR("<hr>\n"));
      }
      AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s():%d idx=%d\n"), __FILE__, __func__, __LINE__, (int)idx);
      const Victron_Serial *cur = v->serial_lst[idx];
      if (cur->last_msg.size() >= 1 && cur->last_msg[cur->last_msg.size()-1].first == "Checksum") {
        HERE();
        for (size_t i = 0; i < cur->last_msg.size()-1; i++) {
          auto p = cur->last_msg[i];
          std::string key = p.first;
          std::string value = p.second;
          std::string unit = "n/a";
          std::string desc = "n/a";
          if (key == "Checksum") continue; // skip the Checksum entry as that will be handled below
          const Tuple *node = get_et(key.c_str());
          if (node != NULL) {
            unit = std::string(node->unit);
            desc = std::string(node->description);
          }
          // WSContentSend_P(PSTR(HTTP_VICTRON_4S_FORTMATSTR), key.c_str(), value.c_str(), unit.c_str(), desc.c_str());
// #define HTTP_VICTRON_4S_FORTMATSTR "<tr>  <th>%s</th> <td></td> <td>%s</td> <td></td> <td>%s</td> <td></td> <td>%s</td>  </tr>"
          {
            std::string formated_value = "";
            std::string desc = "";
            if (node->alt_name) desc = std::string(node->alt_name);
            else desc = std::string(node->description);
            my_fmt(node, value.c_str(), formated_value);
            WSContentSend_PD(PSTR("<tr><th>%s</th><td>%s</td></tr>\n"), desc.c_str(), formated_value.c_str());
          }
        }

        snprintf(tmpbuf, sizeof(tmpbuf), "0x%02x", (unsigned)cur->last_msg[cur->last_msg.size()-1].second[0]);
        // WSContentSend_P(PSTR(HTTP_VICTRON_4S_FORTMATSTR), "Checksum", tmpbuf, "", "Single byte checksum over data");
        WSContentSend_P(PSTR("<tr><th>%s</th><td>%s</td></tr>\n"), "Checksum", tmpbuf);
      } else {
        HERE();
        AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s():%d cur->last_msg.size()=%d first=%s\n"), __FILE__, __func__, __LINE__, (int)cur->last_msg.size(), "xxx");
      }
    } // serial_lst-loop
    snprintf(tmpbuf, sizeof(tmpbuf), "%lld", (long long)v->nbytes);
    // WSContentSend_P(PSTR(HTTP_VICTRON_4S_FORTMATSTR), "nbytes", tmpbuf, "", "Bytes received on UART");
    WSContentSend_P(PSTR("<tr><th>%s</th><td>%s</td></tr>\n"), "Received bytes", tmpbuf);
#endif // USE_WEBSERVER
  } // if json
}


bool Xsns107(uint32_t function) {
  VICTRON *v = &g_Victron;
  v->_xsns107++;

  switch (function) {
    case FUNC_EVERY_50_MSECOND:
      v->_50ms++;
      break;
    case FUNC_EVERY_100_MSECOND:
      break;
    case FUNC_EVERY_200_MSECOND: // seems not to be called...
      break;
    case FUNC_EVERY_250_MSECOND:
      HERE();
      v->_250ms++;
      // Victron250ms((void*)v);
      Victron250msV2((void*)v);
      break;
    case FUNC_LOOP:
      break;
    case FUNC_SLEEP_LOOP:
      break;
    case FUNC_EVERY_SECOND:
      HERE();
      break;
    case FUNC_JSON_APPEND:
      HERE();
      VictronShow(true, (void*)v);
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      HERE();
      VictronShow(false, (void*)v);
      break;
#endif  // USE_WEBSERVER
    case FUNC_INIT:
      HERE();
      VictronInit((void*)v);
      break;
    default:
      break;
  }
  return true;
}
#endif  // USE_VICTRON
