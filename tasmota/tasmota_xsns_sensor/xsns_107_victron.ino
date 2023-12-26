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
#define VICTRON_MSG_SIZE (512) /* must be at least more than bytes in 250ms at 19200 Bd: 1920 / 4 == 480 bytes! */

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

static std::string PID2Devicename(const char* const pid)
{
  typedef struct {
    const char *product_name;
    const char *pid;
  } Pair;

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
  
  std::string rv = std::string(pid);
  for (size_t i = 0; i < sizeof(pid_mapping)/sizeof(pid_mapping[0]); i++) {
    if (0 == strcasecmp(pid, pid_mapping[i].pid)) {
      rv = std::string(pid_mapping[i].product_name) + " (" + pid + ")";
      break;
    }
  }
  return rv;
};

static std::string CS2name(const char* const CS) {

  static const std::pair<int, const char*> CSnames[] PROGMEM = {
    {0, "Off"},
    {1, "Low power"},
    {2, "Fault"},
    {3, "Bulk"},
    {4, "Absorbtion"},
    {5, "Float"},
    {6, "Storage"},
    {7, "Equalize (manual)"},
    {9, "Inverting"},
    {11, "Power supply"},
    {245, "Starting-up"},
    {246, "Repeated Absorption"},
    {247, "Auto Equalize / Recondition"},
    {248, "BatterySafe"},
    {252,"External Control"} 
  };
  std::string rv = std::string(CS);
  int cs_id = 0;
  if (1 == sscanf(CS, "%i", &cs_id)) {
    for (size_t i = 0; i < sizeof(CSnames)/sizeof(CSnames[0]); i++) {
      if (CSnames[i].first == cs_id) {
        rv = std::string(CSnames[i].second);
        break;
      }
    }
  }
  return rv;
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


enum class State {
  IN_KEY,
  IN_VALUE,
  IN_CHECKSUM,
  IN_COMMAND,
  IN_HEX,
  UNKNOWN
};

class Receiver {
  public:
  virtual void getText(const VPSS& data)=0;
  virtual void getHexData(const std::string& cmd, const std::string&arg)=0;
  virtual ~Receiver() {};
};

class Parser {
public:
  Parser() : state(State::UNKNOWN), checksum(0), receiverPtr(0) {};
  void parseCharacter(const char c);
  // whenever data gets ready (either a single  )
  void setReceiver(Receiver* p) { receiverPtr=p; }
private:

  void pushKeyValue();
  void emitHexMessage();
  void emitText();
  State state;
  uint32_t checksum;
  std::string key;
  std::string value;
  std::string hexline;
  VPSS keyValues;
  Receiver* receiverPtr;

};

static const std::size_t NUM_DEVICES = 1;

class VICTRON : Receiver {
  public:
  uint32_t nbytesToday = 0; // at 168 bytes/S  32 bit will overflow. who cares about that number?
  uint32_t _50ms = 0;
  uint32_t _250ms = 0;
  //uint32_t _xsns107 = 0;
  bool last_250ms_had_data = false;
  TasmotaSerial *serial = NULL;
  std::string backlog;
  VPSS last_msg;
  Parser parser;
  void init(TasmotaSerial*p) { serial = p; last_msg.clear(); parser.setReceiver(this); }
  void readSerialBuffer();
  virtual void getText(const VPSS& data);
  virtual void getHexData(const std::string& cmd, const std::string&arg);
  virtual ~VICTRON() {};
} g_Victron[NUM_DEVICES];

void VICTRON::readSerialBuffer() 
{
  if (serial == NULL) return;
  int ret=-1;
  auto bytesAvailable = serial->available();
  if (bytesAvailable > 0) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s(): parsing %d bytes...\n"), __FILE__, __func__, bytesAvailable);
  }
  for( int i=0; i<bytesAvailable; ++i ) {
    auto ret= serial->read();
    if (ret==-1) break; // should never happen because char is available
    nbytesToday++;
    parser.parseCharacter(static_cast<char>(ret&0xff));
  }
}

void VICTRON::getText(const VPSS& data)
{
  last_msg=data;
  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("%s:%s(): got %d KV-pairs\n"), __FILE__, __func__, (int) last_msg.size());
}
void VICTRON::getHexData(const std::string& cmd, const std::string&arg)
{
HERE();
}



// maybe the static array is not a good idea, so we access an instance only by
VICTRON* getVP(int i=0)
{
  if (i<0 || i >= NUM_DEVICES) return nullptr; 
  return & (g_Victron[i]);
};


const Tuple *get_et(const char *key) {
  for (size_t i = 0; i < sizeof(field_metadata)/sizeof(field_metadata[0]); i++) {
    if (0 == strcmp(key, field_metadata[i].label))
      return &field_metadata[i];
  }
  return NULL;
}


static bool read_uart(std::string & rv, VICTRON* v) {
  rv = ""; // empty string
  bool do_loop = true; //
  do {
    uint8_t buf[128] = {0};
    int s_available = v->serial->available();
    size_t toread = MIN(sizeof(buf), s_available);
    size_t  got = v->serial->read(buf, toread);
    if (got == (size_t)s_available) do_loop = false; // no need for a loop
    rv += std::string(buf, buf+got);
    v->nbytesToday += got;
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

Additionally, if the HEX protocol is activated, then HEX blocks can be intermixed
That gives an input that looks like this:

PID	0xA042
FW	150
SER#	HQ1933IGKS2
V	13120
I	220
VPV	16000
PPV	2
CS	3
MPPT	1
ERR	0
LOAD	ON
IL	0
H19	2502
H20	6
H21	53
H22	4
H23	19
HSDS	351
Checksum	œ:55041BF
:A0002000148
:A0102000345
:A0202000200000045
:AD7ED00020085
:AD5ED00200564
:ABCED001B01000086
:ABBED0040065D
:AB3ED0001AA
:AADED000000B1
:AA8ED0001B5
:A30200001FA
:ABCED001A01000087
:AB3ED0002A9
:ABBED0042065B
:ABCED001901000088
:ABBED00470656
:AD5ED00210563
:A0002000148
:A0102000345
:A0202000200000045
:AD7ED00020085

PID	0xA042
FW	150
SER#	HQ1933IGKS2
V	13130
I	210
.....

The frame format of the VE.Direct HEX protocol has the following general format:
: [command] [data][data][…] [check]\n
Where the colon indicates the start of the frame and the newline is the end of frame. The sum of all
data bytes and the check must equal 0x55. Since the normal protocol is in text values the frames are
sent in their hexadecimal ASCII representation, [‘0’ .. ’9’], [‘A’ .. ’F’], must be uppercase. There is no
need to escape any characters.
: [command] [dataHighNibble, dataLowNibble][……] [checkHigh, checkLow] \n
Note: The command is only send as a single nibble. Numbers are sent in Little Endian format. An
error response with value 0xAAAA is sent on framing errors. More details can be found in
the documenation of 

So, I implemented a state machine that discriminates the data stream HEX Blocks and the key-value text blocks
and emit tokens to a given receiver whenever a valid item (either validate text block or Hex Item ) is detected.

**/

void Parser::parseCharacter(const char c)
{
  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("%s:%s(): state: %d char 0x%02x\n"), __FILE__, __func__, (int) state, (int)c);
  switch (state) {
  case State::UNKNOWN:
    if (c=='\r') {
      checksum=static_cast<uint8>(c);
      state = State::IN_KEY;
      key="";
      value="";
    }
    else if (c ==':') {
      checksum=static_cast<uint8>(c);
      state = State::IN_HEX;
    }
    // all other characters are skipped until we sync to \r or :
    break;

  case State::IN_KEY:
    checksum+=static_cast<uint8>(c);
    if (c == '\r') {
      if (key.size() >0) {
        // error, eol before value
        state = State::UNKNOWN;
      }
      //skip
    }
    else if (c =='\n') {
      if (key.size() >0) {
        // error, eol before value
        state = State::UNKNOWN;
      }
      // is counted for checksum but not part of key
    }
    else if (c == '\t') {
      if (key == "Checksum") {
        state = State::IN_CHECKSUM;
      }
      else {
        state = State::IN_VALUE;
        // key done
        //key="";
      }
    }
    else {
      if (key.size() >9) {
        key.clear();
        // protocol error as keys only hae up to 9 chars according protocol definition
        state= State::UNKNOWN;
      }

      key.push_back(c);
    }
    break;

  case State::IN_CHECKSUM:
    {
    int  cs = checksum & 0xff;
    AddLog(LOG_LEVEL_INFO, PSTR("%s:%s(): IN_CHECKSUM %d, char %d, total %d\n"), __FILE__, __func__, cs, (int)c, checksum);
    // operate the checksum
    if (checksum & 0xff == static_cast<uint8_t>(c)) {
      // checksum is OK, we can emit all key/value pairs now
      emitText();
    }
     else {
      // bad luck. clear and start again
      emitText(); // ignore bad checksum until we fixed that calculation bug
    }
    keyValues.clear();
    // this is tricky: the next char may be a : from the next HEX or a carriage return that starts a newtext
    state = State::UNKNOWN;
    checksum=0;
    }
    break;
  
  case State::IN_HEX:
    if (c == '\n') {
      emitHexMessage();
      hexline="";
      state=State::UNKNOWN;  // either text or hex may follow  
    }
    else if (c == '\r') {
      // skip, should not happen, but helpful for testing with file test data
    }
    else {
      hexline.push_back(c);
    };
    break;

  case State::IN_VALUE:
    checksum+=static_cast<uint8>(c);
    if (c == '\r') {
      // end of value because new block seen
      pushKeyValue();
      state = State::IN_KEY;
      key="";
      value="";
    }
    else {
      if (value.size() > 33) {
        // according to protocoll only up to 33 chars as value
        value.clear();
        state = State::UNKNOWN;
      }
      else {
        value.push_back(c);
      }
    }
    break;

  }
};

void Parser::pushKeyValue()
{
  AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s(): gotKeyValue %s: %s\n"), __FILE__, __func__, key.c_str(), value.c_str());
  keyValues.push_back(PSS(key, value));
}

void Parser::emitHexMessage()
{
  AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s(): gotHexMsg %s\n"), __FILE__, __func__, hexline.c_str());

  keyValues.push_back(PSS(key, value));

}

void Parser::emitText()
{
  if (receiverPtr) receiverPtr->getText(keyValues);
}

static void Victron250ms() 
{                // Every 250ms
  VICTRON *v = getVP();
  if (!v) return;
  v->readSerialBuffer();
}   

static void OldVictron250ms() 
{                // Every 250ms
  VICTRON *v = getVP();
  if (!v) return;
  if (v->serial == NULL)
    return;
  int s_available = v->serial->available();
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
    bool rv = read_uart(s, v);
    if (rv) {
      v->backlog += s;
      v->last_250ms_had_data = true;
    } else {
      v->last_250ms_had_data = false;
    }
  }
}

static void VictronInit() {
  static uint callCount=0;
  // the first defvice is assigned to RX TX

  if (PinUsed(GPIO_VICTRON_VEDIRECT_RX)) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s(void):%d Pin(GPIO_VICTRON_VEDIRECT_RX): %d, GPIO_VICTRON_VEDIRECT_RX:(%d)\n"), __FILE__, __func__, __LINE__,
                            (int)Pin(GPIO_VICTRON_VEDIRECT_RX), (int)GPIO_VICTRON_VEDIRECT_RX);
    TasmotaSerial* sp = new TasmotaSerial(Pin(GPIO_VICTRON_VEDIRECT_RX), -1, 1 /* hw fallback */, 0 /* nwmode, 0 default */, VICTRON_MSG_SIZE /* buf size */);
    if (sp == NULL) return;
    if (sp->begin(VICTRON_BAUTRATE)) { // 480 bytes / 250ms
      if (sp->hardwareSerial()) {
        ClaimSerial();
        AddLog(LOG_LEVEL_DEBUG, PSTR("%s: hwserial claimed\n"), __func__);
      }
    }
    // now we have a valid serial device that we can give the first instance
    VICTRON* v = getVP();
    if (v) {
      v->init(sp);
      AddLog(LOG_LEVEL_DEBUG, PSTR("%s: RX buffer %d\n"), __func__, (int)sp->getRxBufferSize());
    }
  }
  // add initialization to all others
}

static void my_fmt(const Tuple *tuple, const char *value, std::string & rv) {
  if (tuple == NULL || value == NULL) return;
  if (0 == strcmp("PID", tuple->label)) {
    rv = PID2Devicename(value);
    return;
  }
  if (0 == strcmp("CS", tuple->label)) {
    rv = CS2name(value);
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


static void VictronShowJSON() {

  VICTRON *v = getVP();
  if (!v) return;

    auto t0 = millis();
    ResponseAppend_P(PSTR(",\"VictronVE.Direct\":[" ));
    if (v->last_msg.size() >= 1 && 1 /*v->last_msg[v->last_msg.size()-1].first == "Checksum" */) {
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
      ResponseAppend_P(PSTR(",\"bytes_read\": %lld"), (long long)v->nbytesToday);
    }
    auto t1 = millis();
    auto delta = t1 - t0;
    ResponseAppend_P(PSTR(",\"MetaTS\": {\"Start_TS\": %lld, \"End_TS\": %lld, \"Delta\": %lld}"), (long long)t0, (long long)t1, (long long)delta);
  }

#ifdef USE_WEBSERVER
static void VictronShowWeb() {
  VICTRON *v = getVP();
  if (!v) return;
  char tmpbuf[20] = {0};
  for (int idx = 0; idx < 1; idx++) { // v->serial_lst.size(); idx++) {
    if (idx > 0) { // not before first
      WSContentSend_P(PSTR("<hr>\n"));
    }
    AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s():%d idx=%d"), __FILE__, __func__, __LINE__, (int)idx);
    const VICTRON *cur = v; //->serial_lst[idx];
    for (size_t i = 0; i < cur->last_msg.size(); i++) {
      auto kvp = cur->last_msg[i];
      std::string key = kvp.first;
      std::string value = kvp.second;
      std::string unit = "";
      std::string desc = key;
      const Tuple *node = get_et(key.c_str());
      if (node != NULL) {
        unit = std::string(node->unit);
        desc = std::string(node->description);
      }
      {
        std::string formated_value = value;
        desc = (node->alt_name != NULL) ? std::string(node->alt_name) : desc;
        my_fmt(node, value.c_str(), formated_value);

        WSContentSend_PD(PSTR("<tr><th>%s</th><td>%s</td></tr>\n"), desc.c_str(), formated_value.c_str());
      }
    }
  } // serial_lst-loop
  snprintf(tmpbuf, sizeof(tmpbuf), "%lld", (long long)v->nbytesToday);
  // WSContentSend_P(PSTR(HTTP_VICTRON_4S_FORTMATSTR), "nbytes", tmpbuf, "", "Bytes received on UART");
  WSContentSend_P(PSTR("<tr><th>%s</th><td>%s</td></tr>\n"), "Received bytes", tmpbuf);
}
#endif // USE_WEBSERVER

static void VicronAtMidnight()
{
   VICTRON *v = getVP();
  if (!v) return;   
  v->nbytesToday=0;   
}

bool Xsns107(uint32_t function) {

  switch (function) {
    case FUNC_EVERY_50_MSECOND:
      break;
    case FUNC_EVERY_100_MSECOND:
      break;
    case FUNC_EVERY_200_MSECOND: // seems not to be called...
      break;
    case FUNC_EVERY_250_MSECOND:
      Victron250ms();
      break;
    case FUNC_LOOP:
      break;
    case FUNC_SLEEP_LOOP:
      break;
    case FUNC_EVERY_SECOND:
      break;
    case FUNC_JSON_APPEND:
      VictronShowJSON();
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      VictronShowWeb();
      break;
#endif  // USE_WEBSERVER
    case FUNC_SAVE_AT_MIDNIGHT:
      VicronAtMidnight();
      break;
    case FUNC_INIT:
      VictronInit();
      break;
//    default:
//      break;
  }
  return true;
}
#endif  // USE_VICTRON
