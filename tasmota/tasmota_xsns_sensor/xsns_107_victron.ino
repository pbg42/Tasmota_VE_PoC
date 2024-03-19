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

#include <TasmotaSerial.h>

#define XSNS_107                 107
#define VICTRON_BAUTRATE 19200
#define VICTRON_MSG_SIZE (512) /* must be at least more than # of bytes in 250ms at 19200 Bd: 1920 / 4 == 480 bytes! */

typedef std::pair<std::string, std::string> PSS;
typedef std::vector<PSS> VPSS;

// make shure that the mapping is held in PROGMEM. copy the value from PROGMEM to RAM, or the key itself if not found
static std::string PID2Devicename(const char* const pid)
{
struct PAIR {
  const char first[40];
  const char second[7];
};
static const PAIR pid_mapping[] PROGMEM = {
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
    if (0 == strcasecmp_P(pid, pid_mapping[i].second)) {
      char tmp[40];
      strncpy_P(tmp, pid_mapping[i].first, sizeof(tmp));
      rv = std::string(tmp) + " (" + pid + ")";
      break;
    }
  }
  return rv;
};

static std::string CS2name(const char* const val) {
  static const size_t SSIZE=28;
  struct Pair {
    const int first;
    const char second[SSIZE];
  }; 

  static const Pair items[] PROGMEM = {
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
  std::string rv = std::string(val);
  int id = 0;
  if (1 == sscanf(val, "%i", &id)) {
    for (size_t i = 0; i < sizeof(items)/sizeof(items[0]); i++) {
      int key=id;
      memcpy_P(&key, &items[i].first, sizeof(key)); 
      if (key == id) {
        char value[SSIZE];
        strncpy_P(value, items[i].second, sizeof(value));
        rv = std::string(value);
        break;
      }
    }
  }
  return rv;
};

static std::string MODE2name(const char* const val) {
  static const size_t SSIZE=9;
  struct Pair {
    const int first;
    const char second[SSIZE];
  }; 

  static const Pair items[] PROGMEM = {
    {1, "CHARGER"},
    {2, "INVERTER"},
    {4, "OFF"},
    {5, "ECO"},
    {253, "SMART"}
  };

  std::string rv = std::string(val);
  int id = 0;
  if (1 == sscanf(val, "%i", &id)) {
    for (size_t i = 0; i < sizeof(items)/sizeof(items[0]); i++) {
      int key=id;
      memcpy_P(&key, &items[i].first, sizeof(key)); 
      if (key == id) {
        char value[SSIZE];
        strncpy_P(value, items[i].second, sizeof(value));
        rv = std::string(value);
        break;
      }
    }
  }
  return rv;
};

static std::string MPPT2name(const char* const val) {
  static const size_t SSIZE=25;
  struct Pair {
    const int first;
    const char second[SSIZE];
  }; 

  static const Pair items[] PROGMEM = {
    {0, "off"},
    {1, "Voltage or current limit"},
    {2, "MPPT"},
  };

  std::string rv = std::string(val);
  int id = 0;
  if (1 == sscanf(val, "%i", &id)) {
    for (size_t i = 0; i < sizeof(items)/sizeof(items[0]); i++) {
      int key=id;
      memcpy_P(&key, &items[i].first, sizeof(key)); 
      if (key == id) {
        char value[SSIZE];
        strncpy_P(value, items[i].second, sizeof(value));
        rv = std::string(value);
        break;
      }
    }
  }
  return rv;
};

static std::string scale(const char* const value, int divider, int precision)
{
  std::string rv = value;
  int v = 0;
  if (1 == sscanf(value, "%i", &v)) {
    char tmp[20];
    float x = static_cast<float>(v) / divider;
    ext_snprintf_P(tmp, sizeof(tmp), PSTR("%*_f"), precision, &x);
    rv = std::string(tmp);
  }
  return rv;
};

static std::string f001(const char* const value) { return scale(value, 1000, 3); }
static std::string f01(const char* const value) { return scale(value, 100, 2); }
static std::string f1(const char* const value) { return scale(value, 10, 1); }

static std::string pass(const char* const val) { 
  std::string ret(val);
  return ret;
}

static std::string stringEncode(const char* const val) { 
  std::string ret("\"");
  ret += val;
  ret += "\"";
  return ret;
}

struct TupleImpl {
  const char label[10];
  const char description[80];
  std::string (*format_func)(const char* const);
  const int isNumber;
  const char unit[4];
  std::string formattedValue(const std::string& value) const  { 
    if (format_func) return (*format_func)(value.c_str());
    return value;
  }
};
 
static const TupleImpl field_metadata[]  PROGMEM = {
// label   description       transformer isNumber  unit                 
  {"V",    "Channel 1 battery voltage", f001, true, "V"},
  
  {"V2",   "Channel 2 battery voltage", f001,true, "V"},
  {"V3",   "Channel 3 battery voltage", f001,true, "V"},
  {"VS",   "Auxiliary (starter) voltage", f001,true, "V"},
  {"VM",   "Mid-point voltage of the battery bank", f001,true, "V"},
  {"DM",   "Mid-point deviation of the battery bank", pass, true, "%%"},
  {"VPV",  "Panel voltage", f001,true, "V"},
  {"PPV",  "Panel power",  pass, true, "W"},
  {"I",    "Channel 1 battery current",  f001, true, "A"},
  {"I2",   "Channel 2 battery current", f001, true, "A"},
  {"I3",   "Channel 3 battery current",  f001, true, "A"},
  {"IL",   "Load current", f001, true, "A"},
  {"LOAD", "Load output state (ON/OFF)",  pass, false, ""},
  {"T",    "Battery temperature", pass, true, "°C"},
  {"P",    "Instantaneous power", pass, true, "W"},
  {"CE",   "Consumed Amp Hours", f001, true,"Ah"},
  {"SOC",  "State-of-charge", pass, true, "%%"},
  {"TTG",  "Time-to-go",  pass, true, "min"},
  {"Alarm", "Alarm condition active", pass, false, ""},
  {"Relay", "Relay state", pass, false, ""},
  {"AR",    "Alarm reason", pass, false, ""},
  {"OR",    "Off reason", pass, false, ""},
  {"H1",    "Depth of the deepest discharge", f001, true, "Ah"},
  {"H2",    "Depth of the last discharge", f001, true, "Ah"},
  {"H3",    "Depth of the average discharge", f001, true, "Ah"},
  {"H4",    "Number of charge cycles", pass, true, ""},
  {"H5",    "Number of full discharges", pass, true, ""},
  {"H6",    "Cumulative Amp Hours drawn", f001, true, "Ah"},
  {"H7",    "Minimum main (battery) voltage", f001, true, "V"},
  {"H8",    "Maximum main (battery) voltage", f001, true, "V"},
  {"H9",    "Number of seconds since last full charge", pass, true, "s"},
  {"H10",   "Number of automatic synchronizations", pass, true, ""},
  {"H11",   "Number of low main voltage alarms",pass, true, ""},
  {"H12",   "Number of high main voltage alarms", pass, true, ""},
  {"H13",   "Number of low auxiliary voltage alarms", pass, true, ""},
  {"H14",   "Number of high auxiliary voltage alarms", pass, true, ""},
  {"H15",   "Minimum auxiliary (battery) voltage", f001, true,"V"},
  {"H16",   "Maximum auxiliary (battery) voltage", f001, true,"V"},
  {"H17",   "Amount of discharged energy (BMV) / Amount of produced energy (DC monitor)", f01, true,"kWh"},
  {"H18",   "Amount of charged energy (BMV) / Amount of consumed energy (DC monitor)",f01, true, "kWh"}, 
  {"H19",   "Yield total", f01, true, "kWh"}, 
  {"H20",   "Yield today", f01, true, "kWh"}, 
  {"H21",    "Maximum power today", pass, true, "W"},
  {"H22",   "Yield yesterday",  f01, true, "kWh"},
  {"H23",    "Maximum power yesterday", pass,true, "W"},
  {"ERR",    "Error code", pass, true, ""},
  {"CS",     "State of operation", CS2name, false, ""},
  {"BMV",    "Model description (deprecated)", pass, false, ""},
  {"FW",     "Firmware version (16 bit)",  pass, false, ""},
  {"FWE",    "Firmware version (24 bit)",  pass, false, ""},
  {"PID",    "Product ID",  PID2Devicename, false, ""},
  {"SER#",   "Serial number", pass, false, ""},
  {"HSDS",   "Day sequence number (0..364)", pass,true, ""},   
  {"MODE",   "Device mode",  MODE2name, false, ""},
  {"AC_OUT_V","AC output voltage", f01, true, "V"},
  {"AC_OUT_I","AC output current", f1, true, "A"},
  {"AC_OUT_S","AC output apparent power", pass, true, "VA"},
  {"WARN",    "Warning reason",  pass, false, ""}, 
  {"MPPT",    "Tracker operation mode", MPPT2name, false, ""}, 
  {"MON",     "DC monitor mode",  pass, false, ""}
};


class Tuple {
  // these instances live in RAM. they are copied from Flash by the static find() method
  private:
  TupleImpl m_impl;
  public:
  Tuple() : m_impl() {}
  bool isNull() const { return m_impl.description[0]=='\0'; }
  //const char* label() const {return m_impl.label;}
  std::string description(const std::string& key) const {return isNull() ? key : m_impl.description;}
  const char* unit() const {return m_impl.unit;}
  bool isNumber() const {return m_impl.isNumber != 0;}
  std::string formatValueForJson(const std::string& value) const {
      std::string ret = m_impl.formattedValue(value );          
      if (isNumber()) return ret;
      return stringEncode(ret.c_str());
  }
  std::string formatValueForWeb(const std::string& value) const {
      std::string val = m_impl.formattedValue(value );          
      if (strlen(unit())) {
        val += " ";
        val += unit();
      }
      return val;
  }
  static Tuple find(const std::string& label) {
    Tuple temp;
    for (size_t i = 0; i < sizeof(field_metadata)/sizeof(field_metadata[0]); i++) {
      if (0 == strcmp_P(label.c_str(), field_metadata[i].label)) {
        memcpy_P(&(temp.m_impl), &field_metadata[i], sizeof(TupleImpl)); // copy to ram
        return temp;
      }
    }
    // construct an empty 
    return temp;
  }
};



enum class State {
  IN_KEY,
  IN_VALUE,
  IN_CHECKSUM,
  IN_COMMAND,
  IN_HEX,
  UNKNOWN
};

// abstract interface for message blocks (after checksum was verified, without checksum)
class Receiver {
  public:
  // emits a vector of key/ value pairs
  virtual void getText(const VPSS& data)=0;
  // emits Hex messages: cmd is the one nible hex response i.e "A" arg is
  virtual void getHexData(const std::string& cmd, const std::string& arg)=0;
  virtual ~Receiver() {};
};

class Parser {
public:
  Parser() : state(State::UNKNOWN), checksum(0), receiverPtr(0), protocolErrorCount(0) {};
  void parseCharacter(const char c);
  // whenever data gets ready, data is passed to the receiver
  void setReceiver(Receiver* p) { receiverPtr=p; }
private:

  void pushKeyValue();
  void emitHexMessage();
  void emitText();
  State state;
  uint8_t checksum;
  std::string key;
  std::string value;
  std::string hexline;
  VPSS keyValues;
  Receiver* receiverPtr;
  uint32_t protocolErrorCount;
};

class VICTRON : Receiver {
  public:
  VICTRON() : nbytesToday(0), serial(0) {}
  uint32_t nbytesToday = 0; // at 168 bytes/S  32 bit will overflow. who cares about that number?
  TasmotaSerial *serial = NULL;
  VPSS last_msg;
  Parser parser;
  void init(TasmotaSerial*p) { serial = p; last_msg.clear(); parser.setReceiver(this); }
  void readSerialBuffer();
  virtual void getText(const VPSS& data);
  virtual void getHexData(const std::string& cmd, const std::string&arg);
  virtual ~VICTRON() {};
};

// as there is no controlled removal of devices, I don't care about dtors for the pointers 
static std::vector<VICTRON*> devices;

void VICTRON::readSerialBuffer() {
  if (serial == NULL) return;
  int ret=-1;
  auto bytesAvailable = serial->available();
  if (bytesAvailable > 0) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s(): parsing %d bytes...\n"), __FILE__, __func__, bytesAvailable);
  }
  for( int i=0; i<bytesAvailable; ++i ) {
    auto val= serial->read();
    if (val==-1) break; // should never happen because char is available
    nbytesToday++;
    parser.parseCharacter(static_cast<char>(val & 0xff));
  }
}

void VICTRON::getText(const VPSS& data) {
  last_msg=data;
  AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s(): got %d KV-pairs\n"), __FILE__, __func__, (int) last_msg.size());
}

void VICTRON::getHexData(const std::string& cmd, const std::string&arg) {
  HERE();
}
/*
 * ## Message format ##
 *
 * The device transmits blocks of data at 1 second intervals. Each field
 * is sent using the following format:
 *
 * <Newline><Field-Label><Tab><Field-Value>
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

Additionally, if the HEX protocol is activated, then HEX blocks can be intermixed with text protocol.
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

So, I implemented a state machine that discriminates the data stream HEX Blocks and the key-value text blocks
and emit tokens to a given receiver whenever a valid item (either validate text block or Hex Item ) is detected.

**/

void Parser::parseCharacter(const char c) {
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
        // protocol error as keys only have up to 9 chars according protocol definition
        state= State::UNKNOWN;
        protocolErrorCount++;
      }

      key.push_back(c);
    }
    break;

  case State::IN_CHECKSUM:
    checksum+=static_cast<uint8>(c);      
    AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s() checksum: char %d, total %d\n"), __FILE__, __func__, (int)c, (int)checksum);
    // operate the checksum
    if (checksum == 0) {
      // checksum is OK, we can emit all key/value pairs now
      emitText();
    }
     else {
      // bad luck. clear and start again
     // emitText();
      protocolErrorCount++;
    }
    keyValues.clear();
    // this is tricky: the next char may be a : from the next HEX or a carriage return that starts a newtext
    state = State::UNKNOWN;
    checksum=0;
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
      if (hexline.size() >75) {
        hexline.clear();
        // protocol error, longest hex payload is 34 byte (History day payload) -->68+1+4+1+one small egg, according protocol definition
        state= State::UNKNOWN;
        protocolErrorCount++;
      }
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
        protocolErrorCount++;
      }
      else {
        value.push_back(c);
      }
    }
    break;

  }
};
void Parser::pushKeyValue() {
  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("%s:%s(): gotKeyValue %s: %s\n"), __FILE__, __func__, key.c_str(), value.c_str());
  keyValues.push_back(PSS(key, value));
}

void Parser::emitHexMessage() {
  AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s(): gotHexMsg %s\n"), __FILE__, __func__, hexline.c_str());
  keyValues.push_back(PSS(key, value));
}

void Parser::emitText() {
  if (receiverPtr) receiverPtr->getText(keyValues);
}

static void Victron250ms() {               
  for( auto v : devices) {
    v->readSerialBuffer();
  }
}   

static void VictronInit() {
  for (uint32_t index = 0; index < MAX_VICTRON_VEDIRECT_DEVICES; index++) {
    if (PinUsed(GPIO_VICTRON_VEDIRECT_RX, index) ) {
      auto pinRX=Pin(GPIO_VICTRON_VEDIRECT_RX, index);
      auto pinTX=Pin(GPIO_VICTRON_VEDIRECT_TX, index); // may be not assigned=-1, then no command interface
      AddLog(LOG_LEVEL_INFO, PSTR("%s:%s():%d Pin(GPIO_VICTRON_VEDIRECT_RX, %d): %d, Pin(GPIO_VICTRON_VEDIRECT_TX,%d): %d\n"), 
            __FILE__, __func__, __LINE__, index, (int)pinRX, index, (int)pinTX);
      TasmotaSerial* sp = new TasmotaSerial(pinRX, pinTX, 1 /* hw fallback */, 0 /* nwmode, 0 default */, VICTRON_MSG_SIZE /* buf size */);
      if (sp == NULL) continue;
      if (sp->begin(VICTRON_BAUTRATE)) { // 480 bytes / 250ms
        if (sp->hardwareSerial()) {
          ClaimSerial();
          AddLog(LOG_LEVEL_INFO, PSTR("%s: hwserial claimed\n"), __func__);
        }
      }
    // now we have a valid serial device that we can give the first instance
      VICTRON* v = new VICTRON;
      if (v) {
        v->init(sp);
        devices.push_back(v);
        AddLog(LOG_LEVEL_INFO, PSTR("%s: Initialized %d with RX buffer %d to instance %d\n"),
         __func__, (int)index, (int)sp->getRxBufferSize(), (int)devices.size());

      }
    }
  }
}

/* Command: <device>,<Cmd>,<Args>
  device 1..4 1 for first device, 2 for second device...
  Cmd: 'P': send Hex-Ping
  Cmd: 'S': Set Register, Args see below
  Cmd: 'G': Get Register Args see below
  Cmd: 'A': request Asyncronious Register, Args see below
  Args for S,G,A: <Register>,<Flags>[,valueFormat[,registerValue]]
  Register: hexCode of address register, i.e '0102' or 'EEF8' always 4 hex symbols [0..1,A..F]
  Flags:
  valueFormat: describes the value format. if not given, value will be shown / intepreted as hex-digits and must hav korrect length for the given
               register. As there are a huge amount of different registers for different devices, this interface does not know them. So you
               need to supply the correct values, otherwise the Hex-Message will be ill-formed and lead to an error.
               Known Formats: un8,un16,un24,un32,sn16,sn32, String32,String20. If not given, the value will be guessed from the received number of bytes,
               if not unique (i.e un16 or sn16) then the signed type will be choosen. Value will be displayed in decimal, except for String32 and String20
  
*/
static const char VICTRON_CMND_START[] PROGMEM = "{\"" D_CMND_SENSOR "%d\":{\"cmd\":\"%s\"";
static const char VICTRON_CMND_END[] PROGMEM = "}}";

bool VictronCommand(){

 int argc = ArgC();
  if(argc <2) {
    return false;
  }
  char argument[XdrvMailbox.data_len];
  char cmd[XdrvMailbox.data_len];
  char arg1[XdrvMailbox.data_len];

  uint32_t device = atoi(ArgV(argument,1));
  ArgV(cmd, 2);
  ArgV(arg1, 2);
  AddLog(LOG_LEVEL_DEBUG, PSTR("cmd:%d,%s,%s"), (int)device, cmd, arg1); 
/* 
  TO BE CONTINUED
  if (device >  devices.size()) {
    AddLog(LOG_LEVEL_ERROR, PSTR("%s:%s(): requested device %d, but have only %d\n"), __FILE__, __func__, (int)device, (int)devices.size());
    return false;
  } 
  Response_P(VICTRON_CMND_START, XSNS_107, device, cmd);
  //    ResponseAppend_P(PSTR("%s%c"), argument, ((channel < (INA3221_NB_CHAN-1))?',':'\0'));
  ResponseAppend_P(VICTRON_CMND_END);
  */
  return true;
}

static void VictronShowJSON() {

  if (devices.empty()) return;
  int devId =0;
  auto t0 = millis();
  for ( const auto v: devices) {
    devId++;
    ResponseAppend_P(PSTR(",\"sensor%d\":{" ), devId);
    for (const auto kvp: v->last_msg) {
      Tuple t = Tuple::find(kvp.first);
      std::string val = t.formatValueForJson(kvp.second);     
      ResponseAppend_P(PSTR("\"%s\": %s,"), kvp.first.c_str(), val.c_str());
    }
    ResponseAppend_P(PSTR("\"bytes_read\": %lld}"), (long long)v->nbytesToday);    
  }
  auto t1 = millis();
  auto delta = t1 - t0;
  ResponseAppend_P(PSTR(",\"MetaTS\": {\"Start_TS\": %lld, \"End_TS\": %lld, \"Delta\": %lld}"), (long long)t0, (long long)t1, (long long)delta);
}

#ifdef USE_WEBSERVER
static void VictronShowWeb() {
  char tmpbuf[20] = {0};
  int idx=0;
  for (const auto v: devices) { 
    if (idx > 0) { // not before first
      WSContentSend_P(PSTR("<hr>\n"));
    }
    idx++;
    AddLog(LOG_LEVEL_DEBUG, PSTR("%s:%s():%d idx=%d"), __FILE__, __func__, __LINE__, (int)idx);
    for (const auto  kvp : v->last_msg) {
      Tuple t = Tuple::find(kvp.first);
      std::string desc = t.description(kvp.first); // fallback to lable for unknown keys
      std::string val = t.formatValueForWeb(kvp.second);
      WSContentSend_PD(PSTR("<tr><th>%s</th><td>%s</td></tr>\n"), desc.c_str(), val.c_str());
    }
    snprintf(tmpbuf, sizeof(tmpbuf), "%lld", (long long)v->nbytesToday);
    WSContentSend_P(PSTR("<tr><th>%s</th><td>%s</td></tr>\n"), "Received bytes", tmpbuf);
  }
}
#endif // USE_WEBSERVER

static void VicronAtMidnight() {
  for( auto v: devices) {
    v->nbytesToday=0;   
  }
}

bool Xsns107(uint32_t function) {
  bool result = false;
  if (FUNC_INIT == function) {
    VictronInit();
  }
  else if (!devices.empty()) {
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
      case FUNC_COMMAND_SENSOR:
        if (XSNS_107 == XdrvMailbox.index) {
          return VictronCommand();  // Return true on success
        }
        break;

    }
  }
  return result;
}
#endif  // USE_VICTRON
