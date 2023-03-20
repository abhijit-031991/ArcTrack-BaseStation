////// Device Details //////

const char* StationID = "101";
const int STID = 101;

#define STATIONVERSION 1

///// Firmware Version /////

const float firmwareVersion = 1.0;

///// Pin Definitions /////

#define SCK     18   // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    23   // GPIO27 -- SX1278's MOSI
#define SS      5    // GPIO18 -- SX1278's CS
#define RST     21   // GPIO14 -- SX1278's RESET
#define DI0     22   // GPIO26 -- SX1278's IRQ(Interrupt Request)

#define SIMCTL  15   // GPIO15 -- SIM800C PWRKEY 
#define SIMRX   16   // GPIO16 -- SIM800C RX PIN
#define SIMTX   17   // GPIO17 -- SIM800C TX PIN   

#define BTLED   12   // GPIO12 -- ESP32 LED PIN
#define BTNAME  "BASE-STATION"

//GPRS credentials //
const char apn[]      = "airtelgprs.com";
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT Credentials //
const char* broker = "65.1.242.158";
const char* fileBroker = "65.1.242.158/deviceTelemetry";
const char* telemetryTopic = "stationTelemetry";
const char* statusSubTopic = "status";
const char* settingsSupTopic = "settings";

// HTTP Credentials //
const int httpPort = 1880;
const int chunkSize = 512;

struct ping{
    uint16_t ta;    
    uint16_t cnt;
    float la;
    float ln;
    uint8_t devtyp;
    bool mortality;
  }__attribute__((__packed__)); 

struct hb{
    uint32_t cnt;
    int stid;
    int csq;
    int bat; 
  }__attribute__((__packed__)); 

  struct reqPing{
    uint16_t tag;
    byte request;
  }__attribute__((__packed__));

struct resPing{
    uint16_t tag;
    byte resp;
  }__attribute__((__packed__));