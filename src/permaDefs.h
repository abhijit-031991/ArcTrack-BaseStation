////// Device Details //////

#define STATIONID 101
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

