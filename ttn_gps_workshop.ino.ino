/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example sends a valid LoRaWAN packet with payload "Hello,
   world!", using frequency and encryption settings matching those of
   the The Things Network.

   This uses ABP (Activation-by-personalisation), where a DevAddr and
   Session keys are preconfigured (unlike OTAA, where a DevEUI and
   application key is configured, while the DevAddr and session keys are
   assigned/generated in the over-the-air-activation procedure).

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate a DevAddr, NwkSKey and
   AppSKey. Each device should have their own unique values for these
   fields.

   Do not forget to define the radio type correctly in config.h.

 *******************************************************************************/
/*
   use sample Code from JP Meijers
   (https://github.com/jpmeijers/RN2483-Arduino-Library/tree/master/examples/TheThingsUno-GPSshield-TTN-Mapper-binary)
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on pins 8(rx) and 9(tx).
   Format is compatible with 'ttnmapper.org', see website to add your mapper

*/
/*code adapted by F. Beks for TTN Workshops in the Eindhoven IoT community */
/*
 * Payload function for the Things Network:
 * 
  function Decoder(b, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var lat = ((b[0]<<16)>>>0) + ((b[1]<<8)>>>0) + b[2];
  lat = (lat / 16777215.0 * 180) - 90;
  var lon = ((b[3]<<16)>>>0) + ((b[4]<<8)>>>0) + b[5];
  lon = (lon / 16777215.0 * 360) - 180;
  var altValue = ((b[6]<<8)>>>0) + b[7];
  var sign = b[6] & (1 << 7);
  var alt
  if(sign)
  {
    alt = 0xFFFF0000 | altValue;
  }
  else
  {
    alt = altValue;
  }
var hdop = b[8] / 10.0;
var airpressure = 970+((b[9] >> 2) & 0x3F);
var temperature = (-2400+6.25*(((b[10] & 0x03) << 8) | b[9]))/100.0; 
return {
lat: lat,
lon: lon,
alt: alt,
hdop:hdop,
airpressure: airpressure,
temperature: temperature
};
}
*/

#include <Arduino.h>

#include "lmic.h"
#include "hal/hal.h"
#include <SPI.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "i2c.h"
#include "i2c_BMP280.h"
BMP280 bmp280;

static const int RXPin = 8, TXPin = 9;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;
#define PMTK_SET_NMEA_UPDATE_05HZ  "$PMTK220,2000*1C"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
//Fill in your ABP credentials below, MSB for NWSKEY and APPSKEY, numeric / hex format for the Devaddr

static const PROGMEM u1_t NWKSKEY[16] = {};
static const PROGMEM u1_t APPSKEY[16] = { };
static const u4_t DEVADDR =  0x0; // <-- Change this address for every node!

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

uint8_t txBuffer[10];
uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
static osjob_t sendjob;
boolean BMP_Available;  //TRUE if BMP280 sensor found

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 55;

// Pin mapping is hardware specific.
// Pin mapping Doug Larue PCB
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,
  .dio = {4, 5, 7},
};
void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.day());
    Serial.print(F("-"));
    Serial.print(gps.date.month());
    Serial.print(F("-"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
void get_coords () {
  bool newData = false;
  unsigned long age;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (ss.available()) {
      char c = ss.read();
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) { // Did a new valid sentence come in?
        newData = true;
      }
    }
  }
  for(int i=0;i<9;i++)
    txBuffer[i]=0;
   
  //Serial.println(gps.location.age());
  if ( newData && (gps.location.age()<1000)) {
    build_packet();
  }
  displayInfo();
  
}
void get_temp()
{
  float temperature, pascal;
  uint16_t t_value, p_value, s_value;
  bmp280.awaitMeasurement();
  bmp280.getTemperature(temperature);
  bmp280.getPressure(pascal);
  bmp280.triggerMeasurement();
  pascal = pascal / 100;
  Serial.print(F("Pressure: "));
  Serial.print(pascal);
  Serial.print(F(" Pa; T: "));
  Serial.print(temperature);
  Serial.println(F(" C"));

  // getting sensor values

  temperature = constrain(temperature, -24, 40); //temp in range -24 to 40 (64 steps)
  pascal = constrain(pascal, 970, 1034); //pressure in range 970 to 1034 (64 steps)*/
  t_value = int16_t((temperature * (100 / 6.25) + 2400 / 6.25)); //0.0625 degree steps with offset
  // no negative values
  p_value = int16_t((pascal - 970) / 1); //1 mbar steps, offset 970.
  s_value = (p_value << 10) + t_value; // putting the bits in the right place
  txBuffer[9] = s_value & 0xFF; //lower byte
  txBuffer[10] = s_value >> 8; //higher byte
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    get_coords();
    if (BMP_Available)
      get_temp();
    LMIC_setTxData2(1, (uint8_t*) txBuffer, sizeof(txBuffer)+1, 0);
    Serial.print(F("Packet queued, size: "));
    Serial.println(sizeof(txBuffer)+1);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("Starting GPS mapper"));
  Serial.println( "Compiled: " __DATE__ ", " __TIME__);
  Serial.println(F("LMIC settings:"));
#ifdef DISABLE_PING
  Serial.println(F("- ping disabled"));
#else
  Serial.println(F("- ping enabled"));
#endif
#ifdef DISABLE_BEACONS
  Serial.println(F("- beacon disabled"));
#else
  Serial.println(F("- beacon enabled"));
#endif
#ifdef DISABLE_JOIN
  Serial.println(F("- join disabled"));
#else
  Serial.println(F("- join enabled"));
#endif
#ifdef LMIC_FAILURE_TO
  Serial.println(F("- LMIC_FAILURE TO enabled"));
#else
  Serial.println(F("- LMIC_FAILURE_TO disabled"));
#endif
#ifdef LMIC_PRINTF_TO
  Serial.println(F("- LMIC_PRINTF TO enabled"));
#else
  Serial.println(F("- LMIC_PRINTF_TO disabled"));
#endif
  Serial.print(F("Probe BMP280: "));
  BMP_Available = bmp280.initialize();
  if (BMP_Available)
  {
    Serial.println(F("found"));
    // onetime-measure:
    bmp280.setEnabled(0);
    bmp280.triggerMeasurement();
  }
  else
  {
    Serial.println(F("missing"));
    txBuffer[9] = 0;
    txBuffer[10] = 0;
  }

  ss.begin(GPSBaud);
  ss.println(F(PMTK_SET_NMEA_OUTPUT_RMCGGA));
  ss.println(F(PMTK_SET_NMEA_UPDATE_1HZ));   // 1 Hz update rate

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}

void build_packet()
{
  LatitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
  LongitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;

  txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;

  txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;

  altitudeGps = gps.altitude.meters();
  txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  hdopGps = gps.hdop.value() / 10;
  txBuffer[8] = hdopGps & 0xFF;

}

