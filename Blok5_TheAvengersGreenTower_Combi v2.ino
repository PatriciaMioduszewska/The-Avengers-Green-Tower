#include <lmic.h>
#include <hal/hal.h>

#include <SoftwareWire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "DFRobot_PH.h"

// Wesley
// static const PROGMEM u1_t NWKSKEY[16] = { 0x53, 0xF0, 0x6C, 0x3E, 0x1C, 0xCF, 0xB2, 0x1C, 0x88, 0xE2, 0x01, 0x93, 0xAE, 0x9C, 0x3A, 0xA4 };
// static const u1_t PROGMEM APPSKEY[16] = { 0xAE, 0x11, 0xE3, 0x86, 0x66, 0x77, 0x00, 0xF8, 0x59, 0xE1, 0x98, 0xCC, 0x07, 0x45, 0x90, 0x6C };
// static const u4_t DEVADDR = 0x260B0EF6;

// Patrycja
// static const PROGMEM u1_t NWKSKEY[16] = { 0x57, 0x95, 0xC1, 0x94, 0xB4, 0xA9, 0x94, 0xEE, 0x99, 0x9D, 0xB1, 0xD3, 0x66, 0xD8, 0x0F, 0x49 };
// static const u1_t PROGMEM APPSKEY[16] = { 0x0B, 0xE3, 0xF7, 0x19, 0xC6, 0x4B, 0x33, 0x77, 0xB6, 0x87, 0xE7, 0xA0, 0xE0, 0x7E, 0xB0, 0x54 };
// static const u4_t DEVADDR = 0x260BDE78;

// Cas
// static const PROGMEM u1_t NWKSKEY[16] = { 0x65, 0xE2, 0xF1, 0xCA, 0x3F, 0x86, 0xBF, 0xF4, 0x5A, 0xF6, 0x29, 0x3E, 0xAF, 0x8A, 0x8C, 0xBE };
// static const u1_t PROGMEM APPSKEY[16] = { 0x91, 0x40, 0xE3, 0xD5, 0x2F, 0x08, 0x16, 0x86, 0x9F, 0x80, 0xEB, 0x05, 0x1A, 0xD2, 0x6B, 0x91 };
// static const u4_t DEVADDR = 0x260BA12A;

// Ilana
// static const PROGMEM u1_t NWKSKEY[16] = { 0x9D, 0xC2, 0x05, 0x0C, 0x0E, 0x24, 0x0D, 0x20, 0x8B, 0xBC, 0x70, 0x28, 0xA7, 0x6F, 0x27, 0xB5 };
// static const u1_t PROGMEM APPSKEY[16] = { 0x79, 0x5F, 0x5C, 0xFB, 0x52, 0x43, 0x4E, 0x06, 0x6D, 0x9F, 0x95, 0xAD, 0xD5, 0x9E, 0x62, 0x2E };
// static const u4_t DEVADDR = 0x260B8EF5;

// Combi
static const PROGMEM u1_t NWKSKEY[16] = { 0x30, 0xCA, 0xEE, 0x1B, 0x90, 0xE9, 0xD1, 0x8B, 0xD2, 0xCF, 0xEF, 0x7B, 0x9B, 0x32, 0x0D, 0x4C };
static const u1_t PROGMEM APPSKEY[16] = { 0x31, 0x0A, 0xC4, 0xE8, 0x5C, 0xFF, 0x71, 0xF9, 0x6F, 0xB6, 0x3D, 0x1E, 0xE2, 0xF3, 0x9C, 0xEA };
static const u4_t DEVADDR = 0x260BD3A8;

void os_getArtEui(u1_t* buf) {}
void os_getDevEui(u1_t* buf) {}
void os_getDevKey(u1_t* buf) {}

const unsigned TX_INTERVAL = 120;
static osjob_t sendjob;

//------------------------------------------------------------------------------------------------------------------------------

// Lora pin definitions
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = { 2, 6, 7 },
};

// pin definitions
#define TEMP_1_PIN 4
#define TEMP_2_PIN 5

#define PH_PIN A0

#define WATER_SDA A2
#define WATER_SCL A1

//#define LIGHT_SDA A4 // Licht library zet het standaard al op A4 en A5
//#define LIGHT_SCL A5 // De water sensor ook maar ik kan geen 2 sensors hebben op dezelfde pinnen

// sensor communicatie met arduino
#define SEN0562_ADDR 0x23 // licht sensor
#define ATTINY1_HIGH_ADDR 0x78 // water sensors
#define ATTINY2_LOW_ADDR 0x77

// LPP's
#define LPP_DIGITAL_OUTPUT 0x01 // later code voor pomp nodig
#define LPP_LIGHT 0x98
#define LPP_TEMPERATURE 0x67 // 6 7 <---
#define LPP_PERCENTAGE 0x78
#define LPP_PH 0x9D

// channels
#define CHANNEL_LIGHT 1
#define CHANNEL_PH 2
#define CHANNEL_WATER 3
#define CHANNEL_TEMP_1 4
#define CHANNEL_TEMP_2 5
#define CHANNEL_PUMP 6

//------------------------------------------------------------------------------------------------------------------------------

// library communicatie met sensor pinnen
SoftwareWire waterWire(WATER_SDA, WATER_SCL);

OneWire oneWire1(TEMP_1_PIN);
OneWire oneWire2(TEMP_2_PIN);

DallasTemperature temperatuur1(&oneWire1);
DallasTemperature temperatuur2(&oneWire2);

DFRobot_PH ph;

// default values
uint8_t low_data[8] = { 0 };
uint8_t high_data[12] = { 0 };
uint8_t waterLevel = 0;

float voltage = 0.0;
float phValue = 7.0; // 7 is neutraal voor de ph sensor

uint16_t lightValue = 0;

// NaN omdat de temperatuur ook 0 kan zijn, coole plant
float temp1 = NAN;
float temp2 = NAN;

byte payload[40] = { 0 };
uint8_t cursor = 0;

void getLow8SectionValue();
void getHigh12SectionValue();
uint8_t getWaterLevel();
uint16_t readLightSensor();
void readTemperatureSensors();
void readPHSensor();

void do_send(osjob_t* j);

//------------------------------------------------------------------------------------------------------------------------------

// Waterlevel sensor
void getLow8SectionValue() {
  memset(low_data, 0, sizeof(low_data));

  waterWire.requestFrom(ATTINY2_LOW_ADDR, 8);

  if (waterWire.available() == 8) {
    for (int i = 0; i < 8; i++) {
      low_data[i] = waterWire.read();
    }
  }

  delay(10);
}

void getHigh12SectionValue() {
  memset(high_data, 0, sizeof(high_data));

  waterWire.requestFrom(ATTINY1_HIGH_ADDR, 12);

  if (waterWire.available() == 12) {
    for (int i = 0; i < 12; i++) {
      high_data[i] = waterWire.read();
    }
  }

  delay(10);
}

uint8_t getWaterLevel() {
  uint32_t touch_val = 0;

  getLow8SectionValue();
  for (int i = 0; i < 8; i++) {
    if (low_data[i] > 100) { // 100%
      touch_val |= (uint32_t)1 << i;
    }
  }
  getHigh12SectionValue();
  for (int i = 0; i < 12; i++) {
    if (high_data[i] > 100) { // 100%
      touch_val |= (uint32_t)1 << (8 + i);
    }
  }

  uint8_t trig_section = 0;

  while (touch_val & 0x01) {
    trig_section++;
    touch_val >>= 1;
  }

  return trig_section * 5;
}

// Licht sensor
uint16_t readLightSensor() {
  uint8_t buf[4] = { 0 };

  Wire.beginTransmission(SEN0562_ADDR);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)SEN0562_ADDR, (uint8_t)2);
  uint8_t i = 0;

  while (Wire.available() && i < 2) {
    buf[i++] = Wire.read();
  }

  uint16_t data = (buf[0] << 8) | buf[1];

  float lux = ((float)data) / 1.2;

  return (uint16_t)lux;
}

// Temperatuur sensor
void readTemperatureSensors() {
  temperatuur1.requestTemperatures();
  temperatuur2.requestTemperatures();

  if (temperatuur1.getDeviceCount() > 0) {
    temp1 = temperatuur1.getTempCByIndex(0);
  } else {
    temp1 = NAN;
  }

  if (temperatuur2.getDeviceCount() > 0) {
    temp2 = temperatuur2.getTempCByIndex(0);
  } else {
    temp2 = NAN;
  }
}

// pH sensor
void readPHSensor() {
  analogRead(PH_PIN);
  delay(10);

  int rawADC = 0;

  for (int i = 0; i < 10; i++) {
    rawADC += analogRead(PH_PIN);
    delay(10);
  }

  rawADC /= 10;

  voltage = (rawADC / 1024.0) * 5000.0;

  phValue = ph.readPH(voltage, 25.0);
}

//------------------------------------------------------------------------------------------------------------------------------

void onEvent(ev_t ev) {
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

//------------------------------------------------------------------------------------------------------------------------------

void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Licht
    lightValue = readLightSensor();
    Serial.print(F("Licht sterkte: "));
    Serial.print(lightValue);
    Serial.println(F(" lux"));

    // pH
    readPHSensor();
    Serial.print(F("pH: "));
    Serial.println(phValue, 2);

    // Waterniveau
    waterLevel = getWaterLevel();
    Serial.print(F("Water hoogte op: "));
    Serial.print(waterLevel);
    Serial.println(F("%"));

    // Temperatuur
    readTemperatureSensors();
    Serial.print(F("Temperatuur 1: "));
    if (isnan(temp1)) {
      Serial.println(F("Niet gevonden"));
    } else {
      Serial.print(temp1, 2);
      Serial.println(F(" C"));
    }
    Serial.print(F("Temperatuur 2: "));
    if (isnan(temp2)) {
      Serial.println(F("Niet gevonden"));
    } else {
      Serial.print(temp2, 2);
      Serial.println(F(" C"));
    }

    cursor = 0;
    memset(payload, 0, sizeof(payload));

    // Payload (0x00s) om te sturen naar The Things Network decoder te sturen
    int16_t lightEncoded = lightValue;
    payload[cursor++] = CHANNEL_LIGHT;
    payload[cursor++] = LPP_LIGHT;
    payload[cursor++] = highByte(lightEncoded);
    payload[cursor++] = lowByte(lightEncoded);

    uint8_t phEncoded = (uint8_t)(phValue * 10);
    payload[cursor++] = CHANNEL_PH;
    payload[cursor++] = LPP_PH;
    payload[cursor++] = phEncoded;

    uint8_t waterEncoded = waterLevel;
    payload[cursor++] = CHANNEL_WATER;
    payload[cursor++] = LPP_PERCENTAGE;
    payload[cursor++] = waterEncoded;

    // temp sensor gaat snel kaduuk dus kapotte sensor data veranderen naar iets leesbaars (volg de isnan)
    int16_t temp1Encoded;
    if (isnan(temp1)) {
      temp1Encoded = -1270;   // -127.0 = sensor niet gevonden
    } else {
      temp1Encoded = (int16_t)(temp1 * 10);
    }
    payload[cursor++] = CHANNEL_TEMP_1;
    payload[cursor++] = LPP_TEMPERATURE;
    payload[cursor++] = highByte(temp1Encoded);
    payload[cursor++] = lowByte(temp1Encoded);


    int16_t temp2Encoded;
    if (isnan(temp2)) {
      temp2Encoded = -1270;   // -127.0 = sensor niet gevonden
    } else {
      temp2Encoded = (int16_t)(temp2 * 10);
    }
    payload[cursor++] = CHANNEL_TEMP_2;
    payload[cursor++] = LPP_TEMPERATURE;
    payload[cursor++] = highByte(temp2Encoded);
    payload[cursor++] = lowByte(temp2Encoded);

    payload[cursor++] = CHANNEL_PUMP;
    payload[cursor++] = LPP_DIGITAL_OUTPUT;
    payload[cursor++] = 0x00;

    // laat de volledige byte package zien in println om te kijken of alle sensor data wel verstuurd word
    Serial.print("Payload bytes: ");

    for (int i = 0; i < cursor; i++) {
      Serial.print(payload[i], HEX);
      Serial.print(" ");
    }

    Serial.println();

    LMIC_setTxData2(1, payload, cursor, 0);

    Serial.println(F("Packet queued"));
  }
}

//------------------------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting"));

  // Start sensoren
  waterWire.begin();
  Wire.begin();
  temperatuur1.begin();
  temperatuur2.begin();

  os_init();
  LMIC_reset();

#ifdef PROGMEM
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];

  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));

  LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#else
  LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

#elif defined(CFG_us915)
  LMIC_selectSubBand(1);
#endif
  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7, 14);

  os_setCallback(&sendjob, do_send);
}

//------------------------------------------------------------------------------------------------------------------------------

void loop()
{
    os_runloop_once();
}