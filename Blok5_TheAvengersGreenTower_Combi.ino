#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <SoftwareWire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
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
// static const PROGMEM u1_t NWKSKEY[16] = {  };
// static const u1_t PROGMEM APPSKEY[16] = {  };
// static const u4_t DEVADDR = 0x ;

void os_getArtEui(u1_t* buf) {}
void os_getDevEui(u1_t* buf) {}
void os_getDevKey(u1_t* buf) {}

const unsigned TX_INTERVAL = 15;
static osjob_t sendjob;

// ------------------------------------------------------------------

// Lora pin definitions
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = { 2, 6, 7 },
};

// pin definitions
#define TEMP_PIN_1 4
#define TEMP_PIN_2 5

#define PH_PIN A0

#define WATER_SDA A2
#define WATER_SCL A1

#define LIGHT_SDA A4
#define LIGHT_SCL A5

// low power payloads
#define SEN0562_ADDR 0x23

#define ATTINY1_HIGH_ADDR 0x78
#define ATTINY2_LOW_ADDR 0x77
#define WATER_THRESHOLD 100

#define LPP_DIGITAL_OUTPUT 0x01
#define LPP_ANALOG_INPUT 0x02
#define LPP_LUMINOSITY 0x65
#define LPP_TEMPERATURE 0x67

// channel definitions
#define CHANNEL_LIGHT 1
#define CHANNEL_PH 2
#define CHANNEL_WATER 3
#define CHANNEL_TEMP_1 4
#define CHANNEL_TEMP_2 5
#define CHANNEL_PUMP 6

// -------------------------------------------------------------------

// library ....
SoftwareWire waterWire(WATER_SDA, WATER_SCL);

OneWire oneWire1(TEMP_PIN_1);
OneWire oneWire2(TEMP_PIN_2);

DallasTemperature sensor1(&oneWire1);
DallasTemperature sensor2(&oneWire2);

DFRobot_PH ph;

// default values
unsigned char low_data[8] = { 0 };
unsigned char high_data[12] = { 0 };

uint8_t waterLevel = 0;

float voltage = 0.0;
float phValue = 7.0;

uint16_t lightValue = 0;

float temp1 = 0.0;
float temp2 = 0.0;

byte payload[40] = { 0 };
uint8_t cursor = 0;

void getLow8SectionValue();
void getHigh12SectionValue();
uint8_t getWaterLevel();
uint16_t readLightSensor();
void readTemperatureSensors();
void readPHSensor();
void do_send(osjob_t* j);

//--------------------------------------------------------

// Waterlevel sensor ....
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
  getHigh12SectionValue();

  for (int i = 0; i < 8; i++) {
    if (low_data[i] > WATER_THRESHOLD) {
      touch_val |= (uint32_t)1 << i;
    }
  }

  for (int i = 0; i < 12; i++) {
    if (high_data[i] > WATER_THRESHOLD) {
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

// Licht sensor ....
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

// Temperatuur sensor ...
void readTemperatureSensors() {
  sensor1.requestTemperatures();
  sensor2.requestTemperatures();

  temp1 = sensor1.getTempCByIndex(0);
  temp2 = sensor2.getTempCByIndex(0);

}

// pH sensor ...
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

  Serial.print("pH: ");
  Serial.println(phValue, 2);
}

//-----------------------------------------------------------------

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

// -----------------------------------------------------------------

void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    ///////////////////////////////////////////////
    Serial.println(F("=== SENSOR READ START ==="));

    Serial.println(F("Reading light sensor..."));
    lightValue = readLightSensor();
    Serial.print(F("Light sensor OK: "));
    Serial.print(lightValue);
    Serial.println(F(" lx"));

    Serial.println(F("Reading pH sensor..."));
    readPHSensor();
    Serial.print(F("pH sensor OK: "));
    Serial.println(phValue, 2);

    Serial.println(F("Reading water level..."));
    waterLevel = getWaterLevel();
    Serial.print(F("Water level OK: "));
    Serial.print(waterLevel);
    Serial.println(F("%"));

    Serial.println(F("Reading temperature sensors..."));
    readTemperatureSensors();

    Serial.print(F("Temperature 1: "));
    Serial.print(temp1);
    Serial.println(F(" C"));

    Serial.print(F("Temperature 2: "));
    Serial.print(temp2);
    Serial.println(F(" C"));

    Serial.println(F("=== SENSOR READ END ==="));

    ////////////////////////////////////////////////////
    cursor = 0;

    memset(payload, 0, sizeof(payload));  ///////////////////////

    int16_t lightEncoded = (int16_t)lightValue;
    payload[cursor++] = CHANNEL_LIGHT;
    payload[cursor++] = LPP_LUMINOSITY;
    payload[cursor++] = highByte(lightEncoded);
    payload[cursor++] = lowByte(lightEncoded);

    int16_t phEncoded = (int16_t)(phValue * 100);
    payload[cursor++] = CHANNEL_PH;
    payload[cursor++] = LPP_ANALOG_INPUT;
    payload[cursor++] = highByte(phEncoded);
    payload[cursor++] = lowByte(phEncoded);

    int16_t waterEncoded = (int16_t)waterLevel;
    payload[cursor++] = CHANNEL_WATER;
    payload[cursor++] = LPP_ANALOG_INPUT;
    payload[cursor++] = highByte(waterEncoded);
    payload[cursor++] = lowByte(waterEncoded);

    int16_t temp1Encoded = (int16_t)(temp1 * 10);
    payload[cursor++] = CHANNEL_TEMP_1;
    payload[cursor++] = LPP_TEMPERATURE;
    payload[cursor++] = highByte(temp1Encoded);
    payload[cursor++] = lowByte(temp1Encoded);

    int16_t temp2Encoded = (int16_t)(temp2 * 10);
    payload[cursor++] = CHANNEL_TEMP_2;
    payload[cursor++] = LPP_TEMPERATURE;
    payload[cursor++] = highByte(temp2Encoded);
    payload[cursor++] = lowByte(temp2Encoded);

    //
    payload[cursor++] = CHANNEL_PUMP;
    payload[cursor++] = LPP_DIGITAL_OUTPUT;
    payload[cursor++] = 0x00;

    LMIC_setTxData2(1, payload, cursor, 0);

    Serial.println(F("Packet queued"));
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting"));

  waterWire.begin();
  Wire.begin();
  ph.begin();
  sensor1.begin();  //////////////////////////
  sensor2.begin();  ///////////////////////////

  int count1 = sensor1.getDeviceCount();/////////////////////
  int count2 = sensor2.getDeviceCount();///////////////////////

  // Print beschikbare temp sonsors
  int outputNumber;
  if (count1 > 0 && count2 > 0) {
    outputNumber = 2;
  } else if (count1 > 0 || count2 > 0) {
    outputNumber = 1;
  } else {
    outputNumber = 0;
  }
  Serial.print("Aantal temperatuur sensoren: ");
  Serial.println(outputNumber);


#ifdef VCC_ENABLE
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  os_init();

  LMIC_reset();

#ifdef PROGMEM
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];

  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));//////////////
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));//////////////

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

  do_send(&sendjob);
}

void loop()
{
    os_runloop_once();
}