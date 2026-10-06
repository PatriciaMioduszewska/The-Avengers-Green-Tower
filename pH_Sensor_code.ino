#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <EEPROM.h>
#include <LowPower.h>
#include "DFRobot_PH.h"

// --- Hardware Pins ---
#define PH_PIN A0       // pH analog input
#define PH_POWER_PIN 5  // Digital pin to switch pH sensor VCC

// --- Cayenne LPP Types ---
#define LPP_ANALOG_INPUT 0x02

// --- Interval Configuration ---
// Sleep duration between transmissions (seconds)
const unsigned long SLEEP_SECONDS = 30;

DFRobot_PH ph;
float voltage = 0.0;
float phValue = 7.0;

// Flag to coordinate sleep with LMIC execution
volatile bool transmissionComplete = false;

// --- LoRaWAN credentials (ABP) ---
// Keys and DevAddr must match TTN in MSB (Big-Endian) format
static const PROGMEM u1_t NWKSKEY[16] = { 0x57, 0x95, 0xC1, 0x94, 0xB4, 0xA9, 0x94, 0xEE, 0x99, 0x9D, 0xB1, 0xD3, 0x66, 0xD8, 0x0F, 0x49 };
static const u1_t PROGMEM APPSKEY[16] = { 0x0B, 0xE3, 0xF7, 0x19, 0xC6, 0x4B, 0x33, 0x77, 0xB6, 0x87, 0xE7, 0xA0, 0xE0, 0x7E, 0xB0, 0x54 };
static const u4_t DEVADDR = 0x260BDE78;

void os_getArtEui(u1_t* buf) {}
void os_getDevEui(u1_t* buf) {}
void os_getDevKey(u1_t* buf) {}

static osjob_t sendjob;

const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = { 2, 6, 7 },
};

void do_send(osjob_t* j);

void onEvent(ev_t ev) {
  switch (ev) {
    case EV_TXCOMPLETE:
      Serial.println(F("TX Completed"));
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received ACK"));
      }
      transmissionComplete = true;
      break;

    case EV_RESET:
    case EV_LINK_DEAD:
    case EV_LINK_ALIVE:
      break;

    default:
      break;
  }
}

void readSensors() {
  // Power on the sensor
  pinMode(PH_POWER_PIN, OUTPUT);
  digitalWrite(PH_POWER_PIN, HIGH);
  delay(1000);  // Allow sensor circuit to stabilize

  // Flush ADC with a dummy read, then sample
  analogRead(PH_PIN);
  delay(10);

  int rawADC = 0;
  for (int i = 0; i < 10; i++) {
    rawADC += analogRead(PH_PIN);
    delay(10);
  }
  rawADC /= 10;

  voltage = (rawADC / 1024.0) * 5000.0;
  // 25.0 is the standard reference temperature (°C) for pH calculation
  phValue = ph.readPH(voltage, 25.0);

  // Float the pin to avoid sinking reverse current through MCU diodes during sleep
  digitalWrite(PH_POWER_PIN, LOW);
  pinMode(PH_POWER_PIN, INPUT);

  Serial.print(F("pH: "));
  Serial.println(phValue, 2);
}

void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
    return;
  }

  // 1. Pack payload with Cayenne LPP formatting (4 bytes)
  byte payload[4];
  uint8_t cursor = 0;

  // Channel 2: pH Sensor (Analog Input)
  payload[cursor++] = 0x02;
  payload[cursor++] = LPP_ANALOG_INPUT;
  int16_t phInt = (int16_t)(phValue * 100);
  payload[cursor++] = highByte(phInt);
  payload[cursor++] = lowByte(phInt);

  // 2. Queue packet for transmission
  LMIC_setTxData2(1, payload, cursor, 0);
  Serial.println(F("Packet queued"));
}

void initLMICSession() {
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
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);
#elif defined(CFG_us915)
  LMIC_selectSubBand(1);
#endif

  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7, 14);
}

void sleepSeconds(unsigned long seconds) {
  // Save frame counter before radio shutdown
  uint32_t currentSeqnoUp = LMIC.seqnoUp;

  // 1. Put LoRa radio to sleep
  LMIC_shutdown();

  // 2. Put MCU to deep sleep in 8-second watchdog chunks
  unsigned long loops = seconds / 8;
  for (unsigned long i = 0; i < loops; i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }

  // 3. Re-initialize radio and LMIC session on wake-up
  os_init();
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 3 / 100);
  initLMICSession();

  // 4. Restore frame counter so TTN does not reject as replay
  LMIC.seqnoUp = currentSeqnoUp;
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting low-power node..."));

  ph.begin();

  os_init();
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 3 / 100);
  initLMICSession();

  // Sample sensor and queue initial transmission
  readSensors();
  do_send(&sendjob);
}

void loop() {
  // Run LMIC scheduler until the transmission and receive windows finish
  os_runloop_once();

  if (transmissionComplete) {
    transmissionComplete = false;
    Serial.println(F("Entering deep sleep..."));
    Serial.flush();  // Ensure serial buffer is empty before sleep

    // Sleep MCU + Radio
    sleepSeconds(SLEEP_SECONDS);

    Serial.println(F("Woke up from sleep!"));

    // Take a new measurement after waking up, then queue
    readSensors();
    do_send(&sendjob);
  }
}
