#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <lmic.h>
#include <hal/hal.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <axp20x.h>
#include <config.h>

#define BatteryService BLEUUID((uint16_t)0x180F)
BLECharacteristic BatteryLevelCharacteristic(BLEUUID((uint16_t)0x2A19), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor BatteryLevelDescriptor(BLEUUID((uint16_t)0x2901));
BLECharacteristic UartCharacteristic(UART_CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEServer *pServer;
bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

uint8_t getBatLevel();

Preferences prefs;

// Delay between Lora Send
uint8_t sendInterval[] = {10, 20, 30, 60, 120};
bool firstSend = true;

HardwareSerial GPSSerial(1);
TinyGPSPlus GPS;

AXP20X_Class axp;

String LoraStatus;
uint8_t tdata[9];
uint8_t bdata[6];

static osjob_t sendjob;

void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}
void do_send(osjob_t *j); //

// Setup Lora Pins
const lmic_pinmap lmic_pins = {
    .nss = RADIO_CS_PIN,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RADIO_RST_PIN,
    .dio = {RADIO_DI0_PIN, RADIO_DIO1_PIN, RADIO_BUSY_PIN}};

bool gpsHasFix()
{
  return (GPS.location.isValid() &&
          GPS.location.age() < 2000 &&
          GPS.hdop.isValid() &&
          GPS.hdop.value() <= 300 &&
          GPS.hdop.age() < 2000 &&
          GPS.altitude.isValid() &&
          GPS.altitude.age() < 2000);
}

unsigned long now = millis();
unsigned long bleSpan = 5000UL;
unsigned long lastBle = now + bleSpan;
uint8_t lastBatLevel = 0;

// Lora Event Handling
void onEvent(ev_t ev)
{
  switch (ev)
  {
  case EV_SCAN_TIMEOUT:
    Serial.println("LoRa: onEvent: EV_SCAN_TIMEOUT");
    LoraStatus = "SCANTIMEO";
    break;
  case EV_BEACON_FOUND:
    Serial.println("LoRa: onEvent: EV_BEACON_FOUND");
    LoraStatus = "BEAC_FOUN";
    break;
  case EV_BEACON_MISSED:
    Serial.println("LoRa: onEvent: EV_BEACON_MISSED");
    LoraStatus = "BEAC_MISS";
    break;
  case EV_BEACON_TRACKED:
    Serial.println("LoRa: onEvent: EV_BEACON_TRACKED");
    LoraStatus = "BEAC_TRAC";
    break;
  case EV_JOINING:
    Serial.println("LoRa: onEvent: EV_JOINING");
    LoraStatus = "JOINING";
    break;
  case EV_JOINED:
    Serial.println("LoRa: onEvent: EV_JOINED");
    LoraStatus = "JOINED";
    LMIC_setLinkCheckMode(0);
    break;
  case EV_RFU1:
    Serial.println("LoRa: onEvent: EV_RFU1");
    LoraStatus = "RFU1";
    break;
  case EV_JOIN_FAILED:
    Serial.println("LoRa: onEvent: EV_JOIN_FAILED");
    LoraStatus = "JOIN_FAIL";
    break;
  case EV_REJOIN_FAILED:
    Serial.println("LoRa: onEvent: EV_REJOIN_FAILED");
    LoraStatus = "REJOIN_FA";
    break;
  case EV_TXCOMPLETE:
    Serial.println("LoRa: onEvent: EV_TXCOMPLETE");
    LoraStatus = "TXCOMPL";
    digitalWrite(BOARD_LED, LED_OFF);
    if (LMIC.txrxFlags & TXRX_ACK)
    {
      LoraStatus = "Recvd Ack";
    }
    prefs.putUInt("frameCountUp", LMIC.seqnoUp);
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks((sendInterval[SEND_INTERVAL])), do_send);
    break;
  case EV_LOST_TSYNC:
    Serial.println("LoRa: onEvent: EV_LOST_TSYNC");
    LoraStatus = "LOST_TSYN";
    break;
  case EV_RESET:
    Serial.println("LoRa: onEvent: EV_RESET");
    LoraStatus = "RESET";
    break;
  case EV_RXCOMPLETE:
    Serial.println("LoRa: onEvent: EV_RXCOMPLETE");
    LoraStatus = "RXCOMPL";
    break;
  case EV_LINK_DEAD:
    Serial.println("LoRa: onEvent: EV_LINK_DEAD");
    LoraStatus = "LINK_DEAD";
    break;
  case EV_LINK_ALIVE:
    Serial.println("LoRa: onEvent: EV_LINK_ALIVE");
    LoraStatus = "LINK_ALIV";
    break;
  default:
    LoraStatus = "UNKNOWN";
    break;
  }
}

void do_send(osjob_t *j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    LoraStatus = "TXRXPEND";
  }
  else
  {
    double lat = GPS.location.lat();
    double lng = GPS.location.lng();
    if (gpsHasFix() && (firstSend || GPS.distanceBetween(prefs.getDouble("lat", 0.0), prefs.getDouble("lng", 0.0), lat, lng) > MINIMUM_DISTANCE))
    {
      firstSend = false;
      uint32_t LatitudeBinary = ((lat + 90) / 180.0) * 16777215;
      uint32_t LongitudeBinary = ((lng + 180) / 360.0) * 16777215;
      uint16_t altitudeGps = GPS.altitude.meters();
      uint8_t hdopGps = GPS.hdop.value() / 10;
      //uint8_t batteryLevel = getBatLevel();
      if (TTN_MAPPER)
      {
        tdata[0] = (LatitudeBinary >> 16) & 0xFF;
        tdata[1] = (LatitudeBinary >> 8) & 0xFF;
        tdata[2] = LatitudeBinary & 0xFF;
        tdata[3] = (LongitudeBinary >> 16) & 0xFF;
        tdata[4] = (LongitudeBinary >> 8) & 0xFF;
        tdata[5] = LongitudeBinary & 0xFF;
        tdata[6] = (altitudeGps >> 8) & 0xFF;
        tdata[7] = altitudeGps & 0xFF;
        tdata[8] = hdopGps & 0xFF;
        LMIC_setTxData2(1, tdata, sizeof(tdata), 0);
      }
      else
      {
        //TODO: 
        // 62 byte buffer containing 10 coordinates à 6 bytes and 2 bytes for meta (battery?) -> airtime: 
        // 133,4@SF7 -> 14s pause, 224 messages/d -> 2240 coords/d -> 6,2h tracking @ 10s interval, message every 100s
        // 246,3@SF8 -> 25s pause, 121 messages/d -> 1210 coords/d -> 3,3h tracking
        // 431,1@SF9 -> 42s pause, 69 messages/d -> 690 coords/d -> 1,9h tracking
        // OR
        // 115 byte buffer containing 19 coordinates à 6 bytes and 1 byte for meta (battery?) -> airtime:
        // 215.3@SF7 -> 22s pause, 139 messages/d -> 2641 coords/d ->  7,3h tracking @ 10s interval, message every 190s
        // 379.4@SF8 -> 38s pause, 79 messages/d -> 1501 coords/d -> 4,1h tracking
        // 676.9@SF9 -> 68s pause, 
        // OR
        // 121 byte buffer containing 20 coordinates à 6 bytes and 1 byte for meta (battery?) -> airtime:
        // 220,4@SF7 -> 22s pause, 136 messages/d -> 2720 coords/d -> 7,5h tracking @ 10s interval, message every 200s
        // 389.6@SF8 ->
        // 
        // OR
        // 185 byte buffer containing 30 coordinates à 6 bytes and 5 bytes for meta (battery?) -> airtime:
        // 317,7@SF7 -> 32s pause, 94 messages/d -> 2820 coords/d -> 7,8h tracking @ 10s interval, message every 300s
        // 553,5@SF8 -> 56s pause, 54 messages/d -> 1620 coords/d -> 4,5h tracking
        // OR
        // 222 byte buffer containing 37 coordinates à 6 bytes -> airtime:
        // 368,9@SF7 -> 37s pause, 81 messages/d -> 2997 coords/d -> 8,3h tracking @ 10s interval, message every 370s
        // 655,9@SF8 -> 66s pause, 45 messages/d -> 1665 coords/d -> 4,6h tracking
        bdata[0] = (LatitudeBinary >> 16) & 0xFF;
        bdata[1] = (LatitudeBinary >> 8) & 0xFF;
        bdata[2] = LatitudeBinary & 0xFF;
        bdata[3] = (LongitudeBinary >> 16) & 0xFF;
        bdata[4] = (LongitudeBinary >> 8) & 0xFF;
        bdata[5] = LongitudeBinary & 0xFF;
        u1_t port = 1;
        LMIC_setTxData2(port, bdata, sizeof(bdata), 0);
      }
      digitalWrite(BOARD_LED, LED_ON);
      LoraStatus = "QUEUED";
      prefs.putDouble("lat", lat);
      prefs.putDouble("lng", lng);
    }
    else
    {
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
    }
  }
}

void setup()
{
  WiFi.mode(WIFI_OFF);
  WiFi.setSleep(true);
  prefs.begin("lat", false);
  prefs.begin("lng", false);
  prefs.begin("frameCountUp", false);
  prefs.begin("frameCountDown", false);
  prefs.begin("maxvlt", false);
  prefs.begin("minvlt", false);

  GPSSerial.begin(SERIAL_SPEED, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  while (!GPSSerial)
    ;

  Serial.begin(SERIAL_SPEED);

  Wire.begin(I2C_SDA, I2C_SCL);

  while (axp.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL)
  {
    Serial.println("AXP Power begin failed");
  }

  axp.adc1Enable(AXP202_VBUS_VOL_ADC1, 1);
  axp.adc1Enable(AXP202_VBUS_CUR_ADC1, 1);
  axp.EnableCoulombcounter();
  axp.setChargingTargetVoltage(AXP202_TARGET_VOL_4_1V);
  axp.setPowerDownVoltage(POWER_DOWN);
  pinMode(BOARD_LED, OUTPUT);

  //  Init
  os_init();
  LMIC_reset();
  LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
  LMIC_setSeqnoUp(prefs.getUInt("frameCountUp"));

  // Setup EU Channels
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  // Disable Data Rate Adaptation, for Mapping we want static SF7
  LMIC_setAdrMode(0);
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(SF, 14);
  // Don't do Link Checks
  LMIC_setLinkCheckMode(0);

  do_send(&sendjob);
  digitalWrite(BOARD_LED, LED_OFF);

  BLEDevice::init(DEVICE_NAME);
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pBattery = pServer->createService(BatteryService);
  pBattery->addCharacteristic(&BatteryLevelCharacteristic);
  BatteryLevelDescriptor.setValue("Percentage 0 - 100");
  BatteryLevelCharacteristic.addDescriptor(&BatteryLevelDescriptor);
  BatteryLevelCharacteristic.addDescriptor(new BLE2902());

  BLEService *pUART = pServer->createService(UART_SERVICE_UUID);
  UartCharacteristic.addDescriptor(new BLE2902());
  pUART->addCharacteristic(&UartCharacteristic);

  pServer->getAdvertising()->addServiceUUID(BatteryService);
  pServer->getAdvertising()->addServiceUUID(UART_SERVICE_UUID);

  pBattery->start();
  pUART->start();
  // Start advertising
  pServer->getAdvertising()->start();
}

uint8_t getBatLevel()
{
  float volt = axp.getBattVoltage();
  float con = constrain(volt, POWER_DOWN, CHARGE_CUTOFF);
  return map(con, POWER_DOWN, CHARGE_CUTOFF, 0, 100);
}

void reportBatBLE()
{
  uint8_t level = getBatLevel();
  if (lastBatLevel != level)
  {
    lastBatLevel = level;
    BatteryLevelCharacteristic.setValue(&level, 1);
    BatteryLevelCharacteristic.notify();
  }
}

void reportInfoBLE()
{
  float current;
  if (axp.isChargeing())
  {
    current = axp.getBattChargeCurrent();
  }
  else
  {
    current = axp.getBattDischargeCurrent();
  }
  char buf[7];
  dtostrf(current, 5, 1, buf);
  UartCharacteristic.setValue(buf);
  UartCharacteristic.notify();
}

void BLE()
{
  if (deviceConnected && now - lastBle > bleSpan)
  {
    reportBatBLE();
    reportInfoBLE();
    lastBle = now;
  }
  if (!deviceConnected && oldDeviceConnected)
  {
    pServer->getAdvertising()->start();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
  }
}

void loop()
{
  os_runloop_once();
  now = millis();
  while (GPSSerial.available())
  {
    char data = GPSSerial.read();
    GPS.encode(data);
    Serial.write(data);
  }
  while (Serial.available())
  {
    GPSSerial.write(Serial.read());
  }
  BLE();
}