#include <Arduino.h>
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

BLECharacteristic *characteristicTX;
#define BatteryService BLEUUID((uint16_t)0x180F)
BLECharacteristic BatteryLevelCharacteristic(BLEUUID((uint16_t)0x2A19), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor BatteryLevelDescriptor(BLEUUID((uint16_t)0x2901));
bool _BLEClientConnected = false;
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    _BLEClientConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    _BLEClientConnected = false;
  }
};

// Delay between Lora Send
uint8_t sendInterval[] = {20, 30, 40, 10};
uint8_t sendIntervalKey = 3;

HardwareSerial GPSSerial(1);
TinyGPSPlus GPS;

AXP20X_Class axp;

String LoraStatus;

static osjob_t sendjob;

uint8_t loraBuffer[9];

TinyGPSLocation lastLocation = TinyGPSLocation();

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
unsigned long reportSpan = 3000UL;
unsigned long lastReport = now + reportSpan;
unsigned long bleSpan = 3000UL;
unsigned long lastBle = now + bleSpan;

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
    digitalWrite(BUILTIN_LED, LOW);
    if (LMIC.txrxFlags & TXRX_ACK)
    {
      LoraStatus = "Recvd Ack";
    }
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks((sendInterval[sendIntervalKey])), do_send);
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
    if (gpsHasFix() && GPS.distanceBetween(lastLocation.lat(), lastLocation.lng(), GPS.location.lat(), GPS.location.lng()) > 50.0)
    {
      uint32_t LatitudeBinary = ((GPS.location.lat() + 90) / 180.0) * 16777215;
      uint32_t LongitudeBinary = ((GPS.location.lng() + 180) / 360.0) * 16777215;
      uint16_t altitudeGps = GPS.altitude.meters();
      uint8_t hdopGps = GPS.hdop.value() / 10;

      loraBuffer[0] = (LatitudeBinary >> 16) & 0xFF;
      loraBuffer[1] = (LatitudeBinary >> 8) & 0xFF;
      loraBuffer[2] = LatitudeBinary & 0xFF;
      loraBuffer[3] = (LongitudeBinary >> 16) & 0xFF;
      loraBuffer[4] = (LongitudeBinary >> 8) & 0xFF;
      loraBuffer[5] = LongitudeBinary & 0xFF;
      loraBuffer[6] = (altitudeGps >> 8) & 0xFF;
      loraBuffer[7] = altitudeGps & 0xFF;
      loraBuffer[8] = hdopGps & 0xFF;

      LMIC_setTxData2(1, loraBuffer, sizeof(loraBuffer), 0);
      digitalWrite(BUILTIN_LED, HIGH);
      LoraStatus = "QUEUED";
      lastLocation = GPS.location;
    }
    else
    {
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
    }
  }
}

void setup()
{
  GPSSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  while (!GPSSerial)
    ;

  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);

  int ret = axp.begin(Wire, AXP192_SLAVE_ADDRESS);

  if (ret == AXP_FAIL)
  {
    Serial.println("AXP Power begin failed");
    while (1)
      ;
  }

  axp.adc1Enable(AXP202_VBUS_VOL_ADC1, 1);
  axp.adc1Enable(AXP202_VBUS_CUR_ADC1, 1);
  axp.EnableCoulombcounter();

  // Builtin LED will be used to indicate LoRa Activity
  pinMode(BOARD_LED, OUTPUT);

  // LoRa Init
  os_init();
  LMIC_reset();
  LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);

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
  LMIC_setDrTxpow(DR_SF7, 14);
  // Don't do Link Checks
  LMIC_setLinkCheckMode(0);

  do_send(&sendjob);
  digitalWrite(BUILTIN_LED, LOW);

  BLEDevice::init("BLE Battery");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pBattery = pServer->createService(BatteryService);

  pBattery->addCharacteristic(&BatteryLevelCharacteristic);
  BatteryLevelDescriptor.setValue("Percentage 0 - 100");
  BatteryLevelCharacteristic.addDescriptor(&BatteryLevelDescriptor);
  BatteryLevelCharacteristic.addDescriptor(new BLE2902());

  pServer->getAdvertising()->addServiceUUID(BatteryService);

  pBattery->start();
  // Start advertising
  pServer->getAdvertising()->start();
}

void loop()
{
  now = millis();
  while (GPSSerial.available())
  {
    char data = GPSSerial.read();
    GPS.encode(data);
    //Serial.write(data);
  }
  while (Serial.available())
  {
    GPSSerial.write(Serial.read());
  }

  if (now - reportSpan > lastReport)
  {
    lastReport = now;
    Serial.println("=========================");
    Serial.print("VBUS STATUS: ");
    // You can use isVBUSPlug to check whether the USB connection is normal
    if (axp.isVBUSPlug())
    {

      Serial.println("CONNECT");

      // Get USB voltage
      Serial.print("VBUS Voltage: ");
      Serial.print(axp.getVbusVoltage());
      Serial.println(" mV");

      // Get USB current
      Serial.print("VBUS Current: ");
      Serial.print(axp.getVbusCurrent());
      Serial.println(" mA");
    }
    else
    {
      Serial.println("DISCONNECT");
    }

    Serial.println("=========================");

    Serial.print("BATTERY STATUS: ");

    // You can use isBatteryConnect() to check whether the battery is connected properly
    if (axp.isBatteryConnect())
    {
      Serial.println("CONNECT");

      // Get battery voltage
      Serial.print("BAT Voltage: ");
      Serial.print(axp.getBattVoltage());
      Serial.println(" mV");

      // To display the charging status, you must first discharge the battery,
      // and it is impossible to read the full charge when it is fully charged
      if (axp.isChargeing())
      {
        Serial.print("Charge: ");
        Serial.print(axp.getBattChargeCurrent());
        Serial.println(" mA");
      }
      else
      {
        // Show current consumption
        Serial.print("Discharge: ");
        Serial.print(axp.getBattDischargeCurrent());
        Serial.println(" mA");
      }
    }
    else
    {
      Serial.println("DISCONNECT");
    }
    Serial.print("AXP Temperature: ");
    Serial.print(axp.getTemp());
    Serial.println(" °C");
    Serial.print("Coulomb Dis-/Charge: ");
    Serial.print(axp.getBattDischargeCoulomb());
    Serial.print("/");
    Serial.print(axp.getBattChargeCoulomb());
    Serial.println(" C");
    Serial.print("Coulomb Data: ");
    Serial.print(axp.getCoulombData());
    Serial.println(" C");
    Serial.print("Power Down: ");
    Serial.print(axp.getPowerDownVoltage());
    Serial.println(" mV");
    Serial.print("Warning 1: ");
    Serial.print(axp.getVWarningLevel1());
    Serial.println(" mV");
    Serial.print("Warning 2: ");
    Serial.print(axp.getVWarningLevel2());
    Serial.println(" mV");
    Serial.println();
    Serial.println();
  }
  if (axp.getTemp() > 70.0)
  {
    axp.setChargeControlCur(AXP1XX_CHARGE_CUR_630MA);
  }
  else if (axp.getTemp() < 66.0)
  {
    axp.setChargeControlCur(AXP1XX_CHARGE_CUR_1000MA);
  }
  if (_BLEClientConnected && now - lastBle > bleSpan)
  {
    lastBle = now;
    uint8_t level = map(axp.getBattVoltage(), 2600, 4200, 0, 100);
    BatteryLevelCharacteristic.setValue(&level, 1);
    BatteryLevelCharacteristic.notify();
  }
  os_runloop_once();
  yield();
}