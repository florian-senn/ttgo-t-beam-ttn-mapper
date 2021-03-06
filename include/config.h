#include <secrets.h>
#ifndef LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED
#define LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED

// Fill these with the values from https://console.thethingsnetwork.org
static PROGMEM u1_t NWKSKEY[16] = SECRET_NWKSKEY; // LoRaWAN Network Session Key, MSB
static PROGMEM u1_t APPSKEY[16] = SECRET_APPSKEY; // LoRaWAN App Session Key, MSB
static const u4_t DEVADDR = SECRET_DEVADDR;       // LoRaWAN Device Address (HEX)

#define UNUSE_PIN (0)
#define GPS_RX_PIN 34
#define GPS_TX_PIN 12
#define BUTTON_PIN 38
#define BUTTON_PIN_MASK GPIO_SEL_38
#define I2C_SDA 21
#define I2C_SCL 22
#define PMU_IRQ 35

#define RADIO_SCLK_PIN 5
#define RADIO_MISO_PIN 19
#define RADIO_MOSI_PIN 27
#define RADIO_CS_PIN 18
#define RADIO_DI0_PIN 26
#define RADIO_RST_PIN 23
#define RADIO_DIO1_PIN 33
#define RADIO_BUSY_PIN 32
#define DISABLE_PING 1
#define DISABLE_BEACONS 1
#define LMIC_DEBUG_LEVEL 1
#define DISABLE_JOIN 1

#define BOARD_LED 4
#define LED_ON LOW
#define LED_OFF HIGH
#define CHARGE_CUTOFF 4100
#define POWER_DOWN 3300

#define GPS_BAUD_RATE 115200
#define SERIAL_SPEED 115200
#define HAS_GPS

#define UART_SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define UART_CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define UART_CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#endif