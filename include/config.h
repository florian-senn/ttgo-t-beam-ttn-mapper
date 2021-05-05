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

#define BOARD_LED 4
#define LED_ON LOW
#define LED_OFF HIGH

#define GPS_BAUD_RATE 9600
#define HAS_GPS

#define SERVICE_UUID "ab0828b1-198e-4351-b779-901fa0e0371e"
#define CHARACTERISTIC_UUID_RX "4ac8a682-9736-4e5d-932b-e9b31405049c"
#define CHARACTERISTIC_UUID_TX "0972EF8C-7613-4075-AD52-756F33D4DA91"

#endif