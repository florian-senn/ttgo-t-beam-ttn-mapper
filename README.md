# ttgo-t-beam-ttn-mapper

Ready to go configuration for a TTGO T-Beam TTNMapper Node with ADB
Configured ready to build with EU868 LoRaWAN-Frequencies.
Just create /include/secrets.h with

#define SECRET_NWKSKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } // Network Session Key, MSB
#define SECRET_APPSKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } // App Session Key, MSB
#define SECRET_DEVADDR 0x00000000 // Device Address, HEX
