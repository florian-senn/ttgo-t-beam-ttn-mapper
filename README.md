# ttgo-t-beam-ttn-mapper

Ready to go configuration for a TTGO T-Beam TTNMapper Node with ADB and Visual Studio Code and PlatformIO
Configured ready to build with EU868 LoRaWAN-Frequencies.
Just create /include/secrets.h with

#define SECRET_NWKSKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } // Network Session Key, MSB

#define SECRET_APPSKEY { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } // App Session Key, MSB

#define SECRET_DEVADDR 0x00000000 // Device Address, HEX

Launched this repo cause I failed to build a working TTNMapper with the T-Beam with all the other projects, caused by missing building flages.

The magic sauce:

build_flags = 
	-D ARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS
	-D ARDUINO_LMIC_CFG_NETWORK_TTN
	-D CFG_sx1276_radio
	-D CFG_eu868
	-D LMIC_DEBUG_LEVEL=2
