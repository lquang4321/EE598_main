# Compact PCR
## Spring 2021, Senior Design II @ CCNY
**Instructor:** Prof. Seo

**Member**: Quang Truong, Yossel Newman, Yousra Tabrani, Gnimdou Tchalim

# Instruction
This was written using Visual Studio Code (VSC) and the plugin PlatformIO (PIO) to flash to the microcontroller. Arduino IDE can be used, just make sure the libraries match.

**Library Dependencies:** The list is in this folder [here](.pio/libdeps/adafruit_feather_nrf52832).
***"Plotter"*** is not required for the functionality, it's meant to display serial data on Processing. If you wish to install, the instructions are [here](https://github.com/devinaconley/arduino-plotter).


### !!!⚠️CAUTION⚠️!!!!
1) **This** project requires some knowledge in electronics. **DO NOT** attach the heating element directly to a pin. Please use a MOSFET to PWM high power component.
2) Make sure you have at least 5V 2A power supply as the heating element easily spike up to 0.5A.

