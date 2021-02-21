# Compact PCR
## Spring 2021, Senior Design II @ CCNY
**Instructor:** Prof. Seo

**Member**: Quang Truong, Yossel Newman, Yousra Tabrani, Gnimdou Tchalim

# Instruction
This was written using Visual Studio Code (VSC) and the plugin PlatformIO (PIO) to flash to the microcontroller. Arduino IDE can be used, just make sure the libraries match.

**Library Dependencies:** The list is in this folder [here](.pio/libdeps/adafruit_feather_nrf52832).
***"Plotter"*** is not required for the functionality, it's meant to display serial data on Processing. If you wish to install, the instructions are [here](https://github.com/devinaconley/arduino-plotter).


### !!!⚠️CAUTION⚠️!!!! READ FIRST
1) **This** project requires some knowledge in electronics. **DO NOT** attach the heating element directly to a pin. Please use a MOSFET to PWM high power component.
2) Make sure you have at least 5V 2A power supply as the heating element easily spike up to 0.5A.
3) **DO NOT** attach heating element before making sure temperature sensor work, otherwise there is a risk of **uncontrollable** heating.
4) Make sure all operation **DO NOT** exceed any limits per image below:
    ![Feather nRF52 Pinout v1.2](https://cdn-learn.adafruit.com/assets/assets/000/046/248/original/microcontrollers_Feather_NRF52_Pinout_v1.2-1.png?1504885794)
