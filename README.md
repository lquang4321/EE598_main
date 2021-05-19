# Compact PCR
## Spring 2021, Senior Design II @ CCNY
**Instructor:** Prof. Sang-woo Seo

**Member**: Quang Truong, Yossel Newman, Yousra Tabrani, Gnimdou Tchalim

# Instruction
This was written using *Visual Studio Code (VSC)* and the plugin *PlatformIO (PIO)* to flash to the microcontroller. Arduino IDE can be used, just make sure the libraries match. The **microcontroller** from now on only pertains to the [Adafruit Bluefruit nRF52 Feather](https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/introduction). Refer to the manufacturer's instruction and manual for your specific board.

**Library Dependencies:** The list is in this folder [here](.pio/libdeps/adafruit_feather_nrf52832).
***"Plotter"*** is not required for the functionality, it's meant to display serial data on Processing. If you wish to install, the instructions are [here](https://github.com/devinaconley/arduino-plotter).


### !!!⚠️CAUTION⚠️!!!! READ FIRST
1) This project requires some knowledge in electronic. **DO NOT** attach the heating element directly to a pin. Please use a MOSFET to PWM high power component.
2) **DO NOT** attach heating element before making sure temperature sensor work, otherwise there is a risk of **uncontrollable** heating. As of now, heating element operates automatically based on temperature sensor. Disconnected temperature sensor will read -273*C.
3) Make sure all operation **DO NOT** exceed any limits per image below:
    ![Feather nRF52 Pinout v1.2](https://cdn-learn.adafruit.com/assets/assets/000/046/248/original/microcontrollers_Feather_NRF52_Pinout_v1.2-1.png?1504885794)
4) The schematic for the PCR Project:
    ![Compact PCR Schematic](https://github.com/lquang4321/EE598_main/blob/master/schematic/Schematic.png)
