## Embedded software for ESP32 driven, bench powered PEMF device

This repo houses embedded software for an ESP32 driven PEMF prototype (Pulsed Electromagnetic Field therapy). A more detailed overview of the project can be found here: [PEMF-PROTOTYPE](https://github.com/ajdonich/pemf-prototype). The PCB for this v1 supports up to three  electromagnets and contains a 4-channel DAC, 3 voltage-to-current opamp driver circuits, and header to attach a (KeeYees or compatible) 30-pin ESP32 dev board. After flashing, the ESP32 and the electromagnets must be powered by 5V bench supply.

Because the ESP32 is commanded (typically by smartphone, see: [EMController](https://github.com/ajdonich/EMController)) over WIFI, the SSID and password must be set in [ProjectSettings.h](https://github.com/ajdonich/pemf-esp32/blob/main/include/ProjectSettings.h) prior to flashing. The codebase employs Arduino libs and is based on [PlatformIO](https://platformio.org/). Multiple main loop files are included in the codebase for performing distinct functionalities, but only one can be built/deployed at a time, which can be selected via the `build_src_filter` in [platformio.ini](https://github.com/ajdonich/pemf-esp32/blob/main/platformio.ini).

![EMController_ESP32](https://github.com/ajdonich/pemf-prototype/blob/main/EMController_ESP32.png)

