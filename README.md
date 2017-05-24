
# Koruza-Move-Driver-Firmware [![Build Status](https://travis-ci.org/IRNAS/Koruza-Move-Driver-Firmware.svg?branch=master)](https://travis-ci.org/IRNAS/Koruza-Move-Driver-Firmware)

This repository contains firmware for Koruza 2.0 move driver. Koruza move driver is a small Atmel ATmega328p board with magnetic encoder sensors, end buttons, and unipolar stepper drivers. All necessary hardware details and specs are in the repository [Koruza move driver](link1).

In the picture below is the software diagram for the MCU:

![alt tag](https://raw.githubusercontent.com/IRNAS/Koruza-Move-Driver-Firmware/docs/Copy%20of%20koruza_move_driver_sw.png)

The first layer is MCU hardware platform after that are the drivers for hardware peripherals. The third layer contains device drivers for easy communication with other devices on the Koruza move driver.
The last layer is the application layer where the main state machine is implemented.

![alt tag](https://github.com/IRNAS/Koruza-Move-Driver-Firmware/blob/docs/koruza_move_driver_sm.png)

* INIT: init the MCU and other devices on board
* IDLE
* TLV ACTIVE: TLV message received, parsing the received message
* ERROR: bad TLV message received
* STATUS: response with driver status message
* PARAMETERS: response with parameters for calibration
* HOMING: do the homing routine
* MOVE: move to given coordinate
* RESTORE PARAMETERS: restore move parameters.

The firmware upgrade is done through the UART and MCU bootloader, after that the MCU restarts for the new firmware to boot. The MCU can be rebooted (hard restart) asynchronously. 

### Communication
Koruza move driver is using the UART for communicating with the rest of the Koruza unit. A chosen protocol is based on TLV. All necessary documentation for communication is in [TLV communication](https://github.com/IRNAS/Koruza-Move-Driver-Firmware/blob/docs/TLV_communication.md) file.

---

## Licensing

We strive to make KORUZA as usefully open-source as possible.
Hardware including documentation is licensed under [CERN OHL v.1.2. license](http://www.ohwr.org/licenses/cern-ohl/v1.2)

Firmware and software originating from the project is licensed under [GNU GENERAL PUBLIC LICENSE v3](http://www.gnu.org/licenses/gpl-3.0.en.html).

Open data generated by our projects is licensed under [CC0](https://creativecommons.org/publicdomain/zero/1.0/legalcode).

All our websites and additional documentation are licensed under [Creative Commons Attribution-ShareAlike 4 .0 Unported License] (https://creativecommons.org/licenses/by-sa/4.0/legalcode).

What this means is that you can use hardware, firmware, software and documentation without paying a royalty and knowing that you'll be able to use your version forever. You are also free to make changes but if you share these changes then you have to do so on the same conditions that you enjoy.

Koruza, GoodEnoughCNC and IRNAS are all names and marks of Institut IRNAS Rače. 
You may use these names and terms only to attribute the appropriate entity as required by the Open Licences referred to above. You may not use them in any other way and in particular you may not use them to imply endorsement or authorization of any hardware that you design, make or sell.

[link1]: <nolink>
[link2]: <https://github.com/IRNAS/Koruza-Move-Driver-Firmware/blob/docs/TLV_communication.md>

