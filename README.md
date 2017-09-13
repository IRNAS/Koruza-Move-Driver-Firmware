
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
* STATUS: response with driver status message, after the motor positin is restored driver responce with the current position
* PARAMETERS: response with parameters for calibration
* HOMING: do the homing routine
* MOVE: move to given coordinate
* RESTORE PARAMETERS: restore move parameters, before restore it is not posibile to move the motors nor do homing.

The firmware upgrade is done through the UART and MCU bootloader, after that the MCU restarts for the new firmware to boot. The MCU can be rebooted (hard restart) asynchronously. 

### Communication
Koruza move driver is using the UART for communicating with the rest of the Koruza unit. A chosen protocol is based on TLV. All necessary documentation for communication is in [TLV communication](https://github.com/IRNAS/Koruza-Move-Driver-Firmware/blob/docs/TLV_communication.md) file.

---

## Licensing

Firmware and software originating from KORUZA Pro project, including Koruza Move Driver Firmware, is licensed under [GNU GENERAL PUBLIC LICENSE v3](https://www.gnu.org/licenses/gpl-3.0.en.html).

What this means is that you can use this firmware and software without paying a royalty and knowing that you'll be able to use your version forever. You are also free to make changes but if you share these changes then you have to do so on the same conditions that you enjoy.

KORUZA, KORUZA Pro and IRNAS are all names and marks of Institute IRNAS Rače. You may use these names and terms only to attribute the appropriate entity as required by the Open Licence referred to above. You may not use them in any other way and in particular you may not use them to imply endorsement or authorization of any hardware that you design, make or sell.

[link1]: <nolink>
[link2]: <https://github.com/IRNAS/Koruza-Move-Driver-Firmware/blob/docs/TLV_communication.md>

