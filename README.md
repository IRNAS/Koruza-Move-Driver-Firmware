# Koruza-Move-Driver-Firmware

This repository contains firmware for Koruza 2.0 move driver. Koruza move driver is a small Atmel ATmega328p board with magnetic encoder sensrors, end buttons, and unipolar stepper drivers. All nessesery hardware details and specs are in the repository [Koruza move driver](link1).

In the picture below is the software diagram for the MCU:
![alt tag](pic_link1)

First layer is MCU hardware platform, after that are the drivers for hardware pheriperials. Third layer contains device drivers for easy communication with other devices on the Koruza move driver.
Last layer is the aplication layer where the main state mashine is implemented.

![alt tag](pic_link2)

* INIT: init the MCU and other devices on board
* IDLE
* TLV ACTIVE: TLV message received, parsing the received message
* ERROR: bad TLV message received
* STATUS: responce with driver status message
* PARAMETERS: responce with parameters for calibration
* HOMING: do the homing routine
* MOVE: move to given coordinate
* RESTORE PARAMETERS: restore move parameters.

Firmware upgrade is done thrue UART and MCU bootloader, after that the MCU restarts for the new firmware to boot. The MCU can be rebooted (hard restart) asynchronously. 

[link1]: <nolink>
[pic_link1]: <nolink>
[pic_link2]: <nolink>
