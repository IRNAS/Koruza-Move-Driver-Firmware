# Koruza-Move-Driver-Firmware

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

[link1]: <nolink>

