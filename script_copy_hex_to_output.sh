#!/bin/bash

echo "Start script."
echo "Remove hex file with bootloader."
rm ./move_driver_firmware/move_driver_firmware.ino.with_bootloader.eightanaloginputs.hex
echo "Move hex file to correct location."
mv ./move_driver_firmware/move_driver_firmware.ino.eightanaloginputs.hex ./output/move_driver_firmware.ino.hex
