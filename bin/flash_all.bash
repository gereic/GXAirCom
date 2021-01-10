#!/bin/bash
#esptool -p /dev/ttyUSB2  write_flash 0x1000 bootloader_dio_40m.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware_v2.2.0_am_eink_psRam.bin 0x3d0000 test.spiffs
esptool -p /dev/ttyUSB2  write_flash 0x1000 bootloader_dio_40m.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware_v2.2.0_am_oled_psRam.bin 0x3d0000 spiffs.bin