esptool -p /dev/ttyUSB0 write_flash 0x1000 bootloader_dio_40m.bin  0x8000 partitions_8MB.bin 0xe000 boot_app0.bin 0x10000 firmware_v5.00.0_nopsRam_8MB.bin 0x7d0000 spiffs_v5.00.0.bin
