set esptooldir=C:\Users\gereic\AppData\Local\Programs\Python\Python38-32\Scripts\
set firmware=firmware_v1.6.1_esp32noPSRam.bin
REM set firmware=firmware_v1.6.0_esp32PSRam.bin

%esptooldir%esptool.exe -p COM1 -b 921000 write_flash 0x1000 bootloader_dio_40m.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 %firmware% 0x3d0000 spiffs.bin