set PORT=COM1

set esptooldir=C:\Users\gereic\AppData\Local\Programs\Python\Python38-32\Scripts\
REM set firmware=firmware_v1.6.0_esp32PSRam.bin

%esptooldir%esptool.exe -p %PORT% -b 921000 write_flash 0x3d0000 spiffs.bin