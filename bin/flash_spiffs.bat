set PORT=COM1
set version=v5.00.0

set spiffs=spiffs_%version%.bin
set esptooldir=C:\Users\gereic\AppData\Local\Programs\Python\Python310\Scripts\
REM set firmware=firmware_v1.6.0_esp32PSRam.bin

%esptooldir%esptool.exe --chip esp32 -p %PORT% -b 921000 write_flash 0x3d0000 %spiffs%