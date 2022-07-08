set PORT=COM1
set firmware=firmware_v5.00.0_nopsRam.bin

set esptooldir=C:\Users\gereic\AppData\Local\Programs\Python\Python310\Scripts\

REM set firmware=firmware_v1.6.0_esp32PSRam.bin

%esptooldir%esptool.exe -p %PORT% -b 921000 write_flash 0x10000 %firmware%