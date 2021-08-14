set PORT=COM1
set firmware=firmware_v4.12.0_sim7000_psRam.bin

set esptooldir=C:\Users\gereic\AppData\Local\Programs\Python\Python38-32\Scripts\

REM set firmware=firmware_v1.6.0_esp32PSRam.bin

%esptooldir%esptool.exe -p %PORT% -b 921000 write_flash 0x10000 %firmware%