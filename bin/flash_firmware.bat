set PORT=COM1
set version=v5.00.0
set option=sim800_nopsRam
REM set option=psRam
REM set option=nopsRam
REM set option=sim800
REM set option=sim7000_psRam

set firmware=firmware_v5.00.0_nopsRam.bin

set esptooldir=C:\Users\gereic\AppData\Local\Programs\Python\Python310\Scripts\

%esptooldir%esptool.exe --chip esp32 -p %PORT% -b 921000 write_flash 0x10000 %firmware%