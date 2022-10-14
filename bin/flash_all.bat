set PORT=COM1
REM set version=v4.8.4
set version=v5.00.0
REM set option=psRam
set option=nopsRam
REM set option=sim800
REM set option=sim7000_psRam


set firmware=firmware_%version%_%option%.bin
set spiffs=spiffs_%version%.bin
set esptooldir=C:\Users\gereic\AppData\Local\Programs\Python\Python310\Scripts\
REM set firmware=firmware_v1.6.0_esp32PSRam.bin
set baudrate=921000
%esptooldir%esptool.exe --chip esp32 -p %PORT% -b %baudrate% write_flash 0x1000 bootloader_dio_40m.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 %firmware% 0x3d0000 %spiffs%