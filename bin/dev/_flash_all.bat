set PORT=COM1
set version=v4.8.4
REM set version=v4.5.0
set option=psRam
REM set option=nopsRam
REM set option=sim800
REM set option=sim7000_psRam


set firmware=firmware_%version%_%option%.bin
set spiffs=spiffs_%version%.bin
set esptooldir=C:\Users\gereic\AppData\Local\Programs\Python\Python38-32\Scripts\

%esptooldir%esptool.exe -p %PORT% -b 921000 write_flash 0x1000 bootloader_dio_40m.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 %firmware% 0x3d0000 %spiffs%