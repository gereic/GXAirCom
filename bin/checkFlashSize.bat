set PORT=COM26
set esptooldir=C:\Users\gereic\AppData\Local\Programs\Python\Python310\Scripts\
REM set firmware=firmware_v1.6.0_esp32PSRam.bin
%esptooldir%esptool.exe --port %PORT% flash_id