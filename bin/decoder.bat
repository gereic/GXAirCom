set variant=nopsRam
set addresses= 0x401b6938:0x3ffb2c60 0x401b6c19:0x3ffb2c90 0x401b6fcb:0x3ffb2cb0 0x40116b83:0x3ffb2ce0 0x400e77ca:0x3ffb2d20 0x40092032:0x3ffb3290

set addr2linePath=C:\Users\gereic\.platformio\packages\toolchain-xtensa32\bin\

%addr2linePath%xtensa-esp32-elf-addr2line.exe -fipC -e ..\.pio\build\%variant%\firmware.elf %addresses%
REM %addr2linePath%xtensa-esp32-elf-addr2line.exe -fipC -e ..\.pio\build\%variant%\firmware.elf %1