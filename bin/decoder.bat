set variant=sim800_nopsRam
set addresses=0x401c2784:0x3ffdbd00 0x401b75d3:0x3ffdbd20 0x40103bdf:0x3ffdbd50 0x40103dc5:0x3ffdbd90 0x400e8d25:0x3ffdbdb0 0x40091fca:0x3ffdc2e0

set addr2linePath=C:\Users\gereic\.platformio\packages\toolchain-xtensa32\bin\

%addr2linePath%xtensa-esp32-elf-addr2line.exe -fipC -e ..\.pio\build\%variant%\firmware.elf %addresses%
REM %addr2linePath%xtensa-esp32-elf-addr2line.exe -fipC -e ..\.pio\build\%variant%\firmware.elf %1