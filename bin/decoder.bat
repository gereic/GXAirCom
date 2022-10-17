set variant=nopsRam
set addresses=0x400014dc:0x3ffb8a90 0x401435ed:0x3ffb8aa0 0x40148cd6:0x3ffb8db0 0x40148d12:0x3ffb8e40 0x4012553e:0x3ffb8e80 0x400ee989:0x3ffb8ee0 0x400eebdc:0x3ffb8fd0 0x40091fca:0x3ffb9000

set addr2linePath=C:\Users\gereic\.platformio\packages\toolchain-xtensa32\bin\

%addr2linePath%xtensa-esp32-elf-addr2line.exe -fipC -e ..\.pio\build\%variant%\firmware.elf %addresses%
REM %addr2linePath%xtensa-esp32-elf-addr2line.exe -fipC -e ..\.pio\build\%variant%\firmware.elf %1