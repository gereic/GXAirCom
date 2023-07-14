set variant=GsNoBluetoothSim7000
set addresses= 0x400d7ce1:0x3ffe8420 0x400d7f31:0x3ffe8470 0x4010c8de:0x3ffe84c0 0x4010a775:0x3ffe8510 0x4010a885:0x3ffe8560 0x4010aa6d:0x3ffe85b0 0x40107c81:0x3ffe85d0 0x40107d11:0x3ffe8600 0x401085d2:0x3ffe8620

set addr2linePath=C:\Users\GeraldEichler\.platformio\packages\toolchain-xtensa-esp32\bin\

%addr2linePath%xtensa-esp32-elf-addr2line.exe -fipC -e ..\.pio\build\%variant%\firmware.elf %addresses%
REM %addr2linePath%xtensa-esp32-elf-addr2line.exe -fipC -e ..\.pio\build\%variant%\firmware.elf %1