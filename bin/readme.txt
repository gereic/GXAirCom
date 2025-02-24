************* Pins for TTGO-TBeam V1.0/V1.1 ***************
******* LORA ********
SCK 5
MISO 19
MOSI 27
SS 18
RST 23
DIO0 26

******* AXP192 *******
SDA 21
SCL 22 
******* OLED ********
SDA 21
SCL 22 

******* E-Ink *********
BUSY     33
RST      VCC
DC       32
CS       15
CLK      4
DIN      2

******** BARO *********
SDA 13
SCL 14 

******** GPS *********
TXD 34 
RXD 12

******** BUZZER *******
BUZZER 25

******** FUEL-SENSOR *******
FUEL-SENSO 39

******** external Power Switch (only for direct use with kobo to switch on an off automatically) *********
PinExtPowerOnOff 36

******** Anemometer (Davis 6410) *********
WINDDIR_PIN 36
WINDSPEED_PIN 39  (use external Pull 4,7k)

******** Anemometer (TX20) *********
DATA_PIN 39

******** Anemometer (Peet Bros) *********
WINDDIR_PIN 36  (use external Pull 10k)
WINDSPEED_PIN 39  (use external Pull 10k)




************* Pins for TTGO-TBeam V07 ***************
******* LORA ********
SCK 5
MISO 19
MOSI 27
SS 18
RST 23
DIO0 26

******* OLED ********
SDA 21
SCL 22 

******* E-Ink *********
BUSY     33
RST      VCC
DC       32
CS       15
CLK      4
DIN      2

******** BARO *********
SDA 13
SCL 14 

******** BUZZER *******
BUZZER 25

******** FUEL-SENSOR *******
FUEL-SENSO 39


************* Pins for TTGO Lora32 ***************
******* BUTTON ********
BUTTON_PIN 0

******* LED *********
LED_PIN 2


******* LORA ********
SCK 5
MISO 19
MOSI 27
SS 18
RST 14
DIO0 26

******* OLED ********
SDA 4
SCL 15
RST 16

******** BARO *********
SDA 13
SCL 23 

******** GPS *********
TXD 12 
RXD 15

******** BUZZER *******
BUZZER 17

******** BATTERY-Voltate *******
                       ____           ____
BATT_VOLT_PIN 34   ---|____|----|----|____|-----
                  GND  100K   GPIO34   27K     4.2V

******** Temperature-sensor DS18B20 for Ground-station *********
TEMP_PIN 22 external 4,7k Pull-up

******** Anemometer (Davis 6410) *********
GND (red)
3V3 (yellow)
WINDDIR_PIN 36 (green)
WINDSPEED_PIN 37  (use external Pull 4,7k) (black)

******** Anemometer (TX20) *********
DATA_PIN 37

******** Anemometer (Peet Bros) *********
WINDDIR_PIN 36  (use external Pull 10K)
WINDSPEED_PIN 37  (use external Pull 10K)



******** RAIN-Sensor VENTUS W174 *********
RAIN_PIN 38   (need 10K Pull-Up-Resistor) Bucket-Size 0.5l

******** SIM800l module *********
TXD 12
RXD 21
RESET 25

******** FUEL-SENSOR *******
FUEL-SENSO 39



************* Pins for TTGO Lora32 V2.1 1.6 ***************
******* LORA ********
SCK 5
MISO 19
MOSI 27
SS 18
RST 23
DIO0 26

******* OLED ********
SDA 21
SCL 22 

******** BARO *********
SDA 13
SCL 14 

******** GPS *********
TXD 34 
RXD 12

******** BUZZER *******
BUZZER 0

******** FUEL-SENSOR *******
FUEL-SENSO 39


************* Pins for TTGO-TSIM7000G ***************

******* E-Ink *********
BUSY     39  (if e-ink is selected, rain-sensor is not supported)
RST      25  (if e-ink is selected, ds18b20 is not supported)
DC       15
CS       13
CLK      14
DIN      2


******* LORA ********
SCK 18
MISO 19
MOSI 23
SS 5
RST 12
DIO0 32

******** GPS *********
TXD 27 
RXD 26

******* OLED ********
NO OLED-Support YET

******** BARO *********
SDA 21
SCL 22 

******** Temperature-sensor DS18B20 for Ground-station *********
TEMP_PIN 25  (if e-ink is selected, no ds18b20 is supported) external 4,7k Pull-up

******** Anemometer (Davis 6410) *********
WINDDIR_PIN 33
WINDSPEED_PIN 34  (use external Pull 4,7k)

******** Anemometer (TX20) *********
DATA_PIN 34

******** Anemometer (Peet Bros) *********
WINDDIR_PIN 33  (use external Pull 10K)
WINDSPEED_PIN 34  (use external Pull 10K)


******** RAIN-Sensor VENTUS W174 *********
RAIN_PIN 39   (need 10K Pull-Up-Resistor) Bucket-Size 0.5l
              (if e-ink is selected, rain-sensor is not supported)

******** BATTERY-Voltate *******
BATT_VOLT_PIN 35

************* Pins for Heltec Wireless-Stick-Lite V3 ***************

******** BARO *********
SDA 33
SCL 34 

******** Anemometer (Davis 6410) *********
WINDDIR_PIN 2
WINDSPEED_PIN 3  (use external Pull 4,7k)

************* Pins for HELTEC_LORA_V3 ***************

******** BARO *********
SDA 2
SCL 3 

******** Anemometer (Davis 6410) *********
WINDDIR_PIN 6
WINDSPEED_PIN 7  (use external Pull 4,7k)





