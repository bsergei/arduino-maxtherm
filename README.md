# arduino-maxtherm

Maximus Thermometricus. Shows ten DS18B20 values on OLED display
Reads temperature from ten DS18B20 sensors and
prints values to SSD1306 128x64 OLED display.
Uses three independent OneWire channels. Any of the DS18B20 can be
connected to any of the OneWire channel (need restart after wiring).
Sensor binding is done using unique DS18B20 address.
Able to memorize sensor bindings to EEPROM (push button should be pressed once after wiring).

Schematics:
10 DS18B20 connected to digital pins D2, D3 and D4.
1306 128x64 OLED display via I2C connected to pins A4 (SDA) and A5 (SCL).
Button switch to memorize bindings connected to digital pin 10 and Gnd.

Note:
99 occurs when sensor disconnected after reset or other quering error (probably transient).
88 occurs when memorized sensor was not found in network while startup.
86 occurs on system startup and indicates that value has been read yet for this sensor.

Note: system has to be restarted when sensor moved to different bus pins,
or replace sensor with the new one.

To bind sensors:
For empty/noninitizlized EEPROM, just wire and push D10 to Gnd for at least 1 sec.
When need to replace sensor:
1) remove old one and add the new DS18B20 sensor
2) push reset
3) wait for "88.0" value on display
4) push D10 to Gnd for at least 1 sec (Led on D13 will light).
New sensor will bind to the old 88.0 value.
