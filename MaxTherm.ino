/********************************************************************************************
 * 
 * Maximus Thermometricus
 * 
 * Reads temperature from ten DS18B20 sensors and
 * prints values to SSD1306 128x64 OLED display. 
 * Uses three independent OneWire channels. Any of the DS18B20 can be
 * connected to any of the OneWire channel (need restart after wiring).
 * Sensor binding is done using unique DS18B20 address.
 * Able to memorize sensor bindings to EEPROM (push button should be pressed once after wiring).
 * 
 * Schematics:
 * 10 DS18B20 connected to digital pins D2, D3 and D4.
 * 1306 128x64 OLED display via I2C connected to pins A4 (SDA) and A5 (SCL).
 * Button switch to memorize bindings connected to digital pin 10 and Gnd.
 * 
 * Note: 
 * 99 occurs when sensor disconnected after reset or other quering error (probably transient).
 * 88 occurs when memorized sensor was not found in network while startup.
 * 86 occurs on system startup and indicates that value has been read yet for this sensor.
 *
 * Note: system has to be restarted when sensor moved to different bus pins, 
 * or replace sensor with the new one.
 *
 * To bind:
 * For empty/noninitizlized EEPROM, just wire and push D10 to Gnd for at least 1 sec.
 * When need to replace sensor: 
 * 1) remove old one and add the new DS18B20 sensor
 * 2) push reset 
 * 3) wait for "88.0" value on display 
 * 4) push D10 to Gnd for at least 1 sec (Led on D13 will light). 
 * New sensor will bind to the old 88.0 value.
 *
 ********************************************************************************************/

#include "U8glib.h"
#include "OneWire.h"
#include <Bounce2.h>
#include <EEPROM.h>

//U8GLIB_SSD1306_128X64 u8g(13, 11, 10, 9);	// SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_SSD1306_128X64 u8g(4, 5, 6, 7);	// SW SPI Com: SCK = 4, MOSI = 5, CS = 6, A0 = 7 (new white HalTec OLED)
//U8GLIB_SSD1306_128X64 u8g(10, 9);		// HW SPI Com: CS = 10, A0 = 9 (Hardware Pins are  SCK = 13 and MOSI = 11)
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);	// I2C / TWI 
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST);	// Fast I2C / TWI 
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);	// Display which does not send AC
//U8GLIB_SSD1306_ADAFRUIT_128X64 u8g(13, 11, 10, 9);	// SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_SSD1306_ADAFRUIT_128X64 u8g(10, 9);		// HW SPI Com: CS = 10, A0 = 9 (Hardware Pins are  SCK = 13 and MOSI = 11)
//U8GLIB_SSD1306_128X32 u8g(13, 11, 10, 9);	// SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9
//U8GLIB_SSD1306_128X32 u8g(10, 9);             // HW SPI Com: CS = 10, A0 = 9 (Hardware Pins are  SCK = 13 and MOSI = 11)
//U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE);	// I2C / TWI 

// Use three OneWire channels to distribute sensors.
OneWire _ds[3] = {
  OneWire(2), OneWire(3), OneWire(4) };

unsigned long _sensLastUpdated[10] = { 
  0 };
byte _sensCurr = 255;
unsigned long _sensRead = 0;
int _sensUpdateInterval = 100; // in ms.
byte _sens[10][8] = { 
  0 };
byte _sensBus[10] = { 
  0 };

float _sensTemp[10] = { 
  86, 86, 86, 86, 86, 86, 86, 86, 86, 86 }; // Magic value for uninitialized values.
byte _sensCount = 0;

unsigned long _lastDisplayUpdated = 0;

struct eeStruct {
  byte amb1[8];
  byte amb2[8];
  byte boiler1[8];
  byte boiler2[8];
  byte boiler3[8];
  byte hPipe1[8];
  byte hPipe2[8];
  byte floor1[8];
  byte floor2[8];
  byte water1[8];
};
typedef struct eeStruct ee_t;
ee_t ee;

#define PIN_MEM 10
Bounce _memPin = Bounce(); 

void setup(void) {
  pinMode(13, OUTPUT);

  pinMode(PIN_MEM, INPUT_PULLUP);
  _memPin.attach(PIN_MEM);
  _memPin.interval(1000); // interval in ms

  Serial.begin(9600);

  searchSensors();
  EEPROM_readAnything(0, &ee, sizeof(ee));
  setDefaultBindings();
}

void loop(void) {
  protectMillisOverflow();
  updateTemp();
  updateDisplay();
  updateMem();
}

int lastMemValue = HIGH;

void updateMem()
{
  _memPin.update();

  // Get the updated value :
  int value = _memPin.read();

  if ( value == LOW && lastMemValue == HIGH) {
    digitalWrite(13, HIGH );
    memorizeBindings();
    EEPROM_writeAnything(0, &ee, sizeof(ee));
    delay(2000);
  }
  else {
    digitalWrite(13, LOW );
  }

  lastMemValue = value;
}

void updateDisplay()
{
  if ((millis() - _lastDisplayUpdated) > 1000)
  {
    _lastDisplayUpdated = millis();
    for (int i = 0; i < _sensCount; i++)
    {
      Serial.print(_sensTemp[i]);
      Serial.print(" ");
    }
    Serial.println();

    // picture loop
    u8g.firstPage();  
    do {
      u8g.setFont(u8g_font_6x13);

      char temp[10];
      char buf[22];

      String s = "Amb  : ";
      dtostrf(safeGetTemp(ee.amb1), 3, 1, temp);
      s += temp;
      dtostrf(safeGetTemp(ee.amb2), 3, 1, temp);
      s += "/";
      s += temp;
      s.toCharArray(buf, 22);
      u8g.drawStr(1,9, buf);

      s = "Boil : ";
      dtostrf(safeGetTemp(ee.boiler1), 3, 1, temp);
      s += temp;      
      dtostrf(safeGetTemp(ee.boiler2), 3, 1, temp);
      s += "/";
      s += temp;
      dtostrf(safeGetTemp(ee.boiler3), 3, 1, temp);
      s += "/";
      s += temp;
      s.toCharArray(buf, 22);
      u8g.drawStr(1,23, buf);

      s = "Hpipe: ";
      dtostrf(safeGetTemp(ee.hPipe1), 3, 1, temp);
      s += temp;      
      dtostrf(safeGetTemp(ee.hPipe2), 3, 1, temp);
      s += "/";
      s += temp;
      s.toCharArray(buf, 22);
      u8g.drawStr(1, 37, buf);

      s = "Floor: ";
      dtostrf(safeGetTemp(ee.floor1), 3, 1, temp);
      s += temp;      
      dtostrf(safeGetTemp(ee.floor2), 3, 1, temp);
      s += "/";
      s += temp;
      s.toCharArray(buf, 22);
      u8g.drawStr(1, 51, buf);

      s = "Water: ";
      dtostrf(safeGetTemp(ee.water1), 3, 1, temp);
      s += temp; 
      s.toCharArray(buf, 22);
      u8g.drawStr(1, 64, buf);

      u8g.setColorIndex(1);
    } 
    while( u8g.nextPage() );
  }
}

boolean validateAddr(const byte *addr)
{
  return OneWire::crc8(addr, 7) == addr[7];
}

void searchSensors()
{
  int i = 0;

  for (int n = 0; n < 3; n++)
  {
    _ds[n].reset_search();
    byte addr[8];
    while (_ds[n].search(addr))
    {
      if (validateAddr(addr)) // CRC is valid.
      {
        for (int j = 0; j < 8; j++)
          _sens[i][j] = addr[j];
        _sensBus[i] = n;
        i++;
      }
      else
      {
        Serial.println("CRC is not valid. Skip sensor.");
      }
    }
    _sensCount = i;
    _ds[n].reset_search();
  }

  Serial.print("Found ");
  Serial.println(_sensCount);  
}

boolean isValidBinding(int index)
{
  return (indexOfAddress(ee.amb1) == index
    || indexOfAddress(ee.amb2) == index
    || indexOfAddress(ee.boiler1) == index
    || indexOfAddress(ee.boiler2) == index
    || indexOfAddress(ee.boiler3) == index
    || indexOfAddress(ee.hPipe1) == index
    || indexOfAddress(ee.hPipe2) == index
    || indexOfAddress(ee.floor1) == index
    || indexOfAddress(ee.floor2) == index
    || indexOfAddress(ee.water1) == index);
}

void memorizeInvalidBinding(int index)
{
  if (indexOfAddress(ee.amb1) < 0)
  {
    memcpy(ee.amb1, _sens[index], 8);
  }
  else if (indexOfAddress(ee.amb2) < 0)
  {
    memcpy(ee.amb2, _sens[index], 8);
  }
  else if (indexOfAddress(ee.boiler1) < 0)
  {
    memcpy(ee.boiler1, _sens[index], 8);
  }
  else if (indexOfAddress(ee.boiler2) < 0)
  {
    memcpy(ee.boiler2, _sens[index], 8);
  }
  else if (indexOfAddress(ee.boiler3) < 0)
  {
    memcpy(ee.boiler3, _sens[index], 8);
  }
  else if (indexOfAddress(ee.hPipe1) < 0)
  {
    memcpy(ee.hPipe1, _sens[index], 8);
  }
  else if (indexOfAddress(ee.hPipe2) < 0)
  {
    memcpy(ee.hPipe2, _sens[index], 8);
  }
  else if (indexOfAddress(ee.floor1) < 0)
  {
    memcpy(ee.floor1, _sens[index], 8);
  }
  else if (indexOfAddress(ee.floor2) < 0)
  {
    memcpy(ee.floor2, _sens[index], 8);
  }
  else if (indexOfAddress(ee.water1) < 0)
  {
    memcpy(ee.water1, _sens[index], 8);
  }
}

void memorizeBindings()
{
  boolean hasInvalidBindings = true;
  while (hasInvalidBindings) 
  {
    for (int i = 0; i < _sensCount; i++)
    {
      if (!isValidBinding(i))
      {
        memorizeInvalidBinding(i);
        break;
      }
    }
    
    hasInvalidBindings = false;
  }
}

void setDefaultBindings()
{
  byte empty[8] = {
    0                  };

  if (validateAddr(ee.amb1))
    memcpy(ee.amb1, _sensCount >= 1 ? _sens[0] : empty, 8);

  if (!validateAddr(ee.amb2))
    memcpy(ee.amb2, _sensCount >= 2 ? _sens[1] : empty, 8);

  if (!validateAddr(ee.boiler1))
    memcpy(ee.boiler1, _sensCount >= 3 ? _sens[2] : empty, 8);

  if (!validateAddr(ee.boiler2))
    memcpy(ee.boiler2, _sensCount >= 4 ? _sens[3] : empty, 8);  

  if (!validateAddr(ee.boiler3))
    memcpy(ee.boiler3, _sensCount >= 5 ? _sens[4] : empty, 8);

  if (!validateAddr(ee.hPipe1))
    memcpy(ee.hPipe1, _sensCount >= 6 ? _sens[5] : empty, 8);

  if (!validateAddr(ee.hPipe2))
    memcpy(ee.hPipe2, _sensCount >= 7 ? _sens[6] : empty, 8);

  if (!validateAddr(ee.floor1))
    memcpy(ee.floor1, _sensCount >= 8 ? _sens[7] : empty, 8);

  if (!validateAddr(ee.floor2))
    memcpy(ee.floor2, _sensCount >= 9 ? _sens[8] : empty, 8);

  if (!validateAddr(ee.water1))
    memcpy(ee.water1, _sensCount >= 10 ? _sens[9] : empty, 8);
}

boolean isEqualsAddr(const byte *x, const byte *y)
{
  boolean theSame = true;
  for (int j = 0; j < 8; j++)
  {
    if (x[j] != y[j])
    {
      theSame = false;
      break;
    }
  }

  return theSame;
}

int indexOfAddress(const byte *a)
{
  for (int i = 0; i < _sensCount; i++)
  {
    boolean theSame = isEqualsAddr(_sens[i], a);
    if (theSame)
      return i;
  }

  return -1;
}

float safeGetTemp(const byte *a)
{
  int idx = indexOfAddress(a);
  if (idx >= 0)
    return _sensTemp[idx];

  return 88.0;
}

// TODO: Take performance advantage for independent OneWire channels.
void updateTemp()
{
  for (int sensNum = 0; sensNum < _sensCount; sensNum++)
  {
    if (_sensCurr != 255 && _sensCurr != sensNum)
      continue;

    unsigned long time = millis();

    if (_sensCurr == 255 && (time - _sensLastUpdated[sensNum]) < _sensUpdateInterval)
      continue;

    if (_sensCurr == 255)
    {
      _ds[_sensBus[sensNum]].reset();
      _ds[_sensBus[sensNum]].select(_sens[sensNum]);
      _ds[_sensBus[sensNum]].write(0x44, 1);

      // lock bus
      _sensRead = time;
      _sensCurr = sensNum;
    }
    else if (_sensCurr == sensNum && (time - _sensRead) >= 1000)
    {
      byte present = _ds[_sensBus[sensNum]].reset();
      _ds[_sensBus[sensNum]].select(_sens[sensNum]);
      _ds[_sensBus[sensNum]].write(0xBE); // Read Scratchpad

      byte data[9];
      for (int i = 0; i < 9; i++) { // we need 9 bytes
        data[i] = _ds[_sensBus[sensNum]].read();
      }

      byte computedCrc = OneWire::crc8( data, 8);
      if (computedCrc != data[8])
      {
        _sensLastUpdated[sensNum] = time - _sensUpdateInterval / 2;
        _sensTemp[sensNum] = 99.0;

        // release bus
        _sensCurr = 255;
        _sensRead = 0;
        continue;
      }

      // convert the data to actual temperature
      unsigned int raw = (data[1] << 8) | data[0];
      byte cfg = (data[4] & 0x60);
      if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
      // default is 12 bit resolution, 750 ms conversion time

      _sensTemp[sensNum] = (float)raw / 16.0;
      _sensLastUpdated[sensNum] = time;

      // release bus
      _sensCurr = 255;
      _sensRead = 0;
    }
  }
}

unsigned long _lastMillis = 0;
void protectMillisOverflow()
{
  unsigned int currMillis = millis();
  if (_lastMillis > currMillis) 
  {
    // Millis overflow.
    _lastDisplayUpdated = 0;
    _sensRead = 0;
    for (int i = 0; i < _sensCount; i++)
    {
      _sensLastUpdated[i] = 0;
    }
  }
  _lastMillis = currMillis;
}

int EEPROM_writeAnything(int ee, const void* value, int sizeValue)
{
  const byte* p = (const byte*)value;
  int i;
  for (i = 0; i < sizeValue; i++)
    EEPROM.write(ee++, *p++);
  return i;
}

int EEPROM_readAnything(int ee, void* value, int sizeValue)
{
  byte* p = (byte*)value;
  int i;
  for (i = 0; i < sizeValue; i++)
    *p++ = EEPROM.read(ee++);
  return i;
}















