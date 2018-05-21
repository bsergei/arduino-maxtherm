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
  byte sens_a[8];
  byte sens_b[8];
  byte sens_c[8];
  byte sens_d[8];
  byte sens_e[8];
  byte sens_f[8];
  byte sens_g[8];
  byte sens_h[8];
  byte sens_i[8];
  byte sens_j[8];
};
typedef struct eeStruct ee_t;
ee_t ee;

#define PIN_MEM 10
#define PIN_RESET 9
Bounce _memPin = Bounce();

void setup(void) {
  pinMode(13, OUTPUT);

  pinMode(PIN_MEM, INPUT_PULLUP);
  _memPin.attach(PIN_MEM);
  _memPin.interval(1000); // interval in ms

  pinMode(PIN_RESET, INPUT_PULLUP);
  delay(10);
    
  Serial.begin(9600);

  if (digitalRead(PIN_RESET) == LOW) {
   resetEEPROM(); 
   Serial.println("EEPROM reset");
  }

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

void resetEEPROM() 
{
  byte x[80];
  memset(x, 0, 80);
  EEPROM_writeAnything(0, x, 80);
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

      String s = "A=";
      dtostrf(safeGetTemp(ee.sens_a), 7, 2, temp);
      s += temp;      
      s += "  B=";
      dtostrf(safeGetTemp(ee.sens_b), 7, 2, temp);
      s += temp;
      s.toCharArray(buf, 22);
      Serial.println(buf);
      u8g.drawStr(1,9, buf);

      s = "C=";
      dtostrf(safeGetTemp(ee.sens_c), 7, 2, temp);
      s += temp;            
      s += "  D=";
      dtostrf(safeGetTemp(ee.sens_d), 7, 2, temp);
      s += temp;
      s.toCharArray(buf, 22);
      Serial.println(buf);
      u8g.drawStr(1,23, buf);

      s = "E=";
      dtostrf(safeGetTemp(ee.sens_e), 7, 2, temp);
      s += temp;
      s += "  F=";
      dtostrf(safeGetTemp(ee.sens_f), 7, 2, temp);
      s += temp;
      s.toCharArray(buf, 22);
      Serial.println(buf);
      u8g.drawStr(1, 37, buf);

      s = "G=";
      dtostrf(safeGetTemp(ee.sens_g), 7, 2, temp);
      s += temp;
      s += "  H=";
      dtostrf(safeGetTemp(ee.sens_h), 7, 2, temp);
      s += temp;
      s.toCharArray(buf, 22);
      Serial.println(buf);
      u8g.drawStr(1, 51, buf);

      s = "I=";
      dtostrf(safeGetTemp(ee.sens_i), 7, 2, temp);
      s += temp;
      s += "  J=";
      dtostrf(safeGetTemp(ee.sens_j), 7, 2, temp);
      s += temp;
      s.toCharArray(buf, 22);
      Serial.println(buf);
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

  delay(250);
}

boolean isValidBinding(int index)
{
  return (indexOfAddress(ee.sens_a) == index
    || indexOfAddress(ee.sens_b) == index
    || indexOfAddress(ee.sens_c) == index
    || indexOfAddress(ee.sens_d) == index
    || indexOfAddress(ee.sens_e) == index
    || indexOfAddress(ee.sens_f) == index
    || indexOfAddress(ee.sens_g) == index
    || indexOfAddress(ee.sens_h) == index
    || indexOfAddress(ee.sens_i) == index
    || indexOfAddress(ee.sens_j) == index);
}

void memorizeInvalidBinding(int index)
{
  if (indexOfAddress(ee.sens_a) < 0)
  {
    memcpy(ee.sens_a, _sens[index], 8);
  }
  else if (indexOfAddress(ee.sens_b) < 0)
  {
    memcpy(ee.sens_b, _sens[index], 8);
  }
  else if (indexOfAddress(ee.sens_c) < 0)
  {
    memcpy(ee.sens_c, _sens[index], 8);
  }
  else if (indexOfAddress(ee.sens_d) < 0)
  {
    memcpy(ee.sens_d, _sens[index], 8);
  }
  else if (indexOfAddress(ee.sens_e) < 0)
  {
    memcpy(ee.sens_e, _sens[index], 8);
  }
  else if (indexOfAddress(ee.sens_f) < 0)
  {
    memcpy(ee.sens_f, _sens[index], 8);
  }
  else if (indexOfAddress(ee.sens_g) < 0)
  {
    memcpy(ee.sens_g, _sens[index], 8);
  }
  else if (indexOfAddress(ee.sens_h) < 0)
  {
    memcpy(ee.sens_h, _sens[index], 8);
  }
  else if (indexOfAddress(ee.sens_i) < 0)
  {
    memcpy(ee.sens_i, _sens[index], 8);
  }
  else if (indexOfAddress(ee.sens_j) < 0)
  {
    memcpy(ee.sens_j, _sens[index], 8);
  }
}

void memorizeBindings()
{
  boolean hasInvalidBindings = true;
  while (hasInvalidBindings) 
  {
    hasInvalidBindings = false;
    for (int i = 0; i < _sensCount; i++)
    {
      if (!isValidBinding(i))
      {
        memorizeInvalidBinding(i);
        hasInvalidBindings = true;
        break;
      }
    }
  }
}

void setDefaultBindings()
{
  byte empty[8];
  memset(empty, 0, 8);

  if (!validateAddr(ee.sens_a))
    memcpy(ee.sens_a, _sensCount >= 1 ? _sens[0] : empty, 8);

  if (!validateAddr(ee.sens_b))
    memcpy(ee.sens_b, _sensCount >= 2 ? _sens[1] : empty, 8);

  if (!validateAddr(ee.sens_c))
    memcpy(ee.sens_c, _sensCount >= 3 ? _sens[2] : empty, 8);

  if (!validateAddr(ee.sens_d))
    memcpy(ee.sens_d, _sensCount >= 4 ? _sens[3] : empty, 8);  

  if (!validateAddr(ee.sens_e))
    memcpy(ee.sens_e, _sensCount >= 5 ? _sens[4] : empty, 8);

  if (!validateAddr(ee.sens_f))
    memcpy(ee.sens_f, _sensCount >= 6 ? _sens[5] : empty, 8);

  if (!validateAddr(ee.sens_g))
    memcpy(ee.sens_g, _sensCount >= 7 ? _sens[6] : empty, 8);

  if (!validateAddr(ee.sens_h))
    memcpy(ee.sens_h, _sensCount >= 8 ? _sens[7] : empty, 8);

  if (!validateAddr(ee.sens_i))
    memcpy(ee.sens_i, _sensCount >= 9 ? _sens[8] : empty, 8);

  if (!validateAddr(ee.sens_j))
    memcpy(ee.sens_j, _sensCount >= 10 ? _sens[9] : empty, 8);
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
      byte cfg = (data[4] & 0x60);

      uint8_t resolution;
      if (cfg == 0x00) resolution = 9;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) resolution = 10; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) resolution = 11; // 11 bit res, 375 ms
      else resolution = 12; // default is 12 bit resolution, 750 ms conversion time

      _sensTemp[sensNum] = convertToCelsius(data[0], data[1], resolution);
      _sensLastUpdated[sensNum] = time;

      // release bus
      _sensCurr = 255;
      _sensRead = 0;
    }
  }
}

float convertToCelsius(int lowByte, int highByte, byte resolution)
{
  float quality[] = {0.5, 0.25, 0.125, 0.0625};
  uint8_t shift[] = {3, 2, 1, 0};
  int16_t raw = word(highByte, lowByte);
  raw >>= shift[resolution - 9];

  return raw * quality[resolution - 9];
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















