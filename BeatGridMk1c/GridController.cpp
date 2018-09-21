// ----------------------------------------------------------------------------
// 8x8 GridController, based on the Adafruit Trellis
// driver, adapted for SoftWire and Leonardo

// For Leonardo:
#define SDA_PORT PORTD
#define SDA_PIN 1
#define SCL_PORT PORTD
#define SCL_PIN 0

#define I2C_TIMEOUT 100
#define I2C_FASTMODE 1
     
#include <SoftWire.h>
SoftWire Wire = SoftWire();

#include "GridController.h"

#define HT16K33_BLINK_CMD       0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_CMD_BRIGHTNESS  0xE0

/*
These are the lookup tables that convert the LED/button #
to the memory address in the HT16K33 - don't mess with them :)
*/

static const uint8_t PROGMEM
  ledLUT[16] =
    { 0x3A, 0x37, 0x35, 0x34,
      0x28, 0x29, 0x23, 0x24,
      0x16, 0x1B, 0x11, 0x10,
      0x0E, 0x0D, 0x0C, 0x02 },
  buttonLUT[16] =
    { 0x07, 0x04, 0x02, 0x22,
      0x05, 0x06, 0x00, 0x01,
      0x03, 0x10, 0x30, 0x21,
      0x13, 0x12, 0x11, 0x31 };

Grid8x8::Grid8x8(void) {
}

void Grid8x8::begin(uint8_t _addr = 0x70, uint8_t _brightness=15) {
  i2c_addr = _addr;

  Wire.begin();

  Wire.beginTransmission(i2c_addr);
  Wire.write(0x21);  // turn on oscillator
  Wire.endTransmission();
  blinkRate(HT16K33_BLINK_OFF);
  
  setBrightness(_brightness);

  Wire.beginTransmission(i2c_addr);
  Wire.write(0xA1);  // turn on interrupt, active low
  Wire.endTransmission();
}

/* 
Helper button functions, the data is updated every readSwitches() call!
*/

bool Grid8x8::isKeyPressed(uint8_t k) {
  if (k > 15) return false;
  k = pgm_read_byte(&buttonLUT[k]);
  return (keys[k>>4] & _BV(k & 0x0F));
}

bool Grid8x8::wasKeyPressed(uint8_t k) {
  if (k > 15) return false;
  k = pgm_read_byte(&buttonLUT[k]);
  return (lastkeys[k>>4] & _BV(k & 0x0F));
}

boolean Grid8x8::justPressed(uint8_t k) {
  return (isKeyPressed(k) & !wasKeyPressed(k));
}
boolean Grid8x8::justReleased(uint8_t k) {
  return (!isKeyPressed(k) & wasKeyPressed(k));
}

/* 
Helper LED functions, the data is written on writeDisplay()
*/


boolean Grid8x8::isLED(uint8_t x) {
  if (x > 15) return false;
  x = pgm_read_byte(&ledLUT[x]);
  return ((displaybuffer[x >> 4] & _BV(x & 0x0F)) > 0);
}
void Grid8x8::setLED(uint8_t x) {
  if (x > 15) return;
  x = pgm_read_byte(&ledLUT[x]);
  displaybuffer[x >> 4] |= _BV(x & 0x0F);
}
void Grid8x8::clrLED(uint8_t x) {
  if (x > 15) return;
  x = pgm_read_byte(&ledLUT[x]);
  displaybuffer[x >> 4] &= ~_BV(x & 0x0F);
}


/* 
   Gets the switch memory data and updates the last/current read
*/

boolean Grid8x8::readSwitches(void) {
  memcpy(lastkeys, keys, sizeof(keys));

  Wire.beginTransmission((byte)i2c_addr);
  Wire.write(0x40);
  Wire.endTransmission();
  Wire.requestFrom((byte)i2c_addr, (byte)6);
  for (uint8_t i=0; i<6; i++) 
    keys[i] = Wire.read();

  for (uint8_t i=0; i<6; i++) {
    if (lastkeys[i] != keys[i]) {
       for (uint8_t j=0; j<6; j++) {
   //Serial.print(keys[j], HEX); Serial.print(" ");
       }
       //Serial.println();
      return true;
    }
  }
  return false;
}

void Grid8x8::setBrightness(uint8_t b) {
  if (b > 15) b = 15;
  Wire.beginTransmission(i2c_addr);
  Wire.write(HT16K33_CMD_BRIGHTNESS | b);
  Wire.endTransmission();  
}

void Grid8x8::blinkRate(uint8_t b) {
  Wire.beginTransmission(i2c_addr);
  if (b > 3) b = 0; // turn off if not sure
  
  Wire.write(HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1)); 
  Wire.endTransmission();
}


void Grid8x8::writeDisplay(void) {
  Wire.beginTransmission(i2c_addr);
  Wire.write((uint8_t)0x00); // start at address $00

  for (uint8_t i=0; i<8; i++) {
    Wire.write(displaybuffer[i] & 0xFF);    
    Wire.write(displaybuffer[i] >> 8);    
  }
  Wire.endTransmission();  
}

void Grid8x8::clear(void) {
  memset(displaybuffer, 0, sizeof(displaybuffer));
}


// ----------------------------------------------------------------------------

GridController::GridController( Grid8x8 *matrix0, 
                  				      Grid8x8 *matrix1,
				                        Grid8x8 *matrix2,
				                        Grid8x8 *matrix3) {
  matrices[0] = matrix0;
  matrices[1] = matrix1;
  matrices[2] = matrix2;
  matrices[3] = matrix3;
}


void GridController::begin( uint8_t addr0, uint8_t addr1,
                    				uint8_t addr2, uint8_t addr3) {
  matrices[0]->begin(addr0, 15);
  matrices[1]->begin(addr1, 15);
  matrices[2]->begin(addr2, 15);
  matrices[3]->begin(addr3, 15);
}

/* 
Helper button functions, the data is updated every readSwitches() call!
*/

bool GridController::isKeyPressed(uint8_t k) {
  if (k > 63) return false;
  uint8_t matrix, key;

  // determine submatrix #
  matrix = k / 16;
  key = k % 16;

  return  matrices[matrix]->isKeyPressed(key);
}

bool GridController::wasKeyPressed(uint8_t k) {
  if (k > 63) return false;
  uint8_t matrix, key;
  
  // determine submatrix #
  matrix = k / 16;
  key = k % 16;

  return  matrices[matrix]->wasKeyPressed(key);
}

boolean GridController::justPressed(uint8_t k) {
  return (isKeyPressed(k) & !wasKeyPressed(k));
}
boolean GridController::justReleased(uint8_t k) {
  return (!isKeyPressed(k) & wasKeyPressed(k));
}

/* 
Helper LED functions, the data is written on writeDisplay()
*/


boolean GridController::isLED(uint8_t x) {
  if (x > 63) return false;
  uint8_t matrix, led;
  
  // determine submatrix #
  matrix = x / 16;
  led = x % 16;

  return  matrices[matrix]->isLED(led);
}

void GridController::setLED(uint8_t x) {
  if (x > 63) return ;
  uint8_t matrix, led;
  
  // determine submatrix #
  matrix = x / 16;
  led = x % 16;

  matrices[matrix]->setLED(led);
}

void GridController::clrLED(uint8_t x) {
  if (x > 63) return ;
  uint8_t matrix, led;
  
  // determine submatrix #
  matrix = x / 16;
  led = x % 16;

  matrices[matrix]->clrLED(led);
}


/* 
   Gets the switch memory data and updates the last/current read
*/

boolean GridController::readPatternSwitches(void) {
  boolean changed = false;
  for (uint8_t i=0; i<2; i++) {
    if (matrices[i] != 0)
      changed = changed || matrices[i]->readSwitches();
  }
  return changed;
}

boolean GridController::readControlSwitches(void) {
  boolean changed = false;
  for (uint8_t i=2; i<4; i++) {
    if (matrices[i] != 0)
      changed = changed || matrices[i]->readSwitches();
  }
  return changed;
}

boolean GridController::readSwitches(void) {
  boolean changed = false;
  for (uint8_t i=0; i<4; i++) {
    if (matrices[i] != 0)
      changed = changed || matrices[i]->readSwitches();
  }
  return changed;
}

void GridController::setBrightness(uint8_t b) {
 for (uint8_t i=0; i<4; i++) {
   if (matrices[i] != 0)
     matrices[i]->setBrightness(b);
 }
}

void GridController::blinkRate(uint8_t b) {
 for (uint8_t i=0; i<4; i++) {
   if (matrices[i] != 0)
     matrices[i]->blinkRate(b);
 }
}


void GridController::writeDisplay(void) {
 for (uint8_t i=0; i<4; i++) {
   if (matrices[i] != 0)
     matrices[i]->writeDisplay();
 } 
}

void GridController::clear(void) {
 for (uint8_t i=0; i<4; i++) {
   if (matrices[i] != 0)
     matrices[i]->clear();
 }
}

// I've got a thing for tables, despite size.  Fast constant-time lookup.
static const uint8_t PROGMEM
  i2xy64[] = { // Remap 8x8 TrellisSet button index to column/row
    0x00, 0x10, 0x20, 0x30, 0x01, 0x11, 0x21, 0x31,
    0x02, 0x12, 0x22, 0x32, 0x03, 0x13, 0x23, 0x33,
    0x40, 0x50, 0x60, 0x70, 0x41, 0x51, 0x61, 0x71,
    0x42, 0x52, 0x62, 0x72, 0x43, 0x53, 0x63, 0x73,
    0x04, 0x14, 0x24, 0x34, 0x05, 0x15, 0x25, 0x35,
    0x06, 0x16, 0x26, 0x36, 0x07, 0x17, 0x27, 0x37,
    0x44, 0x54, 0x64, 0x74, 0x45, 0x55, 0x65, 0x75,
    0x46, 0x56, 0x66, 0x76, 0x47, 0x57, 0x67, 0x77 },
  xy2i64[8][8] = { // Remap [row][col] to Trellis button/LED index
    {  0,  1,  2,  3, 16, 17, 18, 19 },
    {  4,  5,  6,  7, 20, 21, 22, 23 },
    {  8,  9, 10, 11, 24, 25, 26, 27 },
    { 12, 13, 14, 15, 28, 29, 30, 31 },
    { 32, 33, 34, 35, 48, 49, 50, 51 },
    { 36, 37, 38, 39, 52, 53, 54, 55 },
    { 40, 41, 42, 43, 56, 57, 58, 59 },
    { 44, 45, 46, 47, 60, 61, 62, 63 } };

uint8_t GridController::xy2i(uint8_t x, uint8_t y) {
	if(x > 7) return 255;
	return pgm_read_byte(&xy2i64[y][x]);
}

void GridController::i2xy(uint8_t i, uint8_t *x, uint8_t *y) {
	if(i > 63) {
		*x = *y = 255;
		return;
	}
	uint8_t xy = pgm_read_byte( &i2xy64[i]);
	*x = xy >> 4;
	*y = xy & 15;
}


