// ----------------------------------------------------------------------------
// 8x8 GridController, based on the Adafruit Trellis driver and UNTZtrument.
// - Removed encoder code.
// - Defines classes Grid8x8 and GridController. GridController has 2 separate
//   methods to read the pattern and control switches (= upper and lower part)

#ifndef _GRIDCONTROLLER_H_
#define _GRIDCONTROLLER_H_

// Set _BV if not already set (eg. Arduino DUE, Arduino Zero Pro)
#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

#define LED_ON  1
#define LED_OFF 0

#define HT16K33_BLINK_OFF    0
#define HT16K33_BLINK_2HZ    1
#define HT16K33_BLINK_1HZ    2
#define HT16K33_BLINK_HALFHZ 3


// ----------------------------------------------------------------------------
// Trellis driver, based on Adafruit Trellis

class Grid8x8 {
 public:
  Grid8x8(void);
  void begin(uint8_t _addr, uint8_t _brightness);
  void setBrightness(uint8_t b);
  void blinkRate(uint8_t b);
  void writeDisplay(void);
  void clear(void);
  bool isKeyPressed(uint8_t k);
  bool wasKeyPressed(uint8_t k);
  boolean isLED(uint8_t x);
  void setLED(uint8_t x);
  void clrLED(uint8_t x);
  boolean readSwitches(void);
  boolean justPressed(uint8_t k);
  boolean justReleased(uint8_t k);

  uint16_t displaybuffer[8];

  void init(uint8_t a);

 private:
  uint8_t keys[6], lastkeys[6];
  uint8_t i2c_addr;
};


// ----------------------------------------------------------------------------
class GridController {
 public:
  GridController( Grid8x8* matrix0, 
		              Grid8x8* matrix1=NULL,
	        	      Grid8x8* matrix2=NULL,
	        	      Grid8x8* matrix3=NULL );
  void begin( uint8_t _addr0 = 0x70, uint8_t _addr1 = 0x71,
	            uint8_t _addr2 = 0x72, uint8_t _addr3 = 0x73 );

  void setBrightness(uint8_t b);
  void blinkRate(uint8_t b);
  void writeDisplay(void);
  void clear(void);
  bool isKeyPressed(uint8_t k);
  bool wasKeyPressed(uint8_t k);
  boolean isLED(uint8_t x);
  void setLED(uint8_t x);
  void clrLED(uint8_t x);
  boolean readSwitches(void);
  boolean readPatternSwitches(void);
  boolean readControlSwitches(void);
  boolean justPressed(uint8_t k);
  boolean justReleased(uint8_t k);

  static uint8_t xy2i(uint8_t x, uint8_t y);
  static void    i2xy(uint8_t i, uint8_t *x, uint8_t *y);

 private:
  Grid8x8* matrices[4];
};

#endif // _GRIDCONTROLLER_H_
