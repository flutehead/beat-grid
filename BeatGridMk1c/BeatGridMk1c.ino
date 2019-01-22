// ----------------------------------------------------------------------------
// BeatGrid: An Adafruit Trellis based grid sequencer
// mk1 - Jörg Ockel, October 2017
//
// Building on:
// - Mozzi sound library
// - 909 samples used in fakebitpolytechnic's cheapsynth
// - SoftWire library to be able to work with Arduino Leonardo
// ----------------------------------------------------------------------------

// Save pattern to EEPROM
#include <EEPROM.h>

// Adafruit UNTZtrument, adapted to SoftWire library
#include "GridController.h"

// Separate output of beat sync: Port D pin 4 = D4
// D4 works both on the Arduino Uno and Leonardo
#define SYNC_PORT PORTD
#define SYNC_MASK B00010000

// Use Mozzi library for sound generation
#include <MozziGuts.h>
#include <Sample.h>
#include <tables/saw2048_int8.h>
#include <tables/triangle2048_int8.h>
#include <tables/triangle512_int8.h>
#include <Oscil.h>
#include <EventDelay.h>
#include <ADSR.h>
#include <LowPassFilter.h>

// Drum samples from fakebitpolytechnic's cheapsynth, stripped to the bare
// minimum of samples. Added a snare reverse sample.
#include "kick909short.h"
#include "snare909short.h"
#include "hihatc909short.h"
#include "hihato909short.h"
#include "snare909reverse.h"

// ----------------------------------------------------------------------------

// Mozzi control update rate [Hz], powers of 2, >= 64
#define CONTROL_RATE 128

// use: Sample <table_size, update_rate> SampleName (wavetable)
Sample <kick909short_NUM_CELLS, AUDIO_RATE>     kickSamp(kick909short_DATA);
Sample <snare909short_NUM_CELLS, AUDIO_RATE>    snareSamp(snare909short_DATA);
Sample <hihatc909short_NUM_CELLS, AUDIO_RATE>   hihatcSamp(hihatc909short_DATA);
Sample <hihatoshort_NUM_CELLS, AUDIO_RATE>      hihatoSamp(hihatoshort_DATA);
Sample <snare909reverse_NUM_CELLS, AUDIO_RATE>  snarerevSamp(snare909reverse_DATA);

// Saw oscillator and envelope for bass
//Oscil <TRIANGLE2048_NUM_CELLS, AUDIO_RATE> aBass(TRIANGLE2048_DATA);
Oscil <SAW2048_NUM_CELLS, AUDIO_RATE> aBass(SAW2048_DATA);
ADSR <CONTROL_RATE, AUDIO_RATE> envelope;
LowPassFilter aLP;
Oscil <TRIANGLE512_NUM_CELLS, CONTROL_RATE> kLFO(TRIANGLE512_DATA);
uint8_t lfo1;
uint8_t lfo2;
uint8_t drumVolume = 1;

bool isRandomDrumVolume = true;

// for scheduling sample start
EventDelay kTriggerDelay;

// ----------------------------------------------------------------------------

int bpm = 140;
int stepDelay = 60000L / bpm;
int tickDelay = stepDelay / 12;

uint8_t tickCnt = 0;    // 3 ticks per step
uint8_t beatCnt = 0;    // 4 steps in a beat
uint8_t stepCnt = 0;
uint8_t measure = 0;    // 8 measures


Grid8x8 matrix[4] = {
  Grid8x8(), Grid8x8(),
  Grid8x8(), Grid8x8()
};
 
GridController controller = GridController( &matrix[0], &matrix[1], &matrix[2], &matrix[3] );

#define NUM_KEYS  32
#define NUM_STEPS 32

// Lookup tables for step LEDs:
static const uint8_t
  step2led[NUM_KEYS]  = {  0,  1,  2,  3, 16, 17, 18, 19,
                           4,  5,  6,  7, 20, 21, 22, 23,
                           8,  9, 10, 11, 24, 25, 26, 27,
                          12, 13, 14, 15, 28, 29, 30, 31 },
  led2step[NUM_KEYS]  = {  0,  1,  2,  3,  8,  9, 10, 11,
                          16, 17, 18, 19, 24, 25, 26, 27,
                           4,  5,  6,  7, 12, 13, 14, 15,
                          20, 21, 22, 23, 28, 29, 30, 31 };

// Key assignment
#define CTRL_PAT_0    32
#define CTRL_PAT_1    33
#define CTRL_PLAY     34
#define CTRL_PERF     35
#define CTRL_CLEAR_E  48
#define CTRL_CLEAR    49
#define CTRL_SAVE     50
#define CTRL_LOAD     51
#define CTRL_MUTE_BASS  36
#define CTRL_MUTE_KICK  37
#define CTRL_MUTE_SNARE 38
#define CTRL_MUTE_HH    39

#define CTRL_LEN_0    52
#define CTRL_LEN_2    53
#define CTRL_LEN_4    54
#define CTRL_HOLD     55

#define CTRL_ACC      54
#define CTRL_RPT      55

#define CTRL_BASS     43
#define CTRL_HHOPEN   58
#define CTRL_KICK     61
#define CTRL_SNARE    62
#define CTRL_HHCLOSE  63

#define CTRL_KEY_C1   44
#define CTRL_KEY_Db   41
#define CTRL_KEY_D    45
#define CTRL_KEY_Eb   42 
#define CTRL_KEY_E    46
#define CTRL_KEY_F    47
#define CTRL_KEY_Gb   56 
#define CTRL_KEY_G    60l
#define CTRL_KEY_Ab   57 
#define CTRL_KEY_A    61
#define CTRL_KEY_Bb   58 
#define CTRL_KEY_B    62
#define CTRL_KEY_C2   63 
#define CTRL_OCT_UP   59
#define CTRL_OCT_DOWN 40

const float bassFreq[37] = {
// C      Db     D      Eb     E      F      Gb     G      Ab     A      Bb     B
   32.7,  34.6,  36.7,  38.9,  41.2,  43.7,  46.2,  49.0,  51.9,  55.0,  58.3,  61.7,
   65.4,  69.3,  73.4,  77.8,  82.4,  87.3,  92.5,  98.0, 103.8, 110.0, 116.5, 123.5,
  130.8, 138.6, 146.8, 155.6, 164.8, 174.6, 185.0, 196.0, 207.7, 220.0, 233.1, 246.9,
  261.6
};

uint8_t bassTranspose = 12; // middle octave
uint8_t seqTranspose  = 0;

uint8_t lastBassKey = 0;

#define MASK_KICK    0x01
#define MASK_SNARE   0x04
#define MASK_HHCLOSE 0x02
#define MASK_HHOPEN  0x08
#define MASK_BASS    0x10
#define MASK_GHOST   0x20
#define MASK_ACCENT  0x40
#define MASK_REPEAT  0x80

#define MASK_ON     0x01
#define MASK_OFF    0x02

uint8_t seq[NUM_STEPS];           // Pattern
uint8_t bassNoteVal[NUM_STEPS];   // Bass note value
uint8_t bassNoteTrig[NUM_STEPS];    // Bass trigger

#define SEQ_MODE_RUNNING  true
#define SEQ_MODE_HALTED   false
#define EDIT_MODE_PERFORMANCE true
#define EDIT_MODE_STEPINPUT   false
#define KBD_MODE_DRUM   true
#define KBD_MODE_BASS   false
#define LEN_MODE_0  0
#define LEN_MODE_1  1
#define LEN_MODE_2  2
#define LEN_MODE_4  3
#define LEN_MODE_H  4
#define PAGE_MODE_PATTERN   true
#define PAGE_MODE_PROPERTY  false

uint8_t patternLoaded[2] = { 0, 0};

#define NUM_PROPERTIES  8
uint8_t property[NUM_PROPERTIES];
uint8_t propertyDefinition[NUM_PROPERTIES][4] = {
  { 60, 96, 120, 160 },   // BPM
//  { 0, 2, 4, 5 },         // Kick  bit crush
  { 0, 0, 1, 1 },         // Random drum volume
  { 0, 2, 4, 5 },         // Snare bit crush
  { 0, 2, 4, 5 },         // Hihat bit crush
  { 1, 2, 4, 8 },         // LFO frequency = x/12 Hz
  { 20, 50, 100, 150 },   // Bass LP resonance
  { 20, 50, 100, 200 },   // Bass Attack ms
  { 2, 10, 20, 50 }   // Bass Decay = x *10 ms
};

bool keyboardMode = KBD_MODE_DRUM;
uint8_t instrumentMask = MASK_KICK;
uint8_t lastInstrumentMask = MASK_KICK;
bool sequencerMode = SEQ_MODE_RUNNING;
bool editMode = EDIT_MODE_PERFORMANCE;
bool pageMode = PAGE_MODE_PATTERN;

bool isClearEnabled = false;
bool isLoadEnabled  = false;
bool isSaveEnabled  = false;
bool isBassActive  = true;
bool isKickActive  = true;
bool isSnareActive = true;
bool isHihatActive = true;
bool isAccentEnabled = false;
bool isRepeatEnabled  = false;

uint8_t bassLenMode = LEN_MODE_1;
uint8_t bitCrushKick;
uint8_t bitCrushSnare;
uint8_t bitCrushHihat;

// ----------------------------------------------------------------------------

void setFactoryPattern() {
  seq[ 0]=    B00000001; // 
  seq[ 2]=    B00000010;
  seq[ 4]=    B00000100;
  seq[ 6]=    B00000010;
  seq[ 8]=    B00000001; // 
  seq[10]=    B00000010;
  seq[12]=    B00000100;
  seq[13]=    B00000010;
  seq[14]=    B00000010;
  seq[15]=    B00000010;
  seq[16]=    B00000001; // 
  seq[18]=    B00000010;
  seq[20]=    B00000100;
  seq[22]=    B00000010;
  seq[24]=    B00000001; // 
  seq[25]=    B00000100;
  seq[26]=    B00000010;
  seq[28]=    B10000100;
  seq[30]=    B00001000;

  bassNoteTrig[0]  |= MASK_ON; bassNoteVal[0] = 9;  bassNoteTrig[3]  |= MASK_OFF;  seq[0]  |= MASK_BASS;
  bassNoteTrig[8]  |= MASK_ON; bassNoteVal[8] = 9;  bassNoteTrig[9]  |= MASK_OFF;  seq[8]  |= MASK_BASS;  
  bassNoteTrig[10] |= MASK_ON; bassNoteVal[10]= 12; bassNoteTrig[10] |= MASK_OFF;  seq[10] |= MASK_BASS;
  bassNoteTrig[16] |= MASK_ON; bassNoteVal[16]= 12; bassNoteTrig[17] |= MASK_OFF;  seq[16] |= MASK_BASS;
  bassNoteTrig[20] |= MASK_ON; bassNoteVal[20]= 12; bassNoteTrig[21] |= MASK_OFF;  seq[20] |= MASK_BASS;
  bassNoteTrig[24] |= MASK_ON; bassNoteVal[24]= 14; bassNoteTrig[24] |= MASK_OFF;  seq[24] |= MASK_BASS;
  bassNoteTrig[25] |= MASK_ON; bassNoteVal[25]= 14; bassNoteTrig[25] |= MASK_OFF;  seq[25] |= MASK_BASS;
  bassNoteTrig[28] |= MASK_ON; bassNoteVal[28]= 14; bassNoteTrig[28] |= MASK_OFF;  seq[28] |= MASK_BASS;
  bassNoteTrig[30] |= MASK_ON; bassNoteVal[30]= 14; bassNoteTrig[30] |= MASK_OFF;  seq[30] |= MASK_BASS;
}


// ----------------------------------------------------------------------------

void updateParameters() {
  stepDelay = 60000L / propertyDefinition[0][property[0]];
  tickDelay = stepDelay / 12;
  kTriggerDelay.set(tickDelay); // tick [msec] countdown, within resolution of CONTROL_RATE

  bitCrushKick  = 0;
  (0 == propertyDefinition[1][property[1]]) ? isRandomDrumVolume = false : isRandomDrumVolume = true;
  bitCrushSnare = propertyDefinition[2][property[2]];
  bitCrushHihat = propertyDefinition[3][property[3]];
  
//  kLFO.setFreq(0.166f);
  kLFO.setFreq( (float) propertyDefinition[4][property[4]] / 12.0f );
//  aLP.setResonance(100);  // resonance in the range 0-255, with 255 being most resonant.
  aLP.setResonance( propertyDefinition[5][property[5]] );
//  envelope.setTimes(20,50,1000,100); // A-D-S-R times in [msec]
  envelope.setTimes( propertyDefinition[6][property[6]], 50, 1000, propertyDefinition[7][property[7]] * 10 );
}

// ----------------------------------------------------------------------------

void setup() {
  controller.begin( 0x70, 0x71, 0x72, 0x73 ); // begin() with the addresses of each panel.

  DDRD |= SYNC_MASK;              // define sync pin as output

  keyboardMode = KBD_MODE_BASS;
  clearPattern();                 // separately for bass and drum
  keyboardMode = KBD_MODE_DRUM;
  clearPattern();                 // separately for bass and drum
  instrumentMask = MASK_KICK;
  lastInstrumentMask = MASK_KICK;
  sequencerMode = SEQ_MODE_HALTED;
  editMode = EDIT_MODE_STEPINPUT;
  pageMode = PAGE_MODE_PATTERN;
  
  //setFactoryPattern();
  startupScreen();
  showPattern();

  patternLoaded[0]= EEPROM.read(NUM_STEPS *  3 );   // 3 values
  patternLoaded[1]= EEPROM.read(NUM_STEPS * (3+4)); // 3 values + offset for pattern 0
  
  startMozzi(CONTROL_RATE);

  kickSamp.setFreq((float)     kick909short_SAMPLERATE / (float) kick909short_NUM_CELLS);
  snareSamp.setFreq((float)    snare909short_SAMPLERATE / (float) snare909short_NUM_CELLS);
  hihatcSamp.setFreq((float)   hihatc909short_SAMPLERATE / (float) hihatc909short_NUM_CELLS);
  hihatoSamp.setFreq((float)   hihatoshort_SAMPLERATE / (float) hihatoshort_NUM_CELLS);
  snarerevSamp.setFreq((float) snare909reverse_SAMPLERATE / (float) snare909reverse_NUM_CELLS);

  property[0] = 2;  // BPM
  property[1] = 0;  // Random drum volume
  property[2] = 0;  // Snare bit crush
  property[3] = 0;  // Hihat bit crush
  property[4] = 1;  // LFO
  property[5] = 2;  // Bass LP resonance
  property[6] = 0;  // Bass Attack
  property[7] = 1;  // Bass Release

  envelope.setADLevels(200,200);
  envelope.setTimes(20,50,1000,100); // A-D-S-R times in [msec]
  aBass.setFreq(73.4f); // default
  aLP.setCutoffFreq(4); // default; use the range 0-255 to represent 0-8192 Hz (AUDIO_RATE/2)

  updateParameters(); // with the property values
}

 
// ----------------------------------------------------------------------------

// Save to and load from EEPROM: iPattern must be in {0, 1}

void savePattern( uint8_t iPattern ) {
  int addr = 0;
  if (iPattern) addr = NUM_STEPS << 2;
  
  for (uint8_t i=0; i<NUM_STEPS; i++) {   // 32 steps * 1 byte * 3 arrays = 1 kB
    EEPROM.update(addr,   seq[i]);
    EEPROM.update(addr+1, bassNoteVal[i]);
    EEPROM.update(addr+2, bassNoteTrig[i]);
    addr += 3;
  }
  EEPROM.update(addr, 1); // dirty flag - can use safely because we only increment by 3 ...
  patternLoaded[iPattern] = 1;
}

void loadPattern( uint8_t iPattern ) {
  int addr = 0;
  if (iPattern) addr = NUM_STEPS << 2;

  for (uint8_t i=0; i<NUM_STEPS; i++) {   // 64 steps * 4 byte * 4 arrays = 1 kB
    seq[i]          = EEPROM.read(addr);
    bassNoteVal[i]  = EEPROM.read(addr+1);
    bassNoteTrig[i] = EEPROM.read(addr+2);
    addr += 3;
  }
}

// Clear active pattern - only drum or bass

void clearPattern() {
  if (KBD_MODE_BASS == keyboardMode) {
    for (uint8_t i=0; i<NUM_STEPS; i++) {
      seq[i] &= ~(MASK_BASS | MASK_GHOST);
      bassNoteTrig[i] = 0;
      bassNoteVal[i]  = 0;  
    }
  } else {
    for (uint8_t i=0; i<NUM_STEPS; i++) {
      seq[i] &= ~(MASK_KICK | MASK_SNARE | MASK_HHOPEN | MASK_HHCLOSE | MASK_ACCENT | MASK_REPEAT);
    }
  }
}

// Clear stored pattern: iPattern must be in {0, 1}

void clearPattern( uint8_t iPattern ) {
  int addr = 0;
  if (iPattern) addr = NUM_STEPS << 2;
  
  for (uint8_t i=0; i<NUM_STEPS; i++) { 
    EEPROM.update(addr,   0);
    EEPROM.update(addr+1, 0);
    EEPROM.update(addr+2, 0);
    addr += 3;
  }
  EEPROM.update(addr, 0); // dirty flag - can use safely because we only increment by 3 ...
  patternLoaded[iPattern] = 0;
}

// ----------------------------------------------------------------------------
// Display functions:
// - startupScreen():    Test all LEDs on power-up
// - showPattern():      Show current pattern depending on edit mode
// - showPropertyPage(): Display property settings
// - showBassKeys():     Switch to bass keyboard
// - showDrumKeys():     Switch to drum keyboard
// - showStatus():       Show control keys depending on mode and status

void startupScreen() {
  // light up all the LEDs in order
  for (uint8_t i=0; i<NUM_KEYS; i++) {
    controller.setLED(step2led[i]);
    controller.setLED(32+step2led[i]);
    controller.writeDisplay();    
    delay(20);
  }
  // then turn them off
  for (uint8_t i=0; i<NUM_KEYS; i++) {
    controller.clrLED(step2led[i]);
    controller.clrLED(32+step2led[i]);
    controller.writeDisplay();    
    delay(20);
  }
}

void showPattern() {
  uint8_t currentMask = instrumentMask;
  if ((EDIT_MODE_PERFORMANCE == editMode) && (KBD_MODE_DRUM == keyboardMode)) {
    currentMask = (MASK_KICK | MASK_SNARE | MASK_HHOPEN | MASK_HHCLOSE);
  }
  for (uint8_t i=0; i<NUM_KEYS; i++) {
    if (seq[i] & currentMask) {
      controller.setLED( step2led[i] );
    } else {
      controller.clrLED( step2led[i] );
    }
  }
}

void showPropertyPage() {
  for (uint8_t p=0; p<NUM_PROPERTIES; p++) {
    uint8_t v=0;
    for (v=0; v<=property[p]; v++) {
      controller.setLED((p<<2)+v);
    }
    for (v=property[p]+1; v<4; v++) {
      controller.clrLED((p<<2)+v);
    }
  }
}

void showBassKeys() {
  // Clear drum keys
  controller.clrLED(CTRL_HHOPEN);  controller.clrLED(CTRL_HHCLOSE);
  controller.clrLED(CTRL_KICK);    controller.clrLED(CTRL_SNARE); 
  
  // Light up bass keys
  controller.setLED(CTRL_KEY_C1);  controller.setLED(CTRL_KEY_Db); 
  controller.setLED(CTRL_KEY_D);  controller.setLED(CTRL_KEY_Eb);
  controller.setLED(CTRL_KEY_E);
  controller.setLED(CTRL_KEY_F);  controller.setLED(CTRL_KEY_Gb);
  controller.setLED(CTRL_KEY_G);  controller.setLED(CTRL_KEY_Ab);
  controller.setLED(CTRL_KEY_A);  controller.setLED(CTRL_KEY_Bb);
  controller.setLED(CTRL_KEY_B);
  controller.setLED(CTRL_KEY_C2);  
}

void showDrumKeys() {
  // Clear bass keys
  controller.clrLED(CTRL_KEY_C1); controller.clrLED(CTRL_KEY_Db);
  controller.clrLED(CTRL_KEY_D);  controller.clrLED(CTRL_KEY_Eb);
  controller.clrLED(CTRL_KEY_E);
  controller.clrLED(CTRL_KEY_F);  controller.clrLED(CTRL_KEY_Gb); 
  controller.clrLED(CTRL_KEY_G);  controller.clrLED(CTRL_KEY_Ab);
  controller.clrLED(CTRL_KEY_A);  controller.clrLED(CTRL_KEY_Bb);
  controller.clrLED(CTRL_KEY_B);
  controller.clrLED(CTRL_KEY_C2);
  
  // Light up drum keys
  controller.setLED(CTRL_HHOPEN); controller.setLED(CTRL_HHCLOSE);
  controller.setLED(CTRL_KICK);   controller.setLED(CTRL_SNARE); 
}

void showStatus() {
  controller.setLED(CTRL_PERF);
  if (SEQ_MODE_RUNNING == sequencerMode) {

    // when sequencer is running, show moving step in performance mode
    if ((EDIT_MODE_PERFORMANCE == editMode) && (PAGE_MODE_PATTERN == pageMode)) {
      uint8_t lastStep = stepCnt-1;
      if (!stepCnt) lastStep = 31;
      if (seq[lastStep] & instrumentMask) {
        controller.setLED( step2led[lastStep] );
      } else {
        controller.clrLED( step2led[lastStep] );   
      }
      if (seq[stepCnt] & instrumentMask) {
        controller.clrLED( step2led[stepCnt] );
      } else {
        controller.setLED( step2led[stepCnt] );   
      }
    }
    
    if (3 == beatCnt) {
      controller.clrLED(CTRL_PLAY);
      if (EDIT_MODE_STEPINPUT == editMode) {  // in edit mode, flash instrument keys
        switch (instrumentMask) {
          case MASK_BASS:    controller.clrLED(CTRL_BASS);    break;
          case MASK_KICK:    controller.clrLED(CTRL_KICK);    break;
          case MASK_SNARE:   controller.clrLED(CTRL_SNARE);   break;
          case MASK_HHOPEN:  controller.clrLED(CTRL_HHOPEN);  break;
          case MASK_HHCLOSE: controller.clrLED(CTRL_HHCLOSE); break;
          default: break;
        }
      } else {
        controller.clrLED(CTRL_PERF);         // in perf mode, flash perf key
      }
    } else if (0 == beatCnt) {
      controller.setLED(CTRL_PLAY);
      controller.setLED(CTRL_BASS);   // show always, as this is the toggle switch
      if (EDIT_MODE_STEPINPUT == editMode) {
        if (KBD_MODE_DRUM == keyboardMode) {
          controller.setLED(CTRL_KICK);
          controller.setLED(CTRL_SNARE);
          controller.setLED(CTRL_HHOPEN);
          controller.setLED(CTRL_HHCLOSE);
        }
      }
    }
  } else { // halted
      controller.setLED(CTRL_PLAY);
      controller.setLED(CTRL_BASS);
      if (EDIT_MODE_STEPINPUT == editMode) {
        controller.clrLED(CTRL_PERF);
      }
  }

  patternLoaded[0] ? controller.setLED(CTRL_PAT_0) : controller.clrLED(CTRL_PAT_0);
  patternLoaded[1] ? controller.setLED(CTRL_PAT_1) : controller.clrLED(CTRL_PAT_1);

  isClearEnabled ? controller.setLED(CTRL_CLEAR_E) : controller.clrLED(CTRL_CLEAR_E);
  isLoadEnabled  ? controller.setLED(CTRL_LOAD) :    controller.clrLED(CTRL_LOAD);
  isSaveEnabled  ? controller.setLED(CTRL_SAVE) :    controller.clrLED(CTRL_SAVE);

  isBassActive  ? controller.setLED(CTRL_MUTE_BASS) :  controller.clrLED(CTRL_MUTE_BASS);
  isKickActive  ? controller.setLED(CTRL_MUTE_KICK) :  controller.clrLED(CTRL_MUTE_KICK);
  isSnareActive ? controller.setLED(CTRL_MUTE_SNARE) : controller.clrLED(CTRL_MUTE_SNARE);
  isHihatActive ? controller.setLED(CTRL_MUTE_HH) :    controller.clrLED(CTRL_MUTE_HH);

  if (KBD_MODE_BASS == keyboardMode) {
    if (EDIT_MODE_STEPINPUT == editMode) {
      controller.setLED(CTRL_LEN_0);
      controller.setLED(CTRL_LEN_2);
      controller.setLED(CTRL_LEN_4);    // doubles as CTRL_REV in drum keyboard
      controller.setLED(CTRL_HOLD);     // doubles as CTRL_RPT in drum keyboard
      switch (bassLenMode) {
        case LEN_MODE_0: controller.clrLED(CTRL_LEN_0); break;
        case LEN_MODE_1: break;
        case LEN_MODE_2: controller.clrLED(CTRL_LEN_2); break;
        case LEN_MODE_4: controller.clrLED(CTRL_LEN_4); break;
        case LEN_MODE_H: controller.clrLED(CTRL_HOLD);  break;
        default: break;
      }
    }
    else {
      controller.clrLED(CTRL_LEN_0);
      controller.clrLED(CTRL_LEN_2);
      controller.clrLED(CTRL_LEN_4);
      controller.clrLED(CTRL_HOLD);
    }
  } else {
    controller.clrLED(CTRL_LEN_0);
    controller.clrLED(CTRL_LEN_2);
    isAccentEnabled ? controller.clrLED(CTRL_ACC) : controller.setLED(CTRL_ACC);
    isRepeatEnabled  ? controller.clrLED(CTRL_RPT) : controller.setLED(CTRL_RPT);
  }
  
  controller.setLED(CTRL_CLEAR);  // property page key
}

// ----------------------------------------------------------------------------
// Helper functions for bass keyboard and pattern keys

void handleBassKeyPressed( uint8_t keyNum ) {
  if (SEQ_MODE_HALTED == sequencerMode) {
    lastBassKey = keyNum;
    aBass.setFreq(bassFreq[bassTranspose + keyNum]);
    envelope.noteOn();
  } else {
    if (EDIT_MODE_STEPINPUT == editMode) {
      lastBassKey = keyNum;
    } else { // performance mode: transpose current sequence
      seqTranspose = keyNum;
    }
  }
}

void handleBassKeyReleased() {
  if (SEQ_MODE_HALTED == sequencerMode) {
    envelope.noteOff();
  }
}

void handlePatternKey( uint8_t iPattern ) {
  if (isClearEnabled) {
    clearPattern(iPattern);
    isClearEnabled = false;
  } else if (isLoadEnabled) {
    loadPattern(iPattern);
    isLoadEnabled = false;
  } else if (isSaveEnabled) {
    savePattern(iPattern);
    isSaveEnabled = false;
  }
}

// ---------------------------------------------------------------
// Main control loop, triggered by Mozzi

void updateControl() {
  envelope.update();
  lfo1 = kLFO.next() * (58.0f/127) + 62;   // range: 4 .. 120 - better suited
//  lfo = kLFO.next() * (55.0f/127) + 65;   // range: 10 .. 120
  lfo2 = (tickCnt << 5) + 10;
  aLP.setCutoffFreq(lfo1);

  if(kTriggerDelay.ready()){
    kTriggerDelay.start();

    if (SEQ_MODE_RUNNING == sequencerMode) {
  
      uint8_t thisStep = seq[stepCnt];

      if ((!tickCnt) || (thisStep & MASK_REPEAT)) {
        // First check if reverse snare is on, use volume=0 for it ...
        if ((thisStep & MASK_SNARE) && isSnareActive) {
          if (thisStep & MASK_ACCENT) {   // use accent as reverse effect on snare
            drumVolume = 0;
            snarerevSamp.start();
          } else {
            snareSamp.start();
          }
        }
        // ... then compute drum volume ... 
        if (thisStep & MASK_ACCENT) {   // else use accent to up the volume
          drumVolume = 2;
        } else {
          if (isRandomDrumVolume) {
            drumVolume = (lfo1 & 0x01); // use LFO as pseudo random generator (0 or 1)
          } else {
            drumVolume = 1;
          }
        }
        // ... and then trigger the rest of the drum sounds.
        if ((thisStep & MASK_KICK) && isKickActive) {
          kickSamp.start();
        } 
        if ((thisStep & MASK_HHCLOSE) && isHihatActive) {
          hihatcSamp.start();
        } 
        if ((thisStep & MASK_HHOPEN) && isHihatActive) {
          hihatoSamp.start();
        } 
      }
  
      if (!tickCnt) {
        if ((bassNoteTrig[stepCnt] & MASK_ON) && isBassActive) {
          uint8_t val = bassNoteVal[stepCnt];
          if (val <= 24) {  // index 36 is maximum, leave room for 12 steps to transpose 
            aBass.setFreq( bassFreq[val+seqTranspose] );
          } else {
            aBass.setFreq( bassFreq[val] );
          }
          envelope.noteOn();
        }

        if (0 == beatCnt) {
          SYNC_PORT |= SYNC_MASK;         // sync output HIGH for every 2nd beat
        }
        if (1 == beatCnt) {
          SYNC_PORT &= ~SYNC_MASK;          // sync output back to LOW 
        }
      } else if (1 == tickCnt) {
        if ((bassNoteTrig[stepCnt] & MASK_OFF) && (thisStep & MASK_GHOST)) {
          envelope.noteOff();
        }
      } else if (2 == tickCnt) {
        if (bassNoteTrig[stepCnt] & MASK_OFF) {
          envelope.noteOff();
        }
      }


      tickCnt++;                // 12 ticks one 4 beat measure
      if (tickCnt >= 3) {
        tickCnt = 0;
        beatCnt++;              // tick: 0..3
        stepCnt++;              // beat: 0..4
        if (beatCnt >= 4) {
          beatCnt = 0;
          measure++;
          if (measure >= 8) {
            measure = 0;
            stepCnt = 0;
          }
        }
      }
    }       // end of seq running
    else {  
      // halted
    }      

    if (PAGE_MODE_PATTERN == pageMode) {
      showPattern();
    } else {
      showPropertyPage();
    }
    if (KBD_MODE_BASS == keyboardMode) {
      showBassKeys();
      isAccentEnabled = false; // to be safe ...
      isRepeatEnabled = false;
    } else {
      showDrumKeys();
    }
    showStatus();
    controller.writeDisplay();
   
    if (controller.readControlSwitches()) {
      if (controller.justPressed(CTRL_PLAY)) {
        if (SEQ_MODE_RUNNING == sequencerMode) {
          sequencerMode = SEQ_MODE_HALTED;
          envelope.noteOff();
          SYNC_PORT &= ~SYNC_MASK;          // sync output back to LOW when stopping
        } else {
          tickCnt = 0;
          beatCnt = 0;
          stepCnt = 0;
          measure = 0;
          sequencerMode = SEQ_MODE_RUNNING; // sync output is started with the first beat
        }
      } else if (controller.justPressed(CTRL_CLEAR_E)) {
        isClearEnabled = true;
      } else if (controller.justReleased(CTRL_CLEAR_E)) {
        isClearEnabled = false;
      } else if (controller.justPressed(CTRL_LOAD)) {
        isLoadEnabled = true;
      } else if (controller.justReleased(CTRL_LOAD)) {
        isLoadEnabled = false;
      } else if (controller.justPressed(CTRL_SAVE)) {
        isSaveEnabled = true;
      } else if (controller.justReleased(CTRL_SAVE)) {
        isSaveEnabled = false;
      } else if (controller.justPressed(CTRL_PAT_0)) {
        handlePatternKey(0);
      } else if (controller.justPressed(CTRL_PAT_1)) {
        handlePatternKey(1);
      } else if (controller.justPressed(CTRL_CLEAR)) {
        if (isClearEnabled) {
          clearPattern();
          isClearEnabled = false;
        } else {
          pageMode = !pageMode;
        }
      } else if (controller.justPressed(CTRL_BASS)) {
        if (KBD_MODE_BASS == keyboardMode) {
          keyboardMode = KBD_MODE_DRUM;
          instrumentMask = lastInstrumentMask;
        } else {
          keyboardMode = KBD_MODE_BASS;        
          lastInstrumentMask = instrumentMask;
          instrumentMask = MASK_BASS;
        }
      } else if (controller.justPressed(CTRL_PERF)) {
        if (EDIT_MODE_STEPINPUT == editMode) {
          editMode = EDIT_MODE_PERFORMANCE;
        } else {
          editMode = EDIT_MODE_STEPINPUT;
        }
      } else if (controller.justPressed(CTRL_MUTE_BASS)) {
        isBassActive = !isBassActive;
      } else if (controller.justPressed(CTRL_MUTE_KICK)) {
        isKickActive = !isKickActive;
      } else if (controller.justPressed(CTRL_MUTE_SNARE)) {
        isSnareActive = !isSnareActive;
      } else if (controller.justPressed(CTRL_MUTE_HH)) {
        isHihatActive = !isHihatActive;
      } else if (controller.justPressed(CTRL_LEN_0)) {
        if (KBD_MODE_DRUM == keyboardMode) {
          
        } else {
          bassLenMode = (LEN_MODE_0 == bassLenMode) ? LEN_MODE_1 : LEN_MODE_0;
        }
      } else if (controller.justPressed(CTRL_LEN_2)) {
        if (KBD_MODE_DRUM == keyboardMode) {
          
        } else {
          bassLenMode = (LEN_MODE_2 == bassLenMode) ? LEN_MODE_1 : LEN_MODE_2;
        }
      } else if (controller.justPressed(CTRL_LEN_4)) { // doubles as CTRL_ACC
        if (KBD_MODE_DRUM == keyboardMode) {
          isAccentEnabled = true;
        } else {
          bassLenMode = (LEN_MODE_4 == bassLenMode) ? LEN_MODE_1 : LEN_MODE_4;
        }
      } else if (controller.justReleased(CTRL_LEN_4)) { // doubles as CTRL_REV
        if (KBD_MODE_DRUM == keyboardMode) {
          isAccentEnabled = false;
        }
      } else if (controller.justPressed(CTRL_HOLD)) {   // doubles as CTRL_RPT
        if (KBD_MODE_DRUM == keyboardMode) {
          isRepeatEnabled = true;
        } else {
          bassLenMode = (LEN_MODE_H == bassLenMode) ? LEN_MODE_1 : LEN_MODE_H;
        }
      } else if (controller.justReleased(CTRL_HOLD)) {    // doubles as CTRL_RPT
        if (KBD_MODE_DRUM == keyboardMode) {
          isRepeatEnabled = false;
        }
      } else {
        //
        // Handle bass keyboard
        //
        if (KBD_MODE_BASS == keyboardMode) {
          if (controller.justPressed(CTRL_OCT_DOWN)) {
            switch (bassTranspose) {
              case  0:  break;
              case 12:  bassTranspose = 0;
                        controller.setLED(CTRL_OCT_DOWN);
                        break;
              case 24:  bassTranspose = 12;
                        controller.clrLED(CTRL_OCT_UP);
                        break;
              default:  break;
            }
          } else if (controller.justPressed(CTRL_OCT_UP)) {
            switch (bassTranspose) {
              case  0:  bassTranspose = 12;
                        controller.clrLED(CTRL_OCT_DOWN);
                        break;
              case 12:  bassTranspose = 24;
                        controller.setLED(CTRL_OCT_UP);
                        break;
              case 24:  break;
              default:  break;
            }
          }
          if (controller.justPressed(CTRL_KEY_C1)) {
            handleBassKeyPressed(0);
          } else if (controller.justPressed(CTRL_KEY_Db)) {
            handleBassKeyPressed(1);
          } else if (controller.justPressed(CTRL_KEY_D)) {
            handleBassKeyPressed(2);
          } else if (controller.justPressed(CTRL_KEY_Eb)) {
            handleBassKeyPressed(3);
          } else if (controller.justPressed(CTRL_KEY_E)) {
            handleBassKeyPressed(4);
          } else if (controller.justPressed(CTRL_KEY_F)) {
            handleBassKeyPressed(5);
          } else if (controller.justPressed(CTRL_KEY_Gb)) {
            handleBassKeyPressed(6);
          } else if (controller.justPressed(CTRL_KEY_G)) {
            handleBassKeyPressed(7);
          } else if (controller.justPressed(CTRL_KEY_Ab)) {
            handleBassKeyPressed(8);
          } else if (controller.justPressed(CTRL_KEY_A)) {
            handleBassKeyPressed(9);
          } else if (controller.justPressed(CTRL_KEY_Bb)) {
            handleBassKeyPressed(10);
          } else if (controller.justPressed(CTRL_KEY_B)) {
            handleBassKeyPressed(11);
          } else if (controller.justPressed(CTRL_KEY_C2)) {
            handleBassKeyPressed(12);
          } else if (controller.justReleased(CTRL_KEY_C1)) {
            handleBassKeyReleased();
          } else if (controller.justReleased(CTRL_KEY_Db)) {
            handleBassKeyReleased();
          } else if (controller.justReleased(CTRL_KEY_D)) {
            handleBassKeyReleased();
          } else if (controller.justReleased(CTRL_KEY_Eb)) {
            handleBassKeyReleased();
          } else if (controller.justReleased(CTRL_KEY_E)) {
            handleBassKeyReleased();
          } else if (controller.justReleased(CTRL_KEY_F)) {
            handleBassKeyReleased();
          } else if (controller.justReleased(CTRL_KEY_Gb)) {
            handleBassKeyReleased();
          } else if (controller.justReleased(CTRL_KEY_G)) {
            handleBassKeyReleased();
          } else if (controller.justReleased(CTRL_KEY_Ab)) {
            handleBassKeyReleased();
          } else if (controller.justReleased(CTRL_KEY_A)) {
            handleBassKeyReleased();
          } else if (controller.justReleased(CTRL_KEY_Bb)) {
            handleBassKeyReleased();
          } else if (controller.justReleased(CTRL_KEY_B)) {
            handleBassKeyReleased();
          } else if (controller.justReleased(CTRL_KEY_C2)) {
            handleBassKeyReleased();
          } 
        }
        //
        // Handle drum keyboard
        //
        else {
          if (controller.justPressed(CTRL_HHOPEN)) {
            instrumentMask = MASK_HHOPEN;            
            if (EDIT_MODE_PERFORMANCE == editMode) {
              hihatoSamp.start();                
              if (SEQ_MODE_RUNNING == sequencerMode) {
                if (0 == (seq[stepCnt] & instrumentMask)) {
                  seq[stepCnt] |= instrumentMask;
                  controller.setLED( step2led[stepCnt] );
                }
              } 
            }
          } else if (controller.justPressed(CTRL_KICK)) {
            instrumentMask = MASK_KICK;            
            if (EDIT_MODE_PERFORMANCE == editMode) {
              kickSamp.start();                
              if (SEQ_MODE_RUNNING == sequencerMode) {
                if (0 == (seq[stepCnt] & instrumentMask)) {
                  seq[stepCnt] |= instrumentMask;
                  controller.setLED( step2led[stepCnt] );
                }
              } 
            }
          } else if (controller.justPressed(CTRL_SNARE)) {
            instrumentMask = MASK_SNARE;            
            if (EDIT_MODE_PERFORMANCE == editMode) {
              snareSamp.start();                
              if (SEQ_MODE_RUNNING == sequencerMode) {
                if (0 == (seq[stepCnt] & instrumentMask)) {
                  seq[stepCnt] |= instrumentMask;
                  controller.setLED( step2led[stepCnt] );
                }
              } 
            }
          } else if (controller.justPressed(CTRL_HHCLOSE)) {
            instrumentMask = MASK_HHCLOSE;            
            if (EDIT_MODE_PERFORMANCE == editMode) {
              hihatcSamp.start();                
              if (SEQ_MODE_RUNNING == sequencerMode) {
                if (0 == (seq[stepCnt] & instrumentMask)) {
                  seq[stepCnt] |= instrumentMask;
                  controller.setLED( step2led[stepCnt] );
                }
              } 
            }
          } 
        }
      }
    }

    if (controller.readPatternSwitches()) {
      // go through every button
      for (uint8_t i=0; i<NUM_KEYS; i++) {
        if (controller.justPressed(i)) {
          if (PAGE_MODE_PATTERN == pageMode) {
            
            // Pattern page view
            uint8_t pos = led2step[i];
            if (isRepeatEnabled) {
              seq[pos] ^= MASK_REPEAT;
            }
            if (seq[pos] & instrumentMask) {  // already set: delete
              seq[pos] &= ~(instrumentMask | MASK_ACCENT | MASK_REPEAT);
              controller.clrLED(i);
              if (MASK_BASS == instrumentMask) {
                bassNoteVal[pos] = 0;
                bassNoteTrig[pos] &=  ~(MASK_ON | MASK_OFF);
              }
            } else {                          // inactive: set
              seq[pos] |= instrumentMask;
              controller.setLED(i);
              if (MASK_BASS == instrumentMask) {
                bassNoteVal[pos]  = lastBassKey + bassTranspose;
                bassNoteTrig[pos] |= MASK_ON;
                seq[pos] &= ~MASK_GHOST;  // clear, only set for LEN_MODE_0
                switch (bassLenMode) {
                  case LEN_MODE_0: 
                    bassNoteTrig[pos] |= MASK_OFF;
                    seq[pos] |= MASK_GHOST; 
                    break;
                  case LEN_MODE_1:
                    bassNoteTrig[pos] |= MASK_OFF;
                    break;
                  case LEN_MODE_2:
                    bassNoteTrig[(pos+2) & 0x1F] |= MASK_OFF;  // wrap around at pos 32
                    break;
                  case LEN_MODE_4:
                    bassNoteTrig[(pos+4) & 0x1F] |= MASK_OFF;  // wrap around at pos 32
                    break;
                  case LEN_MODE_H:
                    // do not add a note off at all
                    break;
                  default: break;
                }
              } else {
                if (isAccentEnabled) {
                  seq[pos] |= MASK_ACCENT;
                }
                if (isRepeatEnabled) {
                  seq[pos] |= MASK_REPEAT;
                }
              }
            }
          } else {
            // Property page mode:  map index i to value and property
            uint8_t prop = i>>2;
            property[prop] = (i & 0x03);
            updateParameters();
          }
        }
        
      }
    } // end of pattern switch handling

  } // end of if trigger delay is ready
      
}

// ----------------------------------------------------------------------------
// Compute audio in updateAudio(), use loop() only for audio hook, as
// defined by the Mozzi library

int updateAudio(){
//  int drums = kickSamp.next() + snareSamp.next() + hihatcSamp.next() + hihatoSamp.next();
  // added bit crush effect:
  int drums = ((kickSamp.next()  >> bitCrushKick)  << bitCrushKick) + 
              (((snareSamp.next() + snarerevSamp.next()) >> bitCrushSnare) << bitCrushSnare)  + 
              (((hihatcSamp.next() + hihatoSamp.next()) >> bitCrushHihat) << bitCrushHihat);
  drums = drums << drumVolume;

//  return drums >> 1;
//  return (int)  ( aLP.next(aBass.next()) );  
//  return (int) (envelope.next() * aBass.next())>>6 ;
//  return (int) (envelope.next() * aLP.next(aBass.next()) )>>6 ;
//  return (int) (((envelope.next() * aBass.next())>>6) + drums) >> 1;

  return (int) ((envelope.next() * aLP.next(aBass.next()) )>>10) + drums;
}

void loop(){
  audioHook();
}

// end of file
// ----------------------------------------------------------------------------
