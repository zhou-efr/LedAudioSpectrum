#include <FastLED.h>

#define LED_PIN  3

#define COLOR_ORDER GRB
#define CHIPSET     WS2811

#define BRIGHTNESS 64

const uint8_t kMatrixWidth = 16;
const uint8_t kMatrixHeight = 16;

// Param for different pixel layouts
const bool    kMatrixSerpentineLayout = true;
const bool    kMatrixVertical = false;

uint16_t XY( uint8_t x, uint8_t y)
{
  uint16_t i;
  
  if( kMatrixSerpentineLayout == false) {
    if (kMatrixVertical == false) {
      i = (y * kMatrixWidth) + x;
    } else {
      i = kMatrixHeight * (kMatrixWidth - (x+1))+y;
    }
  }

  if( kMatrixSerpentineLayout == true) {
    if (kMatrixVertical == false) {
      if( y & 0x01) {
        // Odd rows run backwards
        uint8_t reverseX = (kMatrixWidth - 1) - x;
        i = (y * kMatrixWidth) + reverseX;
      } else {
        // Even rows run forwards
        i = (y * kMatrixWidth) + x;
      }
    } else { // vertical positioning
      if ( x & 0x01) {
        i = kMatrixHeight * (kMatrixWidth - (x+1))+y;
      } else {
        i = kMatrixHeight * (kMatrixWidth - x) - (y+1);
      }
    }
  }
  
  return i;
}

#define NUM_LEDS (kMatrixWidth * kMatrixHeight)

const int DARK[] = {0, 0, 0}
const int CYAN[] = {0, 19, 247}
const int BLUE[] = {0, 255, 253}
const int YELLOW[] = {255, 254, 76}
const int PURPLE[] = {255, 0, 248}
const int ORANGE[] = {255, 127, 46}
const int GREEN[] = {0, 255, 68}
const int RED[] = {255, 0, 33}

const int I[][][] = {{CYAN}, {CYAN}, {CYAN}, {CYAN}} 
const int O[][][] = {{YELLOW, YELLOW}, {YELLOW, YELLOW}}
const int T[][][] = {{PURPLE, PURPLE, PURPLE}, {DARK, PURPLE, DARK}}

CRGB leds_plus_safety_pixel[ NUM_LEDS + 1];
CRGB* const leds( leds_plus_safety_pixel + 1);

uint16_t XYsafe( uint8_t x, uint8_t y)
{
  if( x >= kMatrixWidth) return -1;
  if( y >= kMatrixHeight) return -1;
  return XY(x,y);
}

void loop() {
  
}


void setup() {
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);
  FastLED.setBrightness( BRIGHTNESS );
}
