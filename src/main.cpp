#include <Arduino.h>
#include <Arduino.h>
#include "../lib/arduinoFFT/src/arduinoFFT.h"
#include <SPI.h>
#include "../lib/FastLED/src/FastLED.h"

// -----------------------------------------------
//            FASTLED Initialisation
// -----------------------------------------------

#define LED_PIN  3

#define COLOR_ORDER GRB
#define CHIPSET     WS2811

#define BRIGHTNESS 20

const uint8_t kMatrixWidth = 30;
const uint8_t kMatrixHeight = 40;

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
uint16_t XYsafe( uint8_t x, uint8_t y)
{
    if( x >= kMatrixWidth) return -1;
    if( y >= kMatrixHeight) return -1;
    return XY(x,y);
}

#define NUM_LEDS (kMatrixWidth * kMatrixHeight)

CRGB leds_plus_safety_pixel[ NUM_LEDS + 1];
CRGB* const leds( leds_plus_safety_pixel + 1);

// -----------------------------------------------
//              Audio initialisation
// -----------------------------------------------

#define SAMPLES 64            //Must be a power of 2
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW   // Set display type  so that  MD_MAX72xx library treets it properly
#define MAX_DEVICES  4   // Total number display modules
#define CLK_PIN   13  // Clock pin to communicate with display
#define DATA_PIN  11  // Data pin to communicate with display
#define CS_PIN    10  // Control pin to communicate with display
#define  xres 32      // Total number of  columns in the display, must be <= SAMPLES/2
#define  yres 40       // Total number of  rows in the display


int MY_ARRAY[]={0, 128, 192, 224, 240, 248, 252, 254, 255}; // default = standard pattern
int MY_MODE_1[]={0, 128, 192, 224, 240, 248, 252, 254, 255}; // standard pattern
int MY_MODE_2[]={0, 128, 64, 32, 16, 8, 4, 2, 1}; // only peak pattern
int MY_MODE_3[]={0, 128, 192, 160, 144, 136, 132, 130, 129}; // only peak +  bottom point
int MY_MODE_4[]={0, 128, 192, 160, 208, 232, 244, 250, 253}; // one gap in the top , 3rd light onwards
int MY_MODE_5[]={0, 1, 3, 7, 15, 31, 63, 127, 255}; // standard pattern, mirrored vertically


double vReal[SAMPLES];
double vImag[SAMPLES];
char data_avgs[xres];

int yvalue;
int displaycolumn , displayvalue;
int peaks[xres];
const int buttonPin = 5;    // the number of the pushbutton pin
int state = HIGH;             // the current reading from the input pin
int previousState = LOW;   // the previous reading from the input pin
int displaymode = 1;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

arduinoFFT FFT = arduinoFFT();                                    // FFT object

/// --------------------------------------------------------------

void setup() {
    Serial.begin(9600);
    FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);
    FastLED.setBrightness( BRIGHTNESS );
    ADCSRA = 0b11100101;      // set ADC to free running mode and set pre-scalar to 32 (0xe5)
    ADMUX = 0b00000000;       // use pin A0 and external voltage reference
    pinMode(buttonPin, INPUT);
    delay(50);            // wait to get reference voltage stabilized
}

void displayModeChange() {
    int reading = digitalRead(buttonPin);
    if (reading == HIGH && previousState == LOW && millis() - lastDebounceTime > debounceDelay) // works only when pressed

    {

        switch (displaymode) {
            case 1:    //       move from mode 1 to 2
                displaymode = 2;
                for (int i=0 ; i<=8 ; i++ ) {
                    MY_ARRAY[i]=MY_MODE_2[i];
                }
                break;
            case 2:    //       move from mode 2 to 3
                displaymode = 3;
                for (int i=0 ; i<=8 ; i++ ) {
                    MY_ARRAY[i]=MY_MODE_3[i];
                }
                break;
            case 3:    //     move from mode 3 to 4
                displaymode = 4;
                for (int i=0 ; i<=8 ; i++ ) {
                    MY_ARRAY[i]=MY_MODE_4[i];
                }
                break;
            case 4:    //     move from mode 4 to 5
                displaymode = 5;
                for (int i=0 ; i<=8 ; i++ ) {
                    MY_ARRAY[i]=MY_MODE_5[i];
                }
                break;
            case 5:    //      move from mode 5 to 1
                displaymode = 1;
                for (int i=0 ; i<=8 ; i++ ) {
                    MY_ARRAY[i]=MY_MODE_1[i];
                }
                break;
        }

        lastDebounceTime = millis();
    }
    previousState = reading;
}

void fillColumn(int column, int height, uint8_t startHue8, int8_t yHueDelta8, int8_t xHueDelta8){
    if (column >= kMatrixWidth){
        return;
    }
    if (height >= kMatrixHeight){
        return;
    }

    uint8_t lineStartHue = startHue8;

    // Serial.print("[");
    for (int i = kMatrixHeight-1; i > 0; --i) {
        // Serial.print("=");
        if (i > kMatrixHeight-height){
            lineStartHue += yHueDelta8;
            uint8_t pixelHue = lineStartHue + xHueDelta8*column;
            leds[ XY(column, i)] = CHSV( pixelHue, 255, 255);
        }else{
            leds[ XY(column, i)] = CRGB(0, 0, 0);
        }
    }
//    for (int i = 0; i < height; ++i) {
//    }
    // Serial.print("]");
    // Serial.print("\n");
}

void loop() {
    uint32_t ms = millis();
    // ++ Sampling
    for(int i=0; i<SAMPLES; i++)
    {
        while(!(ADCSRA & 0x10));        // wait for ADC to complete current conversion ie ADIF bit set
        ADCSRA = 0b11110101 ;               // clear ADIF bit so that ADC can do next operation (0xf5)
        int value = ADC - 512 ;                 // Read from ADC and subtract DC offset caused value
        vReal[i]= value/8;                      // Copy to bins after compressing
        vImag[i] = 0;
    }
    // -- Sampling


    // ++ FFT
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    // -- FFT


    // ++ re-arrange FFT result to match with no. of columns on display ( xres )
    int step = (SAMPLES/2)/xres;
    int c=0;
    for(int i=0; i<(SAMPLES/2); i+=step)
    {
        data_avgs[c] = 0;
        for (int k=0 ; k< step ; k++) {
            data_avgs[c] = data_avgs[c] + vReal[i+k];
        }
        data_avgs[c] = data_avgs[c]/step;
        c++;
    }
    // -- re-arrange FFT result to match with no. of columns on display ( xres )


    int32_t yHueDelta32 = ((int32_t)cos16( ms * (27/1) ) * (350 / kMatrixWidth)) / 32768;
    int32_t xHueDelta32 = ((int32_t)cos16( ms * (39/1) ) * (310 / kMatrixHeight)) / 32768;
    int32_t msHueDelta32 = ms / 65536;

    // ++ send to display according measured value
    // Serial.print("---FTT---\n");
    for(int i=0; i<xres; i++)
    {
        // int MY_MODE_1[]={0, 128, 192, 224, 240, 248, 252, 254, 255}; // standard pattern
        data_avgs[i] = constrain(data_avgs[i],0,80);            // set max & min values for buckets
        data_avgs[i] = map(data_avgs[i], 0, 80, 0, yres);        // remap averaged values to yres
        yvalue=data_avgs[i];

        // Serial.print("[");
        // Serial.print(i);
        // Serial.print("]: ");
        // Serial.print((int)data_avgs[i]);
        // Serial.print("\n");

        // Serial.print("[");
        // Serial.print(i);
        // Serial.print("]: ");
        // Serial.print((int)yvalue);
        // Serial.print("\n");

        peaks[i] = peaks[i]-10;    // decay by one light
        if (yvalue > peaks[i])
            peaks[i] = yvalue ;
        yvalue = peaks[i];
        displayvalue=MY_ARRAY[yvalue];

        // Serial.print("[");
        // Serial.print("displayvalue");
        // Serial.print("]: ");
        // Serial.print((int)displayvalue);
        // Serial.print("\n");

        displaycolumn=31-i;
        fillColumn(displaycolumn, yvalue, msHueDelta32, yHueDelta32, xHueDelta32);
        // mx.setColumn(displaycolumn, displayvalue);              // for left to right
    }
    // Serial.print("---FTT end---\n");
    // -- send to display according measured value
    if( ms < 5000 ) {
        FastLED.setBrightness( scale8( BRIGHTNESS, (ms * 256) / 5000));
    } else {
        FastLED.setBrightness(BRIGHTNESS);
    }
    FastLED.show();

//    displayModeChange ();         // check if button pressed to change display mode
}
