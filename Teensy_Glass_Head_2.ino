/*  Glass Head Art/Spectrum Analyzer Version 2 using a Teensy 3.6, a 1.5" 
    OLED module w/ SDD1351 chip (128x128), (128) RGBW Neopixel LEDs and the
    Teensy Audio Adapter Board.
    
    The analog signals from the left and right line inputs are mixed and 
    then processed to create an array of frequency magnitudes.  These are
    used to create visual patterns on the Neopixel LEDs in the glass head
    and a standard spectrum analyzer display on the OLED.

    For information on the Teensy Audio adapter: 
       http://www.pjrc.com/teensy/td_libs_Audio.html

    High-level summary:
    1) The touch switches are debounced and stored
    2) Any new operating states or variables are calculated
    3) The L & R channels are mixed and binned by frequency into 512 bins
    4) The expanded set of spectrum data is grouped into 12 bands.
    5) Each of the 128 LEDS has a color & intensity calculated and displayed
    6) The OLED display is updated based on the selected mode
    7) Repeat forever
    
    Revision: 2b
    Date: 23-Feb-2019
    Leon Durivage
*/



#include <gfxfont.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>

#include <Adafruit_SSD1351.h>   // OLED library
#include <Adafruit_NeoPixel.h>  // LED library
#include <Audio.h>              // Teensy Audio library
#include <Wire.h>

#define  TESTPIN  6  // the digital pin to use as a test output
// *************************************************************** MENU STRINGS
String ledBrightness[3] = { "Low", "Med", "Hi " };
String ledMode[3] = { "In/Out", "Bottom/Top", "Demo" };
String colorShift[2] = { "Off" , "On" };
String oledMode[3] = { "Bottom" , "Middle" , "Top" };


// ************************************************************** SWITCH INPUTS
#define  TOUCHLF  29  // digital input connected to the left front position
#define  TOUCHLM   0  // digital input connected to the left middle position
#define  TOUCHLR   1  // digital input connected to the left rear position
#define  TOUCHRF  17  // digital input connected to the right front position
#define  TOUCHRM  16  // digital input connected to the right middle position
#define  TOUCHRR  30  // digital input connector to the right rear position

struct SWITCH {  // a structure to hold the input switches
  byte TouchRightFront;
  byte TouchRightMiddle;
  byte TouchRightRear;
  byte TouchLeftFront;
  byte TouchLeftMiddle;
  byte TouchLeftRear;
};

// define a structure to hold the current switch values
struct SWITCH switchesNow = {0, 0, 0, 0, 0, 0};   

// define a structure to hold the previous switch values
struct SWITCH switchesLast = {0, 0, 0, 0, 0, 0};  


// ************************************************************ OLED PARAMETERS
// 1.5" OLED Display -  uses SPI pins
// Screen dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128 

// requires specific pins to align with the Teensy Audio Adapter
//
#define SCLK_PIN 14
#define MOSI_PIN 7
#define DC_PIN   8
#define CS_PIN   20
#define RST_PIN  21

// Option 1: use any pins but a little slower
Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, CS_PIN, 
                                        DC_PIN, MOSI_PIN, SCLK_PIN, RST_PIN);
//Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, 
                                        //CS_PIN, DC_PIN, RST_PIN);

// Color definitions
#define BLACK     0x0000
#define BLUE      0x001F
#define RED       0xF800
#define GREEN     0x07E0
#define CYAN      0x07FF
#define MAGENTA   0xF81F
#define YELLOW    0xFFE0  
#define WHITE     0xFFFF

// Menu mode = full screen or just the bottom
byte maxBarHeigth[2] = {112, 64};

// ************************************************************* LED PARAMETERS
#define LED_PIN   3    // pin on the Teensy connected to the NeoPixel string
#define NUMLEDS 128    // How many NeoPixels are attached
#define RING1    19    //   How many NeoPixels are in the first, inner ring
#define RING2    27    //     Second ring
#define RING3    37    //     Third ring
#define RING4    45    //     Fourth and outer ring
  
byte colorMap = 85;         // the offset for the color map (which is 0 - 255)
byte LEDdisplayMode = 1;    // 0 = inner ring to outer; 1 = bottom to top; 2 = demo
byte LEDcolorScale = 2;
byte LEDbrightnessLevel = 1;
byte DisplayMode = 1;
byte MenuMode = 0;
byte DemoFunction = 0;
byte inputChange = 0;
byte ColorShift = 1;       // color shifting; 1 = mode on

// When we setup the NeoPixel library, we tell it how many pixels, and which 
//  pin to use to send signals.
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMLEDS, LED_PIN, NEO_GRBW + NEO_KHZ800);

// Strip maximum brightness levels
byte brightnessLevel[3] = {16, 100, 255};


// *************************************************** AUDIO ADAPTER PARAMETERS

const int my_input = AUDIO_INPUT_LINEIN; // use line input from adapter board
//const int my_input = AUDIO_INPUT_MIC;  //  example if using microphone

// Create the Audio components.  
// These should be created in the order data flows: inputs/sources -> processing -> outputs

AudioInputI2S          audio_input;         // Teensy Audio Adapter: input is both channels L&R
AudioMixer4            mixer;               // a mixer object to mix the L & R channels
AudioAnalyzeFFT1024    my_FFT;              // Compute a 1024 point Fast Fourier Transform (FFT)
                                            //  frequency analysis, with real value (magnitude) 
                                            //  output. The frequency resolution is 43 Hz, useful 
                                            //  for detailed audio visualization
AudioConnection        patchCord1(audio_input, 0, mixer, 0);  // send L to mixer input 0
AudioConnection        patchCord2(audio_input, 1, mixer, 1);  // send R to mixer input 1
AudioConnection        patchCord3(mixer, my_FFT);             // send mixed signal to FFT object
AudioOutputI2S         audio_output;        // output to the adapter's headphones & line-out

AudioControlSGTL5000   audio_adapter;       // object to send control signals to the audio adapter

// An array to hold the 12 frequency bands
float level[12];


// ********************************************************************** SETUP
void setup() {  // Set up code runs once - on power up and reset

// Debug monitor mode - comment out for final code
  Serial.begin(9600);

// configure inputs
  pinMode(TOUCHRF, INPUT);    // digital input connected to the left touch switch
  pinMode(TOUCHRM, INPUT);    // digital input connected to the right touch switch
  pinMode(TOUCHRR, INPUT);    // digital input connected to the center touch switch
  pinMode(TOUCHLF, INPUT);    // digital input connected to the right touch switch
  pinMode(TOUCHLM, INPUT);    // digital input connected to the center touch switch
  pinMode(TOUCHLR, INPUT);    // digital input connected to the center touch switch
  
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(32);
  
  tft.begin();
  tft.setRotation(2);
  tft.fillScreen(BLACK);
  
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(0,0);
  tft.println("GlassHead2");

  
  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(12);

  // Enable the audio shield and set the output volume.
  audio_adapter.enable();
  audio_adapter.inputSelect(my_input);
  audio_adapter.volume(0.5);

  // Configure the window algorithm to use
  //  Window Types:

  //  AudioWindowHanning1024 (default)
  //  AudioWindowBartlett1024
  //  AudioWindowBlackman1024
  //  AudioWindowFlattop1024
  //  AudioWindowBlackmanHarris1024
  //  AudioWindowNuttall1024
  //  AudioWindowBlackmanNuttall1024
  //  AudioWindowWelch1024
  //  AudioWindowHamming1024
  //  AudioWindowCosine1024
  //  AudioWindowTukey1024
  my_FFT.windowFunction(AudioWindowHanning1024);

  pinMode(TESTPIN,OUTPUT);  // initialize the loop test pin as an output
  digitalWrite(TESTPIN,LOW);

  inputChange = 0;
}

// *********************************************************************** LOOP
void loop() {
  int i,barHeight;

  digitalWrite(TESTPIN,LOW);
  
// Code start
// ************************************************************** Read Switches
  switchesNow = debounceSwitch();     // read the knife and touch switches

// process the RIGHT FRONT touch swith as LED Mode select switch
  if (switchesNow.TouchRightFront >= 1 && switchesLast.TouchRightFront == 0) {  // 0 to >1 means it was touched
    LEDdisplayMode = (LEDdisplayMode + 1) % 3;  // increment LED Mode from 0 to 2 and wrap around
    switchesLast.TouchRightFront = switchesNow.TouchRightFront;
    inputChange = 1;
  }
  if (switchesNow.TouchRightFront == 0 && switchesLast.TouchRightFront >= 1) {  // 1 to 0 means it was released
    switchesLast.TouchRightFront = switchesNow.TouchRightFront;
  }

// process the RIGHT MIDDLE touch swith as LED Color Map select switch
  if (switchesNow.TouchRightMiddle >= 1 && switchesLast.TouchRightMiddle == 0) {  // 0 to >1 means it was touched
    LEDcolorScale = (LEDcolorScale + 1) % 3;              // increment LED Color Map from 0 to 2 and wrap around
    switchesLast.TouchRightMiddle = switchesNow.TouchRightMiddle;
    inputChange = 1;
  }
  if (switchesNow.TouchRightMiddle == 0 && switchesLast.TouchRightMiddle >= 1) {  // 1 to 0 means it was released
    switchesLast.TouchRightMiddle = switchesNow.TouchRightMiddle;
  }

// process the RIGHT REAR touch swith as LED Brightness Level select switch
  if (switchesNow.TouchRightRear >= 1 && switchesLast.TouchRightRear == 0) {  // 0 to >1 means it was touched
    LEDbrightnessLevel = (LEDbrightnessLevel + 1) % 3;              // increment LED Brightness Level from 0 to 2 and wrap around
    switchesLast.TouchRightRear = switchesNow.TouchRightRear;
    inputChange = 1;
  }
  if (switchesNow.TouchRightRear == 0 && switchesLast.TouchRightRear >= 1) {  // 1 to 0 means it was released
    switchesLast.TouchRightRear = switchesNow.TouchRightRear;
  }

// process the LEFT FRONT touch swith as Display Mode select switch
  if (switchesNow.TouchLeftFront >= 1 && switchesLast.TouchLeftFront == 0) {  // 0 to >1 means it was touched
    DisplayMode = (DisplayMode + 1) % 3;   // increment Display Mode from 0 to 2 and wrap around
    switchesLast.TouchLeftFront = switchesNow.TouchLeftFront;
    inputChange = 1;
  }
  if (switchesNow.TouchLeftFront == 0 && switchesLast.TouchLeftFront >= 1) {  // 1 to 0 means it was released
    switchesLast.TouchLeftFront = switchesNow.TouchLeftFront;
  }

// process the LEFT MIDDLE touch swith as Color Shift On/Off select switch
  if (switchesNow.TouchLeftMiddle >= 1 && switchesLast.TouchLeftMiddle == 0) {  // 0 to >1 means it was touched
    ColorShift = (ColorShift + 1) % 2;   // increment Color Shift from 0 to 1 and wrap around
    switchesLast.TouchLeftMiddle = switchesNow.TouchLeftMiddle;
    inputChange = 1;
  }
  if (switchesNow.TouchLeftMiddle == 0 && switchesLast.TouchLeftMiddle >= 1) {  // 1 to 0 means it was released
    switchesLast.TouchLeftMiddle = switchesNow.TouchLeftMiddle;
  }

// process the LEFT REAR touch swith as Menu On/Off select switch
  if (switchesNow.TouchLeftRear >= 1 && switchesLast.TouchLeftRear == 0) {  // 0 to >1 means it was touched
    MenuMode = (MenuMode + 1) % 2;    // increment Menu Mode from 0 to 1 and wrap around
    switchesLast.TouchLeftRear = switchesNow.TouchLeftRear;
    inputChange = 1;
  }
  if (switchesNow.TouchLeftRear == 0 && switchesLast.TouchLeftRear >= 1) {  // 1 to 0 means it was released
    switchesLast.TouchLeftRear = switchesNow.TouchLeftRear;
  }

// Rotate colors every pass if Color Shift mode selected
  if (ColorShift == 1 ) {   // High means it is selected
    colorMap = (colorMap + 1) % 255;    // increment color Map index counter as long as it's open
  }

  if (MenuMode == 1 && inputChange == 1){
    tft.fillRect( 0, 0, 128, 64, BLACK);
    
    tft.setTextSize(2);
    tft.setTextColor(WHITE);
    tft.setCursor(0,0);
    tft.println("GlassHead2");
    
    tft.setTextSize(1);
    tft.setCursor(0,16); 
    tft.print("LED Mode:");
    tft.println(ledMode[LEDdisplayMode]);

    tft.print("LED Scale:");
    tft.print(LEDcolorScale + 1);
    tft.println(" color");    

    tft.print("LED brightness:");
    tft.println(ledBrightness[LEDbrightnessLevel]);

    tft.print("Display Mode:");
    tft.println(DisplayMode);
  
    tft.print("Color Shift:");
    tft.println(colorShift[ColorShift]);

    inputChange = 0;

  }
  if (MenuMode == 0 && inputChange == 1){
    tft.fillRect( 0, 0, 128, 64, BLACK);
    tft.setTextSize(2);
    tft.setTextColor(WHITE);
    tft.setCursor(0,0);
    tft.println("GlassHead2");
    inputChange = 0;
  }
  
  digitalWrite(TESTPIN,LOW);

// FFT ******************************************************************** FFT
  if (my_FFT.available()) 
  {
    // each time new FFT data is available
    // print it all to the Arduino Serial Monitor
    // 7.6msec my_FFT.windowFunction(AudioWindowHamming1024);

    level[0] =  my_FFT.read(0, 1) * 1.5;
    level[1] =  my_FFT.read(2, 3) * 1.5;
    level[2] =  my_FFT.read(4, 6) * 1.4;
    level[3] =  my_FFT.read(7, 11) * 1.3;
    level[4] =  my_FFT.read(12, 19) * 1.2;
    level[5] =  my_FFT.read(20, 31) * 1.1;
    level[6] =  my_FFT.read(32, 50);
    level[7] =  my_FFT.read(51, 78);
    level[8] =  my_FFT.read(79, 119);
    level[9] =  my_FFT.read(120, 176);
    level[10] = my_FFT.read(177, 254); 
    level[11] = my_FFT.read(255, 511) * .7; 

  }
  digitalWrite(TESTPIN,LOW);
  
// LED Display **************************************************** LED Display

// Set the overall brightness of the LEDs based on the left touch switch
  strip.setBrightness(brightnessLevel[LEDbrightnessLevel]);
 
  if (LEDdisplayMode == 0) {           // Inside to outside - LED 1 through 128
    for (int i = 0; i < NUMLEDS; i++){   // cycle through all the LEDs
      byte LEDcolor = ((i * LEDcolorScale)/2 + colorMap) % 255;  // this results in a color map position
      byte Channel = i / (NUMLEDS / 12) ;     // there are 12 frequency bands, which scales to 128/12 = 11
 
      byte Red1 = (WheelR(LEDcolor) * level[Channel]) ;   // brightness proportionally to the loudness
      byte Green1 = (WheelG(LEDcolor) * level[Channel]) ; //  of the individual channel
      byte Blue1 = (WheelB(LEDcolor) * level[Channel]) ;    
      
      strip.setPixelColor(i, strip.Color(Red1, Green1, Blue1, 0));  // set the LED Color
    }
  }
  
  if (LEDdisplayMode == 1) {                    // Bottom to top - ring-by-ring
    for (int i = 0; i < RING1; i++){   // cycle through all the LEDs in the first ring
      byte LEDcolor = (i * LEDcolorScale + colorMap) % 255;  // this results in a color map position
      byte Channel = (i * 12) / RING1 ;     // there are 12 frequency bands, which scales to RING1/12
 
      byte Red1 = (WheelR(LEDcolor) * level[Channel]) ;   // brightness proportionally to the loudness
      byte Green1 = (WheelG(LEDcolor) * level[Channel]) ; //  of the individual channel
      byte Blue1 = (WheelB(LEDcolor) * level[Channel]) ;    
      
      strip.setPixelColor(i, strip.Color(Red1, Green1, Blue1, 0));  // set the LED color
    }
    for (int i = 0; i < RING2; i++){   // cycle through all the LEDs in the second ring
      byte LEDcolor = (i * LEDcolorScale + colorMap) % 255;  // this results in a color map position
      byte Channel = (i * 12) / RING2 ;     // there are 12 frequency bands, which scales to 128/12
 
      byte Red1 = (WheelR(LEDcolor) * level[Channel]) ;   // brightness proportionally to the loudness
      byte Green1 = (WheelG(LEDcolor) * level[Channel]) ; //  of the individual channel
      byte Blue1 = (WheelB(LEDcolor) * level[Channel]) ;    
      
      strip.setPixelColor(RING1 + i, strip.Color(Red1, Green1, Blue1, 0));  // set the LED Color
    }
    for (int i = 0; i < RING3; i++){   // cycle through all the LEDs in the third ring
      byte LEDcolor = (i * LEDcolorScale + colorMap) % 255;  // this results in a color map position
      byte Channel = (i * 12) / RING3 ;     // there are 12 frequency bands, which scales to 128/12
 
      byte Red1 = (WheelR(LEDcolor) * level[Channel]) ;   // brightness proportionally to the loudness
      byte Green1 = (WheelG(LEDcolor) * level[Channel]) ; //  of the individual channel
      byte Blue1 = (WheelB(LEDcolor) * level[Channel]) ;    
      
      strip.setPixelColor(RING1 + RING2 + i, strip.Color(Red1, Green1, Blue1, 0));  // set the LED color
    }
    for (int i = 0; i < RING4; i++){   // cycle through all the LEDs in the fourth ring
      byte LEDcolor = (i * LEDcolorScale + colorMap) % 255;  // this results in a color map position
      byte Channel = (i * 12) / RING4 ;     // there are 12 frequency bands, which scales to 128/12
 
      byte Red1 = (WheelR(LEDcolor) * level[Channel]) ;   // brightness proportionally to the loudness
      byte Green1 = (WheelG(LEDcolor) * level[Channel]) ; //  of the individual channel
      byte Blue1 = (WheelB(LEDcolor) * level[Channel]) ;    
      
      strip.setPixelColor(RING1 + RING2 + RING3 + i, strip.Color(Red1, Green1, Blue1, 0));  // set the LED color
    }
  }
  
  if (LEDdisplayMode == 2) {  // Demo Mode
    for (int i = 0; i < NUMLEDS; i++){   // cycle through all the LEDs
 
      byte Red1 = WheelR(random(255)) ;   
      byte Green1 = WheelG(random(255)) ; 
      byte Blue1 = WheelB(random(255)) ;    
      
      strip.setPixelColor(i, strip.Color(Red1, Green1, Blue1, 0));  // Bottom to Top
    }
  } 

  strip.show();   //show the new strip colors 
  
  digitalWrite(TESTPIN,HIGH);

// Spectrum display ****************************************** Spectrum Display
  
  if (DisplayMode == 0) {     // Standard bargraph starting at the bottom
    for (i = 0; i < 12; i++) {   // cycle through the twelve frequncy channels
      barHeight = maxBarHeigth[MenuMode] * level[i];  //calculate the bar height

      if (barHeight > maxBarHeigth[MenuMode]) {       // fix at a maximum of 100 pixels
        barHeight = maxBarHeigth[MenuMode];
      }

      byte LCDcolor = (i * LEDcolorScale * 6 + colorMap) % 255;  // this results in a color map position

      for (byte j = 0; j < 8; j++){   // draw muliple lines to create the bar
        // first draw a color line then a black line to the top
        tft.drawFastVLine( i*10 + j, 128-barHeight, barHeight, RGB565_Wheel(LCDcolor));        
        tft.drawFastVLine( i*10 + j, 128 - maxBarHeigth[MenuMode], maxBarHeigth[MenuMode]-barHeight, BLACK);

      }
    }
  }

  if (DisplayMode == 1) {  // Bars mirrored from the center
    for (i = 0; i < 12; i++) {   // cycle through the twelve frequncy channels
      barHeight = maxBarHeigth[MenuMode] * level[i];  //calculate the bar height

      if (barHeight > maxBarHeigth[MenuMode]) {       // fix at a maximum of 100 pixels
        barHeight = maxBarHeigth[MenuMode];
      }

      byte LCDcolor = (i * LEDcolorScale * 6 + colorMap) % 255;  // this results in a color map position

      int top = 128 - maxBarHeigth[MenuMode];
      int seg1 = 128 - ((maxBarHeigth[MenuMode] / 2) + (barHeight / 2));
      
      for (byte j = 0; j < 8; j++){   // draw muliple lines to create the bar
        // first draw a color line then a black line to the top
        //tft.drawFastVLine( i*10 + j, 128-barHeight, barHeight, RGB565_Wheel(LCDcolor));        
        //tft.drawFastVLine( i*10 + j, 128 - maxBarHeigth[MenuMode], maxBarHeigth[MenuMode]-barHeight, BLACK);
        tft.drawFastVLine( i*10 + j, top, maxBarHeigth[MenuMode], BLACK);        
        tft.drawFastVLine( i*10 + j, seg1, barHeight, RGB565_Wheel(LCDcolor));   
      }
    }
  }


  
  digitalWrite(TESTPIN,LOW);
}

// **************************************************************** SUBROUTINES

//***************************************************************************
// debounceSwitch: reads all the touch switch inputs then divides by 2500
//   to return a "1" or greater if the input is touched

struct SWITCH debounceSwitch () {
  struct SWITCH sw = {0,0,0,0,0,0};
  
  sw.TouchRightFront = touchRead(TOUCHRF) / 2500;
  sw.TouchRightMiddle = touchRead(TOUCHRM) / 2500;
  sw.TouchRightRear = touchRead(TOUCHRR) / 2500;
  sw.TouchLeftFront = touchRead(TOUCHLF) / 2500;
  sw.TouchLeftMiddle = touchRead(TOUCHLM) / 2500;
  sw.TouchLeftRear = touchRead(TOUCHLR) / 2500;

   /* Comment out debug messages for final code
    Serial.print("RF:");
    Serial.print(sw.TouchRightFront);
    Serial.print(" ");
    Serial.print("RM:");
    Serial.print(sw.TouchRightMiddle);
    Serial.print(" ");
    Serial.print("RR:");
    Serial.print(sw.TouchRightRear);
    Serial.print(" ");
    Serial.print("LF:");
    Serial.print(sw.TouchLeftFront);
    Serial.print(" ");    
    Serial.print("LM:");
    Serial.print(sw.TouchLeftMiddle);
    Serial.print(" ");    
    Serial.print("LR:");
    Serial.print(sw.TouchLeftRear);
    Serial.println(); 
   */
        
  return (sw);
}

//***************************************************************************
// RGB565_Wheel - Input 0 to 255 to get a color value in RGB565 format.
//  The colors transition from Red to Green to Blue then back to Red.

unsigned int  RGB565_Wheel(byte WheelPos) {
   unsigned int red, green, blue;
  
   WheelPos = 255 - WheelPos;
   if (WheelPos < 85) {
     red = (255 - WheelPos * 3)/8;
     green = 0;
     blue = (WheelPos * 3)/8;
   } 
   else if (WheelPos < 170) {
     WheelPos -= 85;
     red = 0;
     green = (WheelPos * 3)/4;
     blue = (255 - WheelPos * 3)/8;
   } 
   else {
     WheelPos -= 170;
     red = (WheelPos * 3)/8;
     green = (255 - WheelPos * 3)/4;
     blue = 0;
   }
   red = red << 11;
   green = green << 5;
   return ( red + green + blue );
}

//***************************************************************************
// WheelR: just return the Red byte (not the entire RGB word)
byte WheelR(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return 255 - WheelPos * 3; 
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return 0;
  } else {
    WheelPos -= 170;
    return WheelPos * 3;
  }
}

//***************************************************************************
// WheelG: just return the Green byte (not the entire RGB word)
byte WheelG(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return 0; 
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return WheelPos * 3;
  } else {
    WheelPos -= 170;
    return 255 - WheelPos * 3;
  }
}

//***************************************************************************
// WheelB: just return the Blue byte (not the entire RGB word)
byte WheelB(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return WheelPos * 3; 
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return 255 - WheelPos * 3;
  } else {
    WheelPos -= 170;
    return 0;
  }
}
