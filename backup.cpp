/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */


// Set LED_BUILTIN if it is not defined by Arduino framework
// #define LED_BUILTIN 13

#include "Arduino.h"
#include <TimerTC3.h>
#include <U8g2lib.h>
#include <U8x8lib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define BUTTON_PIN D10 //interrupt pin int_6
#define POT_0 A0
#define POT_1 A1
#define POT_2 A2
#define POT_3 A3

#define POT_THRESHOLD 50
#define POT_POLLING 500

#define DEFAULTFONT u8x8_font_chroma48medium8_r //font for U8X8 Library

//U8X8_SH1106_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE); //use this line for the SH1106 1.3" OLED often sold as 1.3" SSD1306


void runMotors();
void displayData();
float normalizeParams();

void running();

void ISR_pauseInterrupt();
void ISR_timer();


volatile bool pauseButton = true;
volatile bool potFlag = true;

int res = 0;


unsigned long lastDebounce = 0;
unsigned long debounceDelay = 500;

//U8G2_SSD1306_128X64_ALT0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // SSD1306 and SSD1308Z are compatible

//U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SH1106_128X64_VCOMH0_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);		// same as the NONAME variant, but maximizes setContrast() range
//U8G2_SH1106_128X64_WINSTAR_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);		// same as the NONAME variant, but uses updated SH1106 init sequence
 
// U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);    //Low spped I2C
 
class params {
  public:
    int vel = 0;
    int force = 0;
    int ext = 0;
    int temp = 0;
};

params paramObj;

void setup()
{

  Serial.begin(9600); // open the serial port at 9600 bps:

  TimerTc3.initialize(POT_POLLING);
  TimerTc3.attachInterrupt(ISR_timer);

  //disp.begin();
  //disp.setFont(DEFAULTFONT);

  u8g2.begin();

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(BUTTON_PIN,INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), ISR_pauseInterrupt, FALLING);
  pinMode(POT_0, INPUT);
  pinMode(POT_1, INPUT);
  pinMode(POT_2, INPUT);
  pinMode(POT_3, INPUT);
  
}

void loop()
{

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(40,10,"Running");
    u8g2.drawFrame(5, 18, 20, 30);
    u8g2.drawBox(7, 20, 16, 26 * ((abs(paramObj.vel-950))/1000));
    u8g2.drawFrame(37, 18, 20, 30);
    u8g2.drawBox(39, 20, 16, 26);
    u8g2.drawFrame(69, 18, 20, 30);
    u8g2.drawBox(71, 20, 16, 26);
    u8g2.drawFrame(100, 18, 20, 30);
    u8g2.drawBox(102, 20, 16, 26);
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0,62,"Speed");
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(37,62,"Force");
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(71,62,"Ext.");
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(96,62,"Delay");
  } while ( u8g2.nextPage() );

  if(potFlag){
    displayData();
    potFlag = false;
  }
  if(pauseButton){
    running();
  }

}

void running(){
  
  //noop for now
  normalizeParams();

  //noop for now
  runMotors(); //need to recieve data from motors while running

  //Serial.print("running\n");
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

float normalizeParams(){

  //normalize parameter based on input type

}

void runMotors(){

  //send the information to the motors

}

void displayData()
{

  //for testing, prints to terminal
  Serial.printf("vel %d, force %d, ext %d, temp %d\n", paramObj.vel, paramObj.force, paramObj.ext, paramObj.temp);

  //production code -> shwo on screen

  char cstr [16];

  /*disp.setCursor(0, 3);
  disp.print(F("    RUNNING"));
  disp.setCursor(0, 4);
  disp.print(F("Speed: "));
  disp.print(F(itoa(paramObj.vel/100, cstr, 16)));
  disp.setCursor(0, 5);
  disp.print(F("Force:"));
  disp.print(F(itoa(paramObj.force/100, cstr, 16)));
  disp.setCursor(0, 6);
  disp.print(F("Extension: "));
  disp.print(F(itoa(paramObj.ext/100, cstr, 16)));
  disp.setCursor(0, 7);
  disp.print(F("Delay: "));
  disp.print(F(itoa(paramObj.temp/100, cstr, 16)));*/

  /*u8g2.clearDisplay();
  u8g2.firstPage();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(0,15,"Hello World!");*/

}

void ISR_pauseInterrupt(){

  if(millis() - lastDebounce > debounceDelay){
    lastDebounce = millis();
    Serial.print("interrupt - ");

  if(pauseButton)
  {
    pauseButton = !pauseButton;
    Serial.print("Stop!\n");
  }
  else
  {
    pauseButton = !pauseButton;
    Serial.print("Go\n");
  }
  }
  
}

void ISR_timer(){

  int vel0 = analogRead(POT_0);
  int force0 = analogRead(POT_1);
  int ext0 = analogRead(POT_2);
  int temp0 = analogRead(POT_3);

  long velDiff = abs(paramObj.vel - vel0);
  int forceDiff = abs(paramObj.force - force0);
  int extDiff = abs(paramObj.ext - ext0);
  int tempDiff = abs(paramObj.temp - temp0);

  if(velDiff > POT_THRESHOLD){
    paramObj.vel = vel0;
    potFlag = true;
  }

  if(forceDiff > POT_THRESHOLD){
    paramObj.force = force0;
    potFlag = true;
  }

  if(extDiff > POT_THRESHOLD){
    paramObj.ext = ext0;
    potFlag = true;
  }

  if(tempDiff > POT_THRESHOLD){
    paramObj.temp = temp0;
    potFlag = true;
  }
}
