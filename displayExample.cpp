//#include <U8x8lib.h>//get library here >  https://github.com/olikraus/u8g2 

#include "header.h"

//Important - select the type of OLED here
//U8X8_SSD1306_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE);  //use this line for standard 0.96" SSD1306
//U8G2_SH1107_128X128_1_HW_I2C disp( U8G2_R1, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE); //[page buffer, size = 128 bytes]
U8X8_SH1106_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE); //use this line for the SH1106 1.3" OLED often sold as 1.3" SSD1306

#define DEFAULTFONT u8x8_font_chroma48medium8_r //font for U8X8 Library

uint16_t writecount;
uint32_t startwritemS, endwritemS, timemS;


void loop()
{
  writecount++;
  Serial.print(writecount);
  Serial.print(F(" Writing to display"));

  startwritemS = millis();
  disp.clear();
  screen1();
  endwritemS = millis();
  timemS = endwritemS - startwritemS;
  disp.setCursor(8, 4);
  disp.print(timemS);
  disp.print(F("mS"));

  Serial.print(F(" - done "));
  Serial.print(timemS);
  Serial.println(F("mS"));

  delay(2000);
}


void screen1()
{
  disp.setCursor(0, 0);
  disp.print(F("Hello World !"));
  disp.setCursor(0, 1);
  disp.print(F("Line 1"));
  disp.setCursor(0, 2);
  disp.print(F("Line 2"));
  disp.setCursor(0, 3);
  disp.print(F("Line 3"));
  disp.setCursor(0, 4);
  disp.print(F("Line 4"));
  disp.setCursor(0, 5);
  disp.print(F("Line 5"));
  disp.setCursor(0, 6);
  disp.print(F("Line 6"));
  disp.setCursor(0, 7);
  disp.print(F("0123456789012345")); //display is 8 lines x 16 charaters when using the
}


void setup()
{
  Serial.begin(115200);
  Serial.println(F("SH1107_128X128_1_HW_I2C_Checker starting"));
  disp.begin();
  disp.setFont(DEFAULTFONT);
}