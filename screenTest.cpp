// #define LED_BUILTIN 13

#include "header.h"

//U8X8_SH1106_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE); //use this line for the SH1106 1.3" OLED often sold as 1.3" SSD1306


volatile bool pauseButton = true;
volatile bool pauseButton0 = false;
volatile bool potFlag = true;

int res = 0;

char topStr[] = "     Running";//"00r/s   Running   00s";

unsigned long lastDebounce = 0;
unsigned long debounceDelay = 500;

//U8G2_SSD1306_128X64_ALT0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // SSD1306 and SSD1308Z are compatible

//U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
//U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SH1107_128X128_1_HW_I2C u8g2( U8G2_R3, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE); //[page buffer, size = 128 bytes]
//U8G2_SH1106_128X64_VCOMH0_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);		// same as the NONAME variant, but maximizes setContrast() range
//U8G2_SH1106_128X64_WINSTAR_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);		// same as the NONAME variant, but uses updated SH1106 init sequence
 
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);    //Low spped I2C

params paramObj;

//Motor Control
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

double calibratedPos = 0;
#define POS_THRESHOLD 3
#define GLOBAL_DIR 1

#define CAL_BUTTON D8


void setup()
{

  Serial.begin(115200); // open the serial port at 9600 bps:

  //display setup
  u8g2.begin();

 
}

void loop()
{

  displayData();
  Serial.print("running");
  delay(1000);

}

void displayData(){

  int offset = 0;

  u8g2.firstPage();
  do {
    
      u8g2.drawStr(40,offset + 10,"Running");
    
    // u8g2.drawFrame(5, offset + 18, 20, 30);
    // u8g2.drawBox( 7, offset + 46 - int(paramObj.dispVel[calMot] * 26), 16, int(paramObj.dispVel[calMot] * 26));
    // u8g2.drawFrame(37, offset + 18, 20, 30);
    // u8g2.drawBox(39, offset + 46 - int(paramObj.dispForce[calMot] * 26), 16, int(paramObj.dispForce[calMot] * 26));
    // u8g2.drawFrame(69, offset + 18, 20, 30);
    // u8g2.drawBox(71, offset + 46 - int(paramObj.dispExt[calMot] * 26), 16, int(paramObj.dispExt[calMot] * 26));
    // u8g2.drawFrame(100, offset + 18, 20, 30);
    // u8g2.drawBox(102, offset + 46 - int(paramObj.dispTemp[calMot] * 26), 16, int(paramObj.dispTemp[calMot] * 26));
    // u8g2.drawStr(0,offset + 60,"Speed");
    // u8g2.drawStr(37,offset + 60,"Force");
    // u8g2.drawStr(71,offset + 60,"Ext.");
    // u8g2.drawStr(96,offset + 60,"Delay");
    // char buff[10];
    // u8g2.drawStr(100, offset + 10, itoa((int)(paramObj.dispTemp[calMot] * 7), buff, 10));
    // u8g2.drawStr(115, offset + 10, "s");
    // u8g2.drawStr(0, offset + 10, itoa((int)(paramObj.dispVel[calMot] * 5), buff, 10));
    // u8g2.drawStr(15, offset + 10, "r/s");
  } while ( u8g2.nextPage() );
}