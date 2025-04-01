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
U8G2_SH1107_128X128_1_HW_I2C u8g2( U8G2_R3, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE); //[page buffer, size = 128 bytes]
//U8G2_SH1106_128X64_VCOMH0_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);		// same as the NONAME variant, but maximizes setContrast() range
//U8G2_SH1106_128X64_WINSTAR_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);		// same as the NONAME variant, but uses updated SH1106 init sequence
 
// U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);    //Low spped I2C
 
class params {
  public:
    float vel = 0;
    float force = 0;
    float ext = 0;
    float temp = 0;

    float dispVel = 0;
    float dispForce = 0;
    float dispExt = 0;
    float dispTemp = 0;
};

params paramObj;

//Motor Control
const uint8_t DXL_ID_0 = 1;
const uint8_t DXL_ID_1 = 2;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

double calibratedPos = 0;
#define POS_THRESHOLD 3
#define GLOBAL_DIR 1

#define CAL_BUTTON D8


void setup()
{

  // Serial.begin(115200); // open the serial port at 9600 bps:

  // TimerTc3.initialize(POT_POLLING);
  // TimerTc3.attachInterrupt(ISR_timer);

  // //display setup
  // u8g2.begin();

  // //motor setup
  // // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // // Get DYNAMIXEL information

  delay(5000);

  for(int i = 0; i < 5; i++){
    if(dxl.ping(i)){
      Serial.print("pinged: ");
      Serial.println(i);
    }
  }

  delay(5000);

  // Serial.println("done pinging");
  // dxl.ping(DXL_ID_0);
  // // dxl.ping(DXL_ID_1);

  // // // Turn off torque when configuring items in EEPROM area
  //dxl.torqueOff(1);
  //dxl.setID(1, 2);
  
  delay(500);
  dxl.ledOn(2);

  


  // dxl.setOperatingMode(DXL_ID_0, OP_CURRENT_BASED_POSITION);
  // dxl.torqueOn(DXL_ID_0);
  // dxl.torqueOff(DXL_ID_1);
  // dxl.setOperatingMode(DXL_ID_1, OP_CURRENT_BASED_POSITION);
  // dxl.torqueOn(DXL_ID_1);

  // // initialize LED digital pin as an output.
  // pinMode(LED_BUILTIN, OUTPUT);

  // pinMode(BUTTON_PIN,INPUT_PULLDOWN);
  // pinMode(CAL_BUTTON,INPUT_PULLDOWN);
  // attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), ISR_pauseInterrupt, FALLING);
  // pinMode(POT_0, INPUT);
  // pinMode(POT_1, INPUT);
  // pinMode(POT_2, INPUT);
  // pinMode(POT_3, INPUT);

  // displayData();
  // delay(1000);
  // // Turn on the LED on DYNAMIXEL
  // dxl.ledOn(DXL_ID_0);
  // dxl.ledOn(DXL_ID_1);
  // // Turn off the LED on DYNAMIXEL
  // calibratedPos = calibrate(DXL_ID_0);

  // //TODO: Seperate each calibrated position for both motors
  // calibratedPos = calibrate(DXL_ID_1);

  // delay(500);

  // dxl.ledOff(DXL_ID_0);  
  // dxl.ledOff(DXL_ID_1);  
}

void loop()
{

  // displayData();

  // if(pauseButton){
  //   //running();
  //   delay(paramObj.dispTemp * MAX_DELAY);
  //   //calibrate();
  //   flex();
  //   delay(paramObj.dispTemp * MAX_DELAY);
  //   while(extend() == 0){
  //   };
  // }

}
