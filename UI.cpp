/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */


// Set LED_BUILTIN if it is not defined by Arduino framework
// #define LED_BUILTIN 13

#include "header.h"

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

char topStr[] = "     Running";//"00r/s   Running   00s";

unsigned long lastDebounce = 0;
unsigned long debounceDelay = 500;

//U8G2_SSD1306_128X64_ALT0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // SSD1306 and SSD1308Z are compatible

//U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
//U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);
U8G2_SH1107_128X128_1_HW_I2C u8g2( U8G2_R1, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE); //[page buffer, size = 128 bytes]
//U8G2_SH1106_128X64_VCOMH0_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);		// same as the NONAME variant, but maximizes setContrast() range
//U8G2_SH1106_128X64_WINSTAR_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);		// same as the NONAME variant, but uses updated SH1106 init sequence
 
// U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);    //Low spped I2C
 
class params {
  public:
    int vel = 0;
    int force = 0;
    int ext = 0;
    int temp = 0;

    float dispVel = 0;
    float dispForce = 0;
    float dispExt = 0;
    float dispTemp = 0;
};

params paramObj;

//Motor Control
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;


void setup()
{

  Serial.begin(115200); // open the serial port at 9600 bps:

  TimerTc3.initialize(POT_POLLING);
  TimerTc3.attachInterrupt(ISR_timer);

  //display setup
  u8g2.begin();

  //motor setup
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  dxl.torqueOn(DXL_ID);

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
  displayData(); //use this in final code

  /*if(potFlag){
    //displayData(); //use this when troubleshooting pot data
    potFlag = false;
  }*/
  if(pauseButton){
    running();
  }

}

void running(){
  
  //noop for now
  //normalizeParams();
  //temporarily flashes LED to simulate running
  digitalWrite(LED_BUILTIN, HIGH);

  //noop for now
  runMotors(); //need to recieve data from motors while running
  
  //temporarily flashes LED to simulate running
  digitalWrite(LED_BUILTIN, LOW);
}

float normalizeParams(){

  //normalize parameter based on input type
  paramObj.dispVel = 26 * ((float)paramObj.vel/850.0);
  if(paramObj.dispVel > 26){
    paramObj.dispVel = 26;
  }
  paramObj.dispForce = 26 * ((float)paramObj.force/850.0);
  if(paramObj.dispForce > 26){
    paramObj.dispForce = 26;
  }
  paramObj.dispExt = 26 * ((float)paramObj.ext/850.0);
  if(paramObj.dispExt > 26){
    paramObj.dispExt = 26;
  }
  paramObj.dispTemp = 26 * ((float)paramObj.temp/850.0);
  if(paramObj.dispTemp > 26){
    paramObj.dispTemp = 26;
  }

}

void runMotors(){

  //send the information to the motors

  //set motor speed to normalized velocity in the range of (0<->25) * 5
  dxl.setGoalVelocity(DXL_ID, paramObj.dispVel * 5);
  Serial.print("set vel to: ");
  Serial.println(paramObj.dispVel * 5);

}

void displayData()
{

  //for testing, prints to terminal
  /*Serial.printf("vel %d, force %d, ext %d, temp %d\n", paramObj.vel, paramObj.force, paramObj.ext, paramObj.temp);
  Serial.print("vel ");
  Serial.println( paramObj.dispVel, 3);// + "force " + paramObj.dispForce + "ext " + paramObj.dispExt + "temp " + paramObj.dispTemp);*/

  //production code -> show on screen

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    if(pauseButton){
      u8g2.drawStr(40,10,"Running");
    }
    else{
      u8g2.drawStr(40,10,"Paused");
    }
    u8g2.drawFrame(5, 18, 20, 30);
    u8g2.drawBox(7, 20 + paramObj.dispVel, 16, 26 - paramObj.dispVel);
    u8g2.drawFrame(37, 18, 20, 30);
    u8g2.drawBox(39, 20 + paramObj.dispForce, 16, 26 - paramObj.dispForce);
    u8g2.drawFrame(69, 18, 20, 30);
    u8g2.drawBox(71, 20 + paramObj.dispExt, 16, 26 - paramObj.dispExt);
    u8g2.drawFrame(100, 18, 20, 30);
    u8g2.drawBox(102, 20 + paramObj.dispTemp, 16, 26 - paramObj.dispTemp);
    u8g2.drawStr(0,60,"Speed");
    u8g2.drawStr(37,60,"Force");
    u8g2.drawStr(71,60,"Ext.");
    u8g2.drawStr(96,60,"Delay");
    char buff[10];
    u8g2.drawStr(100, 10, itoa((int)(abs(paramObj.dispTemp-25)), buff, 10));
    u8g2.drawStr(115, 10, "s");
    u8g2.drawStr(0, 10, itoa((int)(abs(paramObj.dispVel-25)), buff, 10));
    u8g2.drawStr(15, 10, "r/s");
  } while ( u8g2.nextPage() );
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

  normalizeParams();
}
