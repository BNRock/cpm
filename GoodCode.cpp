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
  dxl.setOperatingMode(DXL_ID, OP_CURRENT_BASED_POSITION);
  dxl.torqueOn(DXL_ID);

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(BUTTON_PIN,INPUT_PULLDOWN);
  pinMode(CAL_BUTTON,INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), ISR_pauseInterrupt, FALLING);
  pinMode(POT_0, INPUT);
  pinMode(POT_1, INPUT);
  pinMode(POT_2, INPUT);
  pinMode(POT_3, INPUT);

  displayData();
  delay(1000);
  // Turn on the LED on DYNAMIXEL
  dxl.ledOn(DXL_ID);
  // Turn off the LED on DYNAMIXEL
  calibratedPos = calibrate();

  delay(500);

  dxl.ledOff(DXL_ID);  
}

void loop()
{

  displayData();

  if(pauseButton){
    //running();
    delay(paramObj.dispTemp * MAX_DELAY);
    //calibrate();
    flex();
    delay(paramObj.dispTemp * MAX_DELAY);
    while(extend() == 0){
    };
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

void flex(){
  double updatedPos = calibratedPos;

  double targPos = paramObj.dispExt * MAX_FLEX;
  double targCurr = paramObj.dispForce * MAX_CURR;
  //send the information to the motors
  dxl.setGoalCurrent(DXL_ID, targCurr, UNIT_PERCENT);
  dxl.setGoalPosition(DXL_ID, updatedPos + targPos, UNIT_DEGREE);

  while(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) < updatedPos + targPos - POS_THRESHOLD){
    targCurr = paramObj.dispForce * MAX_CURR;
    dxl.setGoalCurrent(DXL_ID, targCurr, UNIT_PERCENT);
    targPos = paramObj.dispExt * 180;
    //Serial.print(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE));
  }

  // Print present current
  dxl.setGoalCurrent(DXL_ID, 0);
}

int extend(){
  double targCurr = paramObj.dispForce * MAX_CURR;

  dxl.setGoalCurrent(DXL_ID, -1 * targCurr, UNIT_PERCENT);
  dxl.setGoalPosition(DXL_ID, calibratedPos, UNIT_DEGREE);

  while(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE ) > calibratedPos + POS_THRESHOLD){
    targCurr = paramObj.dispForce * MAX_CURR;
    dxl.setGoalCurrent(DXL_ID, -1 * targCurr, UNIT_PERCENT);
  }

  dxl.setGoalCurrent(DXL_ID, 0);

  if(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE ) > calibratedPos + POS_THRESHOLD){
    return 0;
  }
  return 1;
}

float normalizeParams(){

  //normalize parameter based on input type
  paramObj.dispVel = ((float)(fabs(paramObj.vel - 1023)/1023));
  if(paramObj.dispVel > 26){
    paramObj.dispVel = 26;
  }
  paramObj.dispForce = ((float)(fabs(paramObj.force - 1023)/1023));
  if(paramObj.dispForce > 26){
    paramObj.dispForce = 26;
  }
  paramObj.dispExt = ((float)(fabs(paramObj.ext - 1023)/1023));
  if(paramObj.dispExt > 26){
    paramObj.dispExt = 26;
  }
  paramObj.dispTemp = ((float)(fabs(paramObj.temp - 1023)/1023));
  if(paramObj.dispTemp > 26){
    paramObj.dispTemp = 26;
  }

}

void runMotors(){

  
  double updatedPos = calibrate();

  double targPos = 270;
  double targCurr = 5;
  //send the information to the motors
  dxl.setGoalCurrent(DXL_ID, targCurr, UNIT_PERCENT);
  dxl.setGoalPosition(DXL_ID, updatedPos + targPos, UNIT_DEGREE);

  while(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE) < updatedPos + targPos - POS_THRESHOLD){
    Serial.print(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE));
  }

  // Print present current
  dxl.setGoalCurrent(DXL_ID, 0);
  delay(paramObj.dispTemp * 100);  

  dxl.setGoalCurrent(DXL_ID, -1 * targCurr, UNIT_PERCENT);
  dxl.setGoalPosition(DXL_ID, calibratedPos, UNIT_DEGREE);

  while(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE ) > calibratedPos + POS_THRESHOLD){
    Serial.print(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE));
  }

  dxl.setGoalCurrent(DXL_ID, 0);
  delay(paramObj.dispTemp * 100);

}

void displayData(){

  int offset = 50;

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    if(pauseButton){
      u8g2.drawStr(40,offset + 10,"Running");
    }
    else{
      u8g2.drawStr(40,offset + 10,"Paused");
    }
    u8g2.drawFrame(5, offset + 18, 20, 30);
    u8g2.drawBox( 7, offset + 46 - int(paramObj.dispVel * 26), 16, int(paramObj.dispVel * 26));
    u8g2.drawFrame(37, offset + 18, 20, 30);
    u8g2.drawBox(39, offset + 46 - int(paramObj.dispForce * 26), 16, int(paramObj.dispForce * 26));
    u8g2.drawFrame(69, offset + 18, 20, 30);
    u8g2.drawBox(71, offset + 46 - int(paramObj.dispExt * 26), 16, int(paramObj.dispExt * 26));
    u8g2.drawFrame(100, offset + 18, 20, 30);
    u8g2.drawBox(102, offset + 46 - int(paramObj.dispTemp * 26), 16, int(paramObj.dispTemp * 26));
    u8g2.drawStr(0,offset + 60,"Speed");
    u8g2.drawStr(37,offset + 60,"Force");
    u8g2.drawStr(71,offset + 60,"Ext.");
    u8g2.drawStr(96,offset + 60,"Delay");
    char buff[10];
    u8g2.drawStr(100, offset + 10, itoa((int)(paramObj.dispTemp * 7), buff, 10));
    u8g2.drawStr(115, offset + 10, "s");
    u8g2.drawStr(0, offset + 10, itoa((int)(paramObj.dispVel * 5), buff, 10));
    u8g2.drawStr(15, offset + 10, "r/s");
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

  int velDiff = abs(paramObj.vel - vel0);
  int forceDiff = abs(paramObj.force - force0);
  int extDiff = abs(paramObj.ext - ext0);
  int tempDiff = abs(paramObj.temp - temp0);

  if(velDiff > POT_THRESHOLD){
    paramObj.vel = vel0;
    potFlag = true;
    displayData();
  }

  if(forceDiff > POT_THRESHOLD){
    paramObj.force = force0;
    potFlag = true;
    displayData();
  }

  if(extDiff > POT_THRESHOLD){
    paramObj.ext = ext0;
    potFlag = true;
    displayData();
  }

  if(tempDiff > POT_THRESHOLD){
    paramObj.temp = temp0;
    potFlag = true;
    displayData();
  }
  if(digitalRead(CAL_BUTTON) == 1){
    pauseButton0 = true;
  }
  else{
    pauseButton0 = false;
  }

  normalizeParams();
}

double calibrate(){

  while(pauseButton0){
  }

  Serial.print("Calibrating: ");
  Serial.println(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE));

  double targCurr = 2 * GLOBAL_DIR;

  dxl.setGoalPosition(DXL_ID, 180 + dxl.getPresentPosition(DXL_ID, UNIT_DEGREE), UNIT_DEGREE);
  dxl.setGoalCurrent(DXL_ID, targCurr, UNIT_PERCENT);

  double accCurr = dxl.getPresentCurrent(DXL_ID, UNIT_PERCENT);

  double prevCurr = 10 + dxl.getPresentCurrent(DXL_ID, UNIT_PERCENT);
  double diff = abs(prevCurr - accCurr);

  while(diff > 0.05){
    prevCurr = accCurr;
    accCurr = dxl.getPresentCurrent(DXL_ID, UNIT_PERCENT);
    diff = abs(prevCurr - accCurr);
  }

  while(!pauseButton0){//dxl.getPresentVelocity(DXL_ID, UNIT_RPM) >= 0.5){
    accCurr = dxl.getPresentCurrent(DXL_ID, UNIT_PERCENT);
    dxl.setGoalPosition(DXL_ID, 180 + dxl.getPresentPosition(DXL_ID, UNIT_DEGREE), UNIT_DEGREE);
    Serial.println("Calibrating");
  }

  dxl.setGoalCurrent(DXL_ID, 0, UNIT_PERCENT);

  delay(2000);

  double result = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
  Serial.print("Calibrated: ");
  Serial.println(result);

  return result;
}
