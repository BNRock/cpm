// #define LED_BUILTIN 13

#include "header.h"

//U8X8_SH1106_128X64_NONAME_HW_I2C disp(U8X8_PIN_NONE); //use this line for the SH1106 1.3" OLED often sold as 1.3" SSD1306


volatile bool pauseButton = true;
volatile bool pauseButton0 = false;
volatile bool pauseButton1 = false;
volatile bool potFlag = true;

int res = 0;

char topStr[] = "     Running";//"00r/s   Running   00s";

unsigned long lastDebounce = 0;
unsigned long debounceDelay = 500;
unsigned long lastDebounce0 = 0;
unsigned long debounceDelay0 = 500;

//U8G2_SSD1306_128X64_ALT0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // SSD1306 and SSD1308Z are compatible

//U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);


U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SH1107_128X128_1_HW_I2C u8g2( U8G2_R3, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE); //[page buffer, size = 128 bytes]


//U8G2_SH1106_128X64_VCOMH0_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);		// same as the NONAME variant, but maximizes setContrast() range
//U8G2_SH1106_128X64_WINSTAR_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);		// same as the NONAME variant, but uses updated SH1106 init sequence
 
// U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);    //Low spped I2C

params paramObj;

//Motor Control
const uint8_t DXL_ID_0 = 1;
const uint8_t DXL_ID_1 = 2;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

double calibratedPos[] = {0, 0, 0};
#define POS_THRESHOLD 3
#define GLOBAL_DIR 1

#define CAL_BUTTON D8


void setup()
{

  Serial.begin(9600); // open the serial port at 9600 bps:

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
  dxl.ping(DXL_ID_0);
  dxl.ping(DXL_ID_1);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID_0);
  dxl.setOperatingMode(DXL_ID_0, OP_CURRENT_BASED_POSITION);
  dxl.torqueOn(DXL_ID_0);
  dxl.torqueOff(DXL_ID_1);
  dxl.setOperatingMode(DXL_ID_1, OP_CURRENT_BASED_POSITION);
  dxl.torqueOn(DXL_ID_1);

  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(BUTTON_PIN,INPUT_PULLDOWN);
  pinMode(CAL_BUTTON,INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), ISR_pauseInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(CAL_BUTTON), ISR_pauseInterrupt0, FALLING);
  pinMode(POT_0, INPUT);
  pinMode(POT_1, INPUT);
  pinMode(POT_2, INPUT);
  pinMode(POT_3, INPUT);

  displayData();
  delay(1000);
  // Turn on the LED on DYNAMIXEL
  //dxl.ledOn(DXL_ID_0);
  //dxl.ledOn(DXL_ID_1);
  // Turn off the LED on DYNAMIXEL
  calibratedPos[DXL_ID_0] = calibrate(DXL_ID_0);

  //TODO: Seperate each calibrated position for both motors
  calibratedPos[DXL_ID_1] = calibrate(DXL_ID_1);

  delay(500);

  mot = DXL_ID_0;

  // dxl.ledOff(DXL_ID_0);  
  // dxl.ledOff(DXL_ID_1);  

}

void loop()
{

  if(pauseButton){
    //running();
    delay(paramObj.dispTemp[calMot] * MAX_DELAY);
    //calibrate();
    flex(mot);
    delay(paramObj.dispTemp[calMot] * MAX_DELAY);
    while(extend(mot) == 0){};
    mot = opposite(mot);
  }

}

int opposite(int curr){
  if(curr == DXL_ID_0){
    return DXL_ID_1;
  }
  return DXL_ID_0;
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

void flex(int motor){
  double updatedPos[] = {0,0,0};
  updatedPos[DXL_ID_0] = calibratedPos[DXL_ID_0];
  updatedPos[DXL_ID_1] = calibratedPos[DXL_ID_1];

  double targPos = paramObj.dispExt[motor] * MAX_FLEX;
  double targCurr = paramObj.dispForce[motor] * MAX_CURR;
  //send the information to the motors
  dxl.setGoalCurrent(motor, targCurr * motorDir[motor], UNIT_PERCENT);
  dxl.setGoalPosition(motor, updatedPos[motor] + targPos, UNIT_DEGREE);
  // dxl.setGoalCurrent(DXL_ID_1, targCurr, UNIT_PERCENT);
  // dxl.setGoalPosition(DXL_ID_1, updatedPos[DXL_ID_1] + targPos, UNIT_DEGREE);

  //while(dxl.getPresentPosition(DXL_ID_0, UNIT_DEGREE) < updatedPos + targPos - POS_THRESHOLD){
    while(1){
      // Serial.println("flexing");
      // Serial.print("0: ");
      // Serial.print(dxl.getPresentPosition(DXL_ID_0, UNIT_DEGREE ));
      // Serial.print(" -> ");
      // Serial.println(calibratedPos[DXL_ID_0] + targPos - POS_THRESHOLD);
      // Serial.print("1: ");
      // Serial.print(dxl.getPresentPosition(DXL_ID_1, UNIT_DEGREE ));
      // Serial.print(" -> ");
      // Serial.println(calibratedPos[DXL_ID_1] + targPos - POS_THRESHOLD);
      targCurr = paramObj.dispForce[motor] * MAX_CURR;
      dxl.setGoalCurrent(motor, targCurr * motorDir[motor], UNIT_PERCENT);
      //dxl.setGoalCurrent(DXL_ID_1, targCurr, UNIT_PERCENT);
      targPos = paramObj.dispExt[motor] * 180;
      if(dxl.getPresentPosition(motor, UNIT_DEGREE) >= (updatedPos[motor] + targPos - POS_THRESHOLD)){
        break;
      }
      // if(dxl.getPresentPosition(DXL_ID_1, UNIT_DEGREE) >= (updatedPos[DXL_ID_1] + targPos - POT_THRESHOLD)){
      //   break;
      // }
      //Serial.print(dxl.getPresentPosition(DXL_ID_0, UNIT_DEGREE));
    }

  // Print present current
  dxl.setGoalCurrent(motor, 0);
  //dxl.setGoalCurrent(DXL_ID_1, 0);
}

int extend(int motor){
  double targCurr = paramObj.dispForce[motor] * MAX_CURR;

  dxl.setGoalCurrent(motor, -1 * targCurr * motorDir[motor], UNIT_PERCENT);
  dxl.setGoalPosition(motor, calibratedPos[DXL_ID_0], UNIT_DEGREE);
  // dxl.setGoalCurrent(DXL_ID_1, -1 * targCurr, UNIT_PERCENT);
  // dxl.setGoalPosition(DXL_ID_1, calibratedPos[DXL_ID_1], UNIT_DEGREE);

  //while(dxl.getPresentPosition(DXL_ID_0, UNIT_DEGREE ) > calibratedPos + POS_THRESHOLD){
  while(1){
    // Serial.println("extending");
    // Serial.print("0: ");
    // Serial.print(dxl.getPresentPosition(DXL_ID_0, UNIT_DEGREE ));
    // Serial.print(" -> ");
    // Serial.println(calibratedPos[DXL_ID_0]);
    // Serial.print("1: ");
    // Serial.print(dxl.getPresentPosition(DXL_ID_1, UNIT_DEGREE ));
    // Serial.print(" -> ");
    // Serial.println(calibratedPos[DXL_ID_1]);
    targCurr = paramObj.dispForce[motor] * MAX_CURR;
    dxl.setGoalCurrent(motor, -1 * targCurr * motorDir[motor], UNIT_PERCENT);
    //dxl.setGoalCurrent(DXL_ID_1, -1 * targCurr, UNIT_PERCENT);
    
    if(dxl.getPresentPosition(motor, UNIT_DEGREE ) <= (calibratedPos[motor]) + POT_THRESHOLD){
      break;
    }

    // if(dxl.getPresentPosition(DXL_ID_1, UNIT_DEGREE ) <= (calibratedPos[DXL_ID_1]) + POT_THRESHOLD){
    //   break;
    // }
  }

  dxl.setGoalCurrent(motor, 0);
  //dxl.setGoalCurrent(DXL_ID_1, 0);

  if(dxl.getPresentPosition(motor, UNIT_DEGREE ) <= (calibratedPos[motor]) + POT_THRESHOLD){
    return 1;
  }
  // if(dxl.getPresentPosition(DXL_ID_1, UNIT_DEGREE ) <= (calibratedPos[DXL_ID_1]) + POT_THRESHOLD){
  //   return 1;
  // }
  return 0;
}

float normalizeParams(){

  //normalize parameter based on input type
  paramObj.dispVel[calMot] = ((float)(fabs(paramObj.vel[calMot] - 1023)/1023));
  if(paramObj.dispVel[calMot] > 26){
    paramObj.dispVel[calMot] = 26;
  }
  paramObj.dispForce[calMot] = ((float)(fabs(paramObj.force[calMot] - 1023)/1023));
  if(paramObj.dispForce[calMot] > 26){
    paramObj.dispForce[calMot] = 26;
  }
  paramObj.dispExt[calMot] = ((float)(fabs(paramObj.ext[calMot] - 1023)/1023));
  if(paramObj.dispExt[calMot] > 26){
    paramObj.dispExt[calMot] = 26;
  }
  paramObj.dispTemp[calMot] = ((float)(fabs(paramObj.temp[calMot] - 1023)/1023));
  if(paramObj.dispTemp[calMot] > 26){
    paramObj.dispTemp[calMot] = 26;
  }

  return 0;

}

//THIS FUNCTION IS NO LONGER USED
// void runMotors(){

  
//   double updatedPos = calibrate();

//   double targPos = 270;
//   double targCurr = 5;
//   //send the information to the motors
//   dxl.setGoalCurrent(DXL_ID_0, targCurr, UNIT_PERCENT);
//   dxl.setGoalPosition(DXL_ID_0, updatedPos + targPos, UNIT_DEGREE);

//   while(dxl.getPresentPosition(DXL_ID_0, UNIT_DEGREE) < updatedPos + targPos - POS_THRESHOLD){
//     Serial.print(dxl.getPresentPosition(DXL_ID_0, UNIT_DEGREE));
//   }

//   // Print present current
//   dxl.setGoalCurrent(DXL_ID_0, 0);
//   delay(paramObj.dispTemp * 100);  

//   dxl.setGoalCurrent(DXL_ID_0, -1 * targCurr, UNIT_PERCENT);
//   dxl.setGoalPosition(DXL_ID_0, calibratedPos, UNIT_DEGREE);

//   while(dxl.getPresentPosition(DXL_ID_0, UNIT_DEGREE ) > calibratedPos + POS_THRESHOLD){
//     Serial.print(dxl.getPresentPosition(DXL_ID_0, UNIT_DEGREE));
//   }

//   dxl.setGoalCurrent(DXL_ID_0, 0);
//   delay(paramObj.dispTemp * 100);

// }

void displayData(){

  int offset = 0;

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    if(calFlag == 1){
      u8g2.drawStr(40,offset + 10,"Calibrating");
    }
    else{
      if(pauseButton && (calMot == 1)){
        u8g2.drawStr(40,offset + 10,"On - L");
      }
      else if(pauseButton && (calMot != 1)){
        u8g2.drawStr(40,offset + 10,"On - R");
      }
      else if(!pauseButton && (calMot == 1)){
        u8g2.drawStr(40,offset + 10,"Off - L");
      }
      else if(!pauseButton && (calMot != 1)){
        u8g2.drawStr(40,offset + 10,"Off - R");
      }
    }
    u8g2.drawFrame(5, offset + 18, 20, 30);
    u8g2.drawBox( 7, offset + 46 - int(paramObj.dispVel[calMot] * 26), 16, int(paramObj.dispVel[calMot] * 26));
    u8g2.drawFrame(37, offset + 18, 20, 30);
    u8g2.drawBox(39, offset + 46 - int(paramObj.dispForce[calMot] * 26), 16, int(paramObj.dispForce[calMot] * 26));
    u8g2.drawFrame(69, offset + 18, 20, 30);
    u8g2.drawBox(71, offset + 46 - int(paramObj.dispExt[calMot] * 26), 16, int(paramObj.dispExt[calMot] * 26));
    u8g2.drawFrame(100, offset + 18, 20, 30);
    u8g2.drawBox(102, offset + 46 - int(paramObj.dispTemp[calMot] * 26), 16, int(paramObj.dispTemp[calMot] * 26));
    u8g2.drawStr(0,offset + 60,"Speed");
    u8g2.drawStr(37,offset + 60,"Force");
    u8g2.drawStr(71,offset + 60,"Ext.");
    u8g2.drawStr(96,offset + 60,"Delay");
    char buff[10];
    u8g2.drawStr(100, offset + 10, itoa((int)(paramObj.dispTemp[calMot] * 7), buff, 10));
    u8g2.drawStr(115, offset + 10, "s");
    u8g2.drawStr(0, offset + 10, itoa((int)(paramObj.dispVel[calMot] * 5), buff, 10));
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

void ISR_pauseInterrupt0(){

  if(millis() - lastDebounce0 > debounceDelay0){
    lastDebounce0 = millis();
    //Serial.print("interrupt - ");

    if(pauseButton1)
    {
      pauseButton1 = !pauseButton1;
      calMot = opposite(calMot);
      Serial.print("Left!\n");
    }
    else
    {
      pauseButton1 = !pauseButton1;
      calMot = opposite(calMot);
      Serial.print("Right\n");
    }
  }
  
}

void ISR_timer(){

  if(++counter == 500){
    displayData();
    counter = 0;
  }

  int vel0 = analogRead(POT_0);
  int force0 = analogRead(POT_1);
  int ext0 = analogRead(POT_2);
  int temp0 = analogRead(POT_3);

  int velDiff = abs(paramObj.vel[calMot] - vel0);
  int forceDiff = abs(paramObj.force[calMot] - force0);
  int extDiff = abs(paramObj.ext[calMot] - ext0);
  int tempDiff = abs(paramObj.temp[calMot] - temp0);

  if(velDiff > POT_THRESHOLD){
    paramObj.vel[calMot] = vel0;
    potFlag = true;
  }

  if(forceDiff > POT_THRESHOLD){
    paramObj.force[calMot] = force0;
    potFlag = true;
  }

  if(extDiff > POT_THRESHOLD){
    paramObj.ext[calMot] = ext0;
    potFlag = true;
  }

  if(tempDiff > POT_THRESHOLD){
    paramObj.temp[calMot] = temp0;
    potFlag = true;
  }

  if(digitalRead(CAL_BUTTON) == 1){
    //calMot = opposite(calMot);
    pauseButton0 = true;
  }

  else{
    pauseButton0 = false;
  }

  normalizeParams();
}

double calibrate(int motor){

  //motor defines which motor is to be calibrated

  dxl.ledOn(motor);

  while(pauseButton0){
  }

  calFlag = 1;

  Serial.print("Calibrating: ");
  Serial.println(dxl.getPresentPosition(motor, UNIT_DEGREE));

  double targCurr = 2 * motorDir[motor];

  dxl.setGoalPosition(motor, 180 + dxl.getPresentPosition(motor, UNIT_DEGREE), UNIT_DEGREE);
  dxl.setGoalCurrent(motor, targCurr, UNIT_PERCENT);

  double accCurr = dxl.getPresentCurrent(motor, UNIT_PERCENT);

  double prevCurr = 10 + dxl.getPresentCurrent(motor, UNIT_PERCENT);
  double diff = abs(prevCurr - accCurr);

  while(diff > 0.05){
    prevCurr = accCurr;
    accCurr = dxl.getPresentCurrent(motor, UNIT_PERCENT);
    diff = abs(prevCurr - accCurr);
  }

  while(!pauseButton0){//dxl.getPresentVelocity(DXL_ID_0, UNIT_RPM) >= 0.5){
    accCurr = dxl.getPresentCurrent(motor, UNIT_PERCENT);
    dxl.setGoalPosition(motor, 180 + dxl.getPresentPosition(motor, UNIT_DEGREE), UNIT_DEGREE);
    Serial.println("Calibrating");
  }

  dxl.setGoalCurrent(motor, 0, UNIT_PERCENT);

  delay(2000);

  double result = dxl.getPresentPosition(motor, UNIT_DEGREE);
  Serial.print("Calibrated: ");
  Serial.println(result);

  dxl.ledOff(motor);

  calFlag = 0;

  return result;
}
