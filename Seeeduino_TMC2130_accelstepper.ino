#include <TMC2130Stepper.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <IRremote.h>

#define MaxSpeed    2000  //Max speed in steps/s
#define MaxAcc      5000  //Max accelleration in steps/s/s
#define TurnDir     true  //Focuser rotation direction using true or false
#define RunCurrent  1200  //RMS current (in Ampere) when moving
#define HoldTime    2     //How long (in miliseconds) to hold RunCurrent after a move
#define HoldCurrent 120   //RMS current (in Ampere) when still
#define Microsteps  8     //Can be one of these: 1, 2, 4, 8, 16, 32, 64, 128, 256.
#define TempAvgs    5     //Average this many temperature readings
#define Backlash    100   //How many steps to over-travel for single-direction focusing
#define StartPos    10000 //This defines the position the focuser starts up at. Useful since no negative position values are allowed.

//Pin def for my prototype
#define EnPin       6
#define DirPin      5
#define StepPin     4
#define CsPin       7
#define TempPin     A2
#define RecieverPin 3
#define AuxPin      0
//#define DiagPin     7

//Pin def for ScopeFocus v1.1 - TODO: confirm
//#define EnPin       6
//#define DirPin      5
//#define StepPin     4
//#define CsPin       8
//#define TempPin     A1
//#define RecieverPin 3
//#define ServoPin    2

TMC2130Stepper driver = TMC2130Stepper(EnPin, DirPin, StepPin, CsPin);
AccelStepper stepper = AccelStepper(stepper.DRIVER, StepPin, DirPin);
//Servo servo;
IRrecv irrecv(RecieverPin);
decode_results results;

char      inChar;
char      cmd[8];
char      param[8];
char      line[8];
int32_t   Pos = StartPos;
int16_t   Temperature = 40;
uint8_t   Dir = 0;
bool      isRunning = false;
int16_t   Speed = MaxSpeed;
uint8_t   eoc = 0;
uint8_t   idx = 0;
int16_t   TempSum = 0;
uint32_t  millisLastMove = 0;
float     MeasVolt = 0;
uint32_t  key_value = 0;

void setup() {
  Serial.begin(38400);
  SPI.begin();

  pinMode(CsPin, OUTPUT);
  digitalWrite(CsPin, HIGH);
  analogReadResolution(12);
  
  driver.begin();
  driver.rms_current(RunCurrent);
  driver.microsteps(Microsteps);
  driver.stealthChop(1);
  driver.stealth_autoscale(1);
  driver.interpolate(1);
  
//  driver.off_time(2);
//  driver.blank_time(24);
//  driver.hysterisis_start(0);
//  driver.hysterisis_end(13);
  
  stepper.setMaxSpeed(Speed);
  stepper.setSpeed(Speed);
  stepper.setAcceleration(MaxAcc);
  stepper.setEnablePin(EnPin);
  stepper.setPinsInverted(true, false, true);
  stepper.enableOutputs();

//  servo.attach(AuxPin);
  irrecv.enableIRIn();
  stepper.setCurrentPosition(StartPos);
  pos = StartPos;
  memset(line, 0, 8);
  millisLastMove = millis();
}


void loop(){
  // run the stepper if there's no pending command and if there are pending movements
  if (!Serial.available()){
    if (isRunning){
      driver.rms_current(RunCurrent);
      stepper.run();
      millisLastMove = millis();
    } 
    else{
      if ((millis() - millisLastMove) > HoldTime){
        driver.rms_current(HoldCurrent);
      }
    }
    if (stepper.distanceToGo() == 0 && Dir == 0){
      stepper.run();
      isRunning = false;
    }
    if (stepper.distanceToGo() == 0 && Dir == 1){
      Dir = 0;
      Pos = stepper.currentPosition() - Backlash;
      stepper.moveTo(Pos);
      stepper.run();
      isRunning = true;
    }
  }
  else {
    // read the command until the terminating # character
    while (Serial.available() && !eoc){
      inChar = Serial.read();
      if (inChar != '#' && inChar != ':'){
        line[idx++] = inChar;
        if (idx >= 8){
          idx = 8 - 1;
        }
      }
      else{
        if (inChar == '#'){
          eoc = 1;
        }
      }
    }
  }

  if (irrecv.decode(&results)){
        if (results.value == 0XFFFFFFFF)
          results.value = key_value;
        switch (results.value){
          case 0xFFA25D:
          Speed -= 100;
            if (Speed <= 0){
            Speed = 100;
            }
          delay(100); //TODO non-blocking
          stepper.setMaxSpeed(Speed);
          break;
          
          case 0xFFE21D:
          Speed += 100;
            if (Speed > MaxSpeed){
            Speed = MaxSpeed;
            }
          delay(100); //TODO non-blocking
          stepper.setMaxSpeed(Speed);
          isRunning = true;
          break;

          case 0xFF22DD:
          Pos -= 20;
            if (Pos < 0){
            Pos = 0;
            }
          //delay(100); //TODO non-blocking
          stepper.moveTo(Pos);
          isRunning = true;
          break;

          case 0xFFC23D:
          Pos += 20;
            if (Pos > 100000){
            Pos = 100000;
            }
          //delay(100); //TODO non-blocking
          stepper.moveTo(Pos);
          isRunning = true;
          break;

          case 0xFF9867:
          Pos = 10000;
          stepper.moveTo(Pos);
          isRunning = true;
          break;

//          case 0xFF6897:
//          servo.write(0);
//          break;
//
//          case 0xFFB04F:
//          servo.write(180);
//          break;

        }
        key_value = results.value;
        irrecv.resume();
  }
  
  // process the command we got
  if (eoc) {
    memset(cmd, 0, 8);
    memset(param, 0, 8);
    
    uint8_t len = strlen(line);
    if (len >= 2){
      strncpy(cmd, line, 2);
    }
    if (len > 2){
      strncpy(param, line + 2, len - 2);
    }
    
    memset(line, 0, 8);
    eoc = 0;
    idx = 0;

    // the stand-alone program sends :C# :GB# on startup
    // :C# is a temperature conversion, doesn't require any response

    // LED backlight value, always return "00"
    if (!strcasecmp(cmd, "GB")){
      Serial.print("00#");
    }
    
    // stop a move
    if (!strcasecmp(cmd, "FQ")){
      stepper.moveTo(stepper.currentPosition());
      stepper.run();
      isRunning = true;
    }
    
    // set new motor position
    if (!strcasecmp(cmd, "SN")){
      Pos = hexstr2long(param);
      if (Pos <= stepper.currentPosition()){
        Dir = 0;
      }
      if (Pos > stepper.currentPosition()){
        Dir = 1;
        Pos = Pos + Backlash;
      }
      stepper.moveTo(Pos);
    }
    
    // initiate a move
    if (!strcasecmp(cmd, "FG")){
      driver.rms_current(RunCurrent);
      isRunning = true;
    }

    // home the motor, hard-coded, ignore parameters since we only have one motor
    if (!strcasecmp(cmd, "PH")){
      stepper.setCurrentPosition(10000);
      isRunning = true;
    }

    // firmware value, always return "10"
    if (!strcasecmp(cmd, "GV")){
      Serial.print("01#");
    }

    // get the current motor position
    if (!strcasecmp(cmd, "GP")){
      Pos = stepper.currentPosition();
      char tempString[6];
      sprintf(tempString, "%04X", Pos);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the new motor position (target)
    if (!strcasecmp(cmd, "GN")){
      Pos = stepper.targetPosition();
      char tempString[6];
      sprintf(tempString, "%04X", Pos);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the current temperature
    if (!strcasecmp(cmd, "GT")){
      if(!isRunning){
        TempSum = 0;
        for(uint8_t i=0; i < TempAvgs; i++){
          TempSum += analogRead(TempPin);
          delayMicroseconds(200); //TODO non-blocking
        }
        MeasVolt = 3.3 * TempSum / (4095 * TempAvgs);
        Temperature = float((MeasVolt - 0.5) * 200);
        if(Temperature < 0){
          Temperature += 65536;
        }
      }
      char tempString[6];
      sprintf(tempString, "%04X", Temperature);
      Serial.print(tempString);
      Serial.print("#");
    }
    
    // get the temperature coefficient, hard-coded
    if (!strcasecmp(cmd, "GC")){
      Serial.print("01#");
    }
    
    // get the current motor speed, only values of 02, 04, 08, 10, 20
    if (!strcasecmp(cmd, "GD")){
      char tempString[4];
      sprintf(tempString, "%02X", 02);
      Serial.print(tempString);
      Serial.print("#");
    }
    
    // set speed, only acceptable values are 02, 04, 08, 10, 20
    if (!strcasecmp(cmd, "SD")){
      //Speed = hexstr2long(param);
      //stepper.setSpeed(Speed);
      //stepper.setMaxSpeed(Speed);
    }
    
    // whether half-step is enabled or not, always return "00"
    if (!strcasecmp(cmd, "GH")){
      Serial.print("00#");
    }
    
    // motor is moving - 01 if moving, 00 otherwise
    if (!strcasecmp(cmd, "GI")){
      if (isRunning){
        Serial.print("01#");
      } 
      else {
        Serial.print("00#");
      }
    }
    
    // set current motor position
    if (!strcasecmp(cmd, "SP")){
      Pos = hexstr2long(param);
      stepper.setCurrentPosition(Pos);
    }
  }
}

long hexstr2long(char *line){
  long ret = 0;
  ret = strtol(line, NULL, 16);
  return (ret);
}
