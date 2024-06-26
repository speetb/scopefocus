#include <TMC2130Stepper.h>
#include <AccelStepper.h>
// #include <Servo.h>

#define MaxSpeed    3000   //Max speed in microsteps/s
#define MaxAcc      6000   //Max accelleration in microsteps/s/s
#define TurnDir     true   //Focuser rotation direction using true or false
#define RunCurrent  1000   //RMS current (in mA) when moving
#define HoldTime    5      //How long (in ms) to hold RunCurrent after a move
#define HoldCurrent 100    //RMS current (in mA) when still
#define Microsteps  16     //Can be one of these: 1, 2, 4, 8, 16, 32, 64, 128, 256.
#define TempAvgs    5      //Average this many temperature readings
#define Backlash    500    //How many microsteps to over-travel for single-direction focusing
#define StartPos    10000  //This defines the position in microsteps the focuser starts up at. Useful since no negative position values are allowed.

//Pin def for my prototype
//#define EnPin       6
//#define DirPin      5
//#define StepPin     4
//#define CsPin       7
//#define TempPin     A2
//#define RecieverPin 3
//#define AuxPin      0
//#define DiagPin     7

//Pin def for ScopeFocus v1.1
//#define EnPin       5
//#define DirPin      4
//#define StepPin     3
//#define CsPin       7
//#define TempPin     A0
//#define DiagPin #

//Pin def for ScopeFocus v1.2
#define EnPin       3
#define DirPin      2
#define StepPin     1
#define CsPin       7
#define TempPin     A0

TMC2130Stepper driver = TMC2130Stepper(EnPin, DirPin, StepPin, CsPin);
AccelStepper stepper = AccelStepper(stepper.DRIVER, StepPin, DirPin);

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

void setup() {
  Serial.begin(38400);
//  Serial.begin(9600);
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
  
  stepper.setMaxSpeed(Speed);
  stepper.setSpeed(Speed);
  stepper.setAcceleration(MaxAcc);
  stepper.setEnablePin(EnPin);
  stepper.setPinsInverted(TurnDir, false, true);
  stepper.enableOutputs();
  
  stepper.setCurrentPosition(StartPos);
  Pos = StartPos;
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

    // firmware value
    if (!strcasecmp(cmd, "GV")){
      Serial.print("12#");
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
    
    // get the temperature coefficient, hard-coded 'nothing'
    if (!strcasecmp(cmd, "GC")){
      Serial.print("02#");
    }
    
    // get the current motor speed, only values of 02, 04, 08, 10, 20
    if (!strcasecmp(cmd, "GD")){
      char tempString[4];
      sprintf(tempString, "%02X", int(04));
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
