/*
##############################################################
# File: pidJune16\pidJune16.pde                              #
# Written by Nathan Murrow, Jonah Brooks, and Keith Smee for #
# use with Linn-Benton Community College's ROV Team's        #
# "Phoenix" in the MATE 2011 international ROV competition   #
# at the NASA Johnson Space Center in Houston, Texas from    #
# June 16 - 18, 2011. The LBCC ROV Team is free to use and   #
# modify this code for use in any future activities related  #
# to ROVs.                                                   #
#       June 14, 2011                                        #
##############################################################
# Microcontroller code: This code is compiled and uploaded   #
# to the Arduino Mega 2560 on the ROV. It establishes a      #
# server on the microcontroller which will receive commands  #
# from the Python client on the laptop. This code can check  #
# the pressure sensor, run the h-bridges, run solenoids, and #
# has an option for control using a PID feedback loop.       #
##############################################################
*/

//header files for TCP/IP
#include <SPI.h>
#include <Ethernet.h>

//network info
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[] = {192,168,0,177};
byte gateway[] = {192,168,0,1};
byte subnet[] = {255,255,255,0};

Server server(23);
Client client = 0;

//pin assignments for h-bridges, solenoids, and sensors
const uint8_t LEFT_MOTOR_BIN1 = 30;
const uint8_t LEFT_MOTOR_BIN2 = 28;
const uint8_t LEFT_MOTOR_PWM  = 2;
//const uint8_t LEFT_MOTOR_FAULT= A0;

const uint8_t RIGHT_MOTOR_BIN1 = 26;
const uint8_t RIGHT_MOTOR_BIN2 = 24;
const uint8_t RIGHT_MOTOR_PWM  = 3;
//const uint8_t RIGHT_MOTOR_FAULT= A1;

const uint8_t FRONT1_MOTOR_BIN1 = 34;
const uint8_t FRONT1_MOTOR_BIN2 = 32;
const uint8_t FRONT1_MOTOR_PWM  = 5;
//const uint8_t FRONT_MOTOR_FAULT= A2;

//aft starboard
const uint8_t REAR1_MOTOR_BIN1 = 38;
const uint8_t REAR1_MOTOR_BIN2 = 36;
const uint8_t REAR1_MOTOR_PWM  = 8;
//const uint8_t REAR_MOTOR_FAULT= A3;

const uint8_t FRONT2_MOTOR_BIN1 = 42;
const uint8_t FRONT2_MOTOR_BIN2 = 40;
const uint8_t FRONT2_MOTOR_PWM  = 7;
//const uint8_t FRONT_MOTOR_FAULT= A2;

//aft port
const uint8_t REAR2_MOTOR_BIN1 = 46;
const uint8_t REAR2_MOTOR_BIN2 = 44;
const uint8_t REAR2_MOTOR_PWM  = 6;
//const uint8_t REAR_MOTOR_FAULT= A3;

//*************************************
//SOLENOID PINS
const uint8_t SOLENOID_PIN[] = {23, 25, 27, 29, 31,
                                33, 35, 37, 39, 41};
//*************************************

const uint8_t PRESSURE_SENSOR_PIN = A2;

const uint8_t STANDBY_PIN = 38;

//universal limiter for thruster PWM 
int motor_maxPWM = 234; 

//communications data variables
const int INSTRUCTION_LENGTH = 11;
const int SENSOR_DATA_LENGTH = 2;

//PID variables
float KP = 0;
float KI = 0;
float KD = 0;

int iMax = 1000;
int iMin = -1000;

int lastError = 0;
int sumError = 0;

int pidTarget = 0;
boolean pidOn = false;
boolean hoverOn = true;
boolean yawOn = false;

int pid_maxPWM = 191;
//end PID variables

//runs once on power-up
void setup()
{
  //set all unused pins to HIGH (Justin's suggestion)
  for (int x = 0; x < 53; x++)
  {
    pinMode(x, OUTPUT);
    digitalWrite(x, HIGH);
  }
  
  //pinMode(A0, OUTPUT);
  //pinMode(A1, OUTPUT);
  //pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);
  pinMode(A8, OUTPUT);
  pinMode(A9, OUTPUT);
  pinMode(A10, OUTPUT);
  pinMode(A11, OUTPUT);
  pinMode(A12, OUTPUT);
  pinMode(A13, OUTPUT);
  pinMode(A14, OUTPUT);
  pinMode(A15, OUTPUT);
  //analogWrite(A0, 255);
  //analogWrite(A1, 255);
  //analogWrite(A2, 255);
  analogWrite(A3, 255);
  analogWrite(A4, 255);
  analogWrite(A5, 255);
  analogWrite(A6, 255);
  analogWrite(A7, 255);
  analogWrite(A8, 255);
  analogWrite(A9, 255);
  analogWrite(A10, 255);
  analogWrite(A11, 255);
  analogWrite(A12, 255);
  analogWrite(A13, 255);
  analogWrite(A14, 255);
  analogWrite(A15, 255);
  
  //set pins to default values
  pinMode(LEFT_MOTOR_BIN1,OUTPUT);
  pinMode(LEFT_MOTOR_BIN2,OUTPUT);
  digitalWrite(LEFT_MOTOR_BIN2, LOW);
  pinMode(LEFT_MOTOR_PWM,OUTPUT);
  analogWrite(LEFT_MOTOR_PWM, 0);
  //pinMode(LEFT_MOTOR_FAULT,INPUT);
  
  pinMode(RIGHT_MOTOR_BIN1,OUTPUT);
  pinMode(RIGHT_MOTOR_BIN2,OUTPUT);
  digitalWrite(RIGHT_MOTOR_BIN2, LOW);
  pinMode(RIGHT_MOTOR_PWM,OUTPUT);
  analogWrite(RIGHT_MOTOR_PWM, 0);
  //pinMode(RIGHT_MOTOR_FAULT,INPUT);
  
  pinMode(FRONT1_MOTOR_BIN1,OUTPUT);
  pinMode(FRONT1_MOTOR_BIN2,OUTPUT);
  digitalWrite(FRONT1_MOTOR_BIN2, LOW);
  pinMode(FRONT1_MOTOR_PWM,OUTPUT);
  analogWrite(FRONT1_MOTOR_PWM, 0);
  //pinMode(FRONT_MOTOR_FAULT,INPUT);
  
  pinMode(REAR1_MOTOR_BIN1,OUTPUT);
  pinMode(REAR1_MOTOR_BIN2,OUTPUT);
  digitalWrite(REAR1_MOTOR_BIN2, LOW);
  pinMode(REAR1_MOTOR_PWM,OUTPUT);
  analogWrite(REAR1_MOTOR_PWM, 0);
  //pinMode(REAR_MOTOR_FAULT,INPUT);
  
  pinMode(FRONT2_MOTOR_BIN1,OUTPUT);
  pinMode(FRONT2_MOTOR_BIN2,OUTPUT);
  digitalWrite(FRONT2_MOTOR_BIN2, LOW);
  pinMode(FRONT2_MOTOR_PWM,OUTPUT);
  analogWrite(FRONT2_MOTOR_PWM, 0);
  //pinMode(FRONT_MOTOR_FAULT,INPUT);
  
  pinMode(REAR2_MOTOR_BIN1,OUTPUT);
  pinMode(REAR2_MOTOR_BIN2,OUTPUT);
  digitalWrite(REAR2_MOTOR_BIN2, LOW);
  pinMode(REAR2_MOTOR_PWM,OUTPUT);
  analogWrite(REAR2_MOTOR_PWM, 0);
  //pinMode(REAR_MOTOR_FAULT,INPUT);

  /*for (int i = 0; i < 10; i++)
  {
    pinMode(SOLENOID_PIN[i], OUTPUT);
    digitalWrite(SOLENOID_PIN[i], LOW);
  }*/

  pinMode(PRESSURE_SENSOR_PIN,INPUT);
  
  TCCR3B |= _BV(WGM32);  // Force pins 2, 3, and 5 to Fast PWM mode
  TCCR4B |= _BV(WGM42);  // Force pins 6, 7, and 8 to Fast PWM mode
  
  pinMode(STANDBY_PIN, OUTPUT);
  digitalWrite(STANDBY_PIN, HIGH); // enable the H-Bridges

  //open up network communications
  Ethernet.begin(mac,ip,gateway,subnet);
  Serial.begin(9800);
  server.begin();
  client = server.available();
  
  //the reference voltage for the pressure sensor is 5V
  analogReference(DEFAULT); 
  Serial.println("Setup complete.");
}

//runs continuously after setup()
void loop()
{ 
  Communications();
}

//-----------------------------------------------------------------------------------------------                                                            //Communications Functions                                                                
//-----------------------------------------------------------------------------------------------
//****************************************************
// Handles all communications between systems        *
// This will almost certainly need to be expanded    *
// into a set of functions, possibly a class         *
// This is a working prototype of the basic functions*
//****************************************************
void Communications()
{    
     //this flag allows the microcontroller to reconnect automatically
     static boolean newConnectionFlag = true;
     
     //initialize data arrays
     byte instructionBytes[INSTRUCTION_LENGTH] = {0,0,0,0,0,0,0,0,0,0,0};
     byte sensorData[SENSOR_DATA_LENGTH] = {0,0};
     
     //the client is the laptop running Python
     client = server.available(); 
     if(client.connected())
     {  
       //if the connection is made, un-set the connection flag
       if(newConnectionFlag)
        {  
           Serial.println("Connected to client");
           newConnectionFlag = false;
        }
        //if the microcontroller receives data, it will send data
        if(client.available() > 0)
        {  
           //receive data
           for(int i = 0; i < INSTRUCTION_LENGTH; i++)
           {  
             instructionBytes[i] = client.read();
           }
           //send sensor data
           GetSensorData(sensorData);
           client.write(sensorData, SENSOR_DATA_LENGTH);      
           // Replace with sending sensor data instead of repeating
           //client.write(instructionBytes, INSTRUCTION_LENGTH);
           //execute received data
           DecodeInstruction(instructionBytes);
        }
     }
     //not sure why this line is here, I'm leaving it in because it seems harmless but
     //I'm wary of deleting it at this late stage.
     GetSensorData(sensorData);
}

//*******************************************************************
// Receives a byte array of length INSTRUCTION_LENGTH               *
// First byte of array is OpCode, remaining bytes are operands      *
// Returns nothing, but calls sub-functions based on instruction    *
//*******************************************************************
void DecodeInstruction(byte* instructionArg)
{    byte OpCode = instructionArg[0];
     
     switch (OpCode)
     {  case 'M':
            MotorControl(instructionArg);
            break;
        case 'S':
            SolenoidCommand(instructionArg);
            break;
        case 'P':
            PIDCommand(instructionArg);
            break;
        default:
            Serial.print("Invalid OpCode. Received:");
            Serial.println(OpCode);
            Serial.print(instructionArg[1]);
            Serial.print(instructionArg[2]);
            Serial.print(instructionArg[3]);
            Serial.print(instructionArg[4]);
            Serial.print(instructionArg[5]);
            Serial.print(instructionArg[6]);
            Serial.print(instructionArg[7]);
            Serial.print(instructionArg[8]);
            Serial.print(instructionArg[9]);
            Serial.println(instructionArg[10]);
     }
}

//-------------------------------------------------------------------------------
//Command Functions
//---------------------------------------------------------------------------------


//----------------------------------
// Fills the argument array with the
// data from each sensor pin on the board
//-----------------------------------
void GetSensorData(byte* sensorDataArg)
{  // Read the sensor pins and fill the array here
   int first = 0, second = 0;
   int temp = analogRead(PRESSURE_SENSOR_PIN);
   //because the sensor data is 10 bits, it needs to be split into two bytes
   if (temp > 255)
   {
     first = temp / 256;
     second = temp % 256;
   }
   else
     second = temp;
   sensorDataArg[0] = first;
   sensorDataArg[1] = second;
   //return sensorDataArg;
}

void PIDCommand(byte* PIDVals)
{
  //check that PID is on at the laptop side
  if (int(PIDVals[1]) != 0)
  {
    //turn PID on at the microcontroller side
    if (pidOn == false)
    {
      pidBegin();
    }
  }
  else
  {
    pidEnd();
  }
  
  if (pidOn == true)
  {
    //check for hover mode
    if (int(PIDVals[1]) == 2)
    {
      pidBegin();
      hoverOn = true;
    }
    //check for depth-finding mode
    else if (int(PIDVals[1]) == 3)
    {
      //the desired depth is 10 bits long
      pidBegin(int(PIDVals[2]*256 + PIDVals[3]));
      //turn off hover when finding a depth
      hoverOn = false;
    }
    //set the PID constants to received data
    KP = (float)((int)PIDVals[4]) / 10.0;
    KI = (float)((int)PIDVals[5]) / 1000.0;
    KD = (float)((int)PIDVals[6]) / 10.0;
  }
}

//solenoid switching code (obsolete) 
void SolenoidCommand(byte* solVals)
{
   for(int i = 0; i < 10; i++)
   {
     if (solVals[i+1] == 1)
     {
       digitalWrite(SOLENOID_PIN[i], HIGH);
     }
     else
     {
       digitalWrite(SOLENOID_PIN[i], LOW); 
     }
   }       
}

//----------------------------------
// Accepts the full instruction array
// Array holds: OpCode, LeftMotorSpeed,
// RightMotorSpeed, FrontMotorSpeed,
// RearMotorSpeed, Direction bit flags
//-----------------------------------
void MotorControl(byte* motorSpeeds)
{   
    //store the received command values for PWM
    for(int i = 0; i < 4; i++)
    {   if(motorSpeeds[i+1] < 0)
        {  motorSpeeds[i+1] = abs(motorSpeeds[i+1]); } 
        if(motorSpeeds[i+1] >= motor_maxPWM)
        {  motorSpeeds[i+1] = motor_maxPWM; }
    }
    
    //decode the direction byte of the received motor command
    boolean D[5] = {0,0,0,0,0};
    D[0] = 0; // Not used, OpCode has no speed
    D[1] = (motorSpeeds[5] >> 3) & 1;
    D[2] = (motorSpeeds[5] >> 2) & 1;
    D[3] = (motorSpeeds[5] >> 1) & 1;
    D[4] = (motorSpeeds[5]) & 1;
    
    yawOn = (motorSpeeds[5] >> 4) & 1;
    
    //the horizontal thrusters are controlled independently of the vertical thrusters
    digitalWrite(LEFT_MOTOR_BIN1, D[1]);
    digitalWrite(LEFT_MOTOR_BIN2, !D[1]);
    analogWrite(LEFT_MOTOR_PWM, motorSpeeds[1]);
    
    digitalWrite(RIGHT_MOTOR_BIN1, D[2]);
    digitalWrite(RIGHT_MOTOR_BIN2, !D[2]);
    analogWrite(RIGHT_MOTOR_PWM, motorSpeeds[2]);
    
    /*
    MotorControlHoriz(D[1],D[2], motorSpeeds[1], motorSpeeds[2]);
    */
    
    //if PID is on and there are no vertical thrust commands, use PID
    //otherwise, use the vertical thrust command
    if (pidOn == true && motorSpeeds[3] == 0 && motorSpeeds[4] == 0)
      pidControl();
    else
      MotorControlVert(D[3],D[4],motorSpeeds[3],motorSpeeds[4]);
}

//two thrusters becomes four (the initial design only had two vertical thrusters)
void MotorControlVert(boolean mydir, int myspd)
{
    MotorControlVert(mydir, mydir, myspd, myspd);
}

//controls the vertical thrusters
void MotorControlVert(boolean frontdr, boolean reardr, int frontspd, int rearspd)
{
    //There was a reason for doing this (I think for testing the new thrusters)
    //clearly, it's obsolete now but could be useful in the future anyway
    frontspd = (frontspd * 100) / 100;
    rearspd = (rearspd * 100) / 100;
    
    if (frontspd > motor_maxPWM) frontspd = motor_maxPWM;
    if (rearspd > motor_maxPWM) rearspd = motor_maxPWM;
    
    //Serial.println("MCV");
    if (yawOn == false)
    {
      digitalWrite(FRONT1_MOTOR_BIN1, frontdr);
      digitalWrite(FRONT1_MOTOR_BIN2, !frontdr);
      analogWrite(FRONT1_MOTOR_PWM, frontspd);
    
      digitalWrite(REAR1_MOTOR_BIN1, reardr);
      digitalWrite(REAR1_MOTOR_BIN2, !reardr);
      analogWrite(REAR1_MOTOR_PWM, rearspd);
    
      digitalWrite(FRONT2_MOTOR_BIN1, frontdr);
      digitalWrite(FRONT2_MOTOR_BIN2, !frontdr);
      analogWrite(FRONT2_MOTOR_PWM, frontspd);
    
      digitalWrite(REAR2_MOTOR_BIN1, reardr);
      digitalWrite(REAR2_MOTOR_BIN2, !reardr);
      analogWrite(REAR2_MOTOR_PWM, rearspd);
      
      /*
      Serial.print("F1: ");
      if (frontdr) Serial.print("-");
      Serial.print(frontspd);
      Serial.print(" F2: ");
      if (frontdr) Serial.print("-");
      Serial.println(frontspd);
      Serial.print("R1: ");
      if (reardr) Serial.print("-");
      Serial.print(rearspd);
      Serial.print(" R2: ");
      if (reardr) Serial.print("-");
      Serial.println(rearspd);
      Serial.println();
      */
    }
    else if (yawOn == true)
    {
      digitalWrite(FRONT1_MOTOR_BIN1, frontdr);
      digitalWrite(FRONT1_MOTOR_BIN2, !frontdr);
      analogWrite(FRONT1_MOTOR_PWM, frontspd);
    
      digitalWrite(REAR1_MOTOR_BIN1, frontdr);
      digitalWrite(REAR1_MOTOR_BIN2, !frontdr);
      analogWrite(REAR1_MOTOR_PWM, rearspd);
    
      digitalWrite(FRONT2_MOTOR_BIN1, reardr);
      digitalWrite(FRONT2_MOTOR_BIN2, !reardr);
      analogWrite(FRONT2_MOTOR_PWM, frontspd);
    
      digitalWrite(REAR2_MOTOR_BIN1, reardr);
      digitalWrite(REAR2_MOTOR_BIN2, !reardr);
      analogWrite(REAR2_MOTOR_PWM, rearspd);
      
      /*
      Serial.print("F1: ");
      if (frontdr) Serial.print("-");
      Serial.print(frontspd);
      Serial.print(" F2: ");
      if (reardr) Serial.print("-");
      Serial.println(rearspd);
      Serial.print("R1: ");
      if (frontdr) Serial.print("-");
      Serial.print(frontspd);
      Serial.print(" R2: ");
      if (reardr) Serial.print("-");
      Serial.println(rearspd);
      Serial.println();
      */
    }
}

//the following method was used to experiment with phase-lock
//control of the h-bridges. It's not used now, turned out to be unnecessary.
void MotorControlHoriz(boolean leftdr, boolean rightdr, int leftspd, int rightspd)
{
    
    digitalWrite(LEFT_MOTOR_BIN1, leftdr);
    digitalWrite(LEFT_MOTOR_BIN2, !leftdr);
    analogWrite(LEFT_MOTOR_PWM, leftspd);
    
    digitalWrite(RIGHT_MOTOR_BIN1, rightdr);
    digitalWrite(RIGHT_MOTOR_BIN2, !rightdr);
    analogWrite(RIGHT_MOTOR_PWM, rightspd);
    
    /*
    
    int ratioleft = (leftspd * 100) / 200;
    int ratioright = (rightspd * 100) / 200;
    
    if (leftspd > motor_maxPWM / 2) leftspd = motor_maxPWM / 2;
    if (rightspd > motor_maxPWM / 2) rightspd = motor_maxPWM / 2;
    
    if (leftdr)
      leftspd = 127 + ratioleft;
    else 
      leftspd = 127 - ratioleft;
      
    if (rightdr)
      rightspd = 127 + ratioright;
    else 
      rightspd = 127 - ratioright;
    
    analogWrite(LEFT_MOTOR_BIN1, leftspd);
    analogWrite(RIGHT_MOTOR_BIN1, rightspd);
    
    
    digitalWrite(LEFT_MOTOR_PWM, HIGH);
    digitalWrite(RIGHT_MOTOR_PWM, HIGH);
    
    //analogWrite(LEFT_MOTOR_PWM, motor_maxPWM);
    //analogWrite(RIGHT_MOTOR_PWM, motor_maxPWM);
    
    */
}

//--------------------------------------------------------------------------------
//PID Functions
//--------------------------------------------------------------------------------

void setConstants(int myP, int myI, int myD)
{
  KP = myP;
  KI = myI;
  KD = myD;
}

int getFeedback()
{
  return analogRead(PRESSURE_SENSOR_PIN);
}

//establish a new target depth (not fully tested but it should work)
//sticking to "hover" mode in critical situations is advisable
int setPIDTarget(int value)
{
  pidTarget = value;
  lastError = 0;
  sumError = 0; 
}

//turns PID on with the current depth as the target depth
void pidBegin()
{
  pidBegin(getFeedback());
}

//this is the main PID turn-on method
void pidBegin(int targetpressure)
{
  Serial.println(targetpressure);
  setPIDTarget(targetpressure);
  pidOn = true;
}

void pidEnd()
{
  pidOn = false;
}

//more target-changing stuff, also untested
void pidControl(int targetpressure)
{
  pidBegin(targetpressure);
  pidControl();
}

void pidControl()
{
   //acquire current depth
   int feedback = getFeedback();
   
   //determines proportional error
   int error = feedback - pidTarget;
   
   //determines integral error
   //this is a running total of all error
   //note that negative error is subtracted from positive error
   sumError += error;
   
   //caps the possible integral error
   if (sumError > iMax)
     sumError = iMax;
   if (sumError < iMin)
     sumError = iMin;
   
   //calculates duty cycle
   int pidDuty = KP*error + KD*(error - lastError) + KI*(sumError);
   
   //lastError is used for the differential term
   //(the change in error)
   lastError = error;
   
   //determines direction of the motors
   //it should be the correct direction but can be reversed if necessary
   boolean pidDirection;
   if (pidDuty < 0)
     pidDirection = true;
   else
     pidDirection = false;
     
   //makes sure duty cycle is positive
   pidDuty = abs(pidDuty);
     
   //caps the duty cycle
   //the ethernet shield will cut out if we run vertical thrusters at full power
   //PID must be limited here because it cannot be controlled on the Python side
   if (pidDuty > pid_maxPWM)
     pidDuty = pid_maxPWM;
   
   //control the vertical thrusters with the PID-calculated values
   MotorControlVert(pidDirection, pidDuty);
}
