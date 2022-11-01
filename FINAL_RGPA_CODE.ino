/* This code was written by Luis Vazquez as part of a Robotic Guitar Playing Attachment project
The code is written and uploaded to an Arduino Mega 2560 which when executed it runs 6 DC motors to rotate accordingly until a signal that their corresponding
buttons in the system have been pressed. Then the DC motors will rotate in order to position the servos properly at which point the servo arms will rotate and 
press on the strings until the chord has been strummed. This process repeats to play a whole song. 
For more informtion visit:https://sites.google.com/view/luis-vazquez-personal-website/robotic-playing-guitar-attachment?authuser=1
*/

//PID Library

#include <PIDController.h>
//NOTE: I have changed the PIDController library to set limit range of minSpeed<x<bound AND bound<x<maxSpeed
//where bound is set in the library
#include <SPI.h>
#include <SD.h>
#include <Servo.h>


//ENCODER PINS
  //NOTE: Since we use interrupts (pins 2,3,21,20,18,19) we must have the C1 value of the encoder be one of these pins
#define ENCODER_A1 2
#define ENCODER_A2 30

#define ENCODER_B1 3
#define ENCODER_B2 31

#define ENCODER_C1 20
#define ENCODER_C2 22

#define ENCODER_D1 21
#define ENCODER_D2 23

#define ENCODER_E1 18
#define ENCODER_E2 16

#define ENCODER_F1 19
#define ENCODER_F2 17

//stores the current encoder position
volatile long int encoder_countA = 0;
volatile long int encoder_countB = 0;
volatile long int encoder_countC = 0;
volatile long int encoder_countD = 0;
volatile long int encoder_countE = 0;
volatile long int encoder_countF = 0;

int encoderTopA;
int encoderTopB;
int encoderTopC;
int encoderTopD;
int encoderTopE;
int encoderTopF;

//Motor variables
// **** NOTE: The following defined variables MUST BE PWM pins!!!! Arduino Mega has PWM for 2-13 and 44-46 pins (note pin 3 is used for interrupt)
//'XIN_' X is the input to the DRV3388 board (A or B) and the A,B,C,D,E,F are the motors and 1,2 are the motor input for X
#define AIN_A1 4
#define AIN_A2 5
#define BIN_B1 6
#define BIN_B2 7
#define AIN_C1 8
#define AIN_C2 9
#define BIN_D1 10
#define BIN_D2 11
#define AIN_E1 12
#define AIN_E2 13
#define BIN_F1 44
#define BIN_F2 45

int nSleep1 = 24;
int nSleep2 = 25;
int nSleep3 = 26;

//PID Constants
#define __Kp 50 //Proportional Constant
#define __Ki 1 //Integral Constant
#define __Kd 2000 //Derivative Constant

//PID computation data is stored in thiese variables
int PWM_A = 0;
int PWM_B = 0;
int PWM_C = 0;
int PWM_D = 0;
int PWM_E = 0;
int PWM_F = 0;
PIDController pidcontrollerA;
PIDController pidcontrollerB;
PIDController pidcontrollerC;
PIDController pidcontrollerD;
PIDController pidcontrollerE;
PIDController pidcontrollerF;


//OTHER VARIABLES
const int MOTORNUM = 6;
double dist_per_rot = 33; //distance (mm) per rotation
double encodedVal_per_rot = 207; //encoded output for one rotation
const int maxSpeed = 255;
const int minSpeed = 150; //FIND BEST


//MOTION SETTING
char motorIndicator1;
char motorIndicator2;
char motorIndicator3;
char motorIndicator4;
char motorIndicator5;
char motorIndicator6;
float desiredStruct1 = 0;
float desiredStruct2 = 0;
float desiredStruct3 = 0;
float desiredStruct4 = 0;
float desiredStruct5 = 0;
float desiredStruct6 = 0;
String instruct1;
String instruct2;
String instruct3;
String instruct4;
String instruct5;
String instruct6;
  
//ARRAYS ([length+1] the plus one is for the null)
int motorArray[MOTORNUM * 2 + 1] = {AIN_A1,AIN_A2,BIN_B1,BIN_B2,AIN_C1,AIN_C2,BIN_D1,BIN_D2,AIN_E1,AIN_E2,BIN_F1,BIN_F2};
String instructions[MOTORNUM + 1] = {};
int PWMs[MOTORNUM + 1];
volatile long int encodersCount[MOTORNUM + 1] = {encoder_countA, encoder_countB,encoder_countC,encoder_countE,encoder_countF};


//BUTTONS
#define buttonA 33
#define buttonB 35
#define buttonC 37
#define buttonD 39
#define buttonE 41
#define buttonF 43
int buttons[MOTORNUM + 1] = {buttonA,buttonB,buttonC,buttonD,buttonE,buttonF};
bool inPlace = LOW;

//MICROSD CARD
//NOTE: For the arduino MEGA the connections are as follows: MISO -> 50, MOSI -> 51, SCK -> 52, CS -> 53
const int chipSelect = 53;
String fileName = "TEST1.txt";

//SERVO MOTORS 
Servo servoA;
Servo servoB;
Servo servoC;
Servo servoD;
Servo servoE;
Servo servoF;

//SERVO's VARIABLES
int unpressPosArray[MOTORNUM+1] = {2.5,30,70,105,150,170}; //Need to change depending on what it is
int pressPosArray[MOTORNUM+1] = {60,130,170,10,50,105};
int strings[MOTORNUM+1];
int servoPos[MOTORNUM+1];


//SETUP
void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_A1, INPUT);
  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B1, INPUT);
  pinMode(ENCODER_B2, INPUT);
  pinMode(ENCODER_C1, INPUT);
  pinMode(ENCODER_C2, INPUT);
  pinMode(ENCODER_D1, INPUT);
  pinMode(ENCODER_D2, INPUT);
  pinMode(ENCODER_E1, INPUT);
  pinMode(ENCODER_E2, INPUT);
  pinMode(ENCODER_F1, INPUT);
  pinMode(ENCODER_F2, INPUT);
  
  pinMode(AIN_A1, OUTPUT);
  pinMode(AIN_A2, OUTPUT);
  pinMode(BIN_B1, OUTPUT);
  pinMode(BIN_B2, OUTPUT);
  pinMode(AIN_C1, OUTPUT);
  pinMode(AIN_C2, OUTPUT);
  pinMode(BIN_D1, OUTPUT);
  pinMode(BIN_D2, OUTPUT);
  pinMode(AIN_E1, OUTPUT);
  pinMode(AIN_E2, OUTPUT);
  pinMode(BIN_F1, OUTPUT);
  pinMode(BIN_F2, OUTPUT);
  
  pinMode(nSleep1, OUTPUT);
  pinMode(nSleep2, OUTPUT);
  pinMode(nSleep3, OUTPUT);
  
  pinMode(buttonA, INPUT_PULLUP);
  pinMode(buttonB, INPUT_PULLUP);
  pinMode(buttonC, INPUT_PULLUP);
  pinMode(buttonD, INPUT_PULLUP);
  pinMode(buttonE, INPUT_PULLUP);
  pinMode(buttonF, INPUT_PULLUP);

  digitalWrite(nSleep1, HIGH);
  digitalWrite(nSleep2, HIGH);
  digitalWrite(nSleep3, HIGH);
  digitalWrite(AIN_A1, LOW);
  digitalWrite(AIN_A2, LOW);
  digitalWrite(BIN_B1, LOW);
  digitalWrite(BIN_B2, LOW);
  digitalWrite(AIN_C1, LOW);
  digitalWrite(AIN_C2, LOW);
  digitalWrite(BIN_D1, LOW);
  digitalWrite(BIN_D2, LOW);
  digitalWrite(AIN_E1, LOW);
  digitalWrite(AIN_E2, LOW);
  digitalWrite(BIN_F1, LOW);
  digitalWrite(BIN_F2, LOW);

  //Servos
  servoA.attach(32);
  servoB.attach(34);
  servoC.attach(36);
  servoD.attach(38);
  servoE.attach(40);
  servoF.attach(42);

  servoA.write(unpressPosArray[0]);
  servoB.write(unpressPosArray[1]);
  servoC.write(unpressPosArray[2]);
  servoD.write(unpressPosArray[3]);
  servoE.write(unpressPosArray[4]);
  servoF.write(unpressPosArray[5]);

  //attaching an interrupt to pin ENCODER_AX of the Arduino, and when the pulse is in the RISING edge call
  //the function encoder()
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), encoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C1), encoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_D1), encoder4, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_E1), encoder5, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_F1), encoder6, RISING);

  //initialize the PID instance
  pidcontrollerA.begin();
  pidcontrollerB.begin();
  pidcontrollerC.begin();
  pidcontrollerD.begin();
  pidcontrollerE.begin();
  pidcontrollerF.begin();
  
  // Tuning the PID
  pidcontrollerA.tune(__Kp, __Ki, __Kd);
  pidcontrollerB.tune(__Kp, __Ki, __Kd);
  pidcontrollerC.tune(__Kp, __Ki, __Kd);
  pidcontrollerD.tune(__Kp, __Ki, __Kd);
  pidcontrollerE.tune(__Kp, __Ki, __Kd);
  pidcontrollerF.tune(__Kp, __Ki, __Kd);
  //Limit the PID output (important to get rid of integral windup)
  pidcontrollerA.limit(-minSpeed, maxSpeed);
  pidcontrollerB.limit(-minSpeed, maxSpeed);
  pidcontrollerC.limit(-minSpeed, maxSpeed);
  pidcontrollerD.limit(-minSpeed, maxSpeed);
  pidcontrollerE.limit(-minSpeed, maxSpeed);
  pidcontrollerF.limit(-minSpeed, maxSpeed);

  //MICROSD CARD
  //Initializing the SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1); // do nothing then
  }
  Serial.println("Card initialized");

}


void loop() {
  //Start by making all motors go to the top
  if (inPlace != HIGH){
    move2pos();
  }
  Serial.println("ALL BUTTONS PRESSED");

  //Resetting the encoder's count once at the top
  for (int k=0;k<MOTORNUM;k++){
    encodersCount[k] = 0;
  }
  

  //Reading from the SD card
  File dataFile = SD.open(fileName);
  
  if (dataFile) {
    while (dataFile.available()) {
      servoA.write(unpressPosArray[0]);
      servoB.write(unpressPosArray[1]);
      servoC.write(unpressPosArray[2]);
      servoD.write(unpressPosArray[3]);
      servoE.write(unpressPosArray[4]);
      servoF.write(unpressPosArray[5]);
      
      for (int i = 0; i < MOTORNUM; i++) {
        instructions[i] = dataFile.readStringUntil('\n');
      }

      //The following breaks down the information (not my favourite way but .setpoint() is very particular as to what it takes e.g NOT instructions[x])
      instruct1 = instructions[0];
      instruct2 = instructions[1];
      instruct3 = instructions[2];
      instruct4 = instructions[3];
      instruct5 = instructions[4];
      instruct6 = instructions[5];
      
      //For servo: Need to get last character "U" or "P" from instructX 
      //Note: "P" = 80 "U" = 85

      strings[0] = int(instruct1[1]);
      strings[1] = int(instruct2[1]);
      strings[2] = int(instruct3[1]);
      strings[3] = int(instruct4[1]);
      strings[4] = int(instruct5[1]);
      strings[5] = int(instruct6[1]);
      
      // removing the characters from the string
      instruct1.remove(0,2);
      instruct2.remove(0,2);
      instruct3.remove(0,2);
      instruct4.remove(0,2);
      instruct5.remove(0,2);
      instruct6.remove(0,2); 
      
      //Note: the encoderTopX's are constants (the amount that each motor rotated to get to starting positioned) which will be substracted
      desiredStruct1 = float(trans2rot(1,encoderTopA,instruct1.toInt())); 
      desiredStruct2 = float(trans2rot(1,encoderTopB,instruct2.toInt()));
      desiredStruct3 = float(trans2rot(1,encoderTopC,instruct3.toInt()));
      desiredStruct4 = float(trans2rot(0,encoderTopD,instruct4.toInt()));
      desiredStruct5 = float(trans2rot(0,encoderTopE,instruct5.toInt()));
      desiredStruct6 = float(trans2rot(0,encoderTopF,instruct6.toInt())); 
      
      //Transfering the instruction to the PID controllers
      pidcontrollerA.setpoint(desiredStruct1); //.setpoint doesn't take instructions[0] in
      pidcontrollerB.setpoint(desiredStruct2);
      pidcontrollerC.setpoint(desiredStruct3);
      pidcontrollerD.setpoint(desiredStruct4);
      pidcontrollerE.setpoint(desiredStruct5);
      pidcontrollerF.setpoint(desiredStruct6);

      bool inRange = LOW;

      while (!inRange){ //Keep moving until you find the position w/in certain range(when speed reached is minSpeed (slowing down))
             PWM_A = pidcontrollerA.compute(encoder_countA);
             PWM_B = pidcontrollerB.compute(encoder_countB);
             PWM_C = pidcontrollerC.compute(encoder_countC);
             PWM_D = pidcontrollerD.compute(encoder_countD);
             PWM_E = pidcontrollerE.compute(encoder_countE);
             PWM_F = pidcontrollerF.compute(encoder_countF);
             PWMs[0] = PWM_A;
             PWMs[1] = PWM_B;
             PWMs[2] = PWM_C;
             PWMs[3] = -PWM_D;
             PWMs[4] = -PWM_E;
             PWMs[5] = -PWM_F;


             for (int k=0;k<6;k++){
              Serial.println(PWMs[k]);
             }
             Serial.println("-------------");
             
             //Taking action according to what the PID controllers said to do
             inRange = range(PWMs); //range(desiredStruct1,encoder_countA);
             set_direction(encodersCount, PWMs);
      }
      // Serial follow along statements
             Serial.print("Motor A Desired: ");
             Serial.println(instruct1.toInt());
             Serial.print("Current A position: ");
             Serial.println(rot2trans(encoderTopA,encoder_countA));
             Serial.print("Motor A PWM: ");
             Serial.println(PWMs[0]);
             Serial.println("   --------   ");
             Serial.print("Motor B Desired: ");
             Serial.println(instruct2.toInt());
             Serial.print("Current B position: ");
             Serial.println(rot2trans(encoderTopB,encoder_countB));
             Serial.print("Motor B PWM: ");
             Serial.println(PWMs[1]);
             Serial.println("   --------   ");
             Serial.print("Motor C Desired: ");
             Serial.println(instruct3.toInt());
             Serial.print("Current C position: ");
             Serial.println(rot2trans(encoderTopC,encoder_countC));
             Serial.print("Motor C PWM: ");
             Serial.println(PWMs[2]);
             Serial.println("   --------   ");
             Serial.print("Motor D Desired: ");
             Serial.println(instruct4.toInt());
             Serial.print("Current D position: ");
             Serial.println(-rot2trans(encoderTopD,encoder_countD));
             Serial.print("Motor D PWM: ");
             Serial.println(PWMs[3]);
             Serial.println("   --------   ");
             Serial.print("Motor E Desired: ");
             Serial.println(instruct5.toInt());
             Serial.print("Current E position: ");
             Serial.println(-rot2trans(encoderTopE,encoder_countE));
             Serial.print("Motor E PWM: ");
             Serial.println(PWMs[4]);
             Serial.println("   --------   ");
             Serial.print("Motor F Desired: ");
             Serial.println(instruct6.toInt());
             Serial.print("Current F position: ");
             Serial.println(-rot2trans(encoderTopF,encoder_countF));
             Serial.print("Motor F PWM: ");
             Serial.println(PWMs[5]);
             Serial.println("********************************");

             //SETTING SERVO'S POSITIONS
             for (int r = 0;r<MOTORNUM;r++){ //Note: "P" = 80 "U" = 85
              //Serial.println(strings[r]);
              if (strings[r] == 80){ //if unpressed
                servoPos[r] = pressPosArray[r];
              }
              else{
                servoPos[r] = unpressPosArray[r];
              }
              //Serial.println(servoPos[r]);
              //Serial.println("------------");
             }
             
             servoA.write(servoPos[0]);
             servoB.write(servoPos[1]);
             servoC.write(servoPos[2]);
             servoD.write(servoPos[3]);
             servoE.write(servoPos[4]);
             servoF.write(servoPos[5]);

      delay(1000);
    }
    //Once the song is over power off the motors by "putting the motor drivers to sleep"
  dataFile.close();
  digitalWrite(nSleep1,LOW);
  digitalWrite(nSleep2,LOW);
  digitalWrite(nSleep3,LOW);
    // unpress the strings
  servoA.write(unpressPosArray[0]);
  servoB.write(unpressPosArray[1]);
  servoC.write(unpressPosArray[2]);
  servoD.write(unpressPosArray[3]);
  servoE.write(unpressPosArray[4]);
  servoF.write(unpressPosArray[5]);
  for (int i=0;i<500;i++){
    Serial.println("THE END");
    delay(1000);
  }
  }
  else {
    Serial.println("Error opening File");
  }
}

//Starting sequence function
//->rotate all DC motors until their respective servo cart is at the top (button pressed)
void move2pos() {
  for (int r=0; r < MOTORNUM; r++) {
     PWMs[r] = maxSpeed;
  }
  while (HIGH) {
    int sum = 0;
    for (int k = 0; k < MOTORNUM; k++) {
      if (digitalRead(buttons[k]) == LOW) { //if the button is pressed (active-high button)
        PWMs[k] = 0; //then don't move you're there
      }
      //The following checks that all buttons have been pressed
      for (int i = 0; i < MOTORNUM; i++) {
        sum = sum + abs(PWMs[i]);
      }
      set_direction(encodersCount, PWMs);
      if (sum == 0) { //if all motor speeds set to zero then all buttons have been pressed
        goto pressedButtons;
      }
    }
  }
  pressedButtons:
  inPlace = HIGH;
  encoderTopA = encoder_countA;
  encoderTopB = encoder_countB;
  encoderTopC = encoder_countC;
  encoderTopD = encoder_countD;
  encoderTopE = encoder_countE;
  encoderTopF = encoder_countF; 
  delay(2000);
}

//DIRECTION SETTING FUNCTIONS

//deciding the DC motor directions
void set_direction(volatile long int encodersCount[MOTORNUM + 1], int PWMs[MOTORNUM + 1]) {
  // note that due to the layout of the DC motors, half spin in one direction and the other half in the other
  for (int i = 0; i <= 2; i++) {
    if (PWMs[i] > 0)
      motor_cw(i, PWMs[i]);
    else
      motor_ccw(i, abs(PWMs[i]));
  } 
  //for the other dc motors the direction is reversed
  for (int n = 3; n <= 5; n++) {
    if (PWMs[n] > 0)
      motor_ccw(n, PWMs[n]);
    else
      motor_cw(n, abs(PWMs[n])); 
  }

}

//writing to the DC motors to rotate clockwise
void motor_ccw(int k, int _speed) {
  int index = k * 2;
  if (_speed > minSpeed) {
    analogWrite(motorArray[index], _speed);
    digitalWrite(motorArray[index + 1], LOW);
  }
  else {
    digitalWrite(motorArray[index], LOW);
    digitalWrite(motorArray[index + 1], LOW);
  }
}

//writing to the DC motors to rotate counter-clockwise
void motor_cw(int k, int _speed) {
  int index = k * 2;
  if (_speed > minSpeed) {
    digitalWrite(motorArray[index], LOW);
    analogWrite(motorArray[index + 1], _speed);
  }
  else {
    digitalWrite(motorArray[index], LOW);
    digitalWrite(motorArray[index + 1], LOW);
  }
}

//this fucnti0n translates from translational (servo cart) distance to the equivalent DC motor rotations
float trans2rot(int metal, int encoderTop, int transDist){
   int rotDist = round(transDist/dist_per_rot*encodedVal_per_rot);
   if (metal){
    return encoderTop - rotDist;
   }else{
    return encoderTop + rotDist;
   } 
}

//This function translates from the DC motors rotation to the equivalent servo cart translational distance
int rot2trans(int encoderTop, int rotDist){
   int totalRotDist = encoderTop - rotDist;
   return round(totalRotDist/encodedVal_per_rot*dist_per_rot);
}


bool range(int PWMs[MOTORNUM+1]){
  delayMicroseconds(800); //NEED THIS DELAY IN ORDER TO READ PWM speeds w/o messing it up due to the square waves
  int total =0;
  for (int a=0;a<MOTORNUM;a++){
    if (abs(PWMs[a]) <= minSpeed){
      total++;
    }
  }
  if (total == MOTORNUM ){
    return HIGH;
  }else{
    return LOW;
  }
}


//Using the interrupts, this is where the encoder counts increase (6 interrupts)
void encoder1() {
  if (digitalRead(ENCODER_A2) == HIGH) //If ENCODER A2 is high increase the count
    encoder_countA++;
  else
    encoder_countA--;
}
void encoder2() {
  if (digitalRead(ENCODER_B2) == HIGH)
    encoder_countB++;
  else
    encoder_countB--;
}
void encoder3() {
  if (digitalRead(ENCODER_C2) == HIGH)
    encoder_countC++;
  else
    encoder_countC--;
}
void encoder4() {
  if (digitalRead(ENCODER_D2) == HIGH)
    encoder_countD++;
  else
    encoder_countD--;
}
void encoder5() {
  if (digitalRead(ENCODER_E2) == HIGH)
    encoder_countE++;
  else
    encoder_countE--;
}
void encoder6() {
  if (digitalRead(ENCODER_F2) == HIGH)
    encoder_countF++;
  else
    encoder_countF--;
}
  
