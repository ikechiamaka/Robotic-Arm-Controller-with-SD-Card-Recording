

// Include Wire Library for I2C Communications
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
// Include Adafruit PWM Library
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


//define the buttons
//const int Record = 12;
//const int play = 13;
const int chipSelect = 4;
// Analog pin for potentiometer
//int analogPin = 0;
// Integer to hold potentiometer value
int val = 0, val1=0;
 const int play = 13;   //button pin for recording current position
const int modeBtn = 12;  //button pin for selecting mode 
const int delBtn = 10;
 int record;
int buttonState = 1;
int delbuttonState = 1;
String line, buff; 
File dataFile;
int StoredPos[6] = {0, 0, 0, 0, 0, 0};
int x, y, z, a, b, c;

//define variable for values of the button
//int RecordPressed = 0;
boolean playPressed = false;

// Define Potentiometer Inputs

int controlBase = A0;
int controlShoulder = A1;
int controlElbow = A2;
int controlWrist = A3;
int controlPivot = A6;
int controlJaws = A7;



//define variable for saved position of the servos
//int motorBasePosSave[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//int motorShoulderPosSave[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//int motorElbowPosSave[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//int motorWristPosSave[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//int motorPivotPosSave[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//int motorJawsPosSave[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};



// Define Motor Outputs on PCA9685 board

int motorBase = 0;
int motorShoulder = 1;
int motorElbow = 2;
int motorWrist = 3;
int motorPivot = 4;
int motorJaws = 5;

// Define Motor position variables
int mtrDegreeBase;
int mtrDegreeShoulder;
int mtrDegreeElbow;
int mtrDegreeWrist;
int mtrDegreePivot;
int mtrDegreeJaws;

void setup()
{
  
  // Setup PWM Controller object
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  pinMode(play, INPUT_PULLUP);
  pinMode(delBtn, INPUT_PULLUP);
  pinMode(modeBtn, INPUT_PULLUP);
  pinMode(13,OUTPUT);
  record = digitalRead(modeBtn);
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
 
 
  Serial.print("Initializing SD card...");
 
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  
  // Attach servo on pin 9 to the servo object
//  myservo.attach(9); 
//  myservo1.attach(3);

   dataFile = SD.open("servopos.txt", FILE_WRITE);
  if (! dataFile) {
    Serial.println("error opening datalog.txt");
    // Wait forever since we cant write data
    while (1) ;
  }

  //define buttons as input units
  
}

// Function to move motor to specific position
void moveMotorDeg(int moveDegree, int motorOut)
{
  int pulse_wide, pulse_width;

  // Convert to pulse width
  pulse_wide = map(moveDegree, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);

  //Control Motor
  pwm.setPWM(motorOut, 0, pulse_width);
}

// Function to convert potentiometer position into servo angle
int getDegree(int controlIn)
{
  int potVal, srvDegree;

  // Read values from potentiometer
  potVal = analogRead(controlIn);

  // Calculate angle in degrees
  srvDegree = map(potVal, 0, 1023, 0, 180);

  // Return angle in degrees
  return srvDegree;

}

void loop() {
 delbuttonState = digitalRead(delBtn);

//  if (record == LOW) { //button is in learning mode
    buttonState = digitalRead(play);
  //Control Base Motor

  // Get desired position
  mtrDegreeBase = getDegree(controlBase);
  // Move motor
  moveMotorDeg(mtrDegreeBase, motorBase);

  // Control Shoulder motor

  // Get desired position
  mtrDegreeShoulder = getDegree(controlShoulder);
  // Move motor
  moveMotorDeg(mtrDegreeShoulder, motorShoulder);

  //Control Elbow Motor

  // Get desired position
  mtrDegreeElbow = getDegree(controlElbow);
  // Move motor
  moveMotorDeg(mtrDegreeElbow, motorElbow);


  //Control Wrist Motor

  // Get desired position
  mtrDegreeWrist = getDegree(controlWrist);
  // Move motor
  moveMotorDeg(mtrDegreeWrist, motorWrist);


  //Control Pivot Motor

  // Get desired position
  mtrDegreePivot = getDegree(controlPivot);
  // Move motor
  moveMotorDeg(mtrDegreePivot, motorPivot);


  //Control Jaws Motor

  // Get desired position
  mtrDegreeJaws = getDegree(controlJaws);
  // Move motor
  moveMotorDeg(mtrDegreeJaws, motorJaws);


  // Add short delay
  delay(20);

  //if Record is pressed (HIGH), save the potentiometers position
  //as long as Record is pressed
//  if (digitalRead(Record) == HIGH) {
//    RecordPressed++;
//    switch (RecordPressed) {
//      case 1:
//        motorBasePosSave[0] = mtrDegreeBase;
//        motorShoulderPosSave[0] = mtrDegreeShoulder;
//        motorElbowPosSave[0] = mtrDegreeElbow;
//        motorWristPosSave[0] = mtrDegreeWrist;
//        motorPivotPosSave[0] = mtrDegreePivot;
//        motorJawsPosSave[0] = mtrDegreeJaws;
//        Serial.println("Position #1 Saved");
//
//        delay(500);
//        break;
//      case 2:
//        motorBasePosSave[1] = mtrDegreeBase;
//        motorShoulderPosSave[1] = mtrDegreeShoulder;
//        motorElbowPosSave[1] = mtrDegreeElbow;
//        motorWristPosSave[1] = mtrDegreeWrist;
//        motorPivotPosSave[1] = mtrDegreePivot;
//        motorJawsPosSave[1] = mtrDegreeJaws;
//        Serial.println("Position #2 Saved");
//
//        delay(500);
//        break;
//      case 3:
//        motorBasePosSave[2] = mtrDegreeBase;
//        motorShoulderPosSave[2] = mtrDegreeShoulder;
//        motorElbowPosSave[2] = mtrDegreeElbow;
//        motorWristPosSave[2] = mtrDegreeWrist;
//        motorPivotPosSave[2] = mtrDegreePivot;
//        motorJawsPosSave[2] = mtrDegreeJaws;
//        Serial.println("Position #3 Saved");
//
//        delay(500);
//        break;
//      case 4:
//        motorBasePosSave[3] = mtrDegreeBase;
//        motorShoulderPosSave[3] = mtrDegreeShoulder;
//        motorElbowPosSave[3] = mtrDegreeElbow;
//        motorWristPosSave[3] = mtrDegreeWrist;
//        motorPivotPosSave[3] = mtrDegreePivot;
//        motorJawsPosSave[3] = mtrDegreeJaws;
//        Serial.println("Position #4 Saved");
//
//        delay(500);
//        break;
//      case 5:
//        motorBasePosSave[4] = mtrDegreeBase;
//        motorShoulderPosSave[4] = mtrDegreeShoulder;
//        motorElbowPosSave[4] = mtrDegreeElbow;
//        motorWristPosSave[4] = mtrDegreeWrist;
//        motorPivotPosSave[4] = mtrDegreePivot;
//        motorJawsPosSave[4] = mtrDegreeJaws;
//
//        delay(500);
//        break;
//      case 6:
//        motorBasePosSave[5] = mtrDegreeBase;
//        motorShoulderPosSave[5] = mtrDegreeShoulder;
//        motorElbowPosSave[5] = mtrDegreeElbow;
//        motorWristPosSave[5] = mtrDegreeWrist;
//        motorPivotPosSave[5] = mtrDegreePivot;
//        motorJawsPosSave[5] = mtrDegreeJaws;
//        Serial.println("Position #6 Saved");
//
//        delay(500);
//        break;
//      case 7:
//        motorBasePosSave[6] = mtrDegreeBase;
//        motorShoulderPosSave[6] = mtrDegreeShoulder;
//        motorElbowPosSave[6] = mtrDegreeElbow;
//        motorWristPosSave[6] = mtrDegreeWrist;
//        motorPivotPosSave[6] = mtrDegreePivot;
//        motorJawsPosSave[6] = mtrDegreeJaws;
//        Serial.println("Position #7 Saved");
//
//        delay(500);
//        break;
//      case 8:
//        motorBasePosSave[7] = mtrDegreeBase;
//        motorShoulderPosSave[7] = mtrDegreeShoulder;
//        motorElbowPosSave[7] = mtrDegreeElbow;
//        motorWristPosSave[7] = mtrDegreeWrist;
//        motorPivotPosSave[7] = mtrDegreePivot;
//        motorJawsPosSave[7] = mtrDegreeJaws;
//        Serial.println("Position #8 Saved");
//
//        delay(500);
//        break;
//      case 9:
//        motorBasePosSave[8] = mtrDegreeBase;
//        motorShoulderPosSave[8] = mtrDegreeShoulder;
//        motorElbowPosSave[8] = mtrDegreeElbow;
//        motorWristPosSave[8] = mtrDegreeWrist;
//        motorPivotPosSave[8] = mtrDegreePivot;
//        motorJawsPosSave[8] = mtrDegreeJaws;
//        Serial.println("Position #9 Saved");
//
//        delay(500);
//        break;
//      case 10:
//        motorBasePosSave[9] = mtrDegreeBase;
//        motorShoulderPosSave[9] = mtrDegreeShoulder;
//        motorElbowPosSave[9] = mtrDegreeElbow;
//        motorWristPosSave[9] = mtrDegreeWrist;
//        motorPivotPosSave[9] = mtrDegreePivot;
//        motorJawsPosSave[9] = mtrDegreeJaws;
//        Serial.println("Position #10 Saved");
//
//        delay(500);
//        break;
//      case 11:
//        motorBasePosSave[10] = mtrDegreeBase;
//        motorShoulderPosSave[10] = mtrDegreeShoulder;
//        motorElbowPosSave[10] = mtrDegreeElbow;
//        motorWristPosSave[10] = mtrDegreeWrist;
//        motorPivotPosSave[10] = mtrDegreePivot;
//        motorJawsPosSave[10] = mtrDegreeJaws;
//        Serial.println("Position #11 Saved");
//
//        delay(500);
//        break;
//      case 12:
//        motorBasePosSave[11] = mtrDegreeBase;
//        motorShoulderPosSave[11] = mtrDegreeShoulder;
//        motorElbowPosSave[11] = mtrDegreeElbow;
//        motorWristPosSave[11] = mtrDegreeWrist;
//        motorPivotPosSave[11] = mtrDegreePivot;
//        motorJawsPosSave[11] = mtrDegreeJaws;
//        Serial.println("Position #12 Saved");
//
//        delay(500);
//        break;
//
//    }
//  }
digitalWrite(13, HIGH);
//    dataString += String(val);
    String dataString = "";

    if (record == LOW) {
Serial.print("learning mode");


      
      for (int analogPin = 0; analogPin < 6; analogPin++) {
        if (analogPin == 0) {
          dataString += "x";
          Serial.println(dataString);
        }
        else if (analogPin == 1) {
          dataString += "y";
          Serial.println(dataString);
        }
         else if (analogPin == 2) {
          dataString += "z";
          Serial.println(dataString);
        }
         else if (analogPin == 3) {
          dataString += "a";
          Serial.println(dataString);
        }
         else if (analogPin == 4) {
          dataString += "b";
          Serial.println(dataString);
        }
         else if (analogPin == 5) {
          dataString += "c";
          Serial.println(dataString);
        }
        int sensor = analogRead(analogPin);
        dataString += String(sensor);
        }
  // Write to the servo
  // Delay to allow servo to settle in position
//  myservo.write(val);
dataFile.println(dataString);
      digitalWrite(13, HIGH);
      
      dataFile.flush();
      delay(500);
    }

    else {
      //Serial.println("nista");
      digitalWrite(13, LOW);
    }
  if(delbuttonState == LOW){
      
      SD.remove("servopos.txt");
//      digitalWrite(ledPin2, HIGH);
      Serial.print("servopos.txt deleted!");
      
      }
    else{
    Serial.println("servopos.txt still running"); 
//    digitalWrite(ledPin2, LOW); 
      }
  

  //if play pressed (HIGH), the servos move saved position
  if (buttonState == LOW) {
    playPressed = true;
  }

  if (playPressed) {
    digitalWrite(13, LOW);
    dataFile.close();
    dataFile = SD.open("servopos.txt", FILE_READ);
    while (dataFile.available()) {
      line = dataFile.readStringUntil('\n');
      
      StoredPos[0] = line.indexOf('x');
      StoredPos[1] = line.indexOf('y');
      StoredPos[2] = line.indexOf('z');
      StoredPos[3] = line.indexOf('a');
      StoredPos[4] = line.indexOf('b');
      StoredPos[5] = line.indexOf('c');

      buff = line.substring(StoredPos[0]+1, StoredPos[1]);
      //Serial.println(buff);
      x = buff.toInt();
      buff = line.substring(StoredPos[1]+1, StoredPos[2]);
      //Serial.println(buff);
      y = buff.toInt();
       buff = line.substring(StoredPos[2]+1, StoredPos[3]);
      //Serial.println(buff);
      z = buff.toInt();
      buff = line.substring(StoredPos[3]+1, StoredPos[4]);
      //Serial.println(buff);
      a = buff.toInt();
       buff = line.substring(StoredPos[4]+1, StoredPos[5]);
      //Serial.println(buff);
      b = buff.toInt();
      buff = line.substring(StoredPos[5]+1, StoredPos[6]);
      //Serial.println(buff);
      c = buff.toInt();

      
      moveMotorDeg(x, motorBase);
      moveMotorDeg(y, motorShoulder);
      moveMotorDeg(z, motorElbow);
      moveMotorDeg(a, motorWrist);
      moveMotorDeg(b, motorPivot);
      moveMotorDeg(c, motorJaws);
//    for (int i = 0; i < 12; i++) {
//      moveMotorDeg(motorBasePosSave[i], motorBase);
//      moveMotorDeg(motorShoulderPosSave[i], motorShoulder);
//      moveMotorDeg(motorElbowPosSave[i], motorElbow);
//      moveMotorDeg(motorWristPosSave[i], motorWrist);
//      moveMotorDeg(motorPivotPosSave[i], motorPivot);
//      moveMotorDeg(motorJawsPosSave[i], motorJaws);
//
//      delay(2000);
//    }
  }
  

}}
