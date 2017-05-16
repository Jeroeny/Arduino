#include "printf.h"
#include "RF24.h"

//Variables for sensor readings
int sensor[4] = { 1, 2, 3, 4 };  //PID sensors numbered from 1 to 4.
int sensorReadings[4] = { 0 };
int activeSensors = 0;
float totalReading = 0;
float avgReading = 0;
float lastReading = 0;

//Variables for motors
int rightMotorSpeed = 0;
int leftMotorSpeed = 0;
int maxMotorSpeed = 140;
float blackThreshold = 0.70;

//Variables for motor pins
int pwm_a = 5;
int pwm_b = 6;
int dir_a = 4;
int dir_b = 7;

//Variables for PID Controller
float pV = 0 ; // Process Variable used to adjust motor speeds to keep following the line
float kp = 80;  // This is the Proportional value. Increase this value if the robot doesn't stay on the line.
float kd = 7; // This is the Derivative value. Increase this value to make the robot countersteer more and go straight faster when detecting the line.
float ki = 0.0040; // This is the integral value. Increase this value to make the robot steer more and oscillate around the line more.

float error = 0; // Value between -1.5 and 1.5 to indicate where the line is under the robot. A negative value means the line is to the left and a positive value means the line is to the right
float previousError = 0;
float totalError = 0;

void setup()
{
  Serial.begin(9600); // Activates the Serial output of the arduino. You can see the Serial output with the Serial Monitor in the Arduino IDE.
  
  //Set the motor control pins to be outputs
  pinMode(pwm_a, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);
  
  //Set one motor to HIGH and one to LOW to make the robot go straight
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, LOW);

  Serial.println("Setup Done");
} // end setup

void loop()
{
  followLine();
} // end main loop

//Calculates where the line is and where the robot should turn to
void followLine()
{
  calcPID();
  analogWrite(pwm_a, leftMotorSpeed);
  analogWrite(pwm_b, rightMotorSpeed);
} // end follow_line


//Reads whether the given sensor detects black
bool readSensor(int sensor)
{
  int sensorValue;
  int range = 1024;
  float detectedValue;

  switch (sensor)
  {
    case 0: // read sensor 0.
      sensorValue = analogRead(A0);
      break;
    case 1: // read sensor 1.
      sensorValue = analogRead(A1);
      break;
    case 2: // read sensor 2.
      sensorValue = analogRead(A2);
      break;
    case 3: // read sensor 3.
      sensorValue = analogRead(A3);
      break;
    case 4: // read sensor 4.
      sensorValue = analogRead(A4);
      break;
    case 5: // read sensor 5.
      sensorValue = analogRead(A5);
      break;
  }
  detectedValue = static_cast<float>(sensorValue) / static_cast<float>(range);
  if (detectedValue > blackThreshold) {
    return true;
  }
  else
  {
    return false;
  }
}

void getPIDReading() {
  for (int i = 0; i < 4; i++) {
    sensorReadings[i] = readSensor(sensor[i]);
    if (sensorReadings[i] == 1) {
      activeSensors += 1;
    }
    totalReading += sensorReadings[i] * (i + 1);
  }

  avgReading = totalReading / activeSensors;
  lastReading = avgReading;
  totalReading = 0; activeSensors = 0;
}

void calcPID() {  
  //Get the reading
  getPIDReading();

  //Update the errors
  previousError = error;
   //The present error gets 2.5 subtracted so that if for example the right most sensor detects black
  //then the error becomes negative. If for example only the right sensor detected black then error = 1 - 2.5 = -1.5  
  error = avgReading - 2.5;
  totalError += error;
  
  //Calculate the process variable
  pV = (kp * error) + (kd * (error - previousError)) + (ki * totalError);

  if ( pV > maxMotorSpeed) {
    pV = maxMotorSpeed;
  }
  if ( pV < -maxMotorSpeed ) {
    pV = -maxMotorSpeed;
  }

  if (pV < 0) 
  { //Turn Left
    rightMotorSpeed = maxMotorSpeed;
    leftMotorSpeed = maxMotorSpeed - abs(pV);
  }
  else
  { //Turn Right
    rightMotorSpeed = maxMotorSpeed - pV;
    leftMotorSpeed = maxMotorSpeed;
  }
}
