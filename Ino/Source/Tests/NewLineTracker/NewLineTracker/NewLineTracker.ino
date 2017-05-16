#include "printf.h"
#include "RF24.h"

// Variables for sensors
bool sensors[6] = {false, false, false, false, false, false};
float black = 0.75;

// Variables for motor pins
int pwm_a = 5;
int pwm_b = 6;
int dir_a = 4;
int dir_b = 7;

// Variables for motor
int turbo = 135;
int delayAdjustment = 25;

// Variables for intersections
bool left = false;
bool right = false;

// Variables for dead end or straight tile
bool whiteAvailable = true;
int whiteCounter = 0;
int deadEndCounter = 0;

// Variables for radio
RF24 myRadio(9, 10); 
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

void setup() {
  // Activates the Serial output of the arduino.
  Serial.begin(9600); 
  // Activates the add-on cass.
  printf_begin();  

  myRadio.begin(); 
  myRadio.openWritingPipe(pipes[1]);
  myRadio.openReadingPipe(1, pipes[0]);
  myRadio.startListening();

  //Set the motor control pins to be outputs
  pinMode(pwm_a, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, LOW);

  Serial.println("Setup Done");
}

// Checks if a sensor detects a line
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
  if (detectedValue > black) {
    return true;
  }
  else
  {
    return false;
  }
}

// Reads all the sensors
void readAllSensors()
{
  sensors[0] = readSensor(0);
  sensors[1] = readSensor(1);
  sensors[2] = readSensor(2);
  sensors[3] = readSensor(3);
  sensors[4] = readSensor(4);
  sensors[5] = readSensor(5);
}

// Checks if the autobot is on the line
bool checkOnLine()
{
  if(sensors[1] || sensors[2] || sensors[3] || sensors[4])
  {
    return true;
  }
  else
  {
    return false;
  }
}

// Check if there is an intersection
int checkIntersection()
{
  // check if we can go straight
  if(sensors[1] || sensors[2] || sensors[3] || sensors[4])
  {
    // check if we can go all directions
    if(sensors[0] && sensors[5])
    {
      return 13;
    }
    // check if we can go straight and left
    else if(sensors[0]) 
    {
      return 11;
    }
    // check if we can go straight and right
    else if(sensors[5]) 
    {
      return 12;
    }
  }
  // check if we can go left and right
  else if(sensors[0] && sensors[5]) 
  {
    return 16;
  }
  //check if we can only go left
  else if(sensors[0]) 
  {
    return 14;
  }
  // check if we can only go right
  else if(sensors[5]) 
  {
    return 15;
  }
  // check for straight tile or dead end
  else 
  {
      int whites = 0;//checkWhites();
      // return a dead end
      if(whites == 1)
      {
        return 17;
      }
      // return a straight tile
      else if(whites == 2)
      {
        return 10;
      }
      else
      {
        // there is no intersection
        return 51;
      }
  }
}

// check for a straight tile or a dead end
int checkWhites()
{
  // all the sensors return false
  if (!sensors[1] && !sensors[2] && !sensors[3] && !sensors[4] && whiteAvailable)
  { //Increase the amount of white lines detected when the PID sensors detect white
    whiteCounter++;
    deadEndCounter++;
    whiteAvailable = false;
  }
  else if(checkOnLine())
  {
    whiteAvailable = true;
    deadEndCounter = 0;
  }

  // a dead end has been detected
  if(deadEndCounter > 100)
  {
    return 1;
  }

  // a straight tile has been detected
  else if(whiteCounter == 3){
    whiteCounter = 0;
    whiteAvailable = true;
    return 2;
  }
  else
  {
    return 0;
  }
}

void goStraight()
{
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, LOW);
  analogWrite(pwm_a, turbo);
  analogWrite(pwm_b, turbo);
}

// Make a left turn
void goLeft()
{
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, HIGH);
  analogWrite(pwm_a, turbo - 20);
  analogWrite(pwm_b, turbo + 20);

  while(!sensors[2])
  {
    delay(10);
  }
  followLine();
}

// Make a right turn
void goRight()
{
  digitalWrite(dir_a, LOW);
  digitalWrite(dir_b, LOW);
  analogWrite(pwm_a, turbo + 20);
  analogWrite(pwm_b, turbo - 20);

  while(!sensors[2])
  {
    delay(10);
  }
  followLine();
}

// Stop
void stopGoing()
{
  analogWrite(pwm_a, 0);
  analogWrite(pwm_b, 0);
}

void adjustRight()
{
  digitalWrite(dir_a, LOW);
  digitalWrite(dir_b, LOW);
  analogWrite(pwm_a, turbo + 20);
  analogWrite(pwm_b, turbo);
  
  delay(delayAdjustment);
}

void adjustLeft()
{
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, HIGH);
  analogWrite(pwm_a, turbo);
  analogWrite(pwm_b, turbo + 20);
  
  delay(delayAdjustment);
}

// Follow the line
void followLine()
{
  if(sensors[1] || sensors[2] || sensors[3] || sensors[4])
  {
    if(sensors[0])
    // Autobot is going left, adjust the direction
    if(!sensors[1] && !sensors[2])
    {
      adjustLeft();
    }
    // Autobot is going right, adjust the direction
    else if(!sensors[3] && !sensors[4])
    {
      adjustRight();
    }
    else
    {
      goStraight();
    }
  }
  else
  {
    //stopGoing();
  }
}

// Check left
bool checkLeft()
{
  if(sensors[0])
  {
    left = true;
    return true;
  }
  else
  {
    return false;
  }
}

// Check right
bool checkRight()
{
  if(sensors[5])
  {
    right = true;
    return true;
  }
  else
  {
    return false;
  }
}

void sendMessage(int message)
{
  myRadio.stopListening();
  unsigned long id = message;
  bool ok = myRadio.write(&id, sizeof(unsigned long));
  if(ok)
  {
    printf("Message send: %d \n", message);
  }
  myRadio.startListening();
}

void loop() {
  // Read all the sensor values
  readAllSensors();
  // Start following the line
  followLine();

  if(checkLeft())
  {
    sendMessage(0);
  }
  if(checkRight())
  {
    sendMessage(1);
  }
  // Check for an intersection
  //int intersection = checkIntersection();  
  // there is no intersection, follow the line
  //sendMessage(intersection);
  // there is an intersection, send message to host
}
