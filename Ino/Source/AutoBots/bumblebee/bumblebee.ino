#include "printf.h"
#include "RF24.h"

// define the autobot
// 1 = optimus, 2 = bumblebee
int autoBot = 2;

// Variables for sensor readings
int sensor[4] = { 1, 2, 3, 4 };
int sensorReadings[4] = { 0 };
int activeSensors = 0;
float totalReading = 0;
float avgReading = 0;
float lastReading = 0;
float blackThreshold = 0.75;
bool sensor0 = false;
bool sensor1 = false;
bool sensor2 = false;
bool sensor3 = false;
bool sensor4 = false;
bool sensor5 = false;

// Variables for motors
int rightMotorSpeed = 0;
int leftMotorSpeed = 0;
int maxMotorSpeed = 135;

// Variables for pins
int pwm_a = 5;
int pwm_b = 6;
int dir_a = 4;
int dir_b = 7;

// Variables for PID
float error = 0;
float previousError = 0;
float totalError = 0;
float power = 0 ; 
float kp = 80;
float kd = 7;
float ki = 0.0040;

// Variables for the radio
RF24 myRadio(9, 10);
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
unsigned long messageTime = 0;

// Variables for random shit
bool turning = false;
bool moving = true;
unsigned long intersectionTimer = 0;
bool waitForInfo = false;
int messageID = 0;
bool straightTile = false;

// Variables for the maze
int arrayCounter;
int x = 0;
int y = 0;
int tile = 0;
int tiles[100][1][4]; // X-xoordinate, Y-coordinate, Tile type, Tile orientation
int autobotOrientation = 0; // 0-NORTH, 1-EAST, 2-SOUTH, 3-WEST
int tileOrientation = 0; // 0-NORTH, 1-EAST, 2-SOUTH, 3-WEST

void setup() {
  Serial.begin(9600);
  printf_begin();

  // Start the radio
  myRadio.begin();
  myRadio.openWritingPipe(pipes[1]);
  myRadio.openReadingPipe(1, pipes[0]);
  myRadio.startListening();

  //Set control pins to be outputs
  pinMode(pwm_a, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  // Start the motors
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, LOW);

  // Setup the messageID
  if(autoBot == 1)
  {
    messageID = 100;
  }
  else if(autoBot == 2)
  {
    messageID = 200;
  }
}

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

//Reads all the sensors
void readAllSensors()
{
  sensor0 = readSensor(0);
  sensor1 = readSensor(1);
  sensor2 = readSensor(2);
  sensor3 = readSensor(3);
  sensor4 = readSensor(4);
  sensor5 = readSensor(5);
}

//Checks if any of the PID sensors detected black
bool checkPIDSensors() 
{
  if (sensor1 || sensor2 || sensor3 || sensor4) 
  {
    return true;
  }
  else
  {
    return false;
  }
}

// line following subroutine
void followLine()
{
  calcPID();
  analogWrite(pwm_a, leftMotorSpeed);
  analogWrite(pwm_b, rightMotorSpeed);
}

// calculate motor speeds, used in followline()
void calcPID() 
{
  readLine();

  previousError = error;
  error = avgReading - 2.5;
  totalError += error;

  power = (kp * error) + (kd * (error - previousError)) + (ki * totalError);

  if ( power > maxMotorSpeed) {
    power = maxMotorSpeed;
  }
  if ( power < -maxMotorSpeed ) {
    power = -maxMotorSpeed;
  }

  if(error != -2.5)
  {
    if (power < 0)
    {
      rightMotorSpeed = maxMotorSpeed;
      leftMotorSpeed = maxMotorSpeed - abs(power);
    }
    else
    {
      rightMotorSpeed = maxMotorSpeed - power;
      leftMotorSpeed = maxMotorSpeed;
    }
  }
  else
  {
    delay(175);
  }
}

// check line, used in calcPID()
void readLine() 
{
  for (int i = 0; i < 4; i++) {
    sensorReadings[i] = readSensor(sensor[i]);
    if (sensorReadings[i] == 1) {
      activeSensors += 1;
    }
    totalReading += sensorReadings[i] * (i + 1);
  }
  if (activeSensors != 0)
  {
    avgReading = totalReading / activeSensors;
  } else
  {
    avgReading = 0;
  }
  lastReading = avgReading;
  totalReading = 0; activeSensors = 0;
}

// Make a turn, left = 0, right = 1
void turn(int x)
{
  // initiate turning
  if(x == 0)
  {
    digitalWrite(dir_a, HIGH);
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, maxMotorSpeed + 30);
    analogWrite(pwm_b, maxMotorSpeed + 30);
    turning = true;
  }
  else if(x == 1)
  {
    digitalWrite(dir_a, LOW);
    digitalWrite(dir_b, LOW);
    analogWrite(pwm_a, maxMotorSpeed + 30);
    analogWrite(pwm_b, maxMotorSpeed + 30);
    turning = true;
  }

  // Delay to make sure the PID sensors don't detect the previous line
  delay(250);
  
  // continue turning untill line has been detected
  while(turning == true)
  {
    readAllSensors();
    if (sensor1 || sensor2 || sensor3 || sensor4)
    {
      turning = false;
      straight();
    }
  }

  // Follow the line again
  followLine();
}

//Sets the motors go straight
void straight() 
{
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, LOW);
  analogWrite(pwm_a, maxMotorSpeed);
  analogWrite(pwm_b, maxMotorSpeed);
}

// Communicate with the host
int communicate(int intersection)
{
  if(intersection > 50)
  {
    return 4;
  }
  else
  {
    // send the intersection to the application
    
    // Stop the autobot and wait for instructions
    analogWrite(pwm_a, 0);
    analogWrite(pwm_b, 0);

    // wait for the application to send an instruction to move
    int message = receiveLogMessage();

    return 1;
  }
}

// wait for the message from the host 
int receiveLogMessage()
{
  int receivedMessage;
  // wait for the message back
  while(!myRadio.available())
  {
    delay(10);
  }
  
  // TODO test this shit
  bool ok = myRadio.read(&receivedMessage, sizeof(unsigned long));
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("Message Received: ");
  Serial.println(receivedMessage);
  
  sendLogMessage(receivedMessage);
  
  return receivedMessage;
}

//Send a message to the listener
void sendLogMessage(int messageId) 
{
  
  //Set the radio to send mode
  myRadio.stopListening();
  unsigned long id = messageId;

  //Check whether someone listened to the message
  bool ok = myRadio.write(&id, sizeof(unsigned long));
  if (ok) {
    Serial.println("Message sent:");
    Serial.println(messageId);
    
    analogWrite(pwm_a, 0);
    analogWrite(pwm_b, 0);
  }else{
    Serial.println("Message send failed");
  }
  //Set the radio back to receive mode
  myRadio.startListening();
}

// drive in the chosen direction
void drive(int chosenDirection)
{
  switch(chosenDirection)
  {
    case 0:
      // go straight ahead
      straight();
      break;
    case 1:
      // turn left
      turn(0);
      break;
    case 2:
      // turn right
      turn(1);
      break;
    case 3:
      // turn around 
      turn(0);
      break;
    case 4:
      // Follow the line
      followLine();
    default:
      break;
  }
}

void changeAutobotOrientation(int turn)
{
  switch(turn)
  {
    case 0:
    // no turn
      break;
    case 1:
    // turn right
      if(autobotOrientation < 3)
      {
         autobotOrientation += 1;
      }
      else
      {
        autobotOrientation = 0;
      }
      break;
    case 2:
    // turn left
      if(autobotOrientation > 0)
      {
        autobotOrientation -= 1;
      }
      else
      {
        autobotOrientation = 3;
      }
      break;
    case 3:
    // turn around
      if(autobotOrientation < 2)
      {
        autobotOrientation += 2;
      }
      else if(autobotOrientation == 2)
      {
        autobotOrientation = 0; 
      }
      else
      {
        autobotOrientation = 1;
      }
      break;
  }
}

void changeCoordinates(int turn)
{
  switch(turn)
  {
    case 0:
    // no turn
      switch(autobotOrientation)
      {
        case 0:
          y += 1;
        break;
        case 1:
          x += 1;
        break;
        case 2:
          y -= 1;
        break;
        case 3:
          x -= 1;
        break;
      }
    break;
    case 1:
      // turn right
      switch(autobotOrientation)
      {
        case 0:
          x += 1;
        break;
        case 1:
          y -= 1;
        break;
        case 2:
          x -= 1;
        break;
        case 3:
          y += 1;
        break;
      }
    break;
    case 2:
      // turn left
      switch(autobotOrientation)
      {
        case 0:
          x -= 1;
        break;
        case 1:
          y += 1;
        break;
        case 2:
          x += 1;
        break;
        case 3:
          y -= 1;
        break;
      }
    break;
    case 3:
      // turn around
      switch(autobotOrientation)
      {
        case 0:
          y -= 1;
        break;
        case 1:
          x -= 1;
        break;
        case 2:
          y += 1;
        break;
        case 3:
          x += 1;
        break;
      }
    break;  
  }
}

void loop() 
{
  // Read sensor data
  readAllSensors();
  // Start following the line
  //followLine();

  // temp
   waitForInfo = true;
   
  // check for an intersection
  if((sensor0 || sensor5 || (!sensor1 && !sensor2 && !sensor3 && !sensor4)) && moving)
  {
    analogWrite(pwm_a, 0);
    analogWrite(pwm_b, 0);
    moving = false;
    delay(1000);
  }
  else
  {
    followLine();
  }

//  if(millis() - intersectionTimer > 4000)
//  {
//    straightTile = true;
//    tile = 0;
//    sendLogMessage(200);
//    intersectionTimer = millis();
//  }
  

  // determine the intersection
  if(!moving)
  {
    readAllSensors();
    if(checkPIDSensors())
    {
      waitForInfo = true;
      if(sensor0 && sensor5)
      {
        messageID += 30;
        tile = 3;
        intersectionTimer = millis();
      }
      else if(sensor0)
      {
        messageID += 10;
        tile = 1;
        intersectionTimer = millis();
      }
      else if(sensor5)
      {
        messageID += 20;
        tile = 2;
        intersectionTimer = millis();
      }
    }
    else if(sensor0 && sensor5)
    {
      waitForInfo = true;
      messageID += 60;
      tile = 6;
      intersectionTimer = millis();
    }
    else if(sensor0)
    {
      waitForInfo = true;
      messageID += 40;
      tile = 4;
      intersectionTimer = millis();
    }
    else if(sensor5)
    {
      waitForInfo = true;
      messageID += 50;
      tile = 5;
      intersectionTimer = millis();
    }
    else
    {           
      waitForInfo = true;
      messageID += 70;
      tile = 7;
      intersectionTimer = millis();
      // there is no intersection
    }
    moving = true;
    // determine the direction
    if(messageID == 110 || messageID == 140 || messageID == 160 || messageID == 130)
    {
      messageID += 3;
      sendLogMessage(messageID);
      // turn left
      changeAutobotOrientation(2);
      changeCoordinates(2);
      turn(0);
    }
    else if(messageID == 120 || (messageID == 100 && straightTile == true))
    {
      messageID += 0;
      sendLogMessage(messageID);
      changeCoordinates(0);
      straight();
      delay(140);
    }
    else if(messageID == 150)
    {
      messageID += 1;
      sendLogMessage(messageID);
      // turn right
      changeAutobotOrientation(1);
      changeCoordinates(1);
      turn(1);
    }
    else if(messageID == 170)
    {
      messageID += 2;
      sendLogMessage(messageID);
      // turn around
      changeAutobotOrientation(3);
      changeCoordinates(3);
      turn(1);
    }

    // save the intersection
    if(arrayCounter <= 100)
    {
      tiles[arrayCounter][1][1] = x;
      tiles[arrayCounter][1][2] = y;
      tiles[arrayCounter][1][3] = tile;

      sendLogMessage(tiles[arrayCounter][1][1]);
      sendLogMessage(tiles[arrayCounter][1][2]);
      sendLogMessage(tiles[arrayCounter][1][3]);

      arrayCounter++;
    }    
    

    

    // reset the messageID
    if(autoBot == 1)
    {
      messageID = 100;
    }
    else if(autoBot == 2)
    {
      messageID = 200;
    }
  }

  if(waitForInfo)
  {
    receiveLogMessage();
  }
  
}
