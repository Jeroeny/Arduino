#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

unsigned long optimus = 1;
unsigned long bumblebee = 69;

RF24 optimusRadio (9, 10);
RF24 bumblebeeRadio (9, 10);

const uint64_t pipes[4] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xF0F0F0F0A1LL, 0xF0F0F0F0A2LL };

bool test1 = false;
bool test2 = false;

void setup()
{
  Serial.begin(9600);

  optimusRadio.begin();
  optimusRadio.openReadingPipe(1, pipes[1]);
  optimusRadio.stopListening();
  optimusRadio.startListening();

  bumblebeeRadio.begin();
  bumblebeeRadio.openReadingPipe(3, pipes[2]);
  bumblebeeRadio.stopListening();
  bumblebeeRadio.startListening();

}

void loop(void)
{
  while(test1)
  {
    delay(5000);
    Serial.println(120);
    delay(500);
    Serial.println(213);
    delay(1000);
    Serial.println(120);
    delay(500);
    Serial.println(261);
    delay(1000);
    Serial.println(120);
    delay(500);
    Serial.println(272);
    delay(1000);
    Serial.println(172);
    delay(500);
    Serial.println(210);
    delay(1000);
    Serial.println(113);
    delay(500);
    Serial.println(213);
    delay(1000);
    Serial.println(163);
    delay(500);
    Serial.println(210);
    delay(1000);
    Serial.println(151);
    delay(500);
    Serial.println(251);
    delay(1000);
    Serial.println(120);
    delay(500);
    Serial.println(263);
    delay(1000);
    Serial.println(172);
    delay(500);
    Serial.println(261);
    delay(1000);
    Serial.println(113);
    delay(500);
    Serial.println(251);
    delay(1000);
    Serial.println(143);
    delay(500);
    Serial.println(272);
    delay(1000);
    Serial.println(151);
    delay(500);
    Serial.println(243);
    delay(1000);
    Serial.println(100);
    delay(500);
    Serial.println(210);
    delay(1000);
    Serial.println(120);
    delay(500);
    Serial.println(200);
    delay(1000);
    Serial.println(151);
    delay(500);
    Serial.println(243);
    delay(1000);
    Serial.println(172);
    delay(500);
    Serial.println(251);
    delay(1000);
    Serial.println(143);
    delay(500);
    Serial.println(261);
    delay(1000);
    Serial.println(113);
    delay(500);
    Serial.println(272);
    delay(1000);
    Serial.println(121);
    delay(500);
    Serial.println(210);
    delay(1000);
    Serial.println(143);
    delay(500);
    Serial.println(243);
    delay(1000);
    Serial.println(120);
    delay(500);
    Serial.println(221);
    delay(1000);
    Serial.println(163);
    delay(500);
    Serial.println(263);
    delay(1000);
    Serial.println(110);
    delay(500);
    Serial.println(210);
    delay(1000);
    /*Serial.println(143);
    delay(500);
    Serial.println(210);
    delay(1000);
    Serial.println(143);
    delay(500);
    Serial.println(243);
    delay(1000);
    Serial.println(163);
    delay(500);
    Serial.println(243);
    delay(500);
    Serial.println(261);*/
    test1 = false;
  }
  while(test2)
  {
    delay(5000);
    Serial.println(272);
    delay(500);
    Serial.println(100);
    delay(500);
    Serial.println(221);
    delay(500);
    Serial.println(110);
    delay(500);
    Serial.println(272);
    delay(500);
    Serial.println(110);
    delay(500);
    Serial.println(261);
    delay(500);
    Serial.println(172);
    delay(500);
    Serial.println(221);
    delay(500);
    Serial.println(121);
    delay(500);
    Serial.println(200);
    delay(500);
    Serial.println(172);
    delay(500);
    Serial.println(163);
    delay(500);
    Serial.println(200);
    delay(500);
    Serial.println(243);
    delay(500);
    Serial.println(251);
    delay(500);
    Serial.println(230);
    delay(500);
    Serial.println(200);
    test2 = false;
  }
  listenOptimus();
  listenBumblebee();
  listenApplication();
}

void listenOptimus()
{
  if (optimusRadio.available())
  {
    unsigned long message;
    bool ok = optimusRadio.read(&message, sizeof(unsigned long));
    Serial.println("Received from optimus:");
    Serial.println(message);
//    if (ok)
//    {
//      if (message != optimus)
//      {
        optimusRadio.stopListening();
        optimus = message;
        optimusRadio.startListening();
//      }
//    }
  }
}

void listenBumblebee()
{
  if (bumblebeeRadio.available())
  {
    unsigned long message;
    bool ok = bumblebeeRadio.read(&message, sizeof(unsigned long));
    Serial.println("Received from bumblebee:");
    Serial.println(message);
    //if (ok)
    //{
      //if (message != bumblebee)
      //{
       
        bumblebeeRadio.stopListening();
        bumblebee = message;
        bumblebeeRadio.startListening();
      //}
    //}
  }
}

void listenApplication()
{
  String message;
  while(Serial.available())
  {
    char c = Serial.read();
    message += c;
  }
  sendData(message);
}
//test
void sendData(String message)
{
  char robot = message[0];
  String data = message;
  
  int msgi = data.toInt();

  //Serial.println(bytes);
  
  bool sent = false;
  switch(robot)
  {
    case '1':
      Serial.println("Sending to optimus:" + data);
      //Serial.println(msgi);
      optimusRadio.stopListening();
      optimusRadio.openWritingPipe(pipes[0]);
      sent = optimusRadio.write(&msgi, sizeof(char));
      optimusRadio.startListening();
      if(sent){
        Serial.println("Sent message.");
      }else{
        Serial.println("Failed to send.");
      }
      break;
    case '2':
      Serial.println("Sending to bumblebee:" + data);
      //Serial.println(msgi);
      bumblebeeRadio.stopListening();
      bumblebeeRadio.openWritingPipe(pipes[1]);
      sent = bumblebeeRadio.write(&msgi, sizeof(char));
      bumblebeeRadio.startListening();
      if(sent){
        Serial.println("Sent message.");
      }else{
        Serial.println("Failed to send.");
      }
      break;
    default:
      int i = 0;
      while(i < 5)
      {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        delay(1000);
        i++;
      }
  }
}
