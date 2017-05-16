#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

unsigned long optimus = 1;
unsigned long bumblebee = 69;

RF24 optimusRadio (9, 10);
RF24 bumblebeeRadio (9, 10);

const uint64_t pipes[4] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xF0F0F0F0A1LL, 0xF0F0F0F0A2LL };

void setup()
{
  Serial.begin(9600);
  
  optimusRadio.begin();
  optimusRadio.openReadingPipe(1, pipes[1]);
  optimusRadio.stopListening();
  optimusRadio.startListening();

  bumblebeeRadio.begin();
  bumblebeeRadio.openReadingPipe(3, pipes[3]);
  bumblebeeRadio.stopListening();
  bumblebeeRadio.startListening();
  
}

void loop(void)
{
  listenOptimus();
  listenBumblebee();
}

void listenOptimus()
{
  if (optimusRadio.available())
  {
    unsigned long message;
    bool ok = optimusRadio.read(&message, sizeof(unsigned long));
    if (ok)
    {
      if (message != optimus)
      {
        Serial.println(message);
        optimusRadio.stopListening();
        optimus = message;
        optimusRadio.startListening();
      }
    }
  }
}

void listenBumblebee()
{
  if (bumblebeeRadio.available())
  {
    unsigned long message;
    bool ok = bumblebeeRadio.read(&message, sizeof(unsigned long));
    if (ok)
    {
      if (message != bumblebee)
      {
        Serial.println(message);
        bumblebeeRadio.stopListening();
        bumblebee = message;
        bumblebeeRadio.startListening();
      }
    }
  }
}
