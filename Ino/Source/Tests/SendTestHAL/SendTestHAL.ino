#include "RF24.h"
#include "printf.h"

RF24 myRadio(9, 10); //Radio CE and CSN pin. Refer to the Pin Layout document to see which pins these are on the radio unit.
const uint64_t pipes[2] = { 0xF0F0F0F0A1LL, 0xF0F0F0F0A2LL }; // READ, WRITE

void setup() 
{
  myRadio.begin();
  myRadio.openWritingPipe(pipes[1]);
  myRadio.openReadingPipe(1, pipes[0]);
  myRadio.startListening();
}

void loop() 
{
  for(int i = 70; i < 80; i++)
  {
    sendLogMessage(i);
    delay(1000);
  }
}

void sendLogMessage(int message)
{
  myRadio.stopListening();
  unsigned long id = message;
  myRadio.write(&id, sizeof(unsigned long));
  myRadio.startListening();  
}
