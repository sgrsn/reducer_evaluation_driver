//
// Example for the MCP49x2 *dual* DACs
// For the single MCP49x1 series, see the other bundled example sketch.
//
#include <SPI.h>         // Remember this line!
#include <DAC_MCP49xx.h>

// The Arduino pin used for the slave select / chip select
#define SS_PIN 7

DAC_MCP49xx dac(DAC_MCP49xx::MCP4922, SS_PIN/*, LDAC_PIN*/);

void setup() {
  dac.init(&SPI);
  SPI.begin();
}

void loop() {
  for(int i = 0; i < 4000; i++)
  {
    dac.outputA(i);
    dac.outputB(i);
    delayMicroseconds(100);
  }
}