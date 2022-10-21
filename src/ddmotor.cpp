#include <SPI.h>         // Remember this line!
#include <DAC_MCP49xx.h>

#include <mtl/mtl_ddmotor_driver.hpp>

#define SS_PIN 9
#define TORQUE_PIN 27

MTL::Driver driver;
DAC_MCP49xx dac(DAC_MCP49xx::MCP4922, SS_PIN);
uint8_t buf[64];

double torque_offset = 0;

void setup()
{
  Serial.begin(9600);
  Serial2.begin(115200);

  // DAC Setup
  dac.init(&SPI);
  SPI.begin();
  dac.outputA(0);
  dac.outputB(0);
  delay(1000);
  for(int i = 0; i < 1000; i++)
  {
    int torque_in = analogRead(TORQUE_PIN);
    double torque_voltage = (torque_in * 3.3 / 1024 - 2.5) * 2.;
    torque_offset += torque_voltage * 20. / 5.;
    delay(1);
  }
  torque_offset /= 1000;

  dac.outputA(2000);
  dac.outputB(2000);
}

void loop()
{
  int torque_in = analogRead(TORQUE_PIN);
  double torque_voltage = (torque_in * 3.3 / 1024 - 2.5) * 2.;
  double torque = torque_voltage * 20. / 5. - torque_offset;
  Serial.print(torque);
  Serial.print(", ");
  
  double pos = driver.pos_radians();
  Serial.println(pos * RAD_TO_DEG);
  delay(1);
}


void serialEvent2()
{
  if(Serial2.available() > 0)
  {
    Serial2.readBytes(buf, 8);
    uint8_t data[7] = {};
    for(int i = 1; i < 64-7; i++)
    {
      if(buf[i-1] == 0x00 && buf[i] >> 6 == 0x00)
      {
        for(int s = 0; s < 7; s++)
        {
          data[s] = buf[i+s];
        }
        break;
      }
    }
    driver.decode(data, 7);
  }
}