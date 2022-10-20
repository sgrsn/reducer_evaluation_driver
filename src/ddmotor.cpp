#include <mtl/mtl_ddmotor_driver.hpp>

MTL::Driver driver;
uint8_t buf[64];

void setup()
{
  Serial.begin(9600);
  Serial2.begin(115200);
}

void loop()
{
  delay(100);
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
  int32_t pos_data = driver.pos();
  double pos = (double)pos_data / 2097152. * 2.*M_PI;
  Serial.println(pos * RAD_TO_DEG);
}


void serialEvent2()
{
  if(Serial2.available() > 0)
  {
    Serial2.readBytes(buf, 64);
  }
}