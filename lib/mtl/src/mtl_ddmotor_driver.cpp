#include "mtl/mtl_ddmotor_driver.hpp"

namespace MTL
{

Driver::Driver()
{

}

void Driver::decode(uint8_t* buf, int length)
{
  int32_t pos_data = 0;
  for(int i = 0; i < 6; i++)
  {
    if(((buf[i] >> 6) & 0b11) == no_data[i])
      pos_data |= ((int32_t)buf[i] & 0b00111111) << (6*i);
  }
  // checksum
  uint8_t checksum = 0;
  for(int i = 0; i < 6; i++)
  {
    checksum += buf[i] & (0b00111111);
  }
  checksum &= (0b00111111);
  checksum |= no_data[6] << 6;
  if(buf[6] == checksum)
  {
    pos_data_ = pos_data;
  }
  else{
    Serial.print(pos_data);
    Serial.print(", ");
    Serial.print(checksum);
    Serial.print(", ");
    Serial.print(buf[6]);
    Serial.println(", error");
  }
}

} // namespace MTL