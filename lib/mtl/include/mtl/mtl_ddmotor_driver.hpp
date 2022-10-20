#ifndef MTL_DDMOTOR_DRIVER_HPP
#define MTL_DDMOTOR_DRIVER_HPP

#include <Arduino.h>

namespace MTL
{

const uint8_t no_data[7] = {0b00, 0b10, 0b10, 0b10, 0b10, 0b10, 0b01};

class Driver {

public:
  Driver();
  void decode(uint8_t* buf, int length);
  const int32_t& pos(void) const { return pos_data_; };

private:
  int32_t pos_data_;
};

} // namespace MTL

#endif