#ifndef QEI_H
#define QEI_H

#include <Arduino.h>

const int8_t encodeTable[] = {0, -1,  1,  0, 1,  0,  0, -1, -1,  0,  0,  1, 0,  1, -1,  0 };

class QEI {

public:
    
  QEI(uint8_t A, uint8_t B, int ppr);
  QEI(uint8_t A, uint8_t B, uint8_t Z, int ppr);
  static void isr0 ();
  static QEI * instance0_;
    
  float getDegree();
  bool IsInterruptZ();
 
private:
 
  void init();
  void encode(void);
  void encodeZ();
  
  uint8_t channelA;
  uint8_t channelB;
  uint8_t channelZ;
  
  float angle;
  volatile int currState;
  int prevState;
  int position;
  float _ppr;
  
  bool _IsInterrupt;
};
 
#endif