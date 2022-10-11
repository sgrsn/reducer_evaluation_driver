#include <QEI/QEI.hpp>
#include <Arduino.h>
 
QEI::QEI(uint8_t A, uint8_t B, int ppr) : channelA(A), channelB(B), channelZ(255)
{
  _ppr = ppr;
  init();
}
 
QEI::QEI(uint8_t A, uint8_t B, uint8_t Z, int ppr) : channelA(A), channelB(B), channelZ(Z)
{
  _ppr = ppr;
  init();
  attachInterrupt (channelZ, isr0, CHANGE); 
}

// ISR glue routines
void QEI::isr0()
{
  instance0_->encode(); 
}  // end of Foo::isr0

// for use by ISR glue routines
QEI * QEI::instance0_;

void QEI::init()
{
  attachInterrupt (channelA, isr0, CHANGE); 
  attachInterrupt (channelB, isr0, CHANGE);
  instance0_ = this;
  currState = 0;
  prevState = 0;
  position = 0;
  _IsInterrupt = false;
}
 
float QEI::getDegree()
{
    return float(position) * 360.0/(_ppr*4.0);
}
 
bool QEI::IsInterruptZ()
{
    bool tmp = _IsInterrupt;
    _IsInterrupt = false;
    return tmp;
}
 
void QEI::encode(void)
{
    int8_t chanA  = digitalRead(channelA);
    int8_t chanB  = digitalRead(channelB);
    currState = chanA | (chanB << 1);
    
    if (prevState != currState) {
        position += encodeTable[currState | (prevState<<2)];
        prevState = currState;
    }
}
 
void QEI::encodeZ()
{
    _IsInterrupt = true;
}