#include <SPI.h>         // Remember this line!
#include <DAC_MCP49xx.h>
#include <string>
#include <iomanip>
#include <ios>
#include <MsTimer2.h>
#include <QEI/QEI.hpp>

#include <mtl/mtl_ddmotor_driver.hpp>
#include <sdcardlogger/sdcardlogger.hpp>
#include <LPF/lowpass_filter.hpp>

const double ddmotor_speed = 20; //[rpm]
const double ddmotor_torque = ddmotor_speed / 200.0;
const double break_torque   = 1.00;

#define SS_PIN 9
#define TORQUE_PIN 27
#define RATE 100.0

MTL::Driver driver;
DAC_MCP49xx dac(DAC_MCP49xx::MCP4922, SS_PIN);
uint8_t buf[64] = {};

QEI encoder(34, 35, 2048);

LowPassFilter::Param lpf_param = {1.0, 1.0};
LowPassFilter torque_lpf(lpf_param, 1./RATE);

void setBreakTorque(double torque);
void setMotorTorque(double torque);

void TimerCallBack();

char console_input = '\0';
bool key_on = false;
double torque_offset = 0;

void setup()
{
  // Serial0(To laptop) Setup
  Serial.begin(9600);

  // Serial2(To DDmotor encoder) Setup
  Serial2.begin(115200);

  // DAC Setup
  dac.init(&SPI);
  SPI.begin();
  setBreakTorque(0.);
  setMotorTorque(0.);
  
  Serial.print("please wait");
  // Torque sensor ADC 
  for(int i = 0; i < 1000; i++)
  {
    int torque_in = analogRead(TORQUE_PIN);
    double torque_voltage = (torque_in * 3.3 / 1024 - 2.5) * 2.;
    torque_offset += torque_voltage * 20. / 5.;
    delay(1);
    if(i%100 == 0) Serial.print(".");
  }
  torque_offset /= 1000;
  Serial.println("ok");
  Serial.println("Setup has been completed.");

  // Timer Setup
  MsTimer2::set(1./RATE*1e3, TimerCallBack);
  MsTimer2::start();
}

void loop()
{
}


void TimerCallBack()
{
  int torque_in = analogRead(TORQUE_PIN);
  double torque_voltage = (torque_in * 3.3 / 1024 - 2.5) * 2.;
  double torque = torque_voltage * 20. / 5. - torque_offset;
  double filtered_torque = torque_lpf.update(torque);
  double pos = driver.pos_radians();
  double output_pos = encoder.getRadians();
  setMotorTorque(ddmotor_torque);
  setBreakTorque(break_torque);
  Serial.print("DDMotor[Nm] : ");
  Serial.print(ddmotor_torque);
  Serial.print(", Break[Nm] : ");
  Serial.print(break_torque);
  Serial.print(", Sensor[Nm] : ");
  Serial.print(filtered_torque);
  Serial.print(", Input[rad] : ");
  Serial.print(pos);
  Serial.print(", Output[rad] : ");
  Serial.print(output_pos, 5);
  Serial.println("");
}

void setBreakTorque(double torque)
{
  double torque_max = 1.0;
  if(torque > torque_max) torque = torque_max;
  else if(torque < 0) torque = 0.0;
  dac.outputA((unsigned short)(torque * 4095));
}

void setMotorTorque(double torque)
{
  double torque_max = 1.0;
  if(torque > torque_max) torque = torque_max;
  else if(torque < 0) torque = 0;
  dac.outputB((unsigned short)(torque * 4095));
  /*[Nm] = 4.713[A]  * 0.3[Nm/A]/sqrt(2)) */
  //5[V] =  4.713[A]  * 0.3[Nm/A]/sqrt(2)) * 5[V/Nm]
  //100[%] =  4.713[A]  * 0.3[Nm/A]/sqrt(2)) * 5[V/Nm] * 100/5[/V]

}

void serialEvent()
{
  if(Serial.available() > 0)
  {
    console_input = Serial.read();
    Serial.println(console_input);
    key_on = true;
  }
}

void serialEvent2()
{
  static bool next_first_byte = false;
  static bool read_8bytes_enable = false;
  if(Serial2.available() > 0)
  {
    if(read_8bytes_enable)
    {
      Serial2.readBytes(buf, 8);
      if(buf[0] != 0x00)
      {
        read_8bytes_enable = false;
        next_first_byte = false;
      }
      uint8_t data[7] = {};
      for(int i = 0; i < 7; i++) data[i] = buf[i+1];
      driver.decode(data, 7);
    }
    else{
      // 1バイトずつ取得
      Serial2.readBytes(buf, 1);
      if(buf[0] == 0x00)
      {
        next_first_byte = true;
        return;
      }
      if(next_first_byte && buf[0] >> 6 == 0x00)
      {
        // 残り6バイトは捨てる
        Serial2.readBytes(buf, 6);
        next_first_byte = false;
        read_8bytes_enable = true;
      }
      else
      {
        next_first_byte = false;
      }
    }
  }
  
}

