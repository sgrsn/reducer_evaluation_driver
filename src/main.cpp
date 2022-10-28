#include <SPI.h>         // Remember this line!
#include <DAC_MCP49xx.h>
#include <string>
#include <iomanip>
#include <ios>
#include <StateMachine.h>
#include <MsTimer2.h>
#include <QEI/QEI.hpp>

#include <mtl/mtl_ddmotor_driver.hpp>
#include <sdcardlogger/sdcardlogger.hpp>
#include <LPF/lowpass_filter.hpp>

const char exp_file_name[]  = "exp1028_20rpm_break10.csv";
const double ddmotor_speed = 20; //[rpm]
const double ddmotor_torque = ddmotor_speed / 200.0;
const double break_torque   = 1.0;
const double ramp_time = 3.0;
const double exp_time = 8.0;

double start_time = 0.0;

#define SS_PIN 9
#define TORQUE_PIN 27
#define RATE 300.0

SDcardLogger logger(exp_file_name);
MTL::Driver driver;
DAC_MCP49xx dac(DAC_MCP49xx::MCP4922, SS_PIN);
uint8_t buf[64] = {};


QEI encoder(34, 35, 2048);

LowPassFilter::Param lpf_param = {1.0, 1.0};
LowPassFilter torque_lpf(lpf_param, 1./RATE);

void InfoState();
void SetupState();
void WaitState();
void ControlState();
void StopState();
void SaveState();
bool transitionInfoToSetup();
bool transitionSetupToWait();
bool transitionWaitToControl();
bool transitionControlToStop();
bool transitionStopToSave();
bool transitionSaveToWait();

void setBreakTorque(double torque);
void setMotorTorque(double torque);

void TimerCallBack();

StateMachine machine = StateMachine();
State* info_state = machine.addState(&InfoState);
State* setup_state = machine.addState(&SetupState);
State* wait_state = machine.addState(&WaitState);
State* control_state = machine.addState(&ControlState);
State* stop_state = machine.addState(&StopState);
State* save_state = machine.addState(&SaveState);

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

  // State Machine Setup
  info_state    ->addTransition(&transitionInfoToSetup,   setup_state);
  setup_state   ->addTransition(&transitionSetupToWait,   wait_state);
  wait_state    ->addTransition(&transitionWaitToControl, control_state);
  control_state ->addTransition(&transitionControlToStop, stop_state);
  stop_state    ->addTransition(&transitionStopToSave,    save_state);
  save_state    ->addTransition(&transitionSaveToWait,    wait_state);

  // Timer Setup
  MsTimer2::set(1./RATE*1e3, TimerCallBack);
  MsTimer2::start();
}

void loop()
{
}

void InfoState(){
  if(machine.executeOnce)
  {
    Serial.println("Info State");
    Serial.println("Please set the sdcard");
  }
}

bool transitionInfoToSetup(){
  if(key_on)
  {
    key_on = false;
    return true;
  }
  return false;
}

void SetupState(){
  Serial.println("Setup State");
  if(machine.executeOnce)
  {
    // SDcard Setup
    logger.sdcard_setup();
    
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
    Serial.println("Please press any key to switch control ON/OFF");
    logger.dump2sdcard(std::vector<std::string>{"Time [ms]", "DDmotor Torque [Nm]", "Break Torque [Nm]", "Torque Sensor [Nm]", "DDmotor position [rad]", "Output position [rad]"});
  }
}

bool transitionSetupToWait(){
  return true;
}

void WaitState(){
  if(machine.executeOnce)
    Serial.println("Wait State");
  start_time = millis();
}

bool transitionWaitToControl(){
  if(key_on)
  {
    key_on = false;
    return true;
  }
  return false;
}

void ControlState(){
  if(machine.executeOnce)
  {
    Serial.println("Control State");
    start_time = millis();
  }
  int torque_in = analogRead(TORQUE_PIN);
  double torque_voltage = (torque_in * 3.3 / 1024 - 2.5) * 2.;
  double torque = torque_voltage * 20. / 5. - torque_offset;
  double filtered_torque = torque_lpf.update(torque);
  double pos = driver.pos_radians();
  double output_pos = encoder.getRadians();
  double dd_current_torque = ddmotor_torque * (double)(millis()-start_time)/1000. / ramp_time;
  double break_current_torque = break_torque * (double)(millis()-start_time)/1000. / ramp_time;
  if(dd_current_torque > ddmotor_torque) dd_current_torque = ddmotor_torque;
  if(break_current_torque > break_torque) break_current_torque = break_torque;
  setMotorTorque(dd_current_torque);
  setBreakTorque(break_current_torque);
  /*std::stringstream ss;
  ss.str("");
  ss << std::fixed;
  ss << std::setprecision(3);
  ss << std::showpos;
  ss << "torque:" << torque << " [Nm], ";
  ss << "pos:" << pos * RAD_TO_DEG << " [deg]";
  Serial.println(ss.str().c_str());*/
  Serial.print("DDMotor[Nm] : ");
  Serial.print(dd_current_torque);
  Serial.print(", Break[Nm] : ");
  Serial.print(break_current_torque);
  Serial.print(", Sensor[Nm] : ");
  Serial.print(filtered_torque);
  Serial.print(", Input[rad] : ");
  Serial.print(pos);
  Serial.print(", Output[rad] : ");
  Serial.print(output_pos);
  Serial.println("");

  logger.dump2sdcard(std::vector<double>{millis()-start_time, dd_current_torque, break_current_torque, filtered_torque, pos, output_pos});
}

bool transitionControlToStop(){
  if(millis() - start_time > exp_time*1e3 || key_on)
  {
    key_on = false;
    return true;
  }
  return false;
}

void StopState(){
  if(machine.executeOnce)
    Serial.println("Stop State");

  setMotorTorque(0.);
  setBreakTorque(0.);
}

bool transitionStopToSave(){
  if(key_on)
  {
    key_on = false;
    return true;
  }
  return false;
}

void SaveState(){
  Serial.println("Save State");
  if(machine.executeOnce)
  {
    logger.sdcard_close();
  }
}

bool transitionSaveToWait(){
  return true;
}

void TimerCallBack()
{
  machine.run();
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

