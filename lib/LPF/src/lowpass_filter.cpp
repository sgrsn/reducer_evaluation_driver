#include <LPF/lowpass_filter.hpp>

#include <cmath>
#include <stdexcept>
#include <sstream>

LowPassFilter::LowPassFilter(const Param& param, const double& time_step) : param_(param)
{
  arg_.resize(2,0);
  arg_[0] = (param.gain * M_PI * param.cut_off_freq * time_step)/(1. + M_PI * param.cut_off_freq * time_step);
  arg_[1] = (1 - M_PI * param.cut_off_freq * time_step)/(1. + M_PI * param.cut_off_freq * time_step);
  output_.resize(2,0.0);
  input_.resize(2,0.0);
}

const double& LowPassFilter::update(const double& input)
{
  input_[0] = input;
  output_[0] = arg_[0]*(input_[0] + input_[1]) + arg_[1]*output_[1];     
  input_[1] = input_[0];
  output_[1] = output_[0];
  return output_[0];
}