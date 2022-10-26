#ifndef LOWPASS_FILTER_HPP
#define LOWPASS_FILTER_HPP

#include <vector>

class LowPassFilter
{
 public:
  struct Param{
    double cut_off_freq{100.0};
    double gain{1.0};
  };

  LowPassFilter(const Param& param, const double& time_step=1.0);

  const double& update(const double& input);
  const double& update(const double& input, const double& time_step);

  void param(const Param& param);
  const Param& param(void) const { return param_; };
  
 private:
  std::vector<double> input_;
  std::vector<double> output_;
  double time_step_;
  Param param_;
  std::vector<double> arg_;
};

#endif // LOWPASS_FILTER_HPP