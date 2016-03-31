#include "pid_controller.h"

PID::PID(kp_val, ki_val, kd_val, min, max)
{
  set_tunings(kp_val, ki_val, kd_val);
  set_output_limits(double min, double max);
}

double PID::compute(double set_point, double current_point)
{
  double current_time = boost::posix_time::ptime microsec_clock::universal_time();
  double error = current_point - set_point;
  double delta_time = current_time - previous_time;
  //Reiman sum
  double error_integral = error * (current_time - previous_time);
  //for derivative
  double delta_error = (error - previous_error) / delta_time;
  //calculate PID output
  double output = kp * error + ki * error_integral + kd * delta_error;

  output = check_limits(output);

  previous_output = output;
  previous_error = error;
  previous_time = current_time;

return output;
}

void PID::set_output_limits(double min, double max)
{
  min_output = min;
  max_output = max;
}

void PID::set_tunings(double kp_val, double ki_val, double kd_val)
{
  kp = kp_val;
  ki = ki_val;
  kd = kd_val;
}

double PID::check_limits(double output)
{
  if(output >= max)
  {
  	return max;
  }
  else if(output <= min)
  {
  	return min;
  }
  else
  {
  	return output;
  }
}

double PID::get_kp()
{
  return kp;
}

double PID::get_ki()
{
  return ki;
}

double PID::get_kd()
{
  return kd;
}