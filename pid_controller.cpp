#include "pid_controller.h"

/**
* Contructor function for the PID object. Sets the kp, ki, kd
* values as well as the min/max output limits.
* @param kp_val The P tuning parameter.
* @param ki_val The I tuning parameter.
* @param kd_val The D tuning parameter.
* @param min The minimum output limit.
* @param max The maximum output limit.
*/
PID::PID(double kp_val, double ki_val, double kd_val, double min, double max)
{
  set_tunings(kp_val, ki_val, kd_val);
  set_output_limits(min, max);
}


/**
* compute calculates the error based on goal position and current position.
* Using the change in error and the change in time since the last iteration,
* it approximates the integral and derivative under the error curve, and then
* calculates the output based on the tuning values. The function will then save 
* the current values to be used for the next iteration.
* This function uses the ptime microsec_clock::universal_time() object from the
* boost posix_time library.
* @param set_point The goal position
* @param current_point The current position
* @returns The output values.
* Outputs a value between the min and max output limit.
*/
double PID::compute(double set_point, double current_point)
{
  double current_time = boost::posix_time::ptime microsec_clock::universal_time();
  double error = current_point - set_point;
  double delta_time = current_time - previous_time;

  //Calculate the area under the error curve
  double error_integral = error * delta_time;

  //Calculate the derivative (change in error / change in time)
  double delta_error = (error - previous_error) / delta_time;

  //Calculate PID output based on tunings
  double output = kp * error + ki * error_integral + kd * delta_error;

  //Make sure output is within the min/max limits
  output = check_limits(output);

  //Save current values to be used in the calculations for next iteration
  previous_output = output;
  previous_error = error;
  previous_time = current_time;

return output;
}

/**
* set_output_limits sets the limitations to the output value.
* @param min The minimum output required.
* @param max The maximum output required.
*/
void PID::set_output_limits(double min, double max)
{
  min_output = min;
  max_output = max;
}

/**
* set_tunings sets the tuning parameters.
* @param kp_val The P tuning parameter.
* @param ki_val The I tuning parameter.
* @param kd_val The D tuning parameter.
*/
void PID::set_tunings(double kp_val, double ki_val, double kd_val)
{
  kp = kp_val;
  ki = ki_val;
  kd = kd_val;
}

/**
* check_limits takes in the calculated outputs and makes sure that it
* is within the min/max output limit. If the proposed output is greater
* than max, it will set output as max. If the proposed output is less
* than min, it will set output as min. Otherwise output stays the same.
* @param output The raw output value before the check.
* @returns An output value within the limited bounds.
*/
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

/**
* get_kp
* @returns The kp tuning parameter.
*/
double PID::get_kp()
{
  return kp;
}

/**
* get_ki
* @returns The ki tuning parameter.
*/
double PID::get_ki()
{
  return ki;
}

/**
* get_kd
* @returns The kd tuning parameter.
*/
double PID::get_kd()
{
  return kd;
}