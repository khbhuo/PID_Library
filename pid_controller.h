/**
* C++ PID Library
* Purpose: A general purpose C++ PID library
* @version 1.0 03/31/2016
* @author Kevin Huo
*/

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <cstdlib>
#include "boost/date_time/posix_time/posix_time.hpp"

class PID{
  public:
    PID(double kp_val, double ki_val, double kd_val, double min, double max);
    double calculateOutput(double set_point, double current_point);
    double setOutputLimits(double min, double max);
    double setTunings(double kp_val, double ki_val, double kd_val);
    double getKp();
    double getKi();
    double getKd();

  private:
    double checkLimits(double output);
    double kp = 0.0, ki = 0.0, kd = 0.0, previous_output = 0.0, 
        previous_error = 0.0, previous_time = 0.0, min_output = 0.0,
        max_output = 0.0;
};

#endif