/************************************************************************
* C++ PID Library   - Version 1.0
* Written by: Kevin Huo
* Special Thanks To: Toni Ogunmade
************************************************************************/
#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <cmath>
#include <cstdlib>
#include "boost/date_time/posix_time/posix_time.hpp"

class PID{
  public:
    PID();
    double compute(double set_point, double current_point);
    void set_kp(double kp_val);
    void set_ki(double ki_val);
    void set_kd(double kd_val);
    double get_kp();
    double get_ki();
    double get_kd();

  private:
    double check_limits(double output);
    double kp = 0.0, ki = 0.0, kd = 0.0, 
    previous_output = 0.0, previous_error = 0.0, previous_time = 0.0
    set_point = 0.0, current_point = 0.0
    min_output = 0.0, max_output = 0.0;
};

#endif