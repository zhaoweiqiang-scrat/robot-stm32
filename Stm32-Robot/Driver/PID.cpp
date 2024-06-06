#include "PID.h"

PID::PID(float min_val, float max_val, float kp, float ki, float kd)
{
  min_val_ = min_val;
  max_val_ = max_val;
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

double PID::compute(float setpoint, float measured_value)
{
  double error=0.0;
  //static double pid;
	
	
  //setpoint is constrained between min and max to prevent pid from having too much error
  error = setpoint - measured_value;
  integral_ += error;
  //derivative_ = error - prev_error_;
  derivative_ = error - last_error_;
  if(setpoint == 0 && error == 0){
    integral_ = 0;
  }

  //pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
	pid += (kp_ * (error-last_error_)) + (ki_ * error) + (kd_ * (error-2*last_error_+prev_error_));
 // prev_error_ = error;
	prev_error_ = last_error_;
	last_error_ = error;
	
  //printf("setpoint=:%f\r\n",setpoint);
  //printf("measured_value=:%f\r\n",measured_value);
  //printf("pid=:%f\r\n",pid);

  return constrain(pid, min_val_, max_val_);
}

void PID::updateConstants(float kp, float ki, float kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
