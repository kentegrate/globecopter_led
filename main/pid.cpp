#include "pid.hpp"

PID::PID(){
  error = 0;
  error_prev = 0;
  error_prevprev = 0;
  output = 0;
}

void PID::setGain(double Kp, double Ki, double Kd){
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}
double PID::update(double target, double state){
  error_prevprev = error_prev;
  error_prev = error;
  error = state-target;
  output += Kp*(error-error_prev) +
    Ki*error +
    Kd*((error-error_prev)-(error_prev-error_prevprev));
  return output;
}
