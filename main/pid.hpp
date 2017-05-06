class PID{
private:
  double error;
  double error_prev;
  double error_prevprev;
  double output;
  double Kp;
  double Ki;
  double Kd;
public:
  PID();
  void setGain(double Kp, double Ki, double Kd);
  double update(double target, double state);
};
