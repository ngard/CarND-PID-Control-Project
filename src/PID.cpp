#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  p_error = 0; i_error = 0; d_error = 0;
  this->Kp = Kp; this->Ki = Ki; this->Kd = Kd;
  cte_integral = 0; cte_prev = 0;
}

double PID::CalcOutput(double cte)
{
  double delta_cte = cte - cte_prev;
  cte_integral += cte;
  double output = - Kp * cte - Kd * delta_cte - Ki * cte_integral;
  cte_prev = cte;
  return output;
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}

