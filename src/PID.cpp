#include "PID.h"
#include <cmath>
#include <iostream>
#include <iomanip>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, KtoTune k_to_tune, Trial trial) {
  std::cerr << "Init() Kp:" << Kp << " Ki:" << Ki << " Kd:" << Kd << " KtoTune:" << k_to_tune << " Trial:" << trial<< std::endl;
  p_error = 0; i_error = 0; d_error = 0;
  this->Kp = Kp; this->Ki = Ki; this->Kd = Kd;
  k = k_to_tune;
  this->trial = trial;
  cte_integral = 0; cte_prev = 0;
  count_timestep = 0;
  cte_average = 0;
}

double PID::CalcOutput(double cte)
{
  double delta_cte = cte - cte_prev;
  cte_integral += cte;
  double out_p = - Kp * cte, out_d = - Kd * delta_cte, out_i = - Ki * cte_integral;
  double output = out_p + out_d + out_i;
  std::cerr << std::fixed << std::showpos
	    << "output:" << output << " P:" << out_p << " D:" << out_d << " I:" << out_i
	    << " cte:" << cte << " delta_cte:" << delta_cte << " cte_integral:" << cte_integral
	    << " error:" << cte_average << " count:" << count_timestep
	    << "\r" << std::flush;
  cte_prev = cte;
  return output;
}

void PID::UpdateError(double cte) {
  double cte2 = cte*cte;
  cte_average = (cte_average*count_timestep + cte2)/(count_timestep+1);
  ++count_timestep;
}

double PID::TotalError() {
  return cte_average;
}

unsigned PID::TimeCount() {
  return count_timestep;
}
