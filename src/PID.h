#ifndef PID_H
#define PID_H

class PID {
public:
  enum KtoTune {
		Initial,
		TuneP,
		TuneI,
		TuneD
  };

  enum Trial {
	      First,
	      Second
  };

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * State
  */
  KtoTune k;
  Trial trial;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, KtoTune k_to_tune = Initial, Trial trial = First);

  /*
  * Calculate PID values and output given cross track error.
  */
  double CalcOutput(double cte);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Return the number of timestep after Init().
  */
  unsigned TimeCount();

private:
  // integral of CTE
  double cte_integral;
  // CTE of previous timestep
  double cte_prev;

  // number of timesteps after Init()
  unsigned count_timestep;
  // average of CTE after Init()
  double cte_average;
};

#endif /* PID_H */
