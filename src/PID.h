#ifndef PID_H
#define PID_H
#include <vector>

class PID {
public:
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
    

  std::vector<double> dp;
  int step;
  int n;
  int param_index;
    // number of steps to allow changes to settle, then to evaluate error
  int n_settle_steps;
  int n_eval_steps;
  double total_error;
  double best_error;

  bool activate_twiddle;
  bool has_prev_cte_;
  bool up;
  double prev_cte_;
    
    
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
  void Init(double kp, double ki, double kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);
    
  double GetValue();
    
  double Error();
    
  double TotalError();
    
  void AppendDeltaDp(int index, double delta);

  /*
  * Calculate the total PID error.
  */
};

#endif /* PID_H */
