#include "PID.h"
#include <math.h>
#include <iostream>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID():has_prev_cte_(false), prev_cte_(0){}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd)
{
    
    this -> Kp = kp; // const pointer, addr will never change
    this -> Ki = ki;
    this -> Kd = kd;
    
    
    // initialize errors.
    this -> p_error = 0.0;
    this -> i_error = 0.0;
    this -> d_error = 0.0;
    
    // initialize dp
    dp = {0.1*Kp,0.1*Kd,0.1*Ki};
    
    // in case of a first iteration
    has_prev_cte_ = false;
    prev_cte_ = 0;
    
    // switch on/ff twiddle
    activate_twiddle = true;
    
    // step counter
    step = 1;
    // index counter
    param_index = 2;  // start from 0 after first twiddle loop
    
    // initial step count to calculate total error.
    n = 100;
    
    //Initialization
    total_error = 0;
    best_error = std::numeric_limits<double>::max();
    
}

// Update Error module
void PID::UpdateError(double cte)
{
    // build error update function, meanwhile to ensure correct initial d_error
    if (!has_prev_cte_) {
        prev_cte_ = cte;
        has_prev_cte_ = true;
    }

    this -> p_error = cte;
    this -> i_error += cte;
    this -> d_error = cte - prev_cte_;
    
    prev_cte_ = cte;
    
    if(step > n) total_error += pow(cte,2) ;
    
    TotalError();
    
    step++;

}

double PID::Error() {
    return total_error/(step - n);
}



double PID::GetValue() {
    return -1.0 * (Kp * p_error + Ki * i_error + Kd * d_error);
}

double PID::TotalError() {
//    cout << "  Total_error = " << total_error << endl;
     cout << "step: " << step << endl;
    //        cout << " step: " << step << endl;
        if(best_error > 1e9) {
            
            best_error = total_error;
            
            std::cout << "New best Kp: " << Kp << " Ki: " << Ki
            << " Kd: " << Kd << " Error: " << best_error << std::endl;
            cout<<'\n';

        }
        // if any improvement, then print out the best error and PID setup
        if (total_error < best_error && step > n) {
            
            best_error = total_error;
            // speed up the iternation step size to 1.1 if goes the right way.
            
            dp[param_index] *= 1.1;

            // next parameter index
            param_index = (param_index + 1) % 3;
            up = true;
            
            std::cout << "New best Kp: " << Kp << " Ki: " << Ki
            << " Kd: " << Kd << " Error: " << best_error;
            cout<<'\n';
        }
        
        else {
            std::cout << "Skipped worst Kp: " << Kp << " Ki: " << Ki
            << " Kd: " << Kd << " Error:  " << best_error;
            cout<<'\n';
            if(up == true) up = false;
            else{
            // or else slow down the step. update PID.
            AppendDeltaDp(param_index,dp[param_index]);
            dp[param_index] *= 0.9;
            // next parameter index
            param_index = (param_index + 1) % 3;
            up = true;
                
            }
        }
        
        
        // update PID with if
        if(up) {
            AppendDeltaDp(param_index,dp[param_index]);
            std::cout << "if up, then : " << Kp << " Ki: " << Ki
            << " Kd: " << Kd << " Error: " << best_error;
            cout<<'\n';
        }else{
            AppendDeltaDp(param_index,-2 * dp[param_index]);
            std::cout << "if no up, then : " << Kp << " Ki: " << Ki
            << " Kd: " << Kd << " Error: " << best_error;
            cout<<'\n';
        }


}

void PID::AppendDeltaDp(int index, double delta) {
    if (index == 0) {
        Kp += delta;
    }
    else if (index == 1) {
        Kd += delta;
    }
    else if (index == 2) {
        Ki += delta;
    }
    else {
        std::cout << "AppendDeltaDp: index out of bounds";
    }
}










