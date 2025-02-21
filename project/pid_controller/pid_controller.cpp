/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   Kp = Kpi;
   Ki = Kii;
   Kd = Kdi;

   p_err = 0.0;
   i_err = 0.0;
   d_err = 0.0;

   out_max = output_lim_maxi;
   out_min = output_lim_mini;

   dt = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/

   // Integral error calculation
   i_err = i_err + (cte*dt);

   // Derivative calculation - time derivative
   if(dt > 0)
   {
      // the simulated delta time from Carla is valid since it is bigger than zero
      // time shift on k and do [k]-[k-1], ideally, it 
      // should be [k+1]-[k] which is not feasible
      d_error = (cte - p_err)/dt;
   }
   else
   {
      // else, there was no valid delta time
      d_error = 0.0;
   }

   // Proportinal error calculation
   // update the proportional after the derivative term to have the k and k-1 errors
   p_err = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;

   // calculate the PID terms
   control = Kp*p_err + Ki*i_err + Kd*d_err;

   // saturate the output accordingly to their limit
   control = max(min(control, out_max), out_min);

   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   dt = new_delta_time;
   return dt;
}