/************************************/
/* C functions, included in corpus s_innerloop.c*/
/************************************/

#include "outerloop.h"
#include "transformations.h" //for park transformation
// innerloop.h includes
//      - constants
//      - inclusion of other libraries
//      - Function Declarations


//  ----------------------- Variable ------------------------------------
float PI_AW_controller(float error, float kp, float ki, float output_min, float output_max, float KAW,PI_ControllerState *state);

float Ta;


PI_ControllerState flux_ControllerState;
PI_ControllerState speed_ControllerState;

        
//  ----------------------------------------------------------------------

// Initialization
// called in s_innerloop.c
// To initialize the parameters:
void init_var_outerloop()
{
    // alte Zustände zurücksetzen

 Ta = 2.000000000000000e-04; //this is discretization system and this is the period

}

//  ----------------------------------------------------------------------

// Innerloop controller

float outerloop(double Psi_Rn,double Psi_R,
                float kp_spd, float ki_spd, float KAW_spd,
                float kp_flux, float ki_flux, float KAW_flux,
                float i_max,double *idq_ref,
                double n_ref,double n_M) {


    double error = Psi_Rn - Psi_R;
    idq_ref[0] =  PI_AW_controller(error, kp_flux, ki_flux, -i_max, i_max, KAW_flux, &flux_ControllerState);
    error = n_ref - n_M;
    idq_ref[1] =  PI_AW_controller(error, kp_spd, ki_flux, -i_max, i_max, KAW_spd, &speed_ControllerState);

    //PI controller for q axis to be added, 
    //may need struct to distinguish the integrator,error_last,output_last value of d and q axis 
    return 0;

}

// PI Controller function with anti-windup

float PI_AW_controller(float error, float kp, float ki, float output_min, float output_max, float KAW,PI_ControllerState *state) {

    float output = 0;

    // Compute the proportional term

    float proportional = kp * error;

    // Compute the integral term

    state->integrator += ki * Ta * error;

    // Combine proportional and integral terms

    output = proportional + state->integrator;

    // 
    // //----------decoupling-------------
    // 
    // output += U_decoupling;

    // -------Apply output limits------------

    float output_limited = output;

    if (output > output_max) {

        output_limited = output_max;

    } else if (output < output_min) {

        output_limited = output_min;

    }

    // Anti-windup correction

    float feedback_correction = KAW * (output - output_limited);

    state->integrator -= feedback_correction * Ta;  // Adjust the integrator to prevent windup

    // Update states

    state->output_last = output_limited;

    state->error_last = error;

    return output_limited;

}
