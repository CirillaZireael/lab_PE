/************************************/
/* C functions, included in corpus s_innerloop.c*/
/************************************/

#include "innerloop.h"
#include "transformations.h" //for park transformation
// innerloop.h includes
//      - constants
//      - inclusion of other libraries
//      - Function Declarations


//  ----------------------- Variable ------------------------------------
float output_last;
float error_last;
float Ta;
float integrator;


void init_var(float fsa);
void abc_dq_trans(float in_a, float in_b, float in_c, float gamma, double *out_d, double *out_q);
        
//  ----------------------------------------------------------------------

// Initialization
// called in s_innerloop.c
// To initialize the parameters:
void init_var_svm()
{
    // alte Zustände zurücksetzen
 output_last = 0;
 error_last = 0;
    integrator = 0;
 Ta = 2.000000000000000e-04; //this is discretization system and this is the period
    init_var(1/Ta);
}

//  ----------------------------------------------------------------------

// Innerloop PID

float innerloop(float i_ref, float kp, float ki, float output_min, float output_max, float KAW,float gamma,double *i_abc) {

     double i_d_fdb, i_q_fdb;
    abc_dq_trans(i_abc[0], i_abc[1], i_abc[2],gamma, &i_d_fdb, &i_q_fdb);
    double error = i_ref - i_d_fdb;
    float output = PI_controller(error, kp, ki, output_min, output_max, KAW);

    return output;

}

// PI Controller function with anti-windup

float PI_controller(float error, float kp, float ki, float output_min, float output_max, float KAW) {

    float output = 0;

    // Compute the proportional term

    float proportional = kp * error;

    // Compute the integral term

    integrator += ki * Ta * error;

    // Combine proportional and integral terms

    output = proportional + integrator;

    // Apply output limits

    float output_limited = output;

    if (output > output_max) {

        output_limited = output_max;

    } else if (output < output_min) {

        output_limited = output_min;

    }

    // Anti-windup correction

    float feedback_correction = KAW * (output - output_limited);

    integrator -= feedback_correction * Ta;  // Adjust the integrator to prevent windup

    // Update states

    output_last = output_limited;

    error_last = error;

    return output_limited;

}

float  in_alpha, in_beta;       	// Zustände:      Induzierte Spannung Us-RsIs
float   cosGamma,sinGamma;          // Variable für Berechnung Cos / Sin (gamma)
//  ----------------------------------------------------------------------

// Initialisierungen
// wird automatisch beim Simulationsstart aufgerufen.
// Parameter:
void init_var( float fsa)
{
    // alte Zustände zurücksetzen
    in_alpha      = 0;
    in_beta       = 0;
   
}

//  ----------------------------------------------------------------------

// abc_dq_transformation
// Paramter: - keine

void abc_dq_trans(float in_a, float in_b, float in_c, float gamma, double *out_d, double *out_q ){
   
      // calc Alpha- Beta-Coponents (Step 1)
           in_alpha = 0.666666666666667*(in_a - 0.5*(in_b+in_c));
           in_beta  = 0.577350269189626*(in_b-in_c);
      // calc dq-Coponents (Step 2)
           cosGamma = cos(gamma);
           sinGamma = sin(gamma);
           *out_d = in_alpha * cosGamma + in_beta  * sinGamma;
           *out_q = in_beta  * cosGamma - in_alpha * sinGamma; 
}
