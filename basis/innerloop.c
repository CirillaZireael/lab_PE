/************************************/
/* C-Functionen, eingebunden in Corpus s_svm.c */
/************************************/

#include "innerloop.h"
#include "transformations.h"
// svm.h enhtält
//      - Constanten
//      - Einbindung weiterer Standardfunktionen (math.h...)
//      - Initialisierung der Funktionen


//  ----------------------- Variablen ------------------------------------
float output_last;
float error_last;
float Ta;
float integrator;



        
//  ----------------------------------------------------------------------

// Initialisierungen
// wird automatisch beim Simulationsstart aufgerufen.
// Parameter:
void init_var_svm()
{
    // alte Zustände zurücksetzen
 output_last = 0;
 error_last = 0;
    integrator = 0;
 Ta = 2.000000000000000e-04;
}

//  ----------------------------------------------------------------------

// Innerloop PID
// Paramter: - keine

float innerloop(float error, float kp, float ki, float output_min, float output_max, float KAW,float gamma) {

    abc_dq_trans(float in_a, float in_b, float in_c, float gamma, double *out_d, double *out_q)
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