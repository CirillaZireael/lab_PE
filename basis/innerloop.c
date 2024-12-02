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

float Ta;
double ASM_p; 
double ASM_Rr;
double ASM_Lh;
double ASM_Lr;
double ASM_sigma;
double ASM_Ls;


PI_ControllerState id_ControllerState;
PI_ControllerState iq_ControllerState;


void init_var(float fsa);
void abc_dq_trans(float in_a, float in_b, float in_c, float gamma, double *out_d, double *out_q);
void decoupling(double I_d,double I_q,double Psi_Rd,double omega_M,double* U_Sd_ent, double* U_Sq_ent);
        
//  ----------------------------------------------------------------------

// Initialization
// called in s_innerloop.c
// To initialize the parameters:
void init_var_svm()
{
    // alte Zustände zurücksetzen

 Ta = 2.000000000000000e-04; //this is discretization system and this is the period
     ASM_p = 1.0;
     ASM_Rr = 1.6;
     ASM_Lh = 0.4040;
     ASM_Lr = 0.4131;
     ASM_sigma = 0.0433;
     ASM_Ls = 0.4131;
    init_var(1/Ta);
}

//  ----------------------------------------------------------------------

// Innerloop controller

float innerloop(double *idq_ref, float kp, float ki, 
                float *v_output, float u_max, float KAW,float gamma,
                double *i_abc,double Psi_Rd,double n_M) {

    double i_d_fdb, i_q_fdb;
    abc_dq_trans(i_abc[0], i_abc[1], i_abc[2],gamma, &i_d_fdb, &i_q_fdb); //park transformation

    double U_Sd_ent, U_Sq_ent;
    double omega_M = n_M*2*PI/60;
    decoupling(idq_ref[0],idq_ref[1],Psi_Rd,omega_M,&U_Sd_ent,&U_Sq_ent);

    double error = idq_ref[0] - i_d_fdb;
    v_output[0] =  PI_controller(error, kp, ki, -u_max, u_max, KAW, &id_ControllerState,U_Sd_ent);
    error = idq_ref[1] - i_q_fdb;
    v_output[1] =  PI_controller(error, kp, ki, -u_max, u_max, KAW, &iq_ControllerState,U_Sq_ent);

    //PI controller for q axis to be added, 
    //may need struct to distinguish the integrator,error_last,output_last value of d and q axis 
    return 0;

}

// PI Controller function with anti-windup

float PI_controller(float error, float kp, float ki, float output_min, float output_max, float KAW,PI_ControllerState *state, double U_decoupling) {

    float output = 0;

    // Compute the proportional term

    float proportional = kp * error;

    // Compute the integral term

    state->integrator += ki * Ta * error;

    // Combine proportional and integral terms

    output = proportional + state->integrator;


    //----------decoupling-------------

    output += U_decoupling;

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

void decoupling(double I_d,double I_q,double Psi_Rd,double omega_M,double* U_Sd_ent, double* U_Sq_ent){
     *U_Sd_ent = ASM_sigma * ASM_Ls * 
                (-ASM_p * omega_M * I_q - (ASM_Rr * ASM_Lh) / ASM_Lr / Psi_Rd * pow(I_q, 2)) 
                - Psi_Rd * ASM_Rr * ASM_Lh / pow(ASM_Lr, 2);

    *U_Sq_ent = ASM_sigma * ASM_Ls * 
                (-ASM_p * omega_M * I_d + (ASM_Rr * ASM_Lh) / ASM_Lr / Psi_Rd * I_q * I_d) 
                - ASM_p * omega_M * Psi_Rd * ASM_Lh / ASM_Lr;
}