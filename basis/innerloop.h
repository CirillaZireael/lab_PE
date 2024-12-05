// HEADER FILE
#include <math.h>
#define TWOTHIRD   0.666666666666667
#define SQRT3RECIP 0.577350269189626
#define ZPI        6.283185307179586476925286766559
#define PI         3.1415926535897932384626433832795
// ----------------------- Prototypen -------------------------------------

// Initialisierungen
// wird automatisch beim Simulationsstart aufgerufen
// Parameter:

float output_last;
float error_last;
float Ta;

typedef struct {
    float integrator;     // Integral term
    float output_last;    // Last output
    float error_last;     // Last error (if needed for derivative terms in full PID)
    float delta_output_last;
} PI_ControllerState;

void init_var_svm();
float PI_controller(float error, float kp, float ki, float output_min, float output_max, float KAW,PI_ControllerState *state, double U_decoupling);
float innerloop(double *idq_ref, float kp, float ki, float *v_output, float output_max, float KAW,float gamma,double *i_abc,double Psi_Rd,double n_M);
float outerloop(double Psi_Rn,double Psi_R,
                float imax,double *idq_ref,
                double n_ref,double n_M);