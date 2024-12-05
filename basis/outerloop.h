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

typedef struct {
    float integrator;     // Integral term
    float output_last;    // Last output
    float error_last;     // Last error (if needed for derivative terms in full PID)
} PI_ControllerState;

void init_var_outerloop();
float outerloop(double Psi_Rn,double Psi_R,
                float kp_spd, float ki_spd, float KAW_spd,
                float kp_flux, float ki_flux, float KAW_flux,
                float i_max,double *idq_ref,
                double n_ref,double n_M);