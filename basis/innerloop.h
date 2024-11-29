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

void init_var_svm();
float PI_controller(float error, float kp, float ki, float output_min, float output_max, float KAW);
float innerloop(float i_ref, float kp, float ki, float output_min, float output_max, float KAW,float gamma,double *i_abc);