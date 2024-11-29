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
void init_var_svm(float f_takt1, float f_sw1);
//
//
// Wird mit der in der Simulink-Maske eingestellten Frequenz aufgerufen
// Paramter:
void svm(float u_dc, float u_soll_betrag, float gamma, double *out_svm);
