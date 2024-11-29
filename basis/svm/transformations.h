// HEADER FILE
#include <math.h>
#define TWOTHIRD   0.666666666666667
#define SQRT3RECIP 0.577350269189626
// ----------------------- Prototypen -------------------------------------

// Initialisierungen
// wird automatisch beim Simulationsstart aufgerufen
// Parameter:
void init_var(float fsa);
//
//
// Wird mit der in der Simulink-Maske eingestellten Frequenz aufgerufen
// Paramter:
void abc_dq_trans(float in_a, float in_b, float in_c, float gamma, double *out_d, double *out_q);
