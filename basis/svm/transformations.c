/************************************/
/* C-Functionen, eingebunden in Corpus s_transformations.c */
/************************************/

// transformations.h enht�lt
//      - Constanten
//      - Einbindung weiterer Standardfunktionen (math.h...)
//      - Initialisierung der Funktionen


//  ----------------------- Variablen ------------------------------------
float  in_alpha, in_beta;       	// Zust�nde:      Induzierte Spannung Us-RsIs
float   cosGamma,sinGamma;          // Variable f�r Berechnung Cos / Sin (gamma)
//  ----------------------------------------------------------------------

// Initialisierungen
// wird automatisch beim Simulationsstart aufgerufen.
// Parameter:
void init_var( float fsa)
{
    // alte Zust�nde zur�cksetzen
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



