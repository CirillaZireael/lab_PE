/*--------------------------------------------------------------------+
Filename: s_abc_dq_transformation.c
By:       
----------------------------------------------------------------------+
Purpose:  C-mex s-function
          KOS-Transformationen 

----------------------------------------------------------------------+
 *               Channel        Value     [Dimension]     Erläuterung           
 * Inputs:        1             abc          [3]             
 *                2             gamma        [1]        Transformationswinkel (Park)       

Block parameter input:
 *               1              fsample      [1]         Abtastfrequenz
 *               2              xy         	 [2]         Beispiel
         

Output:         Channel        Value
 *              1               y_dq         [2]         Ausgangsvariable im dq-KOS
 *              2               debug        [3]         debug


Compile:    mex s_dq_svm_transformation.c //transformations.c
----------------------------------------------------------------------+
Revision history:::
Date:    ID:   Description:

---------------------------------------------------------------------*/


#define S_FUNCTION_NAME  s_dq_svm_transformation
#define S_FUNCTION_LEVEL 2


#define NO_OF_PARAMETERS 1
#define NO_OF_INPUTS 2
#define NO_OF_INPUTS_PORT0   2    // u (dq)
#define NO_OF_INPUTS_PORT1   1    // gamma
#define NO_OF_OUTPUTS 3
#define NO_OF_OUTPUTS_PORT0  1   // |u|
#define NO_OF_OUTPUTS_PORT1  1   // gamma
#define NO_OF_OUTPUTS_PORT2  1   // debug

// Blockparameter einlesen
#define fsample     *(mxGetPr(ssGetSFcnParam(S,0)))         // Abtastfrequenz



// Konstanten definieren
#define TWOPI       6.283185307179586
#define PIDIV2      1.570796326794897

// Macro for limitation of angle value 0->2pi


#include <stdio.h>
#include <math.h>
#include "simstruc.h"

// ----------------------- Eigene Funktionen einbinden ----------------------
//#include "transformations.h"

// ----------------------- Initialisierung  ----------------------
static void mdlInitializeSizes(SimStruct *S)
{
   ssSetNumSFcnParams(S, NO_OF_PARAMETERS);
   if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
   {
      return; /* Parameter mismatch will be reported by Simulink */
   }

   ssSetNumContStates(S, 0);
   ssSetNumDiscStates(S, 0);
   
   if (!ssSetNumInputPorts(S, NO_OF_INPUTS)) return;
   ssSetInputPortWidth(S, 0, NO_OF_INPUTS_PORT0);       // für jeden Input-Channel
   ssSetInputPortWidth(S, 1, NO_OF_INPUTS_PORT1);


   ssSetInputPortDirectFeedThrough(S, 0, 1);            // für jeden Input-Channel
   ssSetInputPortDirectFeedThrough(S, 1, 1); 

   
   if (!ssSetNumOutputPorts(S, NO_OF_OUTPUTS)) return;
   ssSetOutputPortWidth(S, 0, NO_OF_OUTPUTS_PORT0);     
   ssSetOutputPortWidth(S, 1, NO_OF_OUTPUTS_PORT1);     // für jeden Output-Channel
   ssSetOutputPortWidth(S, 2, NO_OF_OUTPUTS_PORT2);     // für jeden Output-Channel



   ssSetNumSampleTimes(S, 1);

                            /* Take care when specifying exception free code - see sfuntmpl.doc */
   ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);

   // user defined variables
}

// Diskrete Durchführungszeit definieren
static void mdlInitializeSampleTimes(SimStruct *S)
{
   // Sample time 
   ssSetSampleTime(S, 0, (1/(fsample)));
   ssSetOffsetTime(S, 0, 0*(1/(fsample))/2);
}


static void mdlOutputs(SimStruct *S, int_T tid)
{
   // Definition Ausgänge    // für jeden Input-Channel
   real_T  *MagRefOut       = ssGetOutputPortRealSignal(S,0);  //  output pointer, MagRef (dq)
   real_T  *PhaseRefOut     = ssGetOutputPortRealSignal(S,1);  //  output pointer, PhaseRef
   real_T  *debug           = ssGetOutputPortRealSignal(S,2);  //  output pointer, debug
   // Definition Eingänge   // für jeden Output-Channel
   InputRealPtrsType input      = ssGetInputPortRealSignalPtrs(S, 0); // input signal
   InputRealPtrsType gamma_in     = ssGetInputPortRealSignalPtrs(S, 1); // Transformation angle
 
   real_T dRef, qRef, gamma_dq,gamma_trans;
   real_T MagRef;
   real_T PhaseRef;
   
   dRef      = *input[0];
   qRef      = *input[1]; 
   gamma_trans  = *gamma_in[0];

   // magnitude
    MagRef = sqrt(dRef*dRef + qRef*qRef);
   // phase
    if (qRef == 0.0 && dRef == 0.0)
      gamma_dq = 0.0;
    else
      gamma_dq = atan2(qRef, dRef);

    PhaseRef = gamma_dq + gamma_trans ;//- PIDIV2; // -PIDIV2 aufgrund Flussorientierung!!;
    if((PhaseRef)>=TWOPI) (PhaseRef)-=TWOPI; 
    if((PhaseRef)<0) (PhaseRef)+=TWOPI;
    // Output
    MagRefOut[0]   = MagRef;       
    PhaseRefOut[0] = PhaseRef;   
    debug[0]       = gamma_trans;   
}


/* Function: mdlTerminate =================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif



