/*--------------------------------------------------------------------+
Filename: s_svm.c
By:       
----------------------------------------------------------------------+
Purpose:  C-mex s-function
          Implementation of SVM (Space Vector Modulation)

----------------------------------------------------------------------+
Channel        Value       [Dimension]    Description           
----------------------------------------------------------------------
Inputs:  
   1           id_ref         [1]         Reference current for d axis   
   2           i_abc          [3]         Feedback current in abc frame
   3           gamma          [1]         Angle of reference space vector [0, 2*PI]

Block parameter input:
   1           f_takt         [1]         Clock frequency
   2           f_sw           [1]         Switching frequency
         
Output:  
   1           pulse1         [1]         Control signal Phase 1
   2           pulse2         [1]         Control signal Phase 2
   3           pulse3         [1]         Control signal Phase 3    


Compile:    mex s_innerloop.c
----------------------------------------------------------------------+
Revision history:::
Date:    ID:   Description:

---------------------------------------------------------------------*/


#define S_FUNCTION_NAME  s_innerloop
#define S_FUNCTION_LEVEL 2


#define NO_OF_PARAMETERS     6
#define NO_OF_INPUTS         3
#define NO_OF_INPUTS_PORT0   1    // id_ref refrence current for d axis 
#define NO_OF_INPUTS_PORT1   3    // i_abc feedback current in abc frame
#define NO_OF_INPUTS_PORT2   1    //gamma Angle of reference space vector [0, 2*PI]
#define NO_OF_OUTPUTS        3
#define NO_OF_OUTPUTS_PORT0  1   // pulse phase 1
#define NO_OF_OUTPUTS_PORT1  1   // pulse phase 2
#define NO_OF_OUTPUTS_PORT2  1   // pulse phase 3

// Blockparameter einlesen

#define f_takt     *(mxGetPr(ssGetSFcnParam(S,0)))         //  Clock frequency
#define f_sw       *(mxGetPr(ssGetSFcnParam(S,1)))         // Switching frequency
#define cc_kp       *(mxGetPr(ssGetSFcnParam(S,2)))         // parameter Kp for current loop
#define cc_kaw       *(mxGetPr(ssGetSFcnParam(S,3)))         // parameter Kaw for antiwindup
#define u_max       *(mxGetPr(ssGetSFcnParam(S,4)))         // voltage limit
#define cc_ki       *(mxGetPr(ssGetSFcnParam(S,5)))         // parameter Ki for current loop

// Konstanten definieren

#include <stdio.h>
#include <math.h>
#include "simstruc.h"

// ----------------------- Eigene Funktionen einbinden ----------------------
#include "innerloop.h"

// ----------------------- Initialisierung  ----------------------
static void mdlInitializeSizes(SimStruct *S)
{
    // Specifies the number of parameters:
    ssSetNumSFcnParams(S, NO_OF_PARAMETERS);
    // Comparison with number of block dialog parameters:
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
        {
        return; // Parameter mismatch will be reported by Simulink
        }
    

   ssSetNumContStates(S, 0);
   ssSetNumDiscStates(S, 0);
   
   // Specifies the number of input ports:
    if (!ssSetNumInputPorts(S, NO_OF_INPUTS)) return;
    // Specifies the width of the input ports:
    ssSetInputPortWidth(S, 0, NO_OF_INPUTS_PORT0);
    ssSetInputPortWidth(S, 1, NO_OF_INPUTS_PORT1);
    ssSetInputPortWidth(S, 2, NO_OF_INPUTS_PORT2);


   // Specifies the direct feedthrough status of the ports (in this case
    // default status because not specified):
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    
    
   if (!ssSetNumOutputPorts(S, NO_OF_OUTPUTS)) return;
   ssSetOutputPortWidth(S, 0, NO_OF_OUTPUTS_PORT0);     
   ssSetOutputPortWidth(S, 1, NO_OF_OUTPUTS_PORT1); 
   ssSetOutputPortWidth(S, 2, NO_OF_OUTPUTS_PORT2);  // f�r jeden Output-Channel



   // Specifies the number of sample times:
    ssSetNumSampleTimes(S, 1);

    // Specifies S-function options (in this case performance improvement
    // exception-free S-functions):
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);

   // user defined variables
   init_var_svm((f_takt), (f_sw));
}

// Define Discrete Execution Time
static void mdlInitializeSampleTimes(SimStruct *S)
{
   // Sample time 
   ssSetSampleTime(S, 0, (1/(f_takt)));
   ssSetOffsetTime(S, 0, 0*(1/(f_takt))/2);
}


static void mdlOutputs(SimStruct *S, int_T tid)
{
   // Definition output    
   real_T  *pulse1    = ssGetOutputPortRealSignal(S,0);  //  Phase 1
   real_T  *pulse2    = ssGetOutputPortRealSignal(S,1);  //  Phase 2
   real_T  *pulse3    = ssGetOutputPortRealSignal(S,2);  //  Phase 3
   // Definition input   
   InputRealPtrsType id_ref          = ssGetInputPortRealSignalPtrs(S, 0); // id_ref refrence current for d axis 
   InputRealPtrsType i_abc =  ssGetInputPortRealSignalPtrs(S, 1); // current in abc frame
   InputRealPtrsType gamma         = ssGetInputPortRealSignalPtrs(S, 2); // Angle of reference space vector [0, 2*PI]
 
   real_T out_svm[3] = {0,0,0};
   
   //real_T Is_B = *Is_AB1[1];
   
//=========================================================================================
   //innnerloop output - d axis voltage  
   out_svm[0] = innerloop(*id_ref[0],cc_kp,cc_ki,-u_max,u_max,cc_kaw,*gamma[0],i_abc);

//=========================================================================================


   pulse1[0]   = out_svm[0];      // output
   pulse2[0]   = out_svm[1];
   pulse3[0]   = out_svm[2];
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



