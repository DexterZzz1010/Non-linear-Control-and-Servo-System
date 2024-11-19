/* File : compensator.c
 * Abstract:
 *
 */

#define S_FUNCTION_NAME compensator
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define U(element) (*uPtrs[element]) /* Pointer to Input Port0 */

static real_T absolut(real_T v)
{
  if (v<0) {
    return -1.0*v;
  }
  else {
    return v;
  }
}
/* Function: mdlInitializeSizes =================================
 */
static void mdlInitializeSizes(SimStruct *S)
{
  ssSetNumSFcnParams(S,3);
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
    return;
  }
  ssSetNumContStates(S,0);
  ssSetNumDiscStates(S,3);

  if (!ssSetNumInputPorts(S,1)) return;
  ssSetInputPortWidth(S,0,5);
  ssSetInputPortDirectFeedThrough(S,0,1);

  if (!ssSetNumOutputPorts(S,1)) return;
  ssSetOutputPortWidth(S,0,1);

  ssSetNumSampleTimes(S,1);
  ssSetNumRWork(S,2);
  ssSetNumIWork(S,1);
  ssSetNumPWork(S,0);
  ssSetNumModes(S,0);
  ssSetNumNonsampledZCs(S,0);

  ssSetOptions(S,SS_OPTION_EXCEPTION_FREE_CODE);
}

/* Function: mdlInitializeSampleTimes ===========================
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
  real_T *h = mxGetPr(ssGetSFcnParam(S,0));
  ssSetSampleTime(S,0, h[0]);
  ssSetOffsetTime(S,0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ============================
 */
static void mdlInitializeConditions(SimStruct *S)
{
  real_T *x = ssGetRealDiscStates(S);
  real_T *parbank = ssGetRWork(S);
  int_T  *mode = ssGetIWork(S);
  real_T *temp;

  temp = mxGetPr(ssGetSFcnParam(S,1));
  parbank[0] = temp[0];           /* thp */
  temp = mxGetPr(ssGetSFcnParam(S,2));
  parbank[1] = temp[0];           /* thn */
  mode[0] = 0;               /* mode is off=0 on=1 */
  
  x[0]=0.0;    /* y(k-1) */
  x[1]=0.0;    /* y(k-2) */
  x[2]=0.0;    /* last updown = 0*/
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
  InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
  real_T *y       = ssGetOutputPortRealSignal(S,0);
  real_T *x       = ssGetRealDiscStates(S);
  real_T *parbank = ssGetRWork(S);
  int_T  *mode    = ssGetIWork(S);
  real_T  t       = ssGetT(S); 
  real_T  v       = 0.0;
  real_T  yp      = U(0);
  /* y=U(0),pos=U(1),neg=U(2),offset=U(3),updown=U(4)*/

  yp = yp - U(3);

  if (U(4) != x[2]) {
    mode[0] = 1;    
  }

  if ((t > 0.1) & (mode[0] == 1)){
    if (yp > parbank[0]) {
      v = U(1);
    }
    else if (yp < parbank[1]) {
      v = U(2);
    }
    else {
      if (U(4) == 1.0) {
        v = U(2);
        if ((absolut(U(0)-x[0]) < absolut(x[0]-x[1])) | yp > 0) {
          mode[0] = 0;
          v = U(1);
	  /*printf("On my way up, I changed at y = "); 
            printf("%6.3f",U(0));
            printf(", t = "); 
            printf("%6.3f\n",t);*/
        }
      }
      else if (U(4) == -1.0) {
        v = U(1);
        if ((absolut(U(0)-x[0]) < absolut(x[0]-x[1])) | yp < 0) {
          mode[0] = 0;
          v = U(2);
	  /*printf("On my way down, I changed at y = "); 
            printf("%6.3f",U(0));
            printf(", t = "); 
            printf("%6.3f\n",t);*/
        }
      }
    }
  }
  else {
    if (U(4) == 1.0) {
      v = U(1);
    }
    else {
      v = U(2);
    }
  }
   
y[0]=v;

}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
  real_T  *x = ssGetRealDiscStates(S);
  InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
  x[1]=x[0];        /* y(k-1) */
  x[0]=U(0);        /* y(k)   */
  x[2]=U(4);
}

/* Function: mdlTerminate =============================
 */
static void mdlTerminate(SimStruct *S)
{
printf("--------------------------------------------\n");
}

#ifdef MATLAB_MEX_FILE  /*Is this file being compiled as a MEX-file? */
#include "simulink.c"   /*MEX-file interface mechanism */
#else
#include "cg_sfun.h"    /*Code generation registration function */
#endif
