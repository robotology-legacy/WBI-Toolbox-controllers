/* Include files */

#include "balancingController_with_OOT_and_transfer_sfun.h"
#include "balancingController_with_OOT_and_transfer_sfun_debug_macros.h"
#include "c2_balancingController_with_OOT_and_transfer.h"
#include "c3_balancingController_with_OOT_and_transfer.h"
#include "c5_balancingController_with_OOT_and_transfer.h"
#include "c6_balancingController_with_OOT_and_transfer.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _balancingController_with_OOT_and_transferMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void balancingController_with_OOT_and_transfer_initializer(void)
{
}

void balancingController_with_OOT_and_transfer_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_balancingController_with_OOT_and_transfer_method_dispatcher
  (SimStruct *simstructPtr, unsigned int chartFileNumber, const char* specsCksum,
   int_T method, void *data)
{
  if (chartFileNumber==2) {
    c2_balancingController_with_OOT_and_transfer_method_dispatcher(simstructPtr,
      method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_balancingController_with_OOT_and_transfer_method_dispatcher(simstructPtr,
      method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_balancingController_with_OOT_and_transfer_method_dispatcher(simstructPtr,
      method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_balancingController_with_OOT_and_transfer_method_dispatcher(simstructPtr,
      method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_balancingController_with_OOT_and_transfer_process_check_sum_call
  ( int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3855755302U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3838334479U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(589036524U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1834745803U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2959118837U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(499359450U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(4291527238U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4071532606U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 2:
        {
          extern void
            sf_c2_balancingController_with_OOT_and_transfer_get_check_sum
            (mxArray *plhs[]);
          sf_c2_balancingController_with_OOT_and_transfer_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void
            sf_c3_balancingController_with_OOT_and_transfer_get_check_sum
            (mxArray *plhs[]);
          sf_c3_balancingController_with_OOT_and_transfer_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void
            sf_c5_balancingController_with_OOT_and_transfer_get_check_sum
            (mxArray *plhs[]);
          sf_c5_balancingController_with_OOT_and_transfer_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void
            sf_c6_balancingController_with_OOT_and_transfer_get_check_sum
            (mxArray *plhs[]);
          sf_c6_balancingController_with_OOT_and_transfer_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2083502392U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1110276785U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3258378658U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3926592909U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3719355619U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4040529515U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3340472740U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2920653325U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_balancingController_with_OOT_and_transfer_autoinheritance_info
  ( int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(aiChksum, "lJ4v2pbwBS6ocEhT9kCQbC") == 0) {
          extern mxArray
            *sf_c2_balancingController_with_OOT_and_transfer_get_autoinheritance_info
            (void);
          plhs[0] =
            sf_c2_balancingController_with_OOT_and_transfer_get_autoinheritance_info
            ();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "1isKSFc9rt9KI2xKT5gzsH") == 0) {
          extern mxArray
            *sf_c3_balancingController_with_OOT_and_transfer_get_autoinheritance_info
            (void);
          plhs[0] =
            sf_c3_balancingController_with_OOT_and_transfer_get_autoinheritance_info
            ();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "lJ4v2pbwBS6ocEhT9kCQbC") == 0) {
          extern mxArray
            *sf_c5_balancingController_with_OOT_and_transfer_get_autoinheritance_info
            (void);
          plhs[0] =
            sf_c5_balancingController_with_OOT_and_transfer_get_autoinheritance_info
            ();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "I81QhFAIx7U3C8HFXFaJfC") == 0) {
          extern mxArray
            *sf_c6_balancingController_with_OOT_and_transfer_get_autoinheritance_info
            (void);
          plhs[0] =
            sf_c6_balancingController_with_OOT_and_transfer_get_autoinheritance_info
            ();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int
  sf_balancingController_with_OOT_and_transfer_get_eml_resolved_functions_info
  ( int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        extern const mxArray
          *sf_c2_balancingController_with_OOT_and_transfer_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_balancingController_with_OOT_and_transfer_get_eml_resolved_functions_info
          ();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_balancingController_with_OOT_and_transfer_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_balancingController_with_OOT_and_transfer_get_eml_resolved_functions_info
          ();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray
          *sf_c5_balancingController_with_OOT_and_transfer_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_balancingController_with_OOT_and_transfer_get_eml_resolved_functions_info
          ();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray
          *sf_c6_balancingController_with_OOT_and_transfer_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_balancingController_with_OOT_and_transfer_get_eml_resolved_functions_info
          ();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_balancingController_with_OOT_and_transfer_third_party_uses_info(
  int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(tpChksum, "Q6yNILM8fWRC6PLok6rzZG") == 0) {
          extern mxArray
            *sf_c2_balancingController_with_OOT_and_transfer_third_party_uses_info
            (void);
          plhs[0] =
            sf_c2_balancingController_with_OOT_and_transfer_third_party_uses_info
            ();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "rNY3h87hfTnSJi5jlMTvTC") == 0) {
          extern mxArray
            *sf_c3_balancingController_with_OOT_and_transfer_third_party_uses_info
            (void);
          plhs[0] =
            sf_c3_balancingController_with_OOT_and_transfer_third_party_uses_info
            ();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "Q6yNILM8fWRC6PLok6rzZG") == 0) {
          extern mxArray
            *sf_c5_balancingController_with_OOT_and_transfer_third_party_uses_info
            (void);
          plhs[0] =
            sf_c5_balancingController_with_OOT_and_transfer_third_party_uses_info
            ();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "l1Satau1vW4iggK5GgWejG") == 0) {
          extern mxArray
            *sf_c6_balancingController_with_OOT_and_transfer_third_party_uses_info
            (void);
          plhs[0] =
            sf_c6_balancingController_with_OOT_and_transfer_third_party_uses_info
            ();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int
  sf_balancingController_with_OOT_and_transfer_updateBuildInfo_args_info( int
  nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(tpChksum, "Q6yNILM8fWRC6PLok6rzZG") == 0) {
          extern mxArray
            *sf_c2_balancingController_with_OOT_and_transfer_updateBuildInfo_args_info
            (void);
          plhs[0] =
            sf_c2_balancingController_with_OOT_and_transfer_updateBuildInfo_args_info
            ();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "rNY3h87hfTnSJi5jlMTvTC") == 0) {
          extern mxArray
            *sf_c3_balancingController_with_OOT_and_transfer_updateBuildInfo_args_info
            (void);
          plhs[0] =
            sf_c3_balancingController_with_OOT_and_transfer_updateBuildInfo_args_info
            ();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "Q6yNILM8fWRC6PLok6rzZG") == 0) {
          extern mxArray
            *sf_c5_balancingController_with_OOT_and_transfer_updateBuildInfo_args_info
            (void);
          plhs[0] =
            sf_c5_balancingController_with_OOT_and_transfer_updateBuildInfo_args_info
            ();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "l1Satau1vW4iggK5GgWejG") == 0) {
          extern mxArray
            *sf_c6_balancingController_with_OOT_and_transfer_updateBuildInfo_args_info
            (void);
          plhs[0] =
            sf_c6_balancingController_with_OOT_and_transfer_updateBuildInfo_args_info
            ();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void balancingController_with_OOT_and_transfer_debug_initialize(struct
  SfDebugInstanceStruct* debugInstance)
{
  _balancingController_with_OOT_and_transferMachineNumber_ =
    sf_debug_initialize_machine(debugInstance,
    "balancingController_with_OOT_and_transfer","sfun",0,4,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _balancingController_with_OOT_and_transferMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _balancingController_with_OOT_and_transferMachineNumber_,0);
}

void balancingController_with_OOT_and_transfer_register_exported_symbols
  (SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_balancingController_with_OOT_and_transfer_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "balancingController_with_OOT_and_transfer",
      "balancingController_with_OOT_and_transfer");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_balancingController_with_OOT_and_transfer_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
