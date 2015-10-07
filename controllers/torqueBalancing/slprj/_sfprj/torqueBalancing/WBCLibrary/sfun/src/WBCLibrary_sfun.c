/* Include files */

#include "WBCLibrary_sfun.h"
#include "WBCLibrary_sfun_debug_macros.h"
#include "c1_CbqhCBbjbXK2C4fod3g2mC_WBCLibrary.h"
#include "c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _WBCLibraryMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void WBCLibrary_initializer(void)
{
}

void WBCLibrary_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_WBCLibrary_method_dispatcher(SimStruct *simstructPtr, unsigned
  int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==1) {
    if (!strcmp(specsCksum, "CbqhCBbjbXK2C4fod3g2mC")) {
      c1_CbqhCBbjbXK2C4fod3g2mC_WBCLibrary_method_dispatcher(simstructPtr,
        method, data);
      return 1;
    }

    if (!strcmp(specsCksum, "K609Rm7ptn8v3rQ4xprp8")) {
      c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_method_dispatcher(simstructPtr, method,
        data);
      return 1;
    }

    return 0;
  }

  return 0;
}

unsigned int sf_WBCLibrary_process_check_sum_call( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
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
  if (nrhs>2 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"library")) {
      char machineName[100];
      mxGetString(prhs[2], machineName,sizeof(machineName)/sizeof(char));
      machineName[(sizeof(machineName)/sizeof(char)-1)] = '\0';
      if (!strcmp(machineName,"WBCLibrary")) {
        if (nrhs==3) {
          ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(805223063U);
          ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(571377494U);
          ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2440468359U);
          ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1147421542U);
        } else if (nrhs==4) {
          unsigned int chartFileNumber;
          chartFileNumber = (unsigned int)mxGetScalar(prhs[3]);
          switch (chartFileNumber) {
           case 1:
            {
              extern void sf_c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_get_check_sum
                (mxArray *plhs[]);
              sf_c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_get_check_sum(plhs);
              break;
            }

           default:
            ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
            ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
            ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
            ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
          }
        } else {
          return 0;
        }
      } else {
        return 0;
      }
    } else {
      return 0;
    }
  } else {
    return 0;
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_WBCLibrary_autoinheritance_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
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
     case 1:
      {
        if (strcmp(aiChksum, "xkDb8k5eqMP7aXB5oUFLxD") == 0) {
          extern mxArray
            *sf_c1_CbqhCBbjbXK2C4fod3g2mC_WBCLibrary_get_autoinheritance_info
            (void);
          plhs[0] =
            sf_c1_CbqhCBbjbXK2C4fod3g2mC_WBCLibrary_get_autoinheritance_info();
          break;
        }

        if (strcmp(aiChksum, "001U3moJZgYiwaWFxBpiHB") == 0) {
          extern mxArray
            *sf_c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_get_autoinheritance_info
            (void);
          plhs[0] =
            sf_c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_get_autoinheritance_info();
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

unsigned int sf_WBCLibrary_get_eml_resolved_functions_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
     case 1:
      {
        extern const mxArray
          *sf_c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_get_eml_resolved_functions_info
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

unsigned int sf_WBCLibrary_third_party_uses_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
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
     case 1:
      {
        if (strcmp(tpChksum, "CbqhCBbjbXK2C4fod3g2mC") == 0) {
          extern mxArray
            *sf_c1_CbqhCBbjbXK2C4fod3g2mC_WBCLibrary_third_party_uses_info(void);
          plhs[0] =
            sf_c1_CbqhCBbjbXK2C4fod3g2mC_WBCLibrary_third_party_uses_info();
          break;
        }

        if (strcmp(tpChksum, "K609Rm7ptn8v3rQ4xprp8") == 0) {
          extern mxArray
            *sf_c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_third_party_uses_info(void);
          plhs[0] = sf_c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_third_party_uses_info
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

unsigned int sf_WBCLibrary_updateBuildInfo_args_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
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
     case 1:
      {
        if (strcmp(tpChksum, "CbqhCBbjbXK2C4fod3g2mC") == 0) {
          extern mxArray
            *sf_c1_CbqhCBbjbXK2C4fod3g2mC_WBCLibrary_updateBuildInfo_args_info
            (void);
          plhs[0] =
            sf_c1_CbqhCBbjbXK2C4fod3g2mC_WBCLibrary_updateBuildInfo_args_info();
          break;
        }

        if (strcmp(tpChksum, "K609Rm7ptn8v3rQ4xprp8") == 0) {
          extern mxArray
            *sf_c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_updateBuildInfo_args_info
            (void);
          plhs[0] =
            sf_c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void WBCLibrary_debug_initialize(struct SfDebugInstanceStruct* debugInstance)
{
  _WBCLibraryMachineNumber_ = sf_debug_initialize_machine(debugInstance,
    "WBCLibrary","sfun",1,1,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,_WBCLibraryMachineNumber_,
    0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,_WBCLibraryMachineNumber_,0);
}

void WBCLibrary_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_WBCLibrary_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info("WBCLibrary",
      "torqueBalancing");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_WBCLibrary_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
