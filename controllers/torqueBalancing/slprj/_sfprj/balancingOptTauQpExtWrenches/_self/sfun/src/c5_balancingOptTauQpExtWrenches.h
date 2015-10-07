#ifndef __c5_balancingOptTauQpExtWrenches_h__
#define __c5_balancingOptTauQpExtWrenches_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef struct_struct_IJeSM2PPiABtzGLybE5MPF_tag
#define struct_struct_IJeSM2PPiABtzGLybE5MPF_tag

struct struct_IJeSM2PPiABtzGLybE5MPF_tag
{
  real_T pinvTol;
  real_T pinvDamp;
  real_T HessianQP;
};

#endif                                 /*struct_struct_IJeSM2PPiABtzGLybE5MPF_tag*/

#ifndef typedef_c5_struct_IJeSM2PPiABtzGLybE5MPF
#define typedef_c5_struct_IJeSM2PPiABtzGLybE5MPF

typedef struct struct_IJeSM2PPiABtzGLybE5MPF_tag
  c5_struct_IJeSM2PPiABtzGLybE5MPF;

#endif                                 /*typedef_c5_struct_IJeSM2PPiABtzGLybE5MPF*/

#ifndef struct_struct_IHUdSPrvFcumP3hkkkz03F_tag
#define struct_struct_IHUdSPrvFcumP3hkkkz03F_tag

struct struct_IHUdSPrvFcumP3hkkkz03F_tag
{
  real_T qTildeMax;
  real_T PCOM[9];
  real_T ICOM[9];
  real_T DCOM[9];
  real_T PAngularMomentum;
  real_T integral[23];
  real_T impedances[23];
  real_T dampings[23];
  real_T increasingRatesImp[23];
  real_T footSize[4];
};

#endif                                 /*struct_struct_IHUdSPrvFcumP3hkkkz03F_tag*/

#ifndef typedef_c5_struct_IHUdSPrvFcumP3hkkkz03F
#define typedef_c5_struct_IHUdSPrvFcumP3hkkkz03F

typedef struct struct_IHUdSPrvFcumP3hkkkz03F_tag
  c5_struct_IHUdSPrvFcumP3hkkkz03F;

#endif                                 /*typedef_c5_struct_IHUdSPrvFcumP3hkkkz03F*/

#ifndef typedef_SFc5_balancingOptTauQpExtWrenchesInstanceStruct
#define typedef_SFc5_balancingOptTauQpExtWrenchesInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c5_sfEvent;
  boolean_T c5_isStable;
  boolean_T c5_doneDoubleBufferReInit;
  uint8_T c5_is_active_c5_balancingOptTauQpExtWrenches;
  c5_struct_IJeSM2PPiABtzGLybE5MPF c5_reg;
  c5_struct_IHUdSPrvFcumP3hkkkz03F c5_gain;
} SFc5_balancingOptTauQpExtWrenchesInstanceStruct;

#endif                                 /*typedef_SFc5_balancingOptTauQpExtWrenchesInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c5_balancingOptTauQpExtWrenches_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c5_balancingOptTauQpExtWrenches_get_check_sum(mxArray *plhs[]);
extern void c5_balancingOptTauQpExtWrenches_method_dispatcher(SimStruct *S,
  int_T method, void *data);

#endif
