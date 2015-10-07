#ifndef __c8_balancingOptTauQpExtWrenches_h__
#define __c8_balancingOptTauQpExtWrenches_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
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

#ifndef typedef_c8_struct_IHUdSPrvFcumP3hkkkz03F
#define typedef_c8_struct_IHUdSPrvFcumP3hkkkz03F

typedef struct struct_IHUdSPrvFcumP3hkkkz03F_tag
  c8_struct_IHUdSPrvFcumP3hkkkz03F;

#endif                                 /*typedef_c8_struct_IHUdSPrvFcumP3hkkkz03F*/

#ifndef typedef_SFc8_balancingOptTauQpExtWrenchesInstanceStruct
#define typedef_SFc8_balancingOptTauQpExtWrenchesInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c8_sfEvent;
  boolean_T c8_isStable;
  boolean_T c8_doneDoubleBufferReInit;
  uint8_T c8_is_active_c8_balancingOptTauQpExtWrenches;
  c8_struct_IHUdSPrvFcumP3hkkkz03F c8_gain;
} SFc8_balancingOptTauQpExtWrenchesInstanceStruct;

#endif                                 /*typedef_SFc8_balancingOptTauQpExtWrenchesInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c8_balancingOptTauQpExtWrenches_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c8_balancingOptTauQpExtWrenches_get_check_sum(mxArray *plhs[]);
extern void c8_balancingOptTauQpExtWrenches_method_dispatcher(SimStruct *S,
  int_T method, void *data);

#endif
