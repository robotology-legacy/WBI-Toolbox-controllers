#ifndef __c3_torqueBalancing_h__
#define __c3_torqueBalancing_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef struct_struct_f0Sh29boby3czvM1D0NwSH_tag
#define struct_struct_f0Sh29boby3czvM1D0NwSH_tag

struct struct_f0Sh29boby3czvM1D0NwSH_tag
{
  real_T directionOfOscillation[3];
  real_T amplitudeOfOscillation;
  real_T frequencyOfOscillation;
  real_T noOscillationTime;
};

#endif                                 /*struct_struct_f0Sh29boby3czvM1D0NwSH_tag*/

#ifndef typedef_c3_struct_f0Sh29boby3czvM1D0NwSH
#define typedef_c3_struct_f0Sh29boby3czvM1D0NwSH

typedef struct struct_f0Sh29boby3czvM1D0NwSH_tag
  c3_struct_f0Sh29boby3czvM1D0NwSH;

#endif                                 /*typedef_c3_struct_f0Sh29boby3czvM1D0NwSH*/

#ifndef typedef_SFc3_torqueBalancingInstanceStruct
#define typedef_SFc3_torqueBalancingInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c3_sfEvent;
  boolean_T c3_isStable;
  boolean_T c3_doneDoubleBufferReInit;
  uint8_T c3_is_active_c3_torqueBalancing;
  c3_struct_f0Sh29boby3czvM1D0NwSH c3_references;
} SFc3_torqueBalancingInstanceStruct;

#endif                                 /*typedef_SFc3_torqueBalancingInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c3_torqueBalancing_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c3_torqueBalancing_get_check_sum(mxArray *plhs[]);
extern void c3_torqueBalancing_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
