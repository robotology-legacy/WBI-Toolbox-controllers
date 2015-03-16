#ifndef __c1_IT36tFoaVyxqQYvXqVO43D_WBCLibrary_h__
#define __c1_IT36tFoaVyxqQYvXqVO43D_WBCLibrary_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_IT36tFoaVyxqQYvXqVO43D_WBCLibraryInstanceStruct
#define typedef_SFc1_IT36tFoaVyxqQYvXqVO43D_WBCLibraryInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_IT36tFoaVyxqQYvXqVO43D_sfEvent;
  boolean_T c1_IT36tFoaVyxqQYvXqVO43D_isStable;
  boolean_T c1_IT36tFoaVyxqQYvXqVO43D_doneDoubleBufferReInit;
  uint8_T c1_IT36tFoaVyxqQYvXqVO43D_is_active_c1_IT36tFoaVyxqQYvXqVO43D_WB;
  real_T c1_IT36tFoaVyxqQYvXqVO43D_state[23];
  boolean_T c1_IT36tFoaVyxqQYvXqVO43D_state_not_empty;
} SFc1_IT36tFoaVyxqQYvXqVO43D_WBCLibraryInstanceStruct;

#endif                                 /*typedef_SFc1_IT36tFoaVyxqQYvXqVO43D_WBCLibraryInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c1_IT36tFoaVyxqQYvXqVO43D_WBCLibrary_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_IT36tFoaVyxqQYvXqVO43D_WBCLibrary_get_check_sum(mxArray *plhs[]);
extern void c1_IT36tFoaVyxqQYvXqVO43D_WBCLibrary_method_dispatcher(SimStruct *S,
  int_T method, void *data);

#endif
