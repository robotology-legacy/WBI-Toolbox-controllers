#ifndef __c1_yqEEFzHB8NwCm9QhWuhgMC_WBCLibrary_h__
#define __c1_yqEEFzHB8NwCm9QhWuhgMC_WBCLibrary_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_yqEEFzHB8NwCm9QhWuhgMC_WBCLibraryInstanceStruct
#define typedef_SFc1_yqEEFzHB8NwCm9QhWuhgMC_WBCLibraryInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_yqEEFzHB8NwCm9QhWuhgMC_sfEvent;
  boolean_T c1_yqEEFzHB8NwCm9QhWuhgMC_isStable;
  boolean_T c1_yqEEFzHB8NwCm9QhWuhgMC_doneDoubleBufferReInit;
  uint8_T c1_yqEEFzHB8NwCm9QhWuhgMC_is_active_c1_yqEEFzHB8NwCm9QhWuhgMC_WB;
  real_T c1_yqEEFzHB8NwCm9QhWuhgMC_state[15];
  boolean_T c1_yqEEFzHB8NwCm9QhWuhgMC_state_not_empty;
} SFc1_yqEEFzHB8NwCm9QhWuhgMC_WBCLibraryInstanceStruct;

#endif                                 /*typedef_SFc1_yqEEFzHB8NwCm9QhWuhgMC_WBCLibraryInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c1_yqEEFzHB8NwCm9QhWuhgMC_WBCLibrary_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_yqEEFzHB8NwCm9QhWuhgMC_WBCLibrary_get_check_sum(mxArray *plhs[]);
extern void c1_yqEEFzHB8NwCm9QhWuhgMC_WBCLibrary_method_dispatcher(SimStruct *S,
  int_T method, void *data);

#endif
