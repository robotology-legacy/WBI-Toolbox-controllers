#ifndef __c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_h__
#define __c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_K609Rm7ptn8v3rQ4xprp8_WBCLibraryInstanceStruct
#define typedef_SFc1_K609Rm7ptn8v3rQ4xprp8_WBCLibraryInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_K609Rm7ptn8v3rQ4xprp8_sfEvent;
  boolean_T c1_K609Rm7ptn8v3rQ4xprp8_isStable;
  boolean_T c1_K609Rm7ptn8v3rQ4xprp8_doneDoubleBufferReInit;
  uint8_T c1_K609Rm7ptn8v3rQ4xprp8_is_active_c1_K609Rm7ptn8v3rQ4xprp8_WBCL;
  real_T c1_K609Rm7ptn8v3rQ4xprp8_state[3];
  boolean_T c1_K609Rm7ptn8v3rQ4xprp8_state_not_empty;
} SFc1_K609Rm7ptn8v3rQ4xprp8_WBCLibraryInstanceStruct;

#endif                                 /*typedef_SFc1_K609Rm7ptn8v3rQ4xprp8_WBCLibraryInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_get_check_sum(mxArray *plhs[]);
extern void c1_K609Rm7ptn8v3rQ4xprp8_WBCLibrary_method_dispatcher(SimStruct *S,
  int_T method, void *data);

#endif
