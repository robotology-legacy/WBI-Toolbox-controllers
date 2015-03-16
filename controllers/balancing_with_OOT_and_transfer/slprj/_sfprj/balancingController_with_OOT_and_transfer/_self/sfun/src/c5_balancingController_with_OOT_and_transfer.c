/* Include files */

#include <stddef.h>
#include "blas.h"
#include "balancingController_with_OOT_and_transfer_sfun.h"
#include "c5_balancingController_with_OOT_and_transfer.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "balancingController_with_OOT_and_transfer_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c5_debug_family_names[8] = { "Jc", "nargin", "nargout",
  "JcLeftFoot", "JcRightFoot", "nunmberOfFeetOnGround", "qD", "y" };

/* Function Declarations */
static void initialize_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void initialize_params_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void enable_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void disable_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_update_debugger_state_c5_balancingController_with_OOT_and_tra
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void set_sim_state_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_st);
static void finalize_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void sf_gateway_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_chartstep_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void initSimStructsc5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber, uint32_T c5_instanceNumber);
static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData);
static void c5_emlrt_marshallIn
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_y, const char_T *c5_identifier, real_T c5_b_y[6]);
static void c5_b_emlrt_marshallIn
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId, real_T c5_y[6]);
static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static real_T c5_c_emlrt_marshallIn
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_e_sf_marshallOut(void *chartInstanceVoid, real_T
  c5_inData_data[], int32_T c5_inData_sizes[2]);
static void c5_d_emlrt_marshallIn
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId, real_T c5_y_data[],
   int32_T c5_y_sizes[2]);
static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, real_T c5_outData_data[], int32_T
  c5_outData_sizes[2]);
static void c5_info_helper(const mxArray **c5_info);
static const mxArray *c5_emlrt_marshallOut(const char * c5_u);
static const mxArray *c5_b_emlrt_marshallOut(const uint32_T c5_u);
static void c5_b_info_helper(const mxArray **c5_info);
static void c5_c_info_helper(const mxArray **c5_info);
static void c5_d_info_helper(const mxArray **c5_info);
static void c5_pinv(SFc5_balancingController_with_OOT_and_transferInstanceStruct
                    *chartInstance, real_T c5_A_data[], int32_T c5_A_sizes[2],
                    real_T c5_X_data[], int32_T c5_X_sizes[2]);
static void c5_eml_switch_helper
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_check_forloop_overflow_error
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   boolean_T c5_overflow);
static boolean_T c5_isfinite
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x);
static void c5_eml_error
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_eml_scalar_eg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_b_eml_scalar_eg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_threshold
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static real_T c5_abs
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x);
static void c5_realmin
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static real_T c5_eml_div
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x, real_T c5_y);
static void c5_b_threshold
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_c_threshold
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_d_threshold
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static real_T c5_eml_xdotc
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0,
   real_T c5_y_data[], int32_T c5_y_sizes[2], int32_T c5_iy0);
static void c5_scalarEg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, int32_T c5_ix0, real_T c5_y_data[], int32_T
   c5_y_sizes[2], int32_T c5_iy0, real_T c5_b_y_data[], int32_T c5_b_y_sizes[2]);
static void c5_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0,
   real_T c5_b_x_data[], int32_T c5_b_x_sizes[2]);
static void c5_b_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T
   c5_ix0, real_T c5_b_x_data[], int32_T c5_b_x_sizes[2]);
static void c5_eps(SFc5_balancingController_with_OOT_and_transferInstanceStruct *
                   chartInstance);
static void c5_c_eml_scalar_eg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_b_eml_error
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static real_T c5_sqrt
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x);
static void c5_c_eml_error
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_eml_xrotg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_a, real_T c5_b, real_T *c5_b_a, real_T *c5_b_b, real_T *c5_c,
   real_T *c5_s);
static void c5_eml_xrot
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0,
   int32_T c5_iy0, real_T c5_c, real_T c5_s, real_T c5_b_x_data[], int32_T
   c5_b_x_sizes[2]);
static void c5_e_threshold
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_eml_xswap
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0,
   int32_T c5_iy0, real_T c5_b_x_data[], int32_T c5_b_x_sizes[2]);
static void c5_f_threshold
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_d_eml_scalar_eg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_eml_xgesvd
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_A_data[], int32_T c5_A_sizes[2], real_T c5_U_data[], int32_T
   c5_U_sizes[2], real_T c5_S_data[], int32_T *c5_S_sizes, real_T c5_V_data[],
   int32_T c5_V_sizes[2]);
static real_T c5_eml_xnrm2
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0);
static void c5_c_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T
   c5_ix0, real_T c5_b_x_data[], int32_T c5_b_x_sizes[2]);
static real_T c5_b_eml_xdotc
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0,
   real_T c5_y_data[], int32_T c5_y_sizes[2], int32_T c5_iy0);
static void c5_b_scalarEg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_b_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, int32_T c5_ix0, real_T c5_y_data[], int32_T
   c5_y_sizes[2], int32_T c5_iy0, real_T c5_b_y_data[], int32_T c5_b_y_sizes[2]);
static real_T c5_b_eml_xnrm2
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x[6], int32_T c5_ix0);
static void c5_d_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x[6], int32_T c5_ix0, real_T c5_b_x[6]);
static void c5_c_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T
   c5_ix0, real_T c5_y_data[], int32_T c5_y_sizes, int32_T c5_iy0, real_T
   c5_b_y_data[], int32_T *c5_b_y_sizes);
static void c5_d_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes, int32_T
   c5_ix0, real_T c5_y_data[], int32_T c5_y_sizes[2], int32_T c5_iy0, real_T
   c5_b_y_data[], int32_T c5_b_y_sizes[2]);
static real_T c5_c_eml_xdotc
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x[36], int32_T c5_ix0, real_T c5_y[36], int32_T
   c5_iy0);
static void c5_e_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, int32_T c5_ix0, real_T c5_y[36], int32_T c5_iy0,
   real_T c5_b_y[36]);
static void c5_e_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_a, real_T c5_x[36], int32_T c5_ix0, real_T c5_b_x[36]);
static void c5_b_eml_xrot
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x[36], int32_T c5_ix0, int32_T c5_iy0, real_T c5_c, real_T c5_s,
   real_T c5_b_x[36]);
static void c5_e_eml_scalar_eg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_b_eml_xswap
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x[36], int32_T c5_ix0, int32_T c5_iy0, real_T c5_b_x[36]);
static void c5_g_threshold
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_eml_xgemm
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, int32_T c5_k, real_T c5_A_data[], int32_T c5_A_sizes[2], real_T
   c5_B_data[], int32_T c5_B_sizes[2], int32_T c5_ldb, real_T c5_C_data[],
   int32_T c5_C_sizes[2], real_T c5_b_C_data[], int32_T c5_b_C_sizes[2]);
static void c5_f_eml_scalar_eg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static void c5_b_eml_xgemm
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_k, real_T c5_A_data[], int32_T c5_A_sizes[2], real_T c5_B_data[],
   int32_T c5_B_sizes[2], int32_T c5_ldb, real_T c5_C[138], real_T c5_b_C[138]);
static void c5_g_eml_scalar_eg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);
static const mxArray *c5_f_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static int32_T c5_e_emlrt_marshallIn
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static uint8_T c5_f_emlrt_marshallIn
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_b_is_active_c5_balancingController_with_OOT_and_transfer,
   const char_T *c5_identifier);
static uint8_T c5_g_emlrt_marshallIn
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_f_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, int32_T c5_ix0, real_T c5_y_data[], int32_T
   c5_y_sizes[2], int32_T c5_iy0);
static void c5_f_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0);
static void c5_g_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T
   c5_ix0);
static void c5_b_sqrt
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T *c5_x);
static void c5_b_eml_xrotg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T *c5_a, real_T *c5_b, real_T *c5_c, real_T *c5_s);
static void c5_c_eml_xrot
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0,
   int32_T c5_iy0, real_T c5_c, real_T c5_s);
static void c5_c_eml_xswap
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0,
   int32_T c5_iy0);
static void c5_h_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T
   c5_ix0);
static void c5_g_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, int32_T c5_ix0, real_T c5_y_data[], int32_T
   c5_y_sizes[2], int32_T c5_iy0);
static void c5_i_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x[6], int32_T c5_ix0);
static void c5_h_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T
   c5_ix0, real_T c5_y_data[], int32_T *c5_y_sizes, int32_T c5_iy0);
static void c5_i_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes, int32_T
   c5_ix0, real_T c5_y_data[], int32_T c5_y_sizes[2], int32_T c5_iy0);
static void c5_j_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, int32_T c5_ix0, real_T c5_y[36], int32_T c5_iy0);
static void c5_j_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_a, real_T c5_x[36], int32_T c5_ix0);
static void c5_d_eml_xrot
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x[36], int32_T c5_ix0, int32_T c5_iy0, real_T c5_c, real_T c5_s);
static void c5_d_eml_xswap
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x[36], int32_T c5_ix0, int32_T c5_iy0);
static void c5_c_eml_xgemm
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, int32_T c5_k, real_T c5_A_data[], int32_T c5_A_sizes[2], real_T
   c5_B_data[], int32_T c5_B_sizes[2], int32_T c5_ldb, real_T c5_C_data[],
   int32_T c5_C_sizes[2]);
static void c5_d_eml_xgemm
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_k, real_T c5_A_data[], int32_T c5_A_sizes[2], real_T c5_B_data[],
   int32_T c5_B_sizes[2], int32_T c5_ldb, real_T c5_C[138]);
static void init_dsm_address_info
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  chartInstance->c5_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c5_is_active_c5_balancingController_with_OOT_and_transfer = 0U;
}

static void initialize_params_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c5_update_debugger_state_c5_balancingController_with_OOT_and_tra
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  const mxArray *c5_st;
  const mxArray *c5_y = NULL;
  int32_T c5_i0;
  real_T c5_u[6];
  const mxArray *c5_b_y = NULL;
  uint8_T c5_hoistedGlobal;
  uint8_T c5_b_u;
  const mxArray *c5_c_y = NULL;
  real_T (*c5_d_y)[6];
  c5_d_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c5_st = NULL;
  c5_st = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_createcellmatrix(2, 1), false);
  for (c5_i0 = 0; c5_i0 < 6; c5_i0++) {
    c5_u[c5_i0] = (*c5_d_y)[c5_i0];
  }

  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_setcell(c5_y, 0, c5_b_y);
  c5_hoistedGlobal =
    chartInstance->c5_is_active_c5_balancingController_with_OOT_and_transfer;
  c5_b_u = c5_hoistedGlobal;
  c5_c_y = NULL;
  sf_mex_assign(&c5_c_y, sf_mex_create("y", &c5_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 1, c5_c_y);
  sf_mex_assign(&c5_st, c5_y, false);
  return c5_st;
}

static void set_sim_state_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_st)
{
  const mxArray *c5_u;
  real_T c5_dv0[6];
  int32_T c5_i1;
  real_T (*c5_y)[6];
  c5_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c5_doneDoubleBufferReInit = true;
  c5_u = sf_mex_dup(c5_st);
  c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 0)), "y",
                      c5_dv0);
  for (c5_i1 = 0; c5_i1 < 6; c5_i1++) {
    (*c5_y)[c5_i1] = c5_dv0[c5_i1];
  }

  chartInstance->c5_is_active_c5_balancingController_with_OOT_and_transfer =
    c5_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 1)),
    "is_active_c5_balancingController_with_OOT_and_transfer");
  sf_mex_destroy(&c5_u);
  c5_update_debugger_state_c5_balancingController_with_OOT_and_tra(chartInstance);
  sf_mex_destroy(&c5_st);
}

static void finalize_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  int32_T c5_i2;
  int32_T c5_i3;
  int32_T c5_i4;
  int32_T c5_i5;
  real_T *c5_nunmberOfFeetOnGround;
  real_T (*c5_qD)[23];
  real_T (*c5_y)[6];
  real_T (*c5_JcRightFoot)[174];
  real_T (*c5_JcLeftFoot)[174];
  c5_qD = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
  c5_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c5_nunmberOfFeetOnGround = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c5_JcRightFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 1);
  c5_JcLeftFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c5_sfEvent);
  for (c5_i2 = 0; c5_i2 < 174; c5_i2++) {
    _SFD_DATA_RANGE_CHECK((*c5_JcLeftFoot)[c5_i2], 0U);
  }

  for (c5_i3 = 0; c5_i3 < 174; c5_i3++) {
    _SFD_DATA_RANGE_CHECK((*c5_JcRightFoot)[c5_i3], 1U);
  }

  _SFD_DATA_RANGE_CHECK(*c5_nunmberOfFeetOnGround, 2U);
  chartInstance->c5_sfEvent = CALL_EVENT;
  c5_chartstep_c5_balancingController_with_OOT_and_transfer(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY
    (_balancingController_with_OOT_and_transferMachineNumber_,
     chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c5_i4 = 0; c5_i4 < 6; c5_i4++) {
    _SFD_DATA_RANGE_CHECK((*c5_y)[c5_i4], 3U);
  }

  for (c5_i5 = 0; c5_i5 < 23; c5_i5++) {
    _SFD_DATA_RANGE_CHECK((*c5_qD)[c5_i5], 4U);
  }
}

static void c5_chartstep_c5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  real_T c5_hoistedGlobal;
  int32_T c5_i6;
  real_T c5_JcLeftFoot[174];
  int32_T c5_i7;
  real_T c5_JcRightFoot[174];
  real_T c5_nunmberOfFeetOnGround;
  int32_T c5_i8;
  real_T c5_qD[23];
  uint32_T c5_debug_family_var_map[8];
  int32_T c5_Jc_sizes[2];
  real_T c5_Jc_data[348];
  real_T c5_nargin = 4.0;
  real_T c5_nargout = 1.0;
  real_T c5_y[6];
  int32_T c5_i9;
  int32_T c5_i10;
  int32_T c5_i11;
  int32_T c5_i12;
  real_T c5_b_JcLeftFoot[348];
  int32_T c5_i13;
  int32_T c5_i14;
  int32_T c5_i15;
  int32_T c5_i16;
  int32_T c5_Jc;
  int32_T c5_b_Jc;
  int32_T c5_i17;
  int32_T c5_c_Jc;
  int32_T c5_d_Jc;
  int32_T c5_i18;
  int32_T c5_i19;
  int32_T c5_b_Jc_sizes[2];
  int32_T c5_i20;
  int32_T c5_loop_ub;
  int32_T c5_i21;
  real_T c5_b_Jc_data[72];
  int32_T c5_a_sizes[2];
  real_T c5_a_data[72];
  int32_T c5_a;
  int32_T c5_b_a;
  int32_T c5_c_a;
  int32_T c5_d_a;
  int32_T c5_b_loop_ub;
  int32_T c5_i22;
  int32_T c5_i23;
  int32_T c5_b_sizes[2];
  int32_T c5_i24;
  int32_T c5_c_loop_ub;
  int32_T c5_i25;
  real_T c5_b_data[276];
  boolean_T c5_innerDimOk;
  int32_T c5_i26;
  static char_T c5_cv0[21] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 'i', 'n', 'n', 'e', 'r', 'd', 'i', 'm' };

  char_T c5_u[21];
  const mxArray *c5_b_y = NULL;
  int32_T c5_k;
  int32_T c5_i27;
  real_T c5_c_y[138];
  int32_T c5_b_a_sizes[2];
  int32_T c5_e_a;
  int32_T c5_f_a;
  int32_T c5_d_loop_ub;
  int32_T c5_i28;
  real_T c5_b_a_data[72];
  int32_T c5_b_b_sizes[2];
  int32_T c5_b;
  int32_T c5_b_b;
  int32_T c5_e_loop_ub;
  int32_T c5_i29;
  real_T c5_b_b_data[276];
  int32_T c5_i30;
  real_T c5_c_b[23];
  int32_T c5_i31;
  int32_T c5_i32;
  int32_T c5_i33;
  real_T c5_C[6];
  int32_T c5_i34;
  int32_T c5_i35;
  int32_T c5_i36;
  int32_T c5_i37;
  int32_T c5_i38;
  int32_T c5_i39;
  int32_T c5_i40;
  real_T *c5_b_nunmberOfFeetOnGround;
  real_T (*c5_d_y)[6];
  real_T (*c5_b_qD)[23];
  real_T (*c5_b_JcRightFoot)[174];
  real_T (*c5_c_JcLeftFoot)[174];
  c5_b_qD = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
  c5_d_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c5_b_nunmberOfFeetOnGround = (real_T *)ssGetInputPortSignal(chartInstance->S,
    2);
  c5_b_JcRightFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 1);
  c5_c_JcLeftFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c5_sfEvent);
  c5_hoistedGlobal = *c5_b_nunmberOfFeetOnGround;
  for (c5_i6 = 0; c5_i6 < 174; c5_i6++) {
    c5_JcLeftFoot[c5_i6] = (*c5_c_JcLeftFoot)[c5_i6];
  }

  for (c5_i7 = 0; c5_i7 < 174; c5_i7++) {
    c5_JcRightFoot[c5_i7] = (*c5_b_JcRightFoot)[c5_i7];
  }

  c5_nunmberOfFeetOnGround = c5_hoistedGlobal;
  for (c5_i8 = 0; c5_i8 < 23; c5_i8++) {
    c5_qD[c5_i8] = (*c5_b_qD)[c5_i8];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 8U, 8U, c5_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c5_Jc_data, (const int32_T *)
    &c5_Jc_sizes, NULL, 0, 0, (void *)c5_e_sf_marshallOut, (void *)
    c5_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 1U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 2U, c5_c_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_JcLeftFoot, 3U, c5_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_JcRightFoot, 4U, c5_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_nunmberOfFeetOnGround, 5U, c5_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_qD, 6U, c5_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_y, 7U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 3);
  if (CV_EML_IF(0, 1, 0, c5_nunmberOfFeetOnGround == 2.0)) {
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 4);
    c5_i9 = 0;
    c5_i10 = 0;
    for (c5_i11 = 0; c5_i11 < 29; c5_i11++) {
      for (c5_i12 = 0; c5_i12 < 6; c5_i12++) {
        c5_b_JcLeftFoot[c5_i12 + c5_i9] = c5_JcLeftFoot[c5_i12 + c5_i10];
      }

      c5_i9 += 12;
      c5_i10 += 6;
    }

    c5_i13 = 0;
    c5_i14 = 0;
    for (c5_i15 = 0; c5_i15 < 29; c5_i15++) {
      for (c5_i16 = 0; c5_i16 < 6; c5_i16++) {
        c5_b_JcLeftFoot[(c5_i16 + c5_i13) + 6] = c5_JcRightFoot[c5_i16 + c5_i14];
      }

      c5_i13 += 12;
      c5_i14 += 6;
    }

    c5_Jc_sizes[0] = 12;
    c5_Jc_sizes[1] = 29;
    c5_Jc = c5_Jc_sizes[0];
    c5_b_Jc = c5_Jc_sizes[1];
    for (c5_i17 = 0; c5_i17 < 348; c5_i17++) {
      c5_Jc_data[c5_i17] = c5_b_JcLeftFoot[c5_i17];
    }
  } else {
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 6);
    c5_Jc_sizes[0] = 6;
    c5_Jc_sizes[1] = 29;
    c5_c_Jc = c5_Jc_sizes[0];
    c5_d_Jc = c5_Jc_sizes[1];
    for (c5_i18 = 0; c5_i18 < 174; c5_i18++) {
      c5_Jc_data[c5_i18] = c5_JcLeftFoot[c5_i18];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 8);
  c5_i19 = c5_Jc_sizes[0];
  c5_b_Jc_sizes[0] = c5_i19;
  c5_b_Jc_sizes[1] = 6;
  for (c5_i20 = 0; c5_i20 < 6; c5_i20++) {
    c5_loop_ub = c5_i19 - 1;
    for (c5_i21 = 0; c5_i21 <= c5_loop_ub; c5_i21++) {
      c5_b_Jc_data[c5_i21 + c5_b_Jc_sizes[0] * c5_i20] = c5_Jc_data[c5_i21 +
        c5_Jc_sizes[0] * c5_i20];
    }
  }

  c5_pinv(chartInstance, c5_b_Jc_data, c5_b_Jc_sizes, c5_a_data, c5_a_sizes);
  c5_a_sizes[0] = 6;
  c5_a_sizes[1];
  c5_a = c5_a_sizes[0];
  c5_b_a = c5_a_sizes[1];
  c5_c_a = c5_a_sizes[0];
  c5_d_a = c5_a_sizes[1];
  c5_b_loop_ub = c5_c_a * c5_d_a - 1;
  for (c5_i22 = 0; c5_i22 <= c5_b_loop_ub; c5_i22++) {
    c5_a_data[c5_i22] = -c5_a_data[c5_i22];
  }

  c5_i23 = c5_Jc_sizes[0];
  c5_b_sizes[0] = c5_i23;
  c5_b_sizes[1] = 23;
  for (c5_i24 = 0; c5_i24 < 23; c5_i24++) {
    c5_c_loop_ub = c5_i23 - 1;
    for (c5_i25 = 0; c5_i25 <= c5_c_loop_ub; c5_i25++) {
      c5_b_data[c5_i25 + c5_b_sizes[0] * c5_i24] = c5_Jc_data[c5_i25 +
        c5_Jc_sizes[0] * (6 + c5_i24)];
    }
  }

  c5_innerDimOk = ((real_T)c5_a_sizes[1] == (real_T)c5_b_sizes[0]);
  if (!c5_innerDimOk) {
    for (c5_i26 = 0; c5_i26 < 21; c5_i26++) {
      c5_u[c5_i26] = c5_cv0[c5_i26];
    }

    c5_b_y = NULL;
    sf_mex_assign(&c5_b_y, sf_mex_create("y", c5_u, 10, 0U, 1U, 0U, 2, 1, 21),
                  false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message",
      1U, 1U, 14, c5_b_y));
  }

  c5_k = c5_a_sizes[1];
  c5_f_eml_scalar_eg(chartInstance);
  c5_f_eml_scalar_eg(chartInstance);
  for (c5_i27 = 0; c5_i27 < 138; c5_i27++) {
    c5_c_y[c5_i27] = 0.0;
  }

  c5_b_a_sizes[0] = 6;
  c5_b_a_sizes[1] = c5_a_sizes[1];
  c5_e_a = c5_b_a_sizes[0];
  c5_f_a = c5_b_a_sizes[1];
  c5_d_loop_ub = c5_a_sizes[0] * c5_a_sizes[1] - 1;
  for (c5_i28 = 0; c5_i28 <= c5_d_loop_ub; c5_i28++) {
    c5_b_a_data[c5_i28] = c5_a_data[c5_i28];
  }

  c5_b_b_sizes[0] = c5_b_sizes[0];
  c5_b_b_sizes[1] = 23;
  c5_b = c5_b_b_sizes[0];
  c5_b_b = c5_b_b_sizes[1];
  c5_e_loop_ub = c5_b_sizes[0] * c5_b_sizes[1] - 1;
  for (c5_i29 = 0; c5_i29 <= c5_e_loop_ub; c5_i29++) {
    c5_b_b_data[c5_i29] = c5_b_data[c5_i29];
  }

  c5_d_eml_xgemm(chartInstance, c5_k, c5_b_a_data, c5_b_a_sizes, c5_b_b_data,
                 c5_b_b_sizes, c5_k, c5_c_y);
  for (c5_i30 = 0; c5_i30 < 23; c5_i30++) {
    c5_c_b[c5_i30] = c5_qD[c5_i30];
  }

  c5_g_eml_scalar_eg(chartInstance);
  c5_g_eml_scalar_eg(chartInstance);
  for (c5_i31 = 0; c5_i31 < 6; c5_i31++) {
    c5_y[c5_i31] = 0.0;
  }

  for (c5_i32 = 0; c5_i32 < 6; c5_i32++) {
    c5_y[c5_i32] = 0.0;
  }

  for (c5_i33 = 0; c5_i33 < 6; c5_i33++) {
    c5_C[c5_i33] = c5_y[c5_i33];
  }

  for (c5_i34 = 0; c5_i34 < 6; c5_i34++) {
    c5_y[c5_i34] = c5_C[c5_i34];
  }

  c5_f_threshold(chartInstance);
  for (c5_i35 = 0; c5_i35 < 6; c5_i35++) {
    c5_C[c5_i35] = c5_y[c5_i35];
  }

  for (c5_i36 = 0; c5_i36 < 6; c5_i36++) {
    c5_y[c5_i36] = c5_C[c5_i36];
  }

  for (c5_i37 = 0; c5_i37 < 6; c5_i37++) {
    c5_y[c5_i37] = 0.0;
    c5_i38 = 0;
    for (c5_i39 = 0; c5_i39 < 23; c5_i39++) {
      c5_y[c5_i37] += c5_c_y[c5_i38 + c5_i37] * c5_c_b[c5_i39];
      c5_i38 += 6;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -8);
  _SFD_SYMBOL_SCOPE_POP();
  for (c5_i40 = 0; c5_i40 < 6; c5_i40++) {
    (*c5_d_y)[c5_i40] = c5_y[c5_i40];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c5_sfEvent);
}

static void initSimStructsc5_balancingController_with_OOT_and_transfer
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber, uint32_T c5_instanceNumber)
{
  (void)c5_machineNumber;
  (void)c5_chartNumber;
  (void)c5_instanceNumber;
}

static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i41;
  real_T c5_b_inData[6];
  int32_T c5_i42;
  real_T c5_u[6];
  const mxArray *c5_y = NULL;
  SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance;
  chartInstance = (SFc5_balancingController_with_OOT_and_transferInstanceStruct *)
    chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i41 = 0; c5_i41 < 6; c5_i41++) {
    c5_b_inData[c5_i41] = (*(real_T (*)[6])c5_inData)[c5_i41];
  }

  for (c5_i42 = 0; c5_i42 < 6; c5_i42++) {
    c5_u[c5_i42] = c5_b_inData[c5_i42];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_emlrt_marshallIn
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_y, const char_T *c5_identifier, real_T c5_b_y[6])
{
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_y), &c5_thisId, c5_b_y);
  sf_mex_destroy(&c5_y);
}

static void c5_b_emlrt_marshallIn
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId, real_T c5_y[6])
{
  real_T c5_dv1[6];
  int32_T c5_i43;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv1, 1, 0, 0U, 1, 0U, 1, 6);
  for (c5_i43 = 0; c5_i43 < 6; c5_i43++) {
    c5_y[c5_i43] = c5_dv1[c5_i43];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_y;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_b_y[6];
  int32_T c5_i44;
  SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance;
  chartInstance = (SFc5_balancingController_with_OOT_and_transferInstanceStruct *)
    chartInstanceVoid;
  c5_y = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_y), &c5_thisId, c5_b_y);
  sf_mex_destroy(&c5_y);
  for (c5_i44 = 0; c5_i44 < 6; c5_i44++) {
    (*(real_T (*)[6])c5_outData)[c5_i44] = c5_b_y[c5_i44];
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i45;
  real_T c5_b_inData[23];
  int32_T c5_i46;
  real_T c5_u[23];
  const mxArray *c5_y = NULL;
  SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance;
  chartInstance = (SFc5_balancingController_with_OOT_and_transferInstanceStruct *)
    chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i45 = 0; c5_i45 < 23; c5_i45++) {
    c5_b_inData[c5_i45] = (*(real_T (*)[23])c5_inData)[c5_i45];
  }

  for (c5_i46 = 0; c5_i46 < 23; c5_i46++) {
    c5_u[c5_i46] = c5_b_inData[c5_i46];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 23), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  real_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance;
  chartInstance = (SFc5_balancingController_with_OOT_and_transferInstanceStruct *)
    chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(real_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i47;
  int32_T c5_i48;
  int32_T c5_i49;
  real_T c5_b_inData[174];
  int32_T c5_i50;
  int32_T c5_i51;
  int32_T c5_i52;
  real_T c5_u[174];
  const mxArray *c5_y = NULL;
  SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance;
  chartInstance = (SFc5_balancingController_with_OOT_and_transferInstanceStruct *)
    chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_i47 = 0;
  for (c5_i48 = 0; c5_i48 < 29; c5_i48++) {
    for (c5_i49 = 0; c5_i49 < 6; c5_i49++) {
      c5_b_inData[c5_i49 + c5_i47] = (*(real_T (*)[174])c5_inData)[c5_i49 +
        c5_i47];
    }

    c5_i47 += 6;
  }

  c5_i50 = 0;
  for (c5_i51 = 0; c5_i51 < 29; c5_i51++) {
    for (c5_i52 = 0; c5_i52 < 6; c5_i52++) {
      c5_u[c5_i52 + c5_i50] = c5_b_inData[c5_i52 + c5_i50];
    }

    c5_i50 += 6;
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 6, 29), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static real_T c5_c_emlrt_marshallIn
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  real_T c5_y;
  real_T c5_d0;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_d0, 1, 0, 0U, 0, 0U, 0);
  c5_y = c5_d0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_nargout;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y;
  SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance;
  chartInstance = (SFc5_balancingController_with_OOT_and_transferInstanceStruct *)
    chartInstanceVoid;
  c5_nargout = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_nargout), &c5_thisId);
  sf_mex_destroy(&c5_nargout);
  *(real_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_e_sf_marshallOut(void *chartInstanceVoid, real_T
  c5_inData_data[], int32_T c5_inData_sizes[2])
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_b_inData_sizes[2];
  int32_T c5_i53;
  int32_T c5_loop_ub;
  int32_T c5_i54;
  real_T c5_b_inData_data[348];
  int32_T c5_u_sizes[2];
  int32_T c5_i55;
  int32_T c5_b_loop_ub;
  int32_T c5_i56;
  real_T c5_u_data[348];
  const mxArray *c5_y = NULL;
  SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance;
  chartInstance = (SFc5_balancingController_with_OOT_and_transferInstanceStruct *)
    chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_b_inData_sizes[0] = c5_inData_sizes[0];
  c5_b_inData_sizes[1] = 29;
  for (c5_i53 = 0; c5_i53 < 29; c5_i53++) {
    c5_loop_ub = c5_inData_sizes[0] - 1;
    for (c5_i54 = 0; c5_i54 <= c5_loop_ub; c5_i54++) {
      c5_b_inData_data[c5_i54 + c5_b_inData_sizes[0] * c5_i53] =
        c5_inData_data[c5_i54 + c5_inData_sizes[0] * c5_i53];
    }
  }

  c5_u_sizes[0] = c5_b_inData_sizes[0];
  c5_u_sizes[1] = 29;
  for (c5_i55 = 0; c5_i55 < 29; c5_i55++) {
    c5_b_loop_ub = c5_b_inData_sizes[0] - 1;
    for (c5_i56 = 0; c5_i56 <= c5_b_loop_ub; c5_i56++) {
      c5_u_data[c5_i56 + c5_u_sizes[0] * c5_i55] = c5_b_inData_data[c5_i56 +
        c5_b_inData_sizes[0] * c5_i55];
    }
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u_data, 0, 0U, 1U, 0U, 2,
    c5_u_sizes[0], c5_u_sizes[1]), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_d_emlrt_marshallIn
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId, real_T c5_y_data[],
   int32_T c5_y_sizes[2])
{
  int32_T c5_i57;
  uint32_T c5_uv0[2];
  int32_T c5_i58;
  static boolean_T c5_bv0[2] = { true, false };

  boolean_T c5_bv1[2];
  int32_T c5_tmp_sizes[2];
  real_T c5_tmp_data[348];
  int32_T c5_y;
  int32_T c5_b_y;
  int32_T c5_loop_ub;
  int32_T c5_i59;
  (void)chartInstance;
  for (c5_i57 = 0; c5_i57 < 2; c5_i57++) {
    c5_uv0[c5_i57] = 12U + 17U * (uint32_T)c5_i57;
  }

  for (c5_i58 = 0; c5_i58 < 2; c5_i58++) {
    c5_bv1[c5_i58] = c5_bv0[c5_i58];
  }

  sf_mex_import_vs(c5_parentId, sf_mex_dup(c5_u), c5_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c5_bv1, c5_uv0, c5_tmp_sizes);
  c5_y_sizes[0] = c5_tmp_sizes[0];
  c5_y_sizes[1] = 29;
  c5_y = c5_y_sizes[0];
  c5_b_y = c5_y_sizes[1];
  c5_loop_ub = c5_tmp_sizes[0] * c5_tmp_sizes[1] - 1;
  for (c5_i59 = 0; c5_i59 <= c5_loop_ub; c5_i59++) {
    c5_y_data[c5_i59] = c5_tmp_data[c5_i59];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, real_T c5_outData_data[], int32_T
  c5_outData_sizes[2])
{
  const mxArray *c5_Jc;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  int32_T c5_y_sizes[2];
  real_T c5_y_data[348];
  int32_T c5_i60;
  int32_T c5_loop_ub;
  int32_T c5_i61;
  SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance;
  chartInstance = (SFc5_balancingController_with_OOT_and_transferInstanceStruct *)
    chartInstanceVoid;
  c5_Jc = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_Jc), &c5_thisId, c5_y_data,
                        c5_y_sizes);
  sf_mex_destroy(&c5_Jc);
  c5_outData_sizes[0] = c5_y_sizes[0];
  c5_outData_sizes[1] = 29;
  for (c5_i60 = 0; c5_i60 < 29; c5_i60++) {
    c5_loop_ub = c5_y_sizes[0] - 1;
    for (c5_i61 = 0; c5_i61 <= c5_loop_ub; c5_i61++) {
      c5_outData_data[c5_i61 + c5_outData_sizes[0] * c5_i60] = c5_y_data[c5_i61
        + c5_y_sizes[0] * c5_i60];
    }
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

const mxArray
  *sf_c5_balancingController_with_OOT_and_transfer_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c5_nameCaptureInfo = NULL;
  c5_nameCaptureInfo = NULL;
  sf_mex_assign(&c5_nameCaptureInfo, sf_mex_createstruct("structure", 2, 213, 1),
                false);
  c5_info_helper(&c5_nameCaptureInfo);
  c5_b_info_helper(&c5_nameCaptureInfo);
  c5_c_info_helper(&c5_nameCaptureInfo);
  c5_d_info_helper(&c5_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static void c5_info_helper(const mxArray **c5_info)
{
  const mxArray *c5_rhs0 = NULL;
  const mxArray *c5_lhs0 = NULL;
  const mxArray *c5_rhs1 = NULL;
  const mxArray *c5_lhs1 = NULL;
  const mxArray *c5_rhs2 = NULL;
  const mxArray *c5_lhs2 = NULL;
  const mxArray *c5_rhs3 = NULL;
  const mxArray *c5_lhs3 = NULL;
  const mxArray *c5_rhs4 = NULL;
  const mxArray *c5_lhs4 = NULL;
  const mxArray *c5_rhs5 = NULL;
  const mxArray *c5_lhs5 = NULL;
  const mxArray *c5_rhs6 = NULL;
  const mxArray *c5_lhs6 = NULL;
  const mxArray *c5_rhs7 = NULL;
  const mxArray *c5_lhs7 = NULL;
  const mxArray *c5_rhs8 = NULL;
  const mxArray *c5_lhs8 = NULL;
  const mxArray *c5_rhs9 = NULL;
  const mxArray *c5_lhs9 = NULL;
  const mxArray *c5_rhs10 = NULL;
  const mxArray *c5_lhs10 = NULL;
  const mxArray *c5_rhs11 = NULL;
  const mxArray *c5_lhs11 = NULL;
  const mxArray *c5_rhs12 = NULL;
  const mxArray *c5_lhs12 = NULL;
  const mxArray *c5_rhs13 = NULL;
  const mxArray *c5_lhs13 = NULL;
  const mxArray *c5_rhs14 = NULL;
  const mxArray *c5_lhs14 = NULL;
  const mxArray *c5_rhs15 = NULL;
  const mxArray *c5_lhs15 = NULL;
  const mxArray *c5_rhs16 = NULL;
  const mxArray *c5_lhs16 = NULL;
  const mxArray *c5_rhs17 = NULL;
  const mxArray *c5_lhs17 = NULL;
  const mxArray *c5_rhs18 = NULL;
  const mxArray *c5_lhs18 = NULL;
  const mxArray *c5_rhs19 = NULL;
  const mxArray *c5_lhs19 = NULL;
  const mxArray *c5_rhs20 = NULL;
  const mxArray *c5_lhs20 = NULL;
  const mxArray *c5_rhs21 = NULL;
  const mxArray *c5_lhs21 = NULL;
  const mxArray *c5_rhs22 = NULL;
  const mxArray *c5_lhs22 = NULL;
  const mxArray *c5_rhs23 = NULL;
  const mxArray *c5_lhs23 = NULL;
  const mxArray *c5_rhs24 = NULL;
  const mxArray *c5_lhs24 = NULL;
  const mxArray *c5_rhs25 = NULL;
  const mxArray *c5_lhs25 = NULL;
  const mxArray *c5_rhs26 = NULL;
  const mxArray *c5_lhs26 = NULL;
  const mxArray *c5_rhs27 = NULL;
  const mxArray *c5_lhs27 = NULL;
  const mxArray *c5_rhs28 = NULL;
  const mxArray *c5_lhs28 = NULL;
  const mxArray *c5_rhs29 = NULL;
  const mxArray *c5_lhs29 = NULL;
  const mxArray *c5_rhs30 = NULL;
  const mxArray *c5_lhs30 = NULL;
  const mxArray *c5_rhs31 = NULL;
  const mxArray *c5_lhs31 = NULL;
  const mxArray *c5_rhs32 = NULL;
  const mxArray *c5_lhs32 = NULL;
  const mxArray *c5_rhs33 = NULL;
  const mxArray *c5_lhs33 = NULL;
  const mxArray *c5_rhs34 = NULL;
  const mxArray *c5_lhs34 = NULL;
  const mxArray *c5_rhs35 = NULL;
  const mxArray *c5_lhs35 = NULL;
  const mxArray *c5_rhs36 = NULL;
  const mxArray *c5_lhs36 = NULL;
  const mxArray *c5_rhs37 = NULL;
  const mxArray *c5_lhs37 = NULL;
  const mxArray *c5_rhs38 = NULL;
  const mxArray *c5_lhs38 = NULL;
  const mxArray *c5_rhs39 = NULL;
  const mxArray *c5_lhs39 = NULL;
  const mxArray *c5_rhs40 = NULL;
  const mxArray *c5_lhs40 = NULL;
  const mxArray *c5_rhs41 = NULL;
  const mxArray *c5_lhs41 = NULL;
  const mxArray *c5_rhs42 = NULL;
  const mxArray *c5_lhs42 = NULL;
  const mxArray *c5_rhs43 = NULL;
  const mxArray *c5_lhs43 = NULL;
  const mxArray *c5_rhs44 = NULL;
  const mxArray *c5_lhs44 = NULL;
  const mxArray *c5_rhs45 = NULL;
  const mxArray *c5_lhs45 = NULL;
  const mxArray *c5_rhs46 = NULL;
  const mxArray *c5_lhs46 = NULL;
  const mxArray *c5_rhs47 = NULL;
  const mxArray *c5_lhs47 = NULL;
  const mxArray *c5_rhs48 = NULL;
  const mxArray *c5_lhs48 = NULL;
  const mxArray *c5_rhs49 = NULL;
  const mxArray *c5_lhs49 = NULL;
  const mxArray *c5_rhs50 = NULL;
  const mxArray *c5_lhs50 = NULL;
  const mxArray *c5_rhs51 = NULL;
  const mxArray *c5_lhs51 = NULL;
  const mxArray *c5_rhs52 = NULL;
  const mxArray *c5_lhs52 = NULL;
  const mxArray *c5_rhs53 = NULL;
  const mxArray *c5_lhs53 = NULL;
  const mxArray *c5_rhs54 = NULL;
  const mxArray *c5_lhs54 = NULL;
  const mxArray *c5_rhs55 = NULL;
  const mxArray *c5_lhs55 = NULL;
  const mxArray *c5_rhs56 = NULL;
  const mxArray *c5_lhs56 = NULL;
  const mxArray *c5_rhs57 = NULL;
  const mxArray *c5_lhs57 = NULL;
  const mxArray *c5_rhs58 = NULL;
  const mxArray *c5_lhs58 = NULL;
  const mxArray *c5_rhs59 = NULL;
  const mxArray *c5_lhs59 = NULL;
  const mxArray *c5_rhs60 = NULL;
  const mxArray *c5_lhs60 = NULL;
  const mxArray *c5_rhs61 = NULL;
  const mxArray *c5_lhs61 = NULL;
  const mxArray *c5_rhs62 = NULL;
  const mxArray *c5_lhs62 = NULL;
  const mxArray *c5_rhs63 = NULL;
  const mxArray *c5_lhs63 = NULL;
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("pinv"), "name", "name", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818828U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c5_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c5_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c5_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c5_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("svd"), "name", "name", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "resolved",
                  "resolved", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818832U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c5_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c5_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c5_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmax"), "name", "name", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c5_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c5_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("isfinite"), "name", "name", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713856U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c5_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c5_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("isinf"), "name", "name", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713856U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c5_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c5_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("isnan"), "name", "name", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713858U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c5_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c5_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_error"), "name", "name",
                  15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343830358U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c5_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/svd.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xgesvd"), "name", "name",
                  16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818806U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c5_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgesvd.m"),
                  "context", "context", 17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_lapack_xgesvd"), "name",
                  "name", 17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818810U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c5_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgesvd.m"),
                  "context", "context", 18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_matlab_zsvdc"), "name",
                  "name", 18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1295284866U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c5_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c5_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c5_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c5_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c5_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("min"), "name", "name", 23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1311255318U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c5_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "context",
                  "context", 24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1378295984U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c5_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c5_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c5_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c5_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c5_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c5_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c5_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c5_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c5_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("max"), "name", "name", 33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1311255316U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c5_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "context",
                  "context", 34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1378295984U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c5_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c5_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c5_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c5_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c5_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_relop"), "name", "name",
                  39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "resolved",
                  "resolved", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1342451182U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c5_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "context",
                  "context", 40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexIntRelop"),
                  "name", "name", 40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1326728322U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c5_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!apply_float_relop"),
                  "context", "context", 41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c5_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass"),
                  "context", "context", 42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c5_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass"),
                  "context", "context", 43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmin"), "name", "name", 43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c5_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "context",
                  "context", 44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c5_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("isnan"), "name", "name", 45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713858U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c5_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c5_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 47);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 47);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 47);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c5_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 48);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 48);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 48);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c5_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 49);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("max"), "name", "name", 49);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 49);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 49);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1311255316U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c5_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 50);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 50);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 50);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 50);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c5_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "context", "context", 51);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 51);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 51);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 51);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c5_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 52);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 52);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 52);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 52);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c5_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 53);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 53);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 53);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 53);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c5_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 54);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 54);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 54);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 54);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c5_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 55);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 55);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 55);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 55);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c5_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 56);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xnrm2"), "name", "name",
                  56);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 56);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"),
                  "resolved", "resolved", 56);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c5_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 57);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 57);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 57);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 57);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c5_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 58);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.xnrm2"),
                  "name", "name", 58);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 58);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "resolved", "resolved", 58);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c5_rhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 59);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 59);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 59);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 59);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c5_rhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!below_threshold"),
                  "context", "context", 60);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 60);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 60);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 60);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c5_rhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 61);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 61);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 61);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 61);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c5_rhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 62);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.refblas.xnrm2"),
                  "name", "name", 62);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 62);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "resolved", "resolved", 62);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c5_rhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 63);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("abs"), "name", "name", 63);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 63);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 63);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c5_rhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c5_rhs0);
  sf_mex_destroy(&c5_lhs0);
  sf_mex_destroy(&c5_rhs1);
  sf_mex_destroy(&c5_lhs1);
  sf_mex_destroy(&c5_rhs2);
  sf_mex_destroy(&c5_lhs2);
  sf_mex_destroy(&c5_rhs3);
  sf_mex_destroy(&c5_lhs3);
  sf_mex_destroy(&c5_rhs4);
  sf_mex_destroy(&c5_lhs4);
  sf_mex_destroy(&c5_rhs5);
  sf_mex_destroy(&c5_lhs5);
  sf_mex_destroy(&c5_rhs6);
  sf_mex_destroy(&c5_lhs6);
  sf_mex_destroy(&c5_rhs7);
  sf_mex_destroy(&c5_lhs7);
  sf_mex_destroy(&c5_rhs8);
  sf_mex_destroy(&c5_lhs8);
  sf_mex_destroy(&c5_rhs9);
  sf_mex_destroy(&c5_lhs9);
  sf_mex_destroy(&c5_rhs10);
  sf_mex_destroy(&c5_lhs10);
  sf_mex_destroy(&c5_rhs11);
  sf_mex_destroy(&c5_lhs11);
  sf_mex_destroy(&c5_rhs12);
  sf_mex_destroy(&c5_lhs12);
  sf_mex_destroy(&c5_rhs13);
  sf_mex_destroy(&c5_lhs13);
  sf_mex_destroy(&c5_rhs14);
  sf_mex_destroy(&c5_lhs14);
  sf_mex_destroy(&c5_rhs15);
  sf_mex_destroy(&c5_lhs15);
  sf_mex_destroy(&c5_rhs16);
  sf_mex_destroy(&c5_lhs16);
  sf_mex_destroy(&c5_rhs17);
  sf_mex_destroy(&c5_lhs17);
  sf_mex_destroy(&c5_rhs18);
  sf_mex_destroy(&c5_lhs18);
  sf_mex_destroy(&c5_rhs19);
  sf_mex_destroy(&c5_lhs19);
  sf_mex_destroy(&c5_rhs20);
  sf_mex_destroy(&c5_lhs20);
  sf_mex_destroy(&c5_rhs21);
  sf_mex_destroy(&c5_lhs21);
  sf_mex_destroy(&c5_rhs22);
  sf_mex_destroy(&c5_lhs22);
  sf_mex_destroy(&c5_rhs23);
  sf_mex_destroy(&c5_lhs23);
  sf_mex_destroy(&c5_rhs24);
  sf_mex_destroy(&c5_lhs24);
  sf_mex_destroy(&c5_rhs25);
  sf_mex_destroy(&c5_lhs25);
  sf_mex_destroy(&c5_rhs26);
  sf_mex_destroy(&c5_lhs26);
  sf_mex_destroy(&c5_rhs27);
  sf_mex_destroy(&c5_lhs27);
  sf_mex_destroy(&c5_rhs28);
  sf_mex_destroy(&c5_lhs28);
  sf_mex_destroy(&c5_rhs29);
  sf_mex_destroy(&c5_lhs29);
  sf_mex_destroy(&c5_rhs30);
  sf_mex_destroy(&c5_lhs30);
  sf_mex_destroy(&c5_rhs31);
  sf_mex_destroy(&c5_lhs31);
  sf_mex_destroy(&c5_rhs32);
  sf_mex_destroy(&c5_lhs32);
  sf_mex_destroy(&c5_rhs33);
  sf_mex_destroy(&c5_lhs33);
  sf_mex_destroy(&c5_rhs34);
  sf_mex_destroy(&c5_lhs34);
  sf_mex_destroy(&c5_rhs35);
  sf_mex_destroy(&c5_lhs35);
  sf_mex_destroy(&c5_rhs36);
  sf_mex_destroy(&c5_lhs36);
  sf_mex_destroy(&c5_rhs37);
  sf_mex_destroy(&c5_lhs37);
  sf_mex_destroy(&c5_rhs38);
  sf_mex_destroy(&c5_lhs38);
  sf_mex_destroy(&c5_rhs39);
  sf_mex_destroy(&c5_lhs39);
  sf_mex_destroy(&c5_rhs40);
  sf_mex_destroy(&c5_lhs40);
  sf_mex_destroy(&c5_rhs41);
  sf_mex_destroy(&c5_lhs41);
  sf_mex_destroy(&c5_rhs42);
  sf_mex_destroy(&c5_lhs42);
  sf_mex_destroy(&c5_rhs43);
  sf_mex_destroy(&c5_lhs43);
  sf_mex_destroy(&c5_rhs44);
  sf_mex_destroy(&c5_lhs44);
  sf_mex_destroy(&c5_rhs45);
  sf_mex_destroy(&c5_lhs45);
  sf_mex_destroy(&c5_rhs46);
  sf_mex_destroy(&c5_lhs46);
  sf_mex_destroy(&c5_rhs47);
  sf_mex_destroy(&c5_lhs47);
  sf_mex_destroy(&c5_rhs48);
  sf_mex_destroy(&c5_lhs48);
  sf_mex_destroy(&c5_rhs49);
  sf_mex_destroy(&c5_lhs49);
  sf_mex_destroy(&c5_rhs50);
  sf_mex_destroy(&c5_lhs50);
  sf_mex_destroy(&c5_rhs51);
  sf_mex_destroy(&c5_lhs51);
  sf_mex_destroy(&c5_rhs52);
  sf_mex_destroy(&c5_lhs52);
  sf_mex_destroy(&c5_rhs53);
  sf_mex_destroy(&c5_lhs53);
  sf_mex_destroy(&c5_rhs54);
  sf_mex_destroy(&c5_lhs54);
  sf_mex_destroy(&c5_rhs55);
  sf_mex_destroy(&c5_lhs55);
  sf_mex_destroy(&c5_rhs56);
  sf_mex_destroy(&c5_lhs56);
  sf_mex_destroy(&c5_rhs57);
  sf_mex_destroy(&c5_lhs57);
  sf_mex_destroy(&c5_rhs58);
  sf_mex_destroy(&c5_lhs58);
  sf_mex_destroy(&c5_rhs59);
  sf_mex_destroy(&c5_lhs59);
  sf_mex_destroy(&c5_rhs60);
  sf_mex_destroy(&c5_lhs60);
  sf_mex_destroy(&c5_rhs61);
  sf_mex_destroy(&c5_lhs61);
  sf_mex_destroy(&c5_rhs62);
  sf_mex_destroy(&c5_lhs62);
  sf_mex_destroy(&c5_rhs63);
  sf_mex_destroy(&c5_lhs63);
}

static const mxArray *c5_emlrt_marshallOut(const char * c5_u)
{
  const mxArray *c5_y = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c5_u)), false);
  return c5_y;
}

static const mxArray *c5_b_emlrt_marshallOut(const uint32_T c5_u)
{
  const mxArray *c5_y = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 7, 0U, 0U, 0U, 0), false);
  return c5_y;
}

static void c5_b_info_helper(const mxArray **c5_info)
{
  const mxArray *c5_rhs64 = NULL;
  const mxArray *c5_lhs64 = NULL;
  const mxArray *c5_rhs65 = NULL;
  const mxArray *c5_lhs65 = NULL;
  const mxArray *c5_rhs66 = NULL;
  const mxArray *c5_lhs66 = NULL;
  const mxArray *c5_rhs67 = NULL;
  const mxArray *c5_lhs67 = NULL;
  const mxArray *c5_rhs68 = NULL;
  const mxArray *c5_lhs68 = NULL;
  const mxArray *c5_rhs69 = NULL;
  const mxArray *c5_lhs69 = NULL;
  const mxArray *c5_rhs70 = NULL;
  const mxArray *c5_lhs70 = NULL;
  const mxArray *c5_rhs71 = NULL;
  const mxArray *c5_lhs71 = NULL;
  const mxArray *c5_rhs72 = NULL;
  const mxArray *c5_lhs72 = NULL;
  const mxArray *c5_rhs73 = NULL;
  const mxArray *c5_lhs73 = NULL;
  const mxArray *c5_rhs74 = NULL;
  const mxArray *c5_lhs74 = NULL;
  const mxArray *c5_rhs75 = NULL;
  const mxArray *c5_lhs75 = NULL;
  const mxArray *c5_rhs76 = NULL;
  const mxArray *c5_lhs76 = NULL;
  const mxArray *c5_rhs77 = NULL;
  const mxArray *c5_lhs77 = NULL;
  const mxArray *c5_rhs78 = NULL;
  const mxArray *c5_lhs78 = NULL;
  const mxArray *c5_rhs79 = NULL;
  const mxArray *c5_lhs79 = NULL;
  const mxArray *c5_rhs80 = NULL;
  const mxArray *c5_lhs80 = NULL;
  const mxArray *c5_rhs81 = NULL;
  const mxArray *c5_lhs81 = NULL;
  const mxArray *c5_rhs82 = NULL;
  const mxArray *c5_lhs82 = NULL;
  const mxArray *c5_rhs83 = NULL;
  const mxArray *c5_lhs83 = NULL;
  const mxArray *c5_rhs84 = NULL;
  const mxArray *c5_lhs84 = NULL;
  const mxArray *c5_rhs85 = NULL;
  const mxArray *c5_lhs85 = NULL;
  const mxArray *c5_rhs86 = NULL;
  const mxArray *c5_lhs86 = NULL;
  const mxArray *c5_rhs87 = NULL;
  const mxArray *c5_lhs87 = NULL;
  const mxArray *c5_rhs88 = NULL;
  const mxArray *c5_lhs88 = NULL;
  const mxArray *c5_rhs89 = NULL;
  const mxArray *c5_lhs89 = NULL;
  const mxArray *c5_rhs90 = NULL;
  const mxArray *c5_lhs90 = NULL;
  const mxArray *c5_rhs91 = NULL;
  const mxArray *c5_lhs91 = NULL;
  const mxArray *c5_rhs92 = NULL;
  const mxArray *c5_lhs92 = NULL;
  const mxArray *c5_rhs93 = NULL;
  const mxArray *c5_lhs93 = NULL;
  const mxArray *c5_rhs94 = NULL;
  const mxArray *c5_lhs94 = NULL;
  const mxArray *c5_rhs95 = NULL;
  const mxArray *c5_lhs95 = NULL;
  const mxArray *c5_rhs96 = NULL;
  const mxArray *c5_lhs96 = NULL;
  const mxArray *c5_rhs97 = NULL;
  const mxArray *c5_lhs97 = NULL;
  const mxArray *c5_rhs98 = NULL;
  const mxArray *c5_lhs98 = NULL;
  const mxArray *c5_rhs99 = NULL;
  const mxArray *c5_lhs99 = NULL;
  const mxArray *c5_rhs100 = NULL;
  const mxArray *c5_lhs100 = NULL;
  const mxArray *c5_rhs101 = NULL;
  const mxArray *c5_lhs101 = NULL;
  const mxArray *c5_rhs102 = NULL;
  const mxArray *c5_lhs102 = NULL;
  const mxArray *c5_rhs103 = NULL;
  const mxArray *c5_lhs103 = NULL;
  const mxArray *c5_rhs104 = NULL;
  const mxArray *c5_lhs104 = NULL;
  const mxArray *c5_rhs105 = NULL;
  const mxArray *c5_lhs105 = NULL;
  const mxArray *c5_rhs106 = NULL;
  const mxArray *c5_lhs106 = NULL;
  const mxArray *c5_rhs107 = NULL;
  const mxArray *c5_lhs107 = NULL;
  const mxArray *c5_rhs108 = NULL;
  const mxArray *c5_lhs108 = NULL;
  const mxArray *c5_rhs109 = NULL;
  const mxArray *c5_lhs109 = NULL;
  const mxArray *c5_rhs110 = NULL;
  const mxArray *c5_lhs110 = NULL;
  const mxArray *c5_rhs111 = NULL;
  const mxArray *c5_lhs111 = NULL;
  const mxArray *c5_rhs112 = NULL;
  const mxArray *c5_lhs112 = NULL;
  const mxArray *c5_rhs113 = NULL;
  const mxArray *c5_lhs113 = NULL;
  const mxArray *c5_rhs114 = NULL;
  const mxArray *c5_lhs114 = NULL;
  const mxArray *c5_rhs115 = NULL;
  const mxArray *c5_lhs115 = NULL;
  const mxArray *c5_rhs116 = NULL;
  const mxArray *c5_lhs116 = NULL;
  const mxArray *c5_rhs117 = NULL;
  const mxArray *c5_lhs117 = NULL;
  const mxArray *c5_rhs118 = NULL;
  const mxArray *c5_lhs118 = NULL;
  const mxArray *c5_rhs119 = NULL;
  const mxArray *c5_lhs119 = NULL;
  const mxArray *c5_rhs120 = NULL;
  const mxArray *c5_lhs120 = NULL;
  const mxArray *c5_rhs121 = NULL;
  const mxArray *c5_lhs121 = NULL;
  const mxArray *c5_rhs122 = NULL;
  const mxArray *c5_lhs122 = NULL;
  const mxArray *c5_rhs123 = NULL;
  const mxArray *c5_lhs123 = NULL;
  const mxArray *c5_rhs124 = NULL;
  const mxArray *c5_lhs124 = NULL;
  const mxArray *c5_rhs125 = NULL;
  const mxArray *c5_lhs125 = NULL;
  const mxArray *c5_rhs126 = NULL;
  const mxArray *c5_lhs126 = NULL;
  const mxArray *c5_rhs127 = NULL;
  const mxArray *c5_lhs127 = NULL;
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 64);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 64);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 64);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 64);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 64);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 64);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 64);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 64);
  sf_mex_assign(&c5_rhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs64), "rhs", "rhs",
                  64);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs64), "lhs", "lhs",
                  64);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 65);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 65);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 65);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 65);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818712U), "fileTimeLo",
                  "fileTimeLo", 65);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 65);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 65);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 65);
  sf_mex_assign(&c5_rhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs65), "rhs", "rhs",
                  65);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs65), "lhs", "lhs",
                  65);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 66);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("realmin"), "name", "name", 66);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 66);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 66);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1307651242U), "fileTimeLo",
                  "fileTimeLo", 66);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 66);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 66);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 66);
  sf_mex_assign(&c5_rhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs66), "rhs", "rhs",
                  66);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs66), "lhs", "lhs",
                  66);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "context",
                  "context", 67);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_realmin"), "name", "name",
                  67);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 67);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "resolved",
                  "resolved", 67);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1307651244U), "fileTimeLo",
                  "fileTimeLo", 67);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 67);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 67);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 67);
  sf_mex_assign(&c5_rhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs67), "rhs", "rhs",
                  67);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs67), "lhs", "lhs",
                  67);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "context",
                  "context", 68);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 68);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 68);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 68);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 68);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 68);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 68);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 68);
  sf_mex_assign(&c5_rhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs68), "rhs", "rhs",
                  68);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs68), "lhs", "lhs",
                  68);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 69);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 69);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 69);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 69);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 69);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 69);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 69);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 69);
  sf_mex_assign(&c5_rhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs69), "rhs", "rhs",
                  69);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs69), "lhs", "lhs",
                  69);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 70);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 70);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 70);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 70);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 70);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 70);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 70);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 70);
  sf_mex_assign(&c5_rhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs70), "rhs", "rhs",
                  70);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs70), "lhs", "lhs",
                  70);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 71);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 71);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 71);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 71);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 71);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 71);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 71);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 71);
  sf_mex_assign(&c5_rhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs71), "rhs", "rhs",
                  71);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs71), "lhs", "lhs",
                  71);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 72);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 72);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 72);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 72);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 72);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 72);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 72);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 72);
  sf_mex_assign(&c5_rhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs72), "rhs", "rhs",
                  72);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs72), "lhs", "lhs",
                  72);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 73);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.int"),
                  "name", "name", 73);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 73);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/int.p"),
                  "resolved", "resolved", 73);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 73);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 73);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 73);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 73);
  sf_mex_assign(&c5_rhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs73), "rhs", "rhs",
                  73);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs73), "lhs", "lhs",
                  73);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!ceval_xnrm2"),
                  "context", "context", 74);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.size_ptr"),
                  "name", "name", 74);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 74);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/size_ptr.p"),
                  "resolved", "resolved", 74);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 74);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 74);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 74);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 74);
  sf_mex_assign(&c5_rhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs74), "rhs", "rhs",
                  74);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs74), "lhs", "lhs",
                  74);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!ceval_xnrm2"),
                  "context", "context", 75);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.c_cast"),
                  "name", "name", 75);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("int32"), "dominantType",
                  "dominantType", 75);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/c_cast.p"),
                  "resolved", "resolved", 75);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 75);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 75);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 75);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 75);
  sf_mex_assign(&c5_rhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs75), "rhs", "rhs",
                  75);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs75), "lhs", "lhs",
                  75);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 76);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_div"), "name", "name", 76);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 76);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 76);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 76);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 76);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 76);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 76);
  sf_mex_assign(&c5_rhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs76), "rhs", "rhs",
                  76);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs76), "lhs", "lhs",
                  76);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 77);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 77);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 77);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 77);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 77);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 77);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 77);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 77);
  sf_mex_assign(&c5_rhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs77), "rhs", "rhs",
                  77);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs77), "lhs", "lhs",
                  77);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 78);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xscal"), "name", "name",
                  78);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 78);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"),
                  "resolved", "resolved", 78);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 78);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 78);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 78);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 78);
  sf_mex_assign(&c5_rhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs78), "rhs", "rhs",
                  78);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs78), "lhs", "lhs",
                  78);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"), "context",
                  "context", 79);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 79);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 79);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 79);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 79);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 79);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 79);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 79);
  sf_mex_assign(&c5_rhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs79), "rhs", "rhs",
                  79);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs79), "lhs", "lhs",
                  79);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"), "context",
                  "context", 80);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.xscal"),
                  "name", "name", 80);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 80);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "resolved", "resolved", 80);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 80);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 80);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 80);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 80);
  sf_mex_assign(&c5_rhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs80), "rhs", "rhs",
                  80);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs80), "lhs", "lhs",
                  80);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 81);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 81);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 81);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 81);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 81);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 81);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 81);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 81);
  sf_mex_assign(&c5_rhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs81), "rhs", "rhs",
                  81);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs81), "lhs", "lhs",
                  81);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p!below_threshold"),
                  "context", "context", 82);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 82);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 82);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 82);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 82);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 82);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 82);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 82);
  sf_mex_assign(&c5_rhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs82), "rhs", "rhs",
                  82);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs82), "lhs", "lhs",
                  82);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 83);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 83);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 83);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 83);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 83);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 83);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 83);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 83);
  sf_mex_assign(&c5_rhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs83), "rhs", "rhs",
                  83);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs83), "lhs", "lhs",
                  83);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 84);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.refblas.xscal"),
                  "name", "name", 84);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 84);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "resolved", "resolved", 84);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 84);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 84);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 84);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 84);
  sf_mex_assign(&c5_rhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs84), "rhs", "rhs",
                  84);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs84), "lhs", "lhs",
                  84);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 85);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 85);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 85);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 85);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 85);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 85);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 85);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 85);
  sf_mex_assign(&c5_rhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs85), "rhs", "rhs",
                  85);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs85), "lhs", "lhs",
                  85);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 86);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 86);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 86);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 86);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 86);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 86);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 86);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 86);
  sf_mex_assign(&c5_rhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs86), "rhs", "rhs",
                  86);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs86), "lhs", "lhs",
                  86);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 87);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 87);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 87);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 87);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 87);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 87);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 87);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 87);
  sf_mex_assign(&c5_rhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs87), "rhs", "rhs",
                  87);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs87), "lhs", "lhs",
                  87);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 88);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 88);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 88);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 88);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 88);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 88);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 88);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 88);
  sf_mex_assign(&c5_rhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs88), "rhs", "rhs",
                  88);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs88), "lhs", "lhs",
                  88);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 89);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.int"),
                  "name", "name", 89);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 89);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/int.p"),
                  "resolved", "resolved", 89);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 89);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 89);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 89);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 89);
  sf_mex_assign(&c5_rhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs89), "rhs", "rhs",
                  89);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs89), "lhs", "lhs",
                  89);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p!ceval_xscal"),
                  "context", "context", 90);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.size_ptr"),
                  "name", "name", 90);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 90);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/size_ptr.p"),
                  "resolved", "resolved", 90);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 90);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 90);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 90);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 90);
  sf_mex_assign(&c5_rhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs90), "rhs", "rhs",
                  90);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs90), "lhs", "lhs",
                  90);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p!ceval_xscal"),
                  "context", "context", 91);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.c_cast"),
                  "name", "name", 91);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("int32"), "dominantType",
                  "dominantType", 91);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/c_cast.p"),
                  "resolved", "resolved", 91);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 91);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 91);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 91);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 91);
  sf_mex_assign(&c5_rhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs91), "rhs", "rhs",
                  91);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs91), "lhs", "lhs",
                  91);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 92);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xdotc"), "name", "name",
                  92);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 92);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"),
                  "resolved", "resolved", 92);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980690U), "fileTimeLo",
                  "fileTimeLo", 92);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 92);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 92);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 92);
  sf_mex_assign(&c5_rhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs92), "rhs", "rhs",
                  92);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs92), "lhs", "lhs",
                  92);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"), "context",
                  "context", 93);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 93);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 93);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 93);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 93);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 93);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 93);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 93);
  sf_mex_assign(&c5_rhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs93), "rhs", "rhs",
                  93);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs93), "lhs", "lhs",
                  93);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotc.m"), "context",
                  "context", 94);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.xdotc"),
                  "name", "name", 94);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 94);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdotc.p"),
                  "resolved", "resolved", 94);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 94);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 94);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 94);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 94);
  sf_mex_assign(&c5_rhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs94), "rhs", "rhs",
                  94);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs94), "lhs", "lhs",
                  94);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdotc.p"),
                  "context", "context", 95);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.xdot"),
                  "name", "name", 95);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 95);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "resolved", "resolved", 95);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 95);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 95);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 95);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 95);
  sf_mex_assign(&c5_rhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs95), "rhs", "rhs",
                  95);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs95), "lhs", "lhs",
                  95);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "context", "context", 96);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 96);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 96);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 96);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 96);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 96);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 96);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 96);
  sf_mex_assign(&c5_rhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs96), "rhs", "rhs",
                  96);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs96), "lhs", "lhs",
                  96);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p!below_threshold"),
                  "context", "context", 97);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 97);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 97);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 97);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 97);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 97);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 97);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 97);
  sf_mex_assign(&c5_rhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs97), "rhs", "rhs",
                  97);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs97), "lhs", "lhs",
                  97);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "context", "context", 98);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.refblas.xdot"),
                  "name", "name", 98);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 98);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdot.p"),
                  "resolved", "resolved", 98);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 98);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 98);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 98);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 98);
  sf_mex_assign(&c5_rhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs98), "rhs", "rhs",
                  98);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs98), "lhs", "lhs",
                  98);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdot.p"),
                  "context", "context", 99);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.refblas.xdotx"),
                  "name", "name", 99);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 99);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "resolved", "resolved", 99);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 99);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 99);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 99);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 99);
  sf_mex_assign(&c5_rhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs99), "rhs", "rhs",
                  99);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs99), "lhs", "lhs",
                  99);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 100);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 100);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 100);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 100);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 100);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 100);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 100);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 100);
  sf_mex_assign(&c5_rhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs100), "rhs", "rhs",
                  100);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs100), "lhs", "lhs",
                  100);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 101);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 101);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 101);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 101);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 101);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 101);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 101);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 101);
  sf_mex_assign(&c5_rhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs101), "rhs", "rhs",
                  101);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs101), "lhs", "lhs",
                  101);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xdotx.p"),
                  "context", "context", 102);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 102);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 102);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 102);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 102);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 102);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 102);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 102);
  sf_mex_assign(&c5_rhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs102), "rhs", "rhs",
                  102);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs102), "lhs", "lhs",
                  102);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "context", "context", 103);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 103);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 103);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 103);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 103);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 103);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 103);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 103);
  sf_mex_assign(&c5_rhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs103), "rhs", "rhs",
                  103);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs103), "lhs", "lhs",
                  103);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p"),
                  "context", "context", 104);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.int"),
                  "name", "name", 104);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 104);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/int.p"),
                  "resolved", "resolved", 104);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 104);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 104);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 104);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 104);
  sf_mex_assign(&c5_rhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs104), "rhs", "rhs",
                  104);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs104), "lhs", "lhs",
                  104);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p!ceval_xdot"),
                  "context", "context", 105);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 105);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 105);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 105);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 105);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 105);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 105);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 105);
  sf_mex_assign(&c5_rhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs105), "rhs", "rhs",
                  105);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs105), "lhs", "lhs",
                  105);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p!ceval_xdot"),
                  "context", "context", 106);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.size_ptr"),
                  "name", "name", 106);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 106);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/size_ptr.p"),
                  "resolved", "resolved", 106);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 106);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 106);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 106);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 106);
  sf_mex_assign(&c5_rhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs106), "rhs", "rhs",
                  106);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs106), "lhs", "lhs",
                  106);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p!ceval_xdot"),
                  "context", "context", 107);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.c_cast"),
                  "name", "name", 107);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("int32"), "dominantType",
                  "dominantType", 107);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/c_cast.p"),
                  "resolved", "resolved", 107);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 107);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 107);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 107);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 107);
  sf_mex_assign(&c5_rhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs107), "rhs", "rhs",
                  107);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs107), "lhs", "lhs",
                  107);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 108);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xaxpy"), "name", "name",
                  108);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 108);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m"),
                  "resolved", "resolved", 108);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 108);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 108);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 108);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 108);
  sf_mex_assign(&c5_rhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs108), "rhs", "rhs",
                  108);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs108), "lhs", "lhs",
                  108);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m"), "context",
                  "context", 109);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 109);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 109);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 109);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 109);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 109);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 109);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 109);
  sf_mex_assign(&c5_rhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs109), "rhs", "rhs",
                  109);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs109), "lhs", "lhs",
                  109);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xaxpy.m"), "context",
                  "context", 110);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.xaxpy"),
                  "name", "name", 110);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 110);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "resolved", "resolved", 110);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 110);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 110);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 110);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 110);
  sf_mex_assign(&c5_rhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs110), "rhs", "rhs",
                  110);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs110), "lhs", "lhs",
                  110);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "context", "context", 111);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 111);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 111);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 111);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 111);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 111);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 111);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 111);
  sf_mex_assign(&c5_rhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs111), "rhs", "rhs",
                  111);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs111), "lhs", "lhs",
                  111);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p!below_threshold"),
                  "context", "context", 112);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 112);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 112);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 112);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 112);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 112);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 112);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 112);
  sf_mex_assign(&c5_rhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs112), "rhs", "rhs",
                  112);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs112), "lhs", "lhs",
                  112);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "context", "context", 113);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 113);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 113);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 113);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 113);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 113);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 113);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 113);
  sf_mex_assign(&c5_rhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs113), "rhs", "rhs",
                  113);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs113), "lhs", "lhs",
                  113);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "context", "context", 114);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.refblas.xaxpy"),
                  "name", "name", 114);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 114);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "resolved", "resolved", 114);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 114);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 114);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 114);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 114);
  sf_mex_assign(&c5_rhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs114), "rhs", "rhs",
                  114);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs114), "lhs", "lhs",
                  114);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 115);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.isaUint"),
                  "name", "name", 115);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 115);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/isaUint.p"),
                  "resolved", "resolved", 115);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 115);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 115);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 115);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 115);
  sf_mex_assign(&c5_rhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs115), "rhs", "rhs",
                  115);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs115), "lhs", "lhs",
                  115);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 116);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 116);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 116);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 116);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 116);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 116);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 116);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 116);
  sf_mex_assign(&c5_rhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs116), "rhs", "rhs",
                  116);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs116), "lhs", "lhs",
                  116);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 117);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 117);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 117);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 117);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 117);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 117);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 117);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 117);
  sf_mex_assign(&c5_rhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs117), "rhs", "rhs",
                  117);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs117), "lhs", "lhs",
                  117);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 118);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 118);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 118);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 118);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 118);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 118);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 118);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 118);
  sf_mex_assign(&c5_rhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs118), "rhs", "rhs",
                  118);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs118), "lhs", "lhs",
                  118);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xaxpy.p"),
                  "context", "context", 119);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 119);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 119);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 119);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 119);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 119);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 119);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 119);
  sf_mex_assign(&c5_rhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs119), "rhs", "rhs",
                  119);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs119), "lhs", "lhs",
                  119);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p"),
                  "context", "context", 120);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.int"),
                  "name", "name", 120);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 120);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/int.p"),
                  "resolved", "resolved", 120);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 120);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 120);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 120);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 120);
  sf_mex_assign(&c5_rhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs120), "rhs", "rhs",
                  120);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs120), "lhs", "lhs",
                  120);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p!ceval_xaxpy"),
                  "context", "context", 121);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.size_ptr"),
                  "name", "name", 121);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 121);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/size_ptr.p"),
                  "resolved", "resolved", 121);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 121);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 121);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 121);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 121);
  sf_mex_assign(&c5_rhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs121), "rhs", "rhs",
                  121);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs121), "lhs", "lhs",
                  121);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p!ceval_xaxpy"),
                  "context", "context", 122);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.c_cast"),
                  "name", "name", 122);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("int32"), "dominantType",
                  "dominantType", 122);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/c_cast.p"),
                  "resolved", "resolved", 122);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 122);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 122);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 122);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 122);
  sf_mex_assign(&c5_rhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs122), "rhs", "rhs",
                  122);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs122), "lhs", "lhs",
                  122);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xaxpy.p!below_threshold"),
                  "context", "context", 123);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("length"), "name", "name", 123);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 123);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 123);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 123);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 123);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 123);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 123);
  sf_mex_assign(&c5_rhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs123), "rhs", "rhs",
                  123);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs123), "lhs", "lhs",
                  123);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 124);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmin"), "name", "name", 124);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 124);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 124);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 124);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 124);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 124);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 124);
  sf_mex_assign(&c5_rhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs124), "rhs", "rhs",
                  124);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs124), "lhs", "lhs",
                  124);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 125);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("abs"), "name", "name", 125);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 125);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 125);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 125);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 125);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 125);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 125);
  sf_mex_assign(&c5_rhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs125), "rhs", "rhs",
                  125);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs125), "lhs", "lhs",
                  125);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 126);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("realmin"), "name", "name", 126);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 126);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 126);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1307651242U), "fileTimeLo",
                  "fileTimeLo", 126);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 126);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 126);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 126);
  sf_mex_assign(&c5_rhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs126), "rhs", "rhs",
                  126);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs126), "lhs", "lhs",
                  126);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 127);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eps"), "name", "name", 127);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 127);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 127);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 127);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 127);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 127);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 127);
  sf_mex_assign(&c5_rhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs127), "rhs", "rhs",
                  127);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs127), "lhs", "lhs",
                  127);
  sf_mex_destroy(&c5_rhs64);
  sf_mex_destroy(&c5_lhs64);
  sf_mex_destroy(&c5_rhs65);
  sf_mex_destroy(&c5_lhs65);
  sf_mex_destroy(&c5_rhs66);
  sf_mex_destroy(&c5_lhs66);
  sf_mex_destroy(&c5_rhs67);
  sf_mex_destroy(&c5_lhs67);
  sf_mex_destroy(&c5_rhs68);
  sf_mex_destroy(&c5_lhs68);
  sf_mex_destroy(&c5_rhs69);
  sf_mex_destroy(&c5_lhs69);
  sf_mex_destroy(&c5_rhs70);
  sf_mex_destroy(&c5_lhs70);
  sf_mex_destroy(&c5_rhs71);
  sf_mex_destroy(&c5_lhs71);
  sf_mex_destroy(&c5_rhs72);
  sf_mex_destroy(&c5_lhs72);
  sf_mex_destroy(&c5_rhs73);
  sf_mex_destroy(&c5_lhs73);
  sf_mex_destroy(&c5_rhs74);
  sf_mex_destroy(&c5_lhs74);
  sf_mex_destroy(&c5_rhs75);
  sf_mex_destroy(&c5_lhs75);
  sf_mex_destroy(&c5_rhs76);
  sf_mex_destroy(&c5_lhs76);
  sf_mex_destroy(&c5_rhs77);
  sf_mex_destroy(&c5_lhs77);
  sf_mex_destroy(&c5_rhs78);
  sf_mex_destroy(&c5_lhs78);
  sf_mex_destroy(&c5_rhs79);
  sf_mex_destroy(&c5_lhs79);
  sf_mex_destroy(&c5_rhs80);
  sf_mex_destroy(&c5_lhs80);
  sf_mex_destroy(&c5_rhs81);
  sf_mex_destroy(&c5_lhs81);
  sf_mex_destroy(&c5_rhs82);
  sf_mex_destroy(&c5_lhs82);
  sf_mex_destroy(&c5_rhs83);
  sf_mex_destroy(&c5_lhs83);
  sf_mex_destroy(&c5_rhs84);
  sf_mex_destroy(&c5_lhs84);
  sf_mex_destroy(&c5_rhs85);
  sf_mex_destroy(&c5_lhs85);
  sf_mex_destroy(&c5_rhs86);
  sf_mex_destroy(&c5_lhs86);
  sf_mex_destroy(&c5_rhs87);
  sf_mex_destroy(&c5_lhs87);
  sf_mex_destroy(&c5_rhs88);
  sf_mex_destroy(&c5_lhs88);
  sf_mex_destroy(&c5_rhs89);
  sf_mex_destroy(&c5_lhs89);
  sf_mex_destroy(&c5_rhs90);
  sf_mex_destroy(&c5_lhs90);
  sf_mex_destroy(&c5_rhs91);
  sf_mex_destroy(&c5_lhs91);
  sf_mex_destroy(&c5_rhs92);
  sf_mex_destroy(&c5_lhs92);
  sf_mex_destroy(&c5_rhs93);
  sf_mex_destroy(&c5_lhs93);
  sf_mex_destroy(&c5_rhs94);
  sf_mex_destroy(&c5_lhs94);
  sf_mex_destroy(&c5_rhs95);
  sf_mex_destroy(&c5_lhs95);
  sf_mex_destroy(&c5_rhs96);
  sf_mex_destroy(&c5_lhs96);
  sf_mex_destroy(&c5_rhs97);
  sf_mex_destroy(&c5_lhs97);
  sf_mex_destroy(&c5_rhs98);
  sf_mex_destroy(&c5_lhs98);
  sf_mex_destroy(&c5_rhs99);
  sf_mex_destroy(&c5_lhs99);
  sf_mex_destroy(&c5_rhs100);
  sf_mex_destroy(&c5_lhs100);
  sf_mex_destroy(&c5_rhs101);
  sf_mex_destroy(&c5_lhs101);
  sf_mex_destroy(&c5_rhs102);
  sf_mex_destroy(&c5_lhs102);
  sf_mex_destroy(&c5_rhs103);
  sf_mex_destroy(&c5_lhs103);
  sf_mex_destroy(&c5_rhs104);
  sf_mex_destroy(&c5_lhs104);
  sf_mex_destroy(&c5_rhs105);
  sf_mex_destroy(&c5_lhs105);
  sf_mex_destroy(&c5_rhs106);
  sf_mex_destroy(&c5_lhs106);
  sf_mex_destroy(&c5_rhs107);
  sf_mex_destroy(&c5_lhs107);
  sf_mex_destroy(&c5_rhs108);
  sf_mex_destroy(&c5_lhs108);
  sf_mex_destroy(&c5_rhs109);
  sf_mex_destroy(&c5_lhs109);
  sf_mex_destroy(&c5_rhs110);
  sf_mex_destroy(&c5_lhs110);
  sf_mex_destroy(&c5_rhs111);
  sf_mex_destroy(&c5_lhs111);
  sf_mex_destroy(&c5_rhs112);
  sf_mex_destroy(&c5_lhs112);
  sf_mex_destroy(&c5_rhs113);
  sf_mex_destroy(&c5_lhs113);
  sf_mex_destroy(&c5_rhs114);
  sf_mex_destroy(&c5_lhs114);
  sf_mex_destroy(&c5_rhs115);
  sf_mex_destroy(&c5_lhs115);
  sf_mex_destroy(&c5_rhs116);
  sf_mex_destroy(&c5_lhs116);
  sf_mex_destroy(&c5_rhs117);
  sf_mex_destroy(&c5_lhs117);
  sf_mex_destroy(&c5_rhs118);
  sf_mex_destroy(&c5_lhs118);
  sf_mex_destroy(&c5_rhs119);
  sf_mex_destroy(&c5_lhs119);
  sf_mex_destroy(&c5_rhs120);
  sf_mex_destroy(&c5_lhs120);
  sf_mex_destroy(&c5_rhs121);
  sf_mex_destroy(&c5_lhs121);
  sf_mex_destroy(&c5_rhs122);
  sf_mex_destroy(&c5_lhs122);
  sf_mex_destroy(&c5_rhs123);
  sf_mex_destroy(&c5_lhs123);
  sf_mex_destroy(&c5_rhs124);
  sf_mex_destroy(&c5_lhs124);
  sf_mex_destroy(&c5_rhs125);
  sf_mex_destroy(&c5_lhs125);
  sf_mex_destroy(&c5_rhs126);
  sf_mex_destroy(&c5_lhs126);
  sf_mex_destroy(&c5_rhs127);
  sf_mex_destroy(&c5_lhs127);
}

static void c5_c_info_helper(const mxArray **c5_info)
{
  const mxArray *c5_rhs128 = NULL;
  const mxArray *c5_lhs128 = NULL;
  const mxArray *c5_rhs129 = NULL;
  const mxArray *c5_lhs129 = NULL;
  const mxArray *c5_rhs130 = NULL;
  const mxArray *c5_lhs130 = NULL;
  const mxArray *c5_rhs131 = NULL;
  const mxArray *c5_lhs131 = NULL;
  const mxArray *c5_rhs132 = NULL;
  const mxArray *c5_lhs132 = NULL;
  const mxArray *c5_rhs133 = NULL;
  const mxArray *c5_lhs133 = NULL;
  const mxArray *c5_rhs134 = NULL;
  const mxArray *c5_lhs134 = NULL;
  const mxArray *c5_rhs135 = NULL;
  const mxArray *c5_lhs135 = NULL;
  const mxArray *c5_rhs136 = NULL;
  const mxArray *c5_lhs136 = NULL;
  const mxArray *c5_rhs137 = NULL;
  const mxArray *c5_lhs137 = NULL;
  const mxArray *c5_rhs138 = NULL;
  const mxArray *c5_lhs138 = NULL;
  const mxArray *c5_rhs139 = NULL;
  const mxArray *c5_lhs139 = NULL;
  const mxArray *c5_rhs140 = NULL;
  const mxArray *c5_lhs140 = NULL;
  const mxArray *c5_rhs141 = NULL;
  const mxArray *c5_lhs141 = NULL;
  const mxArray *c5_rhs142 = NULL;
  const mxArray *c5_lhs142 = NULL;
  const mxArray *c5_rhs143 = NULL;
  const mxArray *c5_lhs143 = NULL;
  const mxArray *c5_rhs144 = NULL;
  const mxArray *c5_lhs144 = NULL;
  const mxArray *c5_rhs145 = NULL;
  const mxArray *c5_lhs145 = NULL;
  const mxArray *c5_rhs146 = NULL;
  const mxArray *c5_lhs146 = NULL;
  const mxArray *c5_rhs147 = NULL;
  const mxArray *c5_lhs147 = NULL;
  const mxArray *c5_rhs148 = NULL;
  const mxArray *c5_lhs148 = NULL;
  const mxArray *c5_rhs149 = NULL;
  const mxArray *c5_lhs149 = NULL;
  const mxArray *c5_rhs150 = NULL;
  const mxArray *c5_lhs150 = NULL;
  const mxArray *c5_rhs151 = NULL;
  const mxArray *c5_lhs151 = NULL;
  const mxArray *c5_rhs152 = NULL;
  const mxArray *c5_lhs152 = NULL;
  const mxArray *c5_rhs153 = NULL;
  const mxArray *c5_lhs153 = NULL;
  const mxArray *c5_rhs154 = NULL;
  const mxArray *c5_lhs154 = NULL;
  const mxArray *c5_rhs155 = NULL;
  const mxArray *c5_lhs155 = NULL;
  const mxArray *c5_rhs156 = NULL;
  const mxArray *c5_lhs156 = NULL;
  const mxArray *c5_rhs157 = NULL;
  const mxArray *c5_lhs157 = NULL;
  const mxArray *c5_rhs158 = NULL;
  const mxArray *c5_lhs158 = NULL;
  const mxArray *c5_rhs159 = NULL;
  const mxArray *c5_lhs159 = NULL;
  const mxArray *c5_rhs160 = NULL;
  const mxArray *c5_lhs160 = NULL;
  const mxArray *c5_rhs161 = NULL;
  const mxArray *c5_lhs161 = NULL;
  const mxArray *c5_rhs162 = NULL;
  const mxArray *c5_lhs162 = NULL;
  const mxArray *c5_rhs163 = NULL;
  const mxArray *c5_lhs163 = NULL;
  const mxArray *c5_rhs164 = NULL;
  const mxArray *c5_lhs164 = NULL;
  const mxArray *c5_rhs165 = NULL;
  const mxArray *c5_lhs165 = NULL;
  const mxArray *c5_rhs166 = NULL;
  const mxArray *c5_lhs166 = NULL;
  const mxArray *c5_rhs167 = NULL;
  const mxArray *c5_lhs167 = NULL;
  const mxArray *c5_rhs168 = NULL;
  const mxArray *c5_lhs168 = NULL;
  const mxArray *c5_rhs169 = NULL;
  const mxArray *c5_lhs169 = NULL;
  const mxArray *c5_rhs170 = NULL;
  const mxArray *c5_lhs170 = NULL;
  const mxArray *c5_rhs171 = NULL;
  const mxArray *c5_lhs171 = NULL;
  const mxArray *c5_rhs172 = NULL;
  const mxArray *c5_lhs172 = NULL;
  const mxArray *c5_rhs173 = NULL;
  const mxArray *c5_lhs173 = NULL;
  const mxArray *c5_rhs174 = NULL;
  const mxArray *c5_lhs174 = NULL;
  const mxArray *c5_rhs175 = NULL;
  const mxArray *c5_lhs175 = NULL;
  const mxArray *c5_rhs176 = NULL;
  const mxArray *c5_lhs176 = NULL;
  const mxArray *c5_rhs177 = NULL;
  const mxArray *c5_lhs177 = NULL;
  const mxArray *c5_rhs178 = NULL;
  const mxArray *c5_lhs178 = NULL;
  const mxArray *c5_rhs179 = NULL;
  const mxArray *c5_lhs179 = NULL;
  const mxArray *c5_rhs180 = NULL;
  const mxArray *c5_lhs180 = NULL;
  const mxArray *c5_rhs181 = NULL;
  const mxArray *c5_lhs181 = NULL;
  const mxArray *c5_rhs182 = NULL;
  const mxArray *c5_lhs182 = NULL;
  const mxArray *c5_rhs183 = NULL;
  const mxArray *c5_lhs183 = NULL;
  const mxArray *c5_rhs184 = NULL;
  const mxArray *c5_lhs184 = NULL;
  const mxArray *c5_rhs185 = NULL;
  const mxArray *c5_lhs185 = NULL;
  const mxArray *c5_rhs186 = NULL;
  const mxArray *c5_lhs186 = NULL;
  const mxArray *c5_rhs187 = NULL;
  const mxArray *c5_lhs187 = NULL;
  const mxArray *c5_rhs188 = NULL;
  const mxArray *c5_lhs188 = NULL;
  const mxArray *c5_rhs189 = NULL;
  const mxArray *c5_lhs189 = NULL;
  const mxArray *c5_rhs190 = NULL;
  const mxArray *c5_lhs190 = NULL;
  const mxArray *c5_rhs191 = NULL;
  const mxArray *c5_lhs191 = NULL;
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 128);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 128);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 128);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 128);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818782U), "fileTimeLo",
                  "fileTimeLo", 128);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 128);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 128);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 128);
  sf_mex_assign(&c5_rhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs128), "rhs", "rhs",
                  128);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs128), "lhs", "lhs",
                  128);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 129);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_eps"), "name", "name", 129);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 129);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "resolved",
                  "resolved", 129);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 129);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 129);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 129);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 129);
  sf_mex_assign(&c5_rhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs129), "rhs", "rhs",
                  129);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs129), "lhs", "lhs",
                  129);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "context",
                  "context", 130);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 130);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 130);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 130);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 130);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 130);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 130);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 130);
  sf_mex_assign(&c5_rhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs130), "rhs", "rhs",
                  130);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs130), "lhs", "lhs",
                  130);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 131);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 131);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 131);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 131);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 131);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 131);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 131);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 131);
  sf_mex_assign(&c5_rhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs131), "rhs", "rhs",
                  131);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs131), "lhs", "lhs",
                  131);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 132);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_error"), "name", "name",
                  132);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 132);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 132);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343830358U), "fileTimeLo",
                  "fileTimeLo", 132);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 132);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 132);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 132);
  sf_mex_assign(&c5_rhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs132), "rhs", "rhs",
                  132);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs132), "lhs", "lhs",
                  132);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 133);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_const_nonsingleton_dim"),
                  "name", "name", 133);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 133);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "resolved", "resolved", 133);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 133);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 133);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 133);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 133);
  sf_mex_assign(&c5_rhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs133), "rhs", "rhs",
                  133);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs133), "lhs", "lhs",
                  133);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "context", "context", 134);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.constNonSingletonDim"), "name", "name", 134);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 134);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/constNonSingletonDim.m"),
                  "resolved", "resolved", 134);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 134);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 134);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 134);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 134);
  sf_mex_assign(&c5_rhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs134), "rhs", "rhs",
                  134);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs134), "lhs", "lhs",
                  134);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 135);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 135);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 135);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 135);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 135);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 135);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 135);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 135);
  sf_mex_assign(&c5_rhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs135), "rhs", "rhs",
                  135);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs135), "lhs", "lhs",
                  135);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 136);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 136);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 136);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 136);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 136);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 136);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 136);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 136);
  sf_mex_assign(&c5_rhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs136), "rhs", "rhs",
                  136);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs136), "lhs", "lhs",
                  136);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 137);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 137);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 137);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 137);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 137);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 137);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 137);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 137);
  sf_mex_assign(&c5_rhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs137), "rhs", "rhs",
                  137);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs137), "lhs", "lhs",
                  137);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 138);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("isnan"), "name", "name", 138);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 138);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 138);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713858U), "fileTimeLo",
                  "fileTimeLo", 138);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 138);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 138);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 138);
  sf_mex_assign(&c5_rhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs138), "rhs", "rhs",
                  138);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs138), "lhs", "lhs",
                  138);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 139);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 139);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 139);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 139);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 139);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 139);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 139);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 139);
  sf_mex_assign(&c5_rhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs139), "rhs", "rhs",
                  139);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs139), "lhs", "lhs",
                  139);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 140);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 140);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 140);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 140);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 140);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 140);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 140);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 140);
  sf_mex_assign(&c5_rhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs140), "rhs", "rhs",
                  140);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs140), "lhs", "lhs",
                  140);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 141);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_relop"), "name", "name",
                  141);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 141);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "resolved",
                  "resolved", 141);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1342451182U), "fileTimeLo",
                  "fileTimeLo", 141);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 141);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 141);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 141);
  sf_mex_assign(&c5_rhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs141), "rhs", "rhs",
                  141);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs141), "lhs", "lhs",
                  141);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 142);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("sqrt"), "name", "name", 142);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 142);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 142);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343830386U), "fileTimeLo",
                  "fileTimeLo", 142);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 142);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 142);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 142);
  sf_mex_assign(&c5_rhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs142), "rhs", "rhs",
                  142);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs142), "lhs", "lhs",
                  142);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 143);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_error"), "name", "name",
                  143);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 143);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 143);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343830358U), "fileTimeLo",
                  "fileTimeLo", 143);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 143);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 143);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 143);
  sf_mex_assign(&c5_rhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs143), "rhs", "rhs",
                  143);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs143), "lhs", "lhs",
                  143);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 144);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 144);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 144);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 144);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818738U), "fileTimeLo",
                  "fileTimeLo", 144);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 144);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 144);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 144);
  sf_mex_assign(&c5_rhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs144), "rhs", "rhs",
                  144);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs144), "lhs", "lhs",
                  144);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 145);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xrotg"), "name", "name",
                  145);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 145);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m"),
                  "resolved", "resolved", 145);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 145);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 145);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 145);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 145);
  sf_mex_assign(&c5_rhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs145), "rhs", "rhs",
                  145);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs145), "lhs", "lhs",
                  145);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m"), "context",
                  "context", 146);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 146);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 146);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 146);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 146);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 146);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 146);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 146);
  sf_mex_assign(&c5_rhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs146), "rhs", "rhs",
                  146);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs146), "lhs", "lhs",
                  146);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrotg.m"), "context",
                  "context", 147);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.xrotg"),
                  "name", "name", 147);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 147);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p"),
                  "resolved", "resolved", 147);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 147);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 147);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 147);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 147);
  sf_mex_assign(&c5_rhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs147), "rhs", "rhs",
                  147);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs147), "lhs", "lhs",
                  147);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p"),
                  "context", "context", 148);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 148);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 148);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 148);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 148);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 148);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 148);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 148);
  sf_mex_assign(&c5_rhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs148), "rhs", "rhs",
                  148);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs148), "lhs", "lhs",
                  148);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p"),
                  "context", "context", 149);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.refblas.xrotg"),
                  "name", "name", 149);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 149);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "resolved", "resolved", 149);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 149);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 149);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 149);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 149);
  sf_mex_assign(&c5_rhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs149), "rhs", "rhs",
                  149);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs149), "lhs", "lhs",
                  149);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "context", "context", 150);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("abs"), "name", "name", 150);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 150);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 150);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 150);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 150);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 150);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 150);
  sf_mex_assign(&c5_rhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs150), "rhs", "rhs",
                  150);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs150), "lhs", "lhs",
                  150);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "context", "context", 151);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("mrdivide"), "name", "name",
                  151);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 151);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 151);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1388460096U), "fileTimeLo",
                  "fileTimeLo", 151);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 151);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1370009886U), "mFileTimeLo",
                  "mFileTimeLo", 151);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 151);
  sf_mex_assign(&c5_rhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs151), "rhs", "rhs",
                  151);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs151), "lhs", "lhs",
                  151);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 152);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 152);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 152);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 152);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 152);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 152);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 152);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 152);
  sf_mex_assign(&c5_rhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs152), "rhs", "rhs",
                  152);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs152), "lhs", "lhs",
                  152);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 153);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("rdivide"), "name", "name", 153);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 153);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 153);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 153);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 153);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 153);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 153);
  sf_mex_assign(&c5_rhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs153), "rhs", "rhs",
                  153);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs153), "lhs", "lhs",
                  153);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 154);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 154);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 154);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 154);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 154);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 154);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 154);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 154);
  sf_mex_assign(&c5_rhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs154), "rhs", "rhs",
                  154);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs154), "lhs", "lhs",
                  154);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 155);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 155);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 155);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 155);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 155);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 155);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 155);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 155);
  sf_mex_assign(&c5_rhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs155), "rhs", "rhs",
                  155);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs155), "lhs", "lhs",
                  155);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 156);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_div"), "name", "name", 156);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 156);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 156);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 156);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 156);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 156);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 156);
  sf_mex_assign(&c5_rhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs156), "rhs", "rhs",
                  156);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs156), "lhs", "lhs",
                  156);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrotg.p"),
                  "context", "context", 157);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("sqrt"), "name", "name", 157);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 157);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 157);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343830386U), "fileTimeLo",
                  "fileTimeLo", 157);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 157);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 157);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 157);
  sf_mex_assign(&c5_rhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs157), "rhs", "rhs",
                  157);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs157), "lhs", "lhs",
                  157);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrotg.p!eml_ceval_xrotg"),
                  "context", "context", 158);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 158);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 158);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 158);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 158);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 158);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 158);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 158);
  sf_mex_assign(&c5_rhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs158), "rhs", "rhs",
                  158);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs158), "lhs", "lhs",
                  158);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 159);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xrot"), "name", "name",
                  159);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 159);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m"), "resolved",
                  "resolved", 159);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 159);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 159);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 159);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 159);
  sf_mex_assign(&c5_rhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs159), "rhs", "rhs",
                  159);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs159), "lhs", "lhs",
                  159);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m"), "context",
                  "context", 160);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 160);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 160);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 160);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 160);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 160);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 160);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 160);
  sf_mex_assign(&c5_rhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs160), "rhs", "rhs",
                  160);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs160), "lhs", "lhs",
                  160);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xrot.m"), "context",
                  "context", 161);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.xrot"),
                  "name", "name", 161);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 161);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "resolved", "resolved", 161);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 161);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 161);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 161);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 161);
  sf_mex_assign(&c5_rhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs161), "rhs", "rhs",
                  161);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs161), "lhs", "lhs",
                  161);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "context", "context", 162);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 162);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 162);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 162);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 162);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 162);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 162);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 162);
  sf_mex_assign(&c5_rhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs162), "rhs", "rhs",
                  162);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs162), "lhs", "lhs",
                  162);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p!below_threshold"),
                  "context", "context", 163);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 163);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 163);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 163);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 163);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 163);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 163);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 163);
  sf_mex_assign(&c5_rhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs163), "rhs", "rhs",
                  163);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs163), "lhs", "lhs",
                  163);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "context", "context", 164);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 164);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 164);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 164);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 164);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 164);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 164);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 164);
  sf_mex_assign(&c5_rhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs164), "rhs", "rhs",
                  164);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs164), "lhs", "lhs",
                  164);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "context", "context", 165);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.refblas.xrot"),
                  "name", "name", 165);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 165);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrot.p"),
                  "resolved", "resolved", 165);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 165);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 165);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 165);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 165);
  sf_mex_assign(&c5_rhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs165), "rhs", "rhs",
                  165);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs165), "lhs", "lhs",
                  165);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrot.p"),
                  "context", "context", 166);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 166);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 166);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 166);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 166);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 166);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 166);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 166);
  sf_mex_assign(&c5_rhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs166), "rhs", "rhs",
                  166);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs166), "lhs", "lhs",
                  166);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xrot.p"),
                  "context", "context", 167);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 167);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 167);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 167);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 167);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 167);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 167);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 167);
  sf_mex_assign(&c5_rhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs167), "rhs", "rhs",
                  167);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs167), "lhs", "lhs",
                  167);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p"),
                  "context", "context", 168);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.int"),
                  "name", "name", 168);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 168);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/int.p"),
                  "resolved", "resolved", 168);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 168);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 168);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 168);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 168);
  sf_mex_assign(&c5_rhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs168), "rhs", "rhs",
                  168);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs168), "lhs", "lhs",
                  168);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p!ceval_xrot"),
                  "context", "context", 169);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.size_ptr"),
                  "name", "name", 169);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 169);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/size_ptr.p"),
                  "resolved", "resolved", 169);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 169);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 169);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 169);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 169);
  sf_mex_assign(&c5_rhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs169), "rhs", "rhs",
                  169);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs169), "lhs", "lhs",
                  169);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xrot.p!ceval_xrot"),
                  "context", "context", 170);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.c_cast"),
                  "name", "name", 170);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("int32"), "dominantType",
                  "dominantType", 170);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/c_cast.p"),
                  "resolved", "resolved", 170);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 170);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 170);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 170);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 170);
  sf_mex_assign(&c5_rhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs170), "rhs", "rhs",
                  170);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs170), "lhs", "lhs",
                  170);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zsvdc.m"),
                  "context", "context", 171);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xswap"), "name", "name",
                  171);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 171);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"),
                  "resolved", "resolved", 171);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 171);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 171);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 171);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 171);
  sf_mex_assign(&c5_rhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs171), "rhs", "rhs",
                  171);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs171), "lhs", "lhs",
                  171);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"), "context",
                  "context", 172);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 172);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 172);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 172);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 172);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 172);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 172);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 172);
  sf_mex_assign(&c5_rhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs172), "rhs", "rhs",
                  172);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs172), "lhs", "lhs",
                  172);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"), "context",
                  "context", 173);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.xswap"),
                  "name", "name", 173);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 173);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "resolved", "resolved", 173);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 173);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 173);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 173);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 173);
  sf_mex_assign(&c5_rhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs173), "rhs", "rhs",
                  173);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs173), "lhs", "lhs",
                  173);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "context", "context", 174);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.refblas.xswap"),
                  "name", "name", 174);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 174);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "resolved", "resolved", 174);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 174);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 174);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 174);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 174);
  sf_mex_assign(&c5_rhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs174), "rhs", "rhs",
                  174);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs174), "lhs", "lhs",
                  174);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 175);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("abs"), "name", "name", 175);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 175);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 175);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 175);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 175);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 175);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 175);
  sf_mex_assign(&c5_rhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs175), "rhs", "rhs",
                  175);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs175), "lhs", "lhs",
                  175);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 176);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 176);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 176);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 176);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 176);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 176);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 176);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 176);
  sf_mex_assign(&c5_rhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs176), "rhs", "rhs",
                  176);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs176), "lhs", "lhs",
                  176);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 177);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 177);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 177);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 177);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818712U), "fileTimeLo",
                  "fileTimeLo", 177);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 177);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 177);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 177);
  sf_mex_assign(&c5_rhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs177), "rhs", "rhs",
                  177);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs177), "lhs", "lhs",
                  177);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 178);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 178);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 178);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 178);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 178);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 178);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 178);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 178);
  sf_mex_assign(&c5_rhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs178), "rhs", "rhs",
                  178);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs178), "lhs", "lhs",
                  178);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 179);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 179);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 179);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 179);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 179);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 179);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 179);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 179);
  sf_mex_assign(&c5_rhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs179), "rhs", "rhs",
                  179);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs179), "lhs", "lhs",
                  179);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 180);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eps"), "name", "name", 180);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 180);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 180);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 180);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 180);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 180);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 180);
  sf_mex_assign(&c5_rhs180, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs180, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs180), "rhs", "rhs",
                  180);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs180), "lhs", "lhs",
                  180);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 181);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 181);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 181);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 181);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 181);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 181);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 181);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 181);
  sf_mex_assign(&c5_rhs181, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs181, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs181), "rhs", "rhs",
                  181);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs181), "lhs", "lhs",
                  181);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 182);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 182);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 182);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 182);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 182);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 182);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 182);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 182);
  sf_mex_assign(&c5_rhs182, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs182, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs182), "rhs", "rhs",
                  182);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs182), "lhs", "lhs",
                  182);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 183);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_div"), "name", "name", 183);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 183);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 183);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 183);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 183);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 183);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 183);
  sf_mex_assign(&c5_rhs183, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs183, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs183), "rhs", "rhs",
                  183);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs183), "lhs", "lhs",
                  183);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 184);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xscal"), "name", "name",
                  184);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 184);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"),
                  "resolved", "resolved", 184);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 184);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 184);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 184);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 184);
  sf_mex_assign(&c5_rhs184, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs184, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs184), "rhs", "rhs",
                  184);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs184), "lhs", "lhs",
                  184);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 185);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 185);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 185);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 185);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 185);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 185);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 185);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 185);
  sf_mex_assign(&c5_rhs185, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs185, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs185), "rhs", "rhs",
                  185);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs185), "lhs", "lhs",
                  185);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/pinv.m!eml_pinv"),
                  "context", "context", 186);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  186);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 186);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 186);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980690U), "fileTimeLo",
                  "fileTimeLo", 186);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 186);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 186);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 186);
  sf_mex_assign(&c5_rhs186, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs186, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs186), "rhs", "rhs",
                  186);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs186), "lhs", "lhs",
                  186);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 187);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 187);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 187);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 187);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 187);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 187);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 187);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 187);
  sf_mex_assign(&c5_rhs187, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs187, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs187), "rhs", "rhs",
                  187);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs187), "lhs", "lhs",
                  187);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 188);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 188);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 188);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 188);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 188);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 188);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 188);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 188);
  sf_mex_assign(&c5_rhs188, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs188, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs188), "rhs", "rhs",
                  188);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs188), "lhs", "lhs",
                  188);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 189);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 189);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 189);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 189);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 189);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 189);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 189);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 189);
  sf_mex_assign(&c5_rhs189, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs189, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs189), "rhs", "rhs",
                  189);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs189), "lhs", "lhs",
                  189);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 190);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 190);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 190);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 190);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 190);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 190);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 190);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 190);
  sf_mex_assign(&c5_rhs190, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs190, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs190), "rhs", "rhs",
                  190);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs190), "lhs", "lhs",
                  190);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 191);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 191);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 191);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 191);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 191);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 191);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 191);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 191);
  sf_mex_assign(&c5_rhs191, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs191, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs191), "rhs", "rhs",
                  191);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs191), "lhs", "lhs",
                  191);
  sf_mex_destroy(&c5_rhs128);
  sf_mex_destroy(&c5_lhs128);
  sf_mex_destroy(&c5_rhs129);
  sf_mex_destroy(&c5_lhs129);
  sf_mex_destroy(&c5_rhs130);
  sf_mex_destroy(&c5_lhs130);
  sf_mex_destroy(&c5_rhs131);
  sf_mex_destroy(&c5_lhs131);
  sf_mex_destroy(&c5_rhs132);
  sf_mex_destroy(&c5_lhs132);
  sf_mex_destroy(&c5_rhs133);
  sf_mex_destroy(&c5_lhs133);
  sf_mex_destroy(&c5_rhs134);
  sf_mex_destroy(&c5_lhs134);
  sf_mex_destroy(&c5_rhs135);
  sf_mex_destroy(&c5_lhs135);
  sf_mex_destroy(&c5_rhs136);
  sf_mex_destroy(&c5_lhs136);
  sf_mex_destroy(&c5_rhs137);
  sf_mex_destroy(&c5_lhs137);
  sf_mex_destroy(&c5_rhs138);
  sf_mex_destroy(&c5_lhs138);
  sf_mex_destroy(&c5_rhs139);
  sf_mex_destroy(&c5_lhs139);
  sf_mex_destroy(&c5_rhs140);
  sf_mex_destroy(&c5_lhs140);
  sf_mex_destroy(&c5_rhs141);
  sf_mex_destroy(&c5_lhs141);
  sf_mex_destroy(&c5_rhs142);
  sf_mex_destroy(&c5_lhs142);
  sf_mex_destroy(&c5_rhs143);
  sf_mex_destroy(&c5_lhs143);
  sf_mex_destroy(&c5_rhs144);
  sf_mex_destroy(&c5_lhs144);
  sf_mex_destroy(&c5_rhs145);
  sf_mex_destroy(&c5_lhs145);
  sf_mex_destroy(&c5_rhs146);
  sf_mex_destroy(&c5_lhs146);
  sf_mex_destroy(&c5_rhs147);
  sf_mex_destroy(&c5_lhs147);
  sf_mex_destroy(&c5_rhs148);
  sf_mex_destroy(&c5_lhs148);
  sf_mex_destroy(&c5_rhs149);
  sf_mex_destroy(&c5_lhs149);
  sf_mex_destroy(&c5_rhs150);
  sf_mex_destroy(&c5_lhs150);
  sf_mex_destroy(&c5_rhs151);
  sf_mex_destroy(&c5_lhs151);
  sf_mex_destroy(&c5_rhs152);
  sf_mex_destroy(&c5_lhs152);
  sf_mex_destroy(&c5_rhs153);
  sf_mex_destroy(&c5_lhs153);
  sf_mex_destroy(&c5_rhs154);
  sf_mex_destroy(&c5_lhs154);
  sf_mex_destroy(&c5_rhs155);
  sf_mex_destroy(&c5_lhs155);
  sf_mex_destroy(&c5_rhs156);
  sf_mex_destroy(&c5_lhs156);
  sf_mex_destroy(&c5_rhs157);
  sf_mex_destroy(&c5_lhs157);
  sf_mex_destroy(&c5_rhs158);
  sf_mex_destroy(&c5_lhs158);
  sf_mex_destroy(&c5_rhs159);
  sf_mex_destroy(&c5_lhs159);
  sf_mex_destroy(&c5_rhs160);
  sf_mex_destroy(&c5_lhs160);
  sf_mex_destroy(&c5_rhs161);
  sf_mex_destroy(&c5_lhs161);
  sf_mex_destroy(&c5_rhs162);
  sf_mex_destroy(&c5_lhs162);
  sf_mex_destroy(&c5_rhs163);
  sf_mex_destroy(&c5_lhs163);
  sf_mex_destroy(&c5_rhs164);
  sf_mex_destroy(&c5_lhs164);
  sf_mex_destroy(&c5_rhs165);
  sf_mex_destroy(&c5_lhs165);
  sf_mex_destroy(&c5_rhs166);
  sf_mex_destroy(&c5_lhs166);
  sf_mex_destroy(&c5_rhs167);
  sf_mex_destroy(&c5_lhs167);
  sf_mex_destroy(&c5_rhs168);
  sf_mex_destroy(&c5_lhs168);
  sf_mex_destroy(&c5_rhs169);
  sf_mex_destroy(&c5_lhs169);
  sf_mex_destroy(&c5_rhs170);
  sf_mex_destroy(&c5_lhs170);
  sf_mex_destroy(&c5_rhs171);
  sf_mex_destroy(&c5_lhs171);
  sf_mex_destroy(&c5_rhs172);
  sf_mex_destroy(&c5_lhs172);
  sf_mex_destroy(&c5_rhs173);
  sf_mex_destroy(&c5_lhs173);
  sf_mex_destroy(&c5_rhs174);
  sf_mex_destroy(&c5_lhs174);
  sf_mex_destroy(&c5_rhs175);
  sf_mex_destroy(&c5_lhs175);
  sf_mex_destroy(&c5_rhs176);
  sf_mex_destroy(&c5_lhs176);
  sf_mex_destroy(&c5_rhs177);
  sf_mex_destroy(&c5_lhs177);
  sf_mex_destroy(&c5_rhs178);
  sf_mex_destroy(&c5_lhs178);
  sf_mex_destroy(&c5_rhs179);
  sf_mex_destroy(&c5_lhs179);
  sf_mex_destroy(&c5_rhs180);
  sf_mex_destroy(&c5_lhs180);
  sf_mex_destroy(&c5_rhs181);
  sf_mex_destroy(&c5_lhs181);
  sf_mex_destroy(&c5_rhs182);
  sf_mex_destroy(&c5_lhs182);
  sf_mex_destroy(&c5_rhs183);
  sf_mex_destroy(&c5_lhs183);
  sf_mex_destroy(&c5_rhs184);
  sf_mex_destroy(&c5_lhs184);
  sf_mex_destroy(&c5_rhs185);
  sf_mex_destroy(&c5_lhs185);
  sf_mex_destroy(&c5_rhs186);
  sf_mex_destroy(&c5_lhs186);
  sf_mex_destroy(&c5_rhs187);
  sf_mex_destroy(&c5_lhs187);
  sf_mex_destroy(&c5_rhs188);
  sf_mex_destroy(&c5_lhs188);
  sf_mex_destroy(&c5_rhs189);
  sf_mex_destroy(&c5_lhs189);
  sf_mex_destroy(&c5_rhs190);
  sf_mex_destroy(&c5_lhs190);
  sf_mex_destroy(&c5_rhs191);
  sf_mex_destroy(&c5_lhs191);
}

static void c5_d_info_helper(const mxArray **c5_info)
{
  const mxArray *c5_rhs192 = NULL;
  const mxArray *c5_lhs192 = NULL;
  const mxArray *c5_rhs193 = NULL;
  const mxArray *c5_lhs193 = NULL;
  const mxArray *c5_rhs194 = NULL;
  const mxArray *c5_lhs194 = NULL;
  const mxArray *c5_rhs195 = NULL;
  const mxArray *c5_lhs195 = NULL;
  const mxArray *c5_rhs196 = NULL;
  const mxArray *c5_lhs196 = NULL;
  const mxArray *c5_rhs197 = NULL;
  const mxArray *c5_lhs197 = NULL;
  const mxArray *c5_rhs198 = NULL;
  const mxArray *c5_lhs198 = NULL;
  const mxArray *c5_rhs199 = NULL;
  const mxArray *c5_lhs199 = NULL;
  const mxArray *c5_rhs200 = NULL;
  const mxArray *c5_lhs200 = NULL;
  const mxArray *c5_rhs201 = NULL;
  const mxArray *c5_lhs201 = NULL;
  const mxArray *c5_rhs202 = NULL;
  const mxArray *c5_lhs202 = NULL;
  const mxArray *c5_rhs203 = NULL;
  const mxArray *c5_lhs203 = NULL;
  const mxArray *c5_rhs204 = NULL;
  const mxArray *c5_lhs204 = NULL;
  const mxArray *c5_rhs205 = NULL;
  const mxArray *c5_lhs205 = NULL;
  const mxArray *c5_rhs206 = NULL;
  const mxArray *c5_lhs206 = NULL;
  const mxArray *c5_rhs207 = NULL;
  const mxArray *c5_lhs207 = NULL;
  const mxArray *c5_rhs208 = NULL;
  const mxArray *c5_lhs208 = NULL;
  const mxArray *c5_rhs209 = NULL;
  const mxArray *c5_lhs209 = NULL;
  const mxArray *c5_rhs210 = NULL;
  const mxArray *c5_lhs210 = NULL;
  const mxArray *c5_rhs211 = NULL;
  const mxArray *c5_lhs211 = NULL;
  const mxArray *c5_rhs212 = NULL;
  const mxArray *c5_lhs212 = NULL;
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 192);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.refblas.xgemm"),
                  "name", "name", 192);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 192);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 192);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 192);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 192);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 192);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 192);
  sf_mex_assign(&c5_rhs192, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs192, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs192), "rhs", "rhs",
                  192);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs192), "lhs", "lhs",
                  192);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 193);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 193);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 193);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 193);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 193);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 193);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 193);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 193);
  sf_mex_assign(&c5_rhs193, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs193, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs193), "rhs", "rhs",
                  193);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs193), "lhs", "lhs",
                  193);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 194);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 194);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 194);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 194);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 194);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 194);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 194);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 194);
  sf_mex_assign(&c5_rhs194, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs194, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs194), "rhs", "rhs",
                  194);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs194), "lhs", "lhs",
                  194);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 195);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 195);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 195);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 195);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 195);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 195);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 195);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 195);
  sf_mex_assign(&c5_rhs195, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs195, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs195), "rhs", "rhs",
                  195);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs195), "lhs", "lhs",
                  195);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 196);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 196);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 196);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 196);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 196);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 196);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 196);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 196);
  sf_mex_assign(&c5_rhs196, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs196, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs196), "rhs", "rhs",
                  196);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs196), "lhs", "lhs",
                  196);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 197);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 197);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 197);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 197);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 197);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 197);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 197);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 197);
  sf_mex_assign(&c5_rhs197, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs197, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs197), "rhs", "rhs",
                  197);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs197), "lhs", "lhs",
                  197);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "context", "context", 198);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 198);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 198);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 198);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 198);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 198);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 198);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 198);
  sf_mex_assign(&c5_rhs198, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs198, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs198), "rhs", "rhs",
                  198);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs198), "lhs", "lhs",
                  198);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 199);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.int"),
                  "name", "name", 199);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 199);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/int.p"),
                  "resolved", "resolved", 199);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 199);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 199);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 199);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 199);
  sf_mex_assign(&c5_rhs199, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs199, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs199), "rhs", "rhs",
                  199);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs199), "lhs", "lhs",
                  199);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!ceval_xgemm"),
                  "context", "context", 200);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.size_ptr"),
                  "name", "name", 200);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 200);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/size_ptr.p"),
                  "resolved", "resolved", 200);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 200);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 200);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 200);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 200);
  sf_mex_assign(&c5_rhs200, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs200, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs200), "rhs", "rhs",
                  200);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs200), "lhs", "lhs",
                  200);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!ceval_xgemm"),
                  "context", "context", 201);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.c_cast"),
                  "name", "name", 201);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("int32"), "dominantType",
                  "dominantType", 201);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/c_cast.p"),
                  "resolved", "resolved", 201);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 201);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 201);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 201);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 201);
  sf_mex_assign(&c5_rhs201, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs201, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs201), "rhs", "rhs",
                  201);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs201), "lhs", "lhs",
                  201);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!below_threshold"),
                  "context", "context", 202);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("length"), "name", "name", 202);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 202);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 202);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 202);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 202);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 202);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 202);
  sf_mex_assign(&c5_rhs202, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs202, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs202), "rhs", "rhs",
                  202);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs202), "lhs", "lhs",
                  202);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p!below_threshold"),
                  "context", "context", 203);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("length"), "name", "name", 203);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 203);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 203);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 203);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 203);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 203);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 203);
  sf_mex_assign(&c5_rhs203, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs203, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs203), "rhs", "rhs",
                  203);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs203), "lhs", "lhs",
                  203);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xdot.p!below_threshold"),
                  "context", "context", 204);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("length"), "name", "name", 204);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 204);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 204);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 204);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 204);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 204);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 204);
  sf_mex_assign(&c5_rhs204, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs204, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs204), "rhs", "rhs",
                  204);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs204), "lhs", "lhs",
                  204);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength"),
                  "context", "context", 205);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 205);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 205);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 205);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 205);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 205);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 205);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 205);
  sf_mex_assign(&c5_rhs205, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs205, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs205), "rhs", "rhs",
                  205);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs205), "lhs", "lhs",
                  205);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "context", "context", 206);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 206);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 206);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 206);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 206);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 206);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 206);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 206);
  sf_mex_assign(&c5_rhs206, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs206, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs206), "rhs", "rhs",
                  206);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs206), "lhs", "lhs",
                  206);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p!below_threshold"),
                  "context", "context", 207);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 207);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 207);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 207);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 207);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 207);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 207);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 207);
  sf_mex_assign(&c5_rhs207, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs207, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs207), "rhs", "rhs",
                  207);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs207), "lhs", "lhs",
                  207);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 208);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 208);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 208);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 208);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1383877294U), "fileTimeLo",
                  "fileTimeLo", 208);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 208);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 208);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 208);
  sf_mex_assign(&c5_rhs208, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs208, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs208), "rhs", "rhs",
                  208);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs208), "lhs", "lhs",
                  208);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 209);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 209);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 209);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 209);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 209);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 209);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 209);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 209);
  sf_mex_assign(&c5_rhs209, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs209, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs209), "rhs", "rhs",
                  209);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs209), "lhs", "lhs",
                  209);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 210);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 210);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 210);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 210);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 210);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 210);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 210);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 210);
  sf_mex_assign(&c5_rhs210, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs210, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs210), "rhs", "rhs",
                  210);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs210), "lhs", "lhs",
                  210);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 211);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 211);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 211);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 211);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 211);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 211);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 211);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 211);
  sf_mex_assign(&c5_rhs211, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs211, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs211), "rhs", "rhs",
                  211);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs211), "lhs", "lhs",
                  211);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 212);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  212);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 212);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 212);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980690U), "fileTimeLo",
                  "fileTimeLo", 212);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 212);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 212);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 212);
  sf_mex_assign(&c5_rhs212, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs212, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs212), "rhs", "rhs",
                  212);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs212), "lhs", "lhs",
                  212);
  sf_mex_destroy(&c5_rhs192);
  sf_mex_destroy(&c5_lhs192);
  sf_mex_destroy(&c5_rhs193);
  sf_mex_destroy(&c5_lhs193);
  sf_mex_destroy(&c5_rhs194);
  sf_mex_destroy(&c5_lhs194);
  sf_mex_destroy(&c5_rhs195);
  sf_mex_destroy(&c5_lhs195);
  sf_mex_destroy(&c5_rhs196);
  sf_mex_destroy(&c5_lhs196);
  sf_mex_destroy(&c5_rhs197);
  sf_mex_destroy(&c5_lhs197);
  sf_mex_destroy(&c5_rhs198);
  sf_mex_destroy(&c5_lhs198);
  sf_mex_destroy(&c5_rhs199);
  sf_mex_destroy(&c5_lhs199);
  sf_mex_destroy(&c5_rhs200);
  sf_mex_destroy(&c5_lhs200);
  sf_mex_destroy(&c5_rhs201);
  sf_mex_destroy(&c5_lhs201);
  sf_mex_destroy(&c5_rhs202);
  sf_mex_destroy(&c5_lhs202);
  sf_mex_destroy(&c5_rhs203);
  sf_mex_destroy(&c5_lhs203);
  sf_mex_destroy(&c5_rhs204);
  sf_mex_destroy(&c5_lhs204);
  sf_mex_destroy(&c5_rhs205);
  sf_mex_destroy(&c5_lhs205);
  sf_mex_destroy(&c5_rhs206);
  sf_mex_destroy(&c5_lhs206);
  sf_mex_destroy(&c5_rhs207);
  sf_mex_destroy(&c5_lhs207);
  sf_mex_destroy(&c5_rhs208);
  sf_mex_destroy(&c5_lhs208);
  sf_mex_destroy(&c5_rhs209);
  sf_mex_destroy(&c5_lhs209);
  sf_mex_destroy(&c5_rhs210);
  sf_mex_destroy(&c5_lhs210);
  sf_mex_destroy(&c5_rhs211);
  sf_mex_destroy(&c5_lhs211);
  sf_mex_destroy(&c5_rhs212);
  sf_mex_destroy(&c5_lhs212);
}

static void c5_pinv(SFc5_balancingController_with_OOT_and_transferInstanceStruct
                    *chartInstance, real_T c5_A_data[], int32_T c5_A_sizes[2],
                    real_T c5_X_data[], int32_T c5_X_sizes[2])
{
  int32_T c5_m;
  int32_T c5_iv0[2];
  int32_T c5_iv1[2];
  int32_T c5_X;
  int32_T c5_b_X;
  int32_T c5_loop_ub;
  int32_T c5_i62;
  int32_T c5_i63;
  int32_T c5_b;
  int32_T c5_b_b;
  boolean_T c5_overflow;
  int32_T c5_k;
  int32_T c5_b_k;
  int32_T c5_b_A_sizes[2];
  int32_T c5_A;
  int32_T c5_b_A;
  int32_T c5_b_loop_ub;
  int32_T c5_i64;
  real_T c5_b_A_data[72];
  int32_T c5_V_sizes[2];
  real_T c5_V_data[36];
  int32_T c5_s_sizes;
  real_T c5_s_data[6];
  int32_T c5_U_sizes[2];
  real_T c5_U_data[72];
  int32_T c5_S_sizes[2];
  int32_T c5_S;
  int32_T c5_b_S;
  int32_T c5_i65;
  real_T c5_S_data[36];
  int32_T c5_c_k;
  real_T c5_d_k;
  real_T c5_tol;
  int32_T c5_r;
  int32_T c5_e_k;
  int32_T c5_f_k;
  int32_T c5_a;
  int32_T c5_b_a;
  int32_T c5_vcol;
  int32_T c5_b_r;
  int32_T c5_c_b;
  int32_T c5_d_b;
  boolean_T c5_b_overflow;
  int32_T c5_j;
  int32_T c5_b_j;
  real_T c5_y;
  real_T c5_b_y;
  real_T c5_z;
  int32_T c5_c_a;
  int32_T c5_d_a;
  int32_T c5_b_V_sizes[2];
  int32_T c5_V;
  int32_T c5_b_V;
  int32_T c5_i66;
  real_T c5_b_V_data[36];
  int32_T c5_b_U_sizes[2];
  int32_T c5_U;
  int32_T c5_b_U;
  int32_T c5_c_loop_ub;
  int32_T c5_i67;
  real_T c5_b_U_data[72];
  boolean_T exitg1;
  c5_m = c5_A_sizes[0];
  c5_iv0[0] = 6;
  c5_iv0[1] = c5_m;
  c5_X_sizes[0] = 6;
  c5_iv1[0] = 6;
  c5_iv1[1] = c5_iv0[1];
  c5_X_sizes[1] = c5_iv1[1];
  c5_X = c5_X_sizes[0];
  c5_b_X = c5_X_sizes[1];
  c5_loop_ub = 6 * c5_iv0[1] - 1;
  for (c5_i62 = 0; c5_i62 <= c5_loop_ub; c5_i62++) {
    c5_X_data[c5_i62] = 0.0;
  }

  c5_i63 = c5_A_sizes[0] * 6;
  c5_b = c5_i63;
  c5_b_b = c5_b;
  if (1 > c5_b_b) {
    c5_overflow = false;
  } else {
    c5_eml_switch_helper(chartInstance);
    c5_overflow = (c5_b_b > 2147483646);
  }

  if (c5_overflow) {
    c5_check_forloop_overflow_error(chartInstance, true);
  }

  for (c5_k = 1; c5_k <= c5_i63; c5_k++) {
    c5_b_k = c5_k;
    if (!c5_isfinite(chartInstance, c5_A_data[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          c5_b_k, 1, c5_A_sizes[0] * 6, 1, 0) - 1])) {
      c5_eml_error(chartInstance);
    }
  }

  c5_b_A_sizes[0] = c5_A_sizes[0];
  c5_b_A_sizes[1] = 6;
  c5_A = c5_b_A_sizes[0];
  c5_b_A = c5_b_A_sizes[1];
  c5_b_loop_ub = c5_A_sizes[0] * c5_A_sizes[1] - 1;
  for (c5_i64 = 0; c5_i64 <= c5_b_loop_ub; c5_i64++) {
    c5_b_A_data[c5_i64] = c5_A_data[c5_i64];
  }

  c5_eml_xgesvd(chartInstance, c5_b_A_data, c5_b_A_sizes, c5_U_data, c5_U_sizes,
                c5_s_data, &c5_s_sizes, c5_V_data, c5_V_sizes);
  c5_S_sizes[0] = 6;
  c5_S_sizes[1] = 6;
  c5_S = c5_S_sizes[0];
  c5_b_S = c5_S_sizes[1];
  for (c5_i65 = 0; c5_i65 < 36; c5_i65++) {
    c5_S_data[c5_i65] = 0.0;
  }

  for (c5_c_k = 0; c5_c_k < 6; c5_c_k++) {
    c5_d_k = 1.0 + (real_T)c5_c_k;
    c5_S_data[((int32_T)c5_d_k + c5_S_sizes[0] * ((int32_T)c5_d_k - 1)) - 1] =
      c5_s_data[(int32_T)c5_d_k - 1];
  }

  c5_eps(chartInstance);
  c5_tol = (real_T)c5_m * c5_S_data[0] * 2.2204460492503131E-16;
  c5_r = 0;
  c5_e_k = 1;
  exitg1 = false;
  while ((exitg1 == false) && (c5_e_k < 7)) {
    c5_f_k = c5_e_k - 1;
    if (!(c5_S_data[c5_f_k + c5_S_sizes[0] * c5_f_k] > c5_tol)) {
      exitg1 = true;
    } else {
      c5_a = c5_r;
      c5_b_a = c5_a + 1;
      c5_r = c5_b_a;
      c5_e_k++;
    }
  }

  if (c5_r > 0) {
    c5_vcol = 1;
    c5_b_r = c5_r;
    c5_c_b = c5_b_r;
    c5_d_b = c5_c_b;
    if (1 > c5_d_b) {
      c5_b_overflow = false;
    } else {
      c5_eml_switch_helper(chartInstance);
      c5_b_overflow = (c5_d_b > 2147483646);
    }

    if (c5_b_overflow) {
      c5_check_forloop_overflow_error(chartInstance, true);
    }

    for (c5_j = 1; c5_j <= c5_b_r; c5_j++) {
      c5_b_j = c5_j;
      c5_y = c5_S_data[(_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_j, 1, 6, 1, 0) +
                        c5_S_sizes[0] * (_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_j,
        1, 6, 2, 0) - 1)) - 1];
      c5_b_y = c5_y;
      c5_z = 1.0 / c5_b_y;
      c5_f_eml_xscal(chartInstance, c5_z, c5_V_data, c5_V_sizes, c5_vcol);
      c5_c_a = c5_vcol;
      c5_d_a = c5_c_a + 6;
      c5_vcol = c5_d_a;
    }

    c5_b_V_sizes[0] = 6;
    c5_b_V_sizes[1] = 6;
    c5_V = c5_b_V_sizes[0];
    c5_b_V = c5_b_V_sizes[1];
    for (c5_i66 = 0; c5_i66 < 36; c5_i66++) {
      c5_b_V_data[c5_i66] = c5_V_data[c5_i66];
    }

    c5_b_U_sizes[0] = c5_U_sizes[0];
    c5_b_U_sizes[1] = 6;
    c5_U = c5_b_U_sizes[0];
    c5_b_U = c5_b_U_sizes[1];
    c5_c_loop_ub = c5_U_sizes[0] * c5_U_sizes[1] - 1;
    for (c5_i67 = 0; c5_i67 <= c5_c_loop_ub; c5_i67++) {
      c5_b_U_data[c5_i67] = c5_U_data[c5_i67];
    }

    c5_c_eml_xgemm(chartInstance, c5_m, c5_r, c5_b_V_data, c5_b_V_sizes,
                   c5_b_U_data, c5_b_U_sizes, c5_m, c5_X_data, c5_X_sizes);
  }
}

static void c5_eml_switch_helper
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_check_forloop_overflow_error
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   boolean_T c5_overflow)
{
  int32_T c5_i68;
  static char_T c5_cv1[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c5_u[34];
  const mxArray *c5_y = NULL;
  int32_T c5_i69;
  static char_T c5_cv2[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c5_b_u[23];
  const mxArray *c5_b_y = NULL;
  (void)chartInstance;
  (void)c5_overflow;
  for (c5_i68 = 0; c5_i68 < 34; c5_i68++) {
    c5_u[c5_i68] = c5_cv1[c5_i68];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 10, 0U, 1U, 0U, 2, 1, 34), false);
  for (c5_i69 = 0; c5_i69 < 23; c5_i69++) {
    c5_b_u[c5_i69] = c5_cv2[c5_i69];
  }

  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", c5_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c5_y, 14, c5_b_y));
}

static boolean_T c5_isfinite
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x)
{
  real_T c5_b_x;
  boolean_T c5_b_b;
  boolean_T c5_b0;
  real_T c5_c_x;
  boolean_T c5_c_b;
  boolean_T c5_b1;
  (void)chartInstance;
  c5_b_x = c5_x;
  c5_b_b = muDoubleScalarIsInf(c5_b_x);
  c5_b0 = !c5_b_b;
  c5_c_x = c5_x;
  c5_c_b = muDoubleScalarIsNaN(c5_c_x);
  c5_b1 = !c5_c_b;
  return c5_b0 && c5_b1;
}

static void c5_eml_error
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  int32_T c5_i70;
  static char_T c5_cv3[33] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 's', 'v', 'd', '_', 'm', 'a', 't', 'r', 'i', 'x', 'W', 'i',
    't', 'h', 'N', 'a', 'N', 'I', 'n', 'f' };

  char_T c5_u[33];
  const mxArray *c5_y = NULL;
  (void)chartInstance;
  for (c5_i70 = 0; c5_i70 < 33; c5_i70++) {
    c5_u[c5_i70] = c5_cv3[c5_i70];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 10, 0U, 1U, 0U, 2, 1, 33), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c5_y));
}

static void c5_eml_scalar_eg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_b_eml_scalar_eg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_threshold
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c5_abs
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x)
{
  real_T c5_b_x;
  (void)chartInstance;
  c5_b_x = c5_x;
  return muDoubleScalarAbs(c5_b_x);
}

static void c5_realmin
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c5_eml_div
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x, real_T c5_y)
{
  real_T c5_b_x;
  real_T c5_b_y;
  (void)chartInstance;
  c5_b_x = c5_x;
  c5_b_y = c5_y;
  return c5_b_x / c5_b_y;
}

static void c5_b_threshold
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_c_threshold
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_d_threshold
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c5_eml_xdotc
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0,
   real_T c5_y_data[], int32_T c5_y_sizes[2], int32_T c5_iy0)
{
  real_T c5_d;
  int32_T c5_b_n;
  int32_T c5_b_ix0;
  int32_T c5_b_iy0;
  int32_T c5_c_n;
  int32_T c5_c_ix0;
  int32_T c5_c_iy0;
  int32_T c5_d_n;
  int32_T c5_d_ix0;
  int32_T c5_d_iy0;
  int32_T c5_var;
  ptrdiff_t c5_n_t;
  ptrdiff_t c5_incx_t;
  ptrdiff_t c5_incy_t;
  double * c5_xix0_t;
  double * c5_yiy0_t;
  c5_b_n = c5_n;
  c5_b_ix0 = c5_ix0;
  c5_b_iy0 = c5_iy0;
  c5_c_n = c5_b_n;
  c5_c_ix0 = c5_b_ix0;
  c5_c_iy0 = c5_b_iy0;
  c5_c_threshold(chartInstance);
  if (c5_c_n < 1) {
    c5_scalarEg(chartInstance);
    c5_d = 0.0;
  } else {
    c5_d_n = c5_c_n;
    c5_d_ix0 = c5_c_ix0;
    c5_d_iy0 = c5_c_iy0;
    c5_var = c5_d_n;
    c5_n_t = (ptrdiff_t)(c5_var);
    c5_incx_t = (ptrdiff_t)(1);
    c5_incy_t = (ptrdiff_t)(1);
    c5_xix0_t = (double *)(&c5_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_d_ix0,
      1, c5_x_sizes[0] * 6, 1, 0) - 1]);
    c5_yiy0_t = (double *)(&c5_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_d_iy0,
      1, c5_y_sizes[0] * 6, 1, 0) - 1]);
    c5_d = ddot(&c5_n_t, c5_xix0_t, &c5_incx_t, c5_yiy0_t, &c5_incy_t);
  }

  return c5_d;
}

static void c5_scalarEg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, int32_T c5_ix0, real_T c5_y_data[], int32_T
   c5_y_sizes[2], int32_T c5_iy0, real_T c5_b_y_data[], int32_T c5_b_y_sizes[2])
{
  int32_T c5_y;
  int32_T c5_b_y;
  int32_T c5_loop_ub;
  int32_T c5_i71;
  c5_b_y_sizes[0] = c5_y_sizes[0];
  c5_b_y_sizes[1] = 6;
  c5_y = c5_b_y_sizes[0];
  c5_b_y = c5_b_y_sizes[1];
  c5_loop_ub = c5_y_sizes[0] * c5_y_sizes[1] - 1;
  for (c5_i71 = 0; c5_i71 <= c5_loop_ub; c5_i71++) {
    c5_b_y_data[c5_i71] = c5_y_data[c5_i71];
  }

  c5_f_eml_xaxpy(chartInstance, c5_n, c5_a, c5_ix0, c5_b_y_data, c5_b_y_sizes,
                 c5_iy0);
}

static void c5_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0,
   real_T c5_b_x_data[], int32_T c5_b_x_sizes[2])
{
  int32_T c5_x;
  int32_T c5_b_x;
  int32_T c5_i72;
  (void)c5_x_sizes;
  c5_b_x_sizes[0] = 6;
  c5_b_x_sizes[1] = 6;
  c5_x = c5_b_x_sizes[0];
  c5_b_x = c5_b_x_sizes[1];
  for (c5_i72 = 0; c5_i72 < 36; c5_i72++) {
    c5_b_x_data[c5_i72] = c5_x_data[c5_i72];
  }

  c5_f_eml_xscal(chartInstance, c5_a, c5_b_x_data, c5_b_x_sizes, c5_ix0);
}

static void c5_b_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T
   c5_ix0, real_T c5_b_x_data[], int32_T c5_b_x_sizes[2])
{
  int32_T c5_x;
  int32_T c5_b_x;
  int32_T c5_loop_ub;
  int32_T c5_i73;
  c5_b_x_sizes[0] = c5_x_sizes[0];
  c5_b_x_sizes[1] = 6;
  c5_x = c5_b_x_sizes[0];
  c5_b_x = c5_b_x_sizes[1];
  c5_loop_ub = c5_x_sizes[0] * c5_x_sizes[1] - 1;
  for (c5_i73 = 0; c5_i73 <= c5_loop_ub; c5_i73++) {
    c5_b_x_data[c5_i73] = c5_x_data[c5_i73];
  }

  c5_g_eml_xscal(chartInstance, c5_n, c5_a, c5_b_x_data, c5_b_x_sizes, c5_ix0);
}

static void c5_eps(SFc5_balancingController_with_OOT_and_transferInstanceStruct *
                   chartInstance)
{
  (void)chartInstance;
}

static void c5_c_eml_scalar_eg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_b_eml_error
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  int32_T c5_i74;
  static char_T c5_cv4[30] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 's', 'v', 'd', '_', 'N', 'o', 'C', 'o', 'n', 'v', 'e', 'r',
    'g', 'e', 'n', 'c', 'e' };

  char_T c5_u[30];
  const mxArray *c5_y = NULL;
  (void)chartInstance;
  for (c5_i74 = 0; c5_i74 < 30; c5_i74++) {
    c5_u[c5_i74] = c5_cv4[c5_i74];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c5_y));
}

static real_T c5_sqrt
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x)
{
  real_T c5_b_x;
  c5_b_x = c5_x;
  c5_b_sqrt(chartInstance, &c5_b_x);
  return c5_b_x;
}

static void c5_c_eml_error
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  int32_T c5_i75;
  static char_T c5_cv5[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c5_u[30];
  const mxArray *c5_y = NULL;
  int32_T c5_i76;
  static char_T c5_cv6[4] = { 's', 'q', 'r', 't' };

  char_T c5_b_u[4];
  const mxArray *c5_b_y = NULL;
  (void)chartInstance;
  for (c5_i75 = 0; c5_i75 < 30; c5_i75++) {
    c5_u[c5_i75] = c5_cv5[c5_i75];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c5_i76 = 0; c5_i76 < 4; c5_i76++) {
    c5_b_u[c5_i76] = c5_cv6[c5_i76];
  }

  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", c5_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c5_y, 14, c5_b_y));
}

static void c5_eml_xrotg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_a, real_T c5_b, real_T *c5_b_a, real_T *c5_b_b, real_T *c5_c,
   real_T *c5_s)
{
  *c5_b_a = c5_a;
  *c5_b_b = c5_b;
  c5_b_eml_xrotg(chartInstance, c5_b_a, c5_b_b, c5_c, c5_s);
}

static void c5_eml_xrot
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0,
   int32_T c5_iy0, real_T c5_c, real_T c5_s, real_T c5_b_x_data[], int32_T
   c5_b_x_sizes[2])
{
  int32_T c5_x;
  int32_T c5_b_x;
  int32_T c5_loop_ub;
  int32_T c5_i77;
  c5_b_x_sizes[0] = c5_x_sizes[0];
  c5_b_x_sizes[1] = 6;
  c5_x = c5_b_x_sizes[0];
  c5_b_x = c5_b_x_sizes[1];
  c5_loop_ub = c5_x_sizes[0] * c5_x_sizes[1] - 1;
  for (c5_i77 = 0; c5_i77 <= c5_loop_ub; c5_i77++) {
    c5_b_x_data[c5_i77] = c5_x_data[c5_i77];
  }

  c5_c_eml_xrot(chartInstance, c5_n, c5_b_x_data, c5_b_x_sizes, c5_ix0, c5_iy0,
                c5_c, c5_s);
}

static void c5_e_threshold
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_eml_xswap
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0,
   int32_T c5_iy0, real_T c5_b_x_data[], int32_T c5_b_x_sizes[2])
{
  int32_T c5_x;
  int32_T c5_b_x;
  int32_T c5_loop_ub;
  int32_T c5_i78;
  c5_b_x_sizes[0] = c5_x_sizes[0];
  c5_b_x_sizes[1] = 6;
  c5_x = c5_b_x_sizes[0];
  c5_b_x = c5_b_x_sizes[1];
  c5_loop_ub = c5_x_sizes[0] * c5_x_sizes[1] - 1;
  for (c5_i78 = 0; c5_i78 <= c5_loop_ub; c5_i78++) {
    c5_b_x_data[c5_i78] = c5_x_data[c5_i78];
  }

  c5_c_eml_xswap(chartInstance, c5_n, c5_b_x_data, c5_b_x_sizes, c5_ix0, c5_iy0);
}

static void c5_f_threshold
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_d_eml_scalar_eg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_eml_xgesvd
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_A_data[], int32_T c5_A_sizes[2], real_T c5_U_data[], int32_T
   c5_U_sizes[2], real_T c5_S_data[], int32_T *c5_S_sizes, real_T c5_V_data[],
   int32_T c5_V_sizes[2])
{
  int32_T c5_b_A_sizes[2];
  int32_T c5_A;
  int32_T c5_b_A;
  int32_T c5_loop_ub;
  int32_T c5_i79;
  real_T c5_b_A_data[72];
  int32_T c5_n;
  int32_T c5_nru;
  int32_T c5_s_sizes;
  int32_T c5_i80;
  real_T c5_s_data[6];
  int32_T c5_i81;
  real_T c5_e[6];
  int32_T c5_iv2[2];
  int32_T c5_work_sizes;
  int32_T c5_b_loop_ub;
  int32_T c5_i82;
  real_T c5_work_data[12];
  int32_T c5_iv3[2];
  int32_T c5_U;
  int32_T c5_b_U;
  int32_T c5_c_loop_ub;
  int32_T c5_i83;
  int32_T c5_i84;
  real_T c5_Vf[36];
  int32_T c5_varargin_1;
  int32_T c5_varargin_2;
  int32_T c5_x;
  int32_T c5_b_x;
  int32_T c5_xk;
  int32_T c5_c_x;
  int32_T c5_maxval;
  int32_T c5_a;
  int32_T c5_b_a;
  int32_T c5_c;
  int32_T c5_b_varargin_1;
  int32_T c5_b_varargin_2;
  int32_T c5_d_x;
  int32_T c5_e_x;
  int32_T c5_b_xk;
  int32_T c5_f_x;
  int32_T c5_nct;
  int32_T c5_c_a;
  int32_T c5_d_a;
  int32_T c5_nctp1;
  int32_T c5_c_varargin_1;
  int32_T c5_c_varargin_2;
  int32_T c5_g_x;
  int32_T c5_h_x;
  int32_T c5_c_xk;
  int32_T c5_i_x;
  int32_T c5_i85;
  int32_T c5_b;
  int32_T c5_b_b;
  boolean_T c5_overflow;
  int32_T c5_q;
  int32_T c5_b_q;
  int32_T c5_e_a;
  int32_T c5_f_a;
  int32_T c5_qp1;
  int32_T c5_g_a;
  int32_T c5_h_a;
  int32_T c5_qm1;
  int32_T c5_i_a;
  int32_T c5_c_b;
  int32_T c5_j_a;
  int32_T c5_d_b;
  int32_T c5_b_c;
  int32_T c5_k_a;
  int32_T c5_e_b;
  int32_T c5_l_a;
  int32_T c5_f_b;
  int32_T c5_qq;
  int32_T c5_m_a;
  int32_T c5_g_b;
  int32_T c5_n_a;
  int32_T c5_h_b;
  int32_T c5_nmq;
  int32_T c5_o_a;
  int32_T c5_p_a;
  int32_T c5_nmqp1;
  int32_T c5_c_A_sizes[2];
  int32_T c5_c_A;
  int32_T c5_d_A;
  int32_T c5_d_loop_ub;
  int32_T c5_i86;
  real_T c5_c_A_data[72];
  real_T c5_nrm;
  real_T c5_absx;
  real_T c5_d;
  real_T c5_y;
  real_T c5_d1;
  int32_T c5_b_qp1;
  int32_T c5_q_a;
  int32_T c5_r_a;
  int32_T c5_jj;
  int32_T c5_b_jj;
  int32_T c5_s_a;
  int32_T c5_t_a;
  int32_T c5_c_c;
  int32_T c5_u_a;
  int32_T c5_i_b;
  int32_T c5_v_a;
  int32_T c5_j_b;
  int32_T c5_d_c;
  int32_T c5_w_a;
  int32_T c5_k_b;
  int32_T c5_x_a;
  int32_T c5_l_b;
  int32_T c5_qjj;
  int32_T c5_d_A_sizes[2];
  int32_T c5_e_A;
  int32_T c5_f_A;
  int32_T c5_e_loop_ub;
  int32_T c5_i87;
  real_T c5_d_A_data[72];
  int32_T c5_e_A_sizes[2];
  int32_T c5_g_A;
  int32_T c5_h_A;
  int32_T c5_f_loop_ub;
  int32_T c5_i88;
  real_T c5_e_A_data[72];
  real_T c5_t;
  int32_T c5_c_q;
  int32_T c5_b_n;
  int32_T c5_y_a;
  int32_T c5_m_b;
  int32_T c5_ab_a;
  int32_T c5_n_b;
  boolean_T c5_b_overflow;
  int32_T c5_ii;
  int32_T c5_b_ii;
  int32_T c5_o_b;
  int32_T c5_p_b;
  int32_T c5_pmq;
  int32_T c5_i89;
  real_T c5_b_e[6];
  real_T c5_b_absx;
  real_T c5_b_d;
  real_T c5_b_y;
  real_T c5_d2;
  int32_T c5_c_qp1;
  int32_T c5_c_n;
  int32_T c5_bb_a;
  int32_T c5_q_b;
  int32_T c5_cb_a;
  int32_T c5_r_b;
  boolean_T c5_c_overflow;
  int32_T c5_c_ii;
  int32_T c5_d_qp1;
  int32_T c5_db_a;
  int32_T c5_eb_a;
  int32_T c5_c_jj;
  int32_T c5_fb_a;
  int32_T c5_gb_a;
  int32_T c5_e_c;
  int32_T c5_hb_a;
  int32_T c5_s_b;
  int32_T c5_ib_a;
  int32_T c5_t_b;
  int32_T c5_f_c;
  int32_T c5_jb_a;
  int32_T c5_u_b;
  int32_T c5_kb_a;
  int32_T c5_v_b;
  int32_T c5_qp1jj;
  int32_T c5_f_A_sizes[2];
  int32_T c5_i_A;
  int32_T c5_j_A;
  int32_T c5_g_loop_ub;
  int32_T c5_i90;
  real_T c5_f_A_data[72];
  int32_T c5_e_qp1;
  int32_T c5_lb_a;
  int32_T c5_mb_a;
  int32_T c5_d_jj;
  int32_T c5_nb_a;
  int32_T c5_ob_a;
  int32_T c5_g_c;
  int32_T c5_pb_a;
  int32_T c5_w_b;
  int32_T c5_qb_a;
  int32_T c5_x_b;
  int32_T c5_h_c;
  int32_T c5_rb_a;
  int32_T c5_y_b;
  int32_T c5_sb_a;
  int32_T c5_ab_b;
  int32_T c5_b_work_sizes;
  int32_T c5_h_loop_ub;
  int32_T c5_i91;
  real_T c5_b_work_data[12];
  int32_T c5_f_qp1;
  int32_T c5_tb_a;
  int32_T c5_ub_a;
  int32_T c5_d_ii;
  int32_T c5_vb_a;
  int32_T c5_wb_a;
  int32_T c5_i_c;
  int32_T c5_d_varargin_2;
  int32_T c5_varargin_3;
  int32_T c5_c_y;
  int32_T c5_d_y;
  int32_T c5_yk;
  int32_T c5_e_y;
  int32_T c5_m;
  int32_T c5_b_nctp1;
  int32_T c5_xb_a;
  int32_T c5_yb_a;
  boolean_T c5_d_overflow;
  int32_T c5_e_jj;
  int32_T c5_b_nru;
  int32_T c5_bb_b;
  int32_T c5_cb_b;
  boolean_T c5_e_overflow;
  int32_T c5_e_ii;
  int32_T c5_b_nct;
  int32_T c5_ac_a;
  int32_T c5_bc_a;
  int32_T c5_d_q;
  int32_T c5_cc_a;
  int32_T c5_dc_a;
  int32_T c5_ec_a;
  int32_T c5_db_b;
  int32_T c5_fc_a;
  int32_T c5_eb_b;
  int32_T c5_gc_a;
  int32_T c5_hc_a;
  int32_T c5_ic_a;
  int32_T c5_jc_a;
  int32_T c5_j_c;
  int32_T c5_kc_a;
  int32_T c5_fb_b;
  int32_T c5_lc_a;
  int32_T c5_gb_b;
  int32_T c5_k_c;
  int32_T c5_mc_a;
  int32_T c5_hb_b;
  int32_T c5_nc_a;
  int32_T c5_ib_b;
  int32_T c5_g_qp1;
  int32_T c5_oc_a;
  int32_T c5_pc_a;
  boolean_T c5_f_overflow;
  int32_T c5_f_jj;
  int32_T c5_qc_a;
  int32_T c5_rc_a;
  int32_T c5_l_c;
  int32_T c5_sc_a;
  int32_T c5_jb_b;
  int32_T c5_tc_a;
  int32_T c5_kb_b;
  int32_T c5_m_c;
  int32_T c5_uc_a;
  int32_T c5_lb_b;
  int32_T c5_vc_a;
  int32_T c5_mb_b;
  int32_T c5_b_U_sizes[2];
  int32_T c5_c_U;
  int32_T c5_d_U;
  int32_T c5_i_loop_ub;
  int32_T c5_i92;
  real_T c5_b_U_data[72];
  int32_T c5_c_U_sizes[2];
  int32_T c5_e_U;
  int32_T c5_f_U;
  int32_T c5_j_loop_ub;
  int32_T c5_i93;
  real_T c5_c_U_data[72];
  int32_T c5_e_q;
  int32_T c5_d_n;
  int32_T c5_wc_a;
  int32_T c5_nb_b;
  int32_T c5_xc_a;
  int32_T c5_ob_b;
  boolean_T c5_g_overflow;
  int32_T c5_f_ii;
  int32_T c5_yc_a;
  int32_T c5_ad_a;
  int32_T c5_i94;
  int32_T c5_pb_b;
  int32_T c5_qb_b;
  boolean_T c5_h_overflow;
  int32_T c5_g_ii;
  int32_T c5_e_n;
  int32_T c5_rb_b;
  int32_T c5_sb_b;
  boolean_T c5_i_overflow;
  int32_T c5_h_ii;
  int32_T c5_f_q;
  int32_T c5_bd_a;
  int32_T c5_cd_a;
  int32_T c5_tb_b;
  int32_T c5_ub_b;
  int32_T c5_dd_a;
  int32_T c5_ed_a;
  int32_T c5_n_c;
  int32_T c5_vb_b;
  int32_T c5_wb_b;
  int32_T c5_o_c;
  int32_T c5_fd_a;
  int32_T c5_xb_b;
  int32_T c5_gd_a;
  int32_T c5_yb_b;
  int32_T c5_qp1q;
  int32_T c5_h_qp1;
  int32_T c5_hd_a;
  int32_T c5_id_a;
  int32_T c5_g_jj;
  int32_T c5_jd_a;
  int32_T c5_kd_a;
  int32_T c5_p_c;
  int32_T c5_ac_b;
  int32_T c5_bc_b;
  int32_T c5_q_c;
  int32_T c5_ld_a;
  int32_T c5_cc_b;
  int32_T c5_md_a;
  int32_T c5_dc_b;
  int32_T c5_i95;
  real_T c5_b_Vf[36];
  int32_T c5_i96;
  real_T c5_c_Vf[36];
  int32_T c5_i_ii;
  int32_T c5_b_m;
  int32_T c5_ec_b;
  int32_T c5_fc_b;
  boolean_T c5_j_overflow;
  int32_T c5_g_q;
  real_T c5_rt;
  real_T c5_r;
  int32_T c5_nd_a;
  int32_T c5_od_a;
  int32_T c5_r_c;
  int32_T c5_pd_a;
  int32_T c5_gc_b;
  int32_T c5_qd_a;
  int32_T c5_hc_b;
  int32_T c5_s_c;
  int32_T c5_ic_b;
  int32_T c5_jc_b;
  int32_T c5_colq;
  int32_T c5_rd_a;
  int32_T c5_sd_a;
  int32_T c5_t_c;
  int32_T c5_td_a;
  int32_T c5_ud_a;
  int32_T c5_u_c;
  int32_T c5_kc_b;
  int32_T c5_lc_b;
  int32_T c5_v_c;
  int32_T c5_mc_b;
  int32_T c5_nc_b;
  int32_T c5_colqp1;
  int32_T c5_mm;
  real_T c5_iter;
  real_T c5_tiny;
  real_T c5_snorm;
  int32_T c5_c_m;
  int32_T c5_oc_b;
  int32_T c5_pc_b;
  boolean_T c5_k_overflow;
  int32_T c5_j_ii;
  real_T c5_d_varargin_1;
  real_T c5_e_varargin_2;
  real_T c5_f_varargin_2;
  real_T c5_b_varargin_3;
  real_T c5_j_x;
  real_T c5_f_y;
  real_T c5_k_x;
  real_T c5_g_y;
  real_T c5_d_xk;
  real_T c5_b_yk;
  real_T c5_l_x;
  real_T c5_h_y;
  real_T c5_b_maxval;
  real_T c5_e_varargin_1;
  real_T c5_g_varargin_2;
  real_T c5_h_varargin_2;
  real_T c5_c_varargin_3;
  real_T c5_m_x;
  real_T c5_i_y;
  real_T c5_n_x;
  real_T c5_j_y;
  real_T c5_e_xk;
  real_T c5_c_yk;
  real_T c5_o_x;
  real_T c5_k_y;
  int32_T c5_vd_a;
  int32_T c5_wd_a;
  int32_T c5_xd_a;
  int32_T c5_yd_a;
  int32_T c5_i97;
  int32_T c5_ae_a;
  int32_T c5_be_a;
  int32_T c5_k_ii;
  int32_T c5_ce_a;
  int32_T c5_de_a;
  int32_T c5_w_c;
  real_T c5_test0;
  real_T c5_ztest0;
  int32_T c5_ee_a;
  int32_T c5_fe_a;
  int32_T c5_x_c;
  real_T c5_kase;
  int32_T c5_qs;
  int32_T c5_d_m;
  int32_T c5_h_q;
  int32_T c5_ge_a;
  int32_T c5_qc_b;
  int32_T c5_he_a;
  int32_T c5_rc_b;
  boolean_T c5_l_overflow;
  int32_T c5_l_ii;
  real_T c5_test;
  int32_T c5_ie_a;
  int32_T c5_je_a;
  int32_T c5_y_c;
  int32_T c5_ke_a;
  int32_T c5_le_a;
  int32_T c5_ab_c;
  real_T c5_ztest;
  int32_T c5_me_a;
  int32_T c5_ne_a;
  int32_T c5_oe_a;
  int32_T c5_pe_a;
  int32_T c5_bb_c;
  real_T c5_f;
  int32_T c5_qe_a;
  int32_T c5_re_a;
  int32_T c5_cb_c;
  int32_T c5_se_a;
  int32_T c5_te_a;
  int32_T c5_i98;
  int32_T c5_i_q;
  int32_T c5_ue_a;
  int32_T c5_sc_b;
  int32_T c5_ve_a;
  int32_T c5_tc_b;
  boolean_T c5_m_overflow;
  int32_T c5_k;
  int32_T c5_b_k;
  real_T c5_t1;
  real_T c5_b_t1;
  real_T c5_b_f;
  real_T c5_sn;
  real_T c5_cs;
  real_T c5_b_cs;
  real_T c5_b_sn;
  int32_T c5_we_a;
  int32_T c5_xe_a;
  int32_T c5_km1;
  int32_T c5_ye_a;
  int32_T c5_af_a;
  int32_T c5_db_c;
  int32_T c5_uc_b;
  int32_T c5_vc_b;
  int32_T c5_eb_c;
  int32_T c5_wc_b;
  int32_T c5_xc_b;
  int32_T c5_colk;
  int32_T c5_bf_a;
  int32_T c5_cf_a;
  int32_T c5_fb_c;
  int32_T c5_yc_b;
  int32_T c5_ad_b;
  int32_T c5_gb_c;
  int32_T c5_bd_b;
  int32_T c5_cd_b;
  int32_T c5_colm;
  int32_T c5_df_a;
  int32_T c5_ef_a;
  int32_T c5_j_q;
  int32_T c5_e_m;
  int32_T c5_ff_a;
  int32_T c5_dd_b;
  int32_T c5_gf_a;
  int32_T c5_ed_b;
  boolean_T c5_n_overflow;
  int32_T c5_c_k;
  real_T c5_c_t1;
  real_T c5_unusedU0;
  real_T c5_c_sn;
  real_T c5_c_cs;
  int32_T c5_hf_a;
  int32_T c5_if_a;
  int32_T c5_hb_c;
  int32_T c5_jf_a;
  int32_T c5_fd_b;
  int32_T c5_kf_a;
  int32_T c5_gd_b;
  int32_T c5_ib_c;
  int32_T c5_hd_b;
  int32_T c5_id_b;
  int32_T c5_lf_a;
  int32_T c5_mf_a;
  int32_T c5_jb_c;
  int32_T c5_nf_a;
  int32_T c5_jd_b;
  int32_T c5_of_a;
  int32_T c5_kd_b;
  int32_T c5_kb_c;
  int32_T c5_ld_b;
  int32_T c5_md_b;
  int32_T c5_colqm1;
  int32_T c5_pf_a;
  int32_T c5_qf_a;
  int32_T c5_mm1;
  real_T c5_d3;
  real_T c5_d4;
  real_T c5_d5;
  real_T c5_d6;
  real_T c5_d7;
  real_T c5_f_varargin_1[5];
  int32_T c5_ixstart;
  real_T c5_mtmp;
  real_T c5_p_x;
  boolean_T c5_nd_b;
  int32_T c5_ix;
  int32_T c5_b_ix;
  real_T c5_q_x;
  boolean_T c5_od_b;
  int32_T c5_rf_a;
  int32_T c5_sf_a;
  int32_T c5_i99;
  int32_T c5_tf_a;
  int32_T c5_uf_a;
  int32_T c5_c_ix;
  real_T c5_vf_a;
  real_T c5_pd_b;
  boolean_T c5_p;
  real_T c5_b_mtmp;
  real_T c5_scale;
  real_T c5_sm;
  real_T c5_smm1;
  real_T c5_emm1;
  real_T c5_sqds;
  real_T c5_eqds;
  real_T c5_qd_b;
  real_T c5_lb_c;
  real_T c5_shift;
  real_T c5_g;
  int32_T c5_k_q;
  int32_T c5_b_mm1;
  int32_T c5_wf_a;
  int32_T c5_rd_b;
  int32_T c5_xf_a;
  int32_T c5_sd_b;
  boolean_T c5_o_overflow;
  int32_T c5_d_k;
  int32_T c5_yf_a;
  int32_T c5_ag_a;
  int32_T c5_bg_a;
  int32_T c5_cg_a;
  int32_T c5_kp1;
  real_T c5_c_f;
  real_T c5_unusedU1;
  real_T c5_d_sn;
  real_T c5_d_cs;
  int32_T c5_dg_a;
  int32_T c5_eg_a;
  int32_T c5_mb_c;
  int32_T c5_td_b;
  int32_T c5_ud_b;
  int32_T c5_nb_c;
  int32_T c5_vd_b;
  int32_T c5_wd_b;
  int32_T c5_xd_b;
  int32_T c5_yd_b;
  int32_T c5_ob_c;
  int32_T c5_ae_b;
  int32_T c5_be_b;
  int32_T c5_colkp1;
  real_T c5_d_f;
  real_T c5_unusedU2;
  real_T c5_e_sn;
  real_T c5_e_cs;
  int32_T c5_fg_a;
  int32_T c5_gg_a;
  int32_T c5_pb_c;
  int32_T c5_hg_a;
  int32_T c5_ce_b;
  int32_T c5_ig_a;
  int32_T c5_de_b;
  int32_T c5_qb_c;
  int32_T c5_ee_b;
  int32_T c5_fe_b;
  int32_T c5_jg_a;
  int32_T c5_ge_b;
  int32_T c5_kg_a;
  int32_T c5_he_b;
  int32_T c5_rb_c;
  int32_T c5_ie_b;
  int32_T c5_je_b;
  int32_T c5_lg_a;
  int32_T c5_mg_a;
  int32_T c5_sb_c;
  int32_T c5_tmp_sizes;
  int32_T c5_i100;
  real_T c5_tmp_data[6];
  int32_T c5_e_k;
  int32_T c5_b_tmp_sizes[2];
  int32_T c5_i101;
  int32_T c5_i102;
  int32_T c5_i103;
  real_T c5_b_tmp_data[36];
  int32_T c5_i104;
  int32_T c5_j;
  int32_T c5_b_j;
  int32_T c5_i;
  int32_T c5_b_i;
  int32_T c5_tb_c;
  int32_T c5_ng_a;
  int32_T c5_ub_c;
  int32_T c5_ke_b;
  int32_T c5_le_b;
  int32_T c5_og_a;
  int32_T c5_pg_a;
  int32_T c5_vb_c;
  int32_T c5_qg_a;
  int32_T c5_wb_c;
  int32_T c5_me_b;
  int32_T c5_ne_b;
  int32_T c5_xb_c;
  int32_T c5_oe_b;
  int32_T c5_pe_b;
  int32_T c5_yb_c;
  int32_T c5_rg_a;
  int32_T c5_ac_c;
  int32_T c5_sg_a;
  int32_T c5_qe_b;
  int32_T c5_re_b;
  int32_T c5_bc_c;
  int32_T c5_tg_a;
  int32_T c5_se_b;
  int32_T c5_te_b;
  int32_T c5_ug_a;
  int32_T c5_vg_a;
  int32_T c5_ue_b;
  int32_T c5_ve_b;
  int32_T c5_wg_a;
  int32_T c5_xg_a;
  int32_T c5_yg_a;
  int32_T c5_we_b;
  int32_T c5_xe_b;
  int32_T c5_ye_b;
  int32_T c5_af_b;
  int32_T c5_ah_a;
  int32_T c5_bh_a;
  int32_T c5_bf_b;
  int32_T c5_cf_b;
  int32_T c5_ch_a;
  int32_T c5_df_b;
  int32_T c5_ef_b;
  int32_T c5_dh_a;
  real_T c5_d8;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  boolean_T exitg5;
  boolean_T guard11 = false;
  c5_b_A_sizes[0] = c5_A_sizes[0];
  c5_b_A_sizes[1] = 6;
  c5_A = c5_b_A_sizes[0];
  c5_b_A = c5_b_A_sizes[1];
  c5_loop_ub = c5_A_sizes[0] * c5_A_sizes[1] - 1;
  for (c5_i79 = 0; c5_i79 <= c5_loop_ub; c5_i79++) {
    c5_b_A_data[c5_i79] = c5_A_data[c5_i79];
  }

  c5_d_eml_scalar_eg(chartInstance);
  c5_n = c5_b_A_sizes[0];
  c5_nru = c5_n;
  c5_eml_scalar_eg(chartInstance);
  c5_eml_scalar_eg(chartInstance);
  c5_eml_scalar_eg(chartInstance);
  c5_eml_scalar_eg(chartInstance);
  c5_s_sizes = 6;
  for (c5_i80 = 0; c5_i80 < 6; c5_i80++) {
    c5_s_data[c5_i80] = 0.0;
  }

  for (c5_i81 = 0; c5_i81 < 6; c5_i81++) {
    c5_e[c5_i81] = 0.0;
  }

  c5_iv2[0] = c5_n;
  c5_iv2[1] = 1;
  c5_work_sizes = c5_iv2[0];
  c5_b_loop_ub = c5_iv2[0] - 1;
  for (c5_i82 = 0; c5_i82 <= c5_b_loop_ub; c5_i82++) {
    c5_work_data[c5_i82] = 0.0;
  }

  c5_iv2[0] = c5_nru;
  c5_iv2[1] = 6;
  c5_iv3[0] = c5_iv2[0];
  c5_iv3[1] = 6;
  c5_U_sizes[0] = c5_iv3[0];
  c5_U_sizes[1] = 6;
  c5_U = c5_U_sizes[0];
  c5_b_U = c5_U_sizes[1];
  c5_c_loop_ub = c5_iv2[0] * 6 - 1;
  for (c5_i83 = 0; c5_i83 <= c5_c_loop_ub; c5_i83++) {
    c5_U_data[c5_i83] = 0.0;
  }

  for (c5_i84 = 0; c5_i84 < 36; c5_i84++) {
    c5_Vf[c5_i84] = 0.0;
  }

  c5_eml_scalar_eg(chartInstance);
  c5_eml_scalar_eg(chartInstance);
  c5_varargin_1 = c5_n;
  c5_varargin_2 = c5_varargin_1;
  c5_x = c5_varargin_2;
  c5_b_x = c5_x;
  c5_b_eml_scalar_eg(chartInstance);
  c5_xk = c5_b_x;
  c5_c_x = c5_xk;
  c5_b_eml_scalar_eg(chartInstance);
  c5_maxval = c5_c_x;
  c5_a = c5_maxval;
  c5_b_a = c5_a;
  c5_c = c5_b_a;
  c5_b_varargin_1 = c5_c - 1;
  c5_b_varargin_2 = c5_b_varargin_1;
  c5_d_x = c5_b_varargin_2;
  c5_e_x = c5_d_x;
  c5_eml_scalar_eg(chartInstance);
  c5_b_xk = c5_e_x;
  c5_f_x = c5_b_xk;
  c5_eml_scalar_eg(chartInstance);
  c5_nct = muIntScalarMin_sint32(c5_f_x, 6);
  c5_c_a = c5_nct;
  c5_d_a = c5_c_a + 1;
  c5_nctp1 = c5_d_a;
  c5_c_varargin_1 = c5_nct;
  c5_c_varargin_2 = c5_c_varargin_1;
  c5_g_x = c5_c_varargin_2;
  c5_h_x = c5_g_x;
  c5_eml_scalar_eg(chartInstance);
  c5_c_xk = c5_h_x;
  c5_i_x = c5_c_xk;
  c5_eml_scalar_eg(chartInstance);
  c5_i85 = muIntScalarMax_sint32(c5_i_x, 4);
  c5_b = c5_i85;
  c5_b_b = c5_b;
  if (1 > c5_b_b) {
    c5_overflow = false;
  } else {
    c5_eml_switch_helper(chartInstance);
    c5_overflow = (c5_b_b > 2147483646);
  }

  if (c5_overflow) {
    c5_check_forloop_overflow_error(chartInstance, true);
  }

  for (c5_q = 1; c5_q <= c5_i85; c5_q++) {
    c5_b_q = c5_q;
    c5_e_a = c5_b_q;
    c5_f_a = c5_e_a + 1;
    c5_qp1 = c5_f_a;
    c5_g_a = c5_b_q;
    c5_h_a = c5_g_a;
    c5_qm1 = c5_h_a;
    c5_i_a = c5_n;
    c5_c_b = c5_qm1 - 1;
    c5_j_a = c5_i_a;
    c5_d_b = c5_c_b;
    c5_b_c = c5_j_a * c5_d_b;
    c5_k_a = c5_b_q;
    c5_e_b = c5_b_c;
    c5_l_a = c5_k_a;
    c5_f_b = c5_e_b;
    c5_qq = c5_l_a + c5_f_b;
    c5_m_a = c5_n;
    c5_g_b = c5_b_q;
    c5_n_a = c5_m_a;
    c5_h_b = c5_g_b;
    c5_nmq = c5_n_a - c5_h_b;
    c5_o_a = c5_nmq;
    c5_p_a = c5_o_a + 1;
    c5_nmqp1 = c5_p_a;
    if (c5_b_q <= c5_nct) {
      c5_c_A_sizes[0] = c5_b_A_sizes[0];
      c5_c_A_sizes[1] = 6;
      c5_c_A = c5_c_A_sizes[0];
      c5_d_A = c5_c_A_sizes[1];
      c5_d_loop_ub = c5_b_A_sizes[0] * c5_b_A_sizes[1] - 1;
      for (c5_i86 = 0; c5_i86 <= c5_d_loop_ub; c5_i86++) {
        c5_c_A_data[c5_i86] = c5_b_A_data[c5_i86];
      }

      c5_nrm = c5_eml_xnrm2(chartInstance, c5_nmqp1, c5_c_A_data, c5_c_A_sizes,
                            c5_qq);
      if (c5_nrm > 0.0) {
        c5_absx = c5_nrm;
        c5_d = c5_b_A_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_qq, 1,
          c5_b_A_sizes[0] * 6, 1, 0) - 1];
        if (c5_d < 0.0) {
          c5_y = -c5_absx;
        } else {
          c5_y = c5_absx;
        }

        c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_q, 1, 6, 1, 0) - 1] =
          c5_y;
        c5_d1 = c5_eml_div(chartInstance, 1.0, c5_s_data[c5_b_q - 1]);
        c5_h_eml_xscal(chartInstance, c5_nmqp1, c5_d1, c5_b_A_data, c5_b_A_sizes,
                       c5_qq);
        c5_b_A_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_qq, 1, c5_b_A_sizes[0] *
          6, 1, 0) - 1] = c5_b_A_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_qq, 1,
          c5_b_A_sizes[0] * 6, 1, 0) - 1] + 1.0;
        c5_s_data[c5_b_q - 1] = -c5_s_data[c5_b_q - 1];
      } else {
        c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_q, 1, 6, 1, 0) - 1] = 0.0;
      }
    }

    c5_b_qp1 = c5_qp1;
    c5_q_a = c5_b_qp1;
    c5_r_a = c5_q_a;
    if (c5_r_a > 6) {
    } else {
      c5_eml_switch_helper(chartInstance);
    }

    for (c5_jj = c5_b_qp1; c5_jj < 7; c5_jj++) {
      c5_b_jj = c5_jj;
      c5_s_a = c5_b_jj;
      c5_t_a = c5_s_a;
      c5_c_c = c5_t_a;
      c5_u_a = c5_n;
      c5_i_b = c5_c_c - 1;
      c5_v_a = c5_u_a;
      c5_j_b = c5_i_b;
      c5_d_c = c5_v_a * c5_j_b;
      c5_w_a = c5_b_q;
      c5_k_b = c5_d_c;
      c5_x_a = c5_w_a;
      c5_l_b = c5_k_b;
      c5_qjj = c5_x_a + c5_l_b;
      if (c5_b_q <= c5_nct) {
        if (c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_q, 1, 6, 1, 0) - 1]
            != 0.0) {
          c5_d_A_sizes[0] = c5_b_A_sizes[0];
          c5_d_A_sizes[1] = 6;
          c5_e_A = c5_d_A_sizes[0];
          c5_f_A = c5_d_A_sizes[1];
          c5_e_loop_ub = c5_b_A_sizes[0] * c5_b_A_sizes[1] - 1;
          for (c5_i87 = 0; c5_i87 <= c5_e_loop_ub; c5_i87++) {
            c5_d_A_data[c5_i87] = c5_b_A_data[c5_i87];
          }

          c5_e_A_sizes[0] = c5_b_A_sizes[0];
          c5_e_A_sizes[1] = 6;
          c5_g_A = c5_e_A_sizes[0];
          c5_h_A = c5_e_A_sizes[1];
          c5_f_loop_ub = c5_b_A_sizes[0] * c5_b_A_sizes[1] - 1;
          for (c5_i88 = 0; c5_i88 <= c5_f_loop_ub; c5_i88++) {
            c5_e_A_data[c5_i88] = c5_b_A_data[c5_i88];
          }

          c5_t = c5_b_eml_xdotc(chartInstance, c5_nmqp1, c5_d_A_data,
                                c5_d_A_sizes, c5_qq, c5_e_A_data, c5_e_A_sizes,
                                c5_qjj);
          c5_t = -c5_eml_div(chartInstance, c5_t, c5_b_A_data[(c5_b_q +
            c5_b_A_sizes[0] * (c5_b_q - 1)) - 1]);
          c5_g_eml_xaxpy(chartInstance, c5_nmqp1, c5_t, c5_qq, c5_b_A_data,
                         c5_b_A_sizes, c5_qjj);
        }
      }

      c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_jj, 1, 6, 1, 0) - 1] =
        c5_b_A_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_qjj, 1, c5_b_A_sizes[0] *
        6, 1, 0) - 1];
    }

    if (c5_b_q <= c5_nct) {
      c5_c_q = c5_b_q;
      c5_b_n = c5_n;
      c5_y_a = c5_c_q;
      c5_m_b = c5_b_n;
      c5_ab_a = c5_y_a;
      c5_n_b = c5_m_b;
      if (c5_ab_a > c5_n_b) {
        c5_b_overflow = false;
      } else {
        c5_eml_switch_helper(chartInstance);
        c5_b_overflow = (c5_n_b > 2147483646);
      }

      if (c5_b_overflow) {
        c5_check_forloop_overflow_error(chartInstance, true);
      }

      for (c5_ii = c5_c_q; c5_ii <= c5_b_n; c5_ii++) {
        c5_b_ii = c5_ii;
        c5_U_data[(_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_ii, 1, c5_U_sizes[0], 1,
                    0) + c5_U_sizes[0] * (_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_q,
          1, 6, 2, 0) - 1)) - 1] = c5_b_A_data[(_SFD_EML_ARRAY_BOUNDS_CHECK("",
          c5_b_ii, 1, c5_b_A_sizes[0], 1, 0) + c5_b_A_sizes[0] *
          (_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_q, 1, 6, 2, 0) - 1)) - 1];
      }
    }

    if (c5_b_q <= 4) {
      c5_o_b = c5_b_q;
      c5_p_b = c5_o_b;
      c5_pmq = 6 - c5_p_b;
      for (c5_i89 = 0; c5_i89 < 6; c5_i89++) {
        c5_b_e[c5_i89] = c5_e[c5_i89];
      }

      c5_nrm = c5_b_eml_xnrm2(chartInstance, c5_pmq, c5_b_e, c5_qp1);
      if (c5_nrm == 0.0) {
        c5_e[c5_b_q - 1] = 0.0;
      } else {
        c5_b_absx = c5_nrm;
        c5_b_d = c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_qp1, 1, 6, 1, 0) - 1];
        if (c5_b_d < 0.0) {
          c5_b_y = -c5_b_absx;
        } else {
          c5_b_y = c5_b_absx;
        }

        c5_e[c5_b_q - 1] = c5_b_y;
        c5_d2 = c5_eml_div(chartInstance, 1.0, c5_e[c5_b_q - 1]);
        c5_i_eml_xscal(chartInstance, c5_pmq, c5_d2, c5_e, c5_qp1);
        c5_e[c5_qp1 - 1]++;
      }

      c5_e[c5_b_q - 1] = -c5_e[c5_b_q - 1];
      if (c5_qp1 <= c5_n) {
        if (c5_e[c5_b_q - 1] != 0.0) {
          c5_c_qp1 = c5_qp1;
          c5_c_n = c5_n;
          c5_bb_a = c5_c_qp1;
          c5_q_b = c5_c_n;
          c5_cb_a = c5_bb_a;
          c5_r_b = c5_q_b;
          if (c5_cb_a > c5_r_b) {
            c5_c_overflow = false;
          } else {
            c5_eml_switch_helper(chartInstance);
            c5_c_overflow = (c5_r_b > 2147483646);
          }

          if (c5_c_overflow) {
            c5_check_forloop_overflow_error(chartInstance, true);
          }

          for (c5_c_ii = c5_c_qp1; c5_c_ii <= c5_c_n; c5_c_ii++) {
            c5_b_ii = c5_c_ii;
            c5_work_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_ii, 1,
              c5_work_sizes, 1, 0) - 1] = 0.0;
          }

          c5_d_qp1 = c5_qp1;
          c5_db_a = c5_d_qp1;
          c5_eb_a = c5_db_a;
          if (c5_eb_a > 6) {
          } else {
            c5_eml_switch_helper(chartInstance);
          }

          for (c5_c_jj = c5_d_qp1; c5_c_jj < 7; c5_c_jj++) {
            c5_b_jj = c5_c_jj;
            c5_fb_a = c5_b_jj;
            c5_gb_a = c5_fb_a;
            c5_e_c = c5_gb_a;
            c5_hb_a = c5_n;
            c5_s_b = c5_e_c - 1;
            c5_ib_a = c5_hb_a;
            c5_t_b = c5_s_b;
            c5_f_c = c5_ib_a * c5_t_b;
            c5_jb_a = c5_qp1;
            c5_u_b = c5_f_c;
            c5_kb_a = c5_jb_a;
            c5_v_b = c5_u_b;
            c5_qp1jj = c5_kb_a + c5_v_b;
            c5_f_A_sizes[0] = c5_b_A_sizes[0];
            c5_f_A_sizes[1] = 6;
            c5_i_A = c5_f_A_sizes[0];
            c5_j_A = c5_f_A_sizes[1];
            c5_g_loop_ub = c5_b_A_sizes[0] * c5_b_A_sizes[1] - 1;
            for (c5_i90 = 0; c5_i90 <= c5_g_loop_ub; c5_i90++) {
              c5_f_A_data[c5_i90] = c5_b_A_data[c5_i90];
            }

            c5_h_eml_xaxpy(chartInstance, c5_nmq,
                           c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_jj, 1, 6, 1,
              0) - 1], c5_f_A_data, c5_f_A_sizes, c5_qp1jj, c5_work_data,
                           &c5_work_sizes, c5_qp1);
          }

          c5_e_qp1 = c5_qp1;
          c5_lb_a = c5_e_qp1;
          c5_mb_a = c5_lb_a;
          if (c5_mb_a > 6) {
          } else {
            c5_eml_switch_helper(chartInstance);
          }

          for (c5_d_jj = c5_e_qp1; c5_d_jj < 7; c5_d_jj++) {
            c5_b_jj = c5_d_jj;
            c5_t = c5_eml_div(chartInstance, -c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
              "", c5_b_jj, 1, 6, 1, 0) - 1], c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
              c5_qp1, 1, 6, 1, 0) - 1]);
            c5_nb_a = c5_b_jj;
            c5_ob_a = c5_nb_a;
            c5_g_c = c5_ob_a;
            c5_pb_a = c5_n;
            c5_w_b = c5_g_c - 1;
            c5_qb_a = c5_pb_a;
            c5_x_b = c5_w_b;
            c5_h_c = c5_qb_a * c5_x_b;
            c5_rb_a = c5_qp1;
            c5_y_b = c5_h_c;
            c5_sb_a = c5_rb_a;
            c5_ab_b = c5_y_b;
            c5_qp1jj = c5_sb_a + c5_ab_b;
            c5_b_work_sizes = c5_work_sizes;
            c5_h_loop_ub = c5_work_sizes - 1;
            for (c5_i91 = 0; c5_i91 <= c5_h_loop_ub; c5_i91++) {
              c5_b_work_data[c5_i91] = c5_work_data[c5_i91];
            }

            c5_i_eml_xaxpy(chartInstance, c5_nmq, c5_t, c5_b_work_data,
                           c5_b_work_sizes, c5_qp1, c5_b_A_data, c5_b_A_sizes,
                           c5_qp1jj);
          }
        }
      }

      c5_f_qp1 = c5_qp1;
      c5_tb_a = c5_f_qp1;
      c5_ub_a = c5_tb_a;
      if (c5_ub_a > 6) {
      } else {
        c5_eml_switch_helper(chartInstance);
      }

      for (c5_d_ii = c5_f_qp1; c5_d_ii < 7; c5_d_ii++) {
        c5_b_ii = c5_d_ii;
        c5_Vf[(_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_ii, 1, 6, 1, 0) + 6 *
               (c5_b_q - 1)) - 1] = c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_ii,
          1, 6, 1, 0) - 1];
      }
    }
  }

  c5_vb_a = c5_n;
  c5_wb_a = c5_vb_a;
  c5_i_c = c5_wb_a;
  c5_d_varargin_2 = c5_i_c + 1;
  c5_varargin_3 = c5_d_varargin_2;
  c5_c_y = c5_varargin_3;
  c5_d_y = c5_c_y;
  c5_eml_scalar_eg(chartInstance);
  c5_yk = c5_d_y;
  c5_e_y = c5_yk;
  c5_eml_scalar_eg(chartInstance);
  c5_m = muIntScalarMin_sint32(6, c5_e_y);
  if (c5_nct < 6) {
    c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_nctp1, 1, 6, 1, 0) - 1] =
      c5_b_A_data[(_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_nctp1, 1, c5_b_A_sizes[0],
      1, 0) + c5_b_A_sizes[0] * (_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_nctp1, 1, 6,
      2, 0) - 1)) - 1];
  }

  if (c5_n < c5_m) {
    c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_m, 1, 6, 1, 0) - 1] = 0.0;
  }

  if (5 < c5_m) {
    _SFD_EML_ARRAY_BOUNDS_CHECK("", c5_m, 1, 6, 2, 0);
    c5_e[4] = c5_b_A_data[4 + c5_b_A_sizes[0] * 5];
  }

  c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_m, 1, 6, 1, 0) - 1] = 0.0;
  if (c5_nctp1 <= 6) {
    c5_b_nctp1 = c5_nctp1;
    c5_xb_a = c5_b_nctp1;
    c5_yb_a = c5_xb_a;
    if (c5_yb_a > 6) {
      c5_d_overflow = false;
    } else {
      c5_eml_switch_helper(chartInstance);
      c5_d_overflow = false;
    }

    if (c5_d_overflow) {
      c5_check_forloop_overflow_error(chartInstance, true);
    }

    for (c5_e_jj = c5_b_nctp1; c5_e_jj < 7; c5_e_jj++) {
      c5_b_jj = c5_e_jj - 1;
      c5_b_nru = c5_nru;
      c5_bb_b = c5_b_nru;
      c5_cb_b = c5_bb_b;
      if (1 > c5_cb_b) {
        c5_e_overflow = false;
      } else {
        c5_eml_switch_helper(chartInstance);
        c5_e_overflow = (c5_cb_b > 2147483646);
      }

      if (c5_e_overflow) {
        c5_check_forloop_overflow_error(chartInstance, true);
      }

      for (c5_e_ii = 1; c5_e_ii <= c5_b_nru; c5_e_ii++) {
        c5_b_ii = c5_e_ii;
        c5_U_data[(_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_ii, 1, c5_U_sizes[0], 1,
                    0) + c5_U_sizes[0] * c5_b_jj) - 1] = 0.0;
      }

      c5_U_data[c5_b_jj + c5_U_sizes[0] * c5_b_jj] = 1.0;
    }
  }

  c5_b_nct = c5_nct;
  c5_ac_a = c5_b_nct;
  c5_bc_a = c5_ac_a;
  if (c5_bc_a < 1) {
  } else {
    c5_eml_switch_helper(chartInstance);
  }

  for (c5_d_q = c5_b_nct; c5_d_q > 0; c5_d_q--) {
    c5_b_q = c5_d_q;
    c5_cc_a = c5_b_q;
    c5_dc_a = c5_cc_a;
    c5_qp1 = c5_dc_a;
    c5_ec_a = c5_n;
    c5_db_b = c5_b_q;
    c5_fc_a = c5_ec_a;
    c5_eb_b = c5_db_b;
    c5_nmq = c5_fc_a - c5_eb_b;
    c5_gc_a = c5_nmq;
    c5_hc_a = c5_gc_a + 1;
    c5_nmqp1 = c5_hc_a;
    c5_ic_a = c5_b_q;
    c5_jc_a = c5_ic_a;
    c5_j_c = c5_jc_a;
    c5_kc_a = c5_nru;
    c5_fb_b = c5_j_c - 1;
    c5_lc_a = c5_kc_a;
    c5_gb_b = c5_fb_b;
    c5_k_c = c5_lc_a * c5_gb_b;
    c5_mc_a = c5_b_q;
    c5_hb_b = c5_k_c;
    c5_nc_a = c5_mc_a;
    c5_ib_b = c5_hb_b;
    c5_qq = c5_nc_a + c5_ib_b;
    if (c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_q, 1, 6, 1, 0) - 1] !=
        0.0) {
      c5_g_qp1 = c5_qp1 + 1;
      c5_oc_a = c5_g_qp1;
      c5_pc_a = c5_oc_a;
      if (c5_pc_a > 6) {
        c5_f_overflow = false;
      } else {
        c5_eml_switch_helper(chartInstance);
        c5_f_overflow = false;
      }

      if (c5_f_overflow) {
        c5_check_forloop_overflow_error(chartInstance, true);
      }

      for (c5_f_jj = c5_g_qp1; c5_f_jj < 7; c5_f_jj++) {
        c5_b_jj = c5_f_jj;
        c5_qc_a = c5_b_jj;
        c5_rc_a = c5_qc_a;
        c5_l_c = c5_rc_a;
        c5_sc_a = c5_nru;
        c5_jb_b = c5_l_c - 1;
        c5_tc_a = c5_sc_a;
        c5_kb_b = c5_jb_b;
        c5_m_c = c5_tc_a * c5_kb_b;
        c5_uc_a = c5_b_q;
        c5_lb_b = c5_m_c;
        c5_vc_a = c5_uc_a;
        c5_mb_b = c5_lb_b;
        c5_qjj = c5_vc_a + c5_mb_b;
        c5_b_U_sizes[0] = c5_U_sizes[0];
        c5_b_U_sizes[1] = 6;
        c5_c_U = c5_b_U_sizes[0];
        c5_d_U = c5_b_U_sizes[1];
        c5_i_loop_ub = c5_U_sizes[0] * c5_U_sizes[1] - 1;
        for (c5_i92 = 0; c5_i92 <= c5_i_loop_ub; c5_i92++) {
          c5_b_U_data[c5_i92] = c5_U_data[c5_i92];
        }

        c5_c_U_sizes[0] = c5_U_sizes[0];
        c5_c_U_sizes[1] = 6;
        c5_e_U = c5_c_U_sizes[0];
        c5_f_U = c5_c_U_sizes[1];
        c5_j_loop_ub = c5_U_sizes[0] * c5_U_sizes[1] - 1;
        for (c5_i93 = 0; c5_i93 <= c5_j_loop_ub; c5_i93++) {
          c5_c_U_data[c5_i93] = c5_U_data[c5_i93];
        }

        c5_t = c5_eml_xdotc(chartInstance, c5_nmqp1, c5_b_U_data, c5_b_U_sizes,
                            c5_qq, c5_c_U_data, c5_c_U_sizes, c5_qjj);
        c5_t = -c5_eml_div(chartInstance, c5_t,
                           c5_U_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_qq, 1,
          c5_U_sizes[0] * 6, 1, 0) - 1]);
        c5_f_eml_xaxpy(chartInstance, c5_nmqp1, c5_t, c5_qq, c5_U_data,
                       c5_U_sizes, c5_qjj);
      }

      c5_e_q = c5_b_q;
      c5_d_n = c5_n;
      c5_wc_a = c5_e_q;
      c5_nb_b = c5_d_n;
      c5_xc_a = c5_wc_a;
      c5_ob_b = c5_nb_b;
      if (c5_xc_a > c5_ob_b) {
        c5_g_overflow = false;
      } else {
        c5_eml_switch_helper(chartInstance);
        c5_g_overflow = (c5_ob_b > 2147483646);
      }

      if (c5_g_overflow) {
        c5_check_forloop_overflow_error(chartInstance, true);
      }

      for (c5_f_ii = c5_e_q; c5_f_ii <= c5_d_n; c5_f_ii++) {
        c5_b_ii = c5_f_ii;
        c5_U_data[(_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_ii, 1, c5_U_sizes[0], 1,
                    0) + c5_U_sizes[0] * (c5_b_q - 1)) - 1] = -c5_U_data
          [(_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_ii, 1, c5_U_sizes[0], 1, 0) +
            c5_U_sizes[0] * (c5_b_q - 1)) - 1];
      }

      c5_U_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_qq, 1, c5_U_sizes[0] * 6, 1,
        0) - 1] = c5_U_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_qq, 1,
        c5_U_sizes[0] * 6, 1, 0) - 1] + 1.0;
      c5_yc_a = c5_b_q;
      c5_ad_a = c5_yc_a - 1;
      c5_i94 = c5_ad_a;
      c5_pb_b = c5_i94;
      c5_qb_b = c5_pb_b;
      if (1 > c5_qb_b) {
        c5_h_overflow = false;
      } else {
        c5_eml_switch_helper(chartInstance);
        c5_h_overflow = (c5_qb_b > 2147483646);
      }

      if (c5_h_overflow) {
        c5_check_forloop_overflow_error(chartInstance, true);
      }

      for (c5_g_ii = 1; c5_g_ii <= c5_i94; c5_g_ii++) {
        c5_b_ii = c5_g_ii;
        c5_U_data[(_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_ii, 1, c5_U_sizes[0], 1,
                    0) + c5_U_sizes[0] * (c5_b_q - 1)) - 1] = 0.0;
      }
    } else {
      c5_e_n = c5_n;
      c5_rb_b = c5_e_n;
      c5_sb_b = c5_rb_b;
      if (1 > c5_sb_b) {
        c5_i_overflow = false;
      } else {
        c5_eml_switch_helper(chartInstance);
        c5_i_overflow = (c5_sb_b > 2147483646);
      }

      if (c5_i_overflow) {
        c5_check_forloop_overflow_error(chartInstance, true);
      }

      for (c5_h_ii = 1; c5_h_ii <= c5_e_n; c5_h_ii++) {
        c5_b_ii = c5_h_ii;
        c5_U_data[(_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_ii, 1, c5_U_sizes[0], 1,
                    0) + c5_U_sizes[0] * (c5_b_q - 1)) - 1] = 0.0;
      }

      c5_U_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_qq, 1, c5_U_sizes[0] * 6, 1,
        0) - 1] = 1.0;
    }
  }

  for (c5_f_q = 6; c5_f_q > 0; c5_f_q--) {
    c5_b_q = c5_f_q;
    if (c5_b_q <= 4) {
      if (c5_e[c5_b_q - 1] != 0.0) {
        c5_bd_a = c5_b_q;
        c5_cd_a = c5_bd_a + 1;
        c5_qp1 = c5_cd_a;
        c5_tb_b = c5_b_q;
        c5_ub_b = c5_tb_b;
        c5_pmq = 6 - c5_ub_b;
        c5_dd_a = c5_b_q;
        c5_ed_a = c5_dd_a;
        c5_n_c = c5_ed_a;
        c5_vb_b = c5_n_c - 1;
        c5_wb_b = c5_vb_b;
        c5_o_c = 6 * c5_wb_b;
        c5_fd_a = c5_qp1;
        c5_xb_b = c5_o_c;
        c5_gd_a = c5_fd_a;
        c5_yb_b = c5_xb_b;
        c5_qp1q = c5_gd_a + c5_yb_b;
        c5_h_qp1 = c5_qp1;
        c5_hd_a = c5_h_qp1;
        c5_id_a = c5_hd_a;
        if (c5_id_a > 6) {
        } else {
          c5_eml_switch_helper(chartInstance);
        }

        for (c5_g_jj = c5_h_qp1; c5_g_jj < 7; c5_g_jj++) {
          c5_b_jj = c5_g_jj;
          c5_jd_a = c5_b_jj;
          c5_kd_a = c5_jd_a;
          c5_p_c = c5_kd_a;
          c5_ac_b = c5_p_c - 1;
          c5_bc_b = c5_ac_b;
          c5_q_c = 6 * c5_bc_b;
          c5_ld_a = c5_qp1;
          c5_cc_b = c5_q_c;
          c5_md_a = c5_ld_a;
          c5_dc_b = c5_cc_b;
          c5_qp1jj = c5_md_a + c5_dc_b;
          for (c5_i95 = 0; c5_i95 < 36; c5_i95++) {
            c5_b_Vf[c5_i95] = c5_Vf[c5_i95];
          }

          for (c5_i96 = 0; c5_i96 < 36; c5_i96++) {
            c5_c_Vf[c5_i96] = c5_Vf[c5_i96];
          }

          c5_t = c5_c_eml_xdotc(chartInstance, c5_pmq, c5_b_Vf, c5_qp1q, c5_c_Vf,
                                c5_qp1jj);
          c5_t = -c5_eml_div(chartInstance, c5_t,
                             c5_Vf[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_qp1q, 1,
            36, 1, 0) - 1]);
          c5_j_eml_xaxpy(chartInstance, c5_pmq, c5_t, c5_qp1q, c5_Vf, c5_qp1jj);
        }
      }
    }

    for (c5_i_ii = 1; c5_i_ii < 7; c5_i_ii++) {
      c5_b_ii = c5_i_ii - 1;
      c5_Vf[c5_b_ii + 6 * (c5_b_q - 1)] = 0.0;
    }

    c5_Vf[(c5_b_q + 6 * (c5_b_q - 1)) - 1] = 1.0;
  }

  c5_b_m = c5_m;
  c5_ec_b = c5_b_m;
  c5_fc_b = c5_ec_b;
  if (1 > c5_fc_b) {
    c5_j_overflow = false;
  } else {
    c5_eml_switch_helper(chartInstance);
    c5_j_overflow = (c5_fc_b > 2147483646);
  }

  if (c5_j_overflow) {
    c5_check_forloop_overflow_error(chartInstance, true);
  }

  for (c5_g_q = 1; c5_g_q <= c5_b_m; c5_g_q++) {
    c5_b_q = c5_g_q - 1;
    if (c5_s_data[c5_b_q] != 0.0) {
      c5_rt = c5_abs(chartInstance, c5_s_data[c5_b_q]);
      c5_r = c5_eml_div(chartInstance, c5_s_data[c5_b_q], c5_rt);
      c5_s_data[c5_b_q] = c5_rt;
      if (c5_b_q + 1 < c5_m) {
        c5_e[c5_b_q] = c5_eml_div(chartInstance, c5_e[c5_b_q], c5_r);
      }

      c5_nd_a = c5_b_q + 1;
      c5_od_a = c5_nd_a;
      c5_r_c = c5_od_a;
      c5_pd_a = c5_nru;
      c5_gc_b = c5_r_c - 1;
      c5_qd_a = c5_pd_a;
      c5_hc_b = c5_gc_b;
      c5_s_c = c5_qd_a * c5_hc_b;
      c5_ic_b = c5_s_c;
      c5_jc_b = c5_ic_b;
      c5_colq = c5_jc_b;
      c5_g_eml_xscal(chartInstance, c5_n, c5_r, c5_U_data, c5_U_sizes, c5_colq +
                     1);
    }

    if (c5_b_q + 1 < c5_m) {
      if (c5_e[c5_b_q] != 0.0) {
        c5_rt = c5_abs(chartInstance, c5_e[c5_b_q]);
        c5_r = c5_eml_div(chartInstance, c5_rt, c5_e[c5_b_q]);
        c5_e[c5_b_q] = c5_rt;
        c5_rd_a = c5_b_q + 1;
        c5_sd_a = c5_rd_a;
        c5_t_c = c5_sd_a;
        c5_td_a = c5_b_q + 1;
        c5_ud_a = c5_td_a;
        c5_u_c = c5_ud_a;
        c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_t_c + 1, 1, 6, 1, 0) - 1] =
          c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_u_c + 1, 1, 6, 1, 0) - 1]
          * c5_r;
        c5_kc_b = c5_b_q + 1;
        c5_lc_b = c5_kc_b;
        c5_v_c = 6 * c5_lc_b;
        c5_mc_b = c5_v_c;
        c5_nc_b = c5_mc_b;
        c5_colqp1 = c5_nc_b;
        c5_j_eml_xscal(chartInstance, c5_r, c5_Vf, c5_colqp1 + 1);
      }
    }
  }

  c5_mm = c5_m;
  c5_iter = 0.0;
  c5_realmin(chartInstance);
  c5_eps(chartInstance);
  c5_tiny = c5_eml_div(chartInstance, 2.2250738585072014E-308,
                       2.2204460492503131E-16);
  c5_snorm = 0.0;
  c5_c_m = c5_m;
  c5_oc_b = c5_c_m;
  c5_pc_b = c5_oc_b;
  if (1 > c5_pc_b) {
    c5_k_overflow = false;
  } else {
    c5_eml_switch_helper(chartInstance);
    c5_k_overflow = (c5_pc_b > 2147483646);
  }

  if (c5_k_overflow) {
    c5_check_forloop_overflow_error(chartInstance, true);
  }

  for (c5_j_ii = 1; c5_j_ii <= c5_c_m; c5_j_ii++) {
    c5_b_ii = c5_j_ii - 1;
    c5_d_varargin_1 = c5_abs(chartInstance, c5_s_data[c5_b_ii]);
    c5_e_varargin_2 = c5_abs(chartInstance, c5_e[c5_b_ii]);
    c5_f_varargin_2 = c5_d_varargin_1;
    c5_b_varargin_3 = c5_e_varargin_2;
    c5_j_x = c5_f_varargin_2;
    c5_f_y = c5_b_varargin_3;
    c5_k_x = c5_j_x;
    c5_g_y = c5_f_y;
    c5_c_eml_scalar_eg(chartInstance);
    c5_d_xk = c5_k_x;
    c5_b_yk = c5_g_y;
    c5_l_x = c5_d_xk;
    c5_h_y = c5_b_yk;
    c5_c_eml_scalar_eg(chartInstance);
    c5_b_maxval = muDoubleScalarMax(c5_l_x, c5_h_y);
    c5_e_varargin_1 = c5_snorm;
    c5_g_varargin_2 = c5_b_maxval;
    c5_h_varargin_2 = c5_e_varargin_1;
    c5_c_varargin_3 = c5_g_varargin_2;
    c5_m_x = c5_h_varargin_2;
    c5_i_y = c5_c_varargin_3;
    c5_n_x = c5_m_x;
    c5_j_y = c5_i_y;
    c5_c_eml_scalar_eg(chartInstance);
    c5_e_xk = c5_n_x;
    c5_c_yk = c5_j_y;
    c5_o_x = c5_e_xk;
    c5_k_y = c5_c_yk;
    c5_c_eml_scalar_eg(chartInstance);
    c5_snorm = muDoubleScalarMax(c5_o_x, c5_k_y);
  }

  exitg1 = false;
  while ((exitg1 == false) && (c5_m > 0)) {
    if (c5_iter >= 75.0) {
      c5_b_eml_error(chartInstance);
      exitg1 = true;
    } else {
      c5_vd_a = c5_m;
      c5_wd_a = c5_vd_a - 1;
      c5_b_q = c5_wd_a;
      c5_xd_a = c5_m;
      c5_yd_a = c5_xd_a - 1;
      c5_i97 = c5_yd_a;
      c5_ae_a = c5_i97;
      c5_be_a = c5_ae_a;
      if (c5_be_a < 0) {
      } else {
        c5_eml_switch_helper(chartInstance);
      }

      c5_k_ii = c5_i97;
      guard3 = false;
      guard4 = false;
      exitg5 = false;
      while ((exitg5 == false) && (c5_k_ii > -1)) {
        c5_b_ii = c5_k_ii;
        c5_b_q = c5_b_ii;
        if (c5_b_ii == 0) {
          exitg5 = true;
        } else {
          c5_ce_a = c5_b_ii;
          c5_de_a = c5_ce_a;
          c5_w_c = c5_de_a;
          c5_test0 = c5_abs(chartInstance, c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK
                            ("", c5_b_ii, 1, 6, 1, 0) - 1]) + c5_abs
            (chartInstance, c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_w_c + 1,
              1, 6, 1, 0) - 1]);
          c5_ztest0 = c5_abs(chartInstance, c5_e[c5_b_ii - 1]);
          c5_eps(chartInstance);
          if (c5_ztest0 <= 2.2204460492503131E-16 * c5_test0) {
            guard4 = true;
            exitg5 = true;
          } else if (c5_ztest0 <= c5_tiny) {
            guard4 = true;
            exitg5 = true;
          } else {
            guard11 = false;
            if (c5_iter > 20.0) {
              c5_eps(chartInstance);
              if (c5_ztest0 <= 2.2204460492503131E-16 * c5_snorm) {
                guard3 = true;
                exitg5 = true;
              } else {
                guard11 = true;
              }
            } else {
              guard11 = true;
            }

            if (guard11 == true) {
              c5_k_ii--;
              guard3 = false;
              guard4 = false;
            }
          }
        }
      }

      if (guard4 == true) {
        guard3 = true;
      }

      if (guard3 == true) {
        c5_e[c5_b_ii - 1] = 0.0;
      }

      c5_ee_a = c5_m;
      c5_fe_a = c5_ee_a;
      c5_x_c = c5_fe_a;
      if (c5_b_q == c5_x_c - 1) {
        c5_kase = 4.0;
      } else {
        c5_qs = c5_m;
        c5_d_m = c5_m;
        c5_h_q = c5_b_q;
        c5_ge_a = c5_d_m;
        c5_qc_b = c5_h_q;
        c5_he_a = c5_ge_a;
        c5_rc_b = c5_qc_b;
        if (c5_he_a < c5_rc_b) {
          c5_l_overflow = false;
        } else {
          c5_eml_switch_helper(chartInstance);
          c5_l_overflow = (c5_rc_b < -2147483647);
        }

        if (c5_l_overflow) {
          c5_check_forloop_overflow_error(chartInstance, true);
        }

        c5_l_ii = c5_d_m;
        guard2 = false;
        exitg4 = false;
        while ((exitg4 == false) && (c5_l_ii >= c5_h_q)) {
          c5_b_ii = c5_l_ii;
          c5_qs = c5_b_ii;
          if (c5_b_ii == c5_b_q) {
            exitg4 = true;
          } else {
            c5_test = 0.0;
            if (c5_b_ii < c5_m) {
              c5_test = c5_abs(chartInstance, c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", c5_b_ii, 1, 6, 1, 0) - 1]);
            }

            c5_ie_a = c5_b_q;
            c5_je_a = c5_ie_a;
            c5_y_c = c5_je_a;
            if (c5_b_ii > c5_y_c + 1) {
              c5_ke_a = c5_b_ii;
              c5_le_a = c5_ke_a;
              c5_ab_c = c5_le_a;
              c5_test += c5_abs(chartInstance, c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK(
                "", c5_ab_c - 1, 1, 6, 1, 0) - 1]);
            }

            c5_ztest = c5_abs(chartInstance,
                              c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_ii,
              1, 6, 1, 0) - 1]);
            c5_eps(chartInstance);
            if (c5_ztest <= 2.2204460492503131E-16 * c5_test) {
              guard2 = true;
              exitg4 = true;
            } else if (c5_ztest <= c5_tiny) {
              guard2 = true;
              exitg4 = true;
            } else {
              c5_l_ii--;
              guard2 = false;
            }
          }
        }

        if (guard2 == true) {
          c5_s_data[c5_b_ii - 1] = 0.0;
        }

        if (c5_qs == c5_b_q) {
          c5_kase = 3.0;
        } else if (c5_qs == c5_m) {
          c5_kase = 1.0;
        } else {
          c5_kase = 2.0;
          c5_b_q = c5_qs;
        }
      }

      c5_me_a = c5_b_q;
      c5_ne_a = c5_me_a + 1;
      c5_b_q = c5_ne_a;
      switch ((int32_T)c5_kase) {
       case 1:
        c5_oe_a = c5_m;
        c5_pe_a = c5_oe_a;
        c5_bb_c = c5_pe_a;
        c5_f = c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_bb_c - 1, 1, 6, 1, 0) - 1];
        c5_qe_a = c5_m;
        c5_re_a = c5_qe_a;
        c5_cb_c = c5_re_a;
        c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_cb_c - 1, 1, 6, 1, 0) - 1] = 0.0;
        c5_se_a = c5_m;
        c5_te_a = c5_se_a - 1;
        c5_i98 = c5_te_a;
        c5_i_q = c5_b_q;
        c5_ue_a = c5_i98;
        c5_sc_b = c5_i_q;
        c5_ve_a = c5_ue_a;
        c5_tc_b = c5_sc_b;
        if (c5_ve_a < c5_tc_b) {
          c5_m_overflow = false;
        } else {
          c5_eml_switch_helper(chartInstance);
          c5_m_overflow = (c5_tc_b < -2147483647);
        }

        if (c5_m_overflow) {
          c5_check_forloop_overflow_error(chartInstance, true);
        }

        for (c5_k = c5_i98; c5_k >= c5_i_q; c5_k--) {
          c5_b_k = c5_k;
          c5_t1 = c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_k, 1, 6, 1, 0)
            - 1];
          c5_b_t1 = c5_t1;
          c5_b_f = c5_f;
          c5_b_eml_xrotg(chartInstance, &c5_b_t1, &c5_b_f, &c5_cs, &c5_sn);
          c5_t1 = c5_b_t1;
          c5_f = c5_b_f;
          c5_b_cs = c5_cs;
          c5_b_sn = c5_sn;
          c5_s_data[c5_b_k - 1] = c5_t1;
          if (c5_b_k > c5_b_q) {
            c5_we_a = c5_b_k;
            c5_xe_a = c5_we_a - 2;
            c5_km1 = c5_xe_a;
            c5_f = -c5_b_sn * c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_km1 + 1, 1,
              6, 1, 0) - 1];
            c5_e[c5_km1] *= c5_b_cs;
          }

          c5_ye_a = c5_b_k;
          c5_af_a = c5_ye_a;
          c5_db_c = c5_af_a;
          c5_uc_b = c5_db_c - 1;
          c5_vc_b = c5_uc_b;
          c5_eb_c = 6 * c5_vc_b;
          c5_wc_b = c5_eb_c;
          c5_xc_b = c5_wc_b;
          c5_colk = c5_xc_b;
          c5_bf_a = c5_m;
          c5_cf_a = c5_bf_a;
          c5_fb_c = c5_cf_a;
          c5_yc_b = c5_fb_c - 1;
          c5_ad_b = c5_yc_b;
          c5_gb_c = 6 * c5_ad_b;
          c5_bd_b = c5_gb_c;
          c5_cd_b = c5_bd_b;
          c5_colm = c5_cd_b;
          c5_d_eml_xrot(chartInstance, c5_Vf, c5_colk + 1, c5_colm + 1, c5_b_cs,
                        c5_b_sn);
        }
        break;

       case 2:
        c5_df_a = c5_b_q;
        c5_ef_a = c5_df_a - 1;
        c5_qm1 = c5_ef_a;
        c5_f = c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_qm1, 1, 6, 1, 0) - 1];
        c5_e[c5_qm1 - 1] = 0.0;
        c5_j_q = c5_b_q;
        c5_e_m = c5_m;
        c5_ff_a = c5_j_q;
        c5_dd_b = c5_e_m;
        c5_gf_a = c5_ff_a;
        c5_ed_b = c5_dd_b;
        if (c5_gf_a > c5_ed_b) {
          c5_n_overflow = false;
        } else {
          c5_eml_switch_helper(chartInstance);
          c5_n_overflow = (c5_ed_b > 2147483646);
        }

        if (c5_n_overflow) {
          c5_check_forloop_overflow_error(chartInstance, true);
        }

        for (c5_c_k = c5_j_q; c5_c_k <= c5_e_m; c5_c_k++) {
          c5_b_k = c5_c_k - 1;
          c5_t1 = c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_k + 1, 1, 6, 1,
            0) - 1];
          c5_c_t1 = c5_t1;
          c5_unusedU0 = c5_f;
          c5_b_eml_xrotg(chartInstance, &c5_c_t1, &c5_unusedU0, &c5_c_cs,
                         &c5_c_sn);
          c5_t1 = c5_c_t1;
          c5_b_cs = c5_c_cs;
          c5_b_sn = c5_c_sn;
          c5_s_data[c5_b_k] = c5_t1;
          c5_f = -c5_b_sn * c5_e[c5_b_k];
          c5_e[c5_b_k] *= c5_b_cs;
          c5_hf_a = c5_b_k + 1;
          c5_if_a = c5_hf_a;
          c5_hb_c = c5_if_a;
          c5_jf_a = c5_nru;
          c5_fd_b = c5_hb_c - 1;
          c5_kf_a = c5_jf_a;
          c5_gd_b = c5_fd_b;
          c5_ib_c = c5_kf_a * c5_gd_b;
          c5_hd_b = c5_ib_c;
          c5_id_b = c5_hd_b;
          c5_colk = c5_id_b;
          c5_lf_a = c5_qm1;
          c5_mf_a = c5_lf_a;
          c5_jb_c = c5_mf_a;
          c5_nf_a = c5_nru;
          c5_jd_b = c5_jb_c - 1;
          c5_of_a = c5_nf_a;
          c5_kd_b = c5_jd_b;
          c5_kb_c = c5_of_a * c5_kd_b;
          c5_ld_b = c5_kb_c;
          c5_md_b = c5_ld_b;
          c5_colqm1 = c5_md_b;
          c5_c_eml_xrot(chartInstance, c5_n, c5_U_data, c5_U_sizes, c5_colk + 1,
                        c5_colqm1 + 1, c5_b_cs, c5_b_sn);
        }
        break;

       case 3:
        c5_pf_a = c5_m;
        c5_qf_a = c5_pf_a - 1;
        c5_mm1 = c5_qf_a;
        c5_d3 = c5_abs(chartInstance, c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          c5_m, 1, 6, 1, 0) - 1]);
        c5_d4 = c5_abs(chartInstance, c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          c5_mm1, 1, 6, 1, 0) - 1]);
        c5_d5 = c5_abs(chartInstance, c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          c5_mm1, 1, 6, 1, 0) - 1]);
        c5_d6 = c5_abs(chartInstance, c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          c5_b_q, 1, 6, 1, 0) - 1]);
        c5_d7 = c5_abs(chartInstance, c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK("",
          c5_b_q, 1, 6, 1, 0) - 1]);
        c5_f_varargin_1[0] = c5_d3;
        c5_f_varargin_1[1] = c5_d4;
        c5_f_varargin_1[2] = c5_d5;
        c5_f_varargin_1[3] = c5_d6;
        c5_f_varargin_1[4] = c5_d7;
        c5_ixstart = 1;
        c5_mtmp = c5_f_varargin_1[0];
        c5_p_x = c5_mtmp;
        c5_nd_b = muDoubleScalarIsNaN(c5_p_x);
        if (c5_nd_b) {
          c5_eml_switch_helper(chartInstance);
          c5_ix = 2;
          exitg2 = false;
          while ((exitg2 == false) && (c5_ix < 6)) {
            c5_b_ix = c5_ix;
            c5_ixstart = c5_b_ix;
            c5_q_x = c5_f_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_ix, 1,
              5, 1, 0) - 1];
            c5_od_b = muDoubleScalarIsNaN(c5_q_x);
            if (!c5_od_b) {
              c5_mtmp = c5_f_varargin_1[c5_b_ix - 1];
              exitg2 = true;
            } else {
              c5_ix++;
            }
          }
        }

        if (c5_ixstart < 5) {
          c5_rf_a = c5_ixstart;
          c5_sf_a = c5_rf_a + 1;
          c5_i99 = c5_sf_a;
          c5_tf_a = c5_i99;
          c5_uf_a = c5_tf_a;
          if (c5_uf_a > 5) {
          } else {
            c5_eml_switch_helper(chartInstance);
          }

          for (c5_c_ix = c5_i99; c5_c_ix < 6; c5_c_ix++) {
            c5_b_ix = c5_c_ix;
            c5_vf_a = c5_f_varargin_1[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_ix, 1,
              5, 1, 0) - 1];
            c5_pd_b = c5_mtmp;
            c5_p = (c5_vf_a > c5_pd_b);
            if (c5_p) {
              c5_mtmp = c5_f_varargin_1[c5_b_ix - 1];
            }
          }
        }

        c5_b_mtmp = c5_mtmp;
        c5_scale = c5_b_mtmp;
        c5_sm = c5_eml_div(chartInstance, c5_s_data[c5_m - 1], c5_scale);
        c5_smm1 = c5_eml_div(chartInstance, c5_s_data[c5_mm1 - 1], c5_scale);
        c5_emm1 = c5_eml_div(chartInstance, c5_e[c5_mm1 - 1], c5_scale);
        c5_sqds = c5_eml_div(chartInstance, c5_s_data[c5_b_q - 1], c5_scale);
        c5_eqds = c5_eml_div(chartInstance, c5_e[c5_b_q - 1], c5_scale);
        c5_qd_b = c5_eml_div(chartInstance, (c5_smm1 + c5_sm) * (c5_smm1 - c5_sm)
                             + c5_emm1 * c5_emm1, 2.0);
        c5_lb_c = c5_sm * c5_emm1;
        c5_lb_c *= c5_lb_c;
        c5_shift = 0.0;
        guard1 = false;
        if (c5_qd_b != 0.0) {
          guard1 = true;
        } else {
          if (c5_lb_c != 0.0) {
            guard1 = true;
          }
        }

        if (guard1 == true) {
          c5_shift = c5_qd_b * c5_qd_b + c5_lb_c;
          c5_b_sqrt(chartInstance, &c5_shift);
          if (c5_qd_b < 0.0) {
            c5_shift = -c5_shift;
          }

          c5_shift = c5_eml_div(chartInstance, c5_lb_c, c5_qd_b + c5_shift);
        }

        c5_f = (c5_sqds + c5_sm) * (c5_sqds - c5_sm) + c5_shift;
        c5_g = c5_sqds * c5_eqds;
        c5_k_q = c5_b_q;
        c5_b_mm1 = c5_mm1;
        c5_wf_a = c5_k_q;
        c5_rd_b = c5_b_mm1;
        c5_xf_a = c5_wf_a;
        c5_sd_b = c5_rd_b;
        if (c5_xf_a > c5_sd_b) {
          c5_o_overflow = false;
        } else {
          c5_eml_switch_helper(chartInstance);
          c5_o_overflow = (c5_sd_b > 2147483646);
        }

        if (c5_o_overflow) {
          c5_check_forloop_overflow_error(chartInstance, true);
        }

        for (c5_d_k = c5_k_q; c5_d_k <= c5_b_mm1; c5_d_k++) {
          c5_b_k = c5_d_k;
          c5_yf_a = c5_b_k;
          c5_ag_a = c5_yf_a;
          c5_km1 = c5_ag_a;
          c5_bg_a = c5_b_k;
          c5_cg_a = c5_bg_a;
          c5_kp1 = c5_cg_a;
          c5_c_f = c5_f;
          c5_unusedU1 = c5_g;
          c5_b_eml_xrotg(chartInstance, &c5_c_f, &c5_unusedU1, &c5_d_cs,
                         &c5_d_sn);
          c5_f = c5_c_f;
          c5_b_cs = c5_d_cs;
          c5_b_sn = c5_d_sn;
          if (c5_b_k > c5_b_q) {
            c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_km1 - 1, 1, 6, 1, 0) - 1] =
              c5_f;
          }

          c5_f = c5_b_cs * c5_s_data[c5_b_k - 1] + c5_b_sn * c5_e[c5_b_k - 1];
          c5_e[c5_b_k - 1] = c5_b_cs * c5_e[c5_b_k - 1] - c5_b_sn *
            c5_s_data[c5_b_k - 1];
          c5_g = c5_b_sn * c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_kp1 + 1,
            1, 6, 1, 0) - 1];
          c5_s_data[c5_kp1] *= c5_b_cs;
          c5_dg_a = c5_b_k;
          c5_eg_a = c5_dg_a;
          c5_mb_c = c5_eg_a;
          c5_td_b = c5_mb_c - 1;
          c5_ud_b = c5_td_b;
          c5_nb_c = 6 * c5_ud_b;
          c5_vd_b = c5_nb_c;
          c5_wd_b = c5_vd_b;
          c5_colk = c5_wd_b;
          c5_xd_b = c5_b_k;
          c5_yd_b = c5_xd_b;
          c5_ob_c = 6 * c5_yd_b;
          c5_ae_b = c5_ob_c;
          c5_be_b = c5_ae_b;
          c5_colkp1 = c5_be_b;
          c5_d_eml_xrot(chartInstance, c5_Vf, c5_colk + 1, c5_colkp1 + 1,
                        c5_b_cs, c5_b_sn);
          c5_d_f = c5_f;
          c5_unusedU2 = c5_g;
          c5_b_eml_xrotg(chartInstance, &c5_d_f, &c5_unusedU2, &c5_e_cs,
                         &c5_e_sn);
          c5_f = c5_d_f;
          c5_b_cs = c5_e_cs;
          c5_b_sn = c5_e_sn;
          c5_s_data[c5_b_k - 1] = c5_f;
          c5_f = c5_b_cs * c5_e[c5_b_k - 1] + c5_b_sn * c5_s_data[c5_kp1];
          c5_s_data[c5_kp1] = -c5_b_sn * c5_e[c5_b_k - 1] + c5_b_cs *
            c5_s_data[c5_kp1];
          c5_g = c5_b_sn * c5_e[c5_kp1];
          c5_e[c5_kp1] *= c5_b_cs;
          if (c5_b_k < c5_n) {
            c5_fg_a = c5_b_k;
            c5_gg_a = c5_fg_a;
            c5_pb_c = c5_gg_a;
            c5_hg_a = c5_nru;
            c5_ce_b = c5_pb_c - 1;
            c5_ig_a = c5_hg_a;
            c5_de_b = c5_ce_b;
            c5_qb_c = c5_ig_a * c5_de_b;
            c5_ee_b = c5_qb_c;
            c5_fe_b = c5_ee_b;
            c5_colk = c5_fe_b;
            c5_jg_a = c5_nru;
            c5_ge_b = c5_b_k;
            c5_kg_a = c5_jg_a;
            c5_he_b = c5_ge_b;
            c5_rb_c = c5_kg_a * c5_he_b;
            c5_ie_b = c5_rb_c;
            c5_je_b = c5_ie_b;
            c5_colkp1 = c5_je_b;
            c5_c_eml_xrot(chartInstance, c5_n, c5_U_data, c5_U_sizes, c5_colk +
                          1, c5_colkp1 + 1, c5_b_cs, c5_b_sn);
          }
        }

        c5_lg_a = c5_m;
        c5_mg_a = c5_lg_a;
        c5_sb_c = c5_mg_a;
        c5_e[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_sb_c - 1, 1, 6, 1, 0) - 1] =
          c5_f;
        c5_iter++;
        break;

       default:
        if (c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_q, 1, 6, 1, 0) - 1] <
            0.0) {
          c5_s_data[c5_b_q - 1] = -c5_s_data[c5_b_q - 1];
          c5_ng_a = c5_b_q;
          c5_vg_a = c5_ng_a;
          c5_tb_c = c5_vg_a;
          c5_ke_b = c5_tb_c - 1;
          c5_ue_b = c5_ke_b;
          c5_ub_c = 6 * c5_ue_b;
          c5_le_b = c5_ub_c;
          c5_ve_b = c5_le_b;
          c5_colq = c5_ve_b;
          c5_e_eml_scalar_eg(chartInstance);
          c5_d8 = -1.0;
          c5_j_eml_xscal(chartInstance, c5_d8, c5_Vf, c5_colq + 1);
        }

        c5_og_a = c5_b_q;
        c5_wg_a = c5_og_a;
        c5_qp1 = c5_wg_a;
        exitg3 = false;
        while ((exitg3 == false) && (c5_b_q < c5_mm)) {
          if (c5_s_data[c5_b_q - 1] < c5_s_data[_SFD_EML_ARRAY_BOUNDS_CHECK("",
               c5_qp1 + 1, 1, 6, 1, 0) - 1]) {
            c5_rt = c5_s_data[c5_b_q - 1];
            c5_s_data[c5_b_q - 1] = c5_s_data[c5_qp1];
            c5_s_data[c5_qp1] = c5_rt;
            c5_qg_a = c5_b_q;
            c5_yg_a = c5_qg_a;
            c5_vb_c = c5_yg_a;
            c5_me_b = c5_vb_c - 1;
            c5_we_b = c5_me_b;
            c5_wb_c = 6 * c5_we_b;
            c5_ne_b = c5_wb_c;
            c5_xe_b = c5_ne_b;
            c5_colq = c5_xe_b;
            c5_oe_b = c5_b_q;
            c5_ye_b = c5_oe_b;
            c5_xb_c = 6 * c5_ye_b;
            c5_pe_b = c5_xb_c;
            c5_af_b = c5_pe_b;
            c5_colqp1 = c5_af_b;
            c5_d_eml_xswap(chartInstance, c5_Vf, c5_colq + 1, c5_colqp1 + 1);
            c5_rg_a = c5_b_q;
            c5_ah_a = c5_rg_a;
            c5_yb_c = c5_ah_a;
            c5_sg_a = c5_nru;
            c5_qe_b = c5_yb_c - 1;
            c5_bh_a = c5_sg_a;
            c5_bf_b = c5_qe_b;
            c5_ac_c = c5_bh_a * c5_bf_b;
            c5_re_b = c5_ac_c;
            c5_cf_b = c5_re_b;
            c5_colq = c5_cf_b;
            c5_tg_a = c5_nru;
            c5_se_b = c5_b_q;
            c5_ch_a = c5_tg_a;
            c5_df_b = c5_se_b;
            c5_bc_c = c5_ch_a * c5_df_b;
            c5_te_b = c5_bc_c;
            c5_ef_b = c5_te_b;
            c5_colqp1 = c5_ef_b;
            c5_c_eml_xswap(chartInstance, c5_n, c5_U_data, c5_U_sizes, c5_colq +
                           1, c5_colqp1 + 1);
            c5_b_q = c5_qp1 + 1;
            c5_ug_a = c5_b_q;
            c5_dh_a = c5_ug_a;
            c5_qp1 = c5_dh_a;
          } else {
            exitg3 = true;
          }
        }

        c5_iter = 0.0;
        c5_pg_a = c5_m;
        c5_xg_a = c5_pg_a - 1;
        c5_m = c5_xg_a;
        break;
      }
    }
  }

  c5_tmp_sizes = 6;
  for (c5_i100 = 0; c5_i100 < 6; c5_i100++) {
    c5_tmp_data[c5_i100] = 0.0;
  }

  *c5_S_sizes = c5_tmp_sizes;
  c5_eml_switch_helper(chartInstance);
  for (c5_e_k = 1; c5_e_k < 7; c5_e_k++) {
    c5_b_k = c5_e_k - 1;
    c5_S_data[c5_b_k] = c5_s_data[c5_b_k];
  }

  c5_b_tmp_sizes[0] = 6;
  c5_b_tmp_sizes[1] = 6;
  c5_i101 = c5_b_tmp_sizes[0];
  c5_i102 = c5_b_tmp_sizes[1];
  for (c5_i103 = 0; c5_i103 < 36; c5_i103++) {
    c5_b_tmp_data[c5_i103] = 0.0;
  }

  for (c5_i104 = 0; c5_i104 < 2; c5_i104++) {
    c5_V_sizes[c5_i104] = c5_b_tmp_sizes[c5_i104];
  }

  c5_eml_switch_helper(chartInstance);
  for (c5_j = 1; c5_j < 7; c5_j++) {
    c5_b_j = c5_j - 1;
    for (c5_i = 1; c5_i < 7; c5_i++) {
      c5_b_i = c5_i - 1;
      c5_V_data[c5_b_i + c5_V_sizes[0] * c5_b_j] = c5_Vf[c5_b_i + 6 * c5_b_j];
    }
  }
}

static real_T c5_eml_xnrm2
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0)
{
  real_T c5_y;
  int32_T c5_b_n;
  int32_T c5_b_ix0;
  int32_T c5_c_n;
  int32_T c5_c_ix0;
  int32_T c5_var;
  ptrdiff_t c5_n_t;
  ptrdiff_t c5_incx_t;
  double * c5_xix0_t;
  c5_b_n = c5_n;
  c5_b_ix0 = c5_ix0;
  c5_threshold(chartInstance);
  if (c5_b_n < 1) {
    c5_y = 0.0;
  } else {
    c5_c_n = c5_b_n;
    c5_c_ix0 = c5_b_ix0;
    c5_var = c5_c_n;
    c5_n_t = (ptrdiff_t)(c5_var);
    c5_incx_t = (ptrdiff_t)(1);
    c5_xix0_t = (double *)(&c5_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_ix0,
      1, c5_x_sizes[0] * 6, 1, 0) - 1]);
    c5_y = dnrm2(&c5_n_t, c5_xix0_t, &c5_incx_t);
  }

  return c5_y;
}

static void c5_c_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T
   c5_ix0, real_T c5_b_x_data[], int32_T c5_b_x_sizes[2])
{
  int32_T c5_x;
  int32_T c5_b_x;
  int32_T c5_loop_ub;
  int32_T c5_i105;
  c5_b_x_sizes[0] = c5_x_sizes[0];
  c5_b_x_sizes[1] = 6;
  c5_x = c5_b_x_sizes[0];
  c5_b_x = c5_b_x_sizes[1];
  c5_loop_ub = c5_x_sizes[0] * c5_x_sizes[1] - 1;
  for (c5_i105 = 0; c5_i105 <= c5_loop_ub; c5_i105++) {
    c5_b_x_data[c5_i105] = c5_x_data[c5_i105];
  }

  c5_h_eml_xscal(chartInstance, c5_n, c5_a, c5_b_x_data, c5_b_x_sizes, c5_ix0);
}

static real_T c5_b_eml_xdotc
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0,
   real_T c5_y_data[], int32_T c5_y_sizes[2], int32_T c5_iy0)
{
  real_T c5_d;
  int32_T c5_b_n;
  int32_T c5_b_ix0;
  int32_T c5_b_iy0;
  int32_T c5_c_n;
  int32_T c5_c_ix0;
  int32_T c5_c_iy0;
  int32_T c5_d_n;
  int32_T c5_d_ix0;
  int32_T c5_d_iy0;
  int32_T c5_var;
  ptrdiff_t c5_n_t;
  ptrdiff_t c5_incx_t;
  ptrdiff_t c5_incy_t;
  double * c5_xix0_t;
  double * c5_yiy0_t;
  c5_b_n = c5_n;
  c5_b_ix0 = c5_ix0;
  c5_b_iy0 = c5_iy0;
  c5_c_n = c5_b_n;
  c5_c_ix0 = c5_b_ix0;
  c5_c_iy0 = c5_b_iy0;
  c5_c_threshold(chartInstance);
  if (c5_c_n < 1) {
    c5_b_scalarEg(chartInstance);
    c5_d = 0.0;
  } else {
    c5_d_n = c5_c_n;
    c5_d_ix0 = c5_c_ix0;
    c5_d_iy0 = c5_c_iy0;
    c5_var = c5_d_n;
    c5_n_t = (ptrdiff_t)(c5_var);
    c5_incx_t = (ptrdiff_t)(1);
    c5_incy_t = (ptrdiff_t)(1);
    c5_xix0_t = (double *)(&c5_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_d_ix0,
      1, c5_x_sizes[0] * 6, 1, 0) - 1]);
    c5_yiy0_t = (double *)(&c5_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_d_iy0,
      1, c5_y_sizes[0] * 6, 1, 0) - 1]);
    c5_d = ddot(&c5_n_t, c5_xix0_t, &c5_incx_t, c5_yiy0_t, &c5_incy_t);
  }

  return c5_d;
}

static void c5_b_scalarEg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_b_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, int32_T c5_ix0, real_T c5_y_data[], int32_T
   c5_y_sizes[2], int32_T c5_iy0, real_T c5_b_y_data[], int32_T c5_b_y_sizes[2])
{
  int32_T c5_y;
  int32_T c5_b_y;
  int32_T c5_loop_ub;
  int32_T c5_i106;
  c5_b_y_sizes[0] = c5_y_sizes[0];
  c5_b_y_sizes[1] = 6;
  c5_y = c5_b_y_sizes[0];
  c5_b_y = c5_b_y_sizes[1];
  c5_loop_ub = c5_y_sizes[0] * c5_y_sizes[1] - 1;
  for (c5_i106 = 0; c5_i106 <= c5_loop_ub; c5_i106++) {
    c5_b_y_data[c5_i106] = c5_y_data[c5_i106];
  }

  c5_g_eml_xaxpy(chartInstance, c5_n, c5_a, c5_ix0, c5_b_y_data, c5_b_y_sizes,
                 c5_iy0);
}

static real_T c5_b_eml_xnrm2
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x[6], int32_T c5_ix0)
{
  real_T c5_y;
  int32_T c5_b_n;
  int32_T c5_b_ix0;
  int32_T c5_c_n;
  int32_T c5_c_ix0;
  real_T c5_b_x;
  real_T c5_c_x;
  real_T c5_scale;
  int32_T c5_kstart;
  int32_T c5_a;
  int32_T c5_c;
  int32_T c5_b_a;
  int32_T c5_b_c;
  int32_T c5_c_a;
  int32_T c5_b;
  int32_T c5_kend;
  int32_T c5_b_kstart;
  int32_T c5_b_kend;
  int32_T c5_d_a;
  int32_T c5_b_b;
  int32_T c5_e_a;
  int32_T c5_c_b;
  boolean_T c5_overflow;
  int32_T c5_k;
  int32_T c5_b_k;
  real_T c5_d_x;
  real_T c5_e_x;
  real_T c5_absxk;
  real_T c5_t;
  c5_b_n = c5_n;
  c5_b_ix0 = c5_ix0;
  c5_threshold(chartInstance);
  c5_c_n = c5_b_n;
  c5_c_ix0 = c5_b_ix0;
  c5_y = 0.0;
  if (c5_c_n < 1) {
  } else if (c5_c_n == 1) {
    c5_b_x = c5_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_ix0, 1, 6, 1, 0) - 1];
    c5_c_x = c5_b_x;
    c5_y = muDoubleScalarAbs(c5_c_x);
  } else {
    c5_realmin(chartInstance);
    c5_scale = 2.2250738585072014E-308;
    c5_kstart = c5_c_ix0;
    c5_a = c5_c_n;
    c5_c = c5_a;
    c5_b_a = c5_c - 1;
    c5_b_c = c5_b_a;
    c5_c_a = c5_kstart;
    c5_b = c5_b_c;
    c5_kend = c5_c_a + c5_b;
    c5_b_kstart = c5_kstart;
    c5_b_kend = c5_kend;
    c5_d_a = c5_b_kstart;
    c5_b_b = c5_b_kend;
    c5_e_a = c5_d_a;
    c5_c_b = c5_b_b;
    if (c5_e_a > c5_c_b) {
      c5_overflow = false;
    } else {
      c5_eml_switch_helper(chartInstance);
      c5_overflow = (c5_c_b > 2147483646);
    }

    if (c5_overflow) {
      c5_check_forloop_overflow_error(chartInstance, true);
    }

    for (c5_k = c5_b_kstart; c5_k <= c5_b_kend; c5_k++) {
      c5_b_k = c5_k;
      c5_d_x = c5_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_k, 1, 6, 1, 0) - 1];
      c5_e_x = c5_d_x;
      c5_absxk = muDoubleScalarAbs(c5_e_x);
      if (c5_absxk > c5_scale) {
        c5_t = c5_scale / c5_absxk;
        c5_y = 1.0 + c5_y * c5_t * c5_t;
        c5_scale = c5_absxk;
      } else {
        c5_t = c5_absxk / c5_scale;
        c5_y += c5_t * c5_t;
      }
    }

    c5_y = c5_scale * muDoubleScalarSqrt(c5_y);
  }

  return c5_y;
}

static void c5_d_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x[6], int32_T c5_ix0, real_T c5_b_x[6])
{
  int32_T c5_i107;
  for (c5_i107 = 0; c5_i107 < 6; c5_i107++) {
    c5_b_x[c5_i107] = c5_x[c5_i107];
  }

  c5_i_eml_xscal(chartInstance, c5_n, c5_a, c5_b_x, c5_ix0);
}

static void c5_c_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T
   c5_ix0, real_T c5_y_data[], int32_T c5_y_sizes, int32_T c5_iy0, real_T
   c5_b_y_data[], int32_T *c5_b_y_sizes)
{
  int32_T c5_loop_ub;
  int32_T c5_i108;
  int32_T c5_b_x_sizes[2];
  int32_T c5_x;
  int32_T c5_b_x;
  int32_T c5_b_loop_ub;
  int32_T c5_i109;
  real_T c5_b_x_data[72];
  *c5_b_y_sizes = c5_y_sizes;
  c5_loop_ub = c5_y_sizes - 1;
  for (c5_i108 = 0; c5_i108 <= c5_loop_ub; c5_i108++) {
    c5_b_y_data[c5_i108] = c5_y_data[c5_i108];
  }

  c5_b_x_sizes[0] = c5_x_sizes[0];
  c5_b_x_sizes[1] = 6;
  c5_x = c5_b_x_sizes[0];
  c5_b_x = c5_b_x_sizes[1];
  c5_b_loop_ub = c5_x_sizes[0] * c5_x_sizes[1] - 1;
  for (c5_i109 = 0; c5_i109 <= c5_b_loop_ub; c5_i109++) {
    c5_b_x_data[c5_i109] = c5_x_data[c5_i109];
  }

  c5_h_eml_xaxpy(chartInstance, c5_n, c5_a, c5_b_x_data, c5_b_x_sizes, c5_ix0,
                 c5_b_y_data, c5_b_y_sizes, c5_iy0);
}

static void c5_d_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes, int32_T
   c5_ix0, real_T c5_y_data[], int32_T c5_y_sizes[2], int32_T c5_iy0, real_T
   c5_b_y_data[], int32_T c5_b_y_sizes[2])
{
  int32_T c5_y;
  int32_T c5_b_y;
  int32_T c5_loop_ub;
  int32_T c5_i110;
  int32_T c5_b_x_sizes;
  int32_T c5_b_loop_ub;
  int32_T c5_i111;
  real_T c5_b_x_data[12];
  c5_b_y_sizes[0] = c5_y_sizes[0];
  c5_b_y_sizes[1] = 6;
  c5_y = c5_b_y_sizes[0];
  c5_b_y = c5_b_y_sizes[1];
  c5_loop_ub = c5_y_sizes[0] * c5_y_sizes[1] - 1;
  for (c5_i110 = 0; c5_i110 <= c5_loop_ub; c5_i110++) {
    c5_b_y_data[c5_i110] = c5_y_data[c5_i110];
  }

  c5_b_x_sizes = c5_x_sizes;
  c5_b_loop_ub = c5_x_sizes - 1;
  for (c5_i111 = 0; c5_i111 <= c5_b_loop_ub; c5_i111++) {
    c5_b_x_data[c5_i111] = c5_x_data[c5_i111];
  }

  c5_i_eml_xaxpy(chartInstance, c5_n, c5_a, c5_b_x_data, c5_b_x_sizes, c5_ix0,
                 c5_b_y_data, c5_b_y_sizes, c5_iy0);
}

static real_T c5_c_eml_xdotc
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x[36], int32_T c5_ix0, real_T c5_y[36], int32_T
   c5_iy0)
{
  real_T c5_d;
  int32_T c5_b_n;
  int32_T c5_b_ix0;
  int32_T c5_b_iy0;
  int32_T c5_c_n;
  int32_T c5_c_ix0;
  int32_T c5_c_iy0;
  int32_T c5_d_n;
  int32_T c5_d_ix0;
  int32_T c5_d_iy0;
  int32_T c5_e_n;
  int32_T c5_e_ix0;
  int32_T c5_e_iy0;
  int32_T c5_ix;
  int32_T c5_iy;
  int32_T c5_f_n;
  int32_T c5_b;
  int32_T c5_b_b;
  boolean_T c5_overflow;
  int32_T c5_k;
  int32_T c5_a;
  int32_T c5_b_a;
  c5_b_n = c5_n;
  c5_b_ix0 = c5_ix0;
  c5_b_iy0 = c5_iy0;
  c5_c_n = c5_b_n;
  c5_c_ix0 = c5_b_ix0;
  c5_c_iy0 = c5_b_iy0;
  c5_c_threshold(chartInstance);
  c5_d_n = c5_c_n;
  c5_d_ix0 = c5_c_ix0;
  c5_d_iy0 = c5_c_iy0;
  c5_e_n = c5_d_n;
  c5_e_ix0 = c5_d_ix0;
  c5_e_iy0 = c5_d_iy0;
  c5_d = 0.0;
  if (c5_e_n < 1) {
  } else {
    c5_ix = c5_e_ix0;
    c5_iy = c5_e_iy0;
    c5_f_n = c5_e_n;
    c5_b = c5_f_n;
    c5_b_b = c5_b;
    if (1 > c5_b_b) {
      c5_overflow = false;
    } else {
      c5_eml_switch_helper(chartInstance);
      c5_overflow = (c5_b_b > 2147483646);
    }

    if (c5_overflow) {
      c5_check_forloop_overflow_error(chartInstance, true);
    }

    for (c5_k = 1; c5_k <= c5_f_n; c5_k++) {
      c5_d += c5_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_ix, 1, 36, 1, 0) - 1] *
        c5_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_iy, 1, 36, 1, 0) - 1];
      c5_a = c5_ix + 1;
      c5_ix = c5_a;
      c5_b_a = c5_iy + 1;
      c5_iy = c5_b_a;
    }
  }

  return c5_d;
}

static void c5_e_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, int32_T c5_ix0, real_T c5_y[36], int32_T c5_iy0,
   real_T c5_b_y[36])
{
  int32_T c5_i112;
  for (c5_i112 = 0; c5_i112 < 36; c5_i112++) {
    c5_b_y[c5_i112] = c5_y[c5_i112];
  }

  c5_j_eml_xaxpy(chartInstance, c5_n, c5_a, c5_ix0, c5_b_y, c5_iy0);
}

static void c5_e_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_a, real_T c5_x[36], int32_T c5_ix0, real_T c5_b_x[36])
{
  int32_T c5_i113;
  for (c5_i113 = 0; c5_i113 < 36; c5_i113++) {
    c5_b_x[c5_i113] = c5_x[c5_i113];
  }

  c5_j_eml_xscal(chartInstance, c5_a, c5_b_x, c5_ix0);
}

static void c5_b_eml_xrot
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x[36], int32_T c5_ix0, int32_T c5_iy0, real_T c5_c, real_T c5_s,
   real_T c5_b_x[36])
{
  int32_T c5_i114;
  for (c5_i114 = 0; c5_i114 < 36; c5_i114++) {
    c5_b_x[c5_i114] = c5_x[c5_i114];
  }

  c5_d_eml_xrot(chartInstance, c5_b_x, c5_ix0, c5_iy0, c5_c, c5_s);
}

static void c5_e_eml_scalar_eg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_b_eml_xswap
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x[36], int32_T c5_ix0, int32_T c5_iy0, real_T c5_b_x[36])
{
  int32_T c5_i115;
  for (c5_i115 = 0; c5_i115 < 36; c5_i115++) {
    c5_b_x[c5_i115] = c5_x[c5_i115];
  }

  c5_d_eml_xswap(chartInstance, c5_b_x, c5_ix0, c5_iy0);
}

static void c5_g_threshold
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_eml_xgemm
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, int32_T c5_k, real_T c5_A_data[], int32_T c5_A_sizes[2], real_T
   c5_B_data[], int32_T c5_B_sizes[2], int32_T c5_ldb, real_T c5_C_data[],
   int32_T c5_C_sizes[2], real_T c5_b_C_data[], int32_T c5_b_C_sizes[2])
{
  int32_T c5_C;
  int32_T c5_b_C;
  int32_T c5_loop_ub;
  int32_T c5_i116;
  int32_T c5_b_A_sizes[2];
  int32_T c5_A;
  int32_T c5_b_A;
  int32_T c5_i117;
  real_T c5_b_A_data[36];
  int32_T c5_b_B_sizes[2];
  int32_T c5_B;
  int32_T c5_b_B;
  int32_T c5_b_loop_ub;
  int32_T c5_i118;
  real_T c5_b_B_data[72];
  (void)c5_A_sizes;
  c5_b_C_sizes[0] = 6;
  c5_b_C_sizes[1] = c5_C_sizes[1];
  c5_C = c5_b_C_sizes[0];
  c5_b_C = c5_b_C_sizes[1];
  c5_loop_ub = c5_C_sizes[0] * c5_C_sizes[1] - 1;
  for (c5_i116 = 0; c5_i116 <= c5_loop_ub; c5_i116++) {
    c5_b_C_data[c5_i116] = c5_C_data[c5_i116];
  }

  c5_b_A_sizes[0] = 6;
  c5_b_A_sizes[1] = 6;
  c5_A = c5_b_A_sizes[0];
  c5_b_A = c5_b_A_sizes[1];
  for (c5_i117 = 0; c5_i117 < 36; c5_i117++) {
    c5_b_A_data[c5_i117] = c5_A_data[c5_i117];
  }

  c5_b_B_sizes[0] = c5_B_sizes[0];
  c5_b_B_sizes[1] = 6;
  c5_B = c5_b_B_sizes[0];
  c5_b_B = c5_b_B_sizes[1];
  c5_b_loop_ub = c5_B_sizes[0] * c5_B_sizes[1] - 1;
  for (c5_i118 = 0; c5_i118 <= c5_b_loop_ub; c5_i118++) {
    c5_b_B_data[c5_i118] = c5_B_data[c5_i118];
  }

  c5_c_eml_xgemm(chartInstance, c5_n, c5_k, c5_b_A_data, c5_b_A_sizes,
                 c5_b_B_data, c5_b_B_sizes, c5_ldb, c5_b_C_data, c5_b_C_sizes);
}

static void c5_f_eml_scalar_eg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_b_eml_xgemm
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_k, real_T c5_A_data[], int32_T c5_A_sizes[2], real_T c5_B_data[],
   int32_T c5_B_sizes[2], int32_T c5_ldb, real_T c5_C[138], real_T c5_b_C[138])
{
  int32_T c5_i119;
  int32_T c5_b_A_sizes[2];
  int32_T c5_A;
  int32_T c5_b_A;
  int32_T c5_loop_ub;
  int32_T c5_i120;
  real_T c5_b_A_data[72];
  int32_T c5_b_B_sizes[2];
  int32_T c5_B;
  int32_T c5_b_B;
  int32_T c5_b_loop_ub;
  int32_T c5_i121;
  real_T c5_b_B_data[276];
  for (c5_i119 = 0; c5_i119 < 138; c5_i119++) {
    c5_b_C[c5_i119] = c5_C[c5_i119];
  }

  c5_b_A_sizes[0] = 6;
  c5_b_A_sizes[1] = c5_A_sizes[1];
  c5_A = c5_b_A_sizes[0];
  c5_b_A = c5_b_A_sizes[1];
  c5_loop_ub = c5_A_sizes[0] * c5_A_sizes[1] - 1;
  for (c5_i120 = 0; c5_i120 <= c5_loop_ub; c5_i120++) {
    c5_b_A_data[c5_i120] = c5_A_data[c5_i120];
  }

  c5_b_B_sizes[0] = c5_B_sizes[0];
  c5_b_B_sizes[1] = 23;
  c5_B = c5_b_B_sizes[0];
  c5_b_B = c5_b_B_sizes[1];
  c5_b_loop_ub = c5_B_sizes[0] * c5_B_sizes[1] - 1;
  for (c5_i121 = 0; c5_i121 <= c5_b_loop_ub; c5_i121++) {
    c5_b_B_data[c5_i121] = c5_B_data[c5_i121];
  }

  c5_d_eml_xgemm(chartInstance, c5_k, c5_b_A_data, c5_b_A_sizes, c5_b_B_data,
                 c5_b_B_sizes, c5_ldb, c5_b_C);
}

static void c5_g_eml_scalar_eg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *c5_f_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance;
  chartInstance = (SFc5_balancingController_with_OOT_and_transferInstanceStruct *)
    chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(int32_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static int32_T c5_e_emlrt_marshallIn
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  int32_T c5_y;
  int32_T c5_i122;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_i122, 1, 6, 0U, 0, 0U, 0);
  c5_y = c5_i122;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_b_sfEvent;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  int32_T c5_y;
  SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance;
  chartInstance = (SFc5_balancingController_with_OOT_and_transferInstanceStruct *)
    chartInstanceVoid;
  c5_b_sfEvent = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_sfEvent),
    &c5_thisId);
  sf_mex_destroy(&c5_b_sfEvent);
  *(int32_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static uint8_T c5_f_emlrt_marshallIn
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_b_is_active_c5_balancingController_with_OOT_and_transfer,
   const char_T *c5_identifier)
{
  uint8_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c5_b_is_active_c5_balancingController_with_OOT_and_transfer), &c5_thisId);
  sf_mex_destroy(&c5_b_is_active_c5_balancingController_with_OOT_and_transfer);
  return c5_y;
}

static uint8_T c5_g_emlrt_marshallIn
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  uint8_T c5_y;
  uint8_T c5_u0;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_u0, 1, 3, 0U, 0, 0U, 0);
  c5_y = c5_u0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_f_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, int32_T c5_ix0, real_T c5_y_data[], int32_T
   c5_y_sizes[2], int32_T c5_iy0)
{
  int32_T c5_b_n;
  real_T c5_b_a;
  int32_T c5_b_ix0;
  int32_T c5_b_iy0;
  int32_T c5_c_n;
  real_T c5_c_a;
  int32_T c5_c_ix0;
  int32_T c5_c_iy0;
  int32_T c5_var;
  ptrdiff_t c5_n_t;
  ptrdiff_t c5_incx_t;
  ptrdiff_t c5_incy_t;
  double * c5_a_t;
  int32_T c5_y[1];
  double * c5_yiy0_t;
  int32_T c5_b_y[1];
  double * c5_yix0_t;
  c5_b_n = c5_n;
  c5_b_a = c5_a;
  c5_b_ix0 = c5_ix0;
  c5_b_iy0 = c5_iy0;
  c5_d_threshold(chartInstance);
  if (c5_b_n < 1) {
  } else {
    c5_c_n = c5_b_n;
    c5_c_a = c5_b_a;
    c5_c_ix0 = c5_b_ix0;
    c5_c_iy0 = c5_b_iy0;
    c5_var = c5_c_n;
    c5_n_t = (ptrdiff_t)(c5_var);
    c5_incx_t = (ptrdiff_t)(1);
    c5_incy_t = (ptrdiff_t)(1);
    c5_a_t = (double *)(&c5_c_a);
    c5_y[0] = c5_y_sizes[0] * 6;
    c5_yiy0_t = (double *)(&c5_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_iy0,
      1, c5_y_sizes[0] * 6, 1, 0) - 1]);
    c5_b_y[0] = c5_y_sizes[0] * 6;
    c5_yix0_t = (double *)(&c5_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_ix0,
      1, c5_y_sizes[0] * 6, 1, 0) - 1]);
    daxpy(&c5_n_t, c5_a_t, c5_yix0_t, &c5_incx_t, c5_yiy0_t, &c5_incy_t);
  }
}

static void c5_f_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0)
{
  real_T c5_b_a;
  int32_T c5_b_ix0;
  real_T c5_c_a;
  int32_T c5_c_ix0;
  int32_T c5_d_ix0;
  int32_T c5_d_a;
  int32_T c5_i123;
  int32_T c5_e_a;
  int32_T c5_b;
  int32_T c5_f_a;
  int32_T c5_b_b;
  boolean_T c5_overflow;
  int32_T c5_k;
  int32_T c5_b_k;
  (void)c5_x_sizes;
  c5_b_a = c5_a;
  c5_b_ix0 = c5_ix0;
  c5_b_threshold(chartInstance);
  c5_c_a = c5_b_a;
  c5_c_ix0 = c5_b_ix0;
  c5_d_ix0 = c5_c_ix0;
  c5_d_a = c5_c_ix0 + 5;
  c5_i123 = c5_d_a;
  c5_e_a = c5_d_ix0;
  c5_b = c5_i123;
  c5_f_a = c5_e_a;
  c5_b_b = c5_b;
  if (c5_f_a > c5_b_b) {
    c5_overflow = false;
  } else {
    c5_eml_switch_helper(chartInstance);
    c5_overflow = (c5_b_b > 2147483646);
  }

  if (c5_overflow) {
    c5_check_forloop_overflow_error(chartInstance, true);
  }

  for (c5_k = c5_d_ix0; c5_k <= c5_i123; c5_k++) {
    c5_b_k = c5_k;
    c5_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_k, 1, 36, 1, 0) - 1] = c5_c_a
      * c5_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_k, 1, 36, 1, 0) - 1];
  }
}

static void c5_g_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T
   c5_ix0)
{
  int32_T c5_b_n;
  real_T c5_b_a;
  int32_T c5_b_ix0;
  int32_T c5_c_n;
  real_T c5_c_a;
  int32_T c5_c_ix0;
  int32_T c5_var;
  ptrdiff_t c5_n_t;
  ptrdiff_t c5_incx_t;
  int32_T c5_x[1];
  double * c5_xix0_t;
  double * c5_a_t;
  c5_b_n = c5_n;
  c5_b_a = c5_a;
  c5_b_ix0 = c5_ix0;
  c5_b_threshold(chartInstance);
  c5_c_n = c5_b_n;
  c5_c_a = c5_b_a;
  c5_c_ix0 = c5_b_ix0;
  c5_var = c5_c_n;
  c5_n_t = (ptrdiff_t)(c5_var);
  c5_incx_t = (ptrdiff_t)(1);
  c5_x[0] = c5_x_sizes[0] * 6;
  c5_xix0_t = (double *)(&c5_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_ix0, 1,
    c5_x_sizes[0] * 6, 1, 0) - 1]);
  c5_a_t = (double *)(&c5_c_a);
  dscal(&c5_n_t, c5_a_t, c5_xix0_t, &c5_incx_t);
}

static void c5_b_sqrt
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T *c5_x)
{
  if (*c5_x < 0.0) {
    c5_c_eml_error(chartInstance);
  }

  *c5_x = muDoubleScalarSqrt(*c5_x);
}

static void c5_b_eml_xrotg
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T *c5_a, real_T *c5_b, real_T *c5_c, real_T *c5_s)
{
  real_T c5_b_a;
  real_T c5_b_b;
  real_T c5_c_b;
  real_T c5_c_a;
  real_T c5_d_a;
  real_T c5_d_b;
  real_T c5_e_b;
  real_T c5_e_a;
  real_T c5_b_c;
  real_T c5_b_s;
  double * c5_a_t;
  double * c5_b_t;
  double * c5_c_t;
  double * c5_s_t;
  real_T c5_c_c;
  real_T c5_c_s;
  (void)chartInstance;
  c5_b_a = *c5_a;
  c5_b_b = *c5_b;
  c5_c_b = c5_b_b;
  c5_c_a = c5_b_a;
  c5_d_a = c5_c_a;
  c5_d_b = c5_c_b;
  c5_e_b = c5_d_b;
  c5_e_a = c5_d_a;
  c5_b_c = 0.0;
  c5_b_s = 0.0;
  c5_a_t = (double *)(&c5_e_a);
  c5_b_t = (double *)(&c5_e_b);
  c5_c_t = (double *)(&c5_b_c);
  c5_s_t = (double *)(&c5_b_s);
  drotg(c5_a_t, c5_b_t, c5_c_t, c5_s_t);
  c5_c_a = c5_e_a;
  c5_c_b = c5_e_b;
  c5_c_c = c5_b_c;
  c5_c_s = c5_b_s;
  *c5_a = c5_c_a;
  *c5_b = c5_c_b;
  *c5_c = c5_c_c;
  *c5_s = c5_c_s;
}

static void c5_c_eml_xrot
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0,
   int32_T c5_iy0, real_T c5_c, real_T c5_s)
{
  int32_T c5_b_n;
  int32_T c5_b_ix0;
  int32_T c5_b_iy0;
  real_T c5_b_c;
  real_T c5_b_s;
  int32_T c5_c_n;
  int32_T c5_c_ix0;
  int32_T c5_c_iy0;
  real_T c5_c_c;
  real_T c5_c_s;
  int32_T c5_var;
  ptrdiff_t c5_n_t;
  ptrdiff_t c5_incx_t;
  ptrdiff_t c5_incy_t;
  int32_T c5_x[1];
  double * c5_xix0_t;
  double * c5_c_t;
  double * c5_s_t;
  int32_T c5_b_x[1];
  double * c5_xiy0_t;
  c5_b_n = c5_n;
  c5_b_ix0 = c5_ix0;
  c5_b_iy0 = c5_iy0;
  c5_b_c = c5_c;
  c5_b_s = c5_s;
  c5_e_threshold(chartInstance);
  c5_c_n = c5_b_n;
  c5_c_ix0 = c5_b_ix0;
  c5_c_iy0 = c5_b_iy0;
  c5_c_c = c5_b_c;
  c5_c_s = c5_b_s;
  c5_var = c5_c_n;
  c5_n_t = (ptrdiff_t)(c5_var);
  c5_incx_t = (ptrdiff_t)(1);
  c5_incy_t = (ptrdiff_t)(1);
  c5_x[0] = c5_x_sizes[0] * 6;
  c5_xix0_t = (double *)(&c5_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_ix0, 1,
    c5_x_sizes[0] * 6, 1, 0) - 1]);
  c5_c_t = (double *)(&c5_c_c);
  c5_s_t = (double *)(&c5_c_s);
  c5_b_x[0] = c5_x_sizes[0] * 6;
  c5_xiy0_t = (double *)(&c5_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_iy0, 1,
    c5_x_sizes[0] * 6, 1, 0) - 1]);
  drot(&c5_n_t, c5_xix0_t, &c5_incx_t, c5_xiy0_t, &c5_incy_t, c5_c_t, c5_s_t);
}

static void c5_c_eml_xswap
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T c5_ix0,
   int32_T c5_iy0)
{
  int32_T c5_b_n;
  int32_T c5_b_ix0;
  int32_T c5_b_iy0;
  int32_T c5_c_n;
  int32_T c5_c_ix0;
  int32_T c5_c_iy0;
  int32_T c5_ix;
  int32_T c5_iy;
  int32_T c5_d_n;
  int32_T c5_b;
  int32_T c5_b_b;
  boolean_T c5_overflow;
  int32_T c5_k;
  int32_T c5_x[1];
  real_T c5_temp;
  int32_T c5_b_x[1];
  int32_T c5_c_x[1];
  int32_T c5_d_x[1];
  int32_T c5_a;
  int32_T c5_b_a;
  c5_b_n = c5_n;
  c5_b_ix0 = c5_ix0;
  c5_b_iy0 = c5_iy0;
  c5_c_n = c5_b_n;
  c5_c_ix0 = c5_b_ix0;
  c5_c_iy0 = c5_b_iy0;
  c5_ix = c5_c_ix0;
  c5_iy = c5_c_iy0;
  c5_d_n = c5_c_n;
  c5_b = c5_d_n;
  c5_b_b = c5_b;
  if (1 > c5_b_b) {
    c5_overflow = false;
  } else {
    c5_eml_switch_helper(chartInstance);
    c5_overflow = (c5_b_b > 2147483646);
  }

  if (c5_overflow) {
    c5_check_forloop_overflow_error(chartInstance, true);
  }

  for (c5_k = 1; c5_k <= c5_d_n; c5_k++) {
    c5_x[0] = c5_x_sizes[0] * 6;
    c5_temp = c5_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_ix, 1, c5_x_sizes[0] *
      6, 1, 0) - 1];
    c5_b_x[0] = c5_x_sizes[0] * 6;
    c5_c_x[0] = c5_x_sizes[0] * 6;
    c5_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_ix, 1, c5_x_sizes[0] * 6, 1, 0)
      - 1] = c5_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_iy, 1, c5_x_sizes[0] *
      6, 1, 0) - 1];
    c5_d_x[0] = c5_x_sizes[0] * 6;
    c5_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_iy, 1, c5_x_sizes[0] * 6, 1, 0)
      - 1] = c5_temp;
    c5_a = c5_ix + 1;
    c5_ix = c5_a;
    c5_b_a = c5_iy + 1;
    c5_iy = c5_b_a;
  }
}

static void c5_h_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T
   c5_ix0)
{
  int32_T c5_b_n;
  real_T c5_b_a;
  int32_T c5_b_ix0;
  int32_T c5_c_n;
  real_T c5_c_a;
  int32_T c5_c_ix0;
  int32_T c5_var;
  ptrdiff_t c5_n_t;
  ptrdiff_t c5_incx_t;
  int32_T c5_x[1];
  double * c5_xix0_t;
  double * c5_a_t;
  c5_b_n = c5_n;
  c5_b_a = c5_a;
  c5_b_ix0 = c5_ix0;
  c5_b_threshold(chartInstance);
  if (c5_b_n < 1) {
  } else {
    c5_c_n = c5_b_n;
    c5_c_a = c5_b_a;
    c5_c_ix0 = c5_b_ix0;
    c5_var = c5_c_n;
    c5_n_t = (ptrdiff_t)(c5_var);
    c5_incx_t = (ptrdiff_t)(1);
    c5_x[0] = c5_x_sizes[0] * 6;
    c5_xix0_t = (double *)(&c5_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_ix0,
      1, c5_x_sizes[0] * 6, 1, 0) - 1]);
    c5_a_t = (double *)(&c5_c_a);
    dscal(&c5_n_t, c5_a_t, c5_xix0_t, &c5_incx_t);
  }
}

static void c5_g_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, int32_T c5_ix0, real_T c5_y_data[], int32_T
   c5_y_sizes[2], int32_T c5_iy0)
{
  int32_T c5_b_n;
  real_T c5_b_a;
  int32_T c5_b_ix0;
  int32_T c5_b_iy0;
  int32_T c5_c_n;
  real_T c5_c_a;
  int32_T c5_c_ix0;
  int32_T c5_c_iy0;
  int32_T c5_var;
  ptrdiff_t c5_n_t;
  ptrdiff_t c5_incx_t;
  ptrdiff_t c5_incy_t;
  double * c5_a_t;
  int32_T c5_y[1];
  double * c5_yiy0_t;
  int32_T c5_b_y[1];
  double * c5_yix0_t;
  c5_b_n = c5_n;
  c5_b_a = c5_a;
  c5_b_ix0 = c5_ix0;
  c5_b_iy0 = c5_iy0;
  c5_d_threshold(chartInstance);
  if (c5_b_n < 1) {
  } else {
    c5_c_n = c5_b_n;
    c5_c_a = c5_b_a;
    c5_c_ix0 = c5_b_ix0;
    c5_c_iy0 = c5_b_iy0;
    c5_var = c5_c_n;
    c5_n_t = (ptrdiff_t)(c5_var);
    c5_incx_t = (ptrdiff_t)(1);
    c5_incy_t = (ptrdiff_t)(1);
    c5_a_t = (double *)(&c5_c_a);
    c5_y[0] = c5_y_sizes[0] * 6;
    c5_yiy0_t = (double *)(&c5_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_iy0,
      1, c5_y_sizes[0] * 6, 1, 0) - 1]);
    c5_b_y[0] = c5_y_sizes[0] * 6;
    c5_yix0_t = (double *)(&c5_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_ix0,
      1, c5_y_sizes[0] * 6, 1, 0) - 1]);
    daxpy(&c5_n_t, c5_a_t, c5_yix0_t, &c5_incx_t, c5_yiy0_t, &c5_incy_t);
  }
}

static void c5_i_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x[6], int32_T c5_ix0)
{
  int32_T c5_b_n;
  real_T c5_b_a;
  int32_T c5_b_ix0;
  int32_T c5_c_n;
  real_T c5_c_a;
  int32_T c5_c_ix0;
  int32_T c5_d_ix0;
  int32_T c5_d_a;
  int32_T c5_c;
  int32_T c5_b;
  int32_T c5_b_c;
  int32_T c5_e_a;
  int32_T c5_b_b;
  int32_T c5_i124;
  int32_T c5_f_a;
  int32_T c5_c_b;
  int32_T c5_g_a;
  int32_T c5_d_b;
  boolean_T c5_overflow;
  int32_T c5_k;
  int32_T c5_b_k;
  c5_b_n = c5_n;
  c5_b_a = c5_a;
  c5_b_ix0 = c5_ix0;
  c5_b_threshold(chartInstance);
  c5_c_n = c5_b_n;
  c5_c_a = c5_b_a;
  c5_c_ix0 = c5_b_ix0;
  c5_d_ix0 = c5_c_ix0;
  c5_d_a = c5_c_n;
  c5_c = c5_d_a;
  c5_b = c5_c - 1;
  c5_b_c = c5_b;
  c5_e_a = c5_c_ix0;
  c5_b_b = c5_b_c;
  c5_i124 = c5_e_a + c5_b_b;
  c5_f_a = c5_d_ix0;
  c5_c_b = c5_i124;
  c5_g_a = c5_f_a;
  c5_d_b = c5_c_b;
  if (c5_g_a > c5_d_b) {
    c5_overflow = false;
  } else {
    c5_eml_switch_helper(chartInstance);
    c5_overflow = (c5_d_b > 2147483646);
  }

  if (c5_overflow) {
    c5_check_forloop_overflow_error(chartInstance, true);
  }

  for (c5_k = c5_d_ix0; c5_k <= c5_i124; c5_k++) {
    c5_b_k = c5_k;
    c5_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_k, 1, 6, 1, 0) - 1] = c5_c_a *
      c5_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_k, 1, 6, 1, 0) - 1];
  }
}

static void c5_h_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes[2], int32_T
   c5_ix0, real_T c5_y_data[], int32_T *c5_y_sizes, int32_T c5_iy0)
{
  int32_T c5_b_n;
  real_T c5_b_a;
  int32_T c5_b_ix0;
  int32_T c5_b_iy0;
  int32_T c5_c_n;
  real_T c5_c_a;
  int32_T c5_c_ix0;
  int32_T c5_c_iy0;
  int32_T c5_var;
  ptrdiff_t c5_n_t;
  ptrdiff_t c5_incx_t;
  ptrdiff_t c5_incy_t;
  double * c5_a_t;
  double * c5_yiy0_t;
  double * c5_xix0_t;
  c5_b_n = c5_n;
  c5_b_a = c5_a;
  c5_b_ix0 = c5_ix0;
  c5_b_iy0 = c5_iy0;
  c5_d_threshold(chartInstance);
  if (c5_b_n < 1) {
  } else {
    c5_c_n = c5_b_n;
    c5_c_a = c5_b_a;
    c5_c_ix0 = c5_b_ix0;
    c5_c_iy0 = c5_b_iy0;
    c5_var = c5_c_n;
    c5_n_t = (ptrdiff_t)(c5_var);
    c5_incx_t = (ptrdiff_t)(1);
    c5_incy_t = (ptrdiff_t)(1);
    c5_a_t = (double *)(&c5_c_a);
    c5_yiy0_t = (double *)(&c5_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_iy0,
      1, *c5_y_sizes, 1, 0) - 1]);
    c5_xix0_t = (double *)(&c5_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_ix0,
      1, c5_x_sizes[0] * 6, 1, 0) - 1]);
    daxpy(&c5_n_t, c5_a_t, c5_xix0_t, &c5_incx_t, c5_yiy0_t, &c5_incy_t);
  }
}

static void c5_i_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, real_T c5_x_data[], int32_T c5_x_sizes, int32_T
   c5_ix0, real_T c5_y_data[], int32_T c5_y_sizes[2], int32_T c5_iy0)
{
  int32_T c5_b_n;
  real_T c5_b_a;
  int32_T c5_b_ix0;
  int32_T c5_b_iy0;
  int32_T c5_c_n;
  real_T c5_c_a;
  int32_T c5_c_ix0;
  int32_T c5_c_iy0;
  int32_T c5_var;
  ptrdiff_t c5_n_t;
  ptrdiff_t c5_incx_t;
  ptrdiff_t c5_incy_t;
  double * c5_a_t;
  int32_T c5_y[1];
  double * c5_yiy0_t;
  double * c5_xix0_t;
  (void)c5_x_sizes;
  c5_b_n = c5_n;
  c5_b_a = c5_a;
  c5_b_ix0 = c5_ix0;
  c5_b_iy0 = c5_iy0;
  c5_d_threshold(chartInstance);
  if (c5_b_n < 1) {
  } else {
    c5_c_n = c5_b_n;
    c5_c_a = c5_b_a;
    c5_c_ix0 = c5_b_ix0 - 1;
    c5_c_iy0 = c5_b_iy0;
    c5_var = c5_c_n;
    c5_n_t = (ptrdiff_t)(c5_var);
    c5_incx_t = (ptrdiff_t)(1);
    c5_incy_t = (ptrdiff_t)(1);
    c5_a_t = (double *)(&c5_c_a);
    c5_y[0] = c5_y_sizes[0] * 6;
    c5_yiy0_t = (double *)(&c5_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_iy0,
      1, c5_y_sizes[0] * 6, 1, 0) - 1]);
    c5_xix0_t = (double *)(&c5_x_data[c5_c_ix0]);
    daxpy(&c5_n_t, c5_a_t, c5_xix0_t, &c5_incx_t, c5_yiy0_t, &c5_incy_t);
  }
}

static void c5_j_eml_xaxpy
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, real_T c5_a, int32_T c5_ix0, real_T c5_y[36], int32_T c5_iy0)
{
  int32_T c5_b_n;
  real_T c5_b_a;
  int32_T c5_b_ix0;
  int32_T c5_b_iy0;
  int32_T c5_c_n;
  real_T c5_c_a;
  int32_T c5_c_ix0;
  int32_T c5_c_iy0;
  int32_T c5_d_a;
  int32_T c5_ix;
  int32_T c5_e_a;
  int32_T c5_iy;
  int32_T c5_f_a;
  int32_T c5_i125;
  int32_T c5_b;
  int32_T c5_b_b;
  int32_T c5_k;
  int32_T c5_g_a;
  int32_T c5_c;
  int32_T c5_h_a;
  int32_T c5_b_c;
  int32_T c5_i_a;
  int32_T c5_c_c;
  int32_T c5_j_a;
  int32_T c5_k_a;
  c5_b_n = c5_n;
  c5_b_a = c5_a;
  c5_b_ix0 = c5_ix0;
  c5_b_iy0 = c5_iy0;
  c5_d_threshold(chartInstance);
  c5_c_n = c5_b_n;
  c5_c_a = c5_b_a;
  c5_c_ix0 = c5_b_ix0;
  c5_c_iy0 = c5_b_iy0;
  if (c5_c_n < 1) {
  } else if (c5_c_a == 0.0) {
  } else {
    c5_d_a = c5_c_ix0 - 1;
    c5_ix = c5_d_a;
    c5_e_a = c5_c_iy0 - 1;
    c5_iy = c5_e_a;
    c5_f_a = c5_c_n - 1;
    c5_i125 = c5_f_a;
    c5_b = c5_i125;
    c5_b_b = c5_b;
    if (0 > c5_b_b) {
    } else {
      c5_eml_switch_helper(chartInstance);
    }

    for (c5_k = 0; c5_k <= c5_i125; c5_k++) {
      c5_g_a = c5_iy;
      c5_c = c5_g_a;
      c5_h_a = c5_iy;
      c5_b_c = c5_h_a;
      c5_i_a = c5_ix;
      c5_c_c = c5_i_a;
      c5_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c + 1, 1, 36, 1, 0) - 1] =
        c5_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_c + 1, 1, 36, 1, 0) - 1] +
        c5_c_a * c5_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_c + 1, 1, 36, 1, 0) -
        1];
      c5_j_a = c5_ix + 1;
      c5_ix = c5_j_a;
      c5_k_a = c5_iy + 1;
      c5_iy = c5_k_a;
    }
  }
}

static void c5_j_eml_xscal
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_a, real_T c5_x[36], int32_T c5_ix0)
{
  real_T c5_b_a;
  int32_T c5_b_ix0;
  real_T c5_c_a;
  int32_T c5_c_ix0;
  int32_T c5_d_ix0;
  int32_T c5_d_a;
  int32_T c5_i126;
  int32_T c5_e_a;
  int32_T c5_b;
  int32_T c5_f_a;
  int32_T c5_b_b;
  boolean_T c5_overflow;
  int32_T c5_k;
  int32_T c5_b_k;
  c5_b_a = c5_a;
  c5_b_ix0 = c5_ix0;
  c5_b_threshold(chartInstance);
  c5_c_a = c5_b_a;
  c5_c_ix0 = c5_b_ix0;
  c5_d_ix0 = c5_c_ix0;
  c5_d_a = c5_c_ix0 + 5;
  c5_i126 = c5_d_a;
  c5_e_a = c5_d_ix0;
  c5_b = c5_i126;
  c5_f_a = c5_e_a;
  c5_b_b = c5_b;
  if (c5_f_a > c5_b_b) {
    c5_overflow = false;
  } else {
    c5_eml_switch_helper(chartInstance);
    c5_overflow = (c5_b_b > 2147483646);
  }

  if (c5_overflow) {
    c5_check_forloop_overflow_error(chartInstance, true);
  }

  for (c5_k = c5_d_ix0; c5_k <= c5_i126; c5_k++) {
    c5_b_k = c5_k;
    c5_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_k, 1, 36, 1, 0) - 1] = c5_c_a *
      c5_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_k, 1, 36, 1, 0) - 1];
  }
}

static void c5_d_eml_xrot
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x[36], int32_T c5_ix0, int32_T c5_iy0, real_T c5_c, real_T c5_s)
{
  int32_T c5_b_ix0;
  int32_T c5_b_iy0;
  real_T c5_b_c;
  real_T c5_b_s;
  int32_T c5_c_ix0;
  int32_T c5_c_iy0;
  real_T c5_c_c;
  real_T c5_c_s;
  int32_T c5_ix;
  int32_T c5_iy;
  int32_T c5_k;
  real_T c5_temp;
  int32_T c5_a;
  int32_T c5_b_a;
  c5_b_ix0 = c5_ix0;
  c5_b_iy0 = c5_iy0;
  c5_b_c = c5_c;
  c5_b_s = c5_s;
  c5_e_threshold(chartInstance);
  c5_c_ix0 = c5_b_ix0;
  c5_c_iy0 = c5_b_iy0;
  c5_c_c = c5_b_c;
  c5_c_s = c5_b_s;
  c5_ix = c5_c_ix0 - 1;
  c5_iy = c5_c_iy0 - 1;
  for (c5_k = 1; c5_k < 7; c5_k++) {
    c5_temp = c5_c_c * c5_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_ix + 1, 1, 36, 1,
      0) - 1] + c5_c_s * c5_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_iy + 1, 1, 36,
      1, 0) - 1];
    c5_x[c5_iy] = c5_c_c * c5_x[c5_iy] - c5_c_s * c5_x[c5_ix];
    c5_x[c5_ix] = c5_temp;
    c5_a = c5_iy + 1;
    c5_iy = c5_a;
    c5_b_a = c5_ix + 1;
    c5_ix = c5_b_a;
  }
}

static void c5_d_eml_xswap
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   real_T c5_x[36], int32_T c5_ix0, int32_T c5_iy0)
{
  int32_T c5_b_ix0;
  int32_T c5_b_iy0;
  int32_T c5_c_ix0;
  int32_T c5_c_iy0;
  int32_T c5_ix;
  int32_T c5_iy;
  int32_T c5_k;
  real_T c5_temp;
  int32_T c5_a;
  int32_T c5_b_a;
  c5_b_ix0 = c5_ix0;
  c5_b_iy0 = c5_iy0;
  c5_g_threshold(chartInstance);
  c5_c_ix0 = c5_b_ix0;
  c5_c_iy0 = c5_b_iy0;
  c5_ix = c5_c_ix0;
  c5_iy = c5_c_iy0;
  for (c5_k = 1; c5_k < 7; c5_k++) {
    c5_temp = c5_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_ix, 1, 36, 1, 0) - 1];
    c5_x[c5_ix - 1] = c5_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_iy, 1, 36, 1, 0) -
      1];
    c5_x[c5_iy - 1] = c5_temp;
    c5_a = c5_ix + 1;
    c5_ix = c5_a;
    c5_b_a = c5_iy + 1;
    c5_iy = c5_b_a;
  }
}

static void c5_c_eml_xgemm
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_n, int32_T c5_k, real_T c5_A_data[], int32_T c5_A_sizes[2], real_T
   c5_B_data[], int32_T c5_B_sizes[2], int32_T c5_ldb, real_T c5_C_data[],
   int32_T c5_C_sizes[2])
{
  int32_T c5_b_n;
  int32_T c5_b_k;
  int32_T c5_b_ldb;
  int32_T c5_c_n;
  int32_T c5_c_k;
  real_T c5_alpha1;
  int32_T c5_c_ldb;
  real_T c5_beta1;
  char_T c5_TRANSB;
  char_T c5_TRANSA;
  ptrdiff_t c5_m_t;
  int32_T c5_var;
  ptrdiff_t c5_n_t;
  int32_T c5_b_var;
  ptrdiff_t c5_k_t;
  ptrdiff_t c5_lda_t;
  int32_T c5_c_var;
  ptrdiff_t c5_ldb_t;
  ptrdiff_t c5_ldc_t;
  double * c5_alpha1_t;
  double * c5_Aia0_t;
  double * c5_Bib0_t;
  double * c5_beta1_t;
  int32_T c5_iv4[1];
  double * c5_Cic0_t;
  (void)c5_A_sizes;
  (void)c5_B_sizes;
  c5_b_n = c5_n;
  c5_b_k = c5_k;
  c5_b_ldb = c5_ldb;
  c5_f_threshold(chartInstance);
  c5_c_n = c5_b_n;
  c5_c_k = c5_b_k;
  c5_alpha1 = 1.0;
  c5_c_ldb = c5_b_ldb;
  c5_beta1 = 0.0;
  c5_TRANSB = 'C';
  c5_TRANSA = 'N';
  c5_m_t = (ptrdiff_t)(6);
  c5_var = c5_c_n;
  c5_n_t = (ptrdiff_t)(c5_var);
  c5_b_var = c5_c_k;
  c5_k_t = (ptrdiff_t)(c5_b_var);
  c5_lda_t = (ptrdiff_t)(6);
  c5_c_var = c5_c_ldb;
  c5_ldb_t = (ptrdiff_t)(c5_c_var);
  c5_ldc_t = (ptrdiff_t)(6);
  c5_alpha1_t = (double *)(&c5_alpha1);
  c5_Aia0_t = (double *)(&c5_A_data[0]);
  c5_Bib0_t = (double *)(&c5_B_data[0]);
  c5_beta1_t = (double *)(&c5_beta1);
  c5_iv4[0] = 6 * c5_C_sizes[1];
  c5_Cic0_t = (double *)(&c5_C_data[0]);
  dgemm(&c5_TRANSA, &c5_TRANSB, &c5_m_t, &c5_n_t, &c5_k_t, c5_alpha1_t,
        c5_Aia0_t, &c5_lda_t, c5_Bib0_t, &c5_ldb_t, c5_beta1_t, c5_Cic0_t,
        &c5_ldc_t);
}

static void c5_d_eml_xgemm
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance,
   int32_T c5_k, real_T c5_A_data[], int32_T c5_A_sizes[2], real_T c5_B_data[],
   int32_T c5_B_sizes[2], int32_T c5_ldb, real_T c5_C[138])
{
  int32_T c5_b_k;
  int32_T c5_b_ldb;
  int32_T c5_c_k;
  real_T c5_alpha1;
  int32_T c5_c_ldb;
  real_T c5_beta1;
  char_T c5_TRANSB;
  char_T c5_TRANSA;
  ptrdiff_t c5_m_t;
  ptrdiff_t c5_n_t;
  int32_T c5_var;
  ptrdiff_t c5_k_t;
  ptrdiff_t c5_lda_t;
  int32_T c5_b_var;
  ptrdiff_t c5_ldb_t;
  ptrdiff_t c5_ldc_t;
  double * c5_alpha1_t;
  double * c5_Aia0_t;
  double * c5_Bib0_t;
  double * c5_beta1_t;
  double * c5_Cic0_t;
  (void)c5_A_sizes;
  (void)c5_B_sizes;
  c5_b_k = c5_k;
  c5_b_ldb = c5_ldb;
  c5_f_threshold(chartInstance);
  c5_c_k = c5_b_k;
  c5_alpha1 = 1.0;
  c5_c_ldb = c5_b_ldb;
  c5_beta1 = 0.0;
  c5_TRANSB = 'N';
  c5_TRANSA = 'N';
  c5_m_t = (ptrdiff_t)(6);
  c5_n_t = (ptrdiff_t)(23);
  c5_var = c5_c_k;
  c5_k_t = (ptrdiff_t)(c5_var);
  c5_lda_t = (ptrdiff_t)(6);
  c5_b_var = c5_c_ldb;
  c5_ldb_t = (ptrdiff_t)(c5_b_var);
  c5_ldc_t = (ptrdiff_t)(6);
  c5_alpha1_t = (double *)(&c5_alpha1);
  c5_Aia0_t = (double *)(&c5_A_data[0]);
  c5_Bib0_t = (double *)(&c5_B_data[0]);
  c5_beta1_t = (double *)(&c5_beta1);
  c5_Cic0_t = (double *)(&c5_C[0]);
  dgemm(&c5_TRANSA, &c5_TRANSB, &c5_m_t, &c5_n_t, &c5_k_t, c5_alpha1_t,
        c5_Aia0_t, &c5_lda_t, c5_Bib0_t, &c5_ldb_t, c5_beta1_t, c5_Cic0_t,
        &c5_ldc_t);
}

static void init_dsm_address_info
  (SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c5_balancingController_with_OOT_and_transfer_get_check_sum(mxArray *
  plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3346018638U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3851148129U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1939898394U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(862633619U);
}

mxArray
  *sf_c5_balancingController_with_OOT_and_transfer_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("lJ4v2pbwBS6ocEhT9kCQbC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(29);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(29);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(23);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c5_balancingController_with_OOT_and_transfer_third_party_uses_info
  (void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray
  *sf_c5_balancingController_with_OOT_and_transfer_updateBuildInfo_args_info
  (void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray
  *sf_get_sim_state_info_c5_balancingController_with_OOT_and_transfer(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"y\",},{M[8],M[0],T\"is_active_c5_balancingController_with_OOT_and_transfer\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c5_balancingController_with_OOT_and_transfer_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance =
      (SFc5_balancingController_with_OOT_and_transferInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _balancingController_with_OOT_and_transferMachineNumber_,
           5,
           1,
           1,
           0,
           5,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation
          (_balancingController_with_OOT_and_transferMachineNumber_,
           chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,
             _balancingController_with_OOT_and_transferMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _balancingController_with_OOT_and_transferMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"JcLeftFoot");
          _SFD_SET_DATA_PROPS(1,1,1,0,"JcRightFoot");
          _SFD_SET_DATA_PROPS(2,1,1,0,"nunmberOfFeetOnGround");
          _SFD_SET_DATA_PROPS(3,2,0,1,"y");
          _SFD_SET_DATA_PROPS(4,1,1,0,"qD");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,208);
        _SFD_CV_INIT_EML_IF(0,1,0,76,105,141,170);

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 29;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 29;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_c_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)
            c5_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T *c5_nunmberOfFeetOnGround;
          real_T (*c5_JcLeftFoot)[174];
          real_T (*c5_JcRightFoot)[174];
          real_T (*c5_y)[6];
          real_T (*c5_qD)[23];
          c5_qD = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
          c5_y = (real_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
          c5_nunmberOfFeetOnGround = (real_T *)ssGetInputPortSignal
            (chartInstance->S, 2);
          c5_JcRightFoot = (real_T (*)[174])ssGetInputPortSignal
            (chartInstance->S, 1);
          c5_JcLeftFoot = (real_T (*)[174])ssGetInputPortSignal(chartInstance->S,
            0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c5_JcLeftFoot);
          _SFD_SET_DATA_VALUE_PTR(1U, *c5_JcRightFoot);
          _SFD_SET_DATA_VALUE_PTR(2U, c5_nunmberOfFeetOnGround);
          _SFD_SET_DATA_VALUE_PTR(3U, *c5_y);
          _SFD_SET_DATA_VALUE_PTR(4U, *c5_qD);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _balancingController_with_OOT_and_transferMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "Q6yNILM8fWRC6PLok6rzZG";
}

static void sf_opaque_initialize_c5_balancingController_with_OOT_and_transfer
  (void *chartInstanceVar)
{
  chart_debug_initialization
    (((SFc5_balancingController_with_OOT_and_transferInstanceStruct*)
      chartInstanceVar)->S,0);
  initialize_params_c5_balancingController_with_OOT_and_transfer
    ((SFc5_balancingController_with_OOT_and_transferInstanceStruct*)
     chartInstanceVar);
  initialize_c5_balancingController_with_OOT_and_transfer
    ((SFc5_balancingController_with_OOT_and_transferInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_enable_c5_balancingController_with_OOT_and_transfer(void
  *chartInstanceVar)
{
  enable_c5_balancingController_with_OOT_and_transfer
    ((SFc5_balancingController_with_OOT_and_transferInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_disable_c5_balancingController_with_OOT_and_transfer(void *
  chartInstanceVar)
{
  disable_c5_balancingController_with_OOT_and_transfer
    ((SFc5_balancingController_with_OOT_and_transferInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_gateway_c5_balancingController_with_OOT_and_transfer(void *
  chartInstanceVar)
{
  sf_gateway_c5_balancingController_with_OOT_and_transfer
    ((SFc5_balancingController_with_OOT_and_transferInstanceStruct*)
     chartInstanceVar);
}

extern const mxArray*
  sf_internal_get_sim_state_c5_balancingController_with_OOT_and_transfer
  (SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*)
    get_sim_state_c5_balancingController_with_OOT_and_transfer
    ((SFc5_balancingController_with_OOT_and_transferInstanceStruct*)
     chartInfo->chartInstance);        /* raw sim ctx */
  prhs[3] = (mxArray*)
    sf_get_sim_state_info_c5_balancingController_with_OOT_and_transfer();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void
  sf_internal_set_sim_state_c5_balancingController_with_OOT_and_transfer
  (SimStruct* S, const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*)
    sf_get_sim_state_info_c5_balancingController_with_OOT_and_transfer();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c5_balancingController_with_OOT_and_transfer
    ((SFc5_balancingController_with_OOT_and_transferInstanceStruct*)
     chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray*
  sf_opaque_get_sim_state_c5_balancingController_with_OOT_and_transfer(SimStruct*
  S)
{
  return sf_internal_get_sim_state_c5_balancingController_with_OOT_and_transfer
    (S);
}

static void sf_opaque_set_sim_state_c5_balancingController_with_OOT_and_transfer
  (SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c5_balancingController_with_OOT_and_transfer(S, st);
}

static void sf_opaque_terminate_c5_balancingController_with_OOT_and_transfer
  (void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S =
      ((SFc5_balancingController_with_OOT_and_transferInstanceStruct*)
       chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_balancingController_with_OOT_and_transfer_optimization_info();
    }

    finalize_c5_balancingController_with_OOT_and_transfer
      ((SFc5_balancingController_with_OOT_and_transferInstanceStruct*)
       chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc5_balancingController_with_OOT_and_transfer
    ((SFc5_balancingController_with_OOT_and_transferInstanceStruct*)
     chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c5_balancingController_with_OOT_and_transfer
  (SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c5_balancingController_with_OOT_and_transfer
      ((SFc5_balancingController_with_OOT_and_transferInstanceStruct*)
       (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c5_balancingController_with_OOT_and_transfer
  (SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct =
      load_balancingController_with_OOT_and_transfer_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,5);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,5,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,5,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,5);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,5,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,5,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,5);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1429319808U));
  ssSetChecksum1(S,(2855280662U));
  ssSetChecksum2(S,(2063499724U));
  ssSetChecksum3(S,(3333234843U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c5_balancingController_with_OOT_and_transfer(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c5_balancingController_with_OOT_and_transfer(SimStruct *S)
{
  SFc5_balancingController_with_OOT_and_transferInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc5_balancingController_with_OOT_and_transferInstanceStruct *)
    utMalloc(sizeof(SFc5_balancingController_with_OOT_and_transferInstanceStruct));
  memset(chartInstance, 0, sizeof
         (SFc5_balancingController_with_OOT_and_transferInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c5_balancingController_with_OOT_and_transfer;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c5_balancingController_with_OOT_and_transfer;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c5_balancingController_with_OOT_and_transfer;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c5_balancingController_with_OOT_and_transfer;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c5_balancingController_with_OOT_and_transfer;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c5_balancingController_with_OOT_and_transfer;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c5_balancingController_with_OOT_and_transfer;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c5_balancingController_with_OOT_and_transfer;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW =
    mdlRTW_c5_balancingController_with_OOT_and_transfer;
  chartInstance->chartInfo.mdlStart =
    mdlStart_c5_balancingController_with_OOT_and_transfer;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c5_balancingController_with_OOT_and_transfer;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c5_balancingController_with_OOT_and_transfer_method_dispatcher(SimStruct *S,
  int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c5_balancingController_with_OOT_and_transfer(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_balancingController_with_OOT_and_transfer(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_balancingController_with_OOT_and_transfer(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_balancingController_with_OOT_and_transfer_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
