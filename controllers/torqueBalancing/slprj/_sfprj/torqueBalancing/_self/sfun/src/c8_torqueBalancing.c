/* Include files */

#include <stddef.h>
#include "blas.h"
#include "torqueBalancing_sfun.h"
#include "c8_torqueBalancing.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "torqueBalancing_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c8_debug_family_names[14] = { "impedences_at_0",
  "increasingRatesImp", "qTildeMax", "tp", "tn", "qTilde", "nargin", "nargout",
  "qDes", "q", "qMin", "qMax", "gain", "nonlinearImpedences" };

/* Function Declarations */
static void initialize_c8_torqueBalancing(SFc8_torqueBalancingInstanceStruct
  *chartInstance);
static void initialize_params_c8_torqueBalancing
  (SFc8_torqueBalancingInstanceStruct *chartInstance);
static void enable_c8_torqueBalancing(SFc8_torqueBalancingInstanceStruct
  *chartInstance);
static void disable_c8_torqueBalancing(SFc8_torqueBalancingInstanceStruct
  *chartInstance);
static void c8_update_debugger_state_c8_torqueBalancing
  (SFc8_torqueBalancingInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c8_torqueBalancing
  (SFc8_torqueBalancingInstanceStruct *chartInstance);
static void set_sim_state_c8_torqueBalancing(SFc8_torqueBalancingInstanceStruct *
  chartInstance, const mxArray *c8_st);
static void finalize_c8_torqueBalancing(SFc8_torqueBalancingInstanceStruct
  *chartInstance);
static void sf_gateway_c8_torqueBalancing(SFc8_torqueBalancingInstanceStruct
  *chartInstance);
static void c8_chartstep_c8_torqueBalancing(SFc8_torqueBalancingInstanceStruct
  *chartInstance);
static void initSimStructsc8_torqueBalancing(SFc8_torqueBalancingInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c8_machineNumber, uint32_T
  c8_chartNumber, uint32_T c8_instanceNumber);
static const mxArray *c8_sf_marshallOut(void *chartInstanceVoid, void *c8_inData);
static void c8_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_nonlinearImpedences, const char_T
  *c8_identifier, real_T c8_y[23]);
static void c8_b_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[23]);
static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_c_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  c8_struct_IHUdSPrvFcumP3hkkkz03F *c8_y);
static real_T c8_d_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_e_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[9]);
static void c8_f_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[4]);
static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_c_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static const mxArray *c8_d_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static void c8_g_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[23]);
static void c8_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static void c8_info_helper(const mxArray **c8_info);
static const mxArray *c8_emlrt_marshallOut(const char * c8_u);
static const mxArray *c8_b_emlrt_marshallOut(const uint32_T c8_u);
static const mxArray *c8_e_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static int32_T c8_h_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static uint8_T c8_i_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_b_is_active_c8_torqueBalancing, const char_T
  *c8_identifier);
static uint8_T c8_j_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void init_dsm_address_info(SFc8_torqueBalancingInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c8_torqueBalancing(SFc8_torqueBalancingInstanceStruct
  *chartInstance)
{
  chartInstance->c8_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c8_is_active_c8_torqueBalancing = 0U;
}

static void initialize_params_c8_torqueBalancing
  (SFc8_torqueBalancingInstanceStruct *chartInstance)
{
  const mxArray *c8_m0 = NULL;
  const mxArray *c8_mxField;
  c8_struct_IHUdSPrvFcumP3hkkkz03F c8_r0;
  c8_m0 = sf_mex_get_sfun_param(chartInstance->S, 0, 1);
  c8_mxField = sf_mex_getfield(c8_m0, "qTildeMax", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c8_mxField), &c8_r0.qTildeMax, 1, 0, 0U,
                      0, 0U, 0);
  c8_mxField = sf_mex_getfield(c8_m0, "PCOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c8_mxField), c8_r0.PCOM, 1, 0, 0U, 1,
                      0U, 2, 3, 3);
  c8_mxField = sf_mex_getfield(c8_m0, "ICOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c8_mxField), c8_r0.ICOM, 1, 0, 0U, 1,
                      0U, 2, 3, 3);
  c8_mxField = sf_mex_getfield(c8_m0, "DCOM", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c8_mxField), c8_r0.DCOM, 1, 0, 0U, 1,
                      0U, 2, 3, 3);
  c8_mxField = sf_mex_getfield(c8_m0, "PAngularMomentum", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c8_mxField), &c8_r0.PAngularMomentum, 1,
                      0, 0U, 0, 0U, 0);
  c8_mxField = sf_mex_getfield(c8_m0, "integral", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c8_mxField), c8_r0.integral, 1, 0, 0U,
                      1, 0U, 2, 1, 23);
  c8_mxField = sf_mex_getfield(c8_m0, "impedances", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c8_mxField), c8_r0.impedances, 1, 0, 0U,
                      1, 0U, 2, 1, 23);
  c8_mxField = sf_mex_getfield(c8_m0, "dampings", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c8_mxField), c8_r0.dampings, 1, 0, 0U,
                      1, 0U, 2, 1, 23);
  c8_mxField = sf_mex_getfield(c8_m0, "increasingRatesImp", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c8_mxField), c8_r0.increasingRatesImp,
                      1, 0, 0U, 1, 0U, 2, 1, 23);
  c8_mxField = sf_mex_getfield(c8_m0, "footSize", "gain", 0);
  sf_mex_import_named("gain", sf_mex_dup(c8_mxField), c8_r0.footSize, 1, 0, 0U,
                      1, 0U, 2, 2, 2);
  sf_mex_destroy(&c8_m0);
  chartInstance->c8_gain = c8_r0;
}

static void enable_c8_torqueBalancing(SFc8_torqueBalancingInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c8_torqueBalancing(SFc8_torqueBalancingInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c8_update_debugger_state_c8_torqueBalancing
  (SFc8_torqueBalancingInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c8_torqueBalancing
  (SFc8_torqueBalancingInstanceStruct *chartInstance)
{
  const mxArray *c8_st;
  const mxArray *c8_y = NULL;
  int32_T c8_i0;
  real_T c8_u[23];
  const mxArray *c8_b_y = NULL;
  uint8_T c8_hoistedGlobal;
  uint8_T c8_b_u;
  const mxArray *c8_c_y = NULL;
  real_T (*c8_nonlinearImpedences)[23];
  c8_nonlinearImpedences = (real_T (*)[23])ssGetOutputPortSignal
    (chartInstance->S, 1);
  c8_st = NULL;
  c8_st = NULL;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_createcellmatrix(2, 1), false);
  for (c8_i0 = 0; c8_i0 < 23; c8_i0++) {
    c8_u[c8_i0] = (*c8_nonlinearImpedences)[c8_i0];
  }

  c8_b_y = NULL;
  sf_mex_assign(&c8_b_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 2, 1, 23),
                false);
  sf_mex_setcell(c8_y, 0, c8_b_y);
  c8_hoistedGlobal = chartInstance->c8_is_active_c8_torqueBalancing;
  c8_b_u = c8_hoistedGlobal;
  c8_c_y = NULL;
  sf_mex_assign(&c8_c_y, sf_mex_create("y", &c8_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c8_y, 1, c8_c_y);
  sf_mex_assign(&c8_st, c8_y, false);
  return c8_st;
}

static void set_sim_state_c8_torqueBalancing(SFc8_torqueBalancingInstanceStruct *
  chartInstance, const mxArray *c8_st)
{
  const mxArray *c8_u;
  real_T c8_dv0[23];
  int32_T c8_i1;
  real_T (*c8_nonlinearImpedences)[23];
  c8_nonlinearImpedences = (real_T (*)[23])ssGetOutputPortSignal
    (chartInstance->S, 1);
  chartInstance->c8_doneDoubleBufferReInit = true;
  c8_u = sf_mex_dup(c8_st);
  c8_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 0)),
                      "nonlinearImpedences", c8_dv0);
  for (c8_i1 = 0; c8_i1 < 23; c8_i1++) {
    (*c8_nonlinearImpedences)[c8_i1] = c8_dv0[c8_i1];
  }

  chartInstance->c8_is_active_c8_torqueBalancing = c8_i_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 1)),
     "is_active_c8_torqueBalancing");
  sf_mex_destroy(&c8_u);
  c8_update_debugger_state_c8_torqueBalancing(chartInstance);
  sf_mex_destroy(&c8_st);
}

static void finalize_c8_torqueBalancing(SFc8_torqueBalancingInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c8_torqueBalancing(SFc8_torqueBalancingInstanceStruct
  *chartInstance)
{
  int32_T c8_i2;
  int32_T c8_i3;
  int32_T c8_i4;
  int32_T c8_i5;
  int32_T c8_i6;
  real_T (*c8_qMax)[23];
  real_T (*c8_qMin)[23];
  real_T (*c8_q)[23];
  real_T (*c8_nonlinearImpedences)[23];
  real_T (*c8_qDes)[23];
  c8_qMax = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
  c8_qMin = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 2);
  c8_q = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 1);
  c8_nonlinearImpedences = (real_T (*)[23])ssGetOutputPortSignal
    (chartInstance->S, 1);
  c8_qDes = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c8_sfEvent);
  for (c8_i2 = 0; c8_i2 < 23; c8_i2++) {
    _SFD_DATA_RANGE_CHECK((*c8_qDes)[c8_i2], 0U);
  }

  chartInstance->c8_sfEvent = CALL_EVENT;
  c8_chartstep_c8_torqueBalancing(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_torqueBalancingMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c8_i3 = 0; c8_i3 < 23; c8_i3++) {
    _SFD_DATA_RANGE_CHECK((*c8_nonlinearImpedences)[c8_i3], 1U);
  }

  for (c8_i4 = 0; c8_i4 < 23; c8_i4++) {
    _SFD_DATA_RANGE_CHECK((*c8_q)[c8_i4], 2U);
  }

  for (c8_i5 = 0; c8_i5 < 23; c8_i5++) {
    _SFD_DATA_RANGE_CHECK((*c8_qMin)[c8_i5], 3U);
  }

  for (c8_i6 = 0; c8_i6 < 23; c8_i6++) {
    _SFD_DATA_RANGE_CHECK((*c8_qMax)[c8_i6], 4U);
  }
}

static void c8_chartstep_c8_torqueBalancing(SFc8_torqueBalancingInstanceStruct
  *chartInstance)
{
  int32_T c8_i7;
  real_T c8_qDes[23];
  int32_T c8_i8;
  real_T c8_q[23];
  int32_T c8_i9;
  real_T c8_qMin[23];
  int32_T c8_i10;
  real_T c8_qMax[23];
  c8_struct_IHUdSPrvFcumP3hkkkz03F c8_b_gain;
  uint32_T c8_debug_family_var_map[14];
  real_T c8_impedences_at_0[23];
  real_T c8_increasingRatesImp[23];
  real_T c8_qTildeMax;
  real_T c8_tp[23];
  real_T c8_tn[23];
  real_T c8_qTilde[23];
  real_T c8_nargin = 5.0;
  real_T c8_nargout = 1.0;
  real_T c8_nonlinearImpedences[23];
  int32_T c8_i11;
  int32_T c8_i12;
  int32_T c8_i13;
  real_T c8_b[23];
  int32_T c8_i14;
  int32_T c8_i15;
  int32_T c8_i16;
  int32_T c8_i17;
  int32_T c8_i18;
  int32_T c8_i19;
  int32_T c8_i20;
  real_T c8_y;
  real_T c8_b_y;
  real_T c8_c_y;
  int32_T c8_i21;
  int32_T c8_k;
  real_T c8_b_k;
  real_T c8_x;
  real_T c8_b_x;
  int32_T c8_c_k;
  real_T c8_d_k;
  real_T c8_ak;
  real_T c8_a;
  real_T c8_d_y;
  real_T c8_e_y[23];
  int32_T c8_i22;
  int32_T c8_i23;
  real_T (*c8_b_nonlinearImpedences)[23];
  real_T (*c8_b_qMax)[23];
  real_T (*c8_b_qMin)[23];
  real_T (*c8_b_q)[23];
  real_T (*c8_b_qDes)[23];
  c8_b_qMax = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
  c8_b_qMin = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 2);
  c8_b_q = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 1);
  c8_b_nonlinearImpedences = (real_T (*)[23])ssGetOutputPortSignal
    (chartInstance->S, 1);
  c8_b_qDes = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c8_sfEvent);
  for (c8_i7 = 0; c8_i7 < 23; c8_i7++) {
    c8_qDes[c8_i7] = (*c8_b_qDes)[c8_i7];
  }

  for (c8_i8 = 0; c8_i8 < 23; c8_i8++) {
    c8_q[c8_i8] = (*c8_b_q)[c8_i8];
  }

  for (c8_i9 = 0; c8_i9 < 23; c8_i9++) {
    c8_qMin[c8_i9] = (*c8_b_qMin)[c8_i9];
  }

  for (c8_i10 = 0; c8_i10 < 23; c8_i10++) {
    c8_qMax[c8_i10] = (*c8_b_qMax)[c8_i10];
  }

  c8_b_gain = chartInstance->c8_gain;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 14U, 14U, c8_debug_family_names,
    c8_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c8_impedences_at_0, 0U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c8_increasingRatesImp, 1U,
    c8_c_sf_marshallOut, c8_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_qTildeMax, 2U, c8_d_sf_marshallOut,
    c8_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c8_tp, 3U, c8_c_sf_marshallOut,
    c8_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c8_tn, 4U, c8_c_sf_marshallOut,
    c8_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c8_qTilde, 5U, c8_c_sf_marshallOut,
    c8_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_nargin, 6U, c8_d_sf_marshallOut,
    c8_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_nargout, 7U, c8_d_sf_marshallOut,
    c8_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c8_qDes, 8U, c8_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c8_q, 9U, c8_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c8_qMin, 10U, c8_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c8_qMax, 11U, c8_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_b_gain, 12U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c8_nonlinearImpedences, 13U,
    c8_sf_marshallOut, c8_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 3);
  for (c8_i11 = 0; c8_i11 < 23; c8_i11++) {
    c8_impedences_at_0[c8_i11] = c8_b_gain.impedances[c8_i11];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 4);
  for (c8_i12 = 0; c8_i12 < 23; c8_i12++) {
    c8_increasingRatesImp[c8_i12] = c8_b_gain.increasingRatesImp[c8_i12];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 5);
  c8_qTildeMax = c8_b_gain.qTildeMax;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 7);
  for (c8_i13 = 0; c8_i13 < 23; c8_i13++) {
    c8_b[c8_i13] = c8_qMax[c8_i13] - c8_qDes[c8_i13];
  }

  for (c8_i14 = 0; c8_i14 < 23; c8_i14++) {
    c8_b[c8_i14] *= 2.0;
  }

  for (c8_i15 = 0; c8_i15 < 23; c8_i15++) {
    c8_tp[c8_i15] = 3.1415926535897931 / c8_b[c8_i15];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 8);
  for (c8_i16 = 0; c8_i16 < 23; c8_i16++) {
    c8_b[c8_i16] = c8_qMin[c8_i16] - c8_qDes[c8_i16];
  }

  for (c8_i17 = 0; c8_i17 < 23; c8_i17++) {
    c8_b[c8_i17] *= 2.0;
  }

  for (c8_i18 = 0; c8_i18 < 23; c8_i18++) {
    c8_tn[c8_i18] = -3.1415926535897931 / c8_b[c8_i18];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 11);
  for (c8_i19 = 0; c8_i19 < 23; c8_i19++) {
    c8_qTilde[c8_i19] = c8_q[c8_i19] - c8_qDes[c8_i19];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 13);
  for (c8_i20 = 0; c8_i20 < 23; c8_i20++) {
    c8_b[c8_i20] = 3.1415926535897931 * c8_qTilde[c8_i20];
  }

  c8_y = 2.0 * c8_qTildeMax;
  c8_b_y = c8_y;
  c8_c_y = c8_b_y;
  for (c8_i21 = 0; c8_i21 < 23; c8_i21++) {
    c8_b[c8_i21] /= c8_c_y;
  }

  for (c8_k = 0; c8_k < 23; c8_k++) {
    c8_b_k = 1.0 + (real_T)c8_k;
    c8_x = c8_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c8_b_k), 1, 23, 1, 0) - 1];
    c8_b_x = c8_x;
    c8_b_x = muDoubleScalarTan(c8_b_x);
    c8_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("", c8_b_k),
      1, 23, 1, 0) - 1] = c8_b_x;
  }

  for (c8_c_k = 0; c8_c_k < 23; c8_c_k++) {
    c8_d_k = 1.0 + (real_T)c8_c_k;
    c8_ak = c8_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c8_d_k), 1, 23, 1, 0) - 1];
    c8_a = c8_ak;
    c8_d_y = c8_a * c8_a;
    c8_e_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c8_d_k), 1, 23, 1, 0) - 1] = c8_d_y;
  }

  for (c8_i22 = 0; c8_i22 < 23; c8_i22++) {
    c8_nonlinearImpedences[c8_i22] = c8_impedences_at_0[c8_i22] +
      c8_increasingRatesImp[c8_i22] * c8_e_y[c8_i22];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, -13);
  _SFD_SYMBOL_SCOPE_POP();
  for (c8_i23 = 0; c8_i23 < 23; c8_i23++) {
    (*c8_b_nonlinearImpedences)[c8_i23] = c8_nonlinearImpedences[c8_i23];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c8_sfEvent);
}

static void initSimStructsc8_torqueBalancing(SFc8_torqueBalancingInstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c8_machineNumber, uint32_T
  c8_chartNumber, uint32_T c8_instanceNumber)
{
  (void)c8_machineNumber;
  (void)c8_chartNumber;
  (void)c8_instanceNumber;
}

static const mxArray *c8_sf_marshallOut(void *chartInstanceVoid, void *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i24;
  real_T c8_b_inData[23];
  int32_T c8_i25;
  real_T c8_u[23];
  const mxArray *c8_y = NULL;
  SFc8_torqueBalancingInstanceStruct *chartInstance;
  chartInstance = (SFc8_torqueBalancingInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i24 = 0; c8_i24 < 23; c8_i24++) {
    c8_b_inData[c8_i24] = (*(real_T (*)[23])c8_inData)[c8_i24];
  }

  for (c8_i25 = 0; c8_i25 < 23; c8_i25++) {
    c8_u[c8_i25] = c8_b_inData[c8_i25];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 2, 1, 23), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static void c8_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_nonlinearImpedences, const char_T
  *c8_identifier, real_T c8_y[23])
{
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_nonlinearImpedences),
                        &c8_thisId, c8_y);
  sf_mex_destroy(&c8_nonlinearImpedences);
}

static void c8_b_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[23])
{
  real_T c8_dv1[23];
  int32_T c8_i26;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv1, 1, 0, 0U, 1, 0U, 2, 1, 23);
  for (c8_i26 = 0; c8_i26 < 23; c8_i26++) {
    c8_y[c8_i26] = c8_dv1[c8_i26];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_nonlinearImpedences;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[23];
  int32_T c8_i27;
  SFc8_torqueBalancingInstanceStruct *chartInstance;
  chartInstance = (SFc8_torqueBalancingInstanceStruct *)chartInstanceVoid;
  c8_nonlinearImpedences = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_nonlinearImpedences),
                        &c8_thisId, c8_y);
  sf_mex_destroy(&c8_nonlinearImpedences);
  for (c8_i27 = 0; c8_i27 < 23; c8_i27++) {
    (*(real_T (*)[23])c8_outData)[c8_i27] = c8_y[c8_i27];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  c8_struct_IHUdSPrvFcumP3hkkkz03F c8_u;
  const mxArray *c8_y = NULL;
  real_T c8_b_u;
  const mxArray *c8_b_y = NULL;
  int32_T c8_i28;
  real_T c8_c_u[9];
  const mxArray *c8_c_y = NULL;
  int32_T c8_i29;
  real_T c8_d_u[9];
  const mxArray *c8_d_y = NULL;
  int32_T c8_i30;
  real_T c8_e_u[9];
  const mxArray *c8_e_y = NULL;
  real_T c8_f_u;
  const mxArray *c8_f_y = NULL;
  int32_T c8_i31;
  real_T c8_g_u[23];
  const mxArray *c8_g_y = NULL;
  int32_T c8_i32;
  real_T c8_h_u[23];
  const mxArray *c8_h_y = NULL;
  int32_T c8_i33;
  real_T c8_i_u[23];
  const mxArray *c8_i_y = NULL;
  int32_T c8_i34;
  real_T c8_j_u[23];
  const mxArray *c8_j_y = NULL;
  int32_T c8_i35;
  real_T c8_k_u[4];
  const mxArray *c8_k_y = NULL;
  SFc8_torqueBalancingInstanceStruct *chartInstance;
  chartInstance = (SFc8_torqueBalancingInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(c8_struct_IHUdSPrvFcumP3hkkkz03F *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_createstruct("structure", 2, 1, 1), false);
  c8_b_u = c8_u.qTildeMax;
  c8_b_y = NULL;
  sf_mex_assign(&c8_b_y, sf_mex_create("y", &c8_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c8_y, c8_b_y, "qTildeMax", "qTildeMax", 0);
  for (c8_i28 = 0; c8_i28 < 9; c8_i28++) {
    c8_c_u[c8_i28] = c8_u.PCOM[c8_i28];
  }

  c8_c_y = NULL;
  sf_mex_assign(&c8_c_y, sf_mex_create("y", c8_c_u, 0, 0U, 1U, 0U, 2, 3, 3),
                false);
  sf_mex_addfield(c8_y, c8_c_y, "PCOM", "PCOM", 0);
  for (c8_i29 = 0; c8_i29 < 9; c8_i29++) {
    c8_d_u[c8_i29] = c8_u.ICOM[c8_i29];
  }

  c8_d_y = NULL;
  sf_mex_assign(&c8_d_y, sf_mex_create("y", c8_d_u, 0, 0U, 1U, 0U, 2, 3, 3),
                false);
  sf_mex_addfield(c8_y, c8_d_y, "ICOM", "ICOM", 0);
  for (c8_i30 = 0; c8_i30 < 9; c8_i30++) {
    c8_e_u[c8_i30] = c8_u.DCOM[c8_i30];
  }

  c8_e_y = NULL;
  sf_mex_assign(&c8_e_y, sf_mex_create("y", c8_e_u, 0, 0U, 1U, 0U, 2, 3, 3),
                false);
  sf_mex_addfield(c8_y, c8_e_y, "DCOM", "DCOM", 0);
  c8_f_u = c8_u.PAngularMomentum;
  c8_f_y = NULL;
  sf_mex_assign(&c8_f_y, sf_mex_create("y", &c8_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_addfield(c8_y, c8_f_y, "PAngularMomentum", "PAngularMomentum", 0);
  for (c8_i31 = 0; c8_i31 < 23; c8_i31++) {
    c8_g_u[c8_i31] = c8_u.integral[c8_i31];
  }

  c8_g_y = NULL;
  sf_mex_assign(&c8_g_y, sf_mex_create("y", c8_g_u, 0, 0U, 1U, 0U, 2, 1, 23),
                false);
  sf_mex_addfield(c8_y, c8_g_y, "integral", "integral", 0);
  for (c8_i32 = 0; c8_i32 < 23; c8_i32++) {
    c8_h_u[c8_i32] = c8_u.impedances[c8_i32];
  }

  c8_h_y = NULL;
  sf_mex_assign(&c8_h_y, sf_mex_create("y", c8_h_u, 0, 0U, 1U, 0U, 2, 1, 23),
                false);
  sf_mex_addfield(c8_y, c8_h_y, "impedances", "impedances", 0);
  for (c8_i33 = 0; c8_i33 < 23; c8_i33++) {
    c8_i_u[c8_i33] = c8_u.dampings[c8_i33];
  }

  c8_i_y = NULL;
  sf_mex_assign(&c8_i_y, sf_mex_create("y", c8_i_u, 0, 0U, 1U, 0U, 2, 1, 23),
                false);
  sf_mex_addfield(c8_y, c8_i_y, "dampings", "dampings", 0);
  for (c8_i34 = 0; c8_i34 < 23; c8_i34++) {
    c8_j_u[c8_i34] = c8_u.increasingRatesImp[c8_i34];
  }

  c8_j_y = NULL;
  sf_mex_assign(&c8_j_y, sf_mex_create("y", c8_j_u, 0, 0U, 1U, 0U, 2, 1, 23),
                false);
  sf_mex_addfield(c8_y, c8_j_y, "increasingRatesImp", "increasingRatesImp", 0);
  for (c8_i35 = 0; c8_i35 < 4; c8_i35++) {
    c8_k_u[c8_i35] = c8_u.footSize[c8_i35];
  }

  c8_k_y = NULL;
  sf_mex_assign(&c8_k_y, sf_mex_create("y", c8_k_u, 0, 0U, 1U, 0U, 2, 2, 2),
                false);
  sf_mex_addfield(c8_y, c8_k_y, "footSize", "footSize", 0);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static void c8_c_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  c8_struct_IHUdSPrvFcumP3hkkkz03F *c8_y)
{
  emlrtMsgIdentifier c8_thisId;
  static const char * c8_fieldNames[10] = { "qTildeMax", "PCOM", "ICOM", "DCOM",
    "PAngularMomentum", "integral", "impedances", "dampings",
    "increasingRatesImp", "footSize" };

  c8_thisId.fParent = c8_parentId;
  sf_mex_check_struct(c8_parentId, c8_u, 10, c8_fieldNames, 0U, NULL);
  c8_thisId.fIdentifier = "qTildeMax";
  c8_y->qTildeMax = c8_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c8_u, "qTildeMax", "qTildeMax", 0)), &c8_thisId);
  c8_thisId.fIdentifier = "PCOM";
  c8_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c8_u, "PCOM",
    "PCOM", 0)), &c8_thisId, c8_y->PCOM);
  c8_thisId.fIdentifier = "ICOM";
  c8_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c8_u, "ICOM",
    "ICOM", 0)), &c8_thisId, c8_y->ICOM);
  c8_thisId.fIdentifier = "DCOM";
  c8_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c8_u, "DCOM",
    "DCOM", 0)), &c8_thisId, c8_y->DCOM);
  c8_thisId.fIdentifier = "PAngularMomentum";
  c8_y->PAngularMomentum = c8_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getfield(c8_u, "PAngularMomentum", "PAngularMomentum", 0)),
    &c8_thisId);
  c8_thisId.fIdentifier = "integral";
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c8_u,
    "integral", "integral", 0)), &c8_thisId, c8_y->integral);
  c8_thisId.fIdentifier = "impedances";
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c8_u,
    "impedances", "impedances", 0)), &c8_thisId, c8_y->impedances);
  c8_thisId.fIdentifier = "dampings";
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c8_u,
    "dampings", "dampings", 0)), &c8_thisId, c8_y->dampings);
  c8_thisId.fIdentifier = "increasingRatesImp";
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c8_u,
    "increasingRatesImp", "increasingRatesImp", 0)), &c8_thisId,
                        c8_y->increasingRatesImp);
  c8_thisId.fIdentifier = "footSize";
  c8_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getfield(c8_u,
    "footSize", "footSize", 0)), &c8_thisId, c8_y->footSize);
  sf_mex_destroy(&c8_u);
}

static real_T c8_d_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  real_T c8_y;
  real_T c8_d0;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_d0, 1, 0, 0U, 0, 0U, 0);
  c8_y = c8_d0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_e_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[9])
{
  real_T c8_dv2[9];
  int32_T c8_i36;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv2, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c8_i36 = 0; c8_i36 < 9; c8_i36++) {
    c8_y[c8_i36] = c8_dv2[c8_i36];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_f_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[4])
{
  real_T c8_dv3[4];
  int32_T c8_i37;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv3, 1, 0, 0U, 1, 0U, 2, 2, 2);
  for (c8_i37 = 0; c8_i37 < 4; c8_i37++) {
    c8_y[c8_i37] = c8_dv3[c8_i37];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_b_gain;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  c8_struct_IHUdSPrvFcumP3hkkkz03F c8_y;
  SFc8_torqueBalancingInstanceStruct *chartInstance;
  chartInstance = (SFc8_torqueBalancingInstanceStruct *)chartInstanceVoid;
  c8_b_gain = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_gain), &c8_thisId, &c8_y);
  sf_mex_destroy(&c8_b_gain);
  *(c8_struct_IHUdSPrvFcumP3hkkkz03F *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_c_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i38;
  real_T c8_b_inData[23];
  int32_T c8_i39;
  real_T c8_u[23];
  const mxArray *c8_y = NULL;
  SFc8_torqueBalancingInstanceStruct *chartInstance;
  chartInstance = (SFc8_torqueBalancingInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i38 = 0; c8_i38 < 23; c8_i38++) {
    c8_b_inData[c8_i38] = (*(real_T (*)[23])c8_inData)[c8_i38];
  }

  for (c8_i39 = 0; c8_i39 < 23; c8_i39++) {
    c8_u[c8_i39] = c8_b_inData[c8_i39];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 23), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static const mxArray *c8_d_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  real_T c8_u;
  const mxArray *c8_y = NULL;
  SFc8_torqueBalancingInstanceStruct *chartInstance;
  chartInstance = (SFc8_torqueBalancingInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(real_T *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static void c8_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_nargout;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y;
  SFc8_torqueBalancingInstanceStruct *chartInstance;
  chartInstance = (SFc8_torqueBalancingInstanceStruct *)chartInstanceVoid;
  c8_nargout = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_nargout), &c8_thisId);
  sf_mex_destroy(&c8_nargout);
  *(real_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

static void c8_g_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[23])
{
  real_T c8_dv4[23];
  int32_T c8_i40;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv4, 1, 0, 0U, 1, 0U, 1, 23);
  for (c8_i40 = 0; c8_i40 < 23; c8_i40++) {
    c8_y[c8_i40] = c8_dv4[c8_i40];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_qTilde;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[23];
  int32_T c8_i41;
  SFc8_torqueBalancingInstanceStruct *chartInstance;
  chartInstance = (SFc8_torqueBalancingInstanceStruct *)chartInstanceVoid;
  c8_qTilde = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_qTilde), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_qTilde);
  for (c8_i41 = 0; c8_i41 < 23; c8_i41++) {
    (*(real_T (*)[23])c8_outData)[c8_i41] = c8_y[c8_i41];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

const mxArray *sf_c8_torqueBalancing_get_eml_resolved_functions_info(void)
{
  const mxArray *c8_nameCaptureInfo = NULL;
  c8_nameCaptureInfo = NULL;
  sf_mex_assign(&c8_nameCaptureInfo, sf_mex_createstruct("structure", 2, 19, 1),
                false);
  c8_info_helper(&c8_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c8_nameCaptureInfo);
  return c8_nameCaptureInfo;
}

static void c8_info_helper(const mxArray **c8_info)
{
  const mxArray *c8_rhs0 = NULL;
  const mxArray *c8_lhs0 = NULL;
  const mxArray *c8_rhs1 = NULL;
  const mxArray *c8_lhs1 = NULL;
  const mxArray *c8_rhs2 = NULL;
  const mxArray *c8_lhs2 = NULL;
  const mxArray *c8_rhs3 = NULL;
  const mxArray *c8_lhs3 = NULL;
  const mxArray *c8_rhs4 = NULL;
  const mxArray *c8_lhs4 = NULL;
  const mxArray *c8_rhs5 = NULL;
  const mxArray *c8_lhs5 = NULL;
  const mxArray *c8_rhs6 = NULL;
  const mxArray *c8_lhs6 = NULL;
  const mxArray *c8_rhs7 = NULL;
  const mxArray *c8_lhs7 = NULL;
  const mxArray *c8_rhs8 = NULL;
  const mxArray *c8_lhs8 = NULL;
  const mxArray *c8_rhs9 = NULL;
  const mxArray *c8_lhs9 = NULL;
  const mxArray *c8_rhs10 = NULL;
  const mxArray *c8_lhs10 = NULL;
  const mxArray *c8_rhs11 = NULL;
  const mxArray *c8_lhs11 = NULL;
  const mxArray *c8_rhs12 = NULL;
  const mxArray *c8_lhs12 = NULL;
  const mxArray *c8_rhs13 = NULL;
  const mxArray *c8_lhs13 = NULL;
  const mxArray *c8_rhs14 = NULL;
  const mxArray *c8_lhs14 = NULL;
  const mxArray *c8_rhs15 = NULL;
  const mxArray *c8_lhs15 = NULL;
  const mxArray *c8_rhs16 = NULL;
  const mxArray *c8_lhs16 = NULL;
  const mxArray *c8_rhs17 = NULL;
  const mxArray *c8_lhs17 = NULL;
  const mxArray *c8_rhs18 = NULL;
  const mxArray *c8_lhs18 = NULL;
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 0);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 0);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1383877294U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c8_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 1);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 1);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c8_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "context", "context", 2);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("rdivide"), "name", "name", 2);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c8_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 3);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c8_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 4);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c8_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_div"), "name", "name", 5);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c8_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 6);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c8_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "context", "context", 7);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("tan"), "name", "name", 7);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/tan.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1343830386U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c8_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/tan.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalar_tan"), "name",
                  "name", 8);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_tan.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1286818738U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c8_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "context", "context", 9);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("power"), "name", "name", 9);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c8_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 10);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c8_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 11);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 11);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c8_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 12);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c8_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 13);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 13);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c8_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 14);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 14);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c8_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 15);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("floor"), "name", "name", 15);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 15);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363713854U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c8_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 16);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c8_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 17);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1286818726U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c8_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 18);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 18);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 18);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c8_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs18), "lhs", "lhs",
                  18);
  sf_mex_destroy(&c8_rhs0);
  sf_mex_destroy(&c8_lhs0);
  sf_mex_destroy(&c8_rhs1);
  sf_mex_destroy(&c8_lhs1);
  sf_mex_destroy(&c8_rhs2);
  sf_mex_destroy(&c8_lhs2);
  sf_mex_destroy(&c8_rhs3);
  sf_mex_destroy(&c8_lhs3);
  sf_mex_destroy(&c8_rhs4);
  sf_mex_destroy(&c8_lhs4);
  sf_mex_destroy(&c8_rhs5);
  sf_mex_destroy(&c8_lhs5);
  sf_mex_destroy(&c8_rhs6);
  sf_mex_destroy(&c8_lhs6);
  sf_mex_destroy(&c8_rhs7);
  sf_mex_destroy(&c8_lhs7);
  sf_mex_destroy(&c8_rhs8);
  sf_mex_destroy(&c8_lhs8);
  sf_mex_destroy(&c8_rhs9);
  sf_mex_destroy(&c8_lhs9);
  sf_mex_destroy(&c8_rhs10);
  sf_mex_destroy(&c8_lhs10);
  sf_mex_destroy(&c8_rhs11);
  sf_mex_destroy(&c8_lhs11);
  sf_mex_destroy(&c8_rhs12);
  sf_mex_destroy(&c8_lhs12);
  sf_mex_destroy(&c8_rhs13);
  sf_mex_destroy(&c8_lhs13);
  sf_mex_destroy(&c8_rhs14);
  sf_mex_destroy(&c8_lhs14);
  sf_mex_destroy(&c8_rhs15);
  sf_mex_destroy(&c8_lhs15);
  sf_mex_destroy(&c8_rhs16);
  sf_mex_destroy(&c8_lhs16);
  sf_mex_destroy(&c8_rhs17);
  sf_mex_destroy(&c8_lhs17);
  sf_mex_destroy(&c8_rhs18);
  sf_mex_destroy(&c8_lhs18);
}

static const mxArray *c8_emlrt_marshallOut(const char * c8_u)
{
  const mxArray *c8_y = NULL;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c8_u)), false);
  return c8_y;
}

static const mxArray *c8_b_emlrt_marshallOut(const uint32_T c8_u)
{
  const mxArray *c8_y = NULL;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 7, 0U, 0U, 0U, 0), false);
  return c8_y;
}

static const mxArray *c8_e_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_u;
  const mxArray *c8_y = NULL;
  SFc8_torqueBalancingInstanceStruct *chartInstance;
  chartInstance = (SFc8_torqueBalancingInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(int32_T *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static int32_T c8_h_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  int32_T c8_y;
  int32_T c8_i42;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_i42, 1, 6, 0U, 0, 0U, 0);
  c8_y = c8_i42;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_b_sfEvent;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  int32_T c8_y;
  SFc8_torqueBalancingInstanceStruct *chartInstance;
  chartInstance = (SFc8_torqueBalancingInstanceStruct *)chartInstanceVoid;
  c8_b_sfEvent = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_sfEvent),
    &c8_thisId);
  sf_mex_destroy(&c8_b_sfEvent);
  *(int32_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

static uint8_T c8_i_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_b_is_active_c8_torqueBalancing, const char_T
  *c8_identifier)
{
  uint8_T c8_y;
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c8_b_is_active_c8_torqueBalancing), &c8_thisId);
  sf_mex_destroy(&c8_b_is_active_c8_torqueBalancing);
  return c8_y;
}

static uint8_T c8_j_emlrt_marshallIn(SFc8_torqueBalancingInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  uint8_T c8_y;
  uint8_T c8_u0;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_u0, 1, 3, 0U, 0, 0U, 0);
  c8_y = c8_u0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void init_dsm_address_info(SFc8_torqueBalancingInstanceStruct
  *chartInstance)
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

void sf_c8_torqueBalancing_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1613742212U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2701524058U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2203730783U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3883560992U);
}

mxArray *sf_c8_torqueBalancing_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("y9gozZmlWAVcA9yKljzvb");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(23);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(23);
      pr[1] = (double)(1);
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
      pr[0] = (double)(23);
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
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(13));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(23);
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

mxArray *sf_c8_torqueBalancing_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c8_torqueBalancing_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c8_torqueBalancing(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[26],T\"nonlinearImpedences\",},{M[8],M[0],T\"is_active_c8_torqueBalancing\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c8_torqueBalancing_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc8_torqueBalancingInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc8_torqueBalancingInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _torqueBalancingMachineNumber_,
           8,
           1,
           1,
           0,
           6,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation(_torqueBalancingMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_torqueBalancingMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _torqueBalancingMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"qDes");
          _SFD_SET_DATA_PROPS(1,2,0,1,"nonlinearImpedences");
          _SFD_SET_DATA_PROPS(2,1,1,0,"q");
          _SFD_SET_DATA_PROPS(3,1,1,0,"qMin");
          _SFD_SET_DATA_PROPS(4,1,1,0,"qMax");
          _SFD_SET_DATA_PROPS(5,10,0,0,"gain");
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
        _SFD_CV_INIT_EML(0,1,2,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,601);
        _SFD_CV_INIT_EML_FCN(0,1,"stepFcn",603,-1,659);

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_sf_marshallOut,(MexInFcnForType)
            c8_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 23;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(5,SF_STRUCT,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_b_sf_marshallOut,(MexInFcnForType)c8_b_sf_marshallIn);

        {
          real_T (*c8_qDes)[23];
          real_T (*c8_nonlinearImpedences)[23];
          real_T (*c8_q)[23];
          real_T (*c8_qMin)[23];
          real_T (*c8_qMax)[23];
          c8_qMax = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 3);
          c8_qMin = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 2);
          c8_q = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 1);
          c8_nonlinearImpedences = (real_T (*)[23])ssGetOutputPortSignal
            (chartInstance->S, 1);
          c8_qDes = (real_T (*)[23])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c8_qDes);
          _SFD_SET_DATA_VALUE_PTR(1U, *c8_nonlinearImpedences);
          _SFD_SET_DATA_VALUE_PTR(2U, *c8_q);
          _SFD_SET_DATA_VALUE_PTR(3U, *c8_qMin);
          _SFD_SET_DATA_VALUE_PTR(4U, *c8_qMax);
          _SFD_SET_DATA_VALUE_PTR(5U, &chartInstance->c8_gain);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _torqueBalancingMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "tSmLhpyQhK7dMYjW1kR8ZD";
}

static void sf_opaque_initialize_c8_torqueBalancing(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc8_torqueBalancingInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c8_torqueBalancing((SFc8_torqueBalancingInstanceStruct*)
    chartInstanceVar);
  initialize_c8_torqueBalancing((SFc8_torqueBalancingInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c8_torqueBalancing(void *chartInstanceVar)
{
  enable_c8_torqueBalancing((SFc8_torqueBalancingInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c8_torqueBalancing(void *chartInstanceVar)
{
  disable_c8_torqueBalancing((SFc8_torqueBalancingInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c8_torqueBalancing(void *chartInstanceVar)
{
  sf_gateway_c8_torqueBalancing((SFc8_torqueBalancingInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c8_torqueBalancing(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c8_torqueBalancing
    ((SFc8_torqueBalancingInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c8_torqueBalancing();/* state var info */
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

extern void sf_internal_set_sim_state_c8_torqueBalancing(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c8_torqueBalancing();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c8_torqueBalancing((SFc8_torqueBalancingInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c8_torqueBalancing(SimStruct* S)
{
  return sf_internal_get_sim_state_c8_torqueBalancing(S);
}

static void sf_opaque_set_sim_state_c8_torqueBalancing(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c8_torqueBalancing(S, st);
}

static void sf_opaque_terminate_c8_torqueBalancing(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc8_torqueBalancingInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_torqueBalancing_optimization_info();
    }

    finalize_c8_torqueBalancing((SFc8_torqueBalancingInstanceStruct*)
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
  initSimStructsc8_torqueBalancing((SFc8_torqueBalancingInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c8_torqueBalancing(SimStruct *S)
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
    initialize_params_c8_torqueBalancing((SFc8_torqueBalancingInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c8_torqueBalancing(SimStruct *S)
{
  /* Actual parameters from chart:
     gain
   */
  const char_T *rtParamNames[] = { "gain" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0],
    sf_get_param_data_type_id(S,0));
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_torqueBalancing_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,8);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,8,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,8,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,8);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,8,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,8,1);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,8);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2564140811U));
  ssSetChecksum1(S,(4267258977U));
  ssSetChecksum2(S,(842372747U));
  ssSetChecksum3(S,(574328487U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c8_torqueBalancing(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c8_torqueBalancing(SimStruct *S)
{
  SFc8_torqueBalancingInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc8_torqueBalancingInstanceStruct *)utMalloc(sizeof
    (SFc8_torqueBalancingInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc8_torqueBalancingInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c8_torqueBalancing;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c8_torqueBalancing;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c8_torqueBalancing;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c8_torqueBalancing;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c8_torqueBalancing;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c8_torqueBalancing;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c8_torqueBalancing;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c8_torqueBalancing;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c8_torqueBalancing;
  chartInstance->chartInfo.mdlStart = mdlStart_c8_torqueBalancing;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c8_torqueBalancing;
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

void c8_torqueBalancing_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c8_torqueBalancing(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c8_torqueBalancing(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c8_torqueBalancing(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c8_torqueBalancing_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
