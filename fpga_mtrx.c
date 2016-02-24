#include "main.h"
#include "pads.h"

#include "fpga_mtrx.h"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

Mtrx MTRXD1;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 *******************************************************************************
 *******************************************************************************
 * LOCAL FUNCTIONS
 *******************************************************************************
 *******************************************************************************
 */

/**
 *
 */
static void wait_polling(void) {

  orange_led_on();

  while (! FSMCDataFlushed())
    ;

  while(! FPGAMathRdy())
    ;

  orange_led_off();
}

/**
 * @brief   Construct command word with correctly filled sizes
 * @note    Sizes must be in widely used notation e.g. m=1, p=1 denotes
 *          singular matrix from 1 element.
 */
static fpgaword_t fill_sizes_3(size_t m, size_t p, size_t n) {

  m -= 1;
  p -= 1;
  n -= 1;

  osalDbgCheck((m <= FPGA_MTRX_MAX_INDEX) &
               (p <= FPGA_MTRX_MAX_INDEX) &
               (n <= FPGA_MTRX_MAX_INDEX));

  fpgaword_t ret;
  ret   = m << FPGA_MTRX_INDEX_WIDTH * 0;
  ret  |= p << FPGA_MTRX_INDEX_WIDTH * 1;
  ret  |= n << FPGA_MTRX_INDEX_WIDTH * 2;

  return ret;
}

/**
 * @brief   Construct command word with correctly filled sizes
 * @note    Sizes must be in widely used notation e.g. m=1, p=1 denotes
 *          singular matrix from 1 element.
 */
fpgaword_t fill_sizes_2(size_t m, size_t n) {
  m -= 1;
  n -= 1;

  osalDbgCheck((m <= FPGA_MTRX_MAX_INDEX) &
               (n <= FPGA_MTRX_MAX_INDEX));

  fpgaword_t ret;
  ret   = m << FPGA_MTRX_INDEX_WIDTH * 0;
  ret  |= n << FPGA_MTRX_INDEX_WIDTH * 2;

  return ret;
}

/**
 *
 */
static fpgaword_t fill_blk_adr(size_t A, size_t B, size_t C, size_t opcode) {

  osalDbgCheck((A != B) & (B != C) & (C != A));
  osalDbgCheck((A < FPGA_MTRX_BRAMS_CNT) &
               (B < FPGA_MTRX_BRAMS_CNT) &
               (C < FPGA_MTRX_BRAMS_CNT));
  osalDbgCheck(opcode < MATH_OP_LAST);

  fpgaword_t ret;
  ret  = A << FPGA_MTRX_BRAMS_CNT_BITS * 0;
  ret |= B << FPGA_MTRX_BRAMS_CNT_BITS * 1;
  ret |= C << FPGA_MTRX_BRAMS_CNT_BITS * 2;
  ret |= opcode << FPGA_MTRX_BRAMS_CNT_BITS * 3;
  ret |= 1 << FPGA_MTRX_DV_BIT;

  return ret;
}

/**
 *
 */
static fpgaword_t fill_blk_adr_3(size_t A, size_t B, size_t C, size_t opcode) {
  return fill_blk_adr(A, B, C, opcode);
}

/**
 * @brief Using same slice address forbidden
 */
static fpgaword_t generate_safe_B(fpgaword_t A, fpgaword_t C) {
  fpgaword_t B = 0;

  while ((B == A) && (B == C)) {
    B++;
  }
  return B;
}

/**
 * @brief Using same slice address forbidden
 */
fpgaword_t fill_blk_adr_2(fpgaword_t A, fpgaword_t C, fpgaword_t opcode) {

  fpgaword_t B = generate_safe_B(A, C);
  return fill_blk_adr(A, B, C, opcode);
}

/**
 * @brief Using same slice address forbidden
 */
fpgaword_t fill_blk_adr_1(fpgaword_t C, fpgaword_t opcode) {

  fpgaword_t A = 0;
  while (A == C) {
    A++;
  }

  fpgaword_t B = generate_safe_B(A, C);
  return fill_blk_adr(A, B, C, opcode);
}

/**
 *
 */
size_t get_idx(Mtrx *mtrxp, const double *slice) {

  osalDbgCheck(NULL != slice);

  uintptr_t bottom = (uintptr_t)fpgaGetSlicePtr(mtrxp->fpgap, FPGA_WB_SLICE_MUL_BUF0);
  uintptr_t idx = (uintptr_t)slice;

  idx -= bottom;
  idx /= FPGA_WB_SLICE_SIZE * sizeof(fpgaword_t);

  osalDbgCheck(idx < FPGA_MTRX_BRAMS_CNT);

  return idx;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
void fpgaMtrxObjectInit(Mtrx *mtrxp) {

  mtrxp->state = MTRXMUL_STOP;
  mtrxp->op = NULL;
  mtrxp->sizes = NULL;
  mtrxp->constant = NULL;
  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    mtrxp->pool[i] = NULL;
  }
  mtrxp->empty = 0;
}

/**
 *
 */
void fpgaMtrxStart(Mtrx *mtrxp, const FPGADriver *fpgap) {

  if (MTRXMUL_READY == mtrxp->state) {
    return;
  }

  osalDbgCheck(NULL != fpgap);
  osalDbgCheck(FPGA_READY == mtrxp->fpgap->state);
  mtrxp->fpgap = fpgap;

  for (size_t i=0; i<8; i++) {
    mtrxp->pool[i] = (double *)fpgaGetSlicePtr(fpgap, FPGA_WB_SLICE_MUL_BUF0 + i);
  }

  fpgaword_t *ctl = fpgaGetSlicePtr(fpgap, FPGA_WB_SLICE_MUL_CTL);
  mtrxp->op = &ctl[FPGA_CTL_OP_OFFSET];
  mtrxp->sizes = &ctl[FPGA_CTL_SIZES_OFFSET];
  mtrxp->constant = (double *)&ctl[FPGA_CTL_CONSTANT_OFFSET];

  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    mtrxp->empty |= 1U << i;
  }

  mtrxp->state = MTRXMUL_READY;
}

/**
 *
 */
void fpgaMtrxStop(Mtrx *mtrxp) {
  mtrxp->state = MTRXMUL_STOP;
}

/**
 * @brief   Returns pointer to first empty BRAM slice.
 * @retval  NULL if pool empty or error happened.
 */
double * fpgaMtrxMalloc(Mtrx *mtrxp, size_t *slice_idx) {

  // pool is empty
  if (0 == mtrxp->empty) {
    return NULL;
  }

  osalDbgCheck((MTRXMUL_READY == mtrxp->state) && (MTRXMUL_ACTIVE == mtrxp->state));
  osalDbgCheck(mtrxp->empty < (1U << FPGA_MTRX_BRAMS_CNT)); // pool corrupted

  // find first empty slice
  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    const uint32_t mask = 1U << i;
    if ((mtrxp->empty & mask) == mask) {
      mtrxp->empty &= ~mask;
      *slice_idx = i;
      return mtrxp->pool[i];
    }
  }

  // error trap
  osalSysHalt("");
  return NULL;
}

/**
 *
 */
bool fpgaMtrxHaveFreeSlice(Mtrx *mtrxp) {

  return mtrxp->empty > 0;
}

/**
 *
 */
void fpgaMtrxFree(Mtrx *mtrxp, void *slice, size_t slice_idx) {

  // such behavior is C standard.
  if (NULL == slice) {
    return;
  }

  osalDbgCheck((MTRXMUL_READY == mtrxp->state) && (MTRXMUL_ACTIVE == mtrxp->state));

  mtrxp->empty |= 1U << slice_idx;
}

/**
 * @brief   Returns pointer by index. Convenient function.
 */
double * fpgaMtrxDataPtr(Mtrx *mtrxp, size_t slice_idx) {

  osalDbgCheck((MTRXMUL_READY == mtrxp->state) && (MTRXMUL_ACTIVE == mtrxp->state));
  osalDbgCheck(slice_idx < FPGA_MTRX_BRAMS_CNT);

  return mtrxp->pool[slice_idx];
}

/**
 *
 */
void fpgaMtrxDot(Mtrx *mtrxp, size_t m, size_t p, size_t n,
                              size_t A, size_t B, size_t C){

  osalDbgCheck(MTRXMUL_READY == mtrxp->state);

  *mtrxp->sizes = fill_sizes_3(m, p, n);
  *mtrxp->op = fill_blk_adr_3(A, B, C, MATH_OP_DOT);
  wait_polling();
}

/**
 *
 */
void fpgaMtrxAdd(Mtrx *mtrxp, size_t m,           size_t n,
                              size_t A, size_t B, size_t C){

  osalDbgCheck(MTRXMUL_READY == mtrxp->state);

  *mtrxp->sizes = fill_sizes_2(m, n);
  *mtrxp->op = fill_blk_adr_3(A, B, C, MATH_OP_ADD);
  wait_polling();
}

/**
 *
 */
void fpgaMtrxSub(Mtrx *mtrxp, size_t m,           size_t n,
                              size_t A, size_t B, size_t C){

  osalDbgCheck(MTRXMUL_READY == mtrxp->state);

  *mtrxp->sizes = fill_sizes_2(m, n);
  *mtrxp->op = fill_blk_adr_3(A, B, C, MATH_OP_SUB);
  wait_polling();
}

/**
 *
 */
void fpgaMtrxMul(Mtrx *mtrxp, size_t m,           size_t n,
                              size_t A, size_t B, size_t C){

  osalDbgCheck(MTRXMUL_READY == mtrxp->state);

  *mtrxp->sizes = fill_sizes_2(m, n);
  *mtrxp->op = fill_blk_adr_3(A, B, C, MATH_OP_MUL);
  wait_polling();
}

/**
 *
 */
void fpgaMtrxScale(Mtrx *mtrxp, size_t m, size_t n,
                                size_t A, size_t C, double scale){

  osalDbgCheck(MTRXMUL_READY == mtrxp->state);

  *mtrxp->constant = scale;
  *mtrxp->sizes = fill_sizes_2(m, n);
  *mtrxp->op = fill_blk_adr_2(A, C, MATH_OP_SCALE);
  wait_polling();
}

/**
 *
 */
void fpgaMtrxCpy(Mtrx *mtrxp, size_t m, size_t n, size_t A, size_t C) {

  osalDbgCheck(MTRXMUL_READY == mtrxp->state);

  *mtrxp->sizes = fill_sizes_2(m, n);
  *mtrxp->op = fill_blk_adr_2(A, C, MATH_OP_CPY);

  wait_polling();
}

/**
 *
 */
void fpgaMtrxTrn(Mtrx *mtrxp, size_t m, size_t n, size_t A, size_t C) {

  osalDbgCheck(MTRXMUL_READY == mtrxp->state);

  *mtrxp->sizes = fill_sizes_2(m, n);
  *mtrxp->op = fill_blk_adr_2(A, C, MATH_OP_TRN);

  wait_polling();
}

/**
 *
 */
void fpgaMtrxSet(Mtrx *mtrxp, size_t m, size_t n, size_t C, double val) {

  osalDbgCheck(MTRXMUL_READY == mtrxp->state);

  *mtrxp->constant = val;
  *mtrxp->sizes = fill_sizes_2(m, n);
  *mtrxp->op = fill_blk_adr_1(C, MATH_OP_SET);

  wait_polling();
}

/**
 *
 */
void fpgaMtrxDia(Mtrx *mtrxp, size_t m, size_t C, double val) {

  osalDbgCheck(MTRXMUL_READY == mtrxp->state);

  *mtrxp->constant = val;
  *mtrxp->sizes = fill_sizes_2(m, m);
  *mtrxp->op = fill_blk_adr_1(C, MATH_OP_DIA);

  wait_polling();
}



