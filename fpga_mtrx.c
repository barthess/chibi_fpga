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

MtrxMath MTRXD;

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
static fpgaword_t fill_sizes_2(size_t m, size_t n) {
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
  osalDbgCheck(opcode <= MATH_OP_LAST);

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

  while ((B == A) || (B == C)) {
    B++;
  }
  return B;
}

/**
 * @brief Using same slice address forbidden
 */
static fpgaword_t fill_blk_adr_2(fpgaword_t A, fpgaword_t C, fpgaword_t opcode) {

  fpgaword_t B = generate_safe_B(A, C);
  return fill_blk_adr(A, B, C, opcode);
}

/**
 * @brief Using same slice address forbidden
 */
static fpgaword_t fill_blk_adr_1(fpgaword_t C, fpgaword_t opcode) {

  fpgaword_t A = 0;
  while (A == C) {
    A++;
  }

  fpgaword_t B = generate_safe_B(A, C);
  return fill_blk_adr(A, B, C, opcode);
}

/**
 * @brief   Calculate index from BRAM pointer
 */
static size_t get_idx(const double *slice) {

  size_t i = 0;

  for (i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    if (slice == MTRXD.pool[i]) {
      return i;
    }
  }

  osalSysHalt("Incorrect pointer");
  return -1;
}

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */

/**
 *
 */
void fpgaMtrxObjectInit(MtrxMath *mtrxp) {

  mtrxp->state = MTRX_STOP;
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
void fpgaMtrxStart(MtrxMath *mtrxp, const FPGADriver *fpgap) {

  if (MTRX_READY == mtrxp->state) {
    return;
  }

  osalDbgCheck(NULL != fpgap);
  osalDbgCheck(FPGA_READY == fpgap->state);

  mtrxp->fpgap = fpgap;
  osalDbgCheck(MTRX_UNINIT != mtrxp->state);

  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    mtrxp->pool[i] = (double *)fpgaGetSlicePtr(fpgap, FPGA_WB_SLICE_MUL_BUF0 + i);
  }

  fpgaword_t *ctl = fpgaGetSlicePtr(fpgap, FPGA_WB_SLICE_MUL_CTL);
  mtrxp->op = &ctl[FPGA_CTL_OP_OFFSET];
  mtrxp->sizes = &ctl[FPGA_CTL_SIZES_OFFSET];
  mtrxp->constant = (double *)&ctl[FPGA_CTL_CONSTANT_OFFSET];

  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    mtrxp->empty |= 1U << i;
  }

  mtrxp->state = MTRX_READY;
}

/**
 *
 */
void fpgaMtrxStop(MtrxMath *mtrxp) {
  mtrxp->state = MTRX_STOP;
}

/**
 * @brief   Returns pointer to first empty BRAM slice.
 * @retval  NULL if pool empty or error happened.
 */
double * fpgaMtrxMalloc(void) {

  // pool is empty
  if (0 == MTRXD.empty) {
    return NULL;
  }

  osalDbgCheck((MTRX_READY == MTRXD.state) || (MTRX_ACTIVE == MTRXD.state));
  osalDbgCheck(MTRXD.empty < (1U << FPGA_MTRX_BRAMS_CNT)); // pool corrupted

  // find first empty slice
  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    const uint32_t mask = 1U << i;
    if ((MTRXD.empty & mask) == mask) {
      MTRXD.empty &= ~mask;
      return MTRXD.pool[i];
    }
  }

  // error trap
  osalSysHalt("");
  return NULL;
}

/**
 *
 */
void fpgaMtrxFree(void *slice) {

  // such behavior is a C standard when memory allocation failes
  if (NULL == slice) {
    return;
  }

  osalDbgCheck((MTRX_READY == MTRXD.state) || (MTRX_ACTIVE == MTRXD.state));
  MTRXD.empty |= 1U << get_idx(slice);
}

/**
 *
 */
void fpgaMtrxDot(size_t m, size_t p, size_t n,
                 const double *A, const double *B, double *C){

  osalDbgCheck(MTRX_READY == MTRXD.state);

  *MTRXD.sizes = fill_sizes_3(m, p, n);
  *MTRXD.op = fill_blk_adr_3(get_idx(A), get_idx(B), get_idx(C), MATH_OP_DOT);
  wait_polling();
}

/**
 *
 */
void fpgaMtrxAdd(size_t m, size_t n, const double *A, const double *B, double *C){

  osalDbgCheck(MTRX_READY == MTRXD.state);

  *MTRXD.sizes = fill_sizes_2(m, n);
  *MTRXD.op = fill_blk_adr_3(get_idx(A), get_idx(B), get_idx(C), MATH_OP_ADD);
  wait_polling();
}

/**
 *
 */
void fpgaMtrxSub(size_t m, size_t n, const double *A, const double *B, double *C){

  osalDbgCheck(MTRX_READY == MTRXD.state);

  *MTRXD.sizes = fill_sizes_2(m, n);
  *MTRXD.op = fill_blk_adr_3(get_idx(A), get_idx(B), get_idx(C), MATH_OP_SUB);
  wait_polling();
}

/**
 *
 */
void fpgaMtrxMul(size_t m, size_t n, const double *A, const double *B, double *C){

  osalDbgCheck(MTRX_READY == MTRXD.state);

  *MTRXD.sizes = fill_sizes_2(m, n);
  *MTRXD.op = fill_blk_adr_3(get_idx(A), get_idx(B), get_idx(C), MATH_OP_MUL);
  wait_polling();
}

/**
 *
 */
void fpgaMtrxScale(size_t m, size_t n, const double *A, double *C, double scale){

  osalDbgCheck(MTRX_READY == MTRXD.state);

  *MTRXD.constant = scale;
  *MTRXD.sizes = fill_sizes_2(m, n);
  *MTRXD.op = fill_blk_adr_2(get_idx(A), get_idx(C), MATH_OP_SCALE);
  wait_polling();
}

/**
 *
 */
void fpgaMtrxCpy(size_t m, size_t n, const double *A, double *C) {

  osalDbgCheck(MTRX_READY == MTRXD.state);

  *MTRXD.sizes = fill_sizes_2(m, n);
  *MTRXD.op = fill_blk_adr_2(get_idx(A), get_idx(C), MATH_OP_CPY);
  wait_polling();
}

/**
 *
 */
void fpgaMtrxTrn(size_t m, size_t n, const double *A, double *C) {

  osalDbgCheck(MTRX_READY == MTRXD.state);

  *MTRXD.sizes = fill_sizes_2(m, n);
  *MTRXD.op = fill_blk_adr_2(get_idx(A), get_idx(C), MATH_OP_TRN);
  wait_polling();
}

/**
 *
 */
void fpgaMtrxSet(size_t m, size_t n, double *C, double val) {

  osalDbgCheck(MTRX_READY == MTRXD.state);

  *MTRXD.constant = val;
  *MTRXD.sizes = fill_sizes_2(m, n);
  *MTRXD.op = fill_blk_adr_1(get_idx(C), MATH_OP_SET);
  wait_polling();
}

/**
 *
 */
void fpgaMtrxDia(size_t m, double *C, double val) {

  osalDbgCheck(MTRX_READY == MTRXD.state);

  *MTRXD.constant = val;
  *MTRXD.sizes = fill_sizes_2(m, m);
  *MTRXD.op = fill_blk_adr_1(get_idx(C), MATH_OP_DIA);
  wait_polling();
}



