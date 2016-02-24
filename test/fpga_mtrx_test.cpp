#include <cmath>
#include <cstdlib>
#include <cstring>

#include "main.h"
#include "pads.h"

#include "fpga_mtrx_test.hpp"
#include "fpga_mem_test.hpp"

#include "embmatrix2/matrix_soft_engine.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

static const size_t CTL_OP = 0;
static const size_t CTL_SIZES = 1;

#define RAND_POOL_LEN        (65536 / sizeof(double))
#define RAND_POOL_ADR_MASK   (RAND_POOL_LEN - 1)

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static double *fpga_pool[8]; // will be initialized later

static double mtrx_pool[FPGA_MTRX_BRAMS_CNT]
                       [(1 << FPGA_MTRX_INDEX_WIDTH) * (1 << FPGA_MTRX_INDEX_WIDTH)];

__CCM__ static double rand_pool[RAND_POOL_LEN];

static time_measurement_t tmu_soft;
static time_measurement_t tmu_hard;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/**
 *
 */
double rand_double(void) {
  const double MAX_MTRX_NUM = 1000;
  const double MIN_MTRX_NUM = .0001;
  double r = MAX_MTRX_NUM + 1;
  double a, b;

  while ((r > MAX_MTRX_NUM) or (r < MIN_MTRX_NUM)) {
    a = rand();
    b = rand();
    r = a / b;
  }

  if ((rand() & 1) == 1)
    return -r;
  else
    return r;
}

/**
 *
 */
void init_rand_pool(void) {
  for (size_t i=0; i<RAND_POOL_LEN; i++) {
    rand_pool[i] = rand_double();
  }
}

/**
 *
 */
double fast_rand_double(void) {
  return rand_pool[rand() & RAND_POOL_ADR_MASK];
}

/**
 *
 */
double pyramid_accumulate(const double *d, size_t len) {
  size_t N;

  if (len % 2 == 1) {
    N = (len + 1) / 2;
  }
  else {
    N = len / 2;
  }

  double buf[N];
  size_t i = 0, j = 0;
  while (len > 1) {
    buf[i] = d[j] + d[j+1];
    i++;
    j += 2;
    len -= 2;
  }

  // copy last element as is when len is odd
  if (len == 1) {
    buf[i] = d[j];
  }

  if (1 == N)
    return buf[0];
  else
    return(pyramid_accumulate(buf, N)); // recursive call
}

/**
 * @brief   multiply matrix A(m x p) by  B(p x n), put result in C(m x n)
 */
void matrix_multiply_fpga_like(size_t m, size_t p, size_t n,
                               const double *A, const double *B, double *C) {
  size_t i, j, k;
  double tmp[p];

  for(i=0; i<m; i++) {      //each row in A
    for(j=0; j<n; j++) {    //each column in B
      for(k=0; k<p; k++) {  //each element in row A & column B
        tmp[k] = A[i*p + k] * B[k*n + j];
      }
      *C++ = pyramid_accumulate(tmp, p);
    }
  }
}

/**
 *
 */
template <typename T>
static void mtrx_compare_approx(const T *soft_dat, const T *fpga_dat, size_t m, size_t n) {
  T tmp1, tmp2;
  m++;
  n++;
  for(size_t i=0; i<m*n; i++) {
    tmp1 = soft_dat[i];
    tmp2 = fpga_dat[i];
    if (fabsf(tmp1 - tmp2) > 0.1) {
      red_led_on();
      osalSysHalt("");
    }
  }
}

/**
 *
 */
static volatile size_t err_cnt = 0;
template <typename T>
static void mtrx_compare_exact(const T *soft_dat, const T *fpga_dat, size_t m, size_t n) {
  T tmp1, tmp2;

  m++;
  n++;
  for(size_t i=0; i<m*n; i++) {
    tmp1 = soft_dat[i];
    tmp2 = fpga_dat[i];
    if (fabsf(tmp1 - tmp2) > 0) {
      red_led_on();
      err_cnt++;
      osalSysHalt("");
    }
  }
}

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
 *
 */
fpgaword_t fill_sizes_3(size_t m, size_t p, size_t n) {

  fpgaword_t ret;

  osalDbgCheck((m <= FPGA_MTRX_MAX_INDEX) &
               (p <= FPGA_MTRX_MAX_INDEX) &
               (n <= FPGA_MTRX_MAX_INDEX));

  ret   = m << FPGA_MTRX_INDEX_WIDTH * 0;
  ret  |= p << FPGA_MTRX_INDEX_WIDTH * 1;
  ret  |= n << FPGA_MTRX_INDEX_WIDTH * 2;

  return ret;
}

/**
 *
 */
fpgaword_t fill_sizes_2(size_t m, size_t n) {
  fpgaword_t ret;

  osalDbgCheck((m <= FPGA_MTRX_MAX_INDEX) &
               (n <= FPGA_MTRX_MAX_INDEX));

  ret   = m << FPGA_MTRX_INDEX_WIDTH * 0;
  ret  |= n << FPGA_MTRX_INDEX_WIDTH * 2;

  return ret;
}

/**
 *
 */
fpgaword_t _fill_blk_adr(fpgaword_t A, fpgaword_t B, fpgaword_t C, fpgaword_t opcode) {
  fpgaword_t ret;

  osalDbgCheck((A != B) & (B != C) & (C != A));
  osalDbgCheck((A < FPGA_MTRX_BRAMS_CNT) &
               (B < FPGA_MTRX_BRAMS_CNT) &
               (C < FPGA_MTRX_BRAMS_CNT));

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
fpgaword_t fill_blk_adr3(fpgaword_t A, fpgaword_t B, fpgaword_t C, fpgaword_t opcode) {
  return _fill_blk_adr(A, B, C, opcode);
}

/**
 * @brief Using same slice address forbidden
 */
fpgaword_t _generate_B(fpgaword_t A, fpgaword_t C) {
  fpgaword_t B = 0;
  while ((B == A) or (B == C)) {
    B++;
  }
  return B;
}

/**
 * @brief Using same slice address forbidden
 */
fpgaword_t fill_blk_adr_2(fpgaword_t A, fpgaword_t C, fpgaword_t opcode) {

  fpgaword_t B = _generate_B(A, C);
  return _fill_blk_adr(A, B, C, opcode);
}

/**
 * @brief Using same slice address forbidden
 */
fpgaword_t fill_blk_adr_1(fpgaword_t C, fpgaword_t opcode) {

  fpgaword_t A = 0;
  while (A == C) {
    A++;
  }

  fpgaword_t B = _generate_B(A, C);
  return _fill_blk_adr(A, B, C, opcode);
}

/**
 *
 */
void fill_constant(const double val, fpgaword_t *ctl) {
  memcpy(&ctl[4], &val, sizeof(val));
}

/*****************************************************************************************
 * Hardware API
 *
 * функции принимают размеры, уже пригодные для прямой записи в регистры,
 * т.е. М(0х0) - это вырожденная матрица 1х1
 *****************************************************************************************/

void fpga_mtrx_dot(size_t m, size_t p, size_t n,
                   size_t A, size_t B, size_t C,
                   fpgaword_t *ctl) {

  ctl[CTL_SIZES] = fill_sizes_3(m, p, n);
  ctl[CTL_OP]    = fill_blk_adr3(A, B, C, MATH_OP_DOT);

  wait_polling();
}

/**
 *
 */
void fpga_mtrx_add(size_t m,           size_t n,
                   size_t A, size_t B, size_t C,
                   fpgaword_t *ctl) {

  ctl[CTL_SIZES] = fill_sizes_2(m, n);
  ctl[CTL_OP]    = fill_blk_adr3(A, B, C, MATH_OP_ADD);

  wait_polling();
}

/**
 *
 */
void fpga_mtrx_sub(size_t m,           size_t n,
                   size_t A, size_t B, size_t C,
                   fpgaword_t *ctl) {

  ctl[CTL_SIZES] = fill_sizes_2(m, n);
  ctl[CTL_OP]    = fill_blk_adr3(A, B, C, MATH_OP_SUB);

  wait_polling();
}

/**
 *
 */
void fpga_mtrx_mul(size_t m,           size_t n,
                   size_t A, size_t B, size_t C,
                   fpgaword_t *ctl) {

  ctl[CTL_SIZES] = fill_sizes_2(m, n);
  ctl[CTL_OP]    = fill_blk_adr3(A, B, C, MATH_OP_MUL);

  wait_polling();
}

/**
 *
 */
void fpga_mtrx_scale(size_t m,           size_t n,
                     size_t A,           size_t C,
                     fpgaword_t *ctl, double scale) {

  fill_constant(scale, ctl);

  ctl[CTL_SIZES] = fill_sizes_2(m, n);
  ctl[CTL_OP]    = fill_blk_adr_2(A, C, MATH_OP_SCALE);

  wait_polling();
}

/**
 *
 */
void fpga_mtrx_cpy(size_t m,           size_t n,
                   size_t A,           size_t C,
                   fpgaword_t *ctl) {

  ctl[CTL_SIZES] = fill_sizes_2(m, n);
  ctl[CTL_OP]    = fill_blk_adr_2(A, C, MATH_OP_CPY);

  wait_polling();
}

/**
 *
 */
void fpga_mtrx_trn(size_t m,           size_t n,
                   size_t A,           size_t C,
                   fpgaword_t *ctl) {

  ctl[CTL_SIZES] = fill_sizes_2(m, n);
  ctl[CTL_OP]    = fill_blk_adr_2(A, C, MATH_OP_TRN);

  wait_polling();
}

/**
 *
 */
void fpga_mtrx_set(size_t m,           size_t n,
                                       size_t C,
                   fpgaword_t *ctl, double set_val) {

  fill_constant(set_val, ctl);

  ctl[CTL_SIZES] = fill_sizes_2(m, n);
  ctl[CTL_OP]    = fill_blk_adr_1(C, MATH_OP_SET);

  wait_polling();
}

/**
 *
 */
void fpga_mtrx_dia(size_t m,
                                       size_t C,
                   fpgaword_t *ctl, double set_val) {

  fill_constant(set_val, ctl);

  ctl[CTL_SIZES] = fill_sizes_2(m, m);
  ctl[CTL_OP]    = fill_blk_adr_1(C, MATH_OP_DIA);

  wait_polling();
}

/*****************************************************************************************
 * Software variants
 *****************************************************************************************/
/**
 *
 */
void soft_mtrx_dot(size_t m, size_t p, size_t n,
                   size_t A, size_t B, size_t C) {
  m++;
  p++;
  n++;
  matrix_multiply_fpga_like(m, p, n, mtrx_pool[A], mtrx_pool[B], mtrx_pool[C]);
  //matrix::matrix_multiply(m, p, n, mtrx_pool[A], mtrx_pool[B], mtrx_pool[C]);
}

/**
 *
 */
void soft_mtrx_add(size_t m,           size_t n,
                   size_t A, size_t B, size_t C) {
  m++;
  n++;
  matrix::matrix_soft_add(m, n, mtrx_pool[A], mtrx_pool[B], mtrx_pool[C]);
}

/**
 *
 */
void soft_mtrx_sub(size_t m,           size_t n,
                   size_t A, size_t B, size_t C) {
  m++;
  n++;
  matrix::matrix_soft_sub(m, n, mtrx_pool[A], mtrx_pool[B], mtrx_pool[C]);
}

/**
 *
 */
void soft_mtrx_mul(size_t m,           size_t n,
                   size_t A, size_t B, size_t C) {
  m++;
  n++;
  for (size_t i=0; i<m*n; i++) {
    mtrx_pool[C][i] = mtrx_pool[A][i] * mtrx_pool[B][i];
  }
}

/**
 *
 */
void soft_mtrx_scale(size_t m,           size_t n,
                     size_t A,           size_t C,
                     double scale) {
  m++;
  n++;
  matrix::matrix_soft_scale(m*n, mtrx_pool[A], mtrx_pool[C], scale);
}

/**
 *
 */
void soft_mtrx_cpy(size_t m,           size_t n,
                   size_t A,           size_t C) {
  m++;
  n++;
  for (size_t i=0; i<m*n; i++) {
    mtrx_pool[C][i] = mtrx_pool[A][i];
  }
}

/**
 *
 */
void soft_mtrx_set(size_t m,           size_t n,
                                       size_t C,
                   double set_val) {
  m++;
  n++;
  for (size_t i=0; i<m*n; i++) {
    mtrx_pool[C][i] = set_val;
  }
}

/**
 *
 */
void soft_mtrx_trn(size_t m,           size_t n,
                   size_t A,           size_t C) {
  m++;
  n++;
  matrix::matrix_soft_transpose(m, n, mtrx_pool[A], mtrx_pool[C]);
}

/**
 *
 */
void soft_mtrx_dia(size_t m,
                                       size_t C,
                   double set_val) {
  size_t i = 0;

  m++;

  // Old EYE code
//  for (i=0; i<m*m; i++) {
//    mtrx_pool[C][i] = set_val;
//  }
//
//  i = 0;
//  while (i < m*m) {
//    mtrx_pool[C][i] = 1;
//    i += m+1;
//  }

  // New diagonal code
  for (i=0; i<m*m; i++) {
    mtrx_pool[C][i] = 0;
  }

  i = 0;
  while (i < m*m) {
    mtrx_pool[C][i] = set_val;
    i += m+1;
  }
}


/*******************************************************************************
 * manual fill of arrays in FPGA
 *******************************************************************************/
void manual_fill_pattern(double *ptr, double pattern, bool increment, size_t m, size_t n) {
  m++;
  n++;
  for (size_t i=0; i<m*n; i++) {
    ptr[i] = pattern;
    if (increment) {
      pattern += 1;
    }
  }
}

void manual_fill_copy(double *ptr, const double *data, size_t m, size_t n) {
  m++;
  n++;
  for (size_t i=0; i<m*n; i++) {
    ptr[i] = data[i];
  }
}

void manual_fill_rand(double *ptr, size_t m, size_t n) {

  m++;
  n++;
  for (size_t i=0; i<m*n; i++) {
    ptr[i] = fast_rand_double();
  }
}

/*******************************************************************************
 * Test wrappers
 *******************************************************************************/

/**
 *
 */
void fpga_dot_test(size_t m, size_t p, size_t n,
                   size_t A, size_t B, size_t C,
                   fpgaword_t *ctl) {

  manual_fill_rand(mtrx_pool[A], m, p);
  manual_fill_rand(mtrx_pool[B], p, n);
  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, p);
  manual_fill_copy(fpga_pool[B], mtrx_pool[B], p, n);
  manual_fill_pattern(mtrx_pool[C], 666, false, m, n);
  manual_fill_copy(fpga_pool[C], mtrx_pool[C], m, n);

  soft_mtrx_dot(m, p, n, A, B, C);

  chTMStartMeasurementX(&tmu_hard);
  fpga_mtrx_dot(m, p, n, A, B, C, ctl);
  chTMStopMeasurementX(&tmu_hard);

  //mtrx_compare_approx(mtrx_pool[C], fpga_pool[C], m, n);
  mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n);
}

/**
 *
 */
void fpga_add_test(size_t m,           size_t n,
                   size_t A, size_t B, size_t C,
                   fpgaword_t *ctl) {

  manual_fill_rand(mtrx_pool[A], m, n);
  manual_fill_rand(mtrx_pool[B], m, n);
  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
  manual_fill_copy(fpga_pool[B], mtrx_pool[B], m, n);
  soft_mtrx_add(m, n, A, B, C);
  fpga_mtrx_add(m, n, A, B, C, ctl);
  mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n);

  manual_fill_rand(mtrx_pool[A], m, n);
  manual_fill_rand(mtrx_pool[B], m, n);
  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
  manual_fill_copy(fpga_pool[B], mtrx_pool[B], m, n);
  soft_mtrx_sub(m, n, A, B, C);
  fpga_mtrx_sub(m, n, A, B, C, ctl);
  mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n);
}

/**
 *
 */
void fpga_mul_test(size_t m,           size_t n,
                   size_t A, size_t B, size_t C,
                   fpgaword_t *ctl) {

  double scale = fast_rand_double();

  manual_fill_rand(mtrx_pool[A], m, n);
  manual_fill_rand(mtrx_pool[B], m, n);
  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
  manual_fill_copy(fpga_pool[B], mtrx_pool[B], m, n);
  soft_mtrx_scale(m, n, A, C, scale);
  fpga_mtrx_scale(m, n, A, C, ctl, scale);
  mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n);

  manual_fill_rand(mtrx_pool[A], m, n);
  manual_fill_rand(mtrx_pool[B], m, n);
  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
  manual_fill_copy(fpga_pool[B], mtrx_pool[B], m, n);
  soft_mtrx_mul(m, n, A, B, C);
  fpga_mtrx_mul(m, n, A, B, C, ctl);
  mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n);
}

/**
 *
 */
void fpga_mov_test(size_t m,           size_t n,
                   size_t A,           size_t C,
                   fpgaword_t *ctl) {

  double set_val;

  manual_fill_pattern(mtrx_pool[A], m+n, true, m, n);
  manual_fill_pattern(mtrx_pool[1], 111, true, m, n);
  manual_fill_pattern(mtrx_pool[C], -10, true, m, n);
  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
  manual_fill_copy(fpga_pool[1], mtrx_pool[1], m, n);
  manual_fill_copy(fpga_pool[C], mtrx_pool[C], m, n);
  soft_mtrx_cpy(m, n, A, C);
  fpga_mtrx_cpy(m, n, A, C, ctl);
  mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n);

  manual_fill_rand(mtrx_pool[A], m, n);
  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
  manual_fill_rand(mtrx_pool[C], m, n);
  manual_fill_copy(fpga_pool[C], mtrx_pool[C], m, n);
  soft_mtrx_trn(m, n, A, C);
  fpga_mtrx_trn(m, n, A, C, ctl);
  mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n);

  manual_fill_rand(mtrx_pool[A], m, n);
  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
  manual_fill_rand(mtrx_pool[C], m, n);
  manual_fill_copy(fpga_pool[C], mtrx_pool[C], m, n);
  soft_mtrx_cpy(m, n, A, C);
  fpga_mtrx_cpy(m, n, A, C, ctl);
  mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n);

  set_val = fast_rand_double();
  manual_fill_rand(mtrx_pool[A], m, n);
  manual_fill_copy(fpga_pool[A], mtrx_pool[A], m, n);
  manual_fill_rand(mtrx_pool[C], m, n);
  manual_fill_copy(fpga_pool[C], mtrx_pool[C], m, n);
  soft_mtrx_set(m, n, C, set_val);
  fpga_mtrx_set(m, n, C, ctl, set_val);
  mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n);


  // Eye works correctly only with square matrices
  if (m == n) {
    set_val = fast_rand_double();
    manual_fill_rand(mtrx_pool[C], m, n);
    manual_fill_copy(fpga_pool[C], mtrx_pool[A], m, n);
    soft_mtrx_dia(m, C, set_val);
    fpga_mtrx_dia(m, C, ctl, set_val);
    mtrx_compare_exact(mtrx_pool[C], fpga_pool[C], m, n);
  }
}

/**
 *
 */
void test_dot_corner(fpgaword_t *ctl) {
  const size_t A = 0;
  const size_t B = 1;
  const size_t C = 2;

  size_t m, p, n;

  m = 31;
  p = 31;
  n = 31;
  fpga_dot_test(m, p, n, A, B, C, ctl);

  m = 0;
  p = 0;
  n = 0;
  fpga_dot_test(m, p, n, A, B, C, ctl);

  m = 31;
  p = 0;
  n = 0;
  fpga_dot_test(m, p, n, A, B, C, ctl);

  m = 0;
  p = 31;
  n = 0;
  fpga_dot_test(m, p, n, A, B, C, ctl);

  m = 0;
  p = 0;
  n = 31;
  fpga_dot_test(m, p, n, A, B, C, ctl);

  m = 31;
  p = 31;
  n = 0;
  fpga_dot_test(m, p, n, A, B, C, ctl);

  m = 0;
  p = 31;
  n = 31;
  fpga_dot_test(m, p, n, A, B, C, ctl);

  m = 31;
  p = 0;
  n = 31;
  fpga_dot_test(m, p, n, A, B, C, ctl);

  const size_t MAX_BRUTE_SIZE = 4;
  m = MAX_BRUTE_SIZE;
  p = MAX_BRUTE_SIZE;
  n = MAX_BRUTE_SIZE;

  while(m--) {
    while(p--) {
      while(n--) {
        fpga_dot_test(m, p, n, A, B, C, ctl);
      }
      n = MAX_BRUTE_SIZE;
    }
    p = MAX_BRUTE_SIZE;
  }
}

/**
 *
 */
void test_add_corner(fpgaword_t *ctl) {
  const size_t A = 0;
  const size_t B = 1;
  const size_t C = 2;

  size_t m;
  size_t n;

  m = 0;
  n = 0;
  fpga_add_test(m, n, A, B, C, ctl);

  m = 0;
  n = 1;
  fpga_add_test(m, n, A, B, C, ctl);

  m = 1;
  n = 0;
  fpga_add_test(m, n, A, B, C, ctl);

  m = 1;
  n = 1;
  fpga_add_test(m, n, A, B, C, ctl);

  m = 0;
  n = 31;
  fpga_add_test(m, n, A, B, C, ctl);

  m = 31;
  n = 0;
  fpga_add_test(m, n, A, B, C, ctl);

  m = 31;
  n = 31;
  fpga_add_test(m, n, A, B, C, ctl);
}

/**
 *
 */
void test_mul_corner(fpgaword_t *ctl) {
  const size_t A = 0;
  const size_t B = 1;
  const size_t C = 2;
  size_t m, n;

  m = 0;
  n = 0;
  fpga_mul_test(m, n, A, B, C, ctl);

  m = 0;
  n = 1;
  fpga_mul_test(m, n, A, B, C, ctl);

  m = 1;
  n = 0;
  fpga_mul_test(m, n, A, B, C, ctl);

  m = 1;
  n = 1;
  fpga_mul_test(m, n, A, B, C, ctl);

  m = 0;
  n = 31;
  fpga_mul_test(m, n, A, B, C, ctl);

  m = 31;
  n = 0;
  fpga_mul_test(m, n, A, B, C, ctl);

  m = 31;
  n = 31;
  fpga_mul_test(m, n, A, B, C, ctl);
}

/**
 *
 */
void test_mov_corner(fpgaword_t *ctl) {
  const size_t A = 0;
  const size_t C = 2;
  size_t m, n;

  m = 0;
  n = 0;
  fpga_mov_test(m, n, A, C, ctl);

  m = 0;
  n = 1;
  fpga_mov_test(m, n, A, C, ctl);

  m = 1;
  n = 0;
  fpga_mov_test(m, n, A, C, ctl);

  m = 2;
  n = 2;
  fpga_mov_test(m, n, A, C, ctl);

  m = 2;
  n = 1;
  fpga_mov_test(m, n, A, C, ctl);

  m = 0;
  n = 31;
  fpga_mov_test(m, n, A, C, ctl);

  m = 31;
  n = 0;
  fpga_mov_test(m, n, A, C, ctl);

  m = 31;
  n = 31;
  fpga_mov_test(m, n, A, C, ctl);
}

/**
 *
 */
void test_fpga_corner(fpgaword_t *ctl) {
  test_add_corner(ctl);
  test_dot_corner(ctl);
  test_mul_corner(ctl);
  test_mov_corner(ctl);
}

/**
 *
 */
void rand_generate_mpn(fpgaword_t *m, fpgaword_t *p, fpgaword_t *n) {
  uint32_t tmp = rand();

  *m = tmp & FPGA_MTRX_MAX_INDEX;
  tmp >>= FPGA_MTRX_INDEX_WIDTH;

  *p = tmp & FPGA_MTRX_MAX_INDEX;
  tmp >>= FPGA_MTRX_INDEX_WIDTH;

  *n = tmp & FPGA_MTRX_MAX_INDEX;
  tmp >>= FPGA_MTRX_INDEX_WIDTH;
}

/**
 *
 */
void rand_generate_ABC(fpgaword_t *A, fpgaword_t *B, fpgaword_t *C) {
  uint32_t tmp;

  do {
    tmp = rand();

    *A = tmp & (FPGA_MTRX_BRAMS_CNT - 1);
    tmp >>= FPGA_MTRX_BRAMS_CNT_BITS;

    *B = tmp & (FPGA_MTRX_BRAMS_CNT - 1);
    tmp >>= FPGA_MTRX_BRAMS_CNT_BITS;

    *C = tmp & (FPGA_MTRX_BRAMS_CNT - 1);
    tmp >>= FPGA_MTRX_BRAMS_CNT_BITS;
  } while ((*A == *B) or (*B == *C) or (*C == *A));
}

/**
 *
 */
void test_fpga_rand(fpgaword_t *ctl, size_t turns) {
  fpgaword_t m, p, n, A, B, C;

  while(turns--) {
    rand_generate_mpn(&m, &p, &n);
    rand_generate_ABC(&A, &B, &C);

    uint32_t op = rand();
    switch (op & 3){
    case 0:
      fpga_dot_test(m, p, n, A, B, C, ctl);
      break;
    case 1:
      fpga_add_test(m,    n, A, B, C, ctl);
      break;
    case 2:
      fpga_mul_test(m,    n, A, B, C, ctl);
      break;
    case 3:
      fpga_mov_test(m,    n, A,    C, ctl);
      break;
    default:
      osalSysHalt("");
      break;
    }
  }
}

/**
 *
 */
void fill_rand_all(fpgaword_t m, fpgaword_t n) {
  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    manual_fill_rand(mtrx_pool[i], m, n);
    manual_fill_copy(fpga_pool[i], mtrx_pool[i], m, n);
  }
}

/**
 *
 */
void compare_exact_all(fpgaword_t m, fpgaword_t n) {
  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    mtrx_compare_exact(mtrx_pool[i], fpga_pool[i], m, n);
  }
}

/**
 *
 */
void multispep_add(fpgaword_t m, fpgaword_t n, fpgaword_t *ctl, size_t steps) {
  fpgaword_t A, B, C;

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_add(m, n, A, B, C);
    fpga_mtrx_add(m, n, A, B, C, ctl);
  }
}

/**
 *
 */
void multispep_sub(fpgaword_t m, fpgaword_t n, fpgaword_t *ctl, size_t steps) {
  fpgaword_t A, B, C;

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_sub(m, n, A, B, C);
    fpga_mtrx_sub(m, n, A, B, C, ctl);
  }
}

/**
 *
 */
void multispep_mul(fpgaword_t m, fpgaword_t n, fpgaword_t *ctl, size_t steps) {
  fpgaword_t A, B, C;

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_mul(m, n, A, B, C);
    fpga_mtrx_mul(m, n, A, B, C, ctl);
  }
}

/**
 *
 */
void multispep_scale(fpgaword_t m, fpgaword_t n, fpgaword_t *ctl, size_t steps) {
  fpgaword_t A, B, C;

  double scale = rand_double();

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_scale(m, n, A, C, scale);
    fpga_mtrx_scale(m, n, A, C, ctl, scale);
  }
}

/**
 *
 */
void multispep_eye(fpgaword_t m, fpgaword_t *ctl, size_t steps) {
  fpgaword_t A, B, C;

  double scale = rand_double();

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_dia(m, C, scale);
    fpga_mtrx_dia(m, C, ctl, scale);
  }
}

/**
 *
 */
void multispep_set(fpgaword_t m, fpgaword_t n, fpgaword_t *ctl, size_t steps) {
  fpgaword_t A, B, C;

  double set_val = rand_double();

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_set(m, n, C, set_val);
    fpga_mtrx_set(m, n, C, ctl, set_val);
  }
}

/**
 *
 */
void multispep_cpy(fpgaword_t m, fpgaword_t n, fpgaword_t *ctl, size_t steps) {
  fpgaword_t A, B, C;

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_cpy(m, n, A, C);
    fpga_mtrx_cpy(m, n, A, C, ctl);
  }
}

/**
 *
 */
void multispep_trn(fpgaword_t m, fpgaword_t n, fpgaword_t *ctl, size_t steps) {
  fpgaword_t A, B, C;

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_trn(m, n, A, C);
    fpga_mtrx_trn(m, n, A, C, ctl);
  }
}

/**
 *
 */
void multispep_dot(fpgaword_t m, fpgaword_t p, fpgaword_t n, fpgaword_t *ctl, size_t steps) {
  fpgaword_t A, B, C;

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_dot(m, p, n, A, B, C);
    fpga_mtrx_dot(m, p, n, A, B, C, ctl);
  }
}

/**
 *
 */
void _test_fpga_memory_isolation_math(fpgaword_t m, fpgaword_t p, fpgaword_t n, fpgaword_t *ctl) {

  fill_rand_all(m, n);
  multispep_add(m, n, ctl, 17);
  compare_exact_all(m, n);

  multispep_sub(m, n, ctl, 17);
  compare_exact_all(m, n);

  multispep_eye(m, ctl, 3);
  compare_exact_all(m, n);

  fill_rand_all(m, n);
  multispep_mul(m, n, ctl, 7);
  compare_exact_all(m, n);

  multispep_cpy(m, n, ctl, 3);
  compare_exact_all(m, n);

  fill_rand_all(m, n);
  multispep_trn(m, n, ctl, 3);
  compare_exact_all(m, n);

  fill_rand_all(m, n);
  multispep_dot(m, p, n, ctl, 3);
  compare_exact_all(m, n);

  multispep_set(m, n, ctl, 3);
  compare_exact_all(m, n);

  fill_rand_all(m, n);
  multispep_scale(m, n, ctl, 3);
  compare_exact_all(m, n);
}

/**
 *
 */
void test_fpga_memory_isolation_math(fpgaword_t *ctl) {
  fpgaword_t m, p, n;
  m = 31;
  p = 31;
  n = 31;

  _test_fpga_memory_isolation_math(m, p, n, ctl);

  for (size_t i=0; i<10; i++) {
    rand_generate_mpn(&m, &p, &n);
    _test_fpga_memory_isolation_math(m, p, n, ctl);
  }
}

/**
 *
 */
void test_fpga_memory_isolation(void) {
  fpgaword_t m, n;
  m = 31;
  n = 31;

  // forward fill
  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    manual_fill_rand(mtrx_pool[i], m, n);
    manual_fill_copy(fpga_pool[i], mtrx_pool[i], m, n);
  }

  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    mtrx_compare_exact(mtrx_pool[i], fpga_pool[i], m, n);
  }

  // backward fill
  for (size_t i=(FPGA_MTRX_BRAMS_CNT-1); i>0; i--) {
    manual_fill_rand(mtrx_pool[i], m, n);
    manual_fill_copy(fpga_pool[i], mtrx_pool[i], m, n);
  }

  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    mtrx_compare_exact(mtrx_pool[i], fpga_pool[i], m, n);
  }
}

time_measurement_t ram2fpga_tm;
time_measurement_t fpga2ram_tm;
time_measurement_t inversion_ram_tm;
time_measurement_t inversion_fpga_tm;

double ram2fpga_eps;
double fpga2ram_eps;
double inversion_ram_eps;
double inversion_fpga_eps;

/**
 * @brief   Converts time measurement value to elements per second.
 */
double benchmark_tm2eps(const time_measurement_t *tm, size_t m, size_t n) {
  double time_sec = tm->last;
  time_sec /= STM32_SYSCLK;
  double ret = m*n;
  return ret / time_sec;
}

/**
 *
 */
void benchmark(size_t idx, size_t m, size_t n) {
  int inv_status = 0;

  chTMObjectInit(&ram2fpga_tm);
  chTMObjectInit(&fpga2ram_tm);
  chTMObjectInit(&inversion_ram_tm);
  chTMObjectInit(&inversion_fpga_tm);

  /**/
  chTMStartMeasurementX(&ram2fpga_tm);
  for (size_t i=0; i<m*n; i++) {
    fpga_pool[idx][i] = mtrx_pool[idx][i];
  }
  chTMStopMeasurementX(&ram2fpga_tm);
  ram2fpga_eps = benchmark_tm2eps(&ram2fpga_tm, m, n);

  /**/
  chTMStartMeasurementX(&fpga2ram_tm);
  for (size_t i=0; i<m*n; i++) {
    mtrx_pool[idx][i] = fpga_pool[idx][i];
  }
  chTMStopMeasurementX(&fpga2ram_tm);
  fpga2ram_eps = benchmark_tm2eps(&fpga2ram_tm, m, n);

  /**/
  inv_status = 0;
  while (0 == inv_status) {
    for (size_t i=0; i<m*n; i++) {
      double tmp = rand_double();
      fpga_pool[idx][i] = tmp;
      mtrx_pool[idx][i] = tmp;
    }

    chTMStartMeasurementX(&inversion_fpga_tm);
    inv_status = matrix::matrix_soft_inverse(m*n, fpga_pool[idx]);
    chTMStopMeasurementX(&inversion_fpga_tm);
  }
  inversion_fpga_eps = benchmark_tm2eps(&inversion_fpga_tm, m, n);

  /**/
  inv_status = 0;
  while (0 == inv_status) {
    for (size_t i=0; i<m*n; i++) {
      //mtrx_pool[idx][i] = rand_double();
    }

    chTMStartMeasurementX(&inversion_ram_tm);
    inv_status = matrix::matrix_soft_inverse(m*n, mtrx_pool[idx]);
    chTMStopMeasurementX(&inversion_ram_tm);
  }
  inversion_ram_eps = benchmark_tm2eps(&inversion_ram_tm, m, n);
}




/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
volatile fpgaword_t tmp;
volatile double distance;
void fpga_mul_test(FPGADriver *fpgap, size_t turns) {
  chTMObjectInit(&tmu_soft);
  chTMObjectInit(&tmu_hard);

  osalDbgCheck(fpgap->state == FPGA_READY);

  // memtest on brams
  for (size_t i=0; i<8; i++) {
    size_t depth = (1 << FPGA_MTRX_INDEX_WIDTH) * (1 << FPGA_MTRX_INDEX_WIDTH);
    depth *= sizeof(double);
    depth /= sizeof(fpgaword_t);

    fpga_memtest(&FPGAD1, false, 2, FPGA_WB_SLICE_MUL_BUF0 + i, depth);
  }

  // benchmarking
  //benchmark(1, 16, 16);

  // now math test
  for (size_t i=0; i<8; i++) {
    fpga_pool[i] = (double *)fpgaGetSlicePtr(fpgap, FPGA_WB_SLICE_MUL_BUF0 + i);
  }

  fpgaword_t *ctl = fpgaGetSlicePtr(fpgap, FPGA_WB_SLICE_MUL_CTL);

  FPGAMathRst(true);
  osalThreadSleepMilliseconds(100);
  FPGAMathRst(false);

  while(turns--) {
    osalThreadSleep(1);

    srand(chSysGetRealtimeCounterX());
    init_rand_pool();

    test_fpga_rand(ctl, 200);
    test_fpga_corner(ctl);
    test_fpga_memory_isolation();
    test_fpga_memory_isolation_math(ctl);

    green_led_toggle();
  }
}



