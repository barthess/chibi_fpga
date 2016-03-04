#include <cmath>
#include <cstdlib>
#include <cstring>

#include "main.h"
#include "pads.h"

#include "fpga_mtrx_test.hpp"
#include "fpga_mem_test.hpp"

#include "matrix_soft_engine.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

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

static double ram_pool[FPGA_MTRX_BRAMS_CNT]
                       [(1 << FPGA_MTRX_INDEX_WIDTH) * (1 << FPGA_MTRX_INDEX_WIDTH)];

__CCM__ static double rand_pool[RAND_POOL_LEN];

static time_measurement_t sparse_read_tm;
static time_measurement_t dense_read_tm;
static time_measurement_t tm_malloc;
static time_measurement_t tm_free;

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
 * @brief   matrix multiply with B lazy transposed
 */
void matrix_multiply_fpga_like_TB(size_t m, size_t p, size_t n,
                                  const double *A, const double *B, double *C) {
  size_t i, j, k;
  double tmp[p];

  for(i=0; i<m; i++) {      //each row in A
    for(j=0; j<n; j++) {    //each column in B
      for(k=0; k<p; k++) {  //each element in row A & column B
        tmp[k] = A[i*p + k] * B[j*p + k];
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

/*****************************************************************************************
 * Software variants
 *****************************************************************************************/
/**
 *
 */
void soft_mtrx_dot(size_t m, size_t p, size_t n,
                   size_t A, size_t B, size_t C, bool b_transposed) {
  if (b_transposed) {
    matrix_multiply_fpga_like_TB(m, p, n, ram_pool[A], ram_pool[B], ram_pool[C]);
  }
  else {
    matrix_multiply_fpga_like(m, p, n, ram_pool[A], ram_pool[B], ram_pool[C]);
  }
}

/**
 *
 */
void soft_mtrx_add(size_t m,           size_t n,
                   size_t A, size_t B, size_t C) {
  matrix::matrix_soft_add(m, n, ram_pool[A], ram_pool[B], ram_pool[C]);
}

/**
 *
 */
void soft_mtrx_sub(size_t m,           size_t n,
                   size_t A, size_t B, size_t C) {
  matrix::matrix_soft_sub(m, n, ram_pool[A], ram_pool[B], ram_pool[C]);
}

/**
 *
 */
void soft_mtrx_mul(size_t m,           size_t n,
                   size_t A, size_t B, size_t C) {
  for (size_t i=0; i<m*n; i++) {
    ram_pool[C][i] = ram_pool[A][i] * ram_pool[B][i];
  }
}

/**
 *
 */
void soft_mtrx_scale(size_t m,           size_t n,
                     size_t A,           size_t C,
                     double scale) {
  matrix::matrix_soft_scale(m*n, ram_pool[A], ram_pool[C], scale);
}

/**
 *
 */
void soft_mtrx_cpy(size_t m,           size_t n,
                   size_t A,           size_t C) {
  for (size_t i=0; i<m*n; i++) {
    ram_pool[C][i] = ram_pool[A][i];
  }
}

/**
 *
 */
void soft_mtrx_set(size_t m,           size_t n,
                                       size_t C,
                   double set_val) {
  for (size_t i=0; i<m*n; i++) {
    ram_pool[C][i] = set_val;
  }
}

/**
 *
 */
void soft_mtrx_trn(size_t m,           size_t n,
                   size_t A,           size_t C) {
  matrix::matrix_soft_transpose(m, n, ram_pool[A], ram_pool[C]);
}

/**
 *
 */
void soft_mtrx_dia(size_t m,
                                       size_t C,
                   double set_val) {
  size_t i = 0;

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
    ram_pool[C][i] = 0;
  }

  i = 0;
  while (i < m*m) {
    ram_pool[C][i] = set_val;
    i += m+1;
  }
}


/*******************************************************************************
 * manual fill of arrays in FPGA
 *******************************************************************************/
void manual_fill_pattern(double *dst, double pattern, bool increment, size_t m, size_t n) {
  for (size_t i=0; i<m*n; i++) {
    dst[i] = pattern;
    if (increment) {
      pattern += 1;
    }
  }
}

void manual_fill_copy(double *dst, const double *src, size_t m, size_t n) {
  for (size_t i=0; i<m*n; i++) {
    dst[i] = src[i];
  }
}

void manual_fill_rand(double *dst, size_t m, size_t n) {

  for (size_t i=0; i<m*n; i++) {
    dst[i] = fast_rand_double();
  }
}

void manual_fill_rand_sparse(double *dst, size_t m, size_t n, size_t sparseness) {

  for (size_t i=0; i<m*n; i++) {
    if (0 == (rand() & sparseness)) {
      dst[i] = fast_rand_double();
    }
    else {
      dst[i] = 0;
    }
  }
}

/*******************************************************************************
 * Test wrappers
 *******************************************************************************/

/**
 *
 */
void fpga_dot_test(size_t m, size_t p, size_t n,
                   size_t A, size_t B, size_t C) {

  manual_fill_rand(ram_pool[A], m, p);
  manual_fill_rand(ram_pool[B], p, n);
  manual_fill_copy(fpga_pool[A], ram_pool[A], m, p);
  manual_fill_copy(fpga_pool[B], ram_pool[B], p, n);
  manual_fill_pattern(ram_pool[C], 666, false, m, n);
  manual_fill_copy(fpga_pool[C], ram_pool[C], m, n);

  soft_mtrx_dot(m, p, n, A, B, C, false);
  fpgaMtrxDot(m, p, n, fpga_pool[A], fpga_pool[B], fpga_pool[C], false);

  soft_mtrx_dot(m, p, n, A, B, C, true);
  fpgaMtrxDot(m, p, n, fpga_pool[A], fpga_pool[B], fpga_pool[C], true);

  //mtrx_compare_approx(mtrx_pool[C], fpga_pool[C], m, n);
  mtrx_compare_exact(ram_pool[C], fpga_pool[C], m, n);
}

/**
 *
 */
void fpga_add_test(size_t m,           size_t n,
                   size_t A, size_t B, size_t C) {

  manual_fill_rand(ram_pool[A], m, n);
  manual_fill_rand(ram_pool[B], m, n);
  manual_fill_copy(fpga_pool[A], ram_pool[A], m, n);
  manual_fill_copy(fpga_pool[B], ram_pool[B], m, n);
  soft_mtrx_add(m, n, A, B, C);
  fpgaMtrxAdd(m, n, fpga_pool[A], fpga_pool[B], fpga_pool[C]);
  mtrx_compare_exact(ram_pool[C], fpga_pool[C], m, n);

  manual_fill_rand(ram_pool[A], m, n);
  manual_fill_rand(ram_pool[B], m, n);
  manual_fill_copy(fpga_pool[A], ram_pool[A], m, n);
  manual_fill_copy(fpga_pool[B], ram_pool[B], m, n);
  soft_mtrx_sub(m, n, A, B, C);
  fpgaMtrxSub(m, n, fpga_pool[A], fpga_pool[B], fpga_pool[C]);
  mtrx_compare_exact(ram_pool[C], fpga_pool[C], m, n);
}

/**
 *
 */
void fpga_mul_test(size_t m,           size_t n,
                   size_t A, size_t B, size_t C) {

  double scale = fast_rand_double();

  manual_fill_rand(ram_pool[A], m, n);
  manual_fill_rand(ram_pool[B], m, n);
  manual_fill_copy(fpga_pool[A], ram_pool[A], m, n);
  manual_fill_copy(fpga_pool[B], ram_pool[B], m, n);
  soft_mtrx_scale(m, n, A, C, scale);
  fpgaMtrxScale(m, n, fpga_pool[A], fpga_pool[C], scale);
  mtrx_compare_exact(ram_pool[C], fpga_pool[C], m, n);

  manual_fill_rand(ram_pool[A], m, n);
  manual_fill_rand(ram_pool[B], m, n);
  manual_fill_copy(fpga_pool[A], ram_pool[A], m, n);
  manual_fill_copy(fpga_pool[B], ram_pool[B], m, n);
  soft_mtrx_mul(m, n, A, B, C);
  fpgaMtrxMul(m, n, fpga_pool[A], fpga_pool[B], fpga_pool[C]);
  mtrx_compare_exact(ram_pool[C], fpga_pool[C], m, n);
}

/**
 *
 */
void fpga_mov_test(size_t m,           size_t n,
                   size_t A,           size_t C) {

  double set_val;

  manual_fill_pattern(ram_pool[A], m+n, true, m, n);
  manual_fill_pattern(ram_pool[1], 111, true, m, n);
  manual_fill_pattern(ram_pool[C], -10, true, m, n);
  manual_fill_copy(fpga_pool[A], ram_pool[A], m, n);
  manual_fill_copy(fpga_pool[1], ram_pool[1], m, n);
  manual_fill_copy(fpga_pool[C], ram_pool[C], m, n);
  soft_mtrx_cpy(m, n, A, C);
  fpgaMtrxCpy(m, n, fpga_pool[A], fpga_pool[C]);
  mtrx_compare_exact(ram_pool[C], fpga_pool[C], m, n);

  manual_fill_rand(ram_pool[A], m, n);
  manual_fill_copy(fpga_pool[A], ram_pool[A], m, n);
  manual_fill_rand(ram_pool[C], m, n);
  manual_fill_copy(fpga_pool[C], ram_pool[C], m, n);
  soft_mtrx_trn(m, n, A, C);
  fpgaMtrxTrn(m, n, fpga_pool[A], fpga_pool[C]);
  mtrx_compare_exact(ram_pool[C], fpga_pool[C], m, n);

  manual_fill_rand(ram_pool[A], m, n);
  manual_fill_copy(fpga_pool[A], ram_pool[A], m, n);
  manual_fill_rand(ram_pool[C], m, n);
  manual_fill_copy(fpga_pool[C], ram_pool[C], m, n);
  soft_mtrx_cpy(m, n, A, C);
  fpgaMtrxCpy(m, n, fpga_pool[A], fpga_pool[C]);
  mtrx_compare_exact(ram_pool[C], fpga_pool[C], m, n);

  set_val = fast_rand_double();
  manual_fill_rand(ram_pool[A], m, n);
  manual_fill_copy(fpga_pool[A], ram_pool[A], m, n);
  manual_fill_rand(ram_pool[C], m, n);
  manual_fill_copy(fpga_pool[C], ram_pool[C], m, n);
  soft_mtrx_set(m, n, C, set_val);
  fpgaMtrxSet(m, n, fpga_pool[C], set_val);
  mtrx_compare_exact(ram_pool[C], fpga_pool[C], m, n);

  // Eye works correctly only with square matrices
  if (m == n) {
    set_val = fast_rand_double();
    manual_fill_rand(ram_pool[C], m, n);
    manual_fill_copy(fpga_pool[C], ram_pool[A], m, n);
    soft_mtrx_dia(m, C, set_val);
    fpgaMtrxDia(m, fpga_pool[C], set_val);
    mtrx_compare_exact(ram_pool[C], fpga_pool[C], m, n);
  }
}

/**
 *
 */
void test_dot_corner(void) {
  const size_t A = 0;
  const size_t B = 1;
  const size_t C = 2;

  size_t m, p, n;

  m = 32;
  p = 32;
  n = 32;
  fpga_dot_test(m, p, n, A, B, C);

  m = 1;
  p = 1;
  n = 1;
  fpga_dot_test(m, p, n, A, B, C);

  m = 32;
  p = 1;
  n = 1;
  fpga_dot_test(m, p, n, A, B, C);

  m = 1;
  p = 32;
  n = 1;
  fpga_dot_test(m, p, n, A, B, C);

  m = 1;
  p = 1;
  n = 32;
  fpga_dot_test(m, p, n, A, B, C);

  m = 32;
  p = 32;
  n = 1;
  fpga_dot_test(m, p, n, A, B, C);

  m = 1;
  p = 32;
  n = 32;
  fpga_dot_test(m, p, n, A, B, C);

  m = 32;
  p = 1;
  n = 32;
  fpga_dot_test(m, p, n, A, B, C);

  const size_t MAX_BRUTE_SIZE = 5;
  m = MAX_BRUTE_SIZE;
  p = MAX_BRUTE_SIZE;
  n = MAX_BRUTE_SIZE;

  while(m > 1) {
    while(p > 1) {
      while(n > 1) {
        fpga_dot_test(m, p, n, A, B, C);
        n--;
      }
      n = MAX_BRUTE_SIZE;
      p--;
    }
    p = MAX_BRUTE_SIZE;
    m--;
  }
}

/**
 *
 */
void test_add_corner() {
  const size_t A = 0;
  const size_t B = 1;
  const size_t C = 2;

  size_t m;
  size_t n;

  m = 1;
  n = 1;
  fpga_add_test(m, n, A, B, C);

  m = 1;
  n = 2;
  fpga_add_test(m, n, A, B, C);

  m = 2;
  n = 1;
  fpga_add_test(m, n, A, B, C);

  m = 2;
  n = 2;
  fpga_add_test(m, n, A, B, C);

  m = 1;
  n = 32;
  fpga_add_test(m, n, A, B, C);

  m = 32;
  n = 1;
  fpga_add_test(m, n, A, B, C);

  m = 32;
  n = 32;
  fpga_add_test(m, n, A, B, C);
}

/**
 *
 */
void test_mul_corner(void) {
  const size_t A = 0;
  const size_t B = 1;
  const size_t C = 2;
  size_t m, n;

  m = 1;
  n = 1;
  fpga_mul_test(m, n, A, B, C);

  m = 1;
  n = 2;
  fpga_mul_test(m, n, A, B, C);

  m = 2;
  n = 1;
  fpga_mul_test(m, n, A, B, C);

  m = 2;
  n = 2;
  fpga_mul_test(m, n, A, B, C);

  m = 1;
  n = 32;
  fpga_mul_test(m, n, A, B, C);

  m = 32;
  n = 1;
  fpga_mul_test(m, n, A, B, C);

  m = 32;
  n = 32;
  fpga_mul_test(m, n, A, B, C);
}

/**
 *
 */
void test_mov_corner(void) {
  const size_t A = 0;
  const size_t C = 2;
  size_t m, n;

  m = 1;
  n = 1;
  fpga_mov_test(m, n, A, C);

  m = 1;
  n = 2;
  fpga_mov_test(m, n, A, C);

  m = 2;
  n = 1;
  fpga_mov_test(m, n, A, C);

  m = 3;
  n = 3;
  fpga_mov_test(m, n, A, C);

  m = 3;
  n = 2;
  fpga_mov_test(m, n, A, C);

  m = 1;
  n = 32;
  fpga_mov_test(m, n, A, C);

  m = 32;
  n = 1;
  fpga_mov_test(m, n, A, C);

  m = 32;
  n = 32;
  fpga_mov_test(m, n, A, C);
}

/**
 *
 */
void test_fpga_corner(void) {
  test_add_corner();
  test_dot_corner();
  test_mul_corner();
  test_mov_corner();
}

/**
 *
 */
void rand_generate_mpn(fpgaword_t *m, fpgaword_t *p, fpgaword_t *n) {
  uint32_t tmp = rand();

  *m = (tmp & FPGA_MTRX_MAX_INDEX) + 1;
  tmp >>= FPGA_MTRX_INDEX_WIDTH;

  *p = (tmp & FPGA_MTRX_MAX_INDEX) + 1;
  tmp >>= FPGA_MTRX_INDEX_WIDTH;

  *n = (tmp & FPGA_MTRX_MAX_INDEX) + 1;
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
void test_sparce_memcpy(size_t turns) {
  fpgaword_t m, p, n, A, B, C;

  while(turns--) {
    // test sparse copy from FPGA to RAM
    rand_generate_mpn(&m, &p, &n);
    rand_generate_ABC(&A, &B, &C);

    manual_fill_rand_sparse(fpga_pool[A], m, n, 3);
    fpgaMtrxSet(m, n, fpga_pool[B], 1);

    manual_fill_copy(ram_pool[A], fpga_pool[A], m, n);
    manual_fill_copy(ram_pool[B], fpga_pool[B], m, n);

    soft_mtrx_mul(m, n, A, B, C);
    fpgaMtrxMul(m, n, fpga_pool[A], fpga_pool[B], fpga_pool[C]);
    mtrx_compare_exact(ram_pool[C], fpga_pool[C], m, n);

    fpgaMtrxMemcpySparse(m, n, fpga_pool[C], ram_pool[A]);
    mtrx_compare_exact(ram_pool[A], fpga_pool[C], m, n);

    // test sparse copy from RAM to FPGA
    rand_generate_mpn(&m, &p, &n);
    rand_generate_ABC(&A, &B, &C);

    manual_fill_rand_sparse(ram_pool[A], m, n, 3);
    fpgaMtrxMemcpySparse(m, n, ram_pool[A], fpga_pool[A]);
    mtrx_compare_exact(ram_pool[A], fpga_pool[A], m, n);
  }
}

/**
 *
 */
void test_fpga_rand(size_t turns) {
  fpgaword_t m, p, n, A, B, C;

  while(turns--) {
    rand_generate_mpn(&m, &p, &n);
    rand_generate_ABC(&A, &B, &C);

    uint32_t op = rand();
    switch (op & 3){
    case 0:
      fpga_dot_test(m, p, n, A, B, C);
      break;
    case 1:
      fpga_add_test(m,    n, A, B, C);
      break;
    case 2:
      fpga_mul_test(m,    n, A, B, C);
      break;
    case 3:
      fpga_mov_test(m,    n, A,    C);
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
void test_fpga_malloc(size_t turns) {
  double *test_pool[FPGA_MTRX_BRAMS_CNT];
  chTMObjectInit(&tm_malloc);
  chTMObjectInit(&tm_free);

  while(turns--) {
    for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
      chTMStartMeasurementX(&tm_malloc);
      test_pool[i] = fpgaMtrxMalloc();
      chTMStopMeasurementX(&tm_malloc);
      osalDbgCheck(nullptr != test_pool[i]);
    }

    // Malloc now must return NULL because of all BRAM exhausted.
    double *null_val = fpgaMtrxMalloc();
    osalDbgCheck(nullptr == null_val);

    for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
      chTMStartMeasurementX(&tm_free);
      fpgaMtrxFree(test_pool[i]);
      chTMStopMeasurementX(&tm_free);
    }
  }
  return;
}

/**
 *
 */
void fill_rand_all(fpgaword_t m, fpgaword_t n) {
  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    manual_fill_rand(ram_pool[i], m, n);
    manual_fill_copy(fpga_pool[i], ram_pool[i], m, n);
  }
}

/**
 *
 */
void compare_exact_all(fpgaword_t m, fpgaword_t n) {
  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    mtrx_compare_exact(ram_pool[i], fpga_pool[i], m, n);
  }
}

/**
 *
 */
void multispep_add(fpgaword_t m, fpgaword_t n, size_t steps) {
  fpgaword_t A, B, C;

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_add(m, n, A, B, C);
    fpgaMtrxAdd(m, n, fpga_pool[A], fpga_pool[B], fpga_pool[C]);
  }
}

/**
 *
 */
void multispep_sub(fpgaword_t m, fpgaword_t n, size_t steps) {
  fpgaword_t A, B, C;

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_sub(m, n, A, B, C);
    fpgaMtrxSub(m, n, fpga_pool[A], fpga_pool[B], fpga_pool[C]);
  }
}

/**
 *
 */
void multispep_mul(fpgaword_t m, fpgaword_t n, size_t steps) {
  fpgaword_t A, B, C;

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_mul(m, n, A, B, C);
    fpgaMtrxMul(m, n, fpga_pool[A], fpga_pool[B], fpga_pool[C]);
  }
}

/**
 *
 */
void multispep_scale(fpgaword_t m, fpgaword_t n, size_t steps) {
  fpgaword_t A, B, C;

  double scale = rand_double();

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_scale(m, n, A, C, scale);
    fpgaMtrxScale(m, n, fpga_pool[A], fpga_pool[C], scale);
  }
}

/**
 *
 */
void multispep_eye(fpgaword_t m, size_t steps) {
  fpgaword_t A, B, C;

  double scale = rand_double();

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_dia(m, C, scale);
    fpgaMtrxDia(m, fpga_pool[C], scale);
  }
}

/**
 *
 */
void multispep_set(fpgaword_t m, fpgaword_t n, size_t steps) {
  fpgaword_t A, B, C;

  double set_val = rand_double();

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_set(m, n, C, set_val);
    fpgaMtrxSet(m, n, fpga_pool[C], set_val);
  }
}

/**
 *
 */
void multispep_cpy(fpgaword_t m, fpgaword_t n, size_t steps) {
  fpgaword_t A, B, C;

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_cpy(m, n, A, C);
    fpgaMtrxCpy(m, n, fpga_pool[A], fpga_pool[C]);
  }
}

/**
 *
 */
void multispep_trn(fpgaword_t m, fpgaword_t n, size_t steps) {
  fpgaword_t A, B, C;

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_trn(m, n, A, C);
    fpgaMtrxTrn(m, n, fpga_pool[A], fpga_pool[C]);
  }
}

/**
 *
 */
void multispep_dot(fpgaword_t m, fpgaword_t p, fpgaword_t n, size_t steps, bool b_transposed) {
  fpgaword_t A, B, C;

  while(steps--) {
    rand_generate_ABC(&A, &B, &C);
    soft_mtrx_dot(m, p, n, A, B, C, b_transposed);
    fpgaMtrxDot(m, p, n, fpga_pool[A], fpga_pool[B], fpga_pool[C], b_transposed);
  }
}

/**
 *
 */
void _test_fpga_memory_isolation_math(fpgaword_t m, fpgaword_t p, fpgaword_t n) {

  fill_rand_all(m, n);
  multispep_add(m, n, 17);
  compare_exact_all(m, n);

  multispep_sub(m, n, 17);
  compare_exact_all(m, n);

  multispep_eye(m, 3);
  compare_exact_all(m, n);

  fill_rand_all(m, n);
  multispep_mul(m, n, 7);
  compare_exact_all(m, n);

  multispep_cpy(m, n, 3);
  compare_exact_all(m, n);

  fill_rand_all(m, n);
  multispep_trn(m, n, 3);
  compare_exact_all(m, n);

  fill_rand_all(m, n);
  multispep_dot(m, p, n, 3, false);
  compare_exact_all(m, n);
  multispep_dot(m, p, n, 3, true);
  compare_exact_all(m, n);

  multispep_set(m, n, 3);
  compare_exact_all(m, n);

  fill_rand_all(m, n);
  multispep_scale(m, n, 3);
  compare_exact_all(m, n);
}

/**
 *
 */
void test_fpga_memory_isolation_math(void) {
  fpgaword_t m, p, n;
  m = 32;
  p = 32;
  n = 32;

  _test_fpga_memory_isolation_math(m, p, n);

  for (size_t i=0; i<10; i++) {
    rand_generate_mpn(&m, &p, &n);
    _test_fpga_memory_isolation_math(m, p, n);
  }
}

/**
 *
 */
void test_fpga_memory_isolation(void) {
  fpgaword_t m, n;
  m = 32;
  n = 32;

  // forward fill
  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    manual_fill_rand(ram_pool[i], m, n);
    manual_fill_copy(fpga_pool[i], ram_pool[i], m, n);
  }

  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    mtrx_compare_exact(ram_pool[i], fpga_pool[i], m, n);
  }

  // backward fill
  for (size_t i=(FPGA_MTRX_BRAMS_CNT-1); i>0; i--) {
    manual_fill_rand(ram_pool[i], m, n);
    manual_fill_copy(fpga_pool[i], ram_pool[i], m, n);
  }

  for (size_t i=0; i<FPGA_MTRX_BRAMS_CNT; i++) {
    mtrx_compare_exact(ram_pool[i], fpga_pool[i], m, n);
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
    fpga_pool[idx][i] = ram_pool[idx][i];
  }
  chTMStopMeasurementX(&ram2fpga_tm);
  ram2fpga_eps = benchmark_tm2eps(&ram2fpga_tm, m, n);

  /**/
  chTMStartMeasurementX(&fpga2ram_tm);
  for (size_t i=0; i<m*n; i++) {
    ram_pool[idx][i] = fpga_pool[idx][i];
  }
  chTMStopMeasurementX(&fpga2ram_tm);
  fpga2ram_eps = benchmark_tm2eps(&fpga2ram_tm, m, n);

  /**/
  inv_status = 0;
  while (0 == inv_status) {
    for (size_t i=0; i<m*n; i++) {
      double tmp = rand_double();
      fpga_pool[idx][i] = tmp;
      ram_pool[idx][i] = tmp;
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
    inv_status = matrix::matrix_soft_inverse(m*n, ram_pool[idx]);
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
void fpga_mtrx_mem_test(size_t turns) {
  osalDbgCheck(MTRXD.state == MTRX_READY);

  for (size_t i=0; i<8; i++) {
    size_t depth = (1 << FPGA_MTRX_INDEX_WIDTH) * (1 << FPGA_MTRX_INDEX_WIDTH);
    depth *= sizeof(double);
    depth /= sizeof(fpgaword_t);

    fpga_memtest(&FPGAD1, false, turns, FPGA_WB_SLICE_MUL_BUF0 + i, depth);
  }
}

/**
 *
 */
void fpga_mtrx_full_test(size_t turns) {

  fpga_mtrx_mem_test(2);

  // init BRAM pool
  for (size_t i=0; i<8; i++) {
    fpga_pool[i] = (double *)fpgaGetSlicePtr(MTRXD.fpgap, FPGA_WB_SLICE_MUL_BUF0 + i);
  }

  // reset FPGA
  FPGAMathRst(true);
  osalThreadSleepMilliseconds(10);
  FPGAMathRst(false);

  chTMObjectInit(&sparse_read_tm);
  chTMObjectInit(&dense_read_tm);

  // math tests
  while(turns--) {
    osalThreadSleep(1);

    srand(chSysGetRealtimeCounterX());
    init_rand_pool();

    test_fpga_rand(200);
    test_sparce_memcpy(20);
    test_fpga_corner();
    test_fpga_memory_isolation();
    test_fpga_memory_isolation_math();
    test_fpga_malloc(10);

    green_led_toggle();
  }
}



