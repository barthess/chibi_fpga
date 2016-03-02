#ifndef FPGA_MTRX_H_
#define FPGA_MTRX_H_

#include "fpga.h"

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  MTRX_UNINIT  = 0,            /**< Not initialized.                   */
  MTRX_STOP    = 1,            /**< Stopped.                           */
  MTRX_READY   = 2,            /**< Ready.                             */
  MTRX_ACTIVE  = 3,            /**< Active.                            */
} mtrxstate_t;

/**
 * @brief   Forward declaration.
 */
typedef struct MtrxMath MtrxMath;

/**
 * @brief   Structure handling matrix multiplier.
 */
struct MtrxMath {
  /**
   * @brief   Pointer to FPGA driver.
   */
  const FPGADriver  *fpgap;
  /**
   * @brief   Operation word pointer.
   */
  fpgaword_t        *op;
  /**
   * @brief   Sizes word pointer.
   */
  fpgaword_t        *sizes;
  /**
   * @brief   Scale/memset value.
   */
  double            *constant;
  /**
   * @brief   Pool for matrix data.
   */
  double            *pool[FPGA_MTRX_BRAMS_CNT];
  /**
   * @brief   Bitmask for free matrix regions.
   */
  uint32_t          empty;
  /**
   * @brief   Engine state.
   */
  mtrxstate_t       state;
};

/**
 *
 */
extern MtrxMath MTRXD;

/**
 *
 */
#ifdef __cplusplus
extern "C" {
#endif
  void fpgaMtrxObjectInit(MtrxMath *mtrxp);
  void fpgaMtrxStart(MtrxMath *mtrxp, const FPGADriver *fpgap);
  void fpgaMtrxStop(MtrxMath *mtrxp);
  double* fpgaMtrxMalloc(void);
  void fpgaMtrxFree(void *slice);

  void fpgaMtrxDot(size_t m, size_t p, size_t n, const double *A, const double *B, double *C, bool b_transposed);
  void fpgaMtrxAdd(size_t m, size_t n, const double *A, const double *B, double *C);
  void fpgaMtrxSub(size_t m, size_t n, const double *A, const double *B, double *C);
  void fpgaMtrxMul(size_t m, size_t n, const double *A, const double *B, double *C);
  void fpgaMtrxScale(size_t m, size_t n, const double *A, double *C, double scale);
  void fpgaMtrxCpy(size_t m, size_t n, const double *A, double *C);
  void fpgaMtrxTrn(size_t m, size_t n, const double *A, double *C);
  void fpgaMtrxSet(size_t m, size_t n, double *C, double val);
  void fpgaMtrxDia(size_t m, double *C, double val);
#ifdef __cplusplus
}
#endif

#endif /* FPGA_MTRX_H_ */
