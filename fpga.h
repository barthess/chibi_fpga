#ifndef FPGA_H_
#define FPGA_H_

#include "hal_fsmc_sram.h"

#include "fpga_constants.h"

/* bus width between fpga and stm32 */
typedef uint16_t        fpgaword_t;

/* address space size in single wishbone slice in fpga_words */
#define FPGA_WB_SLICE_SIZE        65536

/* total number of wishbone slices */
#define FPGA_WB_SLICE_CNT         16

/* IDs of command slices for differ peripherals */
#define FPGA_WB_SLICE_MEMTEST     0
#define FPGA_WB_SLICE_LED         1

#define FPGA_WB_SLICE_MUL_BUF0    2
#define FPGA_WB_SLICE_MUL_BUF1    3
#define FPGA_WB_SLICE_MUL_BUF2    4
#define FPGA_WB_SLICE_MUL_BUF3    5
#define FPGA_WB_SLICE_MUL_BUF4    6
#define FPGA_WB_SLICE_MUL_BUF5    7
#define FPGA_WB_SLICE_MUL_BUF6    8
#define FPGA_WB_SLICE_MUL_BUF7    9
#define FPGA_WB_SLICE_MUL_CTL     10

#define FPGA_WB_SLICE_PWM_ICU     11
#define FPGA_WB_SLICE_UART        12
#define FPGA_WB_SLICE_RESERVED2   13
#define FPGA_WB_SLICE_RESERVED3   14
#define FPGA_WB_SLICE_RESERVED4   15

/* Matrix engine constants */
#define FPGA_MTRX_INDEX_WIDTH     5 /* number of bits used for matrix indexing */
#define FPGA_MTRX_MAX_INDEX       ((1 << FPGA_MTRX_INDEX_WIDTH) - 1)

#define FPGA_MTRX_BRAMS_CNT_BITS  3 /* number of bits for addressing BRAM slice */
#define FPGA_MTRX_BRAMS_CNT       (1 << FPGA_MTRX_BRAMS_CNT_BITS)

/* offsets in matrix control slice */
#define FPGA_MTRX_CTL_OP_OFFSET        0
#define FPGA_MTRX_CTL_SIZES_OFFSET     1
#define FPGA_MTRX_CTL_CONSTANT_OFFSET  4

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  FPGA_UNINIT = 0,                  /**< Not initialized.                   */
  FPGA_STOP = 1,                    /**< Stopped.                           */
  FPGA_READY = 2,                   /**< Ready.                             */
} fpgastate_t;

/**
 *
 */
typedef struct FPGADriver FPGADriver;

/**
 * @brief   Structure handling matrix multiplier.
 */
struct FPGADriver {
  fpgaword_t  *memspace;
  fpgastate_t state;
};

/**
 *
 */
extern FPGADriver FPGAD1;

/**
 *
 */
#ifdef __cplusplus
extern "C" {
#endif
  void fpgaObjectInit(FPGADriver *fpgap);
  void fpgaStart(FPGADriver *fpgap);
  void fpgaStop(FPGADriver *fpgap);
  fpgaword_t * fpgaGetSlicePtr(const FPGADriver *fpgap, size_t N);
#ifdef __cplusplus
}
#endif

#endif /* FPGA_H_ */


