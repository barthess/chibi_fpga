#ifndef FPGA_ICU_H_
#define FPGA_ICU_H_

#include "fpga.h"

#define FPGA_ICU_CHANNELS        16

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  FPGAICU_UNINIT = 0,             /**< Not initialized.                   */
  FPGAICU_STOP = 1,               /**< Stopped.                           */
  FPGAICU_READY = 2,              /**< Ready.                             */
} fpgaicustate_t;

/**
 * @brief   Forward declaration.
 */
typedef struct FpgaIcu FpgaIcu;

/**
 * @brief   Structure.
 */
struct FpgaIcu {
  fpgaword_t      *icu;
  fpgaicustate_t  state;
};

/**
 *
 */
extern FpgaIcu FPGAICUD1;

/**
 *
 */
#ifdef __cplusplus
extern "C" {
#endif
  void fpgaicuObjectInit(FpgaIcu *icup);
  void fpgaicuStart(FpgaIcu *icup, const FPGADriver *fpgap);
  void fpgaicuStop(FpgaIcu *icup);
  fpgaword_t fpgaicuRead(const FpgaIcu *icup, size_t N);
#ifdef __cplusplus
}
#endif

#endif /* FPGA_ICU_H_ */
