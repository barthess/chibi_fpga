#ifndef FPGA_PWM_H_
#define FPGA_PWM_H_

#include "fpga.h"

#define FPGA_PWM_CHANNELS        16

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  FPGAPWM_UNINIT = 0,             /**< Not initialized.                   */
  FPGAPWM_STOP = 1,               /**< Stopped.                           */
  FPGAPWM_READY = 2,              /**< Ready.                             */
} fpgapwmstate_t;

/**
 * @brief   Forward declaration.
 */
typedef struct FpgaPwm FpgaPwm;

/**
 * @brief   Structure.
 */
struct FpgaPwm {
  fpgaword_t      *pwm;
  fpgapwmstate_t  state;
};

/**
 *
 */
extern FpgaPwm FPGAPWMD1;

/**
 *
 */
#ifdef __cplusplus
extern "C" {
#endif
  void fpgapwmObjectInit(FpgaPwm *pwmp);
  void fpgapwmStart(FpgaPwm *pwmp, const FPGADriver *fpgap);
  void fpgapwmStop(FpgaPwm *pwmp);
  void fpgapwmSet(FpgaPwm *pwmp, fpgaword_t val, size_t N);
#ifdef __cplusplus
}
#endif

#endif /* FPGA_PWM_H_ */
