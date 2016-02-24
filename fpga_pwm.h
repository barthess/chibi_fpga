#ifndef FPGA_PWM_H_
#define FPGA_PWM_H_

#include "fpga.h"

#define FPGA_PWM_CHANNELS        16
#define FPGA_ICU_CHANNELS        16

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  FPGAPWM_UNINIT  = 0,             /**< Not initialized.                   */
  FPGAPWM_STOP    = 1,             /**< Stopped.                           */
  FPGAPWM_READY   = 2,             /**< Ready.                             */
} fpgapwmstate_t;

/**
 * @brief   Forward declaration.
 */
typedef struct FpgaPwm FpgaPwm;

/**
 * @brief   Structure.
 */
struct FpgaPwm {
  fpgaword_t        *pwm;
  const fpgaword_t  *icu;
  const fpgaword_t  *speedometer;
  const uint32_t    *odometer;
  fpgapwmstate_t    state;
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
  void fpgaPwmObjectInit(FpgaPwm *pwmp);
  void fpgaPwmStart(FpgaPwm *pwmp, const FPGADriver *fpgap);
  void fpgaPwmStop(FpgaPwm *pwmp);
  void fpgaPwmSet(FpgaPwm *pwmp, fpgaword_t val, size_t N);
#ifdef __cplusplus
}
#endif

#endif /* FPGA_PWM_H_ */
