#include "main.h"
#include "pads.h"

#include "fpga_pwm.h"

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
FpgaPwm FPGAPWMD1;

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

/*
 *******************************************************************************
 * EXPORTED FUNCTIONS
 *******************************************************************************
 */
/**
 *
 */
void fpgaPwmObjectInit(FpgaPwm *pwmp) {
  pwmp->state = FPGAPWM_STOP;
}

/**
 *
 */
void fpgaPwmStart(FpgaPwm *pwmp, const FPGADriver *fpgap) {

  osalDbgCheck(FPGA_READY == fpgap->state);
  osalDbgCheck(FPGAPWM_UNINIT != pwmp->state);

  if (FPGAPWM_STOP == pwmp->state) {
    pwmp->pwm = fpgaGetSlicePtr(fpgap, FPGA_WB_SLICE_PWM_ICU);
    pwmp->icu = pwmp->pwm + FPGA_PWM_CHANNELS;
    pwmp->speedometer = pwmp->icu + FPGA_ICU_CHANNELS;
    pwmp->odometer = (uint32_t*)pwmp->speedometer + 2;
  }

  pwmp->state = FPGAPWM_READY;
}

/**
 *
 */
void fpgaPwmStop(FpgaPwm *pwmp) {
  pwmp->state = FPGAPWM_STOP;
  pwmp->pwm = NULL;
}
