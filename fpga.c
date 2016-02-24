#include "main.h"
#include "pads.h"

#include "fpga.h"

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

FPGADriver FPGAD1;

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

// sync
//static const SRAMConfig sram_cfg = {
//    (FSMC_BCR_MWID_16 | FSMC_BCR_MTYP_PSRAM | FSMC_BCR_BURSTEN |
//        FSMC_BCR_WREN | FSMC_BCR_CBURSTRW | FSMC_BCR_WAITPOL),
//
//    // BTR
//    (0 << 24) | // DATLAT
//    (1 << 20) | // CLKDIV (0 is not supported, max == 15)
//    (0 << 16),  // BUSTURN
//
//    // BWTR
//    (0 << 24) | // DATLAT
//    (1 << 20) | // CLKDIV (0 is not supported, max == 15)
//    (0 << 16),  // BUSTURN
//};

// async
static const SRAMConfig sram_cfg = {
    (FSMC_BCR_MWID_16 | FSMC_BCR_MTYP_SRAM | FSMC_BCR_WREN | FSMC_BCR_EXTMOD),

    // BTR
    (1 << 16) | // BUSTURN (min = 0) (do not set it to zero for proper FPGA handling)
    (12 << 8)  | // DATAST (min = 1)
    (0 << 0),   // ADDSET (min = 0)

    // BWTR
    (1 << 16) | // BUSTURN (do not set it to zero for proper FPGA handling)
    (4 << 8)  | // DATAST  (for 108MHz FPGA it must be at least 3)
    (0 << 0),   // ADDSET
};

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
void fpgaObjectInit(FPGADriver *fpgap) {
  fpgap->state = FPGA_STOP;
}

/**
 *
 */
void fpgaStart(FPGADriver *fpgap) {

  fsmcSramInit();
  fsmcSramStart(&SRAMD1, &sram_cfg);

  while ( ! FPGAReady()) {
    orange_led_on();
    osalThreadSleepMilliseconds(30);
    orange_led_off();
    osalThreadSleepMilliseconds(70);
  }

  fpgap->memspace = (fpgaword_t *)FSMC_Bank1_1_MAP;
  fpgap->state = FPGA_READY;
}

/**
 *
 */
void fpgaStop(FPGADriver *fpgap) {
  fpgap->state = FPGA_STOP;
  fsmcSramStop(&SRAMD1);
}

/**
 * @brief   Return pointer to specified slice
 */
fpgaword_t * fpgaGetSlicePtr(const FPGADriver *fpgap, size_t N) {

  osalDbgCheck(FPGA_READY == fpgap->state);
  osalDbgCheck(N < FPGA_WB_SLICE_CNT);

  return & fpgap->memspace[N * FPGA_WB_SLICE_SIZE];
}




