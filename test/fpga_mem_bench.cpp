#include <cmath>

#include "main.h"
#include "pads.h"

#include "fpga_mul.h"
#include "fpga_mem_bench.hpp"

using namespace chibios_rt;

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

static const size_t sram_size = FPGA_MTRX_SIZE * FPGA_MTRX_CNT * sizeof(double);

time_measurement_t mem_tmu_w;
time_measurement_t mem_tmu_r;

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */

/*
 *
 */
static void membench(FPGAMemorySpace *memspace) {

  chTMObjectInit(&mem_tmu_r);
  chTMObjectInit(&mem_tmu_w);
  double *ptr = memspace->mtrx;

  double pattern = 1.1;
  for (size_t i=0; i<33*33; i++) {
    test_buf_mtrx[i] = pattern;
    pattern += pattern;
  }

  chTMStartMeasurementX(&mem_tmu_w);
  for (size_t i=0; i<1000; i++) {
    for (size_t i=0; i<33*33; i++) {
      ptr[i] = test_buf_mtrx[i];
    }
    //memcpy(memtest_struct.start, test_buf_mtrx, sizeof(test_buf_mtrx));
  }
  chTMStopMeasurementX(&mem_tmu_w);

  chTMStartMeasurementX(&mem_tmu_r);
  for (size_t i=0; i<1000; i++) {
    for (size_t i=0; i<33*33; i++) {
      test_buf_mtrx[i] = ptr[i];
    }
    //memcpy(test_buf_mtrx, memtest_struct.start, sizeof(test_buf_mtrx));
  }
  chTMStopMeasurementX(&mem_tmu_r);
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
void fpga_mem_bench(FPGAMemorySpace *memspace) {

  membench(memspace);
}






