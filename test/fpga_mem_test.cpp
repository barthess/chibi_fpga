#include <cmath>

#include "main.h"
#include "pads.h"

#include "memtest.h"
#include "fpga_mem_test.hpp"

using namespace chibios_rt;

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
//#define BRAM_DEPTH    (32768 * 1)
#define BRAM_WIDTH    2 // width in bytes

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
static void mem_error_cb(memtest_t *memp, testtype type, size_t index,
                         size_t width, uint32_t got, uint32_t expect);
/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

static memtest_t memtest_struct = {
    nullptr,
    0,
    MEMTEST_WIDTH_64 | MEMTEST_WIDTH_32 | MEMTEST_WIDTH_16,
    mem_error_cb
};

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
static void mem_error_cb(memtest_t *memp, testtype type, size_t index,
                          size_t width, uint32_t got, uint32_t expect) {
  (void)memp;
  (void)type;
  (void)index;
  (void)width;
  (void)got;
  (void)expect;

  green_led_off();
  orange_led_off();
  red_led_on();
  osalSysHalt("Memory broken");
}

/*
 *
 */
static void memtest(memtest_t *testp, time_measurement_t *memtest_tmp) {
  chTMStartMeasurementX(memtest_tmp);
  memtest_run(testp, MEMTEST_RUN_ALL);
  chTMStopMeasurementX(memtest_tmp);
}

/*
 *
 */
static void running_lights(memtest_t *testp, time_measurement_t *memtest_tmp) {

  FPGAMemAutoFill(false);
  osalThreadSleep(1);

  memtest(testp, memtest_tmp);
  green_led_toggle();
  memtest(testp, memtest_tmp);
  red_led_toggle();
  memtest(testp, memtest_tmp);
  orange_led_toggle();
}

/*
 *
 */
void chipscope_test(memtest_t *testp, size_t turns) {
  volatile uint16_t *mem_array = (uint16_t *)testp->start;
  uint16_t tmp;

  FPGAMemAutoFill(false);

  while(turns--) {
    osalSysPolledDelayX(10);
    mem_array[0] = 0x55AA;
    mem_array[1] = 0x1111;

    osalSysPolledDelayX(10);
    tmp = mem_array[0];
    tmp = mem_array[1];
  }
  (void)tmp;
}

/*
 *
 */
void fpga_assisted_zero_addr_match(memtest_t *testp) {
  volatile uint16_t *mem_array = (uint16_t *)testp->start;

  FPGAMemAutoFill(false);
  osalThreadSleep(1);

  mem_array[0] = 0x55AA;
  osalThreadSleep(1);
  osalDbgCheck(true == FPGAbramDbgOk());

  mem_array[0] = 0x55AB;
  osalThreadSleep(1);
  osalDbgCheck(false == FPGAbramDbgOk());
}

/*
 *
 */
void fpga_assisted_fpga_own_addr(memtest_t *testp, size_t depth) {
  volatile uint16_t *mem_array = (uint16_t *)testp->start;
  volatile uint16_t read;

  FPGAMemAutoFill(true);
  osalThreadSleepMilliseconds(10); // wait until FPGA fills all BRAM array

  for (size_t i=0; i<depth; i++) {
    read = mem_array[i];
    osalDbgCheck(i == read);
  }
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 *
 */
void fpga_memtest(FPGADriver *fpgap, bool fpga_assistance, size_t turns,
                  size_t slice, size_t depth) {

  osalDbgCheck(fpgap->state == FPGA_READY);

  memtest_struct.start = fpgaGetSlicePtr(fpgap, slice);
  memtest_struct.size = depth * BRAM_WIDTH;

  time_measurement_t memtest_tm;
  chTMObjectInit(&memtest_tm);

  while (turns--) {
    if (fpga_assistance) {
      fpga_assisted_zero_addr_match(&memtest_struct);
      fpga_assisted_fpga_own_addr(&memtest_struct, depth);
    }

    /* general memtest without FPGA participating */
    running_lights(&memtest_struct, &memtest_tm);
  }

  green_led_off();
  red_led_off();
  orange_led_off();
}



typedef uint16_t memword;
volatile memword pattern = 0xFF00;
volatile memword devnull = 0;
void fpga_memtest_oscillo_probe(FPGADriver *fpgap) {
  volatile memword *ptr = (memword *)fpgap->memspace;

  FPGAMemAutoFill(false);
  osalThreadSleep(1);

  while (true) {
    osalSysLock();
    ptr[0] = pattern;
    ptr[1] = ~pattern;
    port_rt_get_counter_value();
    devnull = ptr[0];
    devnull = ptr[1];
    osalSysUnlock();

    osalThreadSleep(1);
  }
}

