#ifndef FPGA_MEM_BENCH_H_
#define FPGA_MEM_BENCH_H_

#include "fpga.h"

#ifdef __cplusplus
extern "C" {
#endif
  void fpga_led_test(FPGADriver *fpgap, size_t turns);
  void fpga_mem_bench(FPGAMemorySpace *memspace);
#ifdef __cplusplus
}
#endif

#endif /* FPGA_MEM_BENCH_H_ */

