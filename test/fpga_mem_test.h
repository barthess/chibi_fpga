#ifndef FPGA_MEM_TEST_H_
#define FPGA_MEM_TEST_H_

#include "fpga.h"

#ifdef __cplusplus
extern "C" {
#endif
  void fpga_memtest(FPGADriver *fpgap, bool fpga_assistance, size_t turns,
                    size_t slice, size_t depth);
  void fpga_memtest_oscillo_probe(FPGADriver *fpgap);
#ifdef __cplusplus
}
#endif

#endif /* FPGA_MEM_TEST_H_ */

