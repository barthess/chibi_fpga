#ifndef FPGA_MEM_TEST_HPP_
#define FPGA_MEM_TEST_HPP_

#include "fpga.h"

void fpga_memtest(FPGADriver *fpgap, bool fpga_assistance, size_t turns,
                  size_t slice, size_t depth);
void fpga_memtest_oscillo_probe(FPGADriver *fpgap);

#endif /* FPGA_MEM_TEST_HPP_ */

