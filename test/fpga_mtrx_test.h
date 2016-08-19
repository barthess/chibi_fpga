#ifndef FPGA_MTRX_TEST_H_
#define FPGA_MTRX_TEST_H_

#include "fpga_mtrx.h"

#ifdef __cplusplus
extern "C" {
#endif
  void fpgamtrxMemTest(size_t turns);
  void fpgamtrxFullTest(size_t turns);
#ifdef __cplusplus
}
#endif

#endif /* FPGA_MTRX_TEST_H_ */

