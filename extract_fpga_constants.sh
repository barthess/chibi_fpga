#!/bin/bash

#SRC=../xilinx/fsmc2wb/rtl/mtrx_math_constants.vhd
#DST=./src/fpga/fpga_constants.h

SRC=$1
DST=$2

echo "// WARNING! Automatically generated from $SRC" > $DST
echo "// Do not modify it manually!" >> $DST

grep -E "MATH_OP_|CMD_BIT_" $SRC |\
	sed 's/^.[ ]constant/#define/' |\
	sed 's/:[ ]*natural[ ]*:=//' |\
	sed 's/;.*$//' >> $DST

