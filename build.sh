#!/bin/sh


ROOT=`pwd`
TOOLCHAIN_BIN="$ROOT/../toolchain/arm-2011.03/bin"
export PATH=$TOOLCHAIN_BIN:$PATH

#copy firmware
#cp rt73.bin firmware/

make clean
make ea1788_config
make -j 4 CROSS_COMPILE=arm-none-linux-gnueabi- ARCH=arm u-boot.bin

echo == Compiled ==
