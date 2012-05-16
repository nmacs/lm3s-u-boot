#!/bin/sh


ROOT=`pwd`
TOOLCHAIN_BIN="$ROOT/../toolchain/arm-2011.03/bin"
export PATH="$TOOLCHAIN_BIN:$PATH"

make CROSS_COMPILE=arm-none-linux-gnueabi- clean

make uwic_config || exit 1
make -j 1 CROSS_COMPILE=arm-none-linux-gnueabi- ARCH=arm u-boot.bin || exit 1

echo == Compiled ==
