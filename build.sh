#!/bin/sh


ROOT=`pwd`
TOOLCHAIN_BIN="$ROOT/../toolchain/uclinux/arm-2011.03/bin"
export PATH="$TOOLCHAIN_BIN:$PATH"

make CROSS_COMPILE=arm-uclinuxeabi- ARCH=arm clean

make CROSS_COMPILE=arm-uclinuxeabi- uwic_config || exit 1
make -j 4 CROSS_COMPILE=arm-uclinuxeabi- ARCH=arm u-boot.bin || exit 1

size $ROOT/u-boot

mkfs.cramfs "$ROOT/rom_disk" "$ROOT/rom_disk.cramfs" || exit1

echo "== Compiled =="

if [ "$1" = "flash" ]; then
  $ROOT/flash.sh
fi
