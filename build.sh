#!/bin/bash

ROOT=`pwd`
TOOLCHAIN_BIN="$ROOT/../toolchain/uclinux/arm-2011.03/bin"
export PATH="$TOOLCHAIN_BIN:$PATH"

make CROSS_COMPILE=arm-uclinuxeabi- ARCH=arm clean

make CROSS_COMPILE=arm-uclinuxeabi- uwic_config || exit 1
make -j 4 CROSS_COMPILE=arm-uclinuxeabi- ARCH=arm u-boot.bin || exit 1

size $ROOT/u-boot

mkfs.cramfs "$ROOT/rom_disk" "$ROOT/rom_disk.cramfs" || exit1

echo "== Compiled =="

image_size=$[512*1024]
rom_disk_offset=327680
bl_size=$(stat -c%s "$ROOT/u-boot.bin")
rd_size=$(stat -c%s "$ROOT/rom_disk.cramfs")
gap1=$[$rom_disk_offset - $bl_size]
gap2=$[$image_size - $rom_disk_offset - $rd_size]

cat u-boot.bin > image.bin
tr "\000" "\377" < /dev/zero | dd ibs=1 count=$gap1 2> /dev/null >> image.bin
cat rom_disk.cramfs >> image.bin
tr "\000" "\377" < /dev/zero | dd ibs=1 count=$gap2 2> /dev/null >> image.bin

echo "== Image ready =="