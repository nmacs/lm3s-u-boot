#!/bin/bash

root=`pwd`
image="$root/u-boot.bin"
openocd -f "$root/../toolchain/jtag/uwic.cfg" -c "flash_mcu $image" || exit 1

echo "== Flashed =="
