#!/bin/bash

root=`pwd`
image="$root/u-boot.bin"
openocd -f "$root/jtag/uwic-flash.cfg" -c "program_device $image" || exit 1

echo "== Flashed =="
