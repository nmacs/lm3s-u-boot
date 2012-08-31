#!/bin/bash

root=`pwd`
image="$root/u-boot.bin"
openocd -f "$root/board/elster/uwic/uwic-jtag.cfg" -c "flash_mcu $image" || exit 1

echo "== Flashed =="
