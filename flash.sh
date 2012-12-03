#!/bin/bash

root=`pwd`
image="$root/u-boot.bin"
rom_disk="$root/rom_disk.cramfs"
openocd -f "$root/board/elster/uwic/uwic-jtag.cfg" -c "flash_mcu $image $rom_disk" || exit 1

echo "== Flashed =="
