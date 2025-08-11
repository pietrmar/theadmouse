#!/usr/bin/env sh

elf_path="$1"

if [ -z "$elf_path" ]; then
	echo "Usage: $0 <firmware.elf>"
	exit 1
fi

if [ ! -f "$elf_path" ]; then
	echo "Error: File '$elf_path' not found"
	exit 1
fi

echo "Flashing '$elf_path' ..."

arm-none-eabi-gdb "$elf_path" \
	-nx \
	-batch \
	-ex 'target extended-remote /dev/ttyACM0' \
	-ex 'monitor frequency 4M' \
	-ex 'monitor swdp_scan' \
	-ex 'attach 1' \
	-ex 'load' \
	-ex 'quit'
