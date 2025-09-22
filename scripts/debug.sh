#!/usr/bin/env sh

elf_path=""
enable_rtt=false
enable_tpwr=false
run=false

print_help() {
	echo "Usage: $0 [OPTIONS] <firmware.elf>"
	echo "Load and debug an ELF firmware file using arm-none-eabi-gdb."
	echo ""
	echo "Options:"
	echo "  --run            Run the program instead of breaking on main()."
	echo "  --enable-rtt     Enable RTT (Real-Time Transfer) monitoring."
	echo "  --enable-tpwr    Enable target power, if not self-powered or VDD is not connected"
	echo "  -h, --help       Display this help message and exit."
	echo ""
	echo "Arguments:"
	echo "  firmware.elf     Path to the ELF firmware file to debug."
}

while [ "$#" -gt 0 ]; do
	case "$1" in
		--run)
			run=true
			shift
			;;
		--enable-rtt)
			enable_rtt=true
			shift
			;;
		--enable-tpwr)
			enable_tpwr=true
			shift
			;;
		-h | --help)
			print_help
			exit 0
			;;
		-*)
			echo "Error: Unknown option '$1'" >&2
			print_help
			exit 1
			;;
		*)
			if [ -z "$elf_path" ]; then
				elf_path="$1"
			else
				echo "Error: Too many arguments" >&2
				print_help
				exit 1
			fi
			shift
			;;
	esac
done

if [ -z "$elf_path" ]; then
	echo "Error: Missing <firmware.elf> argument" >&2
	print_help
	exit 1
fi

if [ ! -f "$elf_path" ]; then
	echo "Error: File '$elf_path' not found" >&2
	exit 1
fi

if "$enable_rtt"; then
	rtt_cmd="monitor rtt enable"
	echo "==============================================================================="
	echo "WARNING: RTT may be unstable with certain targets, such as nRF52."
	echo "See this GitHub issue for more information: https://github.com/blackmagic-debug/blackmagic/issues/2089"
	echo "==============================================================================="
else
	rtt_cmd="monitor rtt disable"
fi

if "$enable_tpwr"; then
	tpwr_cmd="monitor tpwr enable"
else
	tpwr_cmd="monitor tpwr disable"
fi

if "$run"; then
	launch_cmd="run"
else
	launch_cmd="start"
fi

echo "Loading '$elf_path' ..."

set -x
arm-none-eabi-gdb "$elf_path" \
	-ex "set pagination off" \
	-ex "set confirm off" \
	-ex "target extended-remote /dev/ttyACM0" \
	-ex "monitor frequency 4M" \
	-ex "monitor rtt poll 32 8 10" \
	-ex "$tpwr_cmd" \
	-ex "$rtt_cmd" \
	-ex "monitor swdp_scan" \
	-ex "attach 1" \
	-ex "load" \
	-ex "$launch_cmd" \
	-ex "set confirm on"

