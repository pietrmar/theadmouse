#!/usr/bin/env sh

# Helprscript to poll for /dev/ttyACM* devices and check if their USB VID and PID
# matches 2fe3 and 0100 respectively. If such a device was found start `screen` on
# it with 115200 baud.
# and if

usb_vid="2fe3"
usb_pid="0100"

prefix="/dev/ttyACM"
baudrate="115200"

# Make sure to fix up the terminal after exiting
trap "stty sane" EXIT

usb_vid=$(echo "$usb_vid" | tr '[:upper:]' '[:lower:]')
usb_pid=$(echo "$usb_pid" | tr '[:upper:]' '[:lower:]')

echo "Waiting for ${prefix}* devices with vendor id: $usb_vid and product id: $usb_pid"
while true; do
	last=""

	devices=$(ls /dev/ttyACM* 2>/dev/null)
	for dev in $devices; do
		dev=$(basename "$dev")
		dev_usb_vid=$(tr '[:upper:]' '[:lower:]' < /sys/class/tty/"$dev"/device/../idVendor)
		dev_usb_pid=$(tr '[:upper:]' '[:lower:]' < /sys/class/tty/"$dev"/device/../idProduct)

		if [ "$dev_usb_vid" = "$usb_vid" ] && [ "$dev_usb_pid" = "$usb_pid" ]; then
			last="/dev/$dev"
		fi
	done

	if [ -n "$last" ]; then
		echo "Found $last, starting screen"
		screen "$last" "$baudrate"
		# Fix up the terminal after screen exits
		stty sane
		echo ""
		echo "Finished screen session"
	fi

	sleep 0.1
done
