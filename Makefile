# The ESP32 is addressed by the udev symlink from cap_ws's scripts/setup_udev.sh,
# NOT by a raw ttyUSB number: the lidar is also a CP210x and the two swap
# enumeration order across reboots.
#
# Override for a one-off, rather than editing this file:
#     make upload PORT=/dev/ttyUSB0
#
# If /dev/esp32 is missing, the adapter has been moved to a different USB
# socket -- the rules match on physical port path because neither adapter has
# a unique serial. Re-run `make udev` in ~/cap_ws instead of hardcoding a port.
PORT ?= /dev/esp32
FQBN ?= esp32:esp32:esp32doit-devkit-v1

compile:
	arduino-cli compile . -b $(FQBN) -p $(PORT)
upload:
	arduino-cli upload . -b $(FQBN) -p $(PORT)
monitor:
	arduino-cli monitor . -b $(FQBN) -p $(PORT) --config 57600
