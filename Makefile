compile:
	arduino-cli compile . -b esp32:esp32:esp32doit-devkit-v1 -p /dev/esp32
upload:
	arduino-cli upload . -b esp32:esp32:esp32doit-devkit-v1 -p /dev/esp32
monitor:
	arduino-cli monitor . -b esp32:esp32:esp32doit-devkit-v1 -p /dev/esp32 --config 57600
