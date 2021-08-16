.PHONY: clean upload

all: .make/installed .make/compiled

.make/installed:
	arduino-cli lib install \
	BLE-MIDI \
	"Adafruit LIS3MDL" \
	"Adafruit LSM6DS" 
	touch .make/installed

# Because arduino-cli puts the build artifacts god knows where
.make/compiled: .make/installed firmware.ino src/*.cpp include/*.h
	arduino-cli compile --fqbn esp32:esp32:nodemcu-32s .
	touch .make/compiled

clean:
	arduino-cli cache clean
	rm .make/installed .make/compiled

upload: .make/compiled
	arduino-cli upload --port /dev/cu.usbserial-0001 --fqbn esp32:esp32:nodemcu-32s .
