.PHONY: clean upload

all: .make/installed .make/compiled

.make/installed:
	arduino-cli core install arduino:mbed_nano
	arduino-cli lib install \
		BLE-MIDI \
		Arduino_LSM9DS1 \
		SensorFusion
	touch .make/installed

# Because arduino-cli puts the build artifacts god knows where
.make/compiled: .make/installed firmware.ino src/*.cpp include/*.h
	arduino-cli compile --fqbn arduino:mbed_nano:nano33ble .
	touch .make/compiled

clean:
	arduino-cli cache clean
	rm .make/installed .make/compiled

upload: .make/compiled
	arduino-cli upload --port /dev/cu.usbmodem143201 --fqbn arduino:mbed_nano:nano33ble .
