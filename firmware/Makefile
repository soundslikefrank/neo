.PHONY: clean upload connect

all: .make/installed .make/compiled

.make/installed:
	arduino-cli core install arduino:mbed_nano
	arduino-cli lib install \
		--git-url https://github.com/JonHub/Filters \
		--git-url https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
	arduino-cli lib install \
		BLE-MIDI \
		SensorFusion
	touch .make/installed

# Because arduino-cli puts the build artifacts god knows where
.make/compiled: .make/installed firmware.ino src/*.cpp src/*.h
	arduino-cli compile --fqbn arduino:mbed_nano:nano33ble --build-path build .
	touch .make/compiled

.make/uploaded: .make/compiled
	arduino-cli upload --port $$(ls /dev/tty.usbmodem*) --fqbn arduino:mbed_nano:nano33ble .
	touch .make/uploaded

clean:
	arduino-cli cache clean
	rm .make/installed .make/compiled .make/uploaded

upload: .make/compiled
	arduino-cli upload --port $$(arduino-cli board list | sed -n "s/^.*\/dev\(\S*\).*Arduino\sNano\s33\sBLE.*$$/\/dev\1/p") --fqbn arduino:mbed_nano:nano33ble .

connect: .make/uploaded
	tio -b 115200 $$(ls /dev/tty.usbmodem*)
