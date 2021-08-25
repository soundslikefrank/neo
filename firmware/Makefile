.PHONY: clean upload connect

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

.make/uploaded: .make/compiled
	arduino-cli upload --port $$(ls /dev/tty.usbmodem*) --fqbn arduino:mbed_nano:nano33ble .
	touch .make/uploaded

clean:
	arduino-cli cache clean
	rm .make/installed .make/compiled .make/uploaded

upload:
	arduino-cli upload --port $$(ls /dev/tty.usbmodem*) --fqbn arduino:mbed_nano:nano33ble .

connect: .make/uploaded
	tio -b 115200 $$(ls /dev/tty.usbmodem*)
