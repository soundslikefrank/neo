.PHONY: clean upload connect

all: .make/installed build

.make/installed:
	arduino-cli core install arduino:mbed_nano
	arduino-cli lib install \
		--git-url https://github.com/JonHub/Filters \
		--git-url https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
	arduino-cli lib install \
		BLE-MIDI \
		SensorFusion
	touch .make/installed

build: .make/installed firmware.ino src/*.cpp src/*.h
	arduino-cli compile --fqbn arduino:mbed_nano:nano33ble --build-path build .

.make/uploaded: build
	arduino-cli upload --port $$(arduino-cli board list | sed -n "s/^.*\/dev\(\S*\).*Arduino\sNano\s33\sBLE.*$$/\/dev\1/p") --fqbn arduino:mbed_nano:nano33ble --input-dir build
	touch .make/uploaded

clean:
	arduino-cli cache clean
	rm -rf .make/installed .make/uploaded
	rm -rf build

upload: build
	arduino-cli upload --port $$(arduino-cli board list | sed -n "s/^.*\/dev\(\S*\).*Arduino\sNano\s33\sBLE.*$$/\/dev\1/p") --fqbn arduino:mbed_nano:nano33ble --input-dir build
	touch .make/uploaded

connect: .make/uploaded
	tio -b 115200 $$(arduino-cli board list | sed -n "s/^.*\/dev\(\S*\).*Arduino\sNano\s33\sBLE.*$$/\/dev\1/p")
