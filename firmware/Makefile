.PHONY: calibration-firmware clean connect magneto upload upload-calibration

FIND_PORT = arduino-cli board list | sed -n "s/^.*\/dev\(\S*\).*Arduino\sNano\s33\sBLE.*$$/\/dev\1/p"
FQBN = arduino:mbed_nano:nano33ble

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
	arduino-cli compile --fqbn $(FQBN) --build-path build .

.make/uploaded: build
	arduino-cli upload --port $$($(FIND_PORT)) --fqbn $(FQBN) --input-dir build
	touch .make/uploaded

clean:
	arduino-cli cache clean
	rm -rf .make/installed .make/uploaded
	rm -rf build

upload: build
	arduino-cli upload --port $$($(FIND_PORT)) --fqbn $(FQBN) --input-dir build
	touch .make/uploaded

connect: .make/uploaded
	tio -b 115200 $$($(FIND_PORT))

calibration/calib_firmware/build: .make/installed calibration/calib_firmware/calib_firmware.ino
	arduino-cli compile --fqbn $(FQBN) --build-path calibration/calib_firmware/build calibration/calib_firmware

calibration-firmware: calibration/calib_firmware/build

upload-calibration: calibration/calib_firmware/build
	arduino-cli upload --port $$($(FIND_PORT)) --fqbn $(FQBN) --input-dir calibration/calib_firmware/build

calibration/build/magneto:
	g++ -o calibration/build/magneto calibration/magneto1_4.c

magneto: calibration/build/magneto
