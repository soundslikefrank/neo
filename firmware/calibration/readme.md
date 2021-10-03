# Sensor calibration

1. Compile and upload the calibration script with the ArduinoIDE (or `make calibration` and `make upload-calibration`, if you have `arduino-cli`)
2. Connect to Arduino serial console and follow the instructions
3. Copy the data into CSVs (under calibration/build)
  - Note down gyroscope calibration data
  - Copy the first three columns into the csv for accelerometer calibration
  - Copy the last three columns into the csv for magnetometer calibration
5. Compile magneto calibration script (`make magneto`), it's going to end up in `calibration/build/magneto`
6. Run the script two times, for the accelerometer and the magnetometer data respectively (just run `./magneto`, it's going to ask for the csv filenames)
7. Copy the code initialization statements into the imu.cpp file (adjust the variable names accordingly - A_B/A_Ainv for accelerometer, M_B/M_Ainv for magnetometer)
9. Adjust the gyroscope offsets in the imu.cpp file
