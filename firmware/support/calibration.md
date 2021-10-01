# Sensor calibration

1. Compile and upload the calibration script
2. Connect to a serial console and follow the instructions
3. Copy the data into CSVs:
  - Ignore gyroscope calibration data
  - Copy the first three columns into the csv for accelerometer calibration
  - Copy the last three columns into the csv for magnetometer calibration
4. Compile magneto calibration script
5. Run the script two times, for the accelerometer and the magnetometer data
   respectively
6. Copy the initialization data into the IMU class file
