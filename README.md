![qiblah-cover-2](https://github.com/yahyatawil/qiblah/assets/1148381/188c6657-440e-4591-99fc-eb15e7c3ef34)

# This Project
An open-source qibla finder with tilt compensation using 9-DoF IMU and GPS connected with Arduino.

## Features

The Qiblah project features are:
- Determining the Qibla using the magnetometer and/or the GPS receiver in case the circuit is likely to move for long distances.
- Tilt compensation.
- Magnetometer calibration.
- Open source for non-commercial purposes.
- Built with public and available circuits and easy to rebuild.
- Well documented.

## Hardware
  - Arduino board.
  - BMI270 shuttle board (using [BMI270_AUX_BMM150](https://github.com/yahyatawil/BMI270_AUX_BMM150) library). 
  - Adafruit Mini GPS PA1010D.
  - Monochrome 0.91" 128x32 I2C OLED Display.

<img src="https://github.com/yahyatawil/qiblah/blob/main/imgs/qiblah_hardware.png" width="400" height="400">

## Demo


https://github.com/yahyatawil/qiblah/assets/1148381/e3276048-1e34-4c34-82e1-fbbcd989b52b


## Contribution
This circuit is built and tested in my city. Anyone can re-build and validate it with correct qiblah direction, please report a confirmation in issues. 

## Documentation 
- [Arabic](https://atadiat.com/ar/open-source-qibla-compass-with-tilt-compensation/).
- [English](https://atadiat.com/en/e-open-source-qibla-compass-with-tilt-compensation/).

## References: 
- [The Correct Qibla](http://nurlu.narod.ru/qibla.pdf "The Correct Qibla") 
-  [Implementing a Tilt-Compensated eCompass using Accelerometer and Magnetometer Sensors](https://www.mikrocontroller.net/attachment/292888/AN4248.pdf "Implementing a Tilt-Compensated eCompass using Accelerometer and Magnetometer Sensors").
- [Towards understanding IMU: Basics of Accelerometer and Gyroscope Sensors and How to Compute Pitch, Roll and Yaw Angles](https://atadiat.com/en/e-towards-understanding-imu-basics-of-accelerometer-and-gyroscope-sensors/ "Towards understanding IMU: Basics of Accelerometer and Gyroscope Sensors and How to Compute Pitch, Roll and Yaw Angles").
- [Magnetometer Soft Iron and Hard Iron Calibration: Why and How](https://atadiat.com/en/e-magnetometer-soft-iron-and-hard-iron-calibration-why-how/ "Magnetometer Soft Iron and Hard Iron Calibration: Why and How").
- [Towards understanding IMU: Frames of reference used to represent IMU orientation and how to visualize the circuit orientation using Vpython library](https://atadiat.com/en/e-towards-understanding-imu-frames-vpython-visualize-orientation/ "Towards understanding IMU: Frames of reference used to represent IMU orientation and how to visualize the circuit orientation using Vpython library").


