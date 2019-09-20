# M5WatchCompass
 Watch and Compass Arduino Sketch for M5Stack
 Button A powers down/up to/from light sleep
  Button B displays a watch
Button C displays a compass
 
 Portions of the time code are taken from the "SimpleTime" example of ESP32
 Portions of the AK8963 magnetometer code are taken from:
 https://github.com/kriswiner/ESP32/blob/master/MPU9250_MS5637/MPU9250_MS5637_AHRS.ino"
 by: Kris Winer, December 14, 2016. 
 Buy him beers!
 
 At setup, the internet time is read from a WiFi access point set in SECRET_SSID and SECRET_PASS of M5Watch.h
 Don't forget to include M5Watch.h and fill out SECRET_SSID and SECRET_PASS there.
  
 Also at setup, a magnetometer calibration is performed when newMagCalibration = true.   
 It is very necessary to calibrate the magnetometer before useful readings can be taken.
 Slowly rotate the device aruond all axes for about 20 seconds during calibration.
 Once you have read out the IMU.magbias calibration values for a device, you can hard code them into the variables indicated below to
 speed up things by setting newMagCalibration = false.
 Calibration is especially necessary if you have magnets at the M4Stack bottom!!
