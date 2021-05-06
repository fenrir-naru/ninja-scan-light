NinjaScan-Light
===============

<a href='https://github.com/fenrir-naru/ninja-scan-light/blob/master/board/NinjaScanLight-board_impl.jpg'><img src='https://raw.githubusercontent.com/fenrir-naru/ninja-scan-light/master/board/NinjaScanLight-board_impl-thumb.jpg' width='400px' /></a>

[NinjaScan-Light](https://github.com/fenrir-naru/ninja-scan-light/wiki) is an ultra-small (1.0 x 1.4 inch, approximately 26 x 36 mm) motion logger. 
It consists of
 
1. Silicon Laboratories C8051F381 MCU providing USB connectability, 
1. u-blox NEO-6 series GPS receiver, 
1. 6 DOF inertial sensor, Invensense MPU-6000 (ver.1), or Invensense MPU-9250 (ver.2), 
1. 3-axis magnetic sensor, Freescale MAG3110 (ver.1), or Invensense MPU-9250 (ver.2), 
1. Pressure sensor, Measurement specialties MS5611, 
1. Power management including Li-Ion battery charger, Linear Technology LTC3550, and 
1. MicroSD supporting SDHC/FAT32. 

**More information in [Wiki](https://github.com/fenrir-naru/ninja-scan-light/wiki).**

## Additional information
* Firmware binaries are released in [github](https://github.com/fenrir-naru/ninja-scan-light/releases).
* External links for [related tools](https://fenrir-naru.github.io/archives/), and [Gerbers](https://drive.google.com/folderview?id=0ByrAl6X3Khv2TkJ5Wkp4RmhMWjg&usp=sharing).
* The project owner's website is [Fenrir's BLog](http://fenrir.naruoka.org/).

[![Build Status (Firmware + Linux/RasPi tools by Travis-CI)](https://travis-ci.org/fenrir-naru/ninja-scan-light.svg?branch=master)](https://travis-ci.org/fenrir-naru/ninja-scan-light)
[![CI of firmware and tools](https://github.com/fenrir-naru/ninja-scan-light/workflows/CI%20of%20firmware%20and%20tools/badge.svg?branch=master)](https://github.com/fenrir-naru/ninja-scan-light/actions?query=workflow%3A%22CI+of+firmware+and+tools%22)
[![Build status (Windows tools)](https://ci.appveyor.com/api/projects/status/6r6koh5rophe6yns?svg=true)](https://ci.appveyor.com/project/fenrir-naru/ninja-scan-light)