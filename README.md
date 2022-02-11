# README for Epson Accelerometer Driver for ROS2 Node

## What is this repository for?

* This code provides interface between Epson Accelerometer and ROS2 using either SPI  or UART interface.
* For using the SPI interface, this code uses the [Unofficial wiringPi library](https://github.com/WiringPi/WiringPi/) for accessing GPIO and SPI functions on the Raspberry Pi platform running Ubuntu Linux + ROS2
  * src/epson_accl_spi_ros2_node.cpp is the ROS2 C++ wrapper used to communicate with ROS2
* For using the UART interface, the UART connection can be either direct or by USB-serial converter such as FTDI bridge ICs.
  * src/epson_accl_uart_ros2_node.cpp is the ROS2 C++ wrapper used to communicate with ROS2
* The other source files in src/ are based on the C driver released by Epson:
  [Epson Accelerometer Linux User-space Driver Example](https://vdc.epson.com/ACCL-products/accelerometers)
* Information about ROS2, and tutorials can be found: [ROS.org](https://index.ros.org/doc/ros2/)


## What kind of hardware or software will I likely need?

* ROS2 Dashing or later (via download) [ROS2.org](https://www.ros.org)
  * Installation guide [ROS2.org](https://wiki.ros.org/ROS2/Installation)
* This software was developed and tested on the following:
```
  ROS2:        Dashing or later
  Description: Ubuntu 18.04.5 LTS (GNU/Linux 4.15.0-1062-raspi2 armv7l)
  Release:     18.04.5
  Codename:    bionic
```
* Epson Accelerometer and realted evaluation board [ACCL models](
https://global.epson.com/products_and_drivers/sensing_system/acc/)

### SPI
* The embedded Linux host system will need the SPI interface (SCLK, MISO, MOSI) and 3 GPIOs (CS#, RST#, DRDY) enabled prior to using this software.
  - For Raspberry Pi, the SPI interface must already be enabled using raspi-config
  - This code uses a separate GPIO to manually toggle SCS# chipselect instead of the chipselect assigned to the RPI SPI interface
* Epson Breakout evaluation board or some equivalent to connect to ROS2 host (SPI & GPIOs) [See Evaluation Boards](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)

### UART
* Any generic Linux host system that has an available tty UART port to connect to the Epson ACCL
* Epson USB evaluation board or equivalent FTDI USB-Serial interface to connect to ROS2 host (tty/serial) [See Evaluation Boards](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)


## How do I use the driver?

* This code assumes that the user is familiar with building ROS2 packages using the colcon build process.
* This is *NOT* detailed instructions describing step by step procedures on how to build and install this ROS2 driver.
* Please refer to the ROS.org website for more detailed instructions on the ROS package build process. [ROS.org](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/)
* If the ACCL model & serial interface type is unchanged, then subsequent changes to ACCL settings can be done by just editing the ACCL model specific launch.py file (without re-building the binary).
* The ACCL model specific launch.py file should only be used with the colcon built target binary with the same matching ACCL model.
* **NOTE** Do not mix ACCL model launch.py files & ACCL model colcon built binaries.


## How do I use the driver if usleep() is not supported for time delays?

* **NOTE:** In the hcl_rpi.c & hcl_linux.c, there are wrapper functions for time delays in millisecond and microseconds using seDelayMS() and seDelayMicroSecs(), respectively.
* On embedded Linux platforms, these may need to be redirected to HW platform specific delay routines to avoid serial interface timing violations.
* For example on RaspberryPi, the time delay functions for millisecond and microseconds can be redirected to WiringPi library delay() and delayMicroseconds(), respectively.
* If a hardware delay is not available from a library, then a software delay loop is possible but not preferred.


## How do I use the driver with GPIOs to control ACCL RESET#, DRDY, EXT pins?

* When using this driver to connect using the SPI interface, the use of GPIO pins for connecting to the ACCL SCS# and DRDY is **mandatory** (RESET# is recommended, EXT is optional).
* When using this driver to connect using the UART interface, the use of GPIO pins for connecting to the ACCL RESET#, EXT, or DRDY is **optional**  because these pins are mainly intended for embedded Linux platforms (non-PC based).
* When possible, connecting the RESET# is recommended to force Hardware Reset during every ACCL initialization for better robustness.
* This code is structured to easily redirect to low-level hardware GPIO function calls for easy implementation such as RaspberryPi.


### Modifying hcl_xxx.c
* There is no standard method to implement GPIO connections on embedded Linux platform, but the following files typically need changes:

```
  src/hcl_[platform].c
  src/hcl_gpio_[platform].c
  src/hcl_gpio.h
```

* Typically, an external library needs to be invoked to initialize & enable GPIO HW functions.

* This typically requires changes to hcl_[platform].c, i.e. Use hcl_rpi.c as a template

  - add #include to external library near the top of hcl_[platform].c
  - add the initialization call inside the seInit() function in hcl_[platform].c

For example on an Raspberry Pi, the following changes can be made to hcl_[platform].c:

```
  .
  .
  .
  #include <stdint.h>
  #include <stdio.h>
  #include <wiringPi.h>  // <== Added external library

  int seInit(void)
  {
    // Initialize wiringPi libraries                                                   // <== Added
    printf("\r\nInitializing libraries...");                                           // <== Added
    if(wiringPiSetupGpio() != 0) {                                                     // <== Added external library initialization
      printf("\r\nError: could not initialize wiringPI libraries. Exiting...\r\n");    // <== Added
      return NG;                                                                       // <== Added
    }                                                                                  // <== Added
    printf("...done.");

    return OK;
  }
  .
  .
  .

```

### Modifying hcl_gpio.h
* Typically, the GPIO pins need to be assigned according to pin numbering specific to the HW platform.

* This typically requires changes to hcl_gpio.h

For example on an Raspberry Pi, the following changes to hcl_gpio.h with the following pin mapping:

```
    Epson ACCL                   Raspberry Pi
    ---------------------------------------------------
    EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) Output
    EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24) Input
    EPSON_CS                    RPI_GPIO_P1_16 (GPIO23) Output

```
Note: The RPI SPI0_cs0 is not connected. Chip select is being manually controlled via GPIO on P1_16. 

```
  // Prototypes for generic GPIO functions
  int gpioInit(void);
  int gpioRelease(void);

  void gpioSet(uint8_t pin);
  void gpioClr(uint8_t pin);
  uint8_t gpioGetPinLevel(uint8_t pin);

  #define RPI_GPIO_P1_15              22                    // <== Added
  #define RPI_GPIO_P1_16              23                    // <== Added
  #define RPI_GPIO_P1_18              24                    // <== Added

  #define EPSON_RESET                 RPI_GPIO_P1_15        // <== Added
  #define EPSON_CS                    RPI_GPIO_P1_16        // <== Added 
  #define EPSON_DRDY                  RPI_GPIO_P1_18        // <== Added
  .
  .
  .
```


### Modifying hcl_gpio_xxx.c
* Typically, the external library will have GPIO pin control functions such as set_output, set_input, set, reset, read_pin_level, etc...

* This requires changes to hcl_gpio_[platform].c

  - Redirect function calls in hcl_gpio.c for gpioInit(), gpioRelease(), gpioSet(), gpioClr(), gpioGetPinLevel() to equivalent external library pin control functions.

  - For example on an Raspberry Pi, the following changes to hcl_gpio_rpi.c:

```
  #include "hcl.h"
  #include "hcl_gpio.h"
  #include <wiringPi.h>                         // <== Added external library
  .
  .
  .

  int gpioInit(void)
  {
    pinMode(EPSON_RESET, OUTPUT);               // <== Added external call RESET Output Pin
    pinMode(EPSON_CS, OUTPUT);                  // <== Added external call CS# Output Pin
    pinMode(EPSON_DRDY, INPUT);                 // <== Added external call DRDY Input Pin
    pullUpDnControl(EPSON_DRDY, PUD_OFF) ;      // <== Added external call Disable any internal pullup or pulldown resistance

    return OK;
  }
  .
  .
  .
  int gpioRelease(void)
  {
    return OK;
  }
  .
  .
  .
  void gpioSet(uint8_t pin)
  {
    digitalWrite(pin, HIGH);                    // <== Added external call set pin HIGH
  }

  .
  .
  .
  void gpioClr(uint8_t pin)
  {
    digitalWrite(pin, LOW);                     // <== Added external call set pin LOW
  }

  .
  .
  .
  uint8_t gpioGetPinLevel(uint8_t pin)
  {
    return (digitalRead(pin));                  // <== Added external call to return pin state of input pin
  }

  .
  .
  .
```


## How do I build, install, run this ROS2 package?

* The Epson ACCL ROS2 driver is designed to build in the ROS colcon build environment.
Therefore, a functional colcon workspace in ROS2 is a prerequisite.
* Refer to the ROS2 Tutorials for more info: [ROS2 Tutorial](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/)

### Changing the serial interface type: UART or SPI
**NOTE**: When re-building this software after changing the *interface type* between SPI & UART,
it is necessary to delete the previous library file incase colcon does not properly detect 
that the interface has been switched, and incorrectly builds with the previously built library.

1. To do this, go into the <colcon_workspace>/build folder
2. Delete the **epson_accl_ros2_driver** folder

### Normal build process
For more information on ROS & colcon setup refer to
[Installing and Configuring ROS Environment](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/).


1. Place files (including folders) into a new folder within your colcon workspace "src" folder.
   For example, we recommend using the folder name "epson_accl_ros2"
```
   <colcon_workspace>/src/epson_accl_ros2/ <-- place files here
```
2. Modify the CMakeLists.txt:
   Refer to the comment lines inside the CMakeLists.txt for additional info.
   - select the desired Epson ACCL model (model)
   - select the serial interface type (interface)

   **NOTE:** You *MUST* re-build using colcon build when making any changes in the CmakeLists.txt

3. From the colcon_workspace folder run "colcon build" to build all ROS2 packages located in the <colcon_workspace>/src/ folder.
To build this specific package type the following:
```
   <colcon_workspace>/colcon build --packages-select epson_accl_ros2_driver --symlink-install
```
   Re-run the above "colcon build" command to rebuild the driver after making any changes to the CMakeLists.txt or any of the .c or .cpp or .h source files.
   It is not necessary to "colcon build" if changes are only made to the launch.py files

   *NOTE:* It is recommended to change ACCL settings by editing the parameters in the ROS2 launch.py file, wherever possible, instead of modifying the .c or .cpp source files directly

### Example console output of colcon build for A352 UART:
```
colcon build: error: argument --packages-select: unrecognized argument: --symlinks-install
guest@guest-desktop:~/dev_ws$ colcon build --packages-select epson_accl_ros2_driver --symlink-install
Starting >>> epson_accl_ros2_driver
[Processing: epson_accl_ros2_driver]
[Processing: epson_accl_ros2_driver]
[Processing: epson_accl_ros2_driver]
--- stderr: epson_accl_ros2_driver
[STATUS]---- Building for ACCL Model: A352
[STATUS]---- Building for interface: UART
[STATUS]---- Building for ACCL Model: A352
[STATUS]---- Building for interface: UART
[STATUS]---- Building for ACCL Model: A352
[STATUS]---- Building for interface: UART
---
Finished <<< epson_accl_ros2_driver [1min 41s]

Summary: 1 package finished [1min 43s]
  1 package had stderr output: epson_accl_ros2_driver
guest@guest-desktop:~/dev_ws$

```

4. Reload the current ROS2 environment variables that may have changed after the colcon build process.
```
   From the <colcon_workspace>: . install/setup.bash
```

5. Modify the appropriate launch file for the ACCL model in the launch/ folder to set your desired ACCL configure parameter options at runtime:
**NOTE:** Refer to the ROS2 launch file for inline descriptions.

Parameter            | Comment
-------------------- | -------------
serial_port          | specifies the string value to the tty serial device (ignored for SPI)
frame_id             | specifies the string in the frame_id field of the published message
imu_topic            | specifies the namespace of the topic for publishing
burst_polling_rate   | specifies the polling rate to detect incoming sensor data
mesmod_sel           | specifies standard noise floor or reduced noise floor mode
temp_stabil          | specifies to enable or disable temperature stabilization
ext_sel              | specifies to enable or disable External Trigger
ext_pol              | specifies the polarity of the External Trigger
dout_rate            | specifies the ACCL output data rate
filter_sel           | specifies the ACCL filter setting

**NOTE:** The ROS2 launch file passes ACCL configuration settings to the ACCL at runtime.
           Therefore does not need rebuilding with colcon when changing the launch file.

6. To start the Epson ACCL ROS2 driver use the appropriate launch file (located in launch/) from console.
- The launch file contains parameters for configuring settings at runtime
- All parameters are described in the inline comments of the launch file.

   For example, for the Epson A352 ACCL:
```
   <colcon_workspace>/ros2 launch epson_accl_ros2_driver epson_a352_launch.py
```


### Example console output of launching ROS2 node for A352 UART:
```
guest@guest-desktop:~/dev_ws$ ros2 launch epson_accl_ros2_driver epson_a352_launch.py
[INFO] [launch]: All log files can be found below /home/guest/.ros/log/2021-01-12-16-58-29-655164-guest-desktop-24530
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [accl_node-1]: process started with pid [24540]
[accl_node-1] [INFO] [accl_node]: serial_port:          /dev/ttyUSB0
[accl_node-1] [INFO] [accl_node]: frame_id:             imu_link
[accl_node-1] [INFO] [accl_node]: burst_polling_rate:   4000.0
[accl_node-1] [INFO] [accl_node]: mesmod_sel:   0
[accl_node-1] [INFO] [accl_node]: temp_stabil:  0
[accl_node-1] [INFO] [accl_node]: ext_sel:      0
[accl_node-1] [INFO] [accl_node]: ext_pol:      0
[accl_node-1] [INFO] [accl_node]: dout_rate:    4
[accl_node-1] [INFO] [accl_node]: filter_sel:   8
[accl_node-1] [INFO] [accl_node]: imu_topic:            /epson_accl/data_raw
[accl_node-1] [WARN] [accl_node]: Not specified param flag_out. Set default value:      1
[accl_node-1] [WARN] [accl_node]: Not specified param temp_out. Set default value:      1
[accl_node-1] [WARN] [accl_node]: Not specified param accel_out. Set default value:     1
[accl_node-1] [WARN] [accl_node]: Not specified param count_out. Set default value:     1
[accl_node-1] [WARN] [accl_node]: Not specified param checksum_out. Set default value:  1
[accl_node-1] Attempting to open port.../dev/ttyUSB0
[accl_node-1] [INFO] [accl_node]: Checking sensor power on status...
[accl_node-1] [INFO] [accl_node]: Initializing Sensor...
[accl_node-1] [INFO] [accl_node]: Epson ACCL initialized.
[accl_node-1] [INFO] [accl_node]: PRODUCT ID:   A352AD10
[accl_node-1] [INFO] [accl_node]: SERIAL ID:    W0000501
[accl_node-1]
 
```


## What does this ROS2 ACCL node publish as messages?

- The Epson Accelerometer ROS2 driver will publish imu messages which will only update the linear acceleration fields.
- For ACCL models such as A352, the ACCL messages will only contain fields for linear acceleration (accel) data the other fields will contain 0s

The Epson Accelerometer ROS2 driver will publish ACCL messages as convention per [REP 145](http://www.ros.org/reps/rep-0145.html).

### Example ROS2 topic output
The ROS2 driver will publish to the following ROS2 topics:
```
/epson_accl/data_raw <-- orientation, angular_velocity, & covariance fields will not contain valid data & should be ignored
```
**NOTE** The launch file can remap the message to publish on /epson_accl/data_raw which the user can be modify


#### ROS2 Topic Message /epson_accl/data_raw
```
---
header:
  seq: 18833
  stamp:
    secs: 1603756210
    nsecs: 518862006
  frame_id: "imu_link"
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: 2.56680425218
  y: -3.00368994106
  z: 9.0503388805
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---

```


## Why am I seeing high latencies or slower than expected ACCL data rates

### For SPI interface
- This will largely depend on your host system processing load and latency.
- If your ROS2 platform is running too many ROS2 node packages or simply too slow it may not be able detect the rising edge of the ACCL DRDY signal for then burst reading the ACCL sampling data, and post processing.
- Try modifying the *dout_rate* and *filter_sel* to the slowest setting that
can meet your ROS2 system requirements. 
- Try to monitor the DRDY signal with an oscilloscope on the ACCL to verify the stability of the ACCL DRDY signal when experiencing data rate issues.

### For UART interface
- If your connection between the Epson Accelerometer UART and the Linux host is by FTDI (or similar USB-UART bridge devices, the default latency_timer setting may be too large (i.e. typically 16msec).
- There are 2 recommended methods to reduce this value to 1msec.

#### Modifying latency_timer by sysfs mechanism

- The example below reads the latency_timer setting for /dev/ttyUSB0 which returns 16msec.
- Then, the latency_timer is set to 1msec, and confirmed by readback.

*NOTE*: May require root (sudo su) access on your system to modify. 
```
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
16
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
1
```

#### Modifying low_latency flag using setserial utility

The example below sets the low_latency flag for /dev/ttyUSB0.
This will have the same effect as setting the latency_timer to 1msec.
This can be confirmed by running the setserial command again.

```
user@user-VirtualBox:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0

user@user-VirtualBox:~$ setserial /dev/ttyUSB0 low_latency

user@user-VirtualBox:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0, Flags: low_latency
```


## Package Contents

The Epson Accelerometer ROS2 driver-related sub-folders & root files are:
```
   launch/        <== example launch file for Epson Accelerometer models
   src/           <== source code for ROS2 node C++ wrapper, ACCL C driver, and additional README_src.md
   CmakeLists.txt <== build script for colcon build
   package.xml    <== colcon package description
   README.md      <== general README for the ROS2 driver
```


## License

### The Epson ACCL C++ Wrapper ROS2 Node is released under BSD-3 license.

[This software is BSD-3 licensed.](http://opensource.org/licenses/BSD-3-Clause)

Copyright (c) 2021 Seiko Epson Corp. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

### The Epson Accelerometer C driver software is released as public domain.

THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
NONINFRINGEMENT, SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A
PARTICULAR PURPOSE. IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE
OR CLAIM, ARISING FROM OR IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE
SOFTWARE.


## References
1. https://index.ros.org/doc/ros2/
2. https://github.com/technoroad/ADI_ACCL_TR_Driver_ROS2
