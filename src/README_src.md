# Disclaimer:
--------------
THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT,
SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR
IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.

# Test machine:
--------------
  
## UART Interface,
- Ubuntu 18.04 Mate running in Oracle VirtualBox on Core i7 Win10 PC
- Ubuntu 18.04.5 LTS (GNU/Linux 4.15.0-1062-raspi2 armv7l) running on RaspberryPi B+ (with Epson USB evalboard)


## SPI Interface,
- Ubuntu 18.04.5 LTS (GNU/Linux 4.15.0-1062-raspi2 armv7l) running on RaspberryPi B+ (with Epson USB evalboard)
- WiringPi library for GPIO, SPI, & timer delay functions, [Unofficial wiringPi](https://github.com/WiringPi/WiringPi/)

# Requirements:
--------------

- For using ACCL UART interface, this should work on any generic Unix (POSIX) serial port with gcc or g++

- For using ACCL SPI interface, this is only tested on RaspberryPi using wiringPi Library. Although, this should work on other embedded Linux platforms with some minor changes 

NOTE: This document only applies to compiling and running this Linux userspace software independently on a Linux PC or RaspberryPi with embedded Linux.
      For proper operation in ROS environment refer to the README.md found elsewhere in the root of the archive.


# Important for UART Interface:
-----------------------------
1. The application assumes that the Epson ACCL is connected to serial tty (UART) either through USB evaluation board or directly to the UART port on an embedded Linux system.

2. You may need to enable user access to the dialout group to access the serial tty
```
sudo adduser $USER dialout
```


# Important for RPi SPI Interface:
-----------------------------
1. After the installation process, apply any updates.
   sudo apt-get update
   sudo apt-get upgrade
   
2. Use raspi-config utility to enable the SPI interface:
   sudo raspi-config
   
   Choose the advanced Options -> SPI -> enable SPI kernel module to be loaded by default "Yes"
   sudo reboot
   
3. Verify the SPI interface is enabled.
   lsmod | grep spi_
   ls /dev | grep spi

4. Install the wiringPi library.
   This should already be installed if running a RPi-specific Linux distro, otherwise go to [Unofficial wiringPi](https://github.com/WiringPi/WiringPi/) on how to install

5. The application is designed with the following pinmapping:
```
    Epson ACCL                   Raspberry Pi
    ---------------------------------------------------
    EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) 
    EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24)
    EPSON_CS                    RPI_GPIO_P1_16 (GPIO23)
    SPI_SCK                     RPI_GPIO_P1_23 (GPIO11)
    SPI_MISO                    RPI_GPIO_P1_21 (GPIO9)
    SPI_MOSI                    RPI_GPIO_P1_19 (GPIO10)
```

# Important for usleep():
-----------------------
- **NOTE:** In the hcl_[platform].c, there are functions for time delays in millisecond and microseconds using seDelayMS() and seDelayMicroSecs(), respectively.

- On embedded Linux platforms, these may need to be redirected to HW specific delay routines if usleep() is not supported.

- For example on RaspberryPi, the time delay functions for millisecond and microseconds can be redirected to WiringPi library delay() and delayMicroseconds(), respectively.


# Important for GPIO usage:
-------------------------
- When using this driver to connect the ACCL using the UART interface, the use of GPIO pins for connecting to the ACCL RESET#, EXT, or DRDY is  optional.

- When using this driver to connect the ACCL using the SPI interface, the use of GPIO pins for connecting to the ACCL RESET#, EXT, or DRDY is **mandatory**.

- When possible, connecting the RESET# is recommended to force Hardware Reset during every ACCL initialization.

- This code does not implement low-level GPIO functions. However, the code is structured to easily redirect to low-level hardware GPIO function calls for ease of implementation.
   
- There is no standard method to implement GPIO connections on embedded Linux platform, but the following files typically need changes:

  - src/hcl_[platform].c
  - src/hcl_gpio_[platform].c
  - src/hcl_gpio.h

- Typically, an external GPIO library needs to be invoked to initialize the GPIO HW functions.

### Changes to hcl_xxx.c file
- This requires changes to hcl_[platform].c

  - add #include to external library near the top of hcl_linux.c
  - add #include hcl_gpio.h near the top of the hcl_linux.c
  - add the initialization call inside the seInit() function in hcl_[platform].c
    
- For example on an Raspberry Pi, changes to hcl_rpi.c:

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

### Changes to hcl_gpio.h file
- Typically, the GPIO pins need to be assigned according to pin numbering specific to the HW platform.
- For example on an Raspberry Pi, changes to hcl_gpio.h with the following pinmapping:

```
    Epson ACCL                   Raspberry Pi
    ---------------------------------------------------
    EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) Output
    EPSON_CS                    RPI_GPIO_P1_16 (GPIO23) Output
    EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24) Input

```

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

### Changes to the hcl_gpio_xxx.c file
- Typically, the external library will have GPIO pin control such as set_output, set_input, set, reset, read_pin_level, etc...
- This requires changes to hcl_gpio_[platform].c
- Redirect function calls in hcl_gpio_[platform].c for gpioInit(), gpioRelease(), gpioSet(), gpioClr(), gpioGetPinLevel() to equivalent external library pin control functions.
- For example on an Raspberry Pi, changes to hcl_gpio_rpi.c:

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
    pinMode(EPSON_CS, OUTPUT);                  // <== Added external call RESET Output Pin
    pinMode(EPSON_DRDY, INPUT);                 // <== Added external call DRDY Input Pin
    pullUpDnControl(EPSON_DRDY, PUD_OFF) ;      // <== Added external call Disable any internal pullup or pulldown resistance

    return SUCCESS;
  }
  .
  .
  .
  int gpioRelease(void)
  {
    return SUCCESS;
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

# Important for compiling:
--------------------------

- Compile the application using the Makefile with make:
  - Run "make" specifying the <target> with "MODEL=xxx" parameters.
  - Supported <target> options are: 
```
    screen 
    csvlogger 
    regdump
    all
```
- Supported "MODEL=" parameters are: A352 (support for new models as needed)
- If "MODEL=" is not specified then it assumes MODEL=A352
- Specify the serial interface type using "IF=" macro definition equal "UART" (default) or "SPI. If "IF=" is not specified then it assumes IF=UART

```
Example make commands:
    make clean  <-- recommended before creating new builds
    make screen IF=SPI
    make csvlogger
    make regdump
    make all IF=UART
```
- The executable will be in the found in the same folder as the Makefile and source files.
- **NOTE:** Modify the EpsonAcclOptions struct to configure sensor configuration settings in main() function in main_xxxx.c


# How to run the program:
-----------------------
1. Run the executable from console (may require root access to execute if regular user can not access tty or spi)
```
   sudo ./<executable filename>
```

2. The default csvlogger program creates CSV log of sensor data in a processed scaled log file for 1000 samples:

- DRDY is enabled active HIGH
- Output date rate = 200 Hz (For all other models)
- Filter Tap = 512 Tap FIR Filter Fcutoff 16Hz


### Example CSV log file
```
Date: Tue Oct 27 10:33:54 2020

sample[dec],     nd[hex], tempC[degC],   ax[mG],     ay[mG],     az[mG],     count[dec]
00000000,    8e00,     22.417,   106.30692,  -258.93054,     -972.20007,     000000020
00000001,    8e00,     22.417,   106.38834,  -258.99442,     -972.16180,     000000040
00000002,    8e00,     22.417,   106.48013,  -258.99402,     -972.13220,     000000060
00000003,    8e00,     22.417,   106.57506,  -258.96375,     -972.11719,     000000080
00000004,    8e00,     22.417,   106.66386,  -258.95770,     -972.10846,     000000100
00000005,    8e00,     22.417,   106.73622,  -259.01474,     -972.09027,     000000120
00000006,    8e00,     22.417,   106.78368,  -259.13440,     -972.05402,     000000140
00000007,    8e00,     22.417,   106.80246,  -259.27811,     -972.00787,     000000160
00000008,    8e00,     22.417,   106.79532,  -259.38937,     -971.97339,     000000180
```
- **NOTE:** Output fields can be enabled or disabled in the *EpsonAcclOptions struct* by setting the desired xxx_out to 1 (enabled) or 0 (disabled)

# File listing:
--------------
```
epson_accl_xxx_ros2_node.cpp - ROS2 Node C++ to C wrapper
                             - The ROS2 node is built using ROS2 colcon build environment
                             - xxx can be "spi" or "uart"

hcl.h                       - Dummy abstraction layer header (working template) which defines delay() functions
hcl_linux.c                 - Abstraction layer for generic Linux intended for UART
hcl_rpi.c                   - Abstraction layer for RaspberryPi (HW init, HW release, delay routines) intended for SPI
hcl_gpio.c                  - Abstraction layer for GPIO control functions typically for connection to RESET, DRDY, SCS#
                              This a dummy assignment of pins RESET, DRDY, SCS#
                              Modify or replace if GPIO pins are to be used
hcl_gpio_rpi.c              - Abstraction layer for GPIO control functions typically for connection to RESET, DRDY, SCS# when using wiringPI & RaspberryPi
                              Modify as needed for GPIO pins are to be used
hcl_gpio.h                  - Header for GPIO abstraction
hcl_uart.c                  - Abstraction layer specific for UART IF which uses standard unix termios library calls
hcl_spi_rpi.c               - Abstraction layer specific for SPI IF which uses wiringPi library calls
hcl_uart.h                  - Header for UART IF abstraction
hcl_spi.h                   - Header for SPI IF abstraction
main_csvlogger.c            - Test application - Initialize ACCL, and read sensor data to CSV log file
main_regdump.c              - Test application - Output register settings to console for debug purpose
main_screen.c               - Test application - Initialize ACCL, and read sensor data to console
main_helper.c               - Helper functions for console utilities
main_helper.h               - Header for helper functions for console utilities
Makefile                    - For make utility to compile test applications
README_src.md               - This file.
accel_epsonCommon.c         - Common functions for Epson ACCL
accel_epsonCommon.h         - Header for common C functions of Epson ACCL
accel_epsonGA352.c          - Model specific functions for Epson M-A352
accel_epsonGA352.h          - Model specific header for Epson M-A352
accel_epsonUart.c           - UART dependent functions
accel_epsonSpi.c            - SPI dependent functions
accel_userFir.c             - Example User Defined Filter coefficients  
```

# Change record:
--------------
```
2021-01-15  v1.0    - Initial release
```
