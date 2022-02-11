cmake_minimum_required(VERSION 3.5)
project(epson_accl_ros2_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Specify DEBUG macro to enable any debug code by adding "-DDEBUG" in add_definitions()
#add_definitions(-DDEBUG)

# Refer to the README_src.md inside the src folder for more details about ACCL macro
# Uncomment the model to build for
# Currently only A352 is supported at the time of writing
# NOTE: Change ACCL models require rebuilding binaries with colcon
set(model "A352")
add_definitions(-D${model})
message([STATUS] "---- Building for ACCL Model: ${model}")

# Select the interface type UART or SPI
# For SPI interface, this software has only been tested on RaspberryPi 3 B+ 
set(interface "UART")
#set(interface "SPI")
add_definitions(-D${interface})
message([STATUS] "---- Building for interface: ${interface}")


# Create filelist macro of C sources for library
if (interface STREQUAL "SPI")
  set(platform "RPI")
  add_definitions(-D${platform})
  message([STATUS] "---- Building for platform: ${platform}")
  set(lib_sources
    src/hcl_rpi.c
    src/hcl_gpio_rpi.c
    src/hcl_spi_rpi.c
    src/accel_epsonCommon.c
    src/accel_epsonSpi.c
  )
elseif (interface STREQUAL "UART")
  set(lib_sources
    src/hcl_gpio.c
    src/hcl_linux.c
    src/hcl_uart.c
    src/accel_epsonCommon.c
    src/accel_epsonUart.c
  )
else ()
  message([FATAL_ERROR] "**** Invalid Model")
endif()

# Add model specific C source to filelist macro
if (model STREQUAL "A352")
  set(lib_sources ${lib_sources}
    src/accel_epsonA352.c)
else()
  message([FATAL_ERROR] "**** Invalid Model")
endif()

# Declare static library for Epson ACCL functions
add_library(epson_accl_driver_lib STATIC
  ${lib_sources}
  )


# Declare C++ executable
if (interface STREQUAL "SPI")
  add_executable(accl_node src/epson_accl_spi_ros2_node.cpp)
else()
  add_executable(accl_node src/epson_accl_uart_ros2_node.cpp)
endif()


# For SPI, determine location of wiringPi library on the host system
# wiringPi needed if building on Raspberry Pi platform
# Link Epson ACCL C Library to the ROS node
if (interface STREQUAL "SPI")
  find_library(wiringPi_LIB NAMES wiringPi)
  target_link_libraries(accl_node
    epson_accl_driver_lib
    ${wiringPi_LIB}
    crypt
    rt
    pthread
  )
else ()
  target_link_libraries(accl_node
    epson_accl_driver_lib
  )
endif()


# Link C++ Library to the ROS node
ament_target_dependencies(accl_node
  rclcpp
  std_msgs
  sensor_msgs
  )

# Mark executables and/or libraries for installation
install(
   TARGETS accl_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Mark directories for installation
install(
   DIRECTORY launch DESTINATION share/${PROJECT_NAME}
)

#ament_export_include_directories(include)
#ament_export_dependencies(std_msgs)
ament_package()
