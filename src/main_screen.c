//==============================================================================
//
// 	main_screen.c - Epson IMU sensor test application
//                   - This program initializes the Epson IMU and
//                     sends sensor output to console
//
//
//  THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  NONINFRINGEMENT, SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A
//  PARTICULAR PURPOSE. IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE
//  OR CLAIM, ARISING FROM OR IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE
//  SOFTWARE.
//
//==============================================================================

#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "accel_epsonCommon.h"
#include "hcl.h"
#include "hcl_gpio.h"
#include "main_helper.h"

#ifdef SPI
#include "hcl_spi.h"

#else //UART
#include <termios.h>
#include "hcl_uart.h"

int fd_serial;
// Modify below as needed for hardware
const char *ACCLSERIAL = "/dev/ttyUSB0";

// The baudrate value should be set the the same setting as currently
// flashed value in the ACCL UART_CTRL BAUD_RATE register
const int ACCLBAUD = 460800;
#endif // SPI

// Specify the number of samples to readout before exiting the program
const unsigned int NUM_SAMPLES = 1000;

int main(int argc, char *argv[]) {
  unsigned int sample = 0;

  // Stores the post-processed accelerometer data
  struct EpsonAcclData epson_data;

  // Specify accelerometer options
  // Note: unsupported options should be set to 0
  struct EpsonAcclOptions options = {

      .out_sel_x = 0,  // for x,y,z: 0=Acceleration, 1=Tilt angle
      .out_sel_y = 0,
      .out_sel_z = 0,
      .mesmod_sel = 0,  // 0=standard noise floor, 1=reduced noise floor
      .temp_stabil = 0,
      .ext_sel = 0,  // 0 = Sample Counter 1=Reset Counter 2=External Trigger
      .ext_pol = 0,
      .drdy_on = 1,
      .drdy_pol = 1,
      .dout_rate = CMD_RATE200,
      .filter_sel = CMD_FIRTAP512FC16,
      .flag_out = 1,
      .temp_out = 1,
      .accel_out = 1,
      .count_out = 1,
      .checksum_out = 1,
  };

  // 1) Initialize the Seiko Epson HCL layer
  printf("\r\nInitializing HCL layer...");
  if (!seInit()) {
    printf(
        "\r\nError: could not initialize the Seiko Epson HCL layer. "
        "Exiting...\r\n");
    return -1;
  }
  printf("...done.\r\n");

  // 2) Initialize the GPIO interfaces, For GPIO control of pins SPI CS, RESET,
  // DRDY
  printf("\r\nInitializing GPIO interface...");
  if (!gpioInit()) {
    printf("\r\nError: could not initialize the GPIO layer. Exiting...\r\n");
    seRelease();
    return -1;
  }
  printf("...done.");

#ifdef SPI
  // 3) Initialize SPI Interface
  printf("\r\nInitializing SPI interface...");
  // The max SPI clock rate is 1MHz for current model Epson IMUs
  if (!spiInit(SPI_MODE3, 500000)) {
    printf("\r\nError: could not initialize SPI interface. Exiting...\r\n");
    gpioRelease();
    seRelease();
    return -1;
  }
  printf("...done.");
#else //UART
  // 3) Initialize UART Interface
  printf("\r\nInitializing UART interface...");
  fd_serial = uartInit(ACCLSERIAL, ACCLBAUD);
  if (fd_serial == -1) {
    printf("\r\nError: could not initialize UART interface. Exiting...\r\n");
    gpioRelease();
    seRelease();
    return -1;
  }
  printf("...done.");
#endif //SPI

  // 4) Power on sequence - force sensor to config mode, HW reset sensor
  //      Check for errors
  printf("\r\nChecking accelerometer NOT_READY status...");
  if (!acclPowerOn()) {
    printf("\r\nError: failed to power on accelerometer. Exiting...\r\n");
#ifdef SPI
    spiRelease();
#else //UART
    uartRelease(fd_serial);
#endif //SPI
    gpioRelease();
    seRelease();
    return -1;
  }
  printf("...done.");

  // Initialize sensor with desired settings
  printf("\r\nInitializing accelerometer...");
  if (!acclInitOptions(options)) {
    printf("\r\nError: could not initialize Epson Sensor. Exiting...\r\n");
#ifdef SPI
    spiRelease();
#else //UART
    uartRelease(fd_serial);
#endif //SPI
    gpioRelease();
    seRelease();
    return -1;
  }
  printf("...Epson Sensor initialized.");

  // Initialize text files for data logs
  const time_t date =
      time(NULL);  // Functions for obtaining and printing time and date

  printf("Date: ");
  printf("%s", ctime(&date));
  printf("\r\n...Epson Accelerometer Logging.\r\n");
  acclStart();
  printHeaderRow(stdout, options);
  while (1) {
    // For SPI interface, check if DRDY pin asserted
    // For UART interface, check if UART recv buffer contains a sensor sample
    // packet
#ifdef SPI
    if (acclDataReady()) {
#else //UART
    if (acclDataReadyOptions(options)) {
#endif //SPI
      acclDataReadBurstNOptions(options, &epson_data);
      printSensorRow(stdout, options, &epson_data, sample);
      sample++;
    }
    if (sample > (NUM_SAMPLES - 1)) break;
  }

  const time_t end =
      time(NULL);  // Functions for obtaining and printing time and data

  printf("\r\nEnd: ");
  printf("%s", ctime(&end));

  acclStop();
  seDelayMS(1000);
#ifdef SPI
  spiRelease();
#else //UART
  uartRelease(fd_serial);
#endif //SPI
  gpioRelease();
  seRelease();
  printf("\r\n");
  return 0;
}
