//==============================================================================
//
// 	main_regdump.c - Epson IMU sensor test application
//                 - This program reads all registers values for debug purpose
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

int main(int argc, char *argv[]) {
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

  printf("...done.");

  // Incase, the IMU is currently in sampling mode, force config mode before
  // attempting to read from registers
  acclStop();
  registerDump();
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
