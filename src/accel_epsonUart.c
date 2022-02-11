//==============================================================================
//
//  accel_epsonUart.c - Epson Accelerometer protocol UART specific code
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
#include "accel_epsonCommon.h"
#include "hcl.h"
#include "hcl_gpio.h"
#include "hcl_uart.h"

// These are declared by the main() application for UART IF
extern const char* ACCLSERIAL;  // COM port device name
extern int fd_serial;           // COM port handle

// UART Interface Timing
// TWRITERATE/TREADRATE = 200us min @ 460800 BAUD, 1 command = 3 bytes = 3 *
// 22us = 66us TSTALL = 200us - 66us = 134us
#define EPSON_STALL 134  // Microseconds

// UART Byte Markers
const unsigned char UART_HEADER =
    0x80;  // Placed at the start of all UART cycles
const unsigned char UART_DELIMITER =
    0x0D;  // Placed at the end of all UART cycles

// #defines and variables used by the sampling state machine
unsigned char data[256];
#define START 0
#define DATA 1
#define END 2
static int state = START;
static int data_count = 0;



/*****************************************************************************
** Function name:       acclDataReadyOptions
** Description:         For UART interface check if comport recv buffer
**                      contains a burst of data based on expected byte length
**                      from acclDataByteLength()
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int acclDataReadyOptions(struct EpsonAcclOptions options) {
  seDelayMicroSecs(100);
  unsigned int count = numBytesReadComPort(fd_serial);

  if (count >= acclDataByteLength(options)) return OK;
  return NG;
}

/*****************************************************************************
** Function name:       registerWriteByte
** Description:         Write Byte to Register = Set WIN_ID, Write Data
**                      to Register
** Parameters:          Window Number, Register Address, Register Write Byte,
**                      Verbose Flag
** Return value:        None
*****************************************************************************/
void registerWriteByte(unsigned char winNumber, unsigned char regAddr,
                       unsigned char regByte, unsigned int verbose) {
  unsigned char txData[3];

  txData[0] = ADDR_WIN_CTRL | 0x80;  // msb is 1b for register writes
  txData[1] = winNumber;
  txData[2] = UART_DELIMITER;
  writeComPort(fd_serial, txData, 3);
  EpsonStall();

  txData[0] = regAddr | 0x80;  // msb is 1b for register writes
  txData[1] = regByte;
  txData[2] = UART_DELIMITER;
  writeComPort(fd_serial, txData, 3);
  EpsonStall();

  if (verbose) {
    printf("\r\nREG[0x%02X(W%01X)] < 0x%02X\t", regAddr, winNumber, regByte);
  }
}

/*****************************************************************************
** Function name:       registerRead16
** Description:         Read 16-bit from Register
** Parameters:          Window Number, Register Address, Verbose Flag
** Return value:        Register Read Value 16-bit
*****************************************************************************/
unsigned short registerRead16(unsigned char winNumber, unsigned char regAddr,
                              unsigned int verbose) {
  unsigned char response[4] = {0};
  int size;
  unsigned char txData[3];

  txData[0] = ADDR_WIN_CTRL | 0x80;
  txData[1] = winNumber;
  txData[2] = UART_DELIMITER;
  writeComPort(fd_serial, txData, 3);
  EpsonStall();

  txData[0] =
      regAddr & 0x7E;  // msb is 0b for register reads & address must be even
  txData[1] = 0x00;
  txData[2] = UART_DELIMITER;
  writeComPort(fd_serial, txData, 3);

  EpsonStall();

  // Attempt to read 4 bytes from serial port
  // Validation check: Should be atleast 4 bytes, First byte should be Register
  // Address,
  //                   Last byte should be delimiter
  size = readComPort(fd_serial, &response[0], 4);
  if ((size < 4) || (response[0] != txData[0]) ||
      (response[3] != UART_DELIMITER))
    printf("Returned less data or unexpected data from previous command.\n");

  EpsonStall();

  if (verbose) {
    printf("REG[0x%02X(W%01X)] > 0x%02X%02X\t", regAddr, winNumber, response[1],
           response[2]);
  }
  return (unsigned short)response[1] << 8 | (unsigned short)response[2];
}

/*****************************************************************************
** Function name:       acclDataReadBurstNOptions
** Description:         Retrieves 1 packet from the incoming ACCL stream based
**                      on expected burst length and searching for START and
**                      END markers. Then calls populateEpsonData() to
**                      post process into struct.
** Parameters:          options - struct describing ACCL configuration.
**                      epson_data - struct that is filled with data.
** Return value:        OK or NG
** Notes:
******************************************************************************/
int acclDataReadBurstNOptions(struct EpsonAcclOptions options,
                              struct EpsonAcclData* epson_data) {
  int byte_length = acclDataByteLength(options);
  int data_length = byte_length - 2;  // exclude the START and END markers
  unsigned char byte;

  while (readComPort(fd_serial, &byte, 1) > 0) {
#ifdef DEBUG
    printf("state: %d, byte: 0x%02X\n", state, byte);
#endif
    // State machine to seek out START & END markers and then
    // call to populateEpsonData()
    switch (state) {
      case START:
        if (byte == UART_HEADER) state = DATA;
        break;
      case DATA:
        data[data_count] = byte;
        data_count++;
        if (data_count == data_length) state = END;
        break;
      case END:
        data_count = 0;
        state = START;
        if (byte == UART_DELIMITER) {
#ifdef DEBUG
          for (int i = 0; i < data_length; i++) printf("0x%02X ", data[i]);
          printf("\n");
#endif

          if (options.checksum_out == 1) {
            unsigned short checksum = 0;
            for (int i = 0; i < data_length - 2; i += 2) {
              checksum += (data[i] << 8) + data[i + 1];
            }
            unsigned short epson_checksum =
                (data[data_length - 2] << 8) + data[data_length - 1];

            if (checksum == epson_checksum) {
              populateEpsonData(options, epson_data);
              return OK;
            } else
              printf("checksum failed\n");
          } else {
            populateEpsonData(options, epson_data);
            return OK;
          }
        }
        // Reaches here means checksum validation fails
        return NG;
        break;
      default:
        // Should never get here
        printf("Invalid State in Read Burst Processing\n");
    }
  }
  // No byte received in serial port yet
  return NG;
}


/*****************************************************************************
** Function name:       populateEpsonData
** Description:         Retrieves burst data buffer and converts/parses into
*struct
**                      based on configuration.
** Parameters:          options - struct describing ACCL configuration.
**                      epson_data - struct that is filled with converted data.
** Return value:        none
** Notes:
******************************************************************************/
void populateEpsonData(struct EpsonAcclOptions options,
                       struct EpsonAcclData* epson_data) {
#ifdef DEBUG
  for (int i = 0; i < 22; i++) printf("SD: 0x%02X ", data[i]);
  printf("\n");
#endif

  // stores the accelerometer data array index when parsing out data fields
  int idx = 0;

  // parsing of data fields applying conversion factor if applicable
  if (options.flag_out) {
    unsigned short ndflags = (data[idx] << 8) + data[idx + 1];
    epson_data->ndflags = ndflags;
    idx += 2;
#ifdef DEBUG
    printf("ndflag: %04x\t", epson_data->ndflags);
#endif
  }

  if (options.temp_out) {
      int temp = (data[idx] << 8 * 3) + (data[idx + 1] << 8 * 2) +
                 (data[idx + 2] << 8) + data[idx + 3];
      epson_data->temperature = (temp * EPSON_TEMP_SF) + 34.987;
      idx += 4;
#ifdef DEBUG
    printf("tempC: %8.3f\t", epson_data->temperature);
#endif
  }

  if (options.accel_out) {
      // process x axis data
      int accel_x = (data[idx] << 8 * 3) + (data[idx + 1] << 8 * 2) +
                    (data[idx + 2] << 8) + data[idx + 3];
      if (options.out_sel_x == 0)  // acceleration
        epson_data->accel_x = (EPSON_ACCL_SF * accel_x);
      else  // tilt
        epson_data->accel_x = (EPSON_TILT_SF * accel_x);

      // process y axis data
      int accel_y = (data[idx + 4] << 8 * 3) + (data[idx + 5] << 8 * 2) +
                    (data[idx + 6] << 8) + data[idx + 7];
      if (options.out_sel_y == 0)  // acceleration
        epson_data->accel_y = (EPSON_ACCL_SF * accel_y);
      else  // tilt
        epson_data->accel_y = (EPSON_TILT_SF * accel_y);

      // process z axis data
      int accel_z = (data[idx + 8] << 8 * 3) + (data[idx + 9] << 8 * 2) +
                    (data[idx + 10] << 8) + data[idx + 11];
      if (options.out_sel_z == 0)  // acceleration
        epson_data->accel_z = (EPSON_ACCL_SF * accel_z);
      else  // tilt
        epson_data->accel_z = (EPSON_TILT_SF * accel_z);

      idx += 12;
#ifdef DEBUG
    printf("ax: %8.5f\tay: %8.5f\taz: %8.5f\t",
           epson_data->accel_x * 1000 / 9.80665,
           epson_data->accel_y * 1000 / 9.80665,
           epson_data->accel_z * 1000 / 9.80665);
#endif
  }

  if (options.count_out) {
    int count = (data[idx] << 8) + data[idx + 1];
    epson_data->count = count;
#ifdef DEBUG
    printf("count: %09d\t", epson_data->count);
#endif
  }
}
