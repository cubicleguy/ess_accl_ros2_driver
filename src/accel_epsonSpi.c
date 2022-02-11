//==============================================================================
//
//  accl_epsonSpi.c - Epson Accelerometer protocol SPI specific code
//
//
//  THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT,
//  SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT
//  SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.
//
//==============================================================================
#include "hcl.h"
#include "hcl_gpio.h"
#include "hcl_spi.h"
#include "accel_epsonCommon.h"

// SPI Interface Timing
// TREADRATE = 40 us min, TCYCLERATE @ 0.5MHz = 32 uS, STALL must be atleast
// (40us - 32us) or 20uS which ever is GREATER (20uS)
#define EPSON_STALL 20                // Microseconds,
#define BURST_STALL1 45               // Microseconds
#define BURST_STALL2 4                // Microseconds
#define selEpson() gpioClr(EPSON_CS)  // For asserting chipselect in SPI IF
#define deselEpson() gpioSet(EPSON_CS)
#define burstStall1() \
  seDelayMicroSecs(   \
      BURST_STALL1)  // For delay after issuing Burst Read Cmd in SPI IF
#define burstStall2() \
  seDelayMicroSecs(BURST_STALL2)  // For delay on consecutuve cycles after Burst
                                  // Read Cmd in SPI IF

// Function Prototype Specific to SPI IF
void sensorDataReadN(unsigned short[], unsigned int,
                     unsigned char);  // For G350/V340 SPI IF only
void sensorDataReadBurstN(unsigned short[], unsigned int);

static unsigned short rxdata[128];


/*****************************************************************************
** Function name:       acclDataReady
** Description:         For SPI IF check if DataReady is HIGH.
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int acclDataReady(void) {
  // Sensor data is ready when the DRDY Pin is HIGH
  return (gpioGetPinLevel(EPSON_DRDY));
}

/*****************************************************************************
** Function name:       registerWriteByteNoID
** Description:         Write Byte to Register = Write Data
**                      to Register (no WIN_ID)
** Parameters:          Register Address, Register Write Byte,
**                      Verbose Flag
** Return value:        None
*****************************************************************************/
void registerWriteByteNoId(unsigned char regAddr, unsigned char regByte,
                           unsigned int verbose) {
  selEpson();
  spiTransfer(regAddr | 0x80);  // msb is 1b for register writes
  spiTransfer(regByte);
  EpsonStall();
  deselEpson();

  if (verbose) {
    printf("\r\nREG[0x%02X] < 0x%02X\t", regAddr, regByte);
  }
}

/*****************************************************************************
** Function name:       registerWriteByte
** Description:         Write Byte to Register = Set WIN_ID,
**                      write Data to Register
** Parameters:          Window Number, Register Address, Register
**                      WriteDataByte, Verbose Flag
** Return value:        None
*****************************************************************************/
void registerWriteByte(unsigned char winNumber, unsigned char regAddr,
                       unsigned char regByte, unsigned int verbose) {

  registerWriteByteNoId(ADDR_WIN_CTRL, winNumber, 0);

  registerWriteByteNoId(regAddr, regByte, 0);
  if (verbose) {
    printf("\r\nREG[0x%02X(W%01X)] < 0x%02X\t", regAddr, winNumber, regByte);
  }
}

/*****************************************************************************
** Function name:       registerRead16NoId
** Description:         Read 16-bit from Register (No WIN_ID)
** Parameters:          Register Address, Verbose Flag
** Return value:        Register Read Value 16-bit
*****************************************************************************/
unsigned short registerRead16NoId(unsigned char regAddr, unsigned int verbose) {
  short rxData[] = {0x00, 0x00};

  selEpson();
  spiTransfer(regAddr &
              0x7E);  // msb is 0b for register reads & address must be even
  spiTransfer(0x00);
  EpsonStall();

  rxData[0] = spiTransfer(0x00);
  rxData[1] = spiTransfer(0x00);
  EpsonStall();
  deselEpson();

  if (verbose) {
    printf("REG[0x%02X] > 0x%02X%02X\t", regAddr, rxData[0],
           rxData[1]);
  }
  return (rxData[0] << 8 | rxData[1]);
}

/*****************************************************************************
** Function name:       registerRead16
** Description:         Read 16-bit from Register
** Parameters:          Window Number, Register Address, Verbose Flag
** Return value:        Register Read Value 16-bit
*****************************************************************************/
unsigned short registerRead16(unsigned char winNumber, unsigned char regAddr,
                              unsigned int verbose) {
  unsigned short rxData = 0x0000;

  registerWriteByteNoId(ADDR_WIN_CTRL, winNumber, 0);

  rxData = registerRead16NoId(regAddr, 0);
  if (verbose) {
    printf("REG[0x%02X(W%01X)] > 0x%04X\t", regAddr, winNumber, rxData);
  }
  return rxData;
}

/*****************************************************************************
** Function name:      acclDataReadN
** Description:        Perform SPI sequential register read to acquire sensor
**                     data (address is incremented by 2)
** Parameters:         pointer to signed short array, size of array,
**                     register start address
** Return value:       none
** Notes:
** 1. It is recommended to use acclDataBurstReadN() to contiguously burst read
**    sensor data instead of this function due to inefficiency with large gaps
**    in register addresses to read a burst
**
** 2. Call this function only after detecting DRDY is asserted.
**    This function will send N+1 SPI commands to read N registers.
**    Maximum SPI clock is 2MHz for SPI reads.
*****************************************************************************/
void acclDataReadN(unsigned short sensorReadData[], unsigned int readLen,
                     unsigned char regAddr) {
  unsigned int i;

  selEpson();
  spiTransfer(regAddr);
  spiTransfer(0x00);
  EpsonStall();

  for (i = 0; i < readLen; i++) {
    signed int tmp = spiTransfer(regAddr + (2 * (i + 1)));
    sensorReadData[i] = (tmp << 8) + spiTransfer(0x00);
    EpsonStall();
  }
  deselEpson();
}

/*****************************************************************************
** Function name:       acclDataReadBurstN
** Description:         Perform burst read to acquire sensor data
** Parameters:          pointer to signed short array, size of array
** Return value:        none
** Notes:
** 1. The burst packet consists of 16-bit data units.
** 2. This function will send N+1 SPI commands to read N
**    registers.
**    Maximum SPI clock is 2MHz for burst SPI reads.
** 3. No checksum verification is performed (in this function)
**
*****************************************************************************/
void acclDataReadBurstN(unsigned short sensorReadData[],
                          unsigned int readLen) {
  unsigned int i;

  selEpson();
  spiTransfer(CMD_BURST);
  spiTransfer(0x00);
  burstStall1();

  for (i = 0; i < readLen; i++) {
    signed short tmp = spiTransfer(0x00);
    sensorReadData[i] = (tmp << 8) + spiTransfer(0x00);
#ifdef DEBUG
    printf("\r\n0x%04x:[0x%04X]", i, sensorReadData[i]);
#endif
    burstStall2();
  }
  deselEpson();
}


/*****************************************************************************
** Function name:       acclDataReadBurstNOptions
** Description:         Retrieves 1 packet from the incoming sensor stream based
**                      on expected burst length for SPI IF.
**                      Then calls populateEpsonData() to
**                      post process into struct.
** Parameters:          options - struct describing ACCL configuration.
**                      epson_data - struct that is filled with data.
** Return value:        OK or NG
** Notes:
******************************************************************************/
int acclDataReadBurstNOptions(struct EpsonAcclOptions options,
                                struct EpsonAcclData* epson_data) {

  unsigned int data_length = acclDataByteLength(options) / 2;

// Burst read the sensor data based on calculated burst size from options struct
  acclDataReadBurstN(rxdata, data_length);

  if (options.checksum_out == 1) {
    unsigned short checksum = 0;
    for (unsigned int i = 0; i < data_length - 1; i++) {
      checksum += rxdata[i];
    }
    unsigned short epson_checksum = rxdata[data_length - 1];

    if (checksum == epson_checksum) {
      populateEpsonData(options, epson_data);
      return OK;
    } else {
      printf("checksum failed\n");
    }
  } else {
    populateEpsonData(options, epson_data);
    return OK;
  }

  // Reaches here means checksum validation fails
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

  // stores the sensor data array index when parsing out data fields
  int idx = 0;

  // parsing of data fields applying conversion factor if applicable
  if (options.flag_out) {
    unsigned short ndflags = rxdata[idx];
    epson_data->ndflags = ndflags;
    idx++;
#ifdef DEBUG
    printf("ndflag: %04x\t", epson_data->ndflags);
#endif
  }

  if (options.temp_out) {
      int temp = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
      epson_data->temperature = (temp * EPSON_TEMP_SF) + 34.987;
      idx += 2;
#ifdef DEBUG
    printf("tempC: %8.3f\t", epson_data->temperature);
#endif
  }


  if (options.accel_out) {
      int accel_x = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
      int accel_y = (rxdata[idx + 2] << 8 * 2) + rxdata[idx + 3];
      int accel_z = (rxdata[idx + 4] << 8 * 2) + rxdata[idx + 5];
      if (options.out_sel_x == 0)  // acceleration
        epson_data->accel_x = (EPSON_ACCL_SF * accel_x);
      else  // tilt
        epson_data->accel_x = (EPSON_TILT_SF * accel_x);
      if (options.out_sel_y == 0)  // acceleration
        epson_data->accel_y = (EPSON_ACCL_SF * accel_y);
      else  // tilt
        epson_data->accel_y = (EPSON_TILT_SF * accel_y);
      if (options.out_sel_z == 0)  // acceleration
        epson_data->accel_z = (EPSON_ACCL_SF * accel_z);
      else  // tilt
        epson_data->accel_z = (EPSON_TILT_SF * accel_z);
      idx += 6;
#ifdef DEBUG
    printf("ax: %8.5f\tay: %8.5f\taz: %8.5f\t",
           epson_data->accel_x * 1000 / 9.80665,
           epson_data->accel_y * 1000 / 9.80665,
           epson_data->accel_z * 1000 / 9.80665);
#endif
  }

  if (options.count_out) {
    int count = rxdata[idx];
    epson_data->count = count;
#ifdef DEBUG
    printf("count: %09d\t", epson_data->count);
#endif
  }
}
