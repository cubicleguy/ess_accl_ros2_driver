//==============================================================================
//
//  accel_epsonCommon.c - Epson Accelerometer protocol specific code common
//                        for all Accelerometer models
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

/*****************************************************************************
** Function name:       acclHWReset
** Description:         Toggle the RESET pin, delay, wait for NOT_READY bit=0
**                      This is only applicable on embedded platforms with
**                      GPIO pin connected to ACCL RESET#
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int acclHWReset(void) {
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;

  gpioSet(EPSON_RESET);  // RESET pin HIGH
  seDelayMS(DELAY_EPSON_RESET);
  gpioClr(EPSON_RESET);  // Assert RESET (LOW)
  seDelayMS(DELAY_EPSON_RESET);
  gpioSet(EPSON_RESET);  // Deassert RESET (HIGH)
  seDelayMS(EPSON_POWER_ON_DELAY);

  // Poll NOT_READY bit every 1msec until returns 0
  // Exit after specified retries
  do {
    rxData = registerRead16(CMD_WINDOW1, ADDR_GLOB_CMD_LO, debug);
    seDelayMicroSecs(1000);
    retryCount--;
  } while ((rxData & 0x0400) == 0x0400 && (retryCount != 0));

  if (retryCount == 0) {
    printf("\r\n...Error: NOT_READY stuck HIGH.");
    return NG;
  }
  return OK;
}

/*****************************************************************************
** Function name:       acclPowerOn
** Description:         Initial startup, Goto Config Mode (sanity),
**                      Trigger HW Reset, check for Hardware Error Flags
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int acclPowerOn(void) {
  unsigned short rxData = 0xFFFF;
  unsigned int debug = FALSE;
  unsigned short retryCount = 3000;

  // Safety Measure, Force Exit of Sampling Mode
  do {
    registerWriteByte(CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_END_SAMPLING, debug);
    rxData = registerRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO, debug);
    seDelayMicroSecs(1000);
    retryCount--;
  } while ((rxData & 0x0400) == 0x0000 && (retryCount != 0));

  if (retryCount == 0) {
    printf("\r\n...Error: Stuck in Sampling Mode.");
    return NG;
  }

  // Hardware Reset if connected, and check for NOT_READY flag
  if (!acclHWReset()) return NG;

  // Check for error flags
  rxData = registerRead16(CMD_WINDOW0, ADDR_DIAG_STAT, debug);
  if (rxData == 0x0000)
    return OK;
  else
    return NG;
}

/*****************************************************************************
** Function name:       acclStart
** Description:         Start accelerometer sampling (goto Sampling Mode)
** Parameters:          None
** Return value:        None
*****************************************************************************/
void acclStart(void) {
  unsigned int debug = FALSE;

  registerWriteByte(CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_BEGIN_SAMPLING, debug);
  printf("\r\n...Accelerometer start.");
  seDelayMS(100);
}

/*****************************************************************************
** Function name:       acclStop
** Description:         Stop accelerometer sampling (goto Config Mode)
** Parameters:          None
** Return value:        None
*****************************************************************************/
void acclStop(void) {
  unsigned int debug = FALSE;

  registerWriteByte(CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_END_SAMPLING, debug);
  seDelayMicroSecs(
      200000);  // Provide 200msec for accelerometer to finish sending sample
  printf("\r\n...Accelerometer stop.");
}

/*****************************************************************************
** Function name:       acclReset
** Description:         Send Software Reset to Accelerometer + Delay 800 msec
** Parameters:          None
** Return value:        None
*****************************************************************************/
void acclReset(void) {
  unsigned int debug = FALSE;

  printf("\r\n...Software Reset begin.");
  registerWriteByte(CMD_WINDOW1, ADDR_GLOB_CMD_LO, CMD_SOFTRESET, debug);
  seDelayMS(EPSON_POWER_ON_DELAY);
  printf("\r\n...Software Reset complete.");
}

/*****************************************************************************
** Function name:       acclFlashTest
** Description:         Send Flashtest command to Accelerometer and check status
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int acclFlashTest(void) {
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;

  printf("\r\n...Flash test begin.");
  registerWriteByte(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_FLASHTEST, debug);
  seDelayMS(EPSON_FLASH_TEST_DELAY);
  do {
    rxData = registerRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, debug);
    retryCount--;
  } while ((rxData & 0x0800) == 0x0800 && (retryCount != 0));
  if (retryCount == 0) {
    printf("\r\n...Error: Flashtest bit did not return to 0b.");
    return NG;
  }

  rxData = registerRead16(CMD_WINDOW0, ADDR_DIAG_STAT, debug);
  printf("\r\n...Flash test complete.");

  if ((rxData & 0x0004) != 0x0000) return NG;

  return OK;
}

/*****************************************************************************
** Function name:       acclSelfTest
** Description:         Send SelfTest command to Accelerometer and check status
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int acclSelfTest(void) {
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;

  printf("\r\n...Self test begin.");
  registerWriteByte(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_SELFTEST, debug);
  seDelayMS(EPSON_SELF_TEST_DELAY);
  do {
    rxData = registerRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, debug);
    retryCount--;
  } while ((rxData & 0x0400) == 0x0400 && (retryCount != 0));
  if (retryCount == 0) {
    printf("\r\n...Error: Self test bit did not return to 0b.");
    return NG;
  }

  rxData = registerRead16(CMD_WINDOW0, ADDR_DIAG_STAT, debug);
  printf("\r\n...Self test complete.");

  if ((rxData & 0x0002) == 0x0000)
    return OK;
  else
    return NG;
}

/*****************************************************************************
** Function name:       acclDataByteLength
** Description:         Determines the accelerometer burst read packet data
*length
**                      based on the parameters in the EpsonAcclOptions struct.
** Parameters:          options - struct describing ACCL configuration.
** Return value:        data byte length
*****************************************************************************/
unsigned int acclDataByteLength(struct EpsonAcclOptions options) {
  unsigned int length = 0;

  // 16 bit ND_EA FLAG
  if (options.flag_out) length += 2;

  // 16 or 32 bit Temperature Output
  if (options.temp_out) {
    length += 4;
  }

  // 16 or 32 bit Accl X, Y, Z Output
  if (options.accel_out) {
    length += 12;
  }

  // 16 bit Checksum status output
  if (options.checksum_out) length += 2;

  // 16 bit Count output
  if (options.count_out) length += 2;

#if !defined SPI
  // For Start and End byte when using UART IF
  length += 2;
#endif

  return length;
}

/*****************************************************************************
** Function name:       acclDummyWrite
** Description:         Sets the WINDOW_ID of ACCL to 0
**                      This is a dummy access meant as debug to flush 
**                      the sensor IF on embedded Linux platform 
** Parameters:          None
** Return value:        None
*****************************************************************************/
void acclDummyWrite(void) {
  unsigned int debug = FALSE;

  seDelayMicroSecs(100000);
  registerWriteByte(CMD_WINDOW0, ADDR_WIN_CTRL, 0x00, debug);
  seDelayMicroSecs(100000);
  printf("\r\n...acclDummyWrite.");
}
