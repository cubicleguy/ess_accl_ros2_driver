//==============================================================================
//
//  accel_epsonA352.c - Epson M-A352AD Accelerometer protocol specific code
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

#include "accel_userFir.c"

/*****************************************************************************
** Function name:       registerDump
** Description:         Read all registers for debug purpose
** Parameters:          None
** Return value:        None
*****************************************************************************/
void registerDump(void) {
  unsigned int debug = TRUE;
  printf("\r\nRegister Dump:\r\n\r\n");
  printf("Window 0:\r\n");
  // registerRead16(0x00, 0x00, debug);  //write-only don't read this one
  registerRead16(0x00, 0x02, debug);
  registerRead16(0x00, 0x04, debug);
  registerRead16(0x00, 0x06, debug);
  printf("\r\n");
  registerRead16(0x00, 0x0A, debug);
  registerRead16(0x00, 0x0E, debug);
  registerRead16(0x00, 0x10, debug);
  printf("\r\n");
  registerRead16(0x00, 0x30, debug);
  registerRead16(0x00, 0x32, debug);
  registerRead16(0x00, 0x34, debug);
  printf("\r\n");
  registerRead16(0x00, 0x36, debug);
  registerRead16(0x00, 0x38, debug);
  registerRead16(0x00, 0x3A, debug);
  printf("\r\n");
  registerRead16(0x00, 0x3C, debug);
  registerRead16(0x00, 0x3E, debug);
  registerRead16(0x00, 0x40, debug);
  printf("\r\n");
  registerRead16(0x00, 0x42, debug);
  registerRead16(0x00, 0x44, debug);
  registerRead16(0x00, 0x46, debug);
  printf("\r\n");

  printf("Window 1:\r\n");
  registerRead16(0x01, 0x00, debug);
  registerRead16(0x01, 0x02, debug);
  registerRead16(0x01, 0x04, debug);
  printf("\r\n");
  registerRead16(0x01, 0x06, debug);
  registerRead16(0x01, 0x08, debug);
  registerRead16(0x01, 0x0A, debug);
  printf("\r\n");
  registerRead16(0x01, 0x0C, debug);
  registerRead16(0x01, 0x16, debug);
  registerRead16(0x01, 0x18, debug);
  printf("\r\n");
  registerRead16(0x01, 0x1A, debug);
  registerRead16(0x01, 0x1C, debug);
  registerRead16(0x01, 0x1E, debug);
  printf("\r\n");
  registerRead16(0x01, 0x2C, debug);
  registerRead16(0x01, 0x2E, debug);
  registerRead16(0x01, 0x30, debug);
  printf("\r\n");
  registerRead16(0x01, 0x32, debug);
  registerRead16(0x01, 0x34, debug);
  registerRead16(0x01, 0x36, debug);
  printf("\r\n");
  registerRead16(0x01, 0x46, debug);
  registerRead16(0x01, 0x48, debug);
  registerRead16(0x01, 0x4A, debug);
  printf("\r\n");
  registerRead16(0x01, 0x6A, debug);
  registerRead16(0x01, 0x6C, debug);
  registerRead16(0x01, 0x6E, debug);
  printf("\r\n");
  registerRead16(0x01, 0x70, debug);
  registerRead16(0x01, 0x72, debug);
  registerRead16(0x01, 0x74, debug);
  printf("\r\n");
  registerRead16(0x01, 0x76, debug);
  registerRead16(0x01, 0x78, debug);
  registerRead16(0x01, 0x7A, debug);
  printf("\r\n");
  registerRead16(0x01, 0x7E, debug);
  printf("\r\n");
}

/*****************************************************************************
** Function name:       acclInitOptions
** Description:         Initialize the accelerometer hardware to desired
*settings
**                      based on EpsonAcclOptions
** Parameters:          struct EpsonAcclOptions
** Return value:        OK or NG
**
*****************************************************************************/
int acclInitOptions(struct EpsonAcclOptions options) {
  unsigned int debug = FALSE;

  // SIG_CTRL
  // Output Select for X,Y,Z: 0=acceleration, 1=tilt
  // Measurement Condition, reduced noise floor if mesmod_sel is enabled
  // bias stabilization if temp_stabil is enabled
  int sig_ctrl_lo =
      (options.temp_stabil & 0x01) << 2 | (options.mesmod_sel & 0x01) << 4 |
      (options.out_sel_z & 0x01) << 5 | (options.out_sel_y & 0x01) << 6 |
      (options.out_sel_x & 0x01) << 7;

  // ND flags for accel_out X,Y,Z are enabled if accel_out is enabled
  // ND flag for temp_out is enabled if temp_out is enabled
  int sig_ctrl_hi =
      (options.accel_out & 0x01) << 1 | (options.accel_out & 0x01) << 2 |
      (options.accel_out & 0x01) << 3 | (options.temp_out & 0x01) << 7;

  // MSC_CTRL
  // Configure DRDY function (typically needed for SPI IF)
  // & EXT Trigger (if needed)
  int msc_ctrl_lo =
      (options.drdy_pol & 0x01) << 1 | (options.drdy_on & 0x01) << 2 |
      (options.ext_pol & 0x01) << 5 | (options.ext_sel & 0x01) << 6;

  // SMPL_CTRL
  // Configures the Data Output Rate of the ACCL.
  // Refer to Datasheet for valid Data Output Rate & Filter Setting combinations
  int smpl_ctrl_hi = (options.dout_rate & 0x07);

  // FILTER_CTRL
  // Configures the FIR filter of the ACCL.
  // Refer to Datasheet for valid Data Output Rate & Filter Setting combinations
  int filter_ctrl_lo = (options.filter_sel & 0x0F);

  // BURST_CTRL1
  // These enable or disable certain data fields in the burst read packet
  int burst_ctrl1_lo = (options.checksum_out & 0x1) | (options.count_out & 0x1)
                                                          << 1;

  // turn on all axis, or none
  int burst_ctrl1_hi =
      (options.accel_out & 0x01) | (options.accel_out & 0x01) << 1 |
      (options.accel_out & 0x01) << 2 | (options.temp_out & 0x01) << 6 |
      (options.flag_out & 0x01) << 7;

#ifdef DEBUG
  printf(
      "%02x: %02x\n%02x: %02x\n%02x: %02x\n%02x: %02x\n*%02x: %02x\n%02x: "
      "%02x\n%02x: %02x\n%02x: %02x\n",
      ADDR_SIG_CTRL_LO, sig_ctrl_lo, ADDR_SIG_CTRL_HI, sig_ctrl_hi,
      ADDR_MSC_CTRL_LO, msc_ctrl_lo, ADDR_SMPL_CTRL_HI, smpl_ctrl_hi,
      ADDR_FILTER_CTRL_LO, filter_ctrl_lo, ADDR_UART_CTRL_LO, 0x01,
      ADDR_BURST_CTRL1_LO, burst_ctrl1_lo, ADDR_BURST_CTRL1_HI, burst_ctrl1_hi);
#endif

  registerWriteByte(CMD_WINDOW1, ADDR_SIG_CTRL_HI, sig_ctrl_hi, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_SIG_CTRL_LO, sig_ctrl_lo, debug);

  registerWriteByte(CMD_WINDOW1, ADDR_MSC_CTRL_LO, msc_ctrl_lo, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_SMPL_CTRL_HI, smpl_ctrl_hi, debug);

  // if CMD_USERFIRTAPXXX, setup the user filter coefficients
  if (filter_ctrl_lo >= CMD_USERFIRTAP4) {
    printf("\r\n...Info: Loading User FIR Coefficients...\r\n");

    // write the coefficients starting at address 0x0800
    registerWriteByte(CMD_WINDOW1, ADDR_FIR_UADDR_HI, 0x08, debug);
    registerWriteByte(CMD_WINDOW1, ADDR_FIR_UADDR_LO, 0x00, debug);

    // determine the number of coefficients to write
    int tapnum = 0;
    switch (filter_ctrl_lo) {
      case CMD_USERFIRTAP4:
        tapnum = 4;
        break;
      case CMD_USERFIRTAP64:
        tapnum = 64;
        break;
      case CMD_USERFIRTAP128:
        tapnum = 128;
        break;
      default:
      case CMD_USERFIRTAP512:
        tapnum = 512;
        break;
    }

    // write the coefficients
    if (writeCoefficients(tapnum) != OK) {
      printf("\r\n...Error: Loading User FIR Coefficients Failed!\r\n");
      return FALSE;
    }
    printf("...Complete\r\n");
  }

  registerWriteByte(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, filter_ctrl_lo, debug);

  // Delay for filter config
  seDelayMS(EPSON_FILTER_DELAY);

  // Check that the FILTER_BUSY bit returns 0
  unsigned short rxData;
  unsigned short retryCount = 3000;
  do {
    rxData = registerRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, debug);
    retryCount--;
  } while ((rxData & 0x0020) == 0x0020 && (retryCount != 0));

  if (retryCount == 0) {
    printf("\r\n...Error: Filter busy bit did not return to 0b.");
    return FALSE;
  }

#ifdef SPI  
  // Always disable UART_AUTO mode for burst reading when using SPI IF
  registerWriteByte(CMD_WINDOW1, ADDR_UART_CTRL_LO, 0x00, debug);
#else
  // Always enable UART_AUTO mode for burst reading
  registerWriteByte(CMD_WINDOW1, ADDR_UART_CTRL_LO, 0x01, debug);
#endif

  registerWriteByte(CMD_WINDOW1, ADDR_BURST_CTRL1_LO, burst_ctrl1_lo, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_BURST_CTRL1_HI, burst_ctrl1_hi, debug);

  return TRUE;
}

/*****************************************************************************
** Function name:       writeCoefficients
** Description:         Writes the user filter coefficients to the Accelerometer
**
** Parameters:          None
** Return value:        Sucess or Fail (if timeout)
*****************************************************************************/
int writeCoefficients(int tapnum) {
  int32_t* userFirTable;

  // setup user FIR array with selected values
  switch (tapnum) {
    case 4:
      userFirTable = userFirTap4;
      break;
    case 64:
      userFirTable = userFirTap64;
      break;
    case 128:
      userFirTable = userFirTap128;
      break;
    default:
    case 512:
      userFirTable = userFirTap512;
      break;
  }

  for (int i = 0; i < tapnum;
       i++)  // catch TAP4, TAP64, TAP128 and return early
  {
    registerWriteByte(CMD_WINDOW1, ADDR_FIR_UDATA, userFirTable[i] & 0x000000FF,
                      FALSE);
    registerWriteByte(CMD_WINDOW1, ADDR_FIR_UCMD, CMD_USERFIR_WRITE, FALSE);
    if (testCoefWriteComplete() == NG) return FALSE;

    registerWriteByte(CMD_WINDOW1, ADDR_FIR_UDATA,
                      (userFirTable[i] & 0x0000FF00) >> 8, FALSE);
    registerWriteByte(CMD_WINDOW1, ADDR_FIR_UCMD, CMD_USERFIR_WRITE, FALSE);
    if (testCoefWriteComplete() == NG) return FALSE;

    registerWriteByte(CMD_WINDOW1, ADDR_FIR_UDATA,
                      (userFirTable[i] & 0x00FF0000) >> 16, FALSE);
    registerWriteByte(CMD_WINDOW1, ADDR_FIR_UCMD, CMD_USERFIR_WRITE, FALSE);
    if (testCoefWriteComplete() == NG) return FALSE;

    registerWriteByte(CMD_WINDOW1, ADDR_FIR_UDATA,
                      (userFirTable[i] & 0xFF000000) >> 24, FALSE);
    registerWriteByte(CMD_WINDOW1, ADDR_FIR_UCMD, CMD_USERFIR_WRITE, FALSE);
    if (testCoefWriteComplete() == NG) return FALSE;
  }

  return TRUE;
}

/*****************************************************************************
** Function name:       testCoefWriteComplete
** Description:         Tests to ensure the user filter coefficient write
*completes
**
** Parameters:          None
** Return value:        Sucess or Fail (if timeout)
*****************************************************************************/
int testCoefWriteComplete(void) {
  int timeout = 0;

  while (registerRead16(CMD_WINDOW1, ADDR_FIR_UCMD, FALSE) != 0x0000) {
    timeout++;
    if (timeout > 500) {
      printf("Coefficient Write Failure\n");
      return NG;
    }
  }

  return OK;
}

