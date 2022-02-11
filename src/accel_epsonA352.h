//==============================================================================
//
//  accel_EPSONA352.h - Epson M-A352AD Accelerometer specific definitions
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
#ifndef EPSONA352_H_
#define EPSONA352_H_

#define EPSON_ACCL_SF (0.00000006)  // uG/LSB
#define EPSON_TEMP_SF (-0.0037918)
#define EPSON_TILT_SF (0.000000002)  // urad/LSB

/*                                      -- Commands --
    - ADDR_ address byte of transfer to select the register to access
    - CMD_  data byte of transfer to write to the register selected
     
    - All accesses are 16 bit transfers
    - For SPI IF:
        - For SPI write accesses - 8-bit address with msb=1b (can be even or
   odd) + 8-bit write data
                                 - No response
        - For SPI read accesses - 8-bit address with msb=0b(even only) + 8-bit
   dummy data
                                - Response is transferred on MOSI on next SPI
   access
                                - Return value is 16-bit read data (high byte +
   low byte)
    - For UART IF:
        - For UART write accesses - 8-bit address with msb=1b(can be even or
   odd) + 8-bit write data + Delimiter Byte
                                  - No response
        - For UART read accesses - 8-bit address with msb=0b(even only) + 8-bit
   dummy data + Delimiter Byte
                                 - Response is transferred immediately
                                 - Return value consists of Register Read
   Address + 16-bit read data (high byte + low byte) + Delimiter Byte
    
    - NOTE: Register Address Maps that depend on the WINDOW_ID (page) */

// WINDOW_ID 0
#define ADDR_BURST 0x00         // BURST (W0)
#define ADDR_MODE_CTRL_LO 0x02  // MODE_CTRL Byte0 (W0)
#define ADDR_MODE_CTRL_HI 0x03  // MODE_CTRL Byte1 (W0)
#define ADDR_DIAG_STAT 0x04     // DIAG_STAT (W0)
#define ADDR_FLAG 0x06          // FLAG(ND/EA) (W0)
#define ADDR_COUNT 0x0A         // COUNT (W0)
#define ADDR_TEMP_HIGH 0x0E     // TEMPC HIGH (W0)
#define ADDR_TEMP_LOW 0x10      // TEMPC LOW  (W0)
#define ADDR_XACCL_HIGH 0x30    // XACCL HIGH (W0)
#define ADDR_XACCL_LOW 0x32     // XACCL LOW  (W0)
#define ADDR_YACCL_HIGH 0x34    // YACCL HIGH (W0)
#define ADDR_YACCL_LOW 0x36     // YACCL LOW  (W0)
#define ADDR_ZACCL_HIGH 0x38    // ZACCL HIGH (W0)
#define ADDR_ZACCL_LOW 0x3A     // ZACCL LOW  (W0)
#define ADDR_XTILT_HIGH 0x3C    // XTILT HIGH (W0)
#define ADDR_XTILT_LOW 0x3E     // XTILT LOW  (W0)
#define ADDR_YTILT_HIGH 0x40    // YTILT HIGH (W0)
#define ADDR_YTILT_LOW 0x42     // YTILT LOW  (W0)
#define ADDR_ZTILT_HIGH 0x44    // ZTILT HIGH (W0)
#define ADDR_ZTILT_LOW 0x46     // ZTILT LOW  (W0)

// WINDOW_ID 1
#define ADDR_SIG_CTRL_LO 0x00     // SIG_CTRL Byte0 (W1)
#define ADDR_SIG_CTRL_HI 0x01     // SIG_CTRL Byte1 (W1)
#define ADDR_MSC_CTRL_LO 0x02     // MSC_CTRL Byte0 (W1)
#define ADDR_MSC_CTRL_HI 0x03     // MSC_CTRL Byte1 (W1)
#define ADDR_SMPL_CTRL_LO 0x04    // SMPL_CTRL Byte0 (W1)
#define ADDR_SMPL_CTRL_HI 0x05    // SMPL_CTRL Byte1 (W1)
#define ADDR_FILTER_CTRL_LO 0x06  // FILTER_CTRL (W1)
#define ADDR_UART_CTRL_LO 0x08    // UART_CTRL Byte0 (W1)
#define ADDR_UART_CTRL_HI 0x09    // UART_CTRL Byte1 (W1)
#define ADDR_GLOB_CMD_LO 0x0A     // GLOB_CMD Byte0 (W1)
#define ADDR_BURST_CTRL1_LO 0x0C  // BURST_CTRL1 Byte0 (W1)
#define ADDR_BURST_CTRL1_HI 0x0D  // BURST_CTRL1 Byte1 (W1)
#define ADDR_FIR_UCMD 0x16        // FIR_UCMD (W1)
#define ADDR_FIR_UDATA 0x18       // FIR_UDATA (W1)
#define ADDR_FIR_UADDR_LO 0x1A    // FIR_UADDR Byte0 (W1)
#define ADDR_FIR_UADDR_HI 0x1B    // FIR_UADDR Byte1 (W1)
#define ADDR_LONGFILT_CTRL 0x1C   // LONGFILT_CTRL (W1)
#define ADDR_LONGFILT_TAP 0x1E    // LONGFILT_TAP (W1)
#define ADDR_XOFFSET_HIGH_L 0x2C  // XOFFSET_HIGH_L (W1)
#define ADDR_XOFFSET_HIGH_H 0x2D  // XOFFSET_HIGH_H (W1)
#define ADDR_XOFFSET_LOW_L 0x2E   // XOFFSET_LOW_L (W1)
#define ADDR_XOFFSET_LOW_H 0x2F   // XOFFSET_LOW_H (W1)
#define ADDR_YOFFSET_HIGH_L 0x30  // YOFFSET_HIGH_L (W1)
#define ADDR_YOFFSET_HIGH_H 0x31  // YOFFSET_HIGH_H (W1)
#define ADDR_YOFFSET_LOW_L 0x32   // YOFFSET_LOW_L (W1)
#define ADDR_YOFFSET_LOW_H 0x33   // YOFFSET_LOW_H (W1)
#define ADDR_ZOFFSET_HIGH_L 0x34  // ZOFFSET_HIGH_L (W1)
#define ADDR_ZOFFSET_HIGH_H 0x35  // ZOFFSET_HIGH_H (W1)
#define ADDR_ZOFFSET_LOW_L 0x36   // ZOFFSET_LOW_L (W1)
#define ADDR_ZOFFSET_LOW_H 0x37   // ZOFFSET_LOW_H (W1)
#define ADDR_XALARM_LO 0x46       // XALARM_LO (W1)
#define ADDR_XALARM_UP 0x47       // XALARM_UP (W1)
#define ADDR_YALARM_LO 0x48       // YALARM_LO (W1)
#define ADDR_YALARM_UP 0x49       // YALARM_UP (W1)
#define ADDR_ZALARM_LO 0x4A       // ZALARM_LO (W1)
#define ADDR_ZALARM_UP 0x4B       // ZALARM_UP (W1)

#define ADDR_PROD_ID1 0x6A     // PROD_ID1(W1)
#define ADDR_PROD_ID2 0x6C     // PROD_ID2(W1)
#define ADDR_PROD_ID3 0x6E     // PROD_ID3(W1)
#define ADDR_PROD_ID4 0x70     // PROD_ID4(W1)
#define ADDR_VERSION 0x72      // VERSION(W1)
#define ADDR_SERIAL_NUM1 0x74  // SERIAL_NUM1(W1)
#define ADDR_SERIAL_NUM2 0x76  // SERIAL_NUM2(W1)
#define ADDR_SERIAL_NUM3 0x78  // SERIAL_NUM3(W1)
#define ADDR_SERIAL_NUM4 0x7A  // SERIAL_NUM4(W1)
#define ADDR_WIN_CTRL 0x7E     // WIN_CTRL(W0 or W1)

#define CMD_BURST 0x80    // Write value to Issue Burst Read
#define CMD_WINDOW0 0x00  // Write value for WIN_CTRL to change to Window 0
#define CMD_WINDOW1 0x01  // Write value for WIN_CTRL to change to Window 1
#define CMD_BEGIN_SAMPLING \
  0x01                         // Write value for MODE_CMD_HI to begin sampling
#define CMD_END_SAMPLING 0x02  // Write value for MODE_CMD_HI to stop sampling
#define CMD_ZSENSTEST \
  0x40  // Write value for MSC_CTRL_HI to issue Z Accelerometer Test
#define CMD_YSENSTEST \
  0x20  // Write value for MSC_CTRL_HI to issue Y Accelerometer Test
#define CMD_XSENSTEST \
  0x10  // Write value for MSC_CTRL_HI to issue X Accelerometer Test
#define CMD_FLASHTEST 0x08  // Write value for MSC_CTRL_HI to issue Flash Test
#define CMD_SELFTEST 0x04   // **use the ACCTEST as the default selftest**
#define CMD_ACCTEST \
  0x04  // Write value for MSC_CTRL_HI to issue Accelerometer Test
#define CMD_TEMPTEST \
  0x02  // Write value for MSC_CTRL_HI to issue Temp Sensor Test
#define CMD_VDDTEST 0x01  // Write value for MSC_CTRL_HI to issue Voltage Test

#define CMD_SOFTRESET \
  0x80  // Write value for GLOB_CMD_LO to issue Software Reset
#define CMD_FLASHBKUP 0x08  // Write value for GLOB_CMD_LO to issue Flash Backup
#define CMD_FLASHRST 0x04   // Write value for GLOB_CMD_LO to issue Flash Reset

// Write values for ADDR_SMPL_CTRL_HI to set Output Rate
#define CMD_RATE1000 0x02
#define CMD_RATE500 0x03
#define CMD_RATE200 0x04
#define CMD_RATE100 0x05
#define CMD_RATE50 0x06

// Write values for FILTER_CTRL_LO to set Filter
// The Filter setting should be set according to the sampling rate. Refer
// to the Accelerometer DataSheet to determine a valid filter setting.
#define CMD_FIRTAP64FC83 0x01
#define CMD_FIRTAP64FC220 0x02
#define CMD_FIRTAP128FC36 0x03
#define CMD_FIRTAP128FC110 0x04
#define CMD_FIRTAP128FC350 0x05
#define CMD_FIRTAP512FC9 0x06
#define CMD_FIRTAP512FC16 0x07
#define CMD_FIRTAP512FC60 0x08
#define CMD_FIRTAP512FC210 0x09
#define CMD_FIRTAP512FC460 0x0A
#define CMD_USERFIRTAP4 0x0B
#define CMD_USERFIRTAP64 0x0C
#define CMD_USERFIRTAP128 0x0D
#define CMD_USERFIRTAP512 0x0E

// Write values for ADDR_FIR_UCMD
// These values are used to read/write FIR user coefficient values
#define CMD_USERFIR_READ 0x01
#define CMD_USERFIR_WRITE 0x02

// MODE STAT
#define VAL_SAMPLING_MODE 0x00
#define VAL_CONFIG_MODE 0x04

#endif /* EPSONA352_H_ */
