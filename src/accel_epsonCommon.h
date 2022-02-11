//==============================================================================
//
//  accel_epsonCommon.h - Epson Accelerometer specific definitions common
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
#ifndef EPSONCOMMON_H_
#define EPSONCOMMON_H_

#include <math.h>
#include <stdio.h>

#define TRUE (1)
#define FALSE (0)

// for extra debug messages, un-comment the DEBUG #define
//#define DEBUG (1)

#ifdef A352
#include "accel_epsonA352.h"
#endif

// defines for Epson Accelerometer specific delays
#define DELAY_EPSON_RESET 10      // Milliseconds Reset Pulse Width
#define EPSON_POWER_ON_DELAY 900  // Milliseconds
#define EPSON_FLASH_TEST_DELAY 5  // Milliseconds
#define EPSON_SELF_TEST_DELAY 80  // Milliseconds
#define EPSON_FILTER_DELAY 1      // Milliseconds

#define EpsonStall() \
  seDelayMicroSecs(  \
      EPSON_STALL)  // Required delay between bus cycles for serial timings

//======================================================================
// EpsonAcclOptions{}
// This structure is holds many possible settings for a wide range of
// Epson Accelerometer products.
//
// This is so the structures used in demonstration code will be similar
// and portable between products. Not all accelerometer products will use all
// fields. Most fields correspond to a bitfield value from the accelerometer
// registers. For more information on the bitfield values, refer to the
// Datasheet for the Epson Accelerometer.
//======================================================================
struct EpsonAcclOptions {
  // SIG_CTRL
  int out_sel_x, out_sel_y,
      out_sel_z;   // for x,y,z: 0=Acceleration, 1=Tilt angle
  int mesmod_sel;  // measurement cond: 0=standard noise floor, 1=reduced noise
                   // floor
  int temp_stabil;  // bias stabilization against thermal shock

  // MSC_CTRL
  int ext_sel;   // EXT function: 0=External trigger disabled, 1=enabled
  int ext_pol;   // EXT polarity: 0=rising edge, 1=falling edge
  int drdy_on;   // DRDY function: 0=DRDY disabled, 1=enabled
  int drdy_pol;  // DRDY polarity: 0=Active low, 1=Active high

  // SMPL_CTRL
  int dout_rate;  // see accel_epsonXXXX.h for details

  // FILTER_CTRL
  int filter_sel;  // see accel_epsonXXXX.h for details

  // BURST_CTRL1
  int flag_out, temp_out, accel_out;
  int count_out, checksum_out;

};

//======================================================================
// EpsonAcclData{}
// This structure is designed to hold a variety of possible data for a
// wide range of Epson Accelerometer products.
//
// This is so the structures used in demonstration code will be similar
// and portable between products. Not all accelerometer products will use all
// fields.
//======================================================================
struct EpsonAcclData {
  unsigned short ndflags;
  float temperature;
  float accel_x, accel_y, accel_z;
  int count;
};


//======================================================================
// Accelerometer Functions
// The following functions must be implemented for each Accelerometer MODEL.
//
// - Functions with common code are found in accel_epsonCommon.c
// - Functions that vary based on the interface are found in
//   accel_epsonXXXX.c (i.e. accel_epsonUart.c)
// - Functions that are specific to a Accelerometer MODEL are found in
//   accel_epsonXXXX.c (i.e. accel_epsonA352.c)
//
//======================================================================

// common functions
int acclHWReset(void);
int acclPowerOn(void);
void acclStart(void);
void acclStop(void);
void acclReset(void);
int acclFlashTest(void);
int acclSelfTest(void);
void acclDummyWrite(void);
unsigned int acclDataByteLength(struct EpsonAcclOptions);

// low-level interface functions
int acclDataReady(void);
int acclDataReadyOptions(struct EpsonAcclOptions);
void registerWriteByteNoId(unsigned char, unsigned char,
                       unsigned int);
void registerWriteByte(unsigned char, unsigned char, unsigned char,
                       unsigned int);
unsigned short registerRead16NoId(unsigned char, unsigned int);
unsigned short registerRead16(unsigned char, unsigned char, unsigned int);
int acclDataReadBurstNOptions(struct EpsonAcclOptions, struct EpsonAcclData*);
void populateEpsonData(struct EpsonAcclOptions, struct EpsonAcclData*);

// MODEL specific functions
void registerDump(void);
int acclInitOptions(struct EpsonAcclOptions);
int writeCoefficients(int);       // private helper function
int testCoefWriteComplete(void);  // private helper function

#endif /* EPSONCOMMON_H_ */
