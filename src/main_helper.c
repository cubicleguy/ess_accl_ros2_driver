//==============================================================================
//
//  main_helper.c - Epson Accelerometer helper functions for console utilities
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

#include "accel_epsonCommon.h"
#include "main_helper.h"

/*****************************************************************************
** Function name:       printHeaderRow
** Description:         Output header row
**                      based on EpsonAcclOptions
** Parameters:          fp File pointer to send the output data
**                      struct EpsonAcclOptions
** Return value:        None
**
*****************************************************************************/
void printHeaderRow(FILE *fp, struct EpsonAcclOptions options) {
  fprintf(fp, "\r\nsample[dec]");
  if (options.flag_out) {
    fprintf(fp, ",\t nd[hex]");
  }

  if (options.temp_out) {
    fprintf(fp, ", tempC[degC]");
  }

  if (options.accel_out) {
    // check for acceleration vs. tilt for each axis
    if (options.out_sel_x == 0)
      fprintf(fp, ",\t ax[mG]");
    else
      fprintf(fp, ",\t tiltx[mrad]");
    if (options.out_sel_y == 0)
      fprintf(fp, ",\t ay[mG]");
    else
      fprintf(fp, ",\t tilty[mrad]");
    if (options.out_sel_z == 0)
      fprintf(fp, ",\t az[mG]");
    else
      fprintf(fp, ",\t tiltz[mrad]");
  }

  if (options.count_out) {
    fprintf(fp, ",\t count[dec]");
  }
  fprintf(fp, "\n");
}

/*****************************************************************************
** Function name:       printSensorRow
** Description:         Prints formatted row of sensor data based on how it is
*configured.
** Parameters:          fp - file pointer to send data
**                      options - The struct describing how the ACCL is
*configured.
**                      epson_data - The struct that is filled with data from
*the ACCL.
**                      sample_count - Typically and incrementing index for each
*sample
** Return value:        none
** Notes:
******************************************************************************/
void printSensorRow(FILE *fp, struct EpsonAcclOptions options,
                    struct EpsonAcclData *epson_data, int sample_count) {
  fprintf(fp, "%08d", sample_count);
  if (options.flag_out) {
    fprintf(fp, ",\t %04x", epson_data->ndflags);
  }

  if (options.temp_out) {
    fprintf(fp, ",\t %8.3f", epson_data->temperature);
  }

  if (options.accel_out) {
    // check for acceleration vs. tilt for each axis
    if (options.out_sel_x == 0)
      fprintf(fp, ",\t %8.5f", epson_data->accel_x * 1000);
    else
      fprintf(fp, ",\t %8.5f", epson_data->accel_x * 1000);
    if (options.out_sel_y == 0)
      fprintf(fp, ",\t %8.5f", epson_data->accel_y * 1000);
    else
      fprintf(fp, ",\t %8.5f", epson_data->accel_y * 1000);
    if (options.out_sel_z == 0)
      fprintf(fp, ",\t %8.5f", epson_data->accel_z * 1000);
    else
      fprintf(fp, ",\t %8.5f", epson_data->accel_z * 1000);
  }

  if (options.count_out) {
    fprintf(fp, ",\t %09d", epson_data->count);
  }
  fprintf(fp, "\n");
}
