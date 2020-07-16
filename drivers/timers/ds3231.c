/************************************************************************************
 * drivers/timers/ds3231.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/timers/ds3231.h>

#include "ds3231.h"

#ifdef CONFIG_RTC_DSXXXX

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

/* This RTC implementation supports only date/time RTC hardware */

#ifndef CONFIG_RTC_DATETIME
#  error CONFIG_RTC_DATETIME must be set to use this driver
#endif

#ifdef CONFIG_RTC_HIRES
#  error CONFIG_RTC_HIRES must NOT be set with this driver
#endif

#ifndef CONFIG_DS3231_I2C_FREQUENCY
#  error CONFIG_DS3231_I2C_FREQUENCY is not configured
#  define CONFIG_DS3231_I2C_FREQUENCY 400000
#endif

#if CONFIG_DS3231_I2C_FREQUENCY > 400000
#  error CONFIG_DS3231_I2C_FREQUENCY is out of range
#endif

#define DS3231_I2C_ADDRESS 0x68

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This structure describes the state of the DS3231 chip.  Only a single RTC is
 * supported.
 */

struct ds3231_dev_s
{
  FAR struct i2c_master_s *i2c;  /* Contained reference to the I2C bus driver */
};

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* g_rtc_enabled is set true after the RTC has successfully initialized */

volatile bool g_rtc_enabled = false;

/************************************************************************************
 * Private Data
 ************************************************************************************/

/* The state of the DS3231 chip.  Only a single RTC is supported */

static struct ds3231_dev_s g_ds3231;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: rtc_dumptime
 *
 * Description:
 *   Show the broken out time.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumptime(FAR struct tm *tp, FAR const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("   tm_sec: %08x\n", tp->tm_sec);
  rtcinfo("   tm_min: %08x\n", tp->tm_min);
  rtcinfo("  tm_hour: %08x\n", tp->tm_hour);
  rtcinfo("  tm_mday: %08x\n", tp->tm_mday);
  rtcinfo("   tm_mon: %08x\n", tp->tm_mon);
  rtcinfo("  tm_year: %08x\n", tp->tm_year);
  rtcinfo("  tm_wday: %08x\n", tp->tm_wday);
  rtcinfo("  tm_yday: %08x\n", tp->tm_yday);
  rtcinfo(" tm_isdst: %08x\n", tp->tm_isdst);
}
#else
#  define rtc_dumptime(tp, msg)
#endif

/************************************************************************************
 * Name: rtc_bin2bcd
 *
 * Description:
 *   Converts a 2 digit binary to BCD format
 *
 * Input Parameters:
 *   value - The byte to be converted.
 *
 * Returned Value:
 *   The value in BCD representation
 *
 ************************************************************************************/

static uint8_t rtc_bin2bcd(int value)
{
  uint8_t msbcd = 0;

  while (value >= 10)
    {
      msbcd++;
      value -= 10;
    }

  return (msbcd << 4) | value;
}

/************************************************************************************
 * Name: rtc_bcd2bin
 *
 * Description:
 *   Convert from 2 digit BCD to binary.
 *
 * Input Parameters:
 *   value - The BCD value to be converted.
 *
 * Returned Value:
 *   The value in binary representation
 *
 ************************************************************************************/

static int rtc_bcd2bin(uint8_t value)
{
  int tens = ((int)value >> 4) * 10;
  return tens + (value & 0x0f);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: dsxxxx_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This function is
 *   called once during the OS initialization sequence by board-specific logic.
 *
 *   After dsxxxx_rtc_initialize() is called, the OS function clock_synchronize()
 *   should also be called to synchronize the system timer to a hardware RTC.  That
 *   operation is normally performed automatically by the system during clock
 *   initialization.  However, when an external RTC is used, the board logic will
 *   need to explicitly re-synchronize the system timer to the RTC when the RTC
 *   becomes available.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int dsxxxx_rtc_initialize(FAR struct i2c_master_s *i2c)
{
  /* Remember the i2c device and claim that the RTC is enabled */

  g_ds3231.i2c  = i2c;
  g_rtc_enabled = true;
  return OK;
}

/************************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC and CONFIG_RTC_DATETIME
 *   are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.  That
 *   sub-second accuracy is lost in this interface.  However, since the system time
 *   is reinitialized on each power-up/reset, there will be no timing inaccuracy in
 *   the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_getdatetime(FAR struct tm *tp)
{
  struct i2c_msg_s msg[4];
  uint8_t secaddr;
  uint8_t buffer[7];
  uint8_t seconds;
  int tmp;
  int ret;

  /* If this function is called before the RTC has been initialized (and it will be),
   * then just return the data/time of the epoch, 12:00 am, Jan 1, 1970.
   */

  if (!g_rtc_enabled)
    {
      tp->tm_sec  = 0;
      tp->tm_min  = 0;
      tp->tm_hour = 0;

      /* Jan 1, 1970 was a Thursday */

      tp->tm_wday = 4;
      tp->tm_mday = 1;
      tp->tm_mon  = 0;
      tp->tm_year = 70;
      return -EAGAIN;
    }

  /* Select to begin reading at the seconds register */

  secaddr          = DSXXXX_TIME_SECR;

  msg[0].frequency = CONFIG_DS3231_I2C_FREQUENCY;
  msg[0].addr      = DS3231_I2C_ADDRESS;
  msg[0].flags     = 0;
  msg[0].buffer    = &secaddr;
  msg[0].length    = 1;

  /* Set up to read 7 registers: secondss, minutes, hour, day-of-week, date,
   * month, year
   */

  msg[1].frequency = CONFIG_DS3231_I2C_FREQUENCY;
  msg[1].addr      = DS3231_I2C_ADDRESS;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = 7;

  /* Read the seconds register again */

  msg[2].frequency = CONFIG_DS3231_I2C_FREQUENCY;
  msg[2].addr      = DS3231_I2C_ADDRESS;
  msg[2].flags     = 0;
  msg[2].buffer    = &secaddr;
  msg[2].length    = 1;

  msg[3].frequency = CONFIG_DS3231_I2C_FREQUENCY;
  msg[3].addr      = DS3231_I2C_ADDRESS;
  msg[3].flags     = I2C_M_READ;
  msg[3].buffer    = &seconds;
  msg[3].length    = 1;

  /* Perform the transfer.  The transfer may be performed repeatedly of the
   * seconds values decreases, meaning that that was a rollover in the seconds.
   */

  do
    {
      ret = I2C_TRANSFER(g_ds3231.i2c, msg, 4);
      if (ret < 0)
        {
          rtcerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
          return ret;
        }
    }
  while ((buffer[0] & DSXXXX_TIME_SEC_BCDMASK) >
         (seconds & DSXXXX_TIME_SEC_BCDMASK));

  /* Format the return time */

  /* Return seconds (0-61) */

  tp->tm_sec = rtc_bcd2bin(buffer[0] & DSXXXX_TIME_SEC_BCDMASK);

  /* Return minutes (0-59) */

  tp->tm_min = rtc_bcd2bin(buffer[1] & DSXXXX_TIME_MIN_BCDMASK);

  /* Return hour (0-23).  This assumes 24-hour time was set. */

  tp->tm_hour = rtc_bcd2bin(buffer[2] & DSXXXX_TIME_HOUR24_BCDMASK);

  /* Return the day of the week (0-6) */

  tp->tm_wday = (rtc_bcd2bin(buffer[3]) & DSXXXX_TIME_DAY_MASK) - 1;

  /* Return the day of the month (1-31) */

  tp->tm_mday = rtc_bcd2bin(buffer[4] & DSXXXX_TIME_DATE_BCDMASK);

  /* Return the month (0-11) */

  tp->tm_mon = rtc_bcd2bin(buffer[5] & DSXXXX_TIME_MONTH_BCDMASK) - 1;

  /* Return the years since 1900 */

  tmp = rtc_bcd2bin(buffer[6] & DSXXXX_TIME_YEAR_BCDMASK);

#if defined(CONFIG_RTC_DS3231) || defined(CONFIG_RTC_DS3232)
  if ((buffer[5] & DS323X_TIME_CENTURY_MASK) == DS323X_TIME_1900)
    {
      tp->tm_year = tmp;
    }
  else
    {
      tp->tm_year = tmp + 100;
    }
#else
  /* No century indicator.  The RTC will hold years since 1968 (a leap year like
   * 2000)
   */

  tp->tm_year = tmp + 68;
#endif

  rtc_dumptime(tp, "Returning");
  return OK;
}

/************************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able to
 *   set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
  struct i2c_msg_s msg[3];
  struct tm newtm;
  time_t newtime;
  uint8_t buffer[8];
  uint8_t seconds;
  uint8_t century;
  uint8_t year;
  int ret;

  /* If this function is called before the RTC has been initialized then just return
   * an error.
   */

  if (!g_rtc_enabled)
    {
      return -EAGAIN;
    }

  rtcinfo("Setting time tp=(%d,%d)\n", (int)tp->tv_sec, (int)tp->tv_nsec);

  /* Get the broken out time */

  newtime = (time_t)tp->tv_sec;
  if (tp->tv_nsec >= 500000000)
    {
      /* Round up */

      newtime++;
    }

  if (localtime_r(&newtime, &newtm) == NULL)
    {
      rtcerr("ERROR: localtime_r failed\n");
      return -EINVAL;
    }

  rtc_dumptime(&newtm, "New time");

  /* Construct the message */

  /* Write starting with the seconds register */

  buffer[0] = DSXXXX_TIME_SECR;

  /* Save seconds (0-59) converted to BCD */

  buffer[1] = rtc_bin2bcd(newtm.tm_sec);

  /* Save minutes (0-59) converted to BCD */

  buffer[2] = rtc_bin2bcd(newtm.tm_min);

  /* Save hour (0-23) with 24-hour time indication */

  buffer[3] = rtc_bin2bcd(newtm.tm_hour) | DSXXXX_TIME_24;

  /* Save the day of the week (1-7) */

  buffer[4] = rtc_bin2bcd(newtm.tm_wday + 1);

  /* Save the day of the month (1-31) */

  buffer[5] = rtc_bin2bcd(newtm.tm_mday);

#if defined(CONFIG_RTC_DS3231) || defined(CONFIG_RTC_DS3232)
  /* Handle years in the 20th vs the 21st century */

  if (newtm.tm_year < 100)
    {
      /* Convert years in the range 1900-1999 */

      century = DS323X_TIME_1900;
      year    = newtm.tm_year;
    }
  else
    {
      /* Convert years in the range 2000-2099 */

      century = DS323X_TIME_2000;
      year    = newtm.tm_year - 100;
    }

#else
  /* Use years since 1968 (a leap year like 2000) */

  century = 0;
  year    = newtm.tm_year - 68;
#endif

  /* Save the month (1-12) with century */

  buffer[6] = rtc_bin2bcd(newtm.tm_mon + 1) | century;

  /* Save the year */

  buffer[7] = rtc_bin2bcd(year);

  /* Setup the I2C message */

  msg[0].frequency = CONFIG_DS3231_I2C_FREQUENCY;
  msg[0].addr      = DS3231_I2C_ADDRESS;
  msg[0].flags     = 0;
  msg[0].buffer    = buffer;
  msg[0].length    = 8;

  /* Read back the seconds register */

  msg[1].frequency = CONFIG_DS3231_I2C_FREQUENCY;
  msg[1].addr      = DS3231_I2C_ADDRESS;
  msg[1].flags     = 0;
  msg[1].buffer    = buffer;
  msg[1].length    = 1;

  msg[2].frequency = CONFIG_DS3231_I2C_FREQUENCY;
  msg[2].addr      = DS3231_I2C_ADDRESS;
  msg[2].flags     = I2C_M_READ;
  msg[2].buffer    = &seconds;
  msg[2].length    = 1;

  /* Perform the transfer.  This transfer will be repeated if the seconds
   * count rolls over to a smaller value while writing.
   */

  do
    {
      ret = I2C_TRANSFER(g_ds3231.i2c, msg, 3);
      if (ret < 0)
        {
          rtcerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
          return ret;
        }
    }
  while ((buffer[1] & DSXXXX_TIME_SEC_BCDMASK) >
         (seconds & DSXXXX_TIME_SEC_BCDMASK));

  return OK;
}

#endif /* CONFIG_RTC_DSXXXX */
