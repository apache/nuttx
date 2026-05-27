/****************************************************************************
 * drivers/timers/pcf85263.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <time.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/timers/pcf85263.h>

#include "pcf85263.h"

#ifdef CONFIG_RTC_PCF85263

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* This RTC implementation supports only date/time RTC hardware */

#ifndef CONFIG_RTC_DATETIME
#  error CONFIG_RTC_DATETIME must be set to use this driver
#endif

#ifdef CONFIG_RTC_HIRES
#  error CONFIG_RTC_HIRES must NOT be set with this driver
#endif

#ifndef CONFIG_PCF85263_I2C_FREQUENCY
#  error CONFIG_PCF85263_I2C_FREQUENCY is not configured
#  define CONFIG_PCF85263_I2C_FREQUENCY 400000
#endif

#if CONFIG_PCF85263_I2C_FREQUENCY > 400000
#  error CONFIG_PCF85263_I2C_FREQUENCY is out of range
#endif

#define PCF85263_I2C_ADDRESS 0x51

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the PCF85263 chip.
 * Only a single RTC is supported.
 */

struct pcf85263_dev_s
{
  FAR struct i2c_master_s *i2c;  /* Contained reference to the I2C bus driver */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_rtc_enabled is set true after the RTC has successfully initialized */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The state of the PCF85263 chip.  Only a single RTC is supported */

static struct pcf85263_dev_s g_pcf85263;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
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
 ****************************************************************************/

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

/****************************************************************************
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
 ****************************************************************************/

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

/****************************************************************************
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
 ****************************************************************************/

static int rtc_bcd2bin(uint8_t value)
{
  int tens = ((int)value >> 4) * 10;
  return tens + (value & 0x0f);
}

/****************************************************************************
 * Name: pcf85263_write_reg
 *
 * Description:
 *   Write one or more registers starting at regaddr.
 *
 * Input Parameters:
 *   regaddr - The first register address to write.
 *   buf     - Data to write.
 *   len     - Number of bytes to write.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

static int pcf85263_write_reg(uint8_t regaddr, const uint8_t *buf,
                              uint8_t len)
{
  struct i2c_msg_s msg;
  uint8_t buffer[len + 1];
  int ret;

  buffer[0] = regaddr;
  memcpy(&buffer[1], buf, len);

  msg.frequency = CONFIG_PCF85263_I2C_FREQUENCY;
  msg.addr      = PCF85263_I2C_ADDRESS;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = len + 1;

  ret = I2C_TRANSFER(g_pcf85263.i2c, &msg, 1);
  if (ret < 0)
    {
      rtcerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: pcf85263_read_reg
 *
 * Description:
 *   Read one or more registers starting at regaddr.
 *
 * Input Parameters:
 *   regaddr - The first register address to read.
 *   buf     - Buffer to store the read data.
 *   len     - Number of bytes to read.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

static int pcf85263_read_reg(uint8_t regaddr, uint8_t *buf, uint8_t len)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = CONFIG_PCF85263_I2C_FREQUENCY;
  msg[0].addr      = PCF85263_I2C_ADDRESS;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_PCF85263_I2C_FREQUENCY;
  msg[1].addr      = PCF85263_I2C_ADDRESS;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buf;
  msg[1].length    = len;

  ret = I2C_TRANSFER(g_pcf85263.i2c, msg, 2);
  if (ret < 0)
    {
      rtcerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcf85263_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.
 *   This function is called once during the OS initialization sequence by
 *   board-specific logic.
 *
 *   After pcf85263_rtc_initialize() is called, the OS function
 *   clock_synchronize() should also be called to synchronize the system
 *   timer to a hardware RTC.  That operation is normally performed
 *   automatically by the system during clock initialization.
 *   However, when an external RTC is used, the board logic will need to
 *   explicitly re-synchronize the system timer to the RTC when the RTC
 *   becomes available.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int pcf85263_rtc_initialize(FAR struct i2c_master_s *i2c)
{
  uint8_t val = 0x00;
  int ret;

  /* Remember the i2c device and claim that the RTC is enabled */

  g_pcf85263.i2c = i2c;

  ret = pcf85263_write_reg(PCF85263_CTL_STOP_ENABLE, &val, 1);
  if (ret < 0)
    {
      return ret;
    }

  g_rtc_enabled  = true;
  return OK;
}

/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.
 *   This interface is only supported by the date/time RTC hardware
 *   implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS
 *   during initialization to set up the system time when CONFIG_RTC and
 *   CONFIG_RTC_DATETIME are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.
 *   That sub-second accuracy is lost in this interface.  However, since the
 *   system time is reinitialized on each power-up/reset, there will be no
 *   timing inaccuracy in the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_getdatetime(FAR struct tm *tp)
{
  uint8_t buffer[7];
  uint8_t seconds;
  int ret;

  /* If this function is called before the RTC has been initialized
   * (and it will be), then just return the data/time of the epoch,
   * 12:00 am, Jan 1, 1970.
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

  /* Read 7 registers starting at seconds, then re-read seconds to detect
   * a rollover during the burst read.
   */

  do
    {
      ret = pcf85263_read_reg(PCF85263_RTC_SECONDS, buffer, 7);
      if (ret < 0)
        {
          return ret;
        }

      ret = pcf85263_read_reg(PCF85263_RTC_SECONDS, &seconds, 1);
      if (ret < 0)
        {
          return ret;
        }
    }
  while ((buffer[0] & PCF85263_RTC_SECONDS_MASK) >
         (seconds & PCF85263_RTC_SECONDS_MASK));

  /* Format the return time */

  /* Return seconds (0-61) */

  tp->tm_sec = rtc_bcd2bin(buffer[0] & PCF85263_RTC_SECONDS_MASK);

  /* Return minutes (0-59) */

  tp->tm_min = rtc_bcd2bin(buffer[1] & PCF85263_RTC_MINUTES_MASK);

  /* Return hour (0-23).  This assumes 24-hour time was set. */

  tp->tm_hour = rtc_bcd2bin(buffer[2] & PCF85263_RTC_HOURS24_MASK);

  /* Return the day of the month (1-31) */

  tp->tm_mday = rtc_bcd2bin(buffer[3] & PCF85263_RTC_DAYS_MASK);

  /* Return the day of the week (0-6) */

  tp->tm_wday = (rtc_bcd2bin(buffer[4]) & PCF85263_RTC_WEEKDAYS_MASK);

  /* Return the month (0-11) */

  tp->tm_mon = rtc_bcd2bin(buffer[5] & PCF85263_RTC_MONTHS_MASK) - 1;

  /* Return the years since 1900.  The RTC will hold years since 1968
   * (a leap year like 2000).
   */

  tp->tm_year = rtc_bcd2bin(buffer[6]) + 68;

  rtc_dumptime(tp, "Returning");
  return OK;
}

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able
 *   to set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
  struct tm newtm;
  time_t newtime;
  uint8_t time_buf[8];
  uint8_t val;
  uint8_t seconds;
  int ret;

  /* If this function is called before the RTC has been initialized then just
   * return an error.
   */

  if (!g_rtc_enabled)
    {
      return -EAGAIN;
    }

  rtc_dumptime(tp, "Setting time");

  /* Get the broken out time */

  newtime = tp->tv_sec;
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

  /* Build the time register buffer starting at PCF85263_RTC_100TH_SECONDS */

  /* Clear the 100ths of seconds */

  time_buf[0] = 0;

  /* Save seconds (0-59) converted to BCD */

  time_buf[1] = rtc_bin2bcd(newtm.tm_sec);

  /* Save minutes (0-59) converted to BCD */

  time_buf[2] = rtc_bin2bcd(newtm.tm_min);

  /* Save hour (0-23) with 24-hour time indication */

  time_buf[3] = rtc_bin2bcd(newtm.tm_hour);

  /* Save the day of the month (1-31) */

  time_buf[4] = rtc_bin2bcd(newtm.tm_mday);

  /* Save the day of the week (1-7) */

  time_buf[5] = rtc_bin2bcd(newtm.tm_wday);

  /* Save the month (1-12) */

  time_buf[6] = rtc_bin2bcd(newtm.tm_mon + 1);

  /* Save the year.  Use years since 1968 (a leap year like 2000) */

  time_buf[7] = rtc_bin2bcd(newtm.tm_year - 68);

  /* Perform the transfer.  This transfer will be repeated if the seconds
   * count rolls over to a smaller value while writing.
   */

  do
    {
      /* Stop the RTC */

      val = 0x01;
      ret = pcf85263_write_reg(PCF85263_CTL_STOP_ENABLE, &val, 1);
      if (ret < 0)
        {
          return ret;
        }

      /* Register reset */

      val = 0xa4;
      ret = pcf85263_write_reg(PCF85263_CTL_RESET_REGISTER, &val, 1);
      if (ret < 0)
        {
          return ret;
        }

      /* Write all time registers in a single burst */

      ret = pcf85263_write_reg(PCF85263_RTC_100TH_SECONDS, time_buf, 8);
      if (ret < 0)
        {
          return ret;
        }

      /* Read back seconds to detect a rollover */

      ret = pcf85263_read_reg(PCF85263_RTC_SECONDS, &seconds, 1);
      if (ret < 0)
        {
          return ret;
        }

      /* Restart the RTC */

      val = 0x00;
      ret = pcf85263_write_reg(PCF85263_CTL_STOP_ENABLE, &val, 1);
      if (ret < 0)
        {
          return ret;
        }
    }
  while ((time_buf[1] & PCF85263_RTC_SECONDS_MASK) >
         (seconds & PCF85263_RTC_SECONDS_MASK));

  return OK;
}

#endif /* CONFIG_RTC_PCF85263 */
