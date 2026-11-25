/****************************************************************************
 * drivers/timers/pcf85063.c
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
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/timers/pcf85063.h>

#ifdef CONFIG_RTC_PCF85063

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

#ifndef CONFIG_PCF85063_I2C_FREQUENCY
#  define CONFIG_PCF85063_I2C_FREQUENCY 400000
#endif

#if CONFIG_PCF85063_I2C_FREQUENCY > 400000
#  error CONFIG_PCF85063_I2C_FREQUENCY is out of range
#endif

#ifndef CONFIG_PCF85063_I2C_ADDRESS
#  define CONFIG_PCF85063_I2C_ADDRESS 0x51
#endif

/* Register map.  All time and date registers are BCD, except the weekday,
 * which is a plain 0-6 count.
 */

#define PCF85063_CTRL1             0x00  /* Control_1 */
#define PCF85063_CTRL2             0x01  /* Control_2 */
#define PCF85063_OFFSET            0x02  /* Frequency offset correction */
#define PCF85063_RAM_BYTE          0x03  /* One byte of free RAM */
#define PCF85063_SECONDS           0x04  /* OS flag + seconds */
#define PCF85063_MINUTES           0x05
#define PCF85063_HOURS             0x06
#define PCF85063_DAYS              0x07
#define PCF85063_WEEKDAYS          0x08
#define PCF85063_MONTHS            0x09
#define PCF85063_YEARS             0x0a

/* The PCF85063A additionally has alarm registers at 0x0b-0x0f and a
 * countdown timer at 0x10-0x11.  The PCF85063TP has neither, and they are
 * not used by this driver.
 */

/* Control_1 bits */

#define PCF85063_CTRL1_CAP_SEL     (1 << 0)  /* 1: 12.5 pF, 0: 7 pF load */
#define PCF85063_CTRL1_12_24       (1 << 1)  /* 1: 12-hour mode */
#define PCF85063_CTRL1_CIE         (1 << 2)  /* Correction interrupt enable */
#define PCF85063_CTRL1_SR          (1 << 4)  /* Software reset */
#define PCF85063_CTRL1_STOP        (1 << 5)  /* Stop the RTC clock */
#define PCF85063_CTRL1_EXT_TEST    (1 << 7)  /* External clock test mode */

/* Control_1 bits that are board configuration rather than driver state and
 * are therefore preserved as found.  Everything else is written as zero.
 */

#define PCF85063_CTRL1_KEEP        (PCF85063_CTRL1_CAP_SEL | \
                                    PCF85063_CTRL1_CIE)

/* Seconds register: bit 7 is the oscillator stop (OS) flag.  It is set by
 * the hardware when the oscillator stops (e.g. on power loss with no
 * backup supply) and is cleared by writing the seconds register.
 */

#define PCF85063_SECONDS_OS        (1 << 7)
#define PCF85063_SECONDS_MASK      0x7f
#define PCF85063_MINUTES_MASK      0x7f
#define PCF85063_HOURS24_MASK      0x3f
#define PCF85063_HOURS12_MASK      0x1f
#define PCF85063_HOURS12_PM        (1 << 5)
#define PCF85063_DAYS_MASK         0x3f
#define PCF85063_WEEKDAYS_MASK     0x07
#define PCF85063_MONTHS_MASK       0x1f

/* Number of time/date registers, seconds through years */

#define PCF85063_TIME_NREGS        7

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the PCF85063 chip.
 * Only a single RTC is supported.
 */

struct pcf85063_dev_s
{
  FAR struct i2c_master_s *i2c;  /* Contained reference to the I2C bus driver */
  uint8_t ctrl1;                 /* Control_1 value with the clock running */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_rtc_enabled is set true after the RTC has successfully initialized */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The state of the PCF85063 chip.  Only a single RTC is supported */

static struct pcf85063_dev_s g_pcf85063;

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
 *   tp  - The time to show
 *   msg - A message to show before the time
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumptime(FAR const struct tm *tp, FAR const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("   tm_sec: %08x\n", tp->tm_sec);
  rtcinfo("   tm_min: %08x\n", tp->tm_min);
  rtcinfo("  tm_hour: %08x\n", tp->tm_hour);
  rtcinfo("  tm_mday: %08x\n", tp->tm_mday);
  rtcinfo("   tm_mon: %08x\n", tp->tm_mon);
  rtcinfo("  tm_year: %08x\n", tp->tm_year);
  rtcinfo("  tm_wday: %08x\n", tp->tm_wday);
}
#else
#  define rtc_dumptime(tp, msg)
#endif

/****************************************************************************
 * Name: rtc_bin2bcd
 *
 * Description:
 *   Convert a binary value in the range 0-99 to two BCD digits.
 *
 * Input Parameters:
 *   value - The value to convert
 *
 * Returned Value:
 *   The value in BCD.
 *
 ****************************************************************************/

static uint8_t rtc_bin2bcd(int value)
{
  return (uint8_t)(((value / 10) << 4) | (value % 10));
}

/****************************************************************************
 * Name: rtc_bcd2bin
 *
 * Description:
 *   Convert two BCD digits to binary.
 *
 * Input Parameters:
 *   value - The BCD value to convert
 *
 * Returned Value:
 *   The value in binary.
 *
 ****************************************************************************/

static int rtc_bcd2bin(uint8_t value)
{
  return ((value >> 4) * 10) + (value & 0x0f);
}

/****************************************************************************
 * Name: pcf85063_read_reg
 *
 * Description:
 *   Read consecutive registers starting at regaddr.  The chip freezes its
 *   time counters for the duration of the access, so a burst read of the
 *   time registers is always consistent.
 *
 * Input Parameters:
 *   regaddr - The first register to read
 *   buffer  - Where to return the register contents
 *   buflen  - The number of registers to read
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

static int pcf85063_read_reg(uint8_t regaddr, FAR uint8_t *buffer,
                             size_t buflen)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = CONFIG_PCF85063_I2C_FREQUENCY;
  msg[0].addr      = CONFIG_PCF85063_I2C_ADDRESS;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_PCF85063_I2C_FREQUENCY;
  msg[1].addr      = CONFIG_PCF85063_I2C_ADDRESS;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = buflen;

  ret = I2C_TRANSFER(g_pcf85063.i2c, msg, 2);
  if (ret < 0)
    {
      rtcerr("ERROR: I2C read of register 0x%02x failed: %d\n",
             regaddr, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: pcf85063_write_reg
 *
 * Description:
 *   Write consecutive registers starting at regaddr.  The register address
 *   and the data are sent as a single I2C message, because not every I2C
 *   controller supports continuing a message with I2C_M_NOSTART.
 *
 * Input Parameters:
 *   regaddr - The first register to write
 *   buffer  - The values to write
 *   buflen  - The number of registers to write
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

static int pcf85063_write_reg(uint8_t regaddr, FAR const uint8_t *buffer,
                              size_t buflen)
{
  struct i2c_msg_s msg;
  uint8_t txbuffer[PCF85063_TIME_NREGS + 1];
  int ret;

  if (buflen > PCF85063_TIME_NREGS)
    {
      return -EINVAL;
    }

  txbuffer[0] = regaddr;
  memcpy(&txbuffer[1], buffer, buflen);

  msg.frequency = CONFIG_PCF85063_I2C_FREQUENCY;
  msg.addr      = CONFIG_PCF85063_I2C_ADDRESS;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = buflen + 1;

  ret = I2C_TRANSFER(g_pcf85063.i2c, &msg, 1);
  if (ret < 0)
    {
      rtcerr("ERROR: I2C write of register 0x%02x failed: %d\n",
             regaddr, ret);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcf85063_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.
 *   This function is called once during the OS initialization sequence by
 *   board-specific logic.
 *
 *   After pcf85063_rtc_initialize() is called, the OS function
 *   clock_synchronize() should also be called to synchronize the system
 *   timer to a hardware RTC.
 *
 * Input Parameters:
 *   i2c - The I2C bus the chip is attached to
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int pcf85063_rtc_initialize(FAR struct i2c_master_s *i2c)
{
  uint8_t ctrl1;
  uint8_t hours;
  uint8_t val;
  int hour;
  int ret;

  DEBUGASSERT(i2c != NULL);

  g_pcf85063.i2c = i2c;

  /* Reading Control_1 also checks that the chip answers on the bus */

  ret = pcf85063_read_reg(PCF85063_CTRL1, &ctrl1, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* The running configuration: 24-hour mode, clock running, no test mode.
   * The load capacitance selection depends on the crystal fitted to the
   * board, so it is kept as found, as is the correction interrupt enable.
   */

  g_pcf85063.ctrl1 = ctrl1 & PCF85063_CTRL1_KEEP;

  if ((ctrl1 & PCF85063_CTRL1_12_24) != 0)
    {
      /* The chip was left in 12-hour mode.  The hours register is not
       * converted by the hardware when the mode changes, so convert it
       * here while the clock is stopped.
       */

      val = g_pcf85063.ctrl1 | PCF85063_CTRL1_STOP | PCF85063_CTRL1_12_24;
      ret = pcf85063_write_reg(PCF85063_CTRL1, &val, 1);
      if (ret < 0)
        {
          return ret;
        }

      ret = pcf85063_read_reg(PCF85063_HOURS, &hours, 1);
      if (ret < 0)
        {
          return ret;
        }

      hour = rtc_bcd2bin(hours & PCF85063_HOURS12_MASK) % 12;
      if ((hours & PCF85063_HOURS12_PM) != 0)
        {
          hour += 12;
        }

      val = g_pcf85063.ctrl1 | PCF85063_CTRL1_STOP;
      ret = pcf85063_write_reg(PCF85063_CTRL1, &val, 1);
      if (ret < 0)
        {
          return ret;
        }

      hours = rtc_bin2bcd(hour);
      ret = pcf85063_write_reg(PCF85063_HOURS, &hours, 1);
      if (ret < 0)
        {
          return ret;
        }
    }

  if ((ctrl1 & PCF85063_CTRL1_STOP) != 0)
    {
      rtcwarn("WARNING: RTC was stopped, restarting it\n");
    }

  /* Write the running configuration, which also restarts the clock */

  if (ctrl1 != g_pcf85063.ctrl1)
    {
      ret = pcf85063_write_reg(PCF85063_CTRL1, &g_pcf85063.ctrl1, 1);
      if (ret < 0)
        {
          return ret;
        }
    }

  g_rtc_enabled = true;
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
 * Input Parameters:
 *   tp - The location to return the time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.  -EAGAIN means that
 *   the RTC has not been initialized yet (tp then holds the epoch).
 *   -ENODATA means that the oscillator has stopped since the time was last
 *   set, so the time held by the chip is not valid.
 *
 ****************************************************************************/

int up_rtc_getdatetime(FAR struct tm *tp)
{
  uint8_t buffer[PCF85063_TIME_NREGS];
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

  /* Read seconds through years in a single burst.  The chip freezes the
   * time counters during the access, so no rollover check is needed.
   */

  ret = pcf85063_read_reg(PCF85063_SECONDS, buffer, sizeof(buffer));
  if (ret < 0)
    {
      return ret;
    }

  /* The OS flag means the oscillator has stopped since the time was last
   * set (e.g. a flat or missing backup supply), so the time is not valid.
   */

  if ((buffer[0] & PCF85063_SECONDS_OS) != 0)
    {
      rtcwarn("WARNING: oscillator stopped, time is not valid\n");
      return -ENODATA;
    }

  tp->tm_sec  = rtc_bcd2bin(buffer[0] & PCF85063_SECONDS_MASK);
  tp->tm_min  = rtc_bcd2bin(buffer[1] & PCF85063_MINUTES_MASK);
  tp->tm_hour = rtc_bcd2bin(buffer[2] & PCF85063_HOURS24_MASK);
  tp->tm_mday = rtc_bcd2bin(buffer[3] & PCF85063_DAYS_MASK);
  tp->tm_wday = buffer[4] & PCF85063_WEEKDAYS_MASK;
  tp->tm_mon  = rtc_bcd2bin(buffer[5] & PCF85063_MONTHS_MASK) - 1;

  /* The chip holds two digits of year and treats every year divisible by
   * four as a leap year, which is correct for 2000-2099 only.  Years since
   * 1900 is therefore the register value plus 100.
   */

  tp->tm_year  = rtc_bcd2bin(buffer[6]) + 100;
  tp->tm_isdst = 0;

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
 *   Zero (OK) on success; a negated errno on failure.  -EINVAL is returned
 *   for a date outside 2000-2099, which the chip cannot represent.
 *
 ****************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
  uint8_t buffer[PCF85063_TIME_NREGS];
  struct tm newtm;
  uint8_t val;
  int ret2;
  int ret;

  DEBUGASSERT(tp != NULL);

  /* If this function is called before the RTC has been initialized then
   * just return an error.
   */

  if (!g_rtc_enabled)
    {
      return -EAGAIN;
    }

  if (gmtime_r(&tp->tv_sec, &newtm) == NULL)
    {
      rtcerr("ERROR: gmtime_r failed\n");
      return -EINVAL;
    }

  rtc_dumptime(&newtm, "New time");

  if (newtm.tm_year < 100 || newtm.tm_year > 199)
    {
      rtcerr("ERROR: year %d is outside 2000-2099\n",
             newtm.tm_year + 1900);
      return -EINVAL;
    }

  /* Writing the seconds register with bit 7 clear also clears the OS flag,
   * marking the time as valid again.
   */

  buffer[0] = rtc_bin2bcd(newtm.tm_sec) & PCF85063_SECONDS_MASK;
  buffer[1] = rtc_bin2bcd(newtm.tm_min);
  buffer[2] = rtc_bin2bcd(newtm.tm_hour);
  buffer[3] = rtc_bin2bcd(newtm.tm_mday);
  buffer[4] = newtm.tm_wday & PCF85063_WEEKDAYS_MASK;
  buffer[5] = rtc_bin2bcd(newtm.tm_mon + 1);
  buffer[6] = rtc_bin2bcd(newtm.tm_year - 100);

  /* Stop the clock while writing.  This also resets the prescaler, so the
   * new time starts counting from the moment the clock is released.
   */

  val = g_pcf85063.ctrl1 | PCF85063_CTRL1_STOP;
  ret = pcf85063_write_reg(PCF85063_CTRL1, &val, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = pcf85063_write_reg(PCF85063_SECONDS, buffer, sizeof(buffer));

  /* Restart the clock even if the time write failed, so that a bus error
   * does not leave the RTC stopped.
   */

  val  = g_pcf85063.ctrl1;
  ret2 = pcf85063_write_reg(PCF85063_CTRL1, &val, 1);

  return ret < 0 ? ret : ret2;
}

#endif /* CONFIG_RTC_PCF85063 */
