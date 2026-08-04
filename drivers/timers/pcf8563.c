/****************************************************************************
 * drivers/timers/pcf8563.c
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
#include <time.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/timers/pcf8563.h>

#ifdef CONFIG_RTC_PCF8563

#ifndef CONFIG_RTC_DATETIME
#  error CONFIG_RTC_DATETIME must be set to use this driver
#endif

#ifdef CONFIG_RTC_HIRES
#  error CONFIG_RTC_HIRES must NOT be set with this driver
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register map.  Everything that carries a time is BCD. */

#define PCF8563_CTRL1        0x00
#define PCF8563_CTRL2        0x01
#define PCF8563_SECONDS      0x02
#define PCF8563_MINUTES      0x03
#define PCF8563_HOURS        0x04
#define PCF8563_DAY          0x05
#define PCF8563_WEEKDAY      0x06
#define PCF8563_MONTH        0x07
#define PCF8563_YEAR         0x08

/* Alarm at 0x09 to 0x0c, clock output at 0x0d, and a countdown timer at
 * 0x0e and 0x0f.  None is used here.  They are written down because
 * knowing where they are is most of the work of adding them, and because
 * a reader wondering whether an alarm exists deserves an answer.
 */

#define PCF8563_ALARM_MIN    0x09
#define PCF8563_ALARM_HOUR   0x0a
#define PCF8563_ALARM_DAY    0x0b
#define PCF8563_ALARM_WEEK   0x0c
#define PCF8563_CLKOUT       0x0d
#define PCF8563_TIMER_CTRL   0x0e
#define PCF8563_TIMER        0x0f

/* The seconds register carries a flag rather than a tenth digit: the
 * oscillator has stopped at some point since the time was last set, so
 * what the rest of the register file says is not to be believed.
 */

#define PCF8563_SEC_VL       0x80
#define PCF8563_SEC_MASK     0x7f

#define PCF8563_MIN_MASK     0x7f
#define PCF8563_HOUR_MASK    0x3f
#define PCF8563_DAY_MASK     0x3f
#define PCF8563_WEEKDAY_MASK 0x07
#define PCF8563_MONTH_MASK   0x1f
#define PCF8563_YEAR_MASK    0xff

/* The month register's top bit marks a century rollover.  Which value
 * stands for which century is not settled between parts, so this driver
 * does not try to read one out of it.  See pcf8563_getdatetime().
 */

#define PCF8563_MONTH_C      0x80

/* Stopping the counters while the registers are written keeps a carry
 * from landing in the middle of the write.
 */

#define PCF8563_CTRL1_STOP   0x20

#ifndef CONFIG_PCF8563_I2C_FREQUENCY
#  define CONFIG_PCF8563_I2C_FREQUENCY 100000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pcf8563_dev_s
{
  FAR struct i2c_master_s *i2c;

  /* The century bit exactly as it was last seen, so that it can be
   * written back the same way round.  See pcf8563_getdatetime().
   */

  bool c_polarity;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pcf8563_getdatetime(FAR struct pcf8563_dev_s *priv,
                               FAR struct tm *tp);
static int pcf8563_setdatetime(FAR struct pcf8563_dev_s *priv,
                               FAR const struct tm *tp);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Only one of these is supported, because up_rtc_getdatetime() takes no
 * argument saying which.
 */

static struct pcf8563_dev_s g_pcf8563;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Set once the chip is bound to a bus.  Read by up_rtc_getdatetime(), which
 * the system may call before that has happened.
 */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtc_bin2bcd
 *
 * Description:
 *   Convert a binary value to the two BCD digits the chip stores.
 *
 * Input Parameters:
 *   value - The value to convert, 0 to 99
 *
 * Returned Value:
 *   The value in BCD.
 *
 ****************************************************************************/

static uint8_t rtc_bin2bcd(int value)
{
  int msbcd = 0;

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
 *   Convert two BCD digits read from the chip to binary.
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
  int tens = ((int)value >> 4) * 10;
  return tens + (value & 0x0f);
}

/****************************************************************************
 * Name: pcf8563_getreg
 *
 * Description:
 *   Read consecutive registers, addressing the first and letting the
 *   chip's address pointer run on through the rest.
 *
 * Input Parameters:
 *   priv    - The driver state
 *   regaddr - The first register to read
 *   buffer  - Where to return the register contents
 *   buflen  - How many registers to read
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int pcf8563_getreg(FAR struct pcf8563_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *buffer, size_t buflen)
{
  struct i2c_msg_s msg[2];

  msg[0].frequency = CONFIG_PCF8563_I2C_FREQUENCY;
  msg[0].addr      = PCF8563_I2C_ADDRESS;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_PCF8563_I2C_FREQUENCY;
  msg[1].addr      = PCF8563_I2C_ADDRESS;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = buflen;

  return I2C_TRANSFER(priv->i2c, msg, 2);
}

/****************************************************************************
 * Name: pcf8563_putreg
 *
 * Description:
 *   Write consecutive registers as a single bus transaction, so the
 *   chip sees one write rather than one per register.
 *
 * Input Parameters:
 *   priv    - The driver state
 *   regaddr - The first register to write
 *   buffer  - The values to write
 *   buflen  - How many registers to write
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

static int pcf8563_putreg(FAR struct pcf8563_dev_s *priv, uint8_t regaddr,
                          FAR const uint8_t *buffer, size_t buflen)
{
  struct i2c_msg_s msg[2];

  msg[0].frequency = CONFIG_PCF8563_I2C_FREQUENCY;
  msg[0].addr      = PCF8563_I2C_ADDRESS;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  /* The second message runs on from the first without a stop, which is
   * what makes the pair one write rather than two.
   */

  msg[1].frequency = CONFIG_PCF8563_I2C_FREQUENCY;
  msg[1].addr      = PCF8563_I2C_ADDRESS;
  msg[1].flags     = I2C_M_NOSTART;
  msg[1].buffer    = (FAR uint8_t *)buffer;
  msg[1].length    = buflen;

  return I2C_TRANSFER(priv->i2c, msg, 2);
}

/****************************************************************************
 * Name: pcf8563_getdatetime
 *
 * Description:
 *   Read the time, refusing to hand back one that the chip says is not
 *   trustworthy.
 *
 *   The century bit is recorded here so that a later write can put it back
 *   unchanged.  A read that fails leaves it as it was.
 *
 * Input Parameters:
 *   priv - The driver state
 *   tp   - Where to return the time
 *
 * Returned Value:
 *   Zero on success, -ENODATA if the oscillator has stopped since the time
 *   was last set, or a negated errno on a bus failure.
 *
 ****************************************************************************/

static int pcf8563_getdatetime(FAR struct pcf8563_dev_s *priv,
                               FAR struct tm *tp)
{
  uint8_t buffer[7];
  int year;
  int ret;

  ret = pcf8563_getreg(priv, PCF8563_SECONDS, buffer, sizeof(buffer));
  if (ret < 0)
    {
      rtcerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return ret;
    }

  /* The oscillator has stopped since the time was last set, so every
   * register after this one holds whatever it happened to stop on.  A
   * flat backup cell and a board that has never had its clock set both
   * arrive here, and in each case an answer would be worse than an
   * error: the caller can do something sensible about not knowing the
   * time, and nothing sensible about being told the wrong one.
   */

  if ((buffer[0] & PCF8563_SEC_VL) != 0)
    {
      rtcwarn("WARNING: oscillator stopped, the time is not known\n");
      return -ENODATA;
    }

  tp->tm_sec  = rtc_bcd2bin(buffer[0] & PCF8563_SEC_MASK);
  tp->tm_min  = rtc_bcd2bin(buffer[1] & PCF8563_MIN_MASK);
  tp->tm_hour = rtc_bcd2bin(buffer[2] & PCF8563_HOUR_MASK);
  tp->tm_mday = rtc_bcd2bin(buffer[3] & PCF8563_DAY_MASK);
  tp->tm_wday = buffer[4] & PCF8563_WEEKDAY_MASK;
  tp->tm_mon  = rtc_bcd2bin(buffer[5] & PCF8563_MONTH_MASK) - 1;

  /* The part holds two digits of year and one bit saying a century has
   * rolled over.  Which value of that bit stands for which century is a
   * convention rather than a rule, and parts disagree, so no century is
   * read out of it: this is treated as a clock for the years 2000 to
   * 2099, which is the same thing the mainline Linux driver settled on.
   *
   * The bit is remembered exactly as found so that a write puts it back
   * the way this part expects it.  Interpreting it would mean guessing;
   * preserving it cannot be wrong.
   */

  year = rtc_bcd2bin(buffer[6] & PCF8563_YEAR_MASK);

  priv->c_polarity = (buffer[5] & PCF8563_MONTH_C) != 0;

  tp->tm_year  = year + 100;
  tp->tm_isdst = 0;
  return OK;
}

/****************************************************************************
 * Name: pcf8563_setdatetime
 *
 * Description:
 *   Set the time, stopping the counters across the write so that a carry
 *   cannot land between the seconds and the minutes.
 *
 * Input Parameters:
 *   priv - The driver state
 *   tp   - The time to set
 *
 * Returned Value:
 *   Zero on success, -EINVAL for a year outside 2000 to 2099, or a
 *   negated errno on a bus failure.  The counters are restarted either
 *   way.
 *
 ****************************************************************************/

static int pcf8563_setdatetime(FAR struct pcf8563_dev_s *priv,
                               FAR const struct tm *tp)
{
  uint8_t buffer[7];
  uint8_t ctrl1;
  int year;
  int ret;

  /* Two digits of year and no century that can be trusted means this part
   * covers 2000 to 2099 and nothing else.  Refuse anything outside rather
   * than store a year that reads back as a different one.
   */

  year = tp->tm_year;
  if (year < 100 || year > 199)
    {
      rtcerr("ERROR: year %d is outside 2000 to 2099\n", year + 1900);
      return -EINVAL;
    }

  /* Stop the counters, so that a carry cannot land between the seconds
   * being written and the minutes.
   */

  ctrl1 = PCF8563_CTRL1_STOP;
  ret = pcf8563_putreg(priv, PCF8563_CTRL1, &ctrl1, 1);
  if (ret < 0)
    {
      rtcerr("ERROR: cannot stop the clock: %d\n", ret);
      return ret;
    }

  /* Writing the seconds clears the flag that says the time is unknown,
   * because after this it is known.
   */

  buffer[0] = rtc_bin2bcd(tp->tm_sec) & PCF8563_SEC_MASK;
  buffer[1] = rtc_bin2bcd(tp->tm_min);
  buffer[2] = rtc_bin2bcd(tp->tm_hour);
  buffer[3] = rtc_bin2bcd(tp->tm_mday);
  buffer[4] = tp->tm_wday & PCF8563_WEEKDAY_MASK;
  buffer[5] = rtc_bin2bcd(tp->tm_mon + 1);
  buffer[6] = rtc_bin2bcd(year - 100);

  /* Restore the century bit as last read, so whatever convention this part
   * follows survives being set.  Nothing is interpreted from it, so a chip
   * whose time has never been read back writes it as zero.
   */

  if (priv->c_polarity)
    {
      buffer[5] |= PCF8563_MONTH_C;
    }

  ret = pcf8563_putreg(priv, PCF8563_SECONDS, buffer, sizeof(buffer));
  if (ret < 0)
    {
      rtcerr("ERROR: cannot write the time: %d\n", ret);
    }

  /* Start the counters again whatever happened above, or a failed write
   * leaves a stopped clock.
   */

  ctrl1 = 0;
  pcf8563_putreg(priv, PCF8563_CTRL1, &ctrl1, 1);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcf8563_rtc_initialize
 *
 * Description:
 *   Bind the chip to an I2C bus and make it the system's clock.  Board
 *   logic calls this once the bus exists, since an I2C bus is not available
 *   as early as clock_initialize() runs.
 *
 * Input Parameters:
 *   i2c - The bus the chip is on
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.
 *
 ****************************************************************************/

int pcf8563_rtc_initialize(FAR struct i2c_master_s *i2c)
{
  FAR struct pcf8563_dev_s *priv = &g_pcf8563;

  DEBUGASSERT(i2c != NULL);

  priv->i2c        = i2c;
  priv->c_polarity = false;

  g_rtc_enabled = true;
  return OK;
}

/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the RTC.  This is the interface the
 *   system uses to start its own clock when the only clock is external.
 *
 * Input Parameters:
 *   tp - Where to return the time
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.  -EAGAIN means no bus
 *   has been bound yet, and tp holds the epoch.
 *
 ****************************************************************************/

int up_rtc_getdatetime(FAR struct tm *tp)
{
  /* The system asks for the time before any board has had a chance to
   * bind a bus, and has to be given something.  The epoch is the
   * conventional answer, along with the error that says so.
   */

  if (!g_rtc_enabled)
    {
      tp->tm_sec  = 0;
      tp->tm_min  = 0;
      tp->tm_hour = 0;

      /* The 1st of January 1970 was a Thursday */

      tp->tm_wday = 4;
      tp->tm_mday = 1;
      tp->tm_mon  = 0;
      tp->tm_year = 70;
      return -EAGAIN;
    }

  return pcf8563_getdatetime(&g_pcf8563, tp);
}

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC from a timespec.  Sub-second accuracy is dropped: this
 *   chip counts in whole seconds.
 *
 * Input Parameters:
 *   tp - The time to set
 *
 * Returned Value:
 *   Zero on success, or a negated errno on failure.  -EAGAIN means no bus
 *   has been bound yet.
 *
 ****************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
  struct tm newtime;

  DEBUGASSERT(tp != NULL);

  if (!g_rtc_enabled)
    {
      return -EAGAIN;
    }

  gmtime_r(&tp->tv_sec, &newtime);
  return pcf8563_setdatetime(&g_pcf8563, &newtime);
}

#endif /* CONFIG_RTC_PCF8563 */
