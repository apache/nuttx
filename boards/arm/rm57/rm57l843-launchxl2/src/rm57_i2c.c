/****************************************************************************
 * boards/arm/rm57/rm57l843-launchxl2/src/rm57_i2c.c
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

/* I2C on the LAUNCHXL2-RM57L.
 *
 * Both modules come out on headers, on balls they share with other
 * functions (see board.h for the pin-mux entries):
 *
 *   I2C1  SDA J2.8, SCL J2.9 (BoosterPack site 1; MibSPI3 nCS[2]/nCS[3])
 *   I2C2  SDA J10.9, SCL J10.8 (proto header; also J14.30 / J14.36)
 *
 * Nothing on the board is an I2C device and there are no pull-up resistors,
 * so with nothing attached the buses idle on the pins' weak internal
 * pull-ups only.  The digital loopback self-test below needs neither.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>

#include "rm57_i2c.h"
#include "rm57l843-launchxl2.h"

#ifdef CONFIG_RM57_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Longest single write of the self-test, also used to time the bus */

#define I2CTEST_MAXBYTES    2048

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RM57_I2C_LOOPBACK
static uint8_t g_i2ctest_tx[I2CTEST_MAXBYTES];
static uint8_t g_i2ctest_echo[I2CTEST_MAXBYTES];

static const uint16_t g_i2ctest_lengths[] =
{
  1, 2, 16, 255
};

static const uint32_t g_i2ctest_frequencies[] =
{
  100000, 400000
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_RM57_I2C_LOOPBACK
/****************************************************************************
 * Name: rm57_i2c_check
 *
 * Description:
 *   Compare what came back with what was sent and report one test case.
 *
 * Returned Value:
 *   1 if the case failed, 0 if it passed.
 *
 ****************************************************************************/

static int rm57_i2c_check(int bus, struct i2c_master_s *i2c,
                          const char *name, uint32_t frequency, int ret,
                          size_t nbytes)
{
  size_t got = rm57_i2c_echocount(i2c);
  size_t bad = 0;
  size_t first = 0;
  size_t i;

  for (i = 0; i < MIN(got, nbytes); i++)
    {
      if (g_i2ctest_echo[i] != g_i2ctest_tx[i])
        {
          if (bad++ == 0)
            {
              first = i;
            }
        }
    }

  if (ret < 0 || got != nbytes || bad != 0)
    {
      syslog(LOG_ERR, "I2C%d: loopback %6" PRIu32 " Hz %-9s %4zu bytes: "
             "FAIL (ret %d, %zu back, %zu bad from %zu: %02x vs %02x)\n",
             bus, frequency, name, nbytes, ret, got, bad, first,
             g_i2ctest_echo[first], g_i2ctest_tx[first]);
      return 1;
    }

  syslog(LOG_INFO, "I2C%d: loopback %6" PRIu32 " Hz %-9s %4zu bytes: ok\n",
         bus, frequency, name, nbytes);
  return 0;
}

/****************************************************************************
 * Name: rm57_i2c_looptest
 *
 * Description:
 *   Run the module through I2CMDR.DLB: single writes of several lengths, a
 *   two-message write joined by a repeated START, two messages merged by
 *   I2C_M_NOSTART, an address-only probe, and one long write timed against
 *   the nominal SCL rate.  Master receive cannot be looped back.
 *
 ****************************************************************************/

static int rm57_i2c_looptest(int bus, struct i2c_master_s *i2c)
{
  struct i2c_msg_s msgs[2];
  struct timespec ts;
  uint32_t frequency;
  uint64_t elapsed;
  uint64_t nominal;
  clock_t start;
  int failures = 0;
  size_t nbytes;
  int ret;
  int f;
  int i;

  for (f = 0; f < nitems(g_i2ctest_frequencies); f++)
    {
      frequency = g_i2ctest_frequencies[f];

      memset(msgs, 0, sizeof(msgs));
      msgs[0].frequency = frequency;
      msgs[0].addr      = RM57_I2C_LOOPBACK_ADDR;
      msgs[0].buffer    = g_i2ctest_tx;
      msgs[1]           = msgs[0];

      /* Single writes.  The pattern changes with position and length, so
       * a byte that is dropped, repeated or left over shows up.
       */

      for (i = 0; i < nitems(g_i2ctest_lengths); i++)
        {
          nbytes = g_i2ctest_lengths[i];
          for (size_t j = 0; j < nbytes; j++)
            {
              g_i2ctest_tx[j] = (uint8_t)(j * 7 + nbytes + f);
            }

          rm57_i2c_loopback(i2c, g_i2ctest_echo, sizeof(g_i2ctest_echo));
          msgs[0].length = nbytes;
          ret = I2C_TRANSFER(i2c, msgs, 1);
          failures += rm57_i2c_check(bus, i2c, "write", frequency, ret,
                                     nbytes);
        }

      /* Repeated START: the first segment ends on ARDY with the bus held */

      rm57_i2c_loopback(i2c, g_i2ctest_echo, sizeof(g_i2ctest_echo));
      msgs[0].length = 8;
      msgs[1].buffer = g_i2ctest_tx + 8;
      msgs[1].length = 8;
      msgs[1].flags  = 0;
      ret = I2C_TRANSFER(i2c, msgs, 2);
      failures += rm57_i2c_check(bus, i2c, "restart", frequency, ret, 16);

      /* I2C_M_NOSTART: one segment over two buffers */

      rm57_i2c_loopback(i2c, g_i2ctest_echo, sizeof(g_i2ctest_echo));
      msgs[0].length = 5;
      msgs[1].buffer = g_i2ctest_tx + 5;
      msgs[1].length = 11;
      msgs[1].flags  = I2C_M_NOSTART;
      ret = I2C_TRANSFER(i2c, msgs, 2);
      failures += rm57_i2c_check(bus, i2c, "nostart", frequency, ret, 16);

      /* Address-only probe */

      rm57_i2c_loopback(i2c, g_i2ctest_echo, sizeof(g_i2ctest_echo));
      msgs[0].length = 0;
      ret = I2C_TRANSFER(i2c, msgs, 1);
      failures += rm57_i2c_check(bus, i2c, "probe", frequency, ret, 0);

      /* Timing: nine SCL periods per byte plus the address byte */

      for (size_t j = 0; j < I2CTEST_MAXBYTES; j++)
        {
          g_i2ctest_tx[j] = (uint8_t)(j ^ (j >> 8));
        }

      rm57_i2c_loopback(i2c, g_i2ctest_echo, sizeof(g_i2ctest_echo));
      msgs[0].length = I2CTEST_MAXBYTES;
      start = up_perf_gettime();
      ret = I2C_TRANSFER(i2c, msgs, 1);
      up_perf_convert(up_perf_gettime() - start, &ts);
      failures += rm57_i2c_check(bus, i2c, "timed", frequency, ret,
                                 I2CTEST_MAXBYTES);

#ifdef CONFIG_ARCH_PERF_EVENTS
      elapsed = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
      nominal = (uint64_t)(I2CTEST_MAXBYTES + 1) * 9 * 1000000 / frequency;
      syslog(LOG_INFO, "I2C%d: %4d bytes at %6" PRIu32 " Hz took %" PRIu64
             " us, nominal %" PRIu64 " us (%" PRIu64 "%%)\n",
             bus, I2CTEST_MAXBYTES, frequency, elapsed, nominal,
             elapsed * 100 / nominal);
#else
      /* rm57_boot.c only starts the PMU cycle counter with perf events */

      UNUSED(elapsed);
      UNUSED(nominal);
      syslog(LOG_INFO, "I2C%d: timing needs CONFIG_ARCH_PERF_EVENTS\n",
             bus);
#endif
    }

  rm57_i2c_loopback(i2c, NULL, 0);
  syslog(failures != 0 ? LOG_ERR : LOG_INFO,
         "I2C%d: loopback self-test %s (%d failed)\n",
         bus, failures != 0 ? "FAILED" : "passed", failures);

  return failures == 0 ? OK : -EIO;
}
#endif

/****************************************************************************
 * Name: rm57_i2c_bus
 *
 * Description:
 *   Initialize one bus, self-test it and register it.
 *
 ****************************************************************************/

static int rm57_i2c_bus(int bus)
{
  struct i2c_master_s *i2c;
  int ret = OK;

  i2c = rm57_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      i2cerr("ERROR: I2C%d: initialization failed\n", bus);
      return -ENODEV;
    }

#ifdef CONFIG_RM57_I2C_LOOPBACK
  /* Reported, not returned: a failed self-test should not take the rest of
   * the board bring-up down with it.
   */

  rm57_i2c_looptest(bus, i2c);
#endif

#ifdef CONFIG_I2C_DRIVER
  ret = i2c_register(i2c, bus);
  if (ret < 0)
    {
      i2cerr("ERROR: I2C%d: i2c_register failed: %d\n", bus, ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_i2c_setup
 *
 * Description:
 *   Bring up every enabled I2C bus.  Called from rm57_bringup().
 *
 ****************************************************************************/

int rm57_i2c_setup(void)
{
  int ret = OK;

#ifdef CONFIG_RM57_I2C1
  ret = rm57_i2c_bus(1);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_RM57_I2C2
  ret = rm57_i2c_bus(2);
#endif

  return ret;
}

#endif /* CONFIG_RM57_I2C */
