/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_rng.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>

#include "riscv_internal.h"
#include "hardware/gd32vw55x_trng.h"
#include "hardware/gd32vw55x_rcu.h"

#if defined(CONFIG_GD32VW55X_RNG)
#if defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The TRNG generates a new 32-bit word every few TRNG clock cycles.  The
 * driver polls the DRDY status bit; bound the wait so that a stuck TRNG
 * cannot hang the caller forever.
 *
 * NOTE:  The TRNG shares its interrupt line with the HAU (hash accelerator),
 * so an interrupt driven implementation would have to co-operate with a
 * future HAU driver.  Since a new random word is available after only a few
 * hundred core cycles, this driver simply polls instead.
 */

#define TRNG_TIMEOUT_COUNT  100000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng_dev_s
{
  mutex_t  rd_devlock;   /* Threads can only exclusively access the TRNG */
  uint32_t rd_lastval;   /* Last value read (continuous test, FIPS 140-2) */
  bool     rd_first;     /* True: the first word has not been read yet */
  bool     rd_ready;     /* True: the TRNG has been initialized */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    gd32_rng_initialize(void);
static int     gd32_rng_getword(uint32_t *value);
static ssize_t gd32_rng_read(struct file *filep, char *buffer, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rng_dev_s g_rngdev =
{
  .rd_devlock = NXMUTEX_INITIALIZER,
  .rd_first   = true,
  .rd_ready   = false,
};

static const struct file_operations g_rngops =
{
  NULL,           /* open */
  NULL,           /* close */
  gd32_rng_read,  /* read */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_rng_initialize
 *
 * Description:
 *   Enable the TRNG clock and start the random number generator.
 *
 ****************************************************************************/

static void gd32_rng_initialize(void)
{
  uint32_t regval;

  if (g_rngdev.rd_ready)
    {
      return;
    }

  /* Enable the TRNG peripheral clock */

  regval  = getreg32(GD32VW55X_RCU_AHB2EN);
  regval |= RCU_AHB2EN_TRNGEN;
  putreg32(regval, GD32VW55X_RCU_AHB2EN);

  /* Reset the TRNG so that it starts from a known state */

  regval  = getreg32(GD32VW55X_RCU_AHB2RST);
  regval |= RCU_AHB2RST_TRNGRST;
  putreg32(regval, GD32VW55X_RCU_AHB2RST);

  regval &= ~RCU_AHB2RST_TRNGRST;
  putreg32(regval, GD32VW55X_RCU_AHB2RST);

  /* Enable the TRNG.  Interrupts are not used: the driver polls DRDY */

  regval  = getreg32(GD32VW55X_TRNG_CTL);
  regval |= TRNG_CTL_TRNGEN;
  putreg32(regval, GD32VW55X_TRNG_CTL);

  g_rngdev.rd_first = true;
  g_rngdev.rd_ready = true;
}

/****************************************************************************
 * Name: gd32_rng_getword
 *
 * Description:
 *   Read one valid 32-bit random word from the TRNG, handling the seed and
 *   clock error conditions and performing the continuous random number
 *   generator test required by FIPS PUB 140-2.
 *
 * Input Parameters:
 *   value - The location in which to return the random word.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32_rng_getword(uint32_t *value)
{
  uint32_t regval;
  uint32_t data;
  int retries;
  int i;

  for (retries = 0; retries < 8; retries++)
    {
      /* Wait for a random word to become available */

      for (i = 0; i < TRNG_TIMEOUT_COUNT; i++)
        {
          regval = getreg32(GD32VW55X_TRNG_STAT);
          if ((regval & (TRNG_STAT_DRDY | TRNG_STAT_CECS |
                         TRNG_STAT_SECS)) != 0)
            {
              break;
            }
        }

      if (i >= TRNG_TIMEOUT_COUNT)
        {
          _err("ERROR: TRNG timed out, stat=%08x\n", (unsigned)regval);
          return -ETIMEDOUT;
        }

      /* A clock error means that the TRNG clock is too slow.  There is
       * nothing that this driver can do about it.
       */

      if ((regval & TRNG_STAT_CECS) != 0)
        {
          _err("ERROR: TRNG clock error\n");
          putreg32(TRNG_STAT_CEIF, GD32VW55X_TRNG_STAT);
          return -EIO;
        }

      /* A seed error is recovered by restarting the TRNG */

      if ((regval & TRNG_STAT_SECS) != 0)
        {
          _warn("WARNING: TRNG seed error, restarting\n");

          putreg32(TRNG_STAT_SEIF, GD32VW55X_TRNG_STAT);

          regval  = getreg32(GD32VW55X_TRNG_CTL);
          regval &= ~TRNG_CTL_TRNGEN;
          putreg32(regval, GD32VW55X_TRNG_CTL);

          regval |= TRNG_CTL_TRNGEN;
          putreg32(regval, GD32VW55X_TRNG_CTL);

          g_rngdev.rd_first = true;
          continue;
        }

      data = getreg32(GD32VW55X_TRNG_DATA);

      /* As required by FIPS PUB 140-2, the first random number generated
       * after enabling the TRNG is not used, but is saved for comparison
       * with the next one.  Each subsequent random number is compared with
       * the previous one; the test fails if the two are equal.
       */

      if (g_rngdev.rd_first)
        {
          g_rngdev.rd_first   = false;
          g_rngdev.rd_lastval = data;
          continue;
        }

      if (g_rngdev.rd_lastval == data)
        {
          _warn("WARNING: Repeated random value, retrying\n");
          continue;
        }

      g_rngdev.rd_lastval = data;
      *value = data;
      return OK;
    }

  return -EIO;
}

/****************************************************************************
 * Name: gd32_rng_read
 *
 * Description:
 *   Fill the caller supplied buffer with random data.
 *
 ****************************************************************************/

static ssize_t gd32_rng_read(struct file *filep, char *buffer, size_t buflen)
{
  size_t nread = 0;
  uint32_t data;
  size_t nbytes;
  int ret;

  ret = nxmutex_lock(&g_rngdev.rd_devlock);
  if (ret < 0)
    {
      return ret;
    }

  while (nread < buflen)
    {
      ret = gd32_rng_getword(&data);
      if (ret < 0)
        {
          nxmutex_unlock(&g_rngdev.rd_devlock);
          return nread > 0 ? (ssize_t)nread : (ssize_t)ret;
        }

      nbytes = buflen - nread;
      if (nbytes > sizeof(uint32_t))
        {
          nbytes = sizeof(uint32_t);
        }

      memcpy(&buffer[nread], &data, nbytes);
      nread += nbytes;
    }

  nxmutex_unlock(&g_rngdev.rd_devlock);
  return (ssize_t)nread;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Initialize the TRNG hardware and register the /dev/random driver.
 *   Must be called BEFORE devurandom_register.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_RANDOM
void devrandom_register(void)
{
  gd32_rng_initialize();
  register_driver("/dev/random", &g_rngops, 0444, NULL);
}
#endif

/****************************************************************************
 * Name: devurandom_register
 *
 * Description:
 *   Register /dev/urandom
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_URANDOM_ARCH
void devurandom_register(void)
{
#ifndef CONFIG_DEV_RANDOM
  gd32_rng_initialize();
#endif
  register_driver("/dev/urandom", &g_rngops, 0444, NULL);
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
#endif /* CONFIG_GD32VW55X_RNG */
