/****************************************************************************
 * arch/arm64/src/imx9/imx9_rng.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>

#include <chip.h>

#include "arm64_internal.h"
#include "imx9_ele.h"

#if defined(CONFIG_IMX9_RNG)
#if defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(ARMV8A_DCACHE_LINESIZE) || ARMV8A_DCACHE_LINESIZE == 0
#  undef ARMV8A_DCACHE_LINESIZE
#  define ARMV8A_DCACHE_LINESIZE 64
#endif

/* The ELE writes the result by DMA, so the landing buffer is a whole number
 * of cache lines and nothing else shares them.
 */

#define RNG_BLOCKLEN ARMV8A_DCACHE_LINESIZE

/* Prefilled before every request. Zero could not be told apart from an
 * ELE that answered with zeros, so an untouched buffer reports separately.
 */

#define RNG_FILL 0xaa

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t imx9_rng_read(struct file *filep, char *buffer, size_t
                             buflen);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng_dev_s
{
  mutex_t rd_devlock;               /* Exclusive access to the ELE */
  uint8_t rd_lastval[RNG_BLOCKLEN]; /* Previous block, FIPS test */
  bool rd_first;                    /* No previous block yet */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rng_dev_s g_rngdev =
{
  .rd_devlock = NXMUTEX_INITIALIZER,
  .rd_first   = true,
};

static uint8_t g_rngbuf[RNG_BLOCKLEN]
  aligned_data(ARMV8A_DCACHE_LINESIZE);

static const struct file_operations g_rngops =
{
  NULL,           /* open */
  NULL,           /* close */
  imx9_rng_read,  /* read */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_rng_all
 *
 * Description:
 *   Rejecting a constant block costs nothing, since a genuine one has
 *   probability 2^-512, and it covers the first block, which the continuous
 *   test below cannot.
 *
 ****************************************************************************/

static bool imx9_rng_all(const uint8_t *buf, size_t len, uint8_t val)
{
  size_t i;

  for (i = 0; i < len; i++)
    {
      if (buf[i] != val)
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: imx9_rng_block
 *
 * Description:
 *   Fetch one RNG_BLOCKLEN block from the ELE into g_rngbuf and check it.
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure.  Never returns a block
 *   that failed a health check.
 *
 ****************************************************************************/

static int imx9_rng_block(void)
{
  int ret;

  memset(g_rngbuf, RNG_FILL, sizeof(g_rngbuf));

  ret = imx9_ele_get_random(g_rngbuf, sizeof(g_rngbuf));
  if (ret < 0)
    {
      _err("ERROR: ELE random request failed: %d\n", ret);
      return ret;
    }

  if (imx9_rng_all(g_rngbuf, sizeof(g_rngbuf), RNG_FILL))
    {
      _err("ERROR: buffer untouched; the ELE write never reached here\n");
      return -EIO;
    }

  if (imx9_rng_all(g_rngbuf, sizeof(g_rngbuf), 0))
    {
      _err("ERROR: ELE returned an all-zero block\n");
      return -EIO;
    }

  /* FIPS 140-2 continuous random number generator test: each block is
   * compared with the one before it and the test fails if they are equal.
   */

  if (g_rngdev.rd_first)
    {
      g_rngdev.rd_first = false;
    }
  else if (memcmp(g_rngdev.rd_lastval, g_rngbuf, sizeof(g_rngbuf)) == 0)
    {
      _err("ERROR: ELE repeated a block\n");
      return -EIO;
    }

  memcpy(g_rngdev.rd_lastval, g_rngbuf, sizeof(g_rngbuf));
  return OK;
}

/****************************************************************************
 * Name: imx9_rng_read
 ****************************************************************************/

static ssize_t imx9_rng_read(struct file *filep, char *buffer, size_t buflen)
{
  size_t done = 0;
  int ret;

  ret = nxmutex_lock(&g_rngdev.rd_devlock);
  if (ret < 0)
    {
      return ret;
    }

  while (done < buflen)
    {
      size_t chunk = buflen - done;

      ret = imx9_rng_block();
      if (ret < 0)
        {
          /* A short read is a lie about how much entropy the caller got, so
           * report the failure unless some was already delivered.
           */

          nxmutex_unlock(&g_rngdev.rd_devlock);
          return done > 0 ? (ssize_t)done : ret;
        }

      if (chunk > sizeof(g_rngbuf))
        {
          chunk = sizeof(g_rngbuf);
        }

      memcpy(buffer + done, g_rngbuf, chunk);
      done += chunk;
    }

  /* Leave nothing behind for the next caller to find. */

  memset(g_rngbuf, 0, sizeof(g_rngbuf));

  nxmutex_unlock(&g_rngdev.rd_devlock);
  return (ssize_t)done;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Register the /dev/random driver, backed by the ELE true random number
 *   generator.  Must be called BEFORE devurandom_register.
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
  register_driver("/dev/random", &g_rngops, 0444, NULL);
}
#endif

/****************************************************************************
 * Name: devurandom_register
 *
 * Description:
 *   Register /dev/urandom.  The ELE is the source for both nodes: it is a
 *   hardware generator, so there is nothing weaker to offer here.
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
  register_driver("/dev/urandom", &g_rngops, 0444, NULL);
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
#endif /* CONFIG_IMX9_RNG */
