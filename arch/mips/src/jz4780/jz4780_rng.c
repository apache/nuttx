/****************************************************************************
 * arch/mips/src/jz4780/jz4780_rng.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/param.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/drivers/drivers.h>

#include "mips_internal.h"

#if defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ENTROPY_DELAY 10

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int jz_rng_initialize(void);
static ssize_t jz_rng_read(struct file *filep, char *buffer, size_t);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng_dev_s
{
  mutex_t rd_lock;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rng_dev_s g_rngdev =
{
  .rd_lock = NXMUTEX_INITIALIZER,
};

static const struct file_operations g_rngops =
{
  NULL,            /* open */
  NULL,            /* close */
  jz_rng_read,     /* read */
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: jz_rng_initialize
 *
 * Description:
 *   Enable the random number generator
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

static int jz_rng_initialize(void)
{
  uint32_t retval;
  struct rng_dev_s *priv = (struct rng_dev_s *)&g_rngdev;

  if (nxmutex_lock(&priv->rd_lock) != OK)
    {
      return -EBUSY;
    }

  /* Enable RNG if it is disabled */

  if ((getreg32(ERNG_REG) & ENABLE_RNG) == 0)
    {
      putreg32(ENABLE_RNG, ERNG_REG);
    }

  up_mdelay(1);

  retval = (getreg32(ERNG_REG) & ENABLE_RNG) ? OK : -EIO;

  if (retval != OK)
    {
      _err("Failed to initialize RNG\n");
    }

  nxmutex_unlock(&priv->rd_lock);
  return retval;
}

/****************************************************************************
 * Name: jz_rng_read
 *
 * Description:
 *   This is the standard, NuttX character driver read method
 *
 * Input Parameters:
 *   filep - The VFS file instance
 *   buffer - Buffer in which to return the random samples
 *   buflen - The length of the buffer
 *
 * Returned Value:
 *
 ****************************************************************************/

static ssize_t jz_rng_read(struct file *filep, char *buffer, size_t buflen)
{
  struct rng_dev_s *priv = (struct rng_dev_s *)&g_rngdev;
  ssize_t bytes_left = buflen;
  uint8_t *rd_buf = (uint8_t *)buffer;
  uint32_t last_word;

  if (nxmutex_lock(&priv->rd_lock) != OK)
    {
      return -EBUSY;
    }

  last_word = getreg32(RNG_REG);
  while (bytes_left > 0)
    {
      up_mdelay(ENTROPY_DELAY);
      uint32_t word = getreg32(RNG_REG);
      if (word == last_word)
        {
          /* Try again if the same number is repeated */

          continue;
        }

      last_word = word;

      uint32_t count = MIN(sizeof(word), bytes_left);

      memcpy(rd_buf, &word, count);
      rd_buf += count;
      bytes_left -= count;
    }

  nxmutex_unlock(&priv->rd_lock);
  return buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Initialize the RNG hardware and register the /dev/random driver.
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
  if (jz_rng_initialize() == OK)
    {
      register_driver("/dev/random", &g_rngops, 0444, NULL);
    }
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
  if (jz_rng_initialize() == OK)
    {
      register_driver("/dev/urandom", &g_rngops, 0444, NULL);
    }
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
