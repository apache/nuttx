/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_rng.c
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

#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>

#include "hardware/lpc54_rom.h"

#if defined(CONFIG_LPC54_RNG)
#if defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t lpc54_read(struct file *filep, char *buffer, size_t);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng_dev_s
{
  sem_t rd_devsem;      /* Threads can only exclusively access the RNG */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rng_dev_s g_rngdev;

static const struct file_operations g_rngops =
{
  NULL,            /* open */
  NULL,            /* close */
  lpc54_read,      /* read */
  NULL,            /* write */
  NULL,            /* seek */
  NULL,            /* ioctl */
  NULL             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_read
 ****************************************************************************/

static ssize_t lpc54_read(struct file *filep, char *buffer, size_t buflen)
{
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } value;

  ssize_t remaining;
  int ret;
  int i;

  /* Get exclusive access to ROM random number generator API */

  ret = nxsem_wait(&g_rngdev.rd_devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Copy the requested number of randome bytes. */

  for (remaining = buflen;
       remaining > 0;
      )
    {
      /* Read the next 32-bit random value */

      value.w = LPC54_RNG_READ();

      /* Return byte at a time to avoid alignment complexities (but
       * sacrificing some performance).
       */

      for (i = 0; i < sizeof(uint32_t) && remaining > 0; i++, remaining--)
        {
          *buffer++ = value.b[i];
        }
    }

  nxsem_post(&g_rngdev.rd_devsem);
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
  nxsem_init(&g_rngdev.rd_devsem, 0, 1);
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
  nxsem_init(&g_rngdev.rd_devsem, 0, 1);
#endif
  register_driver("/dev/urandom", &g_rngops, 0444, NULL);
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
#endif /* CONFIG_LPC54_RNG */
