/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_rng.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/drivers/drivers.h>

#include "arm_internal.h"
#include "hardware/rp23xx_trng.h"

#if defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The TRNG delivers entropy 192 bits (six 32-bit EHR words) at a time. */

#define RP23XX_TRNG_EHR_WORDS  6
#define RP23XX_TRNG_EHR_BYTES  (RP23XX_TRNG_EHR_WORDS * sizeof(uint32_t))

/* TRNG_VALID.EHR_VALID: the EHR holds a fresh 192-bit sample. */

#define RP23XX_TRNG_EHR_VALID  (1 << 0)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t rp23xx_rng_read(struct file *filep, char *buffer,
                               size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_rng_lock = NXMUTEX_INITIALIZER;

static const struct file_operations g_rngops =
{
  NULL,             /* open */
  NULL,             /* close */
  rp23xx_rng_read,  /* read */
  NULL,             /* write */
  NULL,             /* seek */
  NULL,             /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_rng_read
 *
 * Description:
 *   Fill 'buffer' with 'buflen' bytes of hardware entropy.  Enables the ring
 *   -oscillator source, then repeatedly waits for a valid 192-bit EHR and
 *   copies it out until the request is satisfied.
 *
 ****************************************************************************/

static ssize_t rp23xx_rng_read(struct file *filep, char *buffer,
                               size_t buflen)
{
  size_t nread = 0;
  int ret;

  ret = nxmutex_lock(&g_rng_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable the entropy source. */

  putreg32(1, RP23XX_TRNG_RND_SOURCE_ENABLE);

  while (nread < buflen)
    {
      uint32_t ehr[RP23XX_TRNG_EHR_WORDS];
      size_t chunk;
      int i;

      /* Wait for the next 192-bit sample to become valid. */

      while ((getreg32(RP23XX_TRNG_TRNG_VALID) & RP23XX_TRNG_EHR_VALID) == 0)
        {
        }

      for (i = 0; i < RP23XX_TRNG_EHR_WORDS; i++)
        {
          ehr[i] = getreg32(RP23XX_TRNG_EHR_DATA(i));
        }

      /* Acknowledge the sample so the engine collects the next one. */

      putreg32(0xffffffff, RP23XX_TRNG_RNG_ICR);

      chunk = buflen - nread;
      if (chunk > RP23XX_TRNG_EHR_BYTES)
        {
          chunk = RP23XX_TRNG_EHR_BYTES;
        }

      memcpy(buffer + nread, ehr, chunk);
      nread += chunk;
    }

  /* Leave the source disabled to save power. */

  putreg32(0, RP23XX_TRNG_RND_SOURCE_ENABLE);

  nxmutex_unlock(&g_rng_lock);
  return (ssize_t)nread;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Register /dev/random, backed by the RP2350 hardware TRNG.  Called
 *   automatically from drivers_initialize() when CONFIG_DEV_RANDOM is set.
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
 *   Register /dev/urandom, backed by the same hardware TRNG.  Called from
 *   drivers_initialize() when CONFIG_DEV_URANDOM selects the arch source.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_URANDOM_ARCH
void devurandom_register(void)
{
  register_driver("/dev/urandom", &g_rngops, 0444, NULL);
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
