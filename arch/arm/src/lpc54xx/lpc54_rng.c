/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_rng.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include "chip/lpc54_rom.h"

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
  NULL             /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL           /* poll */
#endif
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

  for (remaining = buflen; remaining > 0;)
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
  (void)register_driver("/dev/random", &g_rngops, 0444, NULL);
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
  (void)register_driver("/dev/urandom", &g_rngops, 0444, NULL);
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
#endif /* CONFIG_LPC54_RNG */
