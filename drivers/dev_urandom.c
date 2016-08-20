/****************************************************************************
 * drivers/dev_urandom.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: David S. Alessio
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

/* This random number generator is simple, fast and portable.
 *      Ref:  https://en.wikipedia.org/wiki/Xorshift
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <errno.h>

#include <nuttx/lib/lib.h>
#include <nuttx/lib/xorshift128.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>

#if defined(CONFIG_DEV_URANDOM) && !defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_DEV_URANDOM_CONGRUENTIAL) && \
    !defined(CONFIG_DEV_URANDOM_XORSHIFT128)
#  define CONFIG_DEV_URANDOM_XORSHIFT128 1
#endif

#ifdef CONFIG_DEV_URANDOM_XORSHIFT128
#  define PRNG() xorshift128()
#else /* CONFIG_DEV_URANDOM_CONGRUENTIAL */
#  define PRNG() congruential()
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct xorshift128_state_s
{
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint32_t w;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#define min(a, b)   (((a) < (b)) ? (a) : (b))

static ssize_t devurand_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t devurand_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);
#ifndef CONFIG_DISABLE_POLL
static int devurand_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_urand_fops =
{
  NULL,                         /* open */
  NULL,                         /* close */
  devurand_read,                /* read */
  devurand_write,               /* write */
  NULL,                         /* seek */
  NULL                          /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , devurand_poll               /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: congruential
 ****************************************************************************/

#ifdef CONFIG_DEV_URANDOM_CONGRUENTIAL
static uint32_t congruential(void)
{
  /* REVISIT:  We could probably generate a 32-bit value with a single
   * call to nrand().
   */

  return (uint32_t)nrand(65536L) << (uint32_t)nrand(65536L);
}
#endif

/****************************************************************************
 * Name: devurand_read
 ****************************************************************************/

static ssize_t devurand_read(FAR struct file *filep, FAR char *buffer,
                             size_t len)
{
  size_t n;
  uint32_t rnd;

  n = len;

  /* Align buffer pointer to 4-byte boundry */

  if (((uintptr_t)buffer & 0x03) != 0)
    {
      /* Generate a pseudo random number */

      rnd = PRNG();

      while (((uintptr_t)buffer & 0x03) != 0)
        {
          if (n <= 0)
            {
              return len;
            }

          *buffer++ = rnd & 0xff;
          rnd >>= 8;
          --n;
        }
    }

  /* Stuff buffer with PRNGs 4 bytes at a time */

  while (n >= 4)
    {
      *(uint32_t *)buffer = PRNG();
      buffer += 4;
      n -= 4;
    }

  /* Stuff remaining 1, 2, or 3 bytes */

  if (n > 0)
    {
      /* Generate a pseudo random number */

      rnd = PRNG();

      do
        {
          *buffer++ = rnd & 0xFF;
          rnd >>= 8;
        }
      while (--n > 0);
    }

  return len;
}

/****************************************************************************
 * Name: devurand_write
 ****************************************************************************/

static ssize_t devurand_write(FAR struct file *filep, FAR const char *buffer,
                              size_t len)
{
  /* Write can be used to re-seed the PRNG state. */

#ifdef CONFIG_DEV_URANDOM_CONGRUENTIAL
  unsigned int seed = 0;

  if (len < sizeof(unsigned int))
    {
      return -ERANGE;
    }

  memcpy(&seed, buffer, sizeof(unsigned int));
  srand(seed);
  return sizeof(unsigned int);

#else
  struct xorshift128_state_s seed;

  if (len < sizeof(struct xorshift128_state_s))
    {
      return -ERANGE;
    }

  memcpy(&seed, buffer, sizeof(struct xorshift128_state_s));
  xorshift128_seed(seed.w, seed.x, seed.y, seed.z);
  return sizeof(struct xorshift128_state_s);
#endif
}

/****************************************************************************
 * Name: devurand_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int devurand_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  if (setup)
    {
      fds->revents |= (fds->events & (POLLIN | POLLOUT));
      if (fds->revents != 0)
        {
          sem_post(fds->sem);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devurandom_register
 *
 * Description:
 *   Register /dev/urandom
 *
 ****************************************************************************/

void devurandom_register(void)
{
#ifdef CONFIG_DEV_URANDOM_CONGRUENTIAL
  srand(10197);
#else
  /* Seed the PRNG */

  xorshift128_seed(97, 101, 97 << 17, 101 << 25);
#endif

  (void)register_driver("/dev/urandom", &g_urand_fops, 0666, NULL);
}

#endif /* CONFIG_DEV_URANDOM && CONFIG_DEV_URANDOM_ARCH */
