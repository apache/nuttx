/****************************************************************************
 * drivers/crypto/dev_urandom.c
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
#include <assert.h>

#include <nuttx/lib/lib.h>
#include <nuttx/lib/xorshift128.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/random.h>

#if defined(CONFIG_DEV_URANDOM) && !defined(CONFIG_DEV_URANDOM_ARCH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_DEV_URANDOM_CONGRUENTIAL) && \
    !defined(CONFIG_DEV_URANDOM_XORSHIFT128) && \
    !defined(CONFIG_DEV_URANDOM_RANDOM_POOL)
#  ifdef CONFIG_CRYPTO_RANDOM_POOL
#    define CONFIG_DEV_URANDOM_RANDOM_POOL 1
#  else
#    define CONFIG_DEV_URANDOM_XORSHIFT128 1
#  endif
#endif

#ifdef CONFIG_DEV_URANDOM_XORSHIFT128
#  define PRNG() do_xorshift128()
#elif defined(CONFIG_DEV_URANDOM_CONGRUENTIAL)
#  define PRNG() do_congruential()
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

union xorshift128_state_u
{
  struct xorshift128_state_s state;
  uint8_t u[16];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#define min(a, b)   (((a) < (b)) ? (a) : (b))

static ssize_t devurand_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t devurand_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);
static int devurand_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup);

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
  NULL,                         /* ioctl */
  devurand_poll                 /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                        /* unlink */
#endif
};

#ifdef CONFIG_DEV_URANDOM_XORSHIFT128
static union xorshift128_state_u g_prng;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: do_xorshift128
 ****************************************************************************/

#ifdef CONFIG_DEV_URANDOM_XORSHIFT128
static inline uint32_t do_xorshift128(void)
{
  return xorshift128(&g_prng.state);
}
#endif

/****************************************************************************
 * Name: do_congruential
 ****************************************************************************/

#ifdef CONFIG_DEV_URANDOM_CONGRUENTIAL
static inline uint32_t do_congruential(void)
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
#ifdef CONFIG_DEV_URANDOM_RANDOM_POOL
  if (len > 0)
    {
      getrandom(buffer, len);
    }

#else
  size_t n;
  uint32_t rnd;

  n = len;

  /* Align buffer pointer to 4-byte boundary */

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
          *buffer++ = rnd & 0xff;
          rnd >>= 8;
        }
      while (--n > 0);
    }
#endif /* CONFIG_DEV_URANDOM_RANDOM_POOL */

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

  len = min(len, sizeof(unsigned int));
  memcpy(&seed, buffer, len);
  srand(seed);
  return len;

#elif defined(CONFIG_DEV_URANDOM_RANDOM_POOL)
  const unsigned int alignmask = sizeof(uint32_t) - 1;
  const size_t initlen = len;
  uint32_t tmp = 0;
  size_t currlen;

  if (!len)
    {
      return 0;
    }

  /* Seed entropy pool with data from user. */

  if ((uintptr_t)buffer & alignmask)
    {
      /* Make unaligned input aligned. */

      currlen = min(sizeof(uint32_t) - ((uintptr_t)buffer & alignmask), len);
      memcpy(&tmp, buffer, currlen);
      up_rngaddint(RND_SRC_SW, tmp);

      len -= currlen;
      buffer += currlen;
    }

  if (len >= sizeof(uint32_t))
    {
      /* Handle bulk aligned, word-sized data. */

      DEBUGASSERT(((uintptr_t)buffer & alignmask) == 0);
      currlen = len / sizeof(uint32_t);
      up_rngaddentropy(RND_SRC_SW, (FAR uint32_t *)buffer, currlen);
      buffer += currlen * sizeof(uint32_t);
      len %= sizeof(uint32_t);
    }

  if (len > 0)
    {
      /* Handle trailing bytes. */

      DEBUGASSERT(len < sizeof(uint32_t));
      memcpy(&tmp, buffer, len);
      up_rngaddint(RND_SRC_SW, tmp);
    }

  /* Reseeding of random number generator from entropy pool. */

  up_rngreseed();

  return initlen;
#else
  len = min(len, sizeof(g_prng.u));
  memcpy(&g_prng.u, buffer, len);
  return len;
#endif
}

/****************************************************************************
 * Name: devurand_poll
 ****************************************************************************/

static int devurand_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  if (setup)
    {
      fds->revents |= (fds->events & (POLLIN | POLLOUT));
      if (fds->revents != 0)
        {
          nxsem_post(fds->sem);
        }
    }

  return OK;
}

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
  /* Seed the PRNG */

#ifdef CONFIG_DEV_URANDOM_CONGRUENTIAL
  srand(10197);
#elif defined(CONFIG_DEV_URANDOM_RANDOM_POOL)
  up_randompool_initialize();
#else
  g_prng.state.w = 97;
  g_prng.state.x = 101;
  g_prng.state.y = g_prng.state.w << 17;
  g_prng.state.z = g_prng.state.x << 25;
#endif

  register_driver("/dev/urandom", &g_urand_fops, 0666, NULL);
}

#endif /* CONFIG_DEV_URANDOM && CONFIG_DEV_URANDOM_ARCH */
