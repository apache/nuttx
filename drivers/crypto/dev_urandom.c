/****************************************************************************
 * drivers/crypto/dev_urandom.c
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
  NULL,                         /* mmap */
  NULL,                         /* truncate */
  devurand_poll                 /* poll */
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
      arc4random_buf(buffer, len);
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
      poll_notify(&fds, 1, POLLIN | POLLOUT);
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
