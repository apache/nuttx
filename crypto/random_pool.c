/****************************************************************************
 * crypto/random_pool.c
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
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/random.h>
#include <nuttx/board.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/crypto/blake2s.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#define ROTL_32(x,n) ( ((x) << (n)) | ((x) >> (32-(n))) )
#define ROTR_32(x,n) ( ((x) >> (n)) | ((x) << (32-(n))) )

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct blake2xs_rng_s
{
  uint32_t out_node_offset;
  blake2s_param param;
  blake2s_state ctx;
  char out_root[BLAKE2S_OUTBYTES];
};

struct rng_s
{
  mutex_t rd_lock; /* Threads can only exclusively access the RNG */
  volatile uint32_t rd_addptr;
  volatile uint32_t rd_newentr;
  volatile uint8_t rd_rotate;
  volatile uint8_t rd_prev_time;
  volatile uint16_t rd_prev_irq;
  bool output_initialized;
  struct blake2xs_rng_s blake2xs;
};

enum
{
  POOL_SIZE = ENTROPY_POOL_SIZE,
  POOL_MASK = (POOL_SIZE - 1),

  MIN_SEED_NEW_ENTROPY_WORDS = 128,
  MAX_SEED_NEW_ENTROPY_WORDS = 1024
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rng_s g_rng;

#ifdef CONFIG_BOARD_ENTROPY_POOL
/* Entropy pool structure can be provided by board source. Use for this is,
 * for example, allocate entropy pool from special area of RAM which content
 * is kept over system reset.
 */

#  define entropy_pool board_entropy_pool
#else
static struct entropy_pool_s entropy_pool;
#endif

/* Polynomial from paper "The Linux Pseudorandom Number Generator Revisited"
 * x^POOL_SIZE + x^104 + x^76 + x^51 + x^25 + x + 1
 */

static const uint32_t pool_stir[] =
{
  POOL_SIZE, 104, 76, 51, 25, 1
};

/* Derived from IEEE 802.3 CRC-32 */

static const uint32_t pool_twist[8] =
{
  0x00000000, 0x3b6e20c8, 0x76dc4190, 0x4db26158,
  0xedb88320, 0xd6d6a3e8, 0x9b64c2b0, 0xa00ae278
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: addentropy
 *
 * Description:
 *
 * This function adds a number of integers into the entropy pool.
 * The pool is stirred with a polynomial of degree POOL_SIZE over GF(2).
 *
 * Code is inspired by add_entropy_words() function of OpenBSD kernel.
 *
 * Input Parameters:
 *   buf     -   Buffer of integers to be added
 *   n       -   Number of elements in buf
 *   inc_new -   Count element as new entry
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void addentropy(FAR const uint32_t *buf, size_t n, bool inc_new)
{
  /* Compile time check for that POOL_SIZE is power of two. */

  static char pool_size_p2_check[1 - ((POOL_SIZE & (POOL_SIZE - 1)) * 2)];

  UNUSED(pool_size_p2_check);

  while (n-- > 0)
    {
      uint32_t rotate;
      uint32_t w;
      uint32_t i;

      rotate = g_rng.rd_rotate;
      w = rotate ? ROTL_32(*buf, rotate) : *buf;
      i = g_rng.rd_addptr = (g_rng.rd_addptr - 1) & POOL_MASK;

      /* Normal round, we add 7 bits of rotation to the pool.
       * At the beginning of the pool, we add extra 7 bits
       * rotation, in order for successive passes spread the
       * input bits across the pool evenly.
       */

      g_rng.rd_rotate = (rotate + (i ? 7 : 14)) & 31;

      /* XOR pool contents corresponding to polynomial terms */

      w ^= entropy_pool.pool[(i + pool_stir[1]) & POOL_MASK];
      w ^= entropy_pool.pool[(i + pool_stir[2]) & POOL_MASK];
      w ^= entropy_pool.pool[(i + pool_stir[3]) & POOL_MASK];
      w ^= entropy_pool.pool[(i + pool_stir[4]) & POOL_MASK];
      w ^= entropy_pool.pool[(i + pool_stir[5]) & POOL_MASK];
      w ^= entropy_pool.pool[i]; /* 2^POOL_SIZE */

      entropy_pool.pool[i] = (w >> 3) ^ pool_twist[w & 7];
      buf++;

      if (inc_new)
        {
          g_rng.rd_newentr += 1;
        }
    }
}

/****************************************************************************
 * Name: initentropy
 *
 * Description:
 *   Hash entropy pool to BLAKE2s context. This is an internal interface for
 *   seeding out-facing BLAKE2Xs random bit generator from entropy pool.
 *
 *   Code is inspired by extract_entropy() function of OpenBSD kernel.
 *
 *   Note that this function cannot fail, other than by asserting.
 *
 *   Warning: In protected kernel builds, this interface MUST NOT be
 *   exported to userspace. This interface MUST NOT be used as a
 *   general-purpose random bit generator!
 *
 * Input Parameters:
 *   S  - BLAKE2s instance that will absorb entropy pool
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void initentropy(FAR blake2s_state *S)
{
#ifdef CONFIG_SCHED_CPULOAD
  struct cpuload_s load;
#endif
  uint32_t tmp;

  add_sw_randomness(g_rng.rd_newentr);

  /* Absorb the entropy pool */

  blake2s_update(S, (FAR const uint32_t *)entropy_pool.pool,
                 sizeof(entropy_pool.pool));

  /* Add something back so repeated calls to this function
   * return different values.
   */

  tmp = sizeof(entropy_pool.pool);
  tmp <<= 27;
#ifdef CONFIG_SCHED_CPULOAD
  clock_cpuload(0, &load);
  tmp += load.total ^ ROTL_32(load.active, 23);
#endif
  add_sw_randomness(tmp);

  g_rng.rd_newentr = 0;
}

/* The BLAKE2Xs based random number generator algorithm.
 *
 * BLAKE2X is a extensible-output function (XOF) variant of BLAKE2 hash
 * function. One application of XOFs is use as deterministic random bit
 * number generator (DRBG) as used here. BLAKE2 specification is available
 * at https://blake2.net/
 *
 * BLAKE2Xs here  implementation is based on public-domain/CC0 BLAKE2
 * reference implementation by Samual Neves, at
 *
 *   https://github.com/BLAKE2/BLAKE2/tree/master/ref
 *   Copyright 2012, Samuel Neves <sneves@dei.uc.pt>
 */

static void rng_reseed(void)
{
  blake2s_param P =
    {
    };

  /* Reset output node counter. */

  g_rng.blake2xs.out_node_offset = 0;

  /* Initialize parameter block */

  P.digest_length = BLAKE2S_OUTBYTES;
  P.key_length    = 0;
  P.fanout        = 1;
  P.depth         = 1;
  blake2_store32(P.leaf_length, 0);
  blake2_store32(P.node_offset, 0);
  blake2_store16(P.xof_length, 0xffff);
  P.node_depth    = 0;
  P.inner_length  = 0;
  g_rng.blake2xs.param = P;

  blake2s_init_param(&g_rng.blake2xs.ctx, &g_rng.blake2xs.param);

  /* Initialize with randomness from entropy pool */

  initentropy(&g_rng.blake2xs.ctx);

  /* Absorb also the previous root */

  blake2s_update(&g_rng.blake2xs.ctx, g_rng.blake2xs.out_root,
                 sizeof(g_rng.blake2xs.out_root));

  /* Finalize the new root hash */

  blake2s_final(&g_rng.blake2xs.ctx, g_rng.blake2xs.out_root,
                BLAKE2S_OUTBYTES);

  explicit_bzero(&g_rng.blake2xs.ctx, sizeof(g_rng.blake2xs.ctx));

  /* Setup parameters for output phase. */

  g_rng.blake2xs.param.key_length = 0;
  g_rng.blake2xs.param.fanout = 0;
  blake2_store32(g_rng.blake2xs.param.leaf_length, BLAKE2S_OUTBYTES);
  g_rng.blake2xs.param.inner_length = BLAKE2S_OUTBYTES;
  g_rng.blake2xs.param.node_depth = 0;

  g_rng.output_initialized = true;
}

static void rng_buf_internal(FAR uint8_t *bytes, size_t nbytes)
{
  if (!g_rng.output_initialized)
    {
      if (g_rng.rd_newentr < MIN_SEED_NEW_ENTROPY_WORDS)
        {
          cryptwarn("Entropy pool RNG initialized with very low entropy. "
                    " Consider implementing CONFIG_BOARD_INITRNGSEED!\n");
        }

      rng_reseed();
    }
  else if (g_rng.rd_newentr >= MAX_SEED_NEW_ENTROPY_WORDS)
    {
      /* Initial entropy is low. Reseed when we have accumulated more. */

      rng_reseed();
    }
  else if (g_rng.blake2xs.out_node_offset == UINT32_MAX)
    {
      /* Maximum BLAKE2Xs output reached (2^32-1 output blocks, maximum 128
       * GiB bytes), reseed.
       */

      rng_reseed();
    }

  /* Output phase for BLAKE2Xs. */

  for (; nbytes > 0; ++g_rng.blake2xs.out_node_offset)
    {
      size_t block_size = MIN(nbytes, BLAKE2S_OUTBYTES);

      /* Initialize state */

      g_rng.blake2xs.param.digest_length = block_size;
      blake2_store32(g_rng.blake2xs.param.node_offset,
                     g_rng.blake2xs.out_node_offset);
      blake2s_init_param(&g_rng.blake2xs.ctx, &g_rng.blake2xs.param);

      /* Process state and output random bytes */

      blake2s_update(&g_rng.blake2xs.ctx, g_rng.blake2xs.out_root,
                     sizeof(g_rng.blake2xs.out_root));
      blake2s_final(&g_rng.blake2xs.ctx, bytes, block_size);

      bytes += block_size;
      nbytes -= block_size;
    }
}

static void rng_init(void)
{
  cryptinfo("Initializing RNG\n");

  memset(&g_rng, 0, sizeof(struct rng_s));
  nxmutex_init(&g_rng.rd_lock);

  /* We do not initialize output here because this is called
   * quite early in boot and there may not be enough entropy.
   *
   * Board level may define CONFIG_BOARD_INITRNGSEED if it implements
   * early random seeding.
   */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rngaddint
 *
 * Description:
 *   Add one integer to entropy pool, contributing a specific kind
 *   of entropy to pool.
 *
 * Input Parameters:
 *   kindof  - Enumeration constant telling where val came from
 *   val     - Integer to be added
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_rngaddint(enum rnd_source_t kindof, int val)
{
  uint32_t buf[1];

  buf[0] = val;

  up_rngaddentropy(kindof, buf, 1);
}

/****************************************************************************
 * Name: up_rngaddentropy
 *
 * Description:
 *   Add buffer of integers to entropy pool.
 *
 * Input Parameters:
 *   kindof  - Enumeration constant telling where val came from
 *   buf     - Buffer of integers to be added
 *   n       - Number of elements in buf
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_rngaddentropy(enum rnd_source_t kindof, FAR const uint32_t *buf,
                      size_t n)
{
  uint32_t tbuf[1];
  struct timespec ts;
  bool new_inc = true;

  if (kindof == RND_SRC_IRQ && n > 0)
    {
      /* Ignore interrupt randomness if previous interrupt was from same
       * source.
       */

      if (buf[0] == g_rng.rd_prev_irq)
        {
          return;
        }

      g_rng.rd_prev_irq = buf[0];
    }

  /* We don't actually track what kind of entropy we receive,
   * just add it all to pool. One exception is interrupt
   * and timer randomness, where we limit rate of new pool entry
   * counting to prevent high interrupt rate triggering RNG
   * reseeding too fast.
   */

  clock_systime_timespec(&ts);
  tbuf[0] = ROTL_32((uint32_t)ts.tv_nsec, 17) ^ ROTL_32(ts.tv_sec, 3);
  tbuf[0] += ROTL_32(kindof, 27);
  tbuf[0] += ROTL_32((uintptr_t)&tbuf[0], 11);

  if (kindof == RND_SRC_TIME || kindof == RND_SRC_IRQ)
    {
      uint8_t curr_time = ts.tv_sec * 8 + ts.tv_nsec / (NSEC_PER_SEC / 8);

      /* Allow interrupts/timers increase entropy counter at max rate
       * of 8 Hz.
       */

      if (g_rng.rd_prev_time == curr_time)
        {
          new_inc = false;
        }
      else
        {
          g_rng.rd_prev_time = curr_time;
        }
    }

  if (n > 0)
    {
      tbuf[0] ^= buf[0];
      buf++;
      n--;
    }

  addentropy(tbuf, 1, new_inc);

  if (n > 0)
    {
      addentropy(buf, n, new_inc);
    }
}

/****************************************************************************
 * Name: up_rngreseed
 *
 * Description:
 *   Force reseeding random number generator from entropy pool
 *
 ****************************************************************************/

void up_rngreseed(void)
{
  int ret;

  ret = nxmutex_lock(&g_rng.rd_lock);
  if (ret >= 0)
    {
      if (g_rng.rd_newentr >= MIN_SEED_NEW_ENTROPY_WORDS)
        {
          rng_reseed();
        }

      nxmutex_unlock(&g_rng.rd_lock);
    }
}

/****************************************************************************
 * Name: up_randompool_initialize
 *
 * Description:
 *   Initialize entropy pool and random number generator
 *
 ****************************************************************************/

void up_randompool_initialize(void)
{
  rng_init();

#ifdef CONFIG_BOARD_INITRNGSEED
  board_init_rngseed();
#endif
}

/****************************************************************************
 * Name: arc4random_buf
 *
 * Description:
 *   Fill a buffer of arbitrary length with randomness. This is the
 *   preferred interface for getting random numbers. The traditional
 *   /dev/random approach is susceptible for things like the attacker
 *   exhausting file descriptors on purpose.
 *
 *   Note that this function cannot fail, other than by asserting.
 *
 * Input Parameters:
 *   bytes  - Buffer for returned random bytes
 *   nbytes - Number of bytes requested.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arc4random_buf(FAR void *bytes, size_t nbytes)
{
  nxmutex_lock(&g_rng.rd_lock);
  rng_buf_internal(bytes, nbytes);
  nxmutex_unlock(&g_rng.rd_lock);
}

/****************************************************************************
 * Name: arc4random
 *
 * Description:
 *   Returns a single 32-bit value. This is the preferred interface for
 *   getting random numbers. The traditional /dev/random approach is
 *   susceptible for things like the attacker exhausting file
 *   descriptors on purpose.
 *
 *   Note that this function cannot fail, other than by asserting.
 *
 * Returned Value:
 *   a random 32-bit value.
 *
 ****************************************************************************/

uint32_t arc4random(void)
{
  uint32_t ret;

  arc4random_buf(&ret, sizeof(ret));
  return ret;
}
