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
 * The RP2350 embeds an Arm CryptoCell-style true random number generator:
 * a ring oscillator is sampled, von-Neumann + CRNGT + autocorrelation
 * health tests condition the raw bits, and every 192 conditioned bits
 * appear in the six EHR_DATA words.  This driver serves those bits directly
 * as /dev/random and /dev/urandom (a polling reader - entropy is drawn
 * rarely: mbedtls seeds its CTR_DRBG once and reseeds occasionally, so no
 * interrupt plumbing is needed).  Health-test failures discard the block
 * and re-arm the source, so only conditioned entropy is ever returned.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/param.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "hardware/rp23xx_trng.h"
#include "hardware/rp23xx_resets.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RP23XX_TRNG_EHR_WORDS   6              /* 192-bit EHR_DATA block */
#define RP23XX_TRNG_EHR_BYTES   (RP23XX_TRNG_EHR_WORDS * 4)

/* ROSC sample count: cycles the source is sampled per conditioned bit.  A
 * larger value trades throughput for entropy quality; 1000 mirrors the Arm
 * CryptoCell reference and the RP2350 bootrom's TRNG use.
 */

#define RP23XX_TRNG_SAMPLE_CNT  1000

/* All health-test error bits in the interrupt status register.             */

#define RP23XX_TRNG_ISR_ERRORS  (RP23XX_TRNG_RNG_ISR_VN_ERR |     \
                                 RP23XX_TRNG_RNG_ISR_CRNGT_ERR | \
                                 RP23XX_TRNG_RNG_ISR_AUTOCORR_ERR)

/* Bounded spin waiting for a 192-bit block (each needs ~SAMPLE_CNT source
 * cycles; this is generous and only bounds a wedged source).
 */

#define RP23XX_TRNG_MAX_SPIN    1000000

/* Startup health check (a SECOND line of defence behind the CryptoCell's
 * own von-Neumann / CRNGT / autocorrelation tests, which run in hardware on
 * every block and are already enforced below).  We draw a startup sample of
 * the CONDITIONED output and reject a stuck, constant or grossly-repetitive
 * source before the RNG is trusted.  These bounds are deliberately loose -
 * they never fire on healthy entropy but catch a dead/wedged source (all
 * identical blocks, a constant byte, or an absurd run) that would otherwise
 * silently weaken every key and signing nonce derived from it.
 */

#define RP23XX_TRNG_HEALTH_BLOCKS   8   /* 8 * 24 = 192 conditioned bytes */
#define RP23XX_TRNG_HEALTH_BYTES    (RP23XX_TRNG_HEALTH_BLOCKS * \
                                     RP23XX_TRNG_EHR_BYTES)
#define RP23XX_TRNG_MAX_BYTE_RUN    6   /* >=6 equal bytes in a row: fail */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool    rp23xx_rng_selftest(void);
static ssize_t rp23xx_rng_read(struct file *filep, char *buffer, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_rng_lock = NXMUTEX_INITIALIZER;
static bool    g_rng_inited;
static bool    g_rng_healthy;    /* startup health check passed */

static const struct file_operations g_rng_ops =
{
  NULL,                 /* open  */
  NULL,                 /* close */
  rp23xx_rng_read,      /* read  */
  NULL,                 /* write */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_rng_enable_source
 *
 * Description:
 *   (Re-)start the entropy source from a clean state: clear any pending
 *   status, reset the source, and enable sampling.
 ****************************************************************************/

static void rp23xx_rng_enable_source(void)
{
  putreg32(0, RP23XX_TRNG_RND_SOURCE_ENABLE);
  putreg32(RP23XX_TRNG_RNG_ICR_EHR_VALID | RP23XX_TRNG_ISR_ERRORS,
           RP23XX_TRNG_RNG_ICR);
  putreg32(1, RP23XX_TRNG_RND_SOURCE_ENABLE);
}

/****************************************************************************
 * Name: rp23xx_rng_init
 *
 * Description:
 *   Bring the TRNG out of reset and configure the source.  Idempotent.
 ****************************************************************************/

static void rp23xx_rng_init(void)
{
  if (g_rng_inited)
    {
      return;
    }

  /* Deassert the TRNG block reset and wait for it to come ready. */

  modifyreg32(RP23XX_RESETS_RESET, RP23XX_RESETS_RESET_TRNG, 0);
  while ((getreg32(RP23XX_RESETS_RESET_DONE) &
          RP23XX_RESETS_RESET_TRNG) == 0)
    {
    }

  /* Poll-only: mask every TRNG interrupt (status is still observable). */

  putreg32(RP23XX_TRNG_RNG_IMR_VN_ERR_INT_MASK |
           RP23XX_TRNG_RNG_IMR_CRNGT_ERR_INT_MASK |
           RP23XX_TRNG_RNG_IMR_AUTOCORR_ERR_INT_MASK |
           RP23XX_TRNG_RNG_IMR_EHR_VALID_INT_MASK,
           RP23XX_TRNG_RNG_IMR);

  /* Software-reset the core, then configure the ring-oscillator source and
   * the per-bit sample count.  DEBUG_CONTROL = 0 keeps the von-Neumann,
   * CRNGT and autocorrelation health tests ENABLED (no bypass).
   */

  putreg32(1, RP23XX_TRNG_TRNG_SW_RESET);
  putreg32(0, RP23XX_TRNG_TRNG_DEBUG_CONTROL);
  putreg32(0, RP23XX_TRNG_TRNG_CONFIG);          /* ROSC source 0 */
  putreg32(RP23XX_TRNG_SAMPLE_CNT, RP23XX_TRNG_SAMPLE_CNT1);

  rp23xx_rng_enable_source();

  /* Gate the source on a startup health check.  If it fails, leave
   * g_rng_healthy false so every read fails and the entropy consumer
   * fail-stops loudly rather than minting keys on a broken source.
   */

  g_rng_healthy = rp23xx_rng_selftest();
  if (!g_rng_healthy)
    {
      _err("ERROR: TRNG startup health check FAILED - refusing entropy\n");
    }

  g_rng_inited = true;
}

/****************************************************************************
 * Name: rp23xx_rng_collect
 *
 * Description:
 *   Wait for and read one 192-bit conditioned entropy block into out[].
 *   Health-test failures discard the block and re-arm the source.  Returns
 *   0 on success, -ETIMEDOUT if the source never produced a valid block.
 ****************************************************************************/

static int rp23xx_rng_collect(uint32_t out[RP23XX_TRNG_EHR_WORDS])
{
  int spin;
  int i;

  for (spin = 0; spin < RP23XX_TRNG_MAX_SPIN; spin++)
    {
      uint32_t isr = getreg32(RP23XX_TRNG_RNG_ISR);

      if ((isr & RP23XX_TRNG_ISR_ERRORS) != 0)
        {
          /* A health test rejected this block: drop it and re-arm. */

          rp23xx_rng_enable_source();
          continue;
        }

      if ((isr & RP23XX_TRNG_RNG_ISR_EHR_VALID) != 0)
        {
          for (i = 0; i < RP23XX_TRNG_EHR_WORDS; i++)
            {
              out[i] = getreg32(RP23XX_TRNG_EHR_DATA(i));
            }

          /* Reading EHR_DATA clears validity; re-arm for the next block. */

          rp23xx_rng_enable_source();
          return 0;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rp23xx_rng_selftest
 *
 * Description:
 *   Startup quality gate.  Draw a sample of the conditioned source and
 *   reject it if the entropy is obviously broken: identical consecutive
 *   blocks (a stuck source), a constant byte value, or a run of equal bytes
 *   beyond RP23XX_TRNG_MAX_BYTE_RUN.  Returns true if the source looks
 *   healthy.  These are loose bounds - the hardware VN/CRNGT/autocorr tests
 *   (enforced in rp23xx_rng_collect) cover the statistical quality; this
 *   catches the gross, silent failure modes that would otherwise weaken
 *   every derived key.
 ****************************************************************************/

static bool rp23xx_rng_selftest(void)
{
  uint32_t sample[RP23XX_TRNG_HEALTH_BLOCKS][RP23XX_TRNG_EHR_WORDS];
  const uint8_t *bytes = (const uint8_t *)sample;
  int run = 1;
  int i;

  for (i = 0; i < RP23XX_TRNG_HEALTH_BLOCKS; i++)
    {
      if (rp23xx_rng_collect(sample[i]) < 0)
        {
          return false;         /* source never produced a block */
        }

      /* No two consecutive conditioned blocks may be identical (192-bit
       * collision on healthy entropy is ~2^-192; identical means stuck).
       */

      if (i > 0 &&
          memcmp(sample[i], sample[i - 1], RP23XX_TRNG_EHR_BYTES) == 0)
        {
          return false;
        }
    }

  /* Reject a constant byte value and any absurd run of equal bytes. */

  for (i = 1; i < RP23XX_TRNG_HEALTH_BYTES; i++)
    {
      if (bytes[i] == bytes[i - 1])
        {
          if (++run >= RP23XX_TRNG_MAX_BYTE_RUN)
            {
              return false;
            }
        }
      else
        {
          run = 1;
        }
    }

  return true;
}

/****************************************************************************
 * Name: rp23xx_rng_read
 ****************************************************************************/

static ssize_t rp23xx_rng_read(struct file *filep, char *buffer, size_t len)
{
  size_t got = 0;
  int ret;

  ret = nxmutex_lock(&g_rng_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!g_rng_healthy)
    {
      nxmutex_unlock(&g_rng_lock);
      return -EIO;      /* startup health check failed - fail closed */
    }

  while (got < len)
    {
      uint32_t block[RP23XX_TRNG_EHR_WORDS];
      size_t n;

      ret = rp23xx_rng_collect(block);
      if (ret < 0)
        {
          break;
        }

      n = MIN(len - got, RP23XX_TRNG_EHR_BYTES);
      memcpy(buffer + got, block, n);
      got += n;
    }

  nxmutex_unlock(&g_rng_lock);

  if (got == 0)
    {
      return ret < 0 ? ret : -EIO;
    }

  return got;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Initialise the TRNG and register /dev/random.  Overrides the generic
 *   software implementation (this arch selects ARCH_HAVE_RNG).
 ****************************************************************************/

void devrandom_register(void)
{
  rp23xx_rng_init();
  register_driver("/dev/random", &g_rng_ops, 0444, NULL);
}

/****************************************************************************
 * Name: devurandom_register
 *
 * Description:
 *   Initialise the TRNG and register /dev/urandom (same hardware source).
 ****************************************************************************/

void devurandom_register(void)
{
  rp23xx_rng_init();
  register_driver("/dev/urandom", &g_rng_ops, 0444, NULL);
}
