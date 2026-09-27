/*****************************************************************************
 * arch/arm/src/imxrt/imxrt_caam.c
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>

#include "arm_internal.h"
#include "hardware/rt117x/imxrt117x_caam.h"
#include "imxrt_caam.h"
#include "imxrt_periphclks.h"

#ifdef CONFIG_IMXRT_CAAM

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#ifndef ARMV7M_DCACHE_LINESIZE
#  define ARMV7M_DCACHE_LINESIZE 32
#endif

/* One entry each way. A ring deeper than that would only let a second
 * request queue behind a caller that is already waiting for the first.
 */

#define CAAM_RING_ENTRIES     1

/* CAAM writes the result by DMA, so the landing buffer owns whole cache
 * lines and shares them with nothing.
 */

#define CAAM_RNG_BLOCKLEN     ARMV7M_DCACHE_LINESIZE

#define CAAM_DESC_WORDS       8

/* Descriptor words, from the SEC reference descriptor encoding. */

#define CAAM_DESC_HDR(len)    (0xb0800000 | (len))
#define CAAM_OP_RNG_GENERATE  0x82500002
#define CAAM_OP_RNG_INIT_SH0  0x82500006
#define CAAM_OP_RNG_GEN_SK    0x82501000
#define CAAM_JUMP_WAIT_CLASS1 0xa2000001
#define CAAM_LOAD_CLRW        0x10880004
#define CAAM_FIFO_STORE_RNG   0x60340000

/* Entropy sample length, in system clocks. A self test that fails is
 * retried with a longer one, which is how NXP's own code finds a value
 * that passes across voltage and temperature.
 */

#define CAAM_INSTANTIATE_SETTLE 20000

#define CAAM_ENT_DELAY_MIN    3200
#define CAAM_ENT_DELAY_MAX    12800
#define CAAM_ENT_DELAY_STEP   400

#define CAAM_TIMEOUT          100000

/*****************************************************************************
 * Private Data
 *****************************************************************************/

/* The rings and the descriptor are read by CAAM over DMA, and the result is
 * written back the same way, so each one owns its cache lines outright.
 */

static uint32_t g_input_ring[CAAM_RING_ENTRIES]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static uint32_t g_output_ring[CAAM_RING_ENTRIES * 2]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static uint32_t g_desc[CAAM_DESC_WORDS]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static uint8_t g_rngbuf[CAAM_RNG_BLOCKLEN]
  aligned_data(ARMV7M_DCACHE_LINESIZE);

static bool g_initialized;

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: imxrt_caam_clean
 *****************************************************************************/

static void imxrt_caam_clean(void *addr, size_t len)
{
  up_clean_dcache((uintptr_t)addr, (uintptr_t)addr + len);
}

/*****************************************************************************
 * Name: imxrt_caam_invalidate
 *****************************************************************************/

static void imxrt_caam_invalidate(void *addr, size_t len)
{
  up_invalidate_dcache((uintptr_t)addr, (uintptr_t)addr + len);
}

/*****************************************************************************
 * Name: imxrt_caam_run
 *
 * Description:
 *   Submit the descriptor in g_desc to job ring zero and wait for it.
 *
 * Returned Value:
 *   Zero on success, -EIO if the ring reported an error, -ETIMEDOUT if it
 *   never answered.
 *
 *****************************************************************************/

static int imxrt_caam_ring_init(void);

static int imxrt_caam_run(void)
{
  uint32_t status;
  int timeout;

  imxrt_caam_clean(g_desc, sizeof(g_desc));

  g_input_ring[0] = (uint32_t)(uintptr_t)g_desc;
  imxrt_caam_clean(g_input_ring, sizeof(g_input_ring));

  putreg32(1, IMXRT_CAAM_IRJA);

  for (timeout = CAAM_TIMEOUT; timeout > 0; timeout--)
    {
      if (getreg32(IMXRT_CAAM_ORSF) != 0)
        {
          break;
        }
    }

  if (timeout == 0)
    {
      _err("ERROR: job ring did not answer\n");
      return -ETIMEDOUT;
    }

  imxrt_caam_invalidate(g_output_ring, sizeof(g_output_ring));
  status = g_output_ring[1];

  /* Tell the ring the slot is free again whatever the outcome, or the next
   * request finds it still occupied.
   */

  putreg32(1, IMXRT_CAAM_ORJR);

  if (status != 0)
    {
      _err("ERROR: job failed, status 0x%08" PRIx32 "\n", status);
      return -EIO;
    }

  return OK;
}

/*****************************************************************************
 * Name: imxrt_caam_kick_trng
 *
 * Description:
 *   Set the entropy sample length and the frequency limits derived from it,
 *   which is what the self test in the state handle instantiation checks
 *   against.
 *
 *****************************************************************************/

static void imxrt_caam_kick_trng(uint32_t ent_delay)
{
  uint32_t val;

  modifyreg32(IMXRT_CAAM_RTMCTL, 0, CAAM_RTMCTL_PRGM);

  val = getreg32(IMXRT_CAAM_RTSDCTL) & ~CAAM_RTSDCTL_ENT_DLY_MASK;
  putreg32(val | (ent_delay << CAAM_RTSDCTL_ENT_DLY_SHIFT),
           IMXRT_CAAM_RTSDCTL);

  putreg32(ent_delay >> 2, IMXRT_CAAM_RTFRQMIN);
  putreg32(ent_delay << 4, IMXRT_CAAM_RTFRQMAX);

  modifyreg32(IMXRT_CAAM_RTMCTL, CAAM_RTMCTL_PRGM, 0);
}

/*****************************************************************************
 * Name: imxrt_caam_instantiate
 *
 * Description:
 *   Instantiate RNG state handle zero, generating the secure keys with it if
 *   nothing has done so since power on.
 *
 *****************************************************************************/

static int imxrt_caam_instantiate(bool gen_sk)
{
  int words = 2;

  g_desc[1] = CAAM_OP_RNG_INIT_SH0;

  if (gen_sk)
    {
      g_desc[2] = CAAM_JUMP_WAIT_CLASS1;
      g_desc[3] = CAAM_LOAD_CLRW;
      g_desc[4] = 1;
      g_desc[5] = CAAM_OP_RNG_GEN_SK;
      words = 6;
    }

  g_desc[0] = CAAM_DESC_HDR(words);

  return imxrt_caam_run();
}

/*****************************************************************************
 * Name: imxrt_caam_rng_init
 *****************************************************************************/

static int imxrt_caam_rng_init(void)
{
  uint32_t ent_delay;
  bool gen_sk;
  int ret = -EIO;

  if ((getreg32(IMXRT_CAAM_RDSTA) & CAAM_RDSTA_IF0) != 0)
    {
      return OK;
    }

  gen_sk = (getreg32(IMXRT_CAAM_RDSTA) & CAAM_RDSTA_SKVN) == 0;

  for (ent_delay = CAAM_ENT_DELAY_MIN;
       ent_delay <= CAAM_ENT_DELAY_MAX;
       ent_delay += CAAM_ENT_DELAY_STEP)
    {
      int settle;

      imxrt_caam_kick_trng(ent_delay);

      ret = imxrt_caam_instantiate(gen_sk);

      for (settle = CAAM_INSTANTIATE_SETTLE; settle > 0; settle--)
        {
          if ((getreg32(IMXRT_CAAM_RDSTA) & CAAM_RDSTA_IF0) != 0)
            {
              return OK;
            }

          up_udelay(100);
        }

      imxrt_caam_ring_init();

      if ((getreg32(IMXRT_CAAM_RDSTA) & CAAM_RDSTA_IF0) != 0)
        {
          return OK;
        }
    }

  _err("ERROR: RNG would not instantiate\n");
  return ret < 0 ? ret : -EIO;
}

/*****************************************************************************
 * Name: imxrt_caam_ring_init
 *****************************************************************************/

static int imxrt_caam_ring_init(void)
{
  int timeout;

  putreg32(CAAM_JRCR_RESET, IMXRT_CAAM_JRCR);

  for (timeout = CAAM_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(IMXRT_CAAM_JRCR) & CAAM_JRCR_RESET) == 0)
        {
          break;
        }
    }

  if (timeout == 0)
    {
      _err("ERROR: job ring would not reset\n");
      return -ETIMEDOUT;
    }

  memset(g_input_ring, 0, sizeof(g_input_ring));
  memset(g_output_ring, 0, sizeof(g_output_ring));
  imxrt_caam_clean(g_input_ring, sizeof(g_input_ring));
  imxrt_caam_clean(g_output_ring, sizeof(g_output_ring));

  putreg32(0, IMXRT_CAAM_IRBA_H);
  putreg32((uint32_t)(uintptr_t)g_input_ring, IMXRT_CAAM_IRBA_L);
  putreg32(0, IMXRT_CAAM_ORBA_H);
  putreg32((uint32_t)(uintptr_t)g_output_ring, IMXRT_CAAM_ORBA_L);
  putreg32(CAAM_RING_ENTRIES, IMXRT_CAAM_IRS);
  putreg32(CAAM_RING_ENTRIES, IMXRT_CAAM_ORS);

  /* Completion is polled, so the ring interrupt is never wanted. */

  modifyreg32(IMXRT_CAAM_JRCFG1, 0, CAAM_JRCFG1_IMSK);

  return OK;
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: imxrt_caam_initialize
 *****************************************************************************/

int imxrt_caam_initialize(void)
{
  int ret;

  if (g_initialized)
    {
      return OK;
    }

  imxrt_clockall_caam();

  modifyreg32(IMXRT_CAAM_MCFGR, CAAM_MCFGR_AWCACHE_MASK,
              CAAM_MCFGR_AWCACHE_CACH | CAAM_MCFGR_AWCACHE_BUFF |
              CAAM_MCFGR_WDE | CAAM_MCFGR_LARGE_BURST);

  modifyreg32(IMXRT_CAAM_JRSTART, 0, CAAM_JRSTART_JR0);

  ret = imxrt_caam_ring_init();
  if (ret < 0)
    {
      return ret;
    }

  ret = imxrt_caam_rng_init();
  if (ret < 0)
    {
      return ret;
    }

  g_initialized = true;
  return OK;
}

/*****************************************************************************
 * Name: imxrt_caam_get_random
 *****************************************************************************/

int imxrt_caam_get_random(uint8_t *buffer, size_t buflen)
{
  size_t done = 0;
  int ret;

  if (buffer == NULL || buflen == 0)
    {
      return -EINVAL;
    }

  ret = imxrt_caam_initialize();
  if (ret < 0)
    {
      return ret;
    }

  while (done < buflen)
    {
      size_t chunk = buflen - done;

      if (chunk > sizeof(g_rngbuf))
        {
          chunk = sizeof(g_rngbuf);
        }

      memset(g_rngbuf, 0, sizeof(g_rngbuf));
      imxrt_caam_clean(g_rngbuf, sizeof(g_rngbuf));

      g_desc[0] = CAAM_DESC_HDR(4);
      g_desc[1] = CAAM_OP_RNG_GENERATE;
      g_desc[2] = CAAM_FIFO_STORE_RNG | sizeof(g_rngbuf);
      g_desc[3] = (uint32_t)(uintptr_t)g_rngbuf;

      ret = imxrt_caam_run();
      if (ret < 0)
        {
          memset(buffer, 0, buflen);
          return ret;
        }

      imxrt_caam_invalidate(g_rngbuf, sizeof(g_rngbuf));
      memcpy(buffer + done, g_rngbuf, chunk);
      done += chunk;
    }

  /* Leave nothing behind for the next caller to find. */

  memset(g_rngbuf, 0, sizeof(g_rngbuf));
  return OK;
}

#endif /* CONFIG_IMXRT_CAAM */
