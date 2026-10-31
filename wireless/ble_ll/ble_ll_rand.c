/****************************************************************************
 * wireless/ble_ll/ble_ll_rand.c
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
 * Derived from Apache Mynewt NimBLE (nimble/controller/src/ble_ll_rand.c).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "ble.h"
#include "ble_hw.h"
#include "ble_ll.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* for jrand48 */

#define _XOPEN_SOURCE
#define IS_RNUM_BUF_END(x) \
  (x == &g_ble_ll_rnum_buf[CONFIG_BLE_LL_RNG_BUFSIZE - 1])

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is a simple circular buffer for holding N samples of random data */

struct ble_ll_rnum_data
{
  uint8_t *rnd_in;
  uint8_t *rnd_out;
  volatile uint8_t rnd_size;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct ble_ll_rnum_data g_ble_ll_rnum_data;

uint8_t g_ble_ll_rnum_buf[CONFIG_BLE_LL_RNG_BUFSIZE];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_ll_rand_sample
 *
 ****************************************************************************/

void ble_ll_rand_sample(uint8_t rnum)
{
  irqstate_t sr;

  sr = enter_critical_section();
  if (g_ble_ll_rnum_data.rnd_size < CONFIG_BLE_LL_RNG_BUFSIZE)
    {
      ++g_ble_ll_rnum_data.rnd_size;
      g_ble_ll_rnum_data.rnd_in[0] = rnum;
      if (IS_RNUM_BUF_END(g_ble_ll_rnum_data.rnd_in))
        {
          g_ble_ll_rnum_data.rnd_in = g_ble_ll_rnum_buf;
        }
      else
        {
          ++g_ble_ll_rnum_data.rnd_in;
        }
    }
  else
    {
      /* Stop generating random numbers as we are full */

      ble_hw_rng_stop();
    }

  leave_critical_section(sr);
}

/****************************************************************************
 * Name: ble_ll_rand_data_get
 *
 * Description:
 *   Get 'len' bytes of random data
 *
 ****************************************************************************/

int ble_ll_rand_data_get(uint8_t *buf, uint8_t len)
{
  uint8_t rnums;
  irqstate_t sr;

  while (len != 0)
    {
      sr = enter_critical_section();
      rnums = g_ble_ll_rnum_data.rnd_size;
      if (rnums > len)
        {
          rnums = len;
        }

      len -= rnums;
      g_ble_ll_rnum_data.rnd_size -= rnums;
      while (rnums)
        {
          buf[0] = g_ble_ll_rnum_data.rnd_out[0];
          if (IS_RNUM_BUF_END(g_ble_ll_rnum_data.rnd_out))
            {
              g_ble_ll_rnum_data.rnd_out = g_ble_ll_rnum_buf;
            }
          else
            {
              ++g_ble_ll_rnum_data.rnd_out;
            }

          ++buf;
          --rnums;
        }

      leave_critical_section(sr);

      /* Make sure rng is started! */

      ble_hw_rng_start();

      /* Wait till bytes are in buffer. */

      if (len)
        {
          while ((g_ble_ll_rnum_data.rnd_size < len) &&
                 (g_ble_ll_rnum_data.rnd_size < CONFIG_BLE_LL_RNG_BUFSIZE))
            {
              /* Spin here */
            }
        }
    }

  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: ble_ll_rand
 *
 * Description:
 *   Simple wrapper to allow easy replacement of rand()
 *
 ****************************************************************************/

uint32_t ble_ll_rand(void)
{
  static unsigned short xsubi[3];
  static bool init = true;

  if (init)
    {
      init = false;
      ble_ll_rand_data_get((uint8_t *)xsubi, sizeof(xsubi));
    }

  return (uint32_t)jrand48(xsubi);
}

/****************************************************************************
 * Name: ble_ll_rand_prand_get
 *
 * Description:
 *   Called to obtain a "prand" as defined in core V4.2 Vol 6 Part B 1.3.2.2
 *
 * Input Parameters:
 *   prand
 *
 ****************************************************************************/

void ble_ll_rand_prand_get(uint8_t *prand)
{
  uint16_t sum;

  while (1)
    {
      /* Get 24 bits of random data */

      ble_ll_rand_data_get(prand, 3);

      /* Prand cannot be all zeros or 1's. */

      sum = prand[0] + prand[1] + prand[2];
      if ((sum != 0) && (sum != (3 * 0xff)))
        {
          break;
        }
    }

  /* Upper two bits must be 01 */

  prand[2] &= ~0xc0;
  prand[2] |= 0x40;
}

/****************************************************************************
 * Name: ble_ll_rand_start
 *
 * Description:
 *   Start the generation of random numbers
 *
 * Returned Value:
 *   int
 *
 ****************************************************************************/

int ble_ll_rand_start(void)
{
  /* Start the generation of numbers if we are not full */

  if (g_ble_ll_rnum_data.rnd_size < CONFIG_BLE_LL_RNG_BUFSIZE)
    {
      ble_hw_rng_start();
    }

  return 0;
}

/****************************************************************************
 * Name: ble_ll_rand_init
 *
 * Description:
 *   Initialize LL random number generation. Should be called only once on
 *   initialization.
 *
 * Returned Value:
 *   int
 *
 ****************************************************************************/

int ble_ll_rand_init(void)
{
  g_ble_ll_rnum_data.rnd_in = g_ble_ll_rnum_buf;
  g_ble_ll_rnum_data.rnd_out = g_ble_ll_rnum_buf;
  ble_hw_rng_init(ble_ll_rand_sample, 1);
  return 0;
}
