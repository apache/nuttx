/****************************************************************************
 * arch/arm/src/xmc4/xmc4_ccu4.c
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
 * XMC CCU Driver
 *
 * For now, this file contains only helper methods mandatory for xmc tickless
 * feature. Contibutions are welcomed.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "arm_internal.h"
#include "hardware/xmc4_ccu4.h"
#include "xmc4_ccu4.h"

#define CCU4_NDIVIDERS 15

static const uint8_t g_log2divider[CCU4_NDIVIDERS] =
{
        1,  /* TIMER_CLOCK1 -> div2 */
        2,  /* TIMER_CLOCK1 -> div4 */
        3,  /* TIMER_CLOCK2 -> div8 */
        4,  /* TIMER_CLOCK2 -> div16 */
        5,  /* TIMER_CLOCK3 -> div32 */
        6,  /* TIMER_CLOCK3 -> div64 */
        7,  /* TIMER_CLOCK4 -> div128 */
        8,  /* TIMER_CLOCK4 -> div256 */
        9,  /* TIMER_CLOCK4 -> div512 */
        10, /* TIMER_CLOCK4 -> div1024 */
        11, /* TIMER_CLOCK4 -> div2048 */
        12, /* TIMER_CLOCK4 -> div4096 */
        13, /* TIMER_CLOCK4 -> div8192 */
        14, /* TIMER_CLOCK4 -> div16384 */
        15  /* TIMER_CLOCK4 -> div32769 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_ccu4_divfreq_lookup
 *
 * Description:
 *  Given the TC input frequency (Ftcin) and a divider index, return the
 *  value of the divided frequency
 *
 * Input Parameters:
 *   ftcin - TC input frequency
 *   ndx   - Divider index
 *
 * Returned Value:
 *   The divided frequency value
 *
 ****************************************************************************/

static uint32_t xmc4_ccu4_divfreq_lookup(uint32_t ftcin, int ndx)
{
  return ftcin >> g_log2divider[ndx];
}

/****************************************************************************
 * Name: xmc4_ccu4_freqdiv_lookup
 *
 * Description:
 *  Given the TC input frequency (Ftcin) and a divider index, return the
 *  value of the Ftcin divider.
 *
 * Input Parameters:
 *   ftcin - TC input frequency
 *   ndx   - Divider index
 *
 * Returned Value:
 *   The Ftcin input divider value
 *
 ****************************************************************************/

static int xmc4_ccu4_freqdiv_lookup(uint32_t ftcin, int ndx)
{
  return 1 << g_log2divider[ndx];
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_ccu4_divisor
 *
 * Description:
 *   Finds the best MCK divisor given the timer frequency and MCK.  The
 *   result is guaranteed to satisfy the following equation:
 *
 *     (Ftcin / (div * 65536)) <= freq <= (Ftcin / dev)
 *
 *   where:
 *     freq  - the desired frequency
 *     Ftcin - The timer/counter input frequency
 *     div   - With DIV being the highest possible value.
 *
 * Input Parameters:
 *   frequency  Desired timer frequency.
 *   div        Divisor value.
 *   pssiv      PSSIV field value for divisor.
 *
 * Returned Value:
 *   Zero (OK) if a proper divisor has been found, otherwise a negated errno
 *   value indicating the nature of the failure.
 *
 ****************************************************************************/

int xmc4_ccu4_divisor(uint32_t frequency, uint32_t *div, uint32_t *pssiv)
{
  uint32_t ftcin = BOARD_CCU_FREQUENCY;
  int ndx = 0;

  tmrinfo("frequency=%" PRIu32 "\n", frequency);

  /* Satisfy lower bound.  That is, the value of the divider such that:
   *
   *   frequency >= (tc_input_frequency * 65536) / divider.
   */

  while (frequency < (xmc4_ccu4_divfreq_lookup(ftcin, ndx) >> 16))
    {
      if (++ndx > CCU4_NDIVIDERS)
        {
          /* If no divisor can be found, return -ERANGE */

          tmrerr("ERROR: Lower bound search failed\n");
          return -ERANGE;
        }
    }

  /* Try to maximize DIV while still satisfying upper bound.  That the
   * value of the divider such that:
   *
   *   frequency < tc_input_frequency / divider.
   */

  for (; ndx < (CCU4_NDIVIDERS - 1); ndx++)
    {
      if (frequency > xmc4_ccu4_divfreq_lookup(ftcin, ndx + 1))
        {
          break;
        }
    }

  /* Return the divider value */

  if (div)
    {
      uint32_t value = xmc4_ccu4_freqdiv_lookup(ftcin, ndx);
      tmrinfo("return div=%lu\n", (unsigned long)value);
      *div = value;
    }

  /* Return the PSSIV selection */

  if (pssiv)
    {
      tmrinfo("return pssiv=%d\n", ndx + 1);
      *pssiv = ndx + 1;
    }

  return OK;
}
