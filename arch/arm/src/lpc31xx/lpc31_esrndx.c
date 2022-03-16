/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_esrndx.c
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

/* References:
 *   - UM10314 LPC3130/31 User manual Rev. 1.01 — 9 September 2009
 *   - lpc313x.cdl.drivers.zip example driver code
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include "arm_internal.h"
#include "lpc31_cgudrvr.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_esrndx
 *
 * Description:
 *   Given a clock ID, return the index of the corresponding ESR
 *   register (or ESRNDX_INVALID if there is no ESR associated with
 *   this clock ID).  Indexing of ESRs differs slightly from the clock
 *   ID:  There are 92 clock IDs but only 89 ESR regisers. There are no
 *   ESR registers for:
 *
 *  CLKID_I2SRXBCK0         Clock ID 87: I2SRX_BCK0
 *  CLKID_I2SRXBCK1,        Clock ID 88: I2SRX_BCK1
 *
 * and
 *
 *  CLKID_SYSCLKO           Clock ID 91: SYSCLK_O
 *
 ****************************************************************************/

int lpc31_esrndx(enum lpc31_clockid_e clkid)
{
  int esrndx = (int)clkid;

  /* There are 89 Enable Select Registers (ESR).  Indexing for these
   * registers is identical to indexing to other registers (like PCR),
   * except that there are no ESR registers for
   *
   *  CLKID_I2SRXBCK0         Clock ID 87: I2SRX_BCK0
   *  CLKID_I2SRXBCK1,        Clock ID 88: I2SRX_BCK1
   *
   * and
   *
   *  CLKID_SYSCLKO           Clock ID 91: SYSCLK_O
   */

  switch (clkid)
  {
    /* There are no ESR registers corresponding to the following
     * three clocks:
     */

    case CLKID_I2SRXBCK0:
    case CLKID_I2SRXBCK1:
    case CLKID_SYSCLKO:
      esrndx = ESRNDX_INVALID;
      break;

    /* These clock IDs are a special case and need to be adjusted
     * by two:
     *
     * CLKID_SPICLK          Clock ID 89, ESR index 87
     * CLKID_SPICLKGATED     Clock ID 90, ESR index 88
     */

    case CLKID_SPICLK:
    case CLKID_SPICLKGATED:
      esrndx = esrndx - 2;
      break;

    /* The rest of the indices match up and we don't have to do anything. */

    default:
      break;
  }

  return esrndx;
}
