/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_lowinit.c
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

#include <assert.h>

#include <nuttx/arch.h>
#include <arch/pic32mx/cp0.h>

#include "mips_internal.h"
#include "chip.h"
#include "pic32mx.h"
#include "pic32mx_bmx.h"
#include "pic32mx_che.h"
#include "pic32mx_ddp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum Frequencies ******************************************************/

#define MAX_FLASH_HZ     30000000 /* Maximum FLASH speed (Hz) */
#define MAX_PBCLOCK      80000000 /* Max peripheral bus speed (Hz) */

/* Sanity checks ************************************************************/

/* Make sure that the selected clock parameters are sane */

#define CALC_SYSCLOCK  (((BOARD_PLL_INPUT / BOARD_PLL_IDIV) * BOARD_PLL_MULT) / BOARD_PLL_ODIV)
#if CALC_SYSCLOCK != BOARD_CPU_CLOCK
#  error "Bad BOARD_CPU_CLOCK calculation in board.h"
#endif

#define CALC_PBCLOCK  (CALC_SYSCLOCK / BOARD_PBDIV)
#if CALC_PBCLOCK != BOARD_PBCLOCK
#  error "Bad BOARD_PBCLOCK calculation in board.h"
#endif

#if CALC_PBCLOCK > MAX_PBCLOCK
#  error "PBCLOCK exceeds maximum value"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_waitstates
 *
 * Description:
 *   Configure the optimal number of FLASH wait states.
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static inline void pic32mx_waitstates(void)
{
#ifdef CHIP_CHE
  unsigned int nwaits;
  unsigned int residual;
#endif

  /* Disable DRM wait states */

  putreg32(BMX_CON_BMXWSDRM, PIC32MX_BMX_CONCLR);

#ifdef CHIP_CHE
  /* Configure pre-fetch cache FLASH wait states */

  residual = BOARD_CPU_CLOCK;
  nwaits   = 0;

  while (residual > MAX_FLASH_HZ)
    {
      nwaits++;
      residual -= MAX_FLASH_HZ;
    }

  DEBUGASSERT(nwaits < 8);

  /* Set the FLASH wait states -- clearing all other bits! */

  putreg32(nwaits, PIC32MX_CHE_CON);
#endif
}

/****************************************************************************
 * Name: pic32mx_cache
 *
 * Description:
 *   Enable caching.
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static inline void pic32mx_cache(void)
{
  register uint32_t regval;

  /* Enable prefetch on all regions */

#ifdef CHIP_CHE
  regval = getreg32(PIC32MX_CHE_CON);
  regval |= CHE_CON_PREFEN_ALL;
  putreg32(regval, PIC32MX_CHE_CON);
#endif

  /* Enable cache on KSEG 0 in the CP0 CONFIG register */

  asm("\tmfc0 %0,$16,0\n" :  "=r"(regval));
  regval &= ~CP0_CONFIG_K23_MASK;
  regval |= CP0_CONFIG_K23_CACHEABLE;
  asm("\tmtc0 %0,$16,0\n" : : "r" (regval));
}

/****************************************************************************
 * Name: pic32mx_disable_jtag
 *
 * Description:
 *   Disable the JTAG connection
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static inline void pic32mx_disable_jtag(void)
{
#if defined(CHIP_PIC32MX3) || defined(CHIP_PIC32MX4) || defined(CHIP_PIC32MX5) || \
    defined(CHIP_PIC32MX6) || defined(CHIP_PIC32MX7)
  register uint32_t regval;

  regval = getreg32(PIC32MX_DDP_CON);

  /* Clear the JTAG enable bit */

  regval &= ~DDP_CON_JTAGEN;
  putreg32(regval, PIC32MX_DDP_CON);
#endif
}

/****************************************************************************
 * Name: pic32mx_enable_jtag
 *
 * Description:
 *   Enable the JTAG connection
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static inline void pic32mx_enable_jtag(void)
{
#if defined(CHIP_PIC32MX3) || defined(CHIP_PIC32MX4) || defined(CHIP_PIC32MX5) || \
    defined(CHIP_PIC32MX6) || defined(CHIP_PIC32MX7)
  register uint32_t regval;

  regval = getreg32(PIC32MX_DDP_CON);

  /* Set the JTAG enable bit */

  regval |= DDP_CON_JTAGEN;
  putreg32(regval, PIC32MX_DDP_CON);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_lowinit
 *
 * Description:
 *   This performs basic low-level initialization of the system.
 *
 * Assumptions:
 *   Interrupts have not yet been enabled.
 *
 ****************************************************************************/

void pic32mx_lowinit(void)
{
  /* Initialize FLASH wait states */

  pic32mx_waitstates();

  /* Enable caching */

  pic32mx_cache();

  /* Initialize a console (probably a serial console) */

  pic32mx_consoleinit();

  /* Perform early serial initialization (so that we will have debug output
   * available as soon as possible).
   */

#ifdef USE_EARLYSERIALINIT
  up_earlyserialinit();
#endif

#ifdef CONFIG_PIC32MX_JTAG_ENABLE
  pic32mx_enable_jtag();
#else
  pic32mx_disable_jtag();
#endif

  /* Perform board-level initialization */

  pic32mx_boardinitialize();
}
