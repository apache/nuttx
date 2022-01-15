/****************************************************************************
 * arch/sparc/src/bm3823/bm3823-lowinit.c
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

#include "bm3823-config.h"
#include "up_internal.h"
#include "bm3823.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_WDTO_15MS)
#  define WDTO_VALUE WDTO_15MS
#elif defined(CONFIG_WDTO_30MS)
#  define WDTO_VALUE WDTO_30MS
#elif defined(CONFIG_WDTO_60MS)
#  define WDTO_VALUE WDTO_60MS
#elif defined(CONFIG_WDTO_120MS)
#  define WDTO_VALUE WDTO_120MS
#elif defined(CONFIG_WDTO_1250MS)
#  define WDTO_VALUE WDTO_250MS
#elif defined(CONFIG_WDTO_500MS)
#  define WDTO_VALUE WDTO_500MS
#elif defined(CONFIG_WDTO_1S)
#  define WDTO_VALUE WDTO_1S
#elif defined(CONFIG_WDTO_2S)
#  define WDTO_VALUE WDTO_2S
#elif defined(CONFIG_WDTO_4S)
#  define WDTO_VALUE WDTO_4S
#else /* if defined(CONFIG_WDTO_8S) */
#  define WDTO_VALUE WDTO_8S
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
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) up_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Name: up_wdtinit
 *
 * Description:
 *   Initialize the watchdog per the NuttX configuration.
 *
 ****************************************************************************/

void wdt_disable(void)
{
}

void wdt_enable(uint32_t wdt)
{
}

static inline void up_wdtinit(void)
{
#ifdef CONFIG_SPARC_WDT
  wdt_enable(WDTO_VALUE);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowinit
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void up_lowinit(void)
{
  uint32_t *dest;

  /* Disable the watchdog timer */

  wdt_disable();

  /* Initialize the watchdog timer */

  up_wdtinit();

  /* Initialize a console (probably a serial console) */

  up_consoleinit();

  showprogress('A');

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_bss_start; dest < &_end; )
    {
      *dest++ = 0;
    }

  showprogress('B');
  /* Perform early serial initialization (so that we will have debug output
   * available as soon as possible).
   */

#ifdef USE_EARLYSERIALINIT
  up_earlyserialinit();
#endif

  /* Perform board-level initialization */

  bm3823_boardinitialize();

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');
}
