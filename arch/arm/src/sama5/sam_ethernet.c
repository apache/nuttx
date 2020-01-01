/****************************************************************************
 * arch/arm/src/sama5/sam_ethernet.c
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include "sam_ethernet.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: up_gmac_initialize
 *
 * Description:
 *   Initialize the GMAC driver
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_GMAC
static inline void up_gmac_initialize(void)
{
  int ret;

  /* Initialize the GMAC driver */

  ret = sam_gmac_initialize();
  if (ret < 0)
    {
      nerr("ERROR: sam_gmac_initialize failed: %d\n", ret);
    }
}
#else
#  define up_gmac_initialize()
#endif

/****************************************************************************
 * Function: up_emac_initialize
 *
 * Description:
 *   Initialize the EMAC driver
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_EMACA)
static inline void up_emac_initialize(void)
{
  int ret;

  /* Initialize the EMAC driver */

  ret = sam_emac_initialize();
  if (ret < 0)
    {
      nerr("ERROR: up_emac_initialize failed: %d\n", ret);
    }
}
#elif defined(CONFIG_SAMA5_EMACB)
static inline void up_emac_initialize(void)
{
  int ret;

#if defined(CONFIG_SAMA5_EMAC0)
  /* Initialize the EMAC0 driver */

  ret = sam_emac_initialize(EMAC0_INTF);
  if (ret < 0)
    {
      nerr("ERROR: up_emac_initialize(EMAC0) failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_SAMA5_EMAC1)
  /* Initialize the EMAC1 driver */

  ret = sam_emac_initialize(EMAC1_INTF);
  if (ret < 0)
    {
      nerr("ERROR: up_emac_initialize(EMAC1) failed: %d\n", ret);
    }
#endif
}
#else
#  define up_emac_initialize()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: up_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in up_initialize.c.  If both the EMAC
 *   and GMAC are enabled, then this single entry point must initialize
 *   both.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

void up_netinitialize(void)
{
  /* The first device registered with be ETH0 and the second ETH1 */

#ifdef CONFIG_SAMA5_GMAC_ISETH0
  up_gmac_initialize();
  up_emac_initialize();
#else
  up_emac_initialize();
  up_gmac_initialize();
#endif
}

#endif /* CONFIG_NET */
