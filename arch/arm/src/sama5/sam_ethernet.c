/****************************************************************************
 * arch/arm/src/sama5/sam_ethernet.c
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
 * Function: arm_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in arm_initialize.c.  If both the EMAC
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

void arm_netinitialize(void)
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
