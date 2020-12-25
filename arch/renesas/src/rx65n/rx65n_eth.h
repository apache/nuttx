/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_eth.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_RX65N_ETH_H
#define __ARCH_RENESAS_SRC_RX65N_RX65N_ETH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/wdog.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Understood PHY types */

/* Definitions for use with rx65n_phy_boardinitialize */

#define RX65N_NETHERNET 1

#define EMAC0_INTF 0

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Function: rx65n_ethinitialize
 *
 * Description:
 *   Initialize the EMAC driver.
 *
 * Input Parameters:
 *   intf - If multiple EMAC peripherals are supported, this identifies the
 *     the EMAC peripheral being initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called very early in the initialization sequence.
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_EMAC0
int rx65n_ethinitialize(int intf);

/****************************************************************************
 * Function: rx65n_poll_expiry
 *
 * Description:
 *   Poll Expiry timer
 *
 * Input Parameters:
 *   arg  - Input argument
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void rx65n_poll_expiry(wdparm_t arg);

/****************************************************************************
 * Function: rx65n_txtimeout_expiry
 *
 * Description:
 *   txtimeout timer
 *
 * Input Parameters:
 *   arg  - Input argument
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void rx65n_txtimeout_expiry(wdparm_t arg);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RENESAS_SRC_RX65N_RX65N_ETH_H */