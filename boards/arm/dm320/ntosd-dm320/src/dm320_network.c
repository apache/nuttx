/****************************************************************************
 * boards/arm/dm320/ntosd-dm320/src/dm320_network.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_DM90x0)

#include <debug.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "dm320_memorymap.h"
#include "dm320_emif.h"
#include "dm320_gio.h"

extern void dm9x_initialize(void);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_netinitialize
 ****************************************************************************/

void arm_netinitialize(void)
{
  /* CS4 is used for DM9000A Ethernet.  Interrupt is provided via GIO6
   * which must be configured to interrupt on the rising edge.  Bus
   * width is 16-bits.
   */

  ninfo("CS4CTRL1=%04x CS4CTRL2=%04x\n",
        getreg16(DM320_EMIF_CS4CTRL1), getreg16(DM320_EMIF_CS4CTRL2));

  /* It is assumed that bootloader has already configured CS4.  Here,
   * we will only make certain that the GIO is properly configured
   */

  GIO_INPUT(GIO_DM9000A_INT);
  GIO_NONINVERTED(GIO_DM9000A_INT);
  GIO_INTERRUPT(GIO_DM9000A_INT);
  GIO_RISINGEDGE(GIO_DM9000A_INT);

  ninfo("GIO DIR0=%04x INV0=%04x IRQPORT=%04x IRQEDGE=%04x\n",
        getreg16(DM320_GIO_DIR0), getreg16(DM320_GIO_INV0),
        getreg16(DM320_GIO_IRQPORT), getreg16(DM320_GIO_IRQEDGE));

  /* Then initialize the driver */

  dm9x_initialize();
}

#endif /* CONFIG_NET && CONFIG_NET_DM90x0 */
