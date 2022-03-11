/****************************************************************************
 * arch/arm/src/imx6/imx_clockconfig.c
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

#include "arm_internal.h"
#include "hardware/imx_ccm.h"
#include "imx_config.h"
#include "imx_clockconfig.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_clockconfig
 *
 * Description:
 *   Called to initialize the i.MX6.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void imx_clockconfig(void)
{
  /* Don't change the current basic clock configuration if we are running
   * from SDRAM.  In this case, some bootloader logic has already configured
   * clocking and SDRAM.  We are pretty much committed to using things the
   * way that the bootloader has left them.
   *
   * Clocking will be configured at 792 MHz initially when started via
   * U-Boot.  The Linux kernel will uses the CPU frequency scaling code
   * which will switch the processor frequency between 400 MHz and 1GHz based
   * on load and temperature.  For now, NuttX simply leaves the clocking at
   * 792MHz.
   */

#ifndef CONFIG_IMX6_BOOT_SDRAM
#  warning Missing logic
#endif
}
