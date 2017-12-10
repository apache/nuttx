/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_emc.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Parts of this file were adapted from sample code provided for the LPC54xx
 * family from NXP which has a compatible BSD license.
 *
 *   Copyright (c) 2016, Freescale Semiconductor, Inc.
 *   Copyright (c) 2016 - 2017 , NXP
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

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC54_EMC_H
#define __ARCH_ARM_SRC_LPC54XX_LPC54_EMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "lpc54_config.h"

#ifdef CONFIG_LPC54_EMC

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* EMC Feedback clock input source selection */

enum _emc_fbclk_src_e
{
  EMC_INTLOOPBACK = 0, /* Use the internal loop back from EMC_CLK output */
  EMC_FBCLLK           /* Use the external EMC_FBCLK input */
};

/* EMC module basic configuration structure */

struct emc_config_s
{
  bool bigendian;    /* True: Memory is big-endian */
  uint8_t clksrc;    /* The feedback clock source. */
  uint8_t clkdiv;    /* EMC_CLK = AHB_CLK / (emc_clkDiv + 1). */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_emc_initialize
 *
 * Description:
 *   This function enables the EMC clock, initializes the emc system 
 *   configuration, and enable the EMC module.
 *
 ****************************************************************************/

void lpc54_emc_initialize(uintptr_t base,
                          FAR const struct emc_config_s *config);

#endif /* CONFIG_LPC54_EMC */
#endif /* __ARCH_ARM_SRC_LPC54XX_LPC54_EMC_H */
