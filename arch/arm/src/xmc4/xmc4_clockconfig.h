/************************************************************************************
 * arch/arm/src/xmc4/xmc4_clockconfig.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_XMC4_XMC4_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_XMC4_XMC4_CLOCKCONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/************************************************************************************
 * Preprocessor Definitions
 ************************************************************************************/

#define OFI_FREQUENCY 24000000 /* Frequency of internal Backup Clock Source */
#define OSI_FREQUENCY 32768    /* Frequency of internal Slow Clock Source */

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: xmc4_clock_configure
 *
 * Description:
 *   Called to initialize the XMC4xxx chip.  This does whatever setup is
 *   needed to put the  MCU in a usable state.  This includes the
 *   initialization of clocking using the settings in board.h.
 *
 ****************************************************************************/

void xmc4_clock_configure(void);

/****************************************************************************
 * Name: xmc4_get_coreclock
 *
 * Description:
 *   Return the current core clock frequency, fCPU.
 *
 ****************************************************************************/

uint32_t xmc4_get_coreclock(void);

/****************************************************************************
 * Name: xmc4_get_periphclock
 *
 * Description:
 *   The peripheral clock is either fCPU or fCPU/2, depending on the state
 *   of the peripheral divider.
 *
 ****************************************************************************/

uint32_t xmc4_get_periphclock(void);

#endif /* __ARCH_ARM_SRC_XMC4_XMC4_CLOCKCONFIG_H */
