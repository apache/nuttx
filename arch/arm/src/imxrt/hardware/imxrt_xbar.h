/************************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_abar.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_XBAR_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_XBAR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "hardware/imxrt_memorymap.h"

#if defined(CONFIG_ARCH_FAMILY_IMXRT102x)
#  include "hardware/rt102x/imxrt102x_xbar.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT105x)
#  include "hardware/rt105x/imxrt105x_xbar.h"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT106x)
#  include "hardware/rt106x/imxrt106x_xbar.h"
#else
#  error Unrecognized i.MX RT architecture
#endif


/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define IMXRT_SEL_PER_REG    2

#define IMXRT_SEL0_SHIFTS    0    /* Bits 0-6: Input (XBARA_INn) to be muxed to
                                   *           XBARA_OUTm */
#define IMXRT_SEL0_MASK      (0x7f << IMXRT_SEL0_SHIFTS)
                                  /* Bit 7: Reserved */
#define IMXRT_SEL1_SHIFTS    8    /* Bits 8-14: Input (XBARA_INn) to be muxed to
                                   *   XBARA_OUTm */
#define IMXRT_SEL1_MASK      (0x7f << IMXRT_SEL1_SHIFTS)
                                  /* Bit 15 Reserved */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_XBAR_H */
