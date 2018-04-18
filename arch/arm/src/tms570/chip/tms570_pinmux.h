/****************************************************************************************************
 * arch/arm/src/tms570/chip/tms570_pinmux.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TMS570_CHIP_TMS570_PINMUX_H
#define __ARCH_ARM_SRC_TMS570_CHIP_TMS570_PINMUX_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_TMS570LS0232PZ)
#  error No pin multiplexing for the TMS570LS0232PZ
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0332PZ)
#  include "chip/tms570ls04x03x_pinmux.h"
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0432PZ)
#  include "chip/tms570ls04x03x_pinmux.h"
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714PZ)
#  error No pin multiplexing for the TMS570LS0714PZ
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714PGE)
#  error No pin multiplexing for the TMS570LS0714PGE
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714ZWT)
#  error No pin multiplexing for the TMS570LS0714ZWT
#elif defined(CONFIG_ARCH_CHIP_TMS570LS1227ZWT)
#  error No pin multiplexing for the TMS570LS1227ZWT
#elif defined(CONFIG_ARCH_CHIP_TMS570LS3137ZWT)
#  include "chip/tms570ls04x03x_pinmux.h"
#else
#  error "Unrecognized Hercules chip"
#endif

/****************************************************************************************************
 * Pulbic Type Definitions
 ****************************************************************************************************/

/* Each chip-specific pinmux header file defines initializers for a type like: */

struct tms570_pinmux_s
{
  uint8_t mmrndx;  /* Index to the PINMMR register, 0-30 */
  uint8_t shift;   /* Shift value to isolate the pin field in the PINMMR register */
  uint8_t value;   /* The new value for the pin field in the PINMMR register */
};

#endif /* __ARCH_ARM_SRC_TMS570_CHIP_TMS570_PINMUX_H */
