/****************************************************************************************************
 * arch/arm/src/tms570/hardware/tms570_memorymap.h
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

#ifndef __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_MEMORYMAP_H
#define __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_MEMORYMAP_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_TMS570LS0232PZ)
#  error No memory map for the TMS570LS0232PZ
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0332PZ)
#  include "hardware/tms570ls04x03x_memorymap.h"
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0432PZ)
#  include "hardware/tms570ls04x03x_memorymap.h"
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714PZ)
#  error No memory map for the TMS570LS0714PZ
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714PGE)
#  error No memory map for the TMS570LS0714PGE
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714ZWT)
#  error No memory map for the TMS570LS0714ZWT
#elif defined(CONFIG_ARCH_CHIP_TMS570LS1227ZWT)
#  error No memory map for the TMS570LS1227ZWT
#elif defined(CONFIG_ARCH_CHIP_TMS570LS3137ZWT)
#  include "hardware/tms570ls31xx_memorymap.h"
#else
#  error "Unrecognized Hercules chip"
#endif

#endif /* __ARCH_ARM_SRC_TMS570_HARDWARE_TMS570_MEMORYMAP_H */
