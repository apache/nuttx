/************************************************************************************
 * arch/arm/src/sam34/hardware/sam_pinmap.h
 *
 *   Copyright (C) 2012-2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_PINMAP_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_PINMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#if defined(CONFIG_ARCH_CHIP_SAM3U)
#  include "hardware/sam3u_pinmap.h"
#elif defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  include "hardware/sam3x_pinmap.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4CM)
#  include "hardware/sam4cm_pinmap.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4E)
#  include "hardware/sam4e_pinmap.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4L)
#  include "hardware/sam4l_pinmap.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4S)
#  include "hardware/sam4s_pinmap.h"
#else
#  error Unrecognized SAM architecture
#endif

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_PINMAP_H */
