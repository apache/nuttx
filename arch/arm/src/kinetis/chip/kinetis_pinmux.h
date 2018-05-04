/********************************************************************************************
 * arch/arm/src/kinetis/chip/kinetis_pinmux.h
 *
 *   Copyright (C) 2011, 2015-2016, 2018 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_PINMUX_H
#define __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_PINMUX_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* This file is just a wrapper around pin muxing header files for the Kinetis family selected
 * by the logic in chip.h.
 */

#if defined(KINETIS_K20)
#  include "chip/kinetis_k20pinmux.h"
#elif defined(KINETIS_K28)
#  include "chip/kinetis_k28pinmux.h"
#elif defined(KINETIS_K40)
#  include "chip/kinetis_k40pinmux.h"
#elif defined(KINETIS_K60)
#  include "chip/kinetis_k60pinmux.h"
#elif defined(KINETIS_K64)
#  include "chip/kinetis_k64pinmux.h"
#elif defined(KINETIS_K66)
#  include "chip/kinetis_k66pinmux.h"
#else
#  error "No pin multiplexing for this Kinetis part"
#endif

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_CHIP_KINETIS_PINMUX_H */
