/****************************************************************************
 * arch/arm/src/imxrt/imxrt_daisy.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: David Sidrane <david_s5@nscdg.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include "chip.h"
#include "arm_arch.h"
#include "hardware/imxrt_daisy.h"
#include "imxrt_iomuxc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DAISY_INDEX_INVALID     255
#define DAISY_SEL_INVALID       255
#define ALT0                    0
#define ALT1                    1
#define ALT2                    2
#define ALT3                    3
#define ALT4                    4
#define ALT5                    5
#define ALT6                    6
#define ALT7                    7
#define ALT8                    8
#define ALT9                    9

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct imxrt_daisy_entry_t
{
  uint8_t   index;
  uint8_t   sel;
};

struct imxrt_daisy_t
{
  struct imxrt_daisy_entry_t alts[10];
};

/* Include chip-specific daisy input selection */

#if defined(CONFIG_ARCH_FAMILY_IMXRT102x)
#  include "imxrt102x_daisy.c"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT105x)
#  include "imxrt105x_daisy.c"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT106x)
#  include "imxrt106x_daisy.c"
#else
#  error Unrecognized i.MX RT architecture
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_daisy_select
 ****************************************************************************/

void imxrt_daisy_select(unsigned int index, unsigned int alt)
{
  uintptr_t address;
  const struct imxrt_daisy_t *daisy = &g_daisy_select[index];

  index = daisy->alts[alt].index;
  if (index != DAISY_INDEX_INVALID)
    {
      alt = daisy->alts[alt].sel;
      address = IMXRT_IOMUXC_BASE + IMXRT_INPUT_INDEX2OFFSET(index);
      putreg32(alt, address);
    }
}
