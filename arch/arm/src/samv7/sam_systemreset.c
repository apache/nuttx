/****************************************************************************
 * arch/arm/src/samv7/sam_systemreset.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David Sidrane <david_s5@nscdg.com>
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
#include <assert.h>

#include <nuttx/board.h>
#include <arch/samv7/chip.h>

#include "up_arch.h"
#include "chip/sam_rstc.h"

#ifdef CONFIG_SAMV7_SYSTEMRESET

/****************************************************************************
 * Public functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Internal reset logic.
 *
 ****************************************************************************/

void up_systemreset(void)
{
  uint32_t rstcr;
#if defined(CONFIG_SAMV7_EXTRESET_ERST) && CONFIG_SAMV7_EXTRESET_ERST != 0
  uint32_t rstmr;
#endif

  rstcr  = (RSTC_CR_PROCRST | RSTC_CR_KEY);

#if defined(CONFIG_SAMV7_EXTRESET_ERST) && CONFIG_SAMV7_EXTRESET_ERST != 0
  rstcr |= RSTC_CR_EXTRST;

  rstmr  = getreg32(SAM_RSTC_MR);
  rstmr &= ~RSTC_MR_ERSTL_MASK;
  rstmr &= RSTC_MR_ERSTL(CONFIG_SAMV7_EXTRESET_ERST-1) | RSTC_MR_KEY;
  putreg32(rstmr, SAM_RSTC_MR);
#endif

  putreg32(rstcr, SAM_RSTC_CR);

  /* Wait for the reset */

  for (; ; );
}
#endif /* CONFIG_SAMV7_SYSTEMRESET */
