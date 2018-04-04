/****************************************************************************
 * configs/lc823450-xgevk/src/lc823450_mux.c
 *
 *   Copyright 2017 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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
#include <nuttx/arch.h>
#include <stdint.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "lc823450_syscontrol.h"

#include "lc823450-xgevk_mux.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_init_default_mux
 *
 * Description:
 *   Initial pinmux settings at boot
 *
 ****************************************************************************/

void up_init_default_mux(void)
{
  modifyreg32(MCLKCNTAPB, 0,
              MCLKCNTAPB_PORT0_CLKEN |
              MCLKCNTAPB_PORT1_CLKEN |
              MCLKCNTAPB_PORT2_CLKEN |
              MCLKCNTAPB_PORT3_CLKEN |
              MCLKCNTAPB_PORT4_CLKEN |
              MCLKCNTAPB_PORT5_CLKEN);

  modifyreg32(MRSTCNTAPB, 0,
              MRSTCNTAPB_PORT0_RSTB |
              MRSTCNTAPB_PORT1_RSTB |
              MRSTCNTAPB_PORT2_RSTB |
              MRSTCNTAPB_PORT3_RSTB |
              MRSTCNTAPB_PORT4_RSTB |
              MRSTCNTAPB_PORT5_RSTB);

  putreg32(PORT0_MUX, PMDCNT0);
  putreg32(PORT0_PUPD, PUDCNT0);
  putreg32(PORT0_DRV, PTDRVCNT0);
  putreg32(PORT0_DAT, rP0DT);
  putreg32(PORT0_DIR, rP0DRC);

  putreg32(PORT1_MUX, PMDCNT1);
  putreg32(PORT1_PUPD, PUDCNT1);
  putreg32(PORT1_DRV, PTDRVCNT1);
  putreg32(PORT1_DAT, rP1DT);
  putreg32(PORT1_DIR, rP1DRC);

  putreg32(PORT2_MUX, PMDCNT2);
  putreg32(PORT2_PUPD, PUDCNT2);
  putreg32(PORT2_DRV, PTDRVCNT2);
  putreg32(PORT2_DAT, rP2DT);
  putreg32(PORT2_DIR, rP2DRC);

  putreg32(PORT3_MUX, PMDCNT3);
  putreg32(PORT3_PUPD, PUDCNT3);
  putreg32(PORT3_DRV, PTDRVCNT3);
  putreg32(PORT3_DAT, rP3DT);
  putreg32(PORT3_DIR, rP3DRC);

  putreg32(PORT4_MUX, PMDCNT4);
  putreg32(PORT4_PUPD, PUDCNT4);
  putreg32(PORT4_DRV, PTDRVCNT4);
  putreg32(PORT4_DAT, rP4DT);
  putreg32(PORT4_DIR, rP4DRC);

  putreg32(PORT5_MUX, PMDCNT5);
  putreg32(PORT5_PUPD, PUDCNT5);
  putreg32(PORT5_DRV, PTDRVCNT5);
  putreg32(PORT5_DAT, rP5DT);
  putreg32(PORT5_DIR, rP5DRC);

  putreg32(PORT6_PUPD, PUDCNT6);
  putreg32(PORT6_DRV, PTDRVCNT6);
}
