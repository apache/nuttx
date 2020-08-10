/****************************************************************************
 * boards/arm/lc823450/lc823450-xgevk/src/lc823450_mux.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
#include "arm_arch.h"
#include "arm_internal.h"
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
  putreg32(PORT0_DAT, P0DT);
  putreg32(PORT0_DIR, P0DRC);

  putreg32(PORT1_MUX, PMDCNT1);
  putreg32(PORT1_PUPD, PUDCNT1);
  putreg32(PORT1_DRV, PTDRVCNT1);
  putreg32(PORT1_DAT, P1DT);
  putreg32(PORT1_DIR, P1DRC);

  putreg32(PORT2_MUX, PMDCNT2);
  putreg32(PORT2_PUPD, PUDCNT2);
  putreg32(PORT2_DRV, PTDRVCNT2);
  putreg32(PORT2_DAT, P2DT);
  putreg32(PORT2_DIR, P2DRC);

  putreg32(PORT3_MUX, PMDCNT3);
  putreg32(PORT3_PUPD, PUDCNT3);
  putreg32(PORT3_DRV, PTDRVCNT3);
  putreg32(PORT3_DAT, P3DT);
  putreg32(PORT3_DIR, P3DRC);

  putreg32(PORT4_MUX, PMDCNT4);
  putreg32(PORT4_PUPD, PUDCNT4);
  putreg32(PORT4_DRV, PTDRVCNT4);
  putreg32(PORT4_DAT, P4DT);
  putreg32(PORT4_DIR, P4DRC);

  putreg32(PORT5_MUX, PMDCNT5);
  putreg32(PORT5_PUPD, PUDCNT5);
  putreg32(PORT5_DRV, PTDRVCNT5);
  putreg32(PORT5_DAT, P5DT);
  putreg32(PORT5_DIR, P5DRC);

  putreg32(PORT6_PUPD, PUDCNT6);
  putreg32(PORT6_DRV, PTDRVCNT6);
}
