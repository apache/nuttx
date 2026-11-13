/****************************************************************************
 * arch/tricore/src/aurix/tricore_asclin.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_TRICORE_SRC_AURIX_TRICORE_ASCLIN_H
#define __ARCH_TRICORE_SRC_AURIX_TRICORE_ASCLIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_TC3XX)
#  include "hardware/tc3xx_asclin.h"
#elif defined(CONFIG_ARCH_CHIP_TC4XX)
#  include "hardware/tc4xx_asclin.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ASCLIN register bits, the same on TC3xx and TC4xx */

#define ASCLIN_CLC_DISR               (1 << 0)    /* Disable request */

#define ASCLIN_IOCR_ALTI_MASK         (0x7 << 0)  /* RX input select */
#define ASCLIN_IOCR_LB                (1 << 28)   /* Loop back */

#define ASCLIN_FIFOCON_FLUSH          (1 << 0)    /* TX and RX FIFOCON */
#define ASCLIN_FIFOCON_EN             (1 << 1)    /* TX ENO, RX ENI */
#define ASCLIN_FIFOCON_FM_MASK        (0x3 << 4)  /* Interrupt mode */
#define ASCLIN_FIFOCON_W_SHIFT        6           /* TX INW, RX OUTW */
#define ASCLIN_FIFOCON_W_MASK         (0x3 << ASCLIN_FIFOCON_W_SHIFT)
#define ASCLIN_FIFOCON_INTLEVEL_MASK  (0xf << 8)
#define ASCLIN_FIFOCON_FILL_SHIFT     16
#define ASCLIN_FIFOCON_FILL_MASK      (0x1f << ASCLIN_FIFOCON_FILL_SHIFT)

#define ASCLIN_BITCON_PRESCALER_MASK  (0xfff << 0)
#define ASCLIN_BITCON_OVS_SHIFT       16          /* Oversampling - 1 */
#define ASCLIN_BITCON_OVS_MASK        (0xf << ASCLIN_BITCON_OVS_SHIFT)
#define ASCLIN_BITCON_SP_SHIFT        24          /* Sample point */
#define ASCLIN_BITCON_SP_MASK         (0xf << ASCLIN_BITCON_SP_SHIFT)
#define ASCLIN_BITCON_SM              (1 << 31)   /* Three samples */

#define ASCLIN_FRAMECON_IDLE_MASK     (0x7 << 6)
#define ASCLIN_FRAMECON_STOP_SHIFT    9
#define ASCLIN_FRAMECON_STOP_MASK     (0x7 << ASCLIN_FRAMECON_STOP_SHIFT)
#define ASCLIN_FRAMECON_MODE_SHIFT    16
#define ASCLIN_FRAMECON_MODE_MASK     (0x3 << ASCLIN_FRAMECON_MODE_SHIFT)
#define ASCLIN_FRAMECON_MODE_INIT     (0 << ASCLIN_FRAMECON_MODE_SHIFT)
#define ASCLIN_FRAMECON_MODE_ASC      (1 << ASCLIN_FRAMECON_MODE_SHIFT)
#define ASCLIN_FRAMECON_MSB           (1 << 28)
#define ASCLIN_FRAMECON_PEN           (1 << 30)
#define ASCLIN_FRAMECON_ODD           (1 << 31)

#define ASCLIN_DATCON_DATLEN_MASK     (0xf << 0)  /* Data length - 1 */

#define ASCLIN_BRG_DENOMINATOR_MASK   (0xfff << 0)
#define ASCLIN_BRG_NUMERATOR_SHIFT    16
#define ASCLIN_BRG_NUMERATOR_MASK     (0xfff << ASCLIN_BRG_NUMERATOR_SHIFT)

#define ASCLIN_CSR_CLKSEL_MASK        (0x1f << 0)
#define ASCLIN_CSR_CLKSEL_NONE        0
#define ASCLIN_CSR_CLKSEL_FAST        2           /* fASCLINF */
#define ASCLIN_CSR_CON                (1 << 31)   /* Clock on */

#define ASCLIN_FLAGS_PE               (1 << 16)   /* Parity error */
#define ASCLIN_FLAGS_FE               (1 << 18)   /* Frame error */
#define ASCLIN_FLAGS_RFO              (1 << 26)   /* RX FIFO overflow */
#define ASCLIN_FLAGS_RFU              (1 << 27)   /* RX FIFO underflow */
#define ASCLIN_FLAGS_RFL              (1 << 28)   /* RX FIFO level */
#define ASCLIN_FLAGS_TFO              (1 << 30)   /* TX FIFO overflow */

#endif /* __ARCH_TRICORE_SRC_AURIX_TRICORE_ASCLIN_H */
