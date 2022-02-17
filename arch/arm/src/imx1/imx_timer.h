/****************************************************************************
 * arch/arm/src/imx1/imx_timer.h
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

#ifndef __ARCH_ARM_SRC_IMX1_IMX_TIMER_H
#define __ARCH_ARM_SRC_IMX1_IMX_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer Register Offsets ***************************************************/

#define TIMER_TCTL_OFFSET           0x0000 /* Timer control register */
#define TIMER_TPRER_OFFSET          0x0004 /* Timer prescaler register */
#define TIMER_TCMP_OFFSET           0x0008 /* Timer compare register */
#define TIMER_TCR_OFFSET            0x000c /* Timer capture register */
#define TIMER_TCN_OFFSET            0x0010 /* Timer counter register */
#define TIMER_TSTAT_OFFSET          0x0014 /* Timer status register */

/* Timer Register Addresses *************************************************/

#define IMX_TIMER1_TCTL             (IMX_TIMER1_VBASE + TIMER_TCTL_OFFSET)
#define IMX_TIMER1_TPRER            (IMX_TIMER1_VBASE + TIMER_TPRER_OFFSET)
#define IMX_TIMER1_TCMP             (IMX_TIMER1_VBASE + TIMER_TCMP_OFFSET)
#define IMX_TIMER1_TCR              (IMX_TIMER1_VBASE + TIMER_TCR_OFFSET)
#define IMX_TIMER1_TCN              (IMX_TIMER1_VBASE + TIMER_TCN_OFFSET)
#define IMX_TIMER1_TSTAT            (IMX_TIMER1_VBASE + TIMER_TSTAT_OFFSET)

#define IMX_TIMER2_TCTL             (IMX_TIMER2_VBASE + TIMER_TCTL_OFFSET)
#define IMX_TIMER2_TPRER            (IMX_TIMER2_VBASE + TIMER_TPRER_OFFSET)
#define IMX_TIMER2_TCMP             (IMX_TIMER2_VBASE + TIMER_TCMP_OFFSET)
#define IMX_TIMER2_TCR              (IMX_TIMER2_VBASE + TIMER_TCR_OFFSET)
#define IMX_TIMER2_TCN              (IMX_TIMER2_VBASE + TIMER_TCN_OFFSET)
#define IMX_TIMER2_TSTAT            (IMX_TIMER2_VBASE + TIMER_TSTAT_OFFSET)

/* Timer Register Bit Definitions *******************************************/

/* Timer Control Register */

#define TIMER_TCTL_TEN              (1 << 0)  /* Bit 0: Timer Enable */
#define TIMER_TCTL_CLKSOURCE_SHIFT  1         /* Bit 1-4: Clock Source */
#define TIMER_TCTL_CLKSOURCE_MASK   (0x07 << TIMER_TCTL_CLKSOURCE_SHIFT)
#define   TCTL_CLKSOURCE_STOPCOUNT  (0x00 << TIMER_TCTL_CLKSOURCE_SHIFT)
#define   TCTL_CLKSOURCE_PERCLK1    (0x01 << TIMER_TCTL_CLKSOURCE_SHIFT)
#define   TCTL_CLKSOURCE_PERCLK1D16 (0x02 << TIMER_TCTL_CLKSOURCE_SHIFT)
#define   TCTL_CLKSOURCE_TIN        (0x03 << TIMER_TCTL_CLKSOURCE_SHIFT)
#define   TCTL_CLKSOURCE_32KHX      (0x04 << TIMER_TCTL_CLKSOURCE_SHIFT)
#define TIMER_TCTL_IRQEN            (1 << 5)  /* Bit 5: Interrupt Request Enable */
#define TIMER_TCTL_OM               (1 << 6)  /* Bit 6: Output Mode */
#define TIMER_TCTL_CAP              (1 << 7)  /* Bit 7: Capture Edge */
#define TIMER_TCTL_FRR              (1 << 8)  /* Bit 8: Free-Run/Reset */
#define TIMER_TCTL_SWR              (1 << 15) /* Bit 15: Software Reset */

/* Timer Prescaler Register */

#define TIMER_TPRER_PRESCALER_SHIFT 0         /* Bits 0-7: Prescaler */
#define TIMER_TPRER_PRESCALER_MASK  (0xff << TIMER_TPRER_PRESCALER_SHIFT)

/* Timer Status Register  */

#define TIMER_TSTAT_COMP            (1 << 0)  /* Bit 0: Compare Event */
#define TIMER_TSTAT_CAPT            (1 << 1)  /* Bit 1: Capture Event */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_IMX1_IMX_TIMER_H */
