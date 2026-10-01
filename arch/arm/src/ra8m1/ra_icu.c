/****************************************************************************
 * arch/arm/src/ra8m1/ra_icu.c
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

#include <assert.h>
#include <stdint.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/ra8m1_icu.h"
#include "hardware/ra8m1_elc.h"
#include "ra_icu.h"

/* More events than IELSR slots is a configuration error */

static_assert(RA_IRQ_EVT_END <= NR_IRQS,
              "Too many ICU events enabled for the IELSR slots");

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_attach_icu
 *
 * Description:
 *   Every ICU IELSR slot is runtime-configurable -- any peripheral event
 *   can be routed to any slot.  This wires each enabled SCI_B channel's
 *   RXI/TXI/TEI/ERI events into the slots that irq.h assigned them
 *   (SCIn_RXI/TXI/TEI/ERI), so their vector numbers actually fire.
 *
 ****************************************************************************/

void ra_attach_icu(void)
{
#ifdef CONFIG_RA_SCI0_UART
  putreg32(EVENT_SCI0_RXI, R_ICU_IELSR(SCI0_RXI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI0_TXI, R_ICU_IELSR(SCI0_TXI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI0_TEI, R_ICU_IELSR(SCI0_TEI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI0_ERI, R_ICU_IELSR(SCI0_ERI - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_SCI1_UART
  putreg32(EVENT_SCI1_RXI, R_ICU_IELSR(SCI1_RXI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI1_TXI, R_ICU_IELSR(SCI1_TXI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI1_TEI, R_ICU_IELSR(SCI1_TEI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI1_ERI, R_ICU_IELSR(SCI1_ERI - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_SCI2_UART
  putreg32(EVENT_SCI2_RXI, R_ICU_IELSR(SCI2_RXI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI2_TXI, R_ICU_IELSR(SCI2_TXI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI2_TEI, R_ICU_IELSR(SCI2_TEI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI2_ERI, R_ICU_IELSR(SCI2_ERI - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_SCI3_UART
  putreg32(EVENT_SCI3_RXI, R_ICU_IELSR(SCI3_RXI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI3_TXI, R_ICU_IELSR(SCI3_TXI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI3_TEI, R_ICU_IELSR(SCI3_TEI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI3_ERI, R_ICU_IELSR(SCI3_ERI - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_SCI4_UART
  putreg32(EVENT_SCI4_RXI, R_ICU_IELSR(SCI4_RXI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI4_TXI, R_ICU_IELSR(SCI4_TXI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI4_TEI, R_ICU_IELSR(SCI4_TEI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI4_ERI, R_ICU_IELSR(SCI4_ERI - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_SCI9_UART
  putreg32(EVENT_SCI9_RXI, R_ICU_IELSR(SCI9_RXI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI9_TXI, R_ICU_IELSR(SCI9_TXI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI9_TEI, R_ICU_IELSR(SCI9_TEI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI9_ERI, R_ICU_IELSR(SCI9_ERI - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_GPT0_GPT
  putreg32(EVENT_GPT0_COUNTER_OVERFLOW,
           R_ICU_IELSR(GPT0_COUNTER_OVERFLOW - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_GPT1_GPT
  putreg32(EVENT_GPT1_COUNTER_OVERFLOW,
           R_ICU_IELSR(GPT1_COUNTER_OVERFLOW - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_GPT2_GPT
  putreg32(EVENT_GPT2_COUNTER_OVERFLOW,
           R_ICU_IELSR(GPT2_COUNTER_OVERFLOW - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_GPT3_GPT
  putreg32(EVENT_GPT3_COUNTER_OVERFLOW,
           R_ICU_IELSR(GPT3_COUNTER_OVERFLOW - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_GPT4_GPT
  putreg32(EVENT_GPT4_COUNTER_OVERFLOW,
           R_ICU_IELSR(GPT4_COUNTER_OVERFLOW - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_GPT5_GPT
  putreg32(EVENT_GPT5_COUNTER_OVERFLOW,
           R_ICU_IELSR(GPT5_COUNTER_OVERFLOW - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_GPT6_GPT
  putreg32(EVENT_GPT6_COUNTER_OVERFLOW,
           R_ICU_IELSR(GPT6_COUNTER_OVERFLOW - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_GPT7_GPT
  putreg32(EVENT_GPT7_COUNTER_OVERFLOW,
           R_ICU_IELSR(GPT7_COUNTER_OVERFLOW - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_GPT8_GPT
  putreg32(EVENT_GPT8_COUNTER_OVERFLOW,
           R_ICU_IELSR(GPT8_COUNTER_OVERFLOW - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_GPT9_GPT
  putreg32(EVENT_GPT9_COUNTER_OVERFLOW,
           R_ICU_IELSR(GPT9_COUNTER_OVERFLOW - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_GPT10_GPT
  putreg32(EVENT_GPT10_COUNTER_OVERFLOW,
           R_ICU_IELSR(GPT10_COUNTER_OVERFLOW - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_GPT11_GPT
  putreg32(EVENT_GPT11_COUNTER_OVERFLOW,
           R_ICU_IELSR(GPT11_COUNTER_OVERFLOW - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_GPT12_GPT
  putreg32(EVENT_GPT12_COUNTER_OVERFLOW,
           R_ICU_IELSR(GPT12_COUNTER_OVERFLOW - RA_IRQ_FIRST));
#endif
#ifdef CONFIG_RA_GPT13_GPT
  putreg32(EVENT_GPT13_COUNTER_OVERFLOW,
           R_ICU_IELSR(GPT13_COUNTER_OVERFLOW - RA_IRQ_FIRST));
#endif
}

/****************************************************************************
 * Name: ra_clear_ir
 *
 * Description:
 *   Clear the latched interrupt status flag (IELSR.IR) for the ICU slot
 *   backing the given IRQ number.
 *
 ****************************************************************************/

void ra_clear_ir(int irq)
{
  uint32_t slot = irq - RA_IRQ_FIRST;

  modifyreg32(R_ICU_IELSR(slot), R_ICU_IELSR_IR, 0);
  getreg32(R_ICU_IELSR(slot));
}
