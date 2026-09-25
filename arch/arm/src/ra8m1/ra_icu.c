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

#include <stdint.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/ra8m1_icu.h"
#include "hardware/ra8m1_elc.h"
#include "ra_icu.h"

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
