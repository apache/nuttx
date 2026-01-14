/****************************************************************************
 * arch/arm/src/ra4/ra_icu.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "hardware/ra_sci.h"
#include "hardware/ra_mstp.h"
#include "hardware/ra_system.h"
#include "ra_icu.h"
#include "ra_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
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
#ifdef CONFIG_RA_SCI9_UART
  putreg32(EVENT_SCI9_RXI, R_ICU_IELSR(SCI9_RXI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI9_TXI, R_ICU_IELSR(SCI9_TXI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI9_TEI, R_ICU_IELSR(SCI9_TEI - RA_IRQ_FIRST));
  putreg32(EVENT_SCI9_ERI, R_ICU_IELSR(SCI9_ERI - RA_IRQ_FIRST));
#endif
}

void ra_clear_ir(int irq)
{
  uint32_t regaddr;
  regaddr = irq - RA_IRQ_FIRST;
  modifyreg32(R_ICU_IELSR(regaddr), R_ICU_IELSR_IR, 0);
  getreg32(R_ICU_IELSR(regaddr));
}
