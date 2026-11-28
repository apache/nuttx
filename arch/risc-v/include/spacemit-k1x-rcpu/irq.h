/****************************************************************************
 * arch/risc-v/include/spacemit-k1x-rcpu/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_SPACEMIT_K1X_RCPU_IRQ_H
#define __ARCH_RISCV_INCLUDE_SPACEMIT_K1X_RCPU_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The K1 RCPU is a Nuclei N308 core using a Nuclei ECLIC.  In ECLIC mode
 * the interrupt identity is delivered in mcause.EXCCODE[11:0] rather than
 * through a claim/complete register, so the NuttX IRQ number of an external
 * source is simply
 *
 *   IRQ = RISCV_IRQ_ASYNC + <ECLIC source id>
 *
 * The source ids below are the K1 RCPU subsystem interrupts.  They were
 * taken from the interrupt map published with the vendor RTOS firmware
 * (libcpu/risc-v/spacemit/n308/riscv-clic.h in spacemit-com/esos) and are
 * the ids the ECLIC expects in INTIE/INTIP/INTCTL.
 */

/* RISC-V architecturally defined sources */

#define SPACEMIT_K1X_RCPU_IRQ_SFT                  (RISCV_IRQ_ASYNC + 3)
#define SPACEMIT_K1X_RCPU_IRQ_TMR                  (RISCV_IRQ_ASYNC + 7)

/* RCPU subsystem sources.  Only the sources that the vendor firmware
 * documents are listed; the remaining ids up to SPACEMIT_K1X_RCPU_NIRQS are
 * reserved.
 */

#define SPACEMIT_K1X_RCPU_IRQ_DMAC                 (RISCV_IRQ_ASYNC + 19)
#define SPACEMIT_K1X_RCPU_IRQ_RSM                  (RISCV_IRQ_ASYNC + 23)
#define SPACEMIT_K1X_RCPU_IRQ_ADMA_CH0             (RISCV_IRQ_ASYNC + 24)
#define SPACEMIT_K1X_RCPU_IRQ_ADMA_CH1             (RISCV_IRQ_ASYNC + 25)
#define SPACEMIT_K1X_RCPU_IRQ_SSPA_DFE             (RISCV_IRQ_ASYNC + 26)
#define SPACEMIT_K1X_RCPU_IRQ_TIM0                 (RISCV_IRQ_ASYNC + 27)
#define SPACEMIT_K1X_RCPU_IRQ_TIM1                 (RISCV_IRQ_ASYNC + 28)
#define SPACEMIT_K1X_RCPU_IRQ_TIM2                 (RISCV_IRQ_ASYNC + 29)

/* IPC_AP2AUD is the mailbox that carries the rpmsg virtqueues between the
 * application processor and the RCPU.  It is the interrupt that the rptun
 * driver will use.
 */

#define SPACEMIT_K1X_RCPU_IRQ_IPC_AP2AUD           (RISCV_IRQ_ASYNC + 30)

#define SPACEMIT_K1X_RCPU_IRQ_SHUB_I2C0            (RISCV_IRQ_ASYNC + 32)
#define SPACEMIT_K1X_RCPU_IRQ_SHUB_SSP0            (RISCV_IRQ_ASYNC + 34)
#define SPACEMIT_K1X_RCPU_IRQ_UART1                (RISCV_IRQ_ASYNC + 36)
#define SPACEMIT_K1X_RCPU_IRQ_AUD_OCP              (RISCV_IRQ_ASYNC + 37)
#define SPACEMIT_K1X_RCPU_IRQ_HOOK_KEY             (RISCV_IRQ_ASYNC + 38)
#define SPACEMIT_K1X_RCPU_IRQ_AUD_PLUG             (RISCV_IRQ_ASYNC + 39)
#define SPACEMIT_K1X_RCPU_IRQ_AUD_AP_WAKEUP        (RISCV_IRQ_ASYNC + 41)
#define SPACEMIT_K1X_RCPU_IRQ_SENSOR_HUB           (RISCV_IRQ_ASYNC + 42)
#define SPACEMIT_K1X_RCPU_IRQ_GPIO110_PMIC         (RISCV_IRQ_ASYNC + 43)
#define SPACEMIT_K1X_RCPU_IRQ_GPIO115              (RISCV_IRQ_ASYNC + 44)
#define SPACEMIT_K1X_RCPU_IRQ_GPIO116              (RISCV_IRQ_ASYNC + 45)
#define SPACEMIT_K1X_RCPU_IRQ_GPIO117              (RISCV_IRQ_ASYNC + 46)
#define SPACEMIT_K1X_RCPU_IRQ_GPIO66               (RISCV_IRQ_ASYNC + 47)
#define SPACEMIT_K1X_RCPU_IRQ_GPIO67               (RISCV_IRQ_ASYNC + 48)
#define SPACEMIT_K1X_RCPU_IRQ_GPIO68               (RISCV_IRQ_ASYNC + 49)
#define SPACEMIT_K1X_RCPU_IRQ_GPIO69               (RISCV_IRQ_ASYNC + 50)
#define SPACEMIT_K1X_RCPU_IRQ_ADMA1_CH1            (RISCV_IRQ_ASYNC + 52)
#define SPACEMIT_K1X_RCPU_IRQ_ADMA0_CH1            (RISCV_IRQ_ASYNC + 53)
#define SPACEMIT_K1X_RCPU_IRQ_SSPA1                (RISCV_IRQ_ASYNC + 54)
#define SPACEMIT_K1X_RCPU_IRQ_HDMI_ADMA            (RISCV_IRQ_ASYNC + 55)
#define SPACEMIT_K1X_RCPU_IRQ_ADMA0_CH0            (RISCV_IRQ_ASYNC + 58)
#define SPACEMIT_K1X_RCPU_IRQ_SSPA0                (RISCV_IRQ_ASYNC + 59)
#define SPACEMIT_K1X_RCPU_IRQ_CAN0_INT0            (RISCV_IRQ_ASYNC + 60)
#define SPACEMIT_K1X_RCPU_IRQ_CAN0_INT1            (RISCV_IRQ_ASYNC + 61)
#define SPACEMIT_K1X_RCPU_IRQ_IRC                  (RISCV_IRQ_ASYNC + 62)
#define SPACEMIT_K1X_RCPU_IRQ_UART0                (RISCV_IRQ_ASYNC + 63)
#define SPACEMIT_K1X_RCPU_IRQ_SHUB_EDGE_DET        (RISCV_IRQ_ASYNC + 64)

/* SPACEMIT_K1X_RCPU_NIRQS is the number of ECLIC interrupt sources.
 *
 * The ECLIC itself reports this: INFO.NUMINT holds the id of the highest
 * source, so the number of sources is NUMINT + 1.  A stock MUSE-Pi Pro
 * reports INFO = 0x00602053, i.e. NUMINT = 83, hence 84 sources.
 *
 * The value here is the one observed on K1.  spacemit_k1x_rcpu_irq.c
 * validates it against the hardware at start up and reports a mismatch, so
 * a part with a different source count is detected rather than silently
 * mis-handled.
 */

#define SPACEMIT_K1X_RCPU_ECLIC_NUMINT_OBSERVED    83
#define SPACEMIT_K1X_RCPU_NIRQS                    (SPACEMIT_K1X_RCPU_ECLIC_NUMINT_OBSERVED + 1)

#define NR_IRQS                      (RISCV_IRQ_ASYNC + SPACEMIT_K1X_RCPU_NIRQS)

#endif /* __ARCH_RISCV_INCLUDE_SPACEMIT_K1X_RCPU_IRQ_H */
