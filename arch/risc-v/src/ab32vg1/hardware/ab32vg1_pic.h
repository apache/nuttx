/****************************************************************************
 * arch/risc-v/src/ab32vg1/hardware/ab32vg1_pic.h
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

#ifndef __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_PIC_H
#define __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_PIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/ab32vg1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt controller and trap registers.  These are not described in the
 * register manual; the behaviour below was established by the probes in
 * the port's bring-up journal and matches the RT-Thread BSP (ab32vg1.h).
 */

#define AB32VG1_CPU_STATUS      AB32VG1_SFR(5, 0x00) /* 0xc0000002 after an exception */
#define AB32VG1_PICCONCLR       AB32VG1_SFR(5, 0x0c) /* Write 1 to clear PICCON bits */
#define AB32VG1_PICCONSET       AB32VG1_SFR(5, 0x0d) /* Write 1 to set PICCON bits */
#define AB32VG1_PICENCLR        AB32VG1_SFR(5, 0x0e)
#define AB32VG1_PICENSET        AB32VG1_SFR(5, 0x0f)
#define AB32VG1_PICCON          AB32VG1_SFR(5, 0x10) /* Control */
#define AB32VG1_PICEN           AB32VG1_SFR(5, 0x11) /* Enable, 1 bit/source */
#define AB32VG1_PICPR           AB32VG1_SFR(5, 0x12) /* Priority: 1 = high */
#define AB32VG1_PICADR          AB32VG1_SFR(5, 0x13) /* Vector table base */
#define AB32VG1_PICPND          AB32VG1_SFR(5, 0x14) /* Pending */
#define AB32VG1_EPICCON         AB32VG1_SFR(5, 0x1e) /* PICCON.IE saved on trap */
#define AB32VG1_EPC             AB32VG1_SFR(5, 0x1f) /* Trap return address */

/* PICCON */

#define PICCON_IE               (1 << 0)   /* Low priority interrupt enable */
#define PICCON_EN               (1 << 1)   /* Interrupt unit enable */
#define PICCON_INIT             0x00010002 /* As set by the RT-Thread BSP */

/* Vector offsets from PICADR */

#define AB32VG1_VEC_EXCEPTION   0x10       /* Exceptions, high prio IRQs */
#define AB32VG1_VEC_IRQ         0x40       /* Low priority IRQs */
#define AB32VG1_VEC_CACHE       0x80       /* XIP cache miss */

/* Software interrupt: sets PICPND bit 2 */

#define AB32VG1_SWINT_INSN      0xb0030057

#endif /* __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_PIC_H */
