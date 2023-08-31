/****************************************************************************
 * arch/arm/src/mx8mp/hardware/mx8mp_memorymap.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * May include some logic from sample code provided by Infineon:
 *
 *   Copyright (C) 2011-2015 Infineon Technologies AG. All rights reserved.
 *
 * Infineon Technologies AG (Infineon) is supplying this software for use
 * with Infineon's microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such microcontrollers.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS
 * SOFTWARE. INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
 * INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ****************************************************************************/

/* Reference:
 *   "i.MX 8M Plus Applications Processor Reference Manual",
 *   Document Number: IMX8MPRM Rev. 1, 06/2021. NXP
 */

#ifndef __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_MEMORYMAP_H
#define __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Peripheral Memory Map ****************************************************/

#define MX8M_IOMUXC                  0x30330000u
#define MX8M_CCM_ANALOG              0x30360000u
#define MX8M_CCM                     0x30380000u
#define MX8M_SRC                     0x30390000u
#define MX8M_GPC                     0x303A0000u
#define MX8M_RDC                     0x303D0000u
#define MX8M_CSU                     0x303E0000u

#define MX8M_AUDIOMIX                0x30E20000u

#define MX8M_GPT1                    0x302D0000u
#define MX8M_GPT2                    0x302E0000u
#define MX8M_GPT3                    0x302F0000u
#define MX8M_GPT4                    0x30700000u
#define MX8M_GPT5                    0x306F0000u
#define MX8M_GPT6                    0x306E0000u

#define MX8M_PWM1                    0x30660000u
#define MX8M_PWM2                    0x30670000u
#define MX8M_PWM3                    0x30680000u
#define MX8M_PWM4                    0x30690000u

#define MX8M_I2C1                    0x30A20000u
#define MX8M_I2C2                    0x30A30000u
#define MX8M_I2C3                    0x30A40000u
#define MX8M_I2C4                    0x30A50000u
#define MX8M_I2C5                    0x30AD0000u
#define MX8M_I2C6                    0x30AE0000u

#define MX8M_UART1                   0x30860000u
#define MX8M_UART2                   0x30890000u
#define MX8M_UART3                   0x30880000u
#define MX8M_UART4                   0x30A60000u

#define MX8M_ECSPI1                  0x30820000u
#define MX8M_ECSPI2                  0x30830000u
#define MX8M_ECSPI3                  0x30840000u

#define MX8M_GPIO                    0x30200000u
#define MX8M_GPIO1                   (MX8M_GPIO + 0x10000 * 0)
#define MX8M_GPIO2                   (MX8M_GPIO + 0x10000 * 1)
#define MX8M_GPIO3                   (MX8M_GPIO + 0x10000 * 2)
#define MX8M_GPIO4                   (MX8M_GPIO + 0x10000 * 3)
#define MX8M_GPIO5                   (MX8M_GPIO + 0x10000 * 4)

#define MX8M_DDR                     0x40000000u
#define MX8M_OCRAM                   0x00900000u

#endif /* __ARCH_ARM_SRC_MX8MP_HARDWARE_MX8MP_MEMORYMAP_H */
