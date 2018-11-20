/************************************************************************************
 * arch/arm/src/max326xx/hardware/max326_fcr.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_FCR_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_FCR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define MAX326_FCR_REG0_OFFSET      0x0000    /* Function Control Register 0 */

/* Register Addresses ***************************************************************/

#define MAX326_FCR_REG0             (MAX326_FCR_BASE + MAX326_FCR_REG0_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Function Control Register 0 */

#define FCR_REG0_I2C0_SDA_FILTER    (1 << 20) /* Bit 20: I2C0 SDA Filter Enable */
#define FCR_REG0_I2C0_SCL_FILTER    (1 << 21) /* Bit 21: I2C0 SCL Filter Enable */
#define FCR_REG0_I2C1_SDA_FILTER    (1 << 22) /* Bit 22: I2C1 SDA Filter Enable */
#define FCR_REG0_I2C1_SCL_FILTER    (1 << 23) /* Bit 23: I2C1 SCL Filter Enable */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_FCR_H */
