/************************************************************************************
 * arch/arm/include/lpc11xx/chip.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_INCLUDE_LPC11XX_CHIP_H
#define __ARCH_ARM_INCLUDE_LPC11XX_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_LPC1115)
#  define LPC111x               1  /* LPC111x family */
#  define LPC11_FLASH_SIZE      (64*1024) /* 64Kb */
#  define LPC11_SRAM_SIZE       (8*1024)  /*  8Kb */
#  define LPC11_CPUSRAM_SIZE    (8*1024)
#  undef  LPC11_HAVE_BANK0         /* No AHB SRAM bank 0 */
#  undef  LPC11_HAVE_BANK1         /* No AHB SRAM bank 1 */
#  define LPC11_NETHCONTROLLERS 0  /* No Ethernet controller */
#  define LPC11_NUSBHOST        0  /* No USB host controller */
#  define LPC11_NUSBOTG         0  /* No USB OTG controller */
#  define LPC11_NUSBDEV         1  /* One USB device controller */
#  define LPC11_NCAN            1  /* One CAN controller */
#  define LPC11_NI2S            0  /* No I2S modules */
#  define LPC11_NDAC            0  /* No DAC module */
#else
#  error "Unsupported LPC11xx chip"
#endif

/* NVIC priority levels *************************************************************/
/* Each priority field holds a priority value, 0-31. The lower the value, the greater
 * the priority of the corresponding interrupt. The processor implements only
 * bits[7:6] of each field, bits[5:0] read as zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xc0 /* All bits[7:6] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x40 /* Two bits of interrupt priority used */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_LPC11XX_CHIP_H */
