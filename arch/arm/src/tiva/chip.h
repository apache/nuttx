/************************************************************************************
 * arch/arm/src/tiva/chip.h
 *
 *   Copyright (C) 2009-2010, 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_CHIP_H
#define __ARCH_ARM_SRC_TIVA_CHIP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/tiva/chip.h>
#include <arch/tiva/irq.h>

/* Then get all of the register definitions */

#include "chip/tiva_memorymap.h"  /* Memory map */
#include "chip/tiva_syscontrol.h" /* System control module */
#include "chip/tiva_gpio.h"       /* GPIO modules */
#include "chip/tiva_uart.h"       /* UART modules */
#include "chip/tiva_i2c.h"        /* I2C modules */
#include "chip/tiva_ssi.h"        /* SSI modules */
#include "chip/tiva_ethernet.h"   /* Ethernet MAC and PHY */
#include "chip/tiva_flash.h"      /* FLASH */
#include "chip/tiva_eeprom.h"     /* EEPROM */
#include "chip/tiva_timer.h"      /* Timer */
#include "chip/tiva_adc.h"        /* ADC */

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Provide the required number of peripheral interrupt vector definitions as well.
 * The definition TIVA_IRQ_NEXTINT simply comes from the chip-specific IRQ header
 * file included by arch/tiva/irq.h.
 */

#define ARMV7M_PERIPHERAL_INTERRUPTS  TIVA_IRQ_NEXTINT

#endif /* __ARCH_ARM_SRC_TIVA_CHIP_H */
