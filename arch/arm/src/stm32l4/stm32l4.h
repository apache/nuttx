/************************************************************************************
 * arch/arm/src/stm32l4/stm32l4.h
 *
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Authors: Uros Platise <sebastien@lorquet.fr>
 *            Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "up_internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Peripherals **********************************************************************/

#include "chip.h"
#include "stm32l4_adc.h"
#include "stm32l4_can.h"
#include "stm32l4_comp.h"
#include "stm32l4_dac.h"
#include "stm32l4_dbgmcu.h"
#include "stm32l4_dma.h"
#include "stm32l4_exti.h"
#include "stm32l4_flash.h"
#include "stm32l4_fsmc.h"
#include "stm32l4_gpio.h"
#include "stm32l4_i2c.h"
#include "stm32l4_lcd.h"
#include "stm32l4_pwr.h"
#include "stm32l4_rcc.h"
#include "stm32l4_rtc.h"
#include "stm32l4_sdmmc.h"
#include "stm32l4_spi.h"
#include "stm32l4_tim.h"
#include "stm32l4_uart.h"
#include "stm32l4_usbdev.h"
#include "stm32l4_wdg.h"
#include "stm32l4_lowputc.h"

#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_H */
