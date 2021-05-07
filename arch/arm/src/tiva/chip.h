/****************************************************************************
 * arch/arm/src/tiva/chip.h
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

#ifndef __ARCH_ARM_SRC_TIVA_CHIP_H
#define __ARCH_ARM_SRC_TIVA_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/tiva/chip.h>
#include <arch/tiva/irq.h>

/* Then get all of the register definitions */

#include "hardware/tiva_memorymap.h"  /* Memory map */
#include "hardware/tiva_sysctrl.h"    /* System control module */
#include "hardware/tiva_gpio.h"       /* GPIO modules */
#include "hardware/tiva_uart.h"       /* UART modules */
#include "hardware/tiva_i2c.h"        /* I2C modules */
#include "hardware/tiva_ssi.h"        /* SSI modules */
#include "hardware/tiva_ethernet.h"   /* Ethernet MAC and PHY */
#include "hardware/tiva_flash.h"      /* FLASH */
#include "hardware/tiva_eeprom.h"     /* EEPROM */
#include "hardware/tiva_timer.h"      /* Timer */
#include "hardware/tiva_adc.h"        /* ADC */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Provide the required number of peripheral interrupt vector definitions as
 * well. The definition TIVA_IRQ_NEXTINT simply comes from the chip-specific
 * IRQ header file included by arch/tiva/irq.h.
 */

#define ARMV7M_PERIPHERAL_INTERRUPTS  TIVA_IRQ_NEXTINT

#endif /* __ARCH_ARM_SRC_TIVA_CHIP_H */
