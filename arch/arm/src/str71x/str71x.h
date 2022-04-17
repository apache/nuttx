/****************************************************************************
 * arch/arm/src/str71x/str71x.h
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_H
#define __ARCH_ARM_SRC_STR71X_STR71X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Calculate the values of PCLK1 and PCLK2 from settings in board.h.
 *
 * Example:
 *  STR71X_RCCU_MAIN_OSC = 4MHz (not divided by 2)
 *  STR71X_CLK2 = 4MHz
 *  STR71X_PLL1OUT = 16 * STR71X_CLK2 / 2 = 32MHz
 *  CLK3 = 32MHz
 *  RCLK = 32MHz
 *  PCLK1 = 32MHz / 1 = 32MHz
 */

/* PLL1OUT derives from Main OSC->CLK2 */

#ifdef STR71X_PLL1IN_DIV2                              /* CLK2 is input to PLL1 */
#  define STR71X_CLK2  (STR71X_RCCU_MAIN_OSC/2)        /* CLK2 is OSC/2 */
#else
#  define STR71X_CLK2  STR71X_RCCU_MAIN_OSC            /* CLK2 is OSC */
#endif

#define STR71X_PLL1OUT ((STR71X_PLL1OUT_MUL * STR71X_CLK2) / STR71X_PLL1OUT_DIV)

/* PLL2 OUT derives from HCLK */

#define STR71X_PLL2OUT ((STR71X_PLL2OUT_MUL * STR71X_HCLK) / STR71X_PLL2OUT_DIV)

/* Peripheral clocks derive from PLL1OUT->CLK3->RCLK->PCLK1/2 */

#define STR71X_CLK3    STR71X_PLL1OUT                  /* CLK3 hard coded to be PLL1OUT */
#define STR71X_RCLK    STR71X_CLK3                     /* RCLK hard coded to be CLK3 */
#define STR71X_PCLK1   (STR71X_RCLK / STR71X_APB1_DIV) /* PCLK1 derives from RCLK */
#define STR71X_PCLK2   (STR71X_RCLK / STR71X_APB2_DIV) /* PCLK2 derives from RCLK */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: str7x_xtiinitialize
 *
 * Description:
 *   Configure XTI for operation.
 *   Note that the lines are not used as wake-up sources in this
 *   implementation.
 *   Some extensions would be required for that capability.
 *
 ****************************************************************************/

#ifdef CONFIG_STR71X_XTI
int str71x_xtiinitialize(void);
#else
#  define str71x_xtiinitialize()
#endif /* CONFIG_STR71X_XTI */

/****************************************************************************
 * Name: str7x_xticonfig
 *
 * Description:
 *   Configure an external line to provide interrupts.
 *   Interrupt is configured, but disabled on return.
 *
 ****************************************************************************/

#ifdef CONFIG_STR71X_XTI
int str71x_xticonfig(int irq, bool rising);
#else
#  define str71x_xticonfig(irq,rising)
#endif /* CONFIG_STR71X_XTI */

/****************************************************************************
 * Name: str71x_enable_xtiirq
 *
 * Description:
 *   Enable an external interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_STR71X_XTI
void str71x_enable_xtiirq(int irq);
#else
#  define str71x_enable_xtiirq(irq)
#endif /* CONFIG_STR71X_XTI */

/****************************************************************************
 * Name: str71x_disable_xtiirq
 *
 * Description:
 *   Disable an external interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_STR71X_XTI
void str71x_disable_xtiirq(int irq);
#else
#  define str71x_disable_xtiirq(irq)
#endif /* CONFIG_STR71X_XTI */

struct spi_dev_s; /* Forward reference */

/****************************************************************************
 * Name: str71_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port.  This function could get called
 *   multiple times for each STR7 devices that needs an SPI reference.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *str71_spibus_initialize(int port);

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_H */
