/****************************************************************************
 * boards/risc-v/rv32m1/rv32m1-vega/src/rv32m1-vega.h
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

#ifndef __BOARDS_RISCV_RV32M1_RV32M1_VEGA_SRC_RV32M1_VEGA_H
#define __BOARDS_RISCV_RV32M1_RV32M1_VEGA_SRC_RV32M1_VEGA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BUTTONs */

#define BOARD_NBUTTON  4

#define BUTTON_SW2     (GPIO_INPUT|GPIO_INT_EDGE|GPIO_FLOAT |\
                        GPIO_PORTA|GPIO_PIN0)

#define BUTTON_SW3     (GPIO_INPUT|GPIO_INT_EDGE|GPIO_PULLUP|\
                        GPIO_PORTE|GPIO_PIN8)

#define BUTTON_SW4     (GPIO_INPUT|GPIO_INT_EDGE|GPIO_PULLUP|\
                        GPIO_PORTE|GPIO_PIN9)

#define BUTTON_SW5     (GPIO_INPUT|GPIO_INT_EDGE|GPIO_PULLUP|\
                        GPIO_PORTE|GPIO_PIN12)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int rv32m1_bringup(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISCV_RV32M1_RV32M1_VEGA_SRC_RV32M1_VEGA_H */
