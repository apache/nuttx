/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mz_pps.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_PPS_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_PPS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* PPS Register Offsets/Addresses *******************************************/

/* Depends on the peripherals supported by the particular device */

#if defined(CONFIG_ARCH_CHIP_PIC32MZEC)
#  include "hardware/pic32mzec_pps.h"
#elif defined(CONFIG_ARCH_CHIP_PIC32MZEF)
#  include "hardware/pic32mzef_pps.h"
#else
#  error Unknown PIC32MZ family
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PPS Register Bit Field Definitions ***************************************/

/* All registers contain a single 4 bit field (bits 0-3) holding the
 * peripheral pin selection.
 */

#define PPS_MASK 0x0000000f

/* Pin Selection Helper Macros **********************************************/

/* The encoding of the input pin selection is simple.
 * Since we know the devices, we also can infer the register address so we
 * need only the value for the register which is exactly what is provided
 * by macro definitions.
 *
 * The encoding of the output pin selection is a little more complex.
 * Knowing the device does not provide sufficient information.  The output
 * pin definitions include both the register value and the register address
 * and the following helper macros can be used extract one or the other.
 *
 * NOTE: These odd macro forms are used to work around a pre-processor issue.
 * The argument to PPS_OUTPUT_REGADDR is defined to have the form nn,xxxx
 * but the preprocessor would claim that only one parameter is passed.
 *  The following version takes only one parameter and keeps the
 * pre-processor happy.
 */

#define __PPS_OUTPUT_REGADDR(a,b) ((uintptr_t)(b))
#define PPS_OUTPUT_REGADDR(a)  __PPS_OUTPUT_REGADDR(a)

#define __PPS_OUTPUT_REGVAL(a,b)  ((uint32_t)(a))
#define PPS_OUTPUT_REGVAL(a)  __PPS_OUTPUT_REGVAL(a)

/* The following macro converts from a peripheral output pin mapping to the
 * corresponding GPIO port and pin. This allows drivers to do things like
 * temporarily change a pin's configuration from peripheral to GPIO to
 * override some hardware behavior. Having this macro available to driver
 * code relieves the board's include/board.h from redundantly defining both
 * the PPS mapping and the GPIO port/pin information.
 *
 * NOTE: This is written in the same odd macro forms as above for the same
 * reason.
 */
#define __PPS_OUTPUT_REGADDR_TO_GPIO(a,b) ((uint32_t)( \
          ((b) == PIC32MZ_RPA14R) ? (GPIO_PORTA | GPIO_PIN14) : \
          ((b) == PIC32MZ_RPA15R) ? (GPIO_PORTA | GPIO_PIN15) : \
          ((b) == PIC32MZ_RPB0R)  ? (GPIO_PORTB | GPIO_PIN0)  : \
          ((b) == PIC32MZ_RPB10R) ? (GPIO_PORTB | GPIO_PIN10) : \
          ((b) == PIC32MZ_RPB14R) ? (GPIO_PORTB | GPIO_PIN14) : \
          ((b) == PIC32MZ_RPB15R) ? (GPIO_PORTB | GPIO_PIN15) : \
          ((b) == PIC32MZ_RPB1R)  ? (GPIO_PORTB | GPIO_PIN1)  : \
          ((b) == PIC32MZ_RPB2R)  ? (GPIO_PORTB | GPIO_PIN2)  : \
          ((b) == PIC32MZ_RPB3R)  ? (GPIO_PORTB | GPIO_PIN3)  : \
          ((b) == PIC32MZ_RPB5R)  ? (GPIO_PORTB | GPIO_PIN5)  : \
          ((b) == PIC32MZ_RPB6R)  ? (GPIO_PORTB | GPIO_PIN6)  : \
          ((b) == PIC32MZ_RPB7R)  ? (GPIO_PORTB | GPIO_PIN7)  : \
          ((b) == PIC32MZ_RPB8R)  ? (GPIO_PORTB | GPIO_PIN8)  : \
          ((b) == PIC32MZ_RPB9R)  ? (GPIO_PORTB | GPIO_PIN9)  : \
          ((b) == PIC32MZ_RPC13R) ? (GPIO_PORTC | GPIO_PIN13) : \
          ((b) == PIC32MZ_RPC14R) ? (GPIO_PORTC | GPIO_PIN14) : \
          ((b) == PIC32MZ_RPC1R)  ? (GPIO_PORTC | GPIO_PIN1)  : \
          ((b) == PIC32MZ_RPC2R)  ? (GPIO_PORTC | GPIO_PIN2)  : \
          ((b) == PIC32MZ_RPC3R)  ? (GPIO_PORTC | GPIO_PIN3)  : \
          ((b) == PIC32MZ_RPC4R)  ? (GPIO_PORTC | GPIO_PIN4)  : \
          ((b) == PIC32MZ_RPD0R)  ? (GPIO_PORTD | GPIO_PIN0)  : \
          ((b) == PIC32MZ_RPD10R) ? (GPIO_PORTD | GPIO_PIN10) : \
          ((b) == PIC32MZ_RPD11R) ? (GPIO_PORTD | GPIO_PIN11) : \
          ((b) == PIC32MZ_RPD12R) ? (GPIO_PORTD | GPIO_PIN12) : \
          ((b) == PIC32MZ_RPD14R) ? (GPIO_PORTD | GPIO_PIN14) : \
          ((b) == PIC32MZ_RPD15R) ? (GPIO_PORTD | GPIO_PIN15) : \
          ((b) == PIC32MZ_RPD1R)  ? (GPIO_PORTD | GPIO_PIN1)  : \
          ((b) == PIC32MZ_RPD2R)  ? (GPIO_PORTD | GPIO_PIN2)  : \
          ((b) == PIC32MZ_RPD3R)  ? (GPIO_PORTD | GPIO_PIN3)  : \
          ((b) == PIC32MZ_RPD4R)  ? (GPIO_PORTD | GPIO_PIN4)  : \
          ((b) == PIC32MZ_RPD5R)  ? (GPIO_PORTD | GPIO_PIN5)  : \
          ((b) == PIC32MZ_RPD6R)  ? (GPIO_PORTD | GPIO_PIN6)  : \
          ((b) == PIC32MZ_RPD7R)  ? (GPIO_PORTD | GPIO_PIN7)  : \
          ((b) == PIC32MZ_RPD9R)  ? (GPIO_PORTD | GPIO_PIN9)  : \
          ((b) == PIC32MZ_RPE3R)  ? (GPIO_PORTE | GPIO_PIN3)  : \
          ((b) == PIC32MZ_RPE5R)  ? (GPIO_PORTE | GPIO_PIN5)  : \
          ((b) == PIC32MZ_RPE8R)  ? (GPIO_PORTE | GPIO_PIN8)  : \
          ((b) == PIC32MZ_RPE9R)  ? (GPIO_PORTE | GPIO_PIN9)  : \
          ((b) == PIC32MZ_RPF0R)  ? (GPIO_PORTF | GPIO_PIN0)  : \
          ((b) == PIC32MZ_RPF12R) ? (GPIO_PORTF | GPIO_PIN12) : \
          ((b) == PIC32MZ_RPF13R) ? (GPIO_PORTF | GPIO_PIN13) : \
          ((b) == PIC32MZ_RPF1R)  ? (GPIO_PORTF | GPIO_PIN1)  : \
          ((b) == PIC32MZ_RPF2R)  ? (GPIO_PORTF | GPIO_PIN2)  : \
          ((b) == PIC32MZ_RPF3R)  ? (GPIO_PORTF | GPIO_PIN3)  : \
          ((b) == PIC32MZ_RPF4R)  ? (GPIO_PORTF | GPIO_PIN4)  : \
          ((b) == PIC32MZ_RPF5R)  ? (GPIO_PORTF | GPIO_PIN5)  : \
          ((b) == PIC32MZ_RPF8R)  ? (GPIO_PORTF | GPIO_PIN8)  : \
          ((b) == PIC32MZ_RPG0R)  ? (GPIO_PORTG | GPIO_PIN0)  : \
          ((b) == PIC32MZ_RPG1R)  ? (GPIO_PORTG | GPIO_PIN1)  : \
          ((b) == PIC32MZ_RPG6R)  ? (GPIO_PORTG | GPIO_PIN6)  : \
          ((b) == PIC32MZ_RPG7R)  ? (GPIO_PORTG | GPIO_PIN7)  : \
          ((b) == PIC32MZ_RPG8R)  ? (GPIO_PORTG | GPIO_PIN8)  : \
          ((b) == PIC32MZ_RPG9R)  ? (GPIO_PORTG | GPIO_PIN9)  : \
          0                                                     \
        ))
#define PPS_OUTPUT_REGADDR_TO_GPIO(a) __PPS_OUTPUT_REGADDR_TO_GPIO(a)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_PPS_H */
