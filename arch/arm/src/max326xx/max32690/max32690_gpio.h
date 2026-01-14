/****************************************************************************
 * arch/arm/src/max326xx/max32690/max32690_gpio.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX32690_MAX32690_GPIO_H
#define __ARCH_ARM_SRC_MAX326XX_MAX32690_MAX32690_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit-encoded input to max32690_gpio_config() ******************************/

/* 16-Bit Encoding:  DDDW RRRV HSII IMFF
 *
 *   Drive Strength:        DDD
 *   Wakeup:                W
 *   Pin Pull Up/Down:      RRR
 *   Initial Value:         V (output pins)
 *   Input Hysteresis:      H
 *   Slew Rate:             S
 *   Interrupt Mode:        III
 *   Pin Mode:              M
 *   Pin Function:          FF
 */

/* Drive Strength:
 *
 * DDD. .... .... ....
 */

#define GPIO_DRIVE_SHIFT         ( 13 )                      /* Bits 15-13: Drive strength */
#define GPIO_DRIVE_MASK          ( 7 << GPIO_DRIVE_SHIFT )
#  define GPIO_DRIVE_0_V_DIO_H   ( 0 << GPIO_DRIVE_SHIFT )
#  define GPIO_DRIVE_1_V_DIO_H   ( 1 << GPIO_DRIVE_SHIFT )
#  define GPIO_DRIVE_2_V_DIO_H   ( 2 << GPIO_DRIVE_SHIFT )
#  define GPIO_DRIVE_3_V_DIO_H   ( 3 << GPIO_DRIVE_SHIFT )
#  define GPIO_DRIVE_0_V_DIO     ( 4 << GPIO_DRIVE_SHIFT )
#  define GPIO_DRIVE_1_V_DIO     ( 5 << GPIO_DRIVE_SHIFT )
#  define GPIO_DRIVE_2_V_DIO     ( 6 << GPIO_DRIVE_SHIFT )
#  define GPIO_DRIVE_3_V_DIO     ( 7 << GPIO_DRIVE_SHIFT )

/* Wake-UP:
 *
 * ...W .... .... ....
 */

#define GPIO_WAKEUP              ( 1 << 12 )                 /* Bit 12: Wakeup Enable */

/* Pin Pull Up/Down: PP
 *
 * .... RRR. .... ....
 */

#define GPIO_IN_MODE_SHIFT       ( 9 )                       /* Bits 11-9: Pin pull up/down mode */
#define GPIO_IN_MODE_MASK        ( 7 << GPIO_IN_MODE_SHIFT )
#  define GPIO_FLOAT             ( 0 << GPIO_IN_MODE_SHIFT )    /* Neither pull-up nor -down */
#  define GPIO_PULLUP_WEAK       ( 1 << GPIO_IN_MODE_SHIFT )    /* Weak Pull-up resistor enabled     ( 1 M Ω to V DDIO )  */
#  define GPIO_PULLUP_STRONG     ( 2 << GPIO_IN_MODE_SHIFT )    /* Strong Pull-up resistor enabled   ( 25 K Ω to V DDIO )  */
#  define GPIO_PULL_DOWN_WEAK    ( 3 << GPIO_IN_MODE_SHIFT )    /* Weak Pull-down resistor enabled   ( 1 M Ω to V DDIOH ) ?? not GND */
#  define GPIO_PULL_DOWN_STRONG  ( 4 << GPIO_IN_MODE_SHIFT )    /* Strong Pull-down resistor enabled ( 25 K Ω to V DDIO ) ?? not GND */

/* Initial value: V
 *
 * .... ...V .... ....
 */

#define GPIO_VALUE_ONE           ( 1 << 8 )                  /* Bit 8: Initial GPIO output value */

/* Input Hysteresis:
 *
 * .... .... H... ....
 */

#define GPIO_HYSTERESIS          ( 1 << 7 )                  /* Bit 7: Input hysteresis */

/* Slew Rate:
 *
 * .... .... .S.. ....
 */

#define GPIO_SLEW                ( 1 << 6 )                  /* Bit 6: Slew rate mode */

/* Interrupt Mode:
 *
 * .... .... ..II I...
 */

#define GPIO_INT_SHIFT           ( 3 )                       /* Bit 5-3: Initial GPIO output value */
#define GPIO_INT_MASK            ( 7 << GPIO_INT_SHIFT )
#  define GPIO_INTRE             ( 0 << GPIO_INT_SHIFT )
#  define GPIO_INTFE             ( 1 << GPIO_INT_SHIFT )
#  define GPIO_INTBOTH           ( 2 << GPIO_INT_SHIFT )
#  define GPIO_INTLOW            ( 3 << GPIO_INT_SHIFT )
#  define GPIO_INTHIGH           ( 4 << GPIO_INT_SHIFT )

/* Pin Mode bits
 *
 *  ... .... .... .M..
 */

#  define GPIO_OUTPUT_MODE       ( 1 << 2 )                  /* Bits 2: Pin Input / Output mode */

/* Pin Function bits:
 *
 *  .... .... .... ..FF
 */

#define GPIO_FUNC_MASK           ( 3 )                       /* Bits 1-0 GPIO Mask */
#  define GPIO_IO                ( 0 )
#  define GPIO_ALT1              ( 1 )
#  define GPIO_ALT2              ( 2 )
#  define GPIO_ALT3              ( 3 )

#  define GPIO_PINMIN       0
#  define GPIO_PINMAX       31

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*gpio_irq_function_t)(int bank, int pin);

typedef struct
{
    uint8_t gpio_bank;
    uint8_t pin;
    uint16_t config;
    gpio_irq_function_t irq_handler;
} max32690_pinconfig_t;

#define __RW  volatile
#define __RO  volatile const

typedef struct
{
    __RW uint32_t en0;                  /* Offset 0x00: GPIO EN0 Register */
    __RW uint32_t en0_set;              /* Offset 0x04: GPIO EN0_SET Register */
    __RW uint32_t en0_clr;              /* Offset 0x08: GPIO EN0_CLR Register */
    __RW uint32_t outen;                /* Offset 0x0C: GPIO OUTEN Register */
    __RW uint32_t outen_set;            /* Offset 0x10: GPIO OUTEN_SET Register */
    __RW uint32_t outen_clr;            /* Offset 0x14: GPIO OUTEN_CLR Register */
    __RW uint32_t out;                  /* Offset 0x18: GPIO OUT Register */
    __RW uint32_t out_set;              /* Offset 0x1C: GPIO OUT_SET Register */
    __RW uint32_t out_clr;              /* Offset 0x20: GPIO OUT_CLR Register */
    __RO uint32_t in;                   /* Offset 0x24: GPIO IN Register */
    __RW uint32_t intmode;              /* Offset 0x28: GPIO INTMODE Register */
    __RW uint32_t intpol;               /* Offset 0x2C: GPIO INTPOL Register */
    __RW uint32_t inen;                 /* Offset 0x30: GPIO INEN Register */
    __RW uint32_t inten;                /* Offset 0x34: GPIO INTEN Register */
    __RW uint32_t inten_set;            /* Offset 0x38: GPIO INTEN_SET Register */
    __RW uint32_t inten_clr;            /* Offset 0x3C: GPIO INTEN_CLR Register */
    __RO uint32_t intfl;                /* Offset 0x40: GPIO INTFL Register */
    __RO uint32_t rsv_0x44;
    __RW uint32_t intfl_clr;            /* Offset 0x48: GPIO INTFL_CLR Register */
    __RW uint32_t wken;                 /* Offset 0x4C: GPIO WKEN Register */
    __RW uint32_t wken_set;             /* Offset 0x50: GPIO WKEN_SET Register */
    __RW uint32_t wken_clr;             /* Offset 0x54: GPIO WKEN_CLR Register */
    __RO uint32_t rsv_0x58;
    __RW uint32_t dualedge;             /* Offset 0x5C: GPIO DUALEDGE Register */
    __RW uint32_t padctrl0;             /* Offset 0x60: GPIO PADCTRL0 Register */
    __RW uint32_t padctrl1;             /* Offset 0x64: GPIO PADCTRL1 Register */
    __RW uint32_t en1;                  /* Offset 0x68: GPIO EN1 Register */
    __RW uint32_t en1_set;              /* Offset 0x6C: GPIO EN1_SET Register */
    __RW uint32_t en1_clr;              /* Offset 0x70: GPIO EN1_CLR Register */
    __RW uint32_t en2;                  /* Offset 0x74: GPIO EN2 Register */
    __RW uint32_t en2_set;              /* Offset 0x78: GPIO EN2_SET Register */
    __RW uint32_t en2_clr;              /* Offset 0x7C: GPIO EN2_CLR Register */
    __RO uint32_t rsv_0x80_0xa7[10];
    __RW uint32_t hysen;                /* Offset 0xA8: GPIO HYSEN Register */
    __RW uint32_t srsel;                /* Offset 0xAC: GPIO SRSEL Register */
    __RW uint32_t ds0;                  /* Offset 0xB0: GPIO DS0 Register */
    __RW uint32_t ds1;                  /* Offset 0xB4: GPIO DS1 Register */
    __RW uint32_t ps;                   /* Offset 0xB8: GPIO PS Register */
    __RO uint32_t rsv_0xbc;
    __RW uint32_t vssel;                /* Offset 0xC0: GPIO VSSEL Register */
} max32690_gpio_regs_t;

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: max32690_get_gpio_bank_regptr
 *
 * Description:
 *   Get the GPIO access struct address based in the GPIO Bank
 *
 ****************************************************************************/

max32690_gpio_regs_t *max32690_get_gpio_bank_regptr(int bank);

/****************************************************************************
 * Name: max32690_gpio_config
 *
 * Description:
 *   Configure a GPIO pin. Config based on bit-encoded descriptions.
 *
 ****************************************************************************/

int  max32690_gpio_config(max32690_pinconfig_t pinconf);

/****************************************************************************
 * Name: max32690_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool max32690_gpio_read(max32690_pinconfig_t pinconf);

/****************************************************************************
 * Name: max32690_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void max32690_gpio_write(max32690_pinconfig_t pinconf, bool value);

/****************************************************************************
 * Name: max32690_gpio_irqconfig
 *
 * Description:
 *   Enable a GPIO pin interrupt.
 *
 ****************************************************************************/

void max32690_gpio_irq_enable(max32690_pinconfig_t pinconf);

/****************************************************************************
 * Name: max32690_gpio_irqdisable
 *
 * Description:
 *   Disable a GPIO pin interrupt.
 *
 ****************************************************************************/

void max32690_gpio_irq_disable(max32690_pinconfig_t pinconf);

#ifdef __cplusplus
}

#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_MAX326XX_MAX32690_MAX32690_GPIO_H */
