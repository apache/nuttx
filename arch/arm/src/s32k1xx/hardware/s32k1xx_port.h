/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_port.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_PORT_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_PORT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define S32K1XX_PORTA               0
#define S32K1XX_PORTB               1
#define S32K1XX_PORTC               2
#define S32K1XX_PORTD               3
#define S32K1XX_PORTE               4
#define S32K1XX_NPORTS              5

/* PORT Register Offsets ****************************************************/

#define S32K1XX_PORT_PCR_OFFSET(n)  (0 + ((n) << 2)) /* Pin Control Register n=0..31 */

#define S32K1XX_PORT_GPCLR_OFFSET   0x0080  /* Global Pin Control Low Register */
#define S32K1XX_PORT_GPCHR_OFFSET   0x0084  /* Global Pin Control High Register */
#define S32K1XX_PORT_GICLR_OFFSET   0x0088  /* Global Interrupt Control Low Register */
#define S32K1XX_PORT_GICHR_OFFSET   0x008c  /* Global Interrupt Control High Register */
#define S32K1XX_PORT_ISFR_OFFSET    0x00a0  /* Interrupt Status Flag Register */
#define S32K1XX_PORT_DFER_OFFSET    0x00c0  /* Digital Filter Enable Register */
#define S32K1XX_PORT_DFCR_OFFSET    0x00c4  /* Digital Filter Clock Register */
#define S32K1XX_PORT_DFWR_OFFSET    0x00c8  /* Digital Filter Width Register */

/* PORT Register Addresses **************************************************/

#define S32K1XX_PORT_PCR_BASE(p,n)  (S32K1XX_PORT_BASE(p) + S32K1XX_PORT_PCR_OFFSET(n))
#define S32K1XX_PORT_GPCLR(p)       (S32K1XX_PORT_BASE(p) + S32K1XX_PORT_GPCLR_OFFSET)
#define S32K1XX_PORT_GPCHR(p)       (S32K1XX_PORT_BASE(p) + S32K1XX_PORT_GPCHR_OFFSET)
#define S32K1XX_PORT_GICLR(p)       (S32K1XX_PORT_BASE(p) + S32K1XX_PORT_GICLR_OFFSET)
#define S32K1XX_PORT_GICHR(p)       (S32K1XX_PORT_BASE(p) + S32K1XX_PORT_GICHR_OFFSET)
#define S32K1XX_PORT_ISFR(p)        (S32K1XX_PORT_BASE(p) + S32K1XX_PORT_ISFR_OFFSET)
#define S32K1XX_PORT_DFER(p)        (S32K1XX_PORT_BASE(p) + S32K1XX_PORT_DFER_OFFSET)
#define S32K1XX_PORT_DFCR(p)        (S32K1XX_PORT_BASE(p) + S32K1XX_PORT_DFCR_OFFSET)
#define S32K1XX_PORT_DFWR(p)        (S32K1XX_PORT_BASE(p) + S32K1XX_PORT_DFWR_OFFSET)

#define S32K1XX_PORTA_PCR_BASE(n)   (S32K1XX_PORTA_BASE + S32K1XX_PORT_PCR_OFFSET(n))
#define S32K1XX_PORTA_GPCLR         (S32K1XX_PORTA_BASE + S32K1XX_PORT_GPCLR_OFFSET)
#define S32K1XX_PORTA_GPCHR         (S32K1XX_PORTA_BASE + S32K1XX_PORT_GPCHR_OFFSET)
#define S32K1XX_PORTA_GICLR         (S32K1XX_PORTA_BASE + S32K1XX_PORT_GICLR_OFFSET)
#define S32K1XX_PORTA_GICHR         (S32K1XX_PORTA_BASE + S32K1XX_PORT_GICHR_OFFSET)
#define S32K1XX_PORTA_ISFR          (S32K1XX_PORTA_BASE + S32K1XX_PORT_ISFR_OFFSET)
#define S32K1XX_PORTA_DFER          (S32K1XX_PORTA_BASE + S32K1XX_PORT_DFER_OFFSET)
#define S32K1XX_PORTA_DFCR          (S32K1XX_PORTA_BASE + S32K1XX_PORT_DFCR_OFFSET)
#define S32K1XX_PORTA_DFWR          (S32K1XX_PORTA_BASE + S32K1XX_PORT_DFWR_OFFSET)

#define S32K1XX_PORTB_PCR_BASE(n)   (S32K1XX_PORTB_BASE + S32K1XX_PORT_PCR_OFFSET(n))
#define S32K1XX_PORTB_GPCLR         (S32K1XX_PORTB_BASE + S32K1XX_PORT_GPCLR_OFFSET)
#define S32K1XX_PORTB_GPCHR         (S32K1XX_PORTB_BASE + S32K1XX_PORT_GPCHR_OFFSET)
#define S32K1XX_PORTB_GICLR         (S32K1XX_PORTB_BASE + S32K1XX_PORT_GICLR_OFFSET)
#define S32K1XX_PORTB_GICHR         (S32K1XX_PORTB_BASE + S32K1XX_PORT_GICHR_OFFSET)
#define S32K1XX_PORTB_ISFR          (S32K1XX_PORTB_BASE + S32K1XX_PORT_ISFR_OFFSET)
#define S32K1XX_PORTB_DFER          (S32K1XX_PORTB_BASE + S32K1XX_PORT_DFER_OFFSET)
#define S32K1XX_PORTB_DFCR          (S32K1XX_PORTB_BASE + S32K1XX_PORT_DFCR_OFFSET)
#define S32K1XX_PORTB_DFWR          (S32K1XX_PORTB_BASE + S32K1XX_PORT_DFWR_OFFSET)

#define S32K1XX_PORTC_PCR_BASE(n)   (S32K1XX_PORTC_BASE + S32K1XX_PORT_PCR_OFFSET(n))
#define S32K1XX_PORTC_GPCLR         (S32K1XX_PORTC_BASE + S32K1XX_PORT_GPCLR_OFFSET)
#define S32K1XX_PORTC_GPCHR         (S32K1XX_PORTC_BASE + S32K1XX_PORT_GPCHR_OFFSET)
#define S32K1XX_PORTC_GICLR         (S32K1XX_PORTC_BASE + S32K1XX_PORT_GICLR_OFFSET)
#define S32K1XX_PORTC_GICHR         (S32K1XX_PORTC_BASE + S32K1XX_PORT_GICHR_OFFSET)
#define S32K1XX_PORTC_ISFR          (S32K1XX_PORTC_BASE + S32K1XX_PORT_ISFR_OFFSET)
#define S32K1XX_PORTC_DFER          (S32K1XX_PORTC_BASE + S32K1XX_PORT_DFER_OFFSET)
#define S32K1XX_PORTC_DFCR          (S32K1XX_PORTC_BASE + S32K1XX_PORT_DFCR_OFFSET)
#define S32K1XX_PORTC_DFWR          (S32K1XX_PORTC_BASE + S32K1XX_PORT_DFWR_OFFSET)

#define S32K1XX_PORTD_PCR_BASE(n)   (S32K1XX_PORTD_BASE + S32K1XX_PORT_PCR_OFFSET(n))
#define S32K1XX_PORTD_GPCLR         (S32K1XX_PORTD_BASE + S32K1XX_PORT_GPCLR_OFFSET)
#define S32K1XX_PORTD_GPCHR         (S32K1XX_PORTD_BASE + S32K1XX_PORT_GPCHR_OFFSET)
#define S32K1XX_PORTD_GICLR         (S32K1XX_PORTD_BASE + S32K1XX_PORT_GICLR_OFFSET)
#define S32K1XX_PORTD_GICHR         (S32K1XX_PORTD_BASE + S32K1XX_PORT_GICHR_OFFSET)
#define S32K1XX_PORTD_ISFR          (S32K1XX_PORTD_BASE + S32K1XX_PORT_ISFR_OFFSET)
#define S32K1XX_PORTD_DFER          (S32K1XX_PORTD_BASE + S32K1XX_PORT_DFER_OFFSET)
#define S32K1XX_PORTD_DFCR          (S32K1XX_PORTD_BASE + S32K1XX_PORT_DFCR_OFFSET)
#define S32K1XX_PORTD_DFWR          (S32K1XX_PORTD_BASE + S32K1XX_PORT_DFWR_OFFSET)

#define S32K1XX_PORTE_PCR_BASE(n)   (S32K1XX_PORTE_BASE + S32K1XX_PORT_PCR_OFFSET(n))
#define S32K1XX_PORTE_GPCLR         (S32K1XX_PORTE_BASE + S32K1XX_PORT_GPCLR_OFFSET)
#define S32K1XX_PORTE_GPCHR         (S32K1XX_PORTE_BASE + S32K1XX_PORT_GPCHR_OFFSET)
#define S32K1XX_PORTE_GICLR         (S32K1XX_PORTE_BASE + S32K1XX_PORT_GICLR_OFFSET)
#define S32K1XX_PORTE_GICHR         (S32K1XX_PORTE_BASE + S32K1XX_PORT_GICHR_OFFSET)
#define S32K1XX_PORTE_ISFR          (S32K1XX_PORTE_BASE + S32K1XX_PORT_ISFR_OFFSET)
#define S32K1XX_PORTE_DFER          (S32K1XX_PORTE_BASE + S32K1XX_PORT_DFER_OFFSET)
#define S32K1XX_PORTE_DFCR          (S32K1XX_PORTE_BASE + S32K1XX_PORT_DFCR_OFFSET)
#define S32K1XX_PORTE_DFWR          (S32K1XX_PORTE_BASE + S32K1XX_PORT_DFWR_OFFSET)

/* PORT Register Bitfield Definitions ***************************************/

/* Pin Control Register n=0..31 */

#define PORT_PCR_PS                 (1 << 0)  /* Bit 0:  Pull Select */
#  define PORT_PCR_PULLDOWN         (0)       /*         Enable internal pulldown */
#  define PORT_PCR_PULLUP           (1 << 0)  /*         Enable internal pullup */
#define PORT_PCR_PE                 (1 << 1)  /* Bit 1:  Pull Enable */
#define PORT_PCR_PFE                (1 << 4)  /* Bit 4:  Passive Filter Enable */
#define PORT_PCR_DSE                (1 << 6)  /* Bit 6:  Drive Strength Enable */
#define PORT_PCR_MUX_SHIFT          (8)       /* Bits 8-10:  Pin Mux Control */
#define PORT_PCR_MUX_MASK           (7 << PORT_PCR_MUX_SHIFT)
#  define PORT_PCR_MUX_ANALOG       (0 << PORT_PCR_MUX_SHIFT) /* Alternative 0: Pin disable/analog */
#  define PORT_PCR_MUX_GPIO         (1 << PORT_PCR_MUX_SHIFT) /* Alternative 1 (GPIO) */
#  define PORT_PCR_MUX_ALT2         (2 << PORT_PCR_MUX_SHIFT) /* Alternative 2 (chip-specific) */
#  define PORT_PCR_MUX_ALT3         (3 << PORT_PCR_MUX_SHIFT) /* Alternative 3 (chip-specific) */
#  define PORT_PCR_MUX_ALT4         (4 << PORT_PCR_MUX_SHIFT) /* Alternative 4 (chip-specific) */
#  define PORT_PCR_MUX_ALT5         (5 << PORT_PCR_MUX_SHIFT) /* Alternative 5 (chip-specific) */
#  define PORT_PCR_MUX_ALT6         (6 << PORT_PCR_MUX_SHIFT) /* Alternative 6 (chip-specific) */
#  define PORT_PCR_MUX_ALT7         (7 << PORT_PCR_MUX_SHIFT) /* Alternative 7 (chip-specific) */

#define PORT_PCR_LK                 (1 << 15) /* Bit 15: Lock Register */
#define PORT_PCR_IRQC_SHIFT         (16)      /* Bits 16-19:  Interrupt Configuration */
#define PORT_PCR_IRQC_MASK          (15 << PORT_PCR_IRQC_SHIFT)
#  define PORT_PCR_IRQC_DISABLED    (0 << PORT_PCR_IRQC_SHIFT)  /* Interrupt Status Flag (ISF) is disabled */
#  define PORT_PCR_IRQC_DMARISING   (1 << PORT_PCR_IRQC_SHIFT)  /* ISF flag and DMA request on rising edge */
#  define PORT_PCR_IRQC_DMAFALLING  (2 << PORT_PCR_IRQC_SHIFT)  /* ISF flag and DMA request on falling edge */
#  define PORT_PCR_IRQC_DMABOTH     (3 << PORT_PCR_IRQC_SHIFT)  /* ISF flag and DMA request on either edge */
#  define PORT_PCR_IRQC_ZERO        (8 << PORT_PCR_IRQC_SHIFT)  /* ISF flag and Interrupt when logic 0 */
#  define PORT_PCR_IRQC_RISING      (9 << PORT_PCR_IRQC_SHIFT)  /* ISF flag and Interrupt on rising-edge */
#  define PORT_PCR_IRQC_FALLING     (10 << PORT_PCR_IRQC_SHIFT) /* ISF flag and Interrupt on falling-edge */
#  define PORT_PCR_IRQC_BOTH        (11 << PORT_PCR_IRQC_SHIFT) /* ISF flag and Interrupt on either edge */
#  define PORT_PCR_IRQC_ONE         (12 << PORT_PCR_IRQC_SHIFT) /* ISF flag and Interrupt when logic 1 */

#define PORT_PCR_ISF                (1 << 24) /* Bit 24: Interrupt Status Flag */

/* Global Pin Control Low Register */

#define PORT_GPCLR_GPWD_SHIFT       (0)       /* Bits 0-15: Global Pin Write Data */
#define PORT_GPCLR_GPWD_MASK        (0xffff << PORT_GPCLR_GPWD_SHIFT)
#  define PORT_GPCLR_GPWD_PIN(n)    ((uint32_t)(n) << PORT_GPCLR_GPWD_SHIFT) /* Pin n=0..15 */

#define PORT_GPCLR_GPWE_SHIFT       (16)      /* Bits 16-31: Global Pin Write Enable */
#define PORT_GPCLR_GPWE_MASK        (0xffff << PORT_GPCLR_GPWE_SHIFT)
#  define PORT_GPCLR_GPWE_PIN(n)    ((uint32_t)(n) << PORT_GPCLR_GPWE_SHIFT) /* Pin n=0..15 */

/* Global Pin Control High Register */

#define PORT_GPCHR_GPWD_SHIFT       (0)       /* Bits 0-15: Global Pin Write Data */
#define PORT_GPCHR_GPWD_MASK        (0xffff << PORT_GPCHR_GPWD_SHIFT)
#  define PORT_GPCHR_GPWD_PIN(n)    ((uint32_t)((n) - 16) << PORT_GPCHR_GPWD_SHIFT) /* Pin n=16..31 */

#define PORT_GPCHR_GPWE_SHIFT       (16)      /* Bits 16-31: Global Pin Write Enable */
#define PORT_GPCHR_GPWE_MASK        (0xffff << PORT_GPCHR_GPWE_SHIFT)
#  define PORT_GPCHR_GPWE_PIN(n)    ((uint32_t)((n) - 16) << PORT_GPCHR_GPWE_SHIFT) /* Pin n=16..31 */

/* Global Interrupt Control Low Register */

#define PORT_GICLR_GIWD_SHIFT       (0)       /* Bits 0-15: Global Interrupt Write Data */
#define PORT_GICLR_GIWD_MASK        (0xffff << PORT_GICLR_GIWD_SHIFT)
#  define PORT_GICLR_GIWD_PIN(n)    ((uint32_t)(n) << PORT_GICLR_GIWD_SHIFT) /* Pin n=0..15 */

#define PORT_GICLR_GIWE_SHIFT       (16)      /* Bits 16-31: Global Interrupt Write Enable */
#define PORT_GICLR_GIWE_MASK        (0xffff << PORT_GICLR_GIWE_SHIFT)
#  define PORT_GICLR_GIWE_PIN(n)    ((uint32_t)(n) << PORT_GICLR_GIWE_SHIFT) /* Pin n=0..15 */

/* Global Interrupt Control High Register */

#define PORT_GICHR_GIWD_SHIFT       (0)       /* Bits 0-15: Global Interrupt Write Data */
#define PORT_GICHR_GIWD_MASK        (0xffff << PORT_GICHR_GIWD_SHIFT)
#  define PORT_GICHR_GIWD_PIN(n)    ((uint32_t)((n) - 16) << PORT_GICHR_GIWD_SHIFT) /* Pin n=16..31 */

#define PORT_GICHR_GIWE_SHIFT       (16)      /* Bits 16-31: Global Interrupt Write Enable */
#define PORT_GICHR_GIWE_MASK        (0xffff << PORT_GICHR_GIWE_SHIFT)
#  define PORT_GICHR_GIWE_PIN(n)    ((uint32_t)((n) - 16) << PORT_GICHR_GIWE_SHIFT) /* Pin n=16..31 */

/* Interrupt Status Flag Register */

#define PORT_ISFR(n)                (1 << (n))  /* Interrupt Status Flag, n=0-31 */

/* Digital Filter Enable Register */

#define PORT_DFER(n)                (1 << (n))  /* Digital Filter Enable, n=0-31 */

/* Digital Filter Clock Register */

#define PORT_DFCR_CS                (1 << 0)  /* Bit 0:  Clock Source */
#  define PORT_DFCR_BUSCLK          (0)       /*         Digital filters clocked by bus clock */
#  define PORT_DFCR_LPOPCLK         (1 << 0)  /*         Digital filters clocked by LPO clock */

/* Digital Filter Width Register */

#define PORT_DFWR_FILT_SHIFT        (0)       /* Bits 0-4: Filter Length */
#define PORT_DFWR_FILT_MASK         (31 << PORT_DFWR_FILT_SHIFT)
#  define PORT_DFWR_FILT(n)         ((uint32_t)(n) << PORT_DFWR_FILT_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_PORT_H */
