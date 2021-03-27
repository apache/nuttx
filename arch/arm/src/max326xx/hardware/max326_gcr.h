/****************************************************************************
 * arch/arm/src/max326xx/hardware/max326_gcr.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_GCR_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_GCR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define MAX326_GCR_SCON_OFFSET      0x0000  /* System Control Register */
#define MAX326_GCR_RST0_OFFSET      0x0004  /* Reset Register 0 */
#define MAX326_GCR_CLKCTRL_OFFSET   0x0008  /* Clock Control Register */
#define MAX326_GCR_PM_OFFSET        0x000c  /* Power Management Register */
#define MAX326_GCR_PCLKDIS0_OFFSET  0x0024  /* Peripheral Clocks Disable 0 */
#define MAX326_GCR_MEMCTRL_OFFSET   0x0028  /* Memory Clock Control */
#define MAX326_GCR_MEMZCTRL_OFFSET  0x002c  /* Memory Zeroize Register */
#define MAX326_GCR_SYSSTAT_OFFSET   0x0040  /* System Status Flags */
#define MAX326_GCR_RST1_OFFSET      0x0044  /* Reset Register 1 */
#define MAX326_GCR_PCLKDIS1_OFFSET  0x0048  /* Peripheral Clocks Disable 1 */
#define MAX326_GCR_EVTEN_OFFSET     0x004c  /* Event Enable Register */
#define MAX326_GCR_REV_OFFSET       0x0050  /* Revision Register */
#define MAX326_GCR_SYSIE_OFFSET     0x0054  /* System Status Interrupt Enable */

/* Register Addresses *******************************************************/

#define MAX326_GCR_SCON             (MAX326_GCR_BASE + MAX326_GCR_SCON_OFFSET)
#define MAX326_GCR_RST0             (MAX326_GCR_BASE + MAX326_GCR_RST0_OFFSET)
#define MAX326_GCR_CLKCTRL          (MAX326_GCR_BASE + MAX326_GCR_CLKCTRL_OFFSET)
#define MAX326_GCR_PM               (MAX326_GCR_BASE + MAX326_GCR_PM_OFFSET)
#define MAX326_GCR_PCLKDIS0         (MAX326_GCR_BASE + MAX326_GCR_PCLKDIS0_OFFSET)
#define MAX326_GCR_MEMCTRL          (MAX326_GCR_BASE + MAX326_GCR_MEMCTRL_OFFSET)
#define MAX326_GCR_MEMZCTRL         (MAX326_GCR_BASE + MAX326_GCR_MEMZCTRL_OFFSET)
#define MAX326_GCR_SYSSTAT          (MAX326_GCR_BASE + MAX326_GCR_SYSSTAT_OFFSET)
#define MAX326_GCR_RST1             (MAX326_GCR_BASE + MAX326_GCR_RST1_OFFSET)
#define MAX326_GCR_PCLKDIS1         (MAX326_GCR_BASE + MAX326_GCR_PCLKDIS1_OFFSET)
#define MAX326_GCR_EVTEN            (MAX326_GCR_BASE + MAX326_GCR_EVTEN_OFFSET)
#define MAX326_GCR_REV              (MAX326_GCR_BASE + MAX326_GCR_REV_OFFSET)
#define MAX326_GCR_SYSIE            (MAX326_GCR_BASE + MAX326_GCR_SYSIE_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* System Control Register */

#define GCR_SCON_FLASH_PAGEFLIP     (1 >> 4)  /* Bit 4:  Flash Page Flip Flag */
#define GCR_SCON_FPUDIS             (1 >> 5)  /* Bit 5:  Floating Point Unit (FPU) Disable */
#define GCR_SCON_ICC0FLUSH          (1 >> 6)  /* Bit 6:  Instruction Cache Controller Flush */
#define GCR_SCON_SWDDIS             (1 >> 14) /* Bit 14: Serial Wire Debug Disable */

/* Reset Register 0 */

#define GCR_RST0_DMA                (1 << 0)  /* Bit 0:  Standard DMA Reset */
#define GCR_RST0_WDT0               (1 << 1)  /* Bit 1:  Watchdog Timer 0 Reset */
#define GCR_RST0_GPIO0              (1 << 2)  /* Bit 2:  GPIO0 Reset */
#define GCR_RST0_TMR1               (1 << 6)  /* Bit 6:  Timer1 Reset */
#define GCR_RST0_TMR2               (1 << 7)  /* Bit 7:  Timer2 Reset */
#define GCR_RST0_UART0              (1 << 11) /* Bit 11: UART0 Reset */
#define GCR_RST0_UART1              (1 << 12) /* Bit 12: UART1 Reset */
#define GCR_RST0_SPI0               (1 << 13) /* Bit 13: SPI0 Reset */
#define GCR_RST0_SPI1               (1 << 14) /* Bit 14: SPIMSS (SPI1/I2S) Reset */
#define GCR_RST0_I2C0               (1 << 16) /* Bit 16: I2C0 Reset */
#define GCR_RST0_RTC                (1 << 17) /* Bit 17: RTC Reset */
#define GCR_RST0_SOFT               (1 << 29) /* Bit 29: Soft Reset */
#define GCR_RST0_PERIPH             (1 << 30) /* Bit 30: System Peripheral Reset */
#define GCR_RST0_SYSTEM             (1 << 31) /* Bit 31: System Reset */

/* Clock Control Register */

#define GCR_CLKCTRL_PSC_SHIFT       (6)       /* Bits 6-8: System Oscillator Prescaler */
#define GCR_CLKCTRL_PSC_MASK        (7 << GCR_CLKCTRL_PSC_SHIFT)
#  define GCR_CLKCTRL_PSC(n)        ((uint32_t)(n) << GCR_CLKCTRL_PSC_SHIFT)
#define GCR_CLKCTRL_CLKSEL_SHIFT    (9)       /* Bits 9-11: System Oscillator Source Select */
#define GCR_CLKCTRL_CLKSEL_MASK     (7 << GCR_CLKCTRL_CLKSEL_SHIFT)
#  define GCR_CLKCTRL_CLKSEL_HIRC   (0 << GCR_CLKCTRL_CLKSEL_SHIFT) /* High-Frequency Internal Oscillator (HFIO) */
#  define GCR_CLKCTRL_CLKSEL_LIRC8K (3 << GCR_CLKCTRL_CLKSEL_SHIFT) /* 8kHz Low-Frequency Internal Oscillator */
#  define GCR_CLKCTRL_CLKSEL_X32K   (6 << GCR_CLKCTRL_CLKSEL_SHIFT) /* 32.768kHz External Oscillator */

#define GCR_CLKCTRL_CLKRDY          (1 << 13) /* Bit 13: System Oscillator Clock Source Ready */
#define GCR_CLKCTRL_X32KEN          (1 << 17) /* Bit 17: 32.768kHz External Oscillator Enable */
#define GCR_CLKCTRL_HIRCEN          (1 << 18) /* Bit 18: High-Frequency Internal Oscillator (HFIO) Enable */
#define GCR_CLKCTRL_X32KRDY         (1 << 25) /* Bit 25: 32.768kHz External Oscillator Ready Status */
#define GCR_CLKCTRL_HIRCRDY         (1 << 26) /* Bit 26: High-Frequency Internal Oscillator Ready */
#define GCR_CLKCTRL_LIRC8KRDY       (1 << 29) /* Bit 29: 8kHz Internal Oscillator Ready Status */

/* Power Management Register */

#define GCR_PM_MODE_SHIFT           (0)       /* Bits 0-2: Operating Mode */
#define GCR_PM_MODE_MASK            (7 << GCR_PM_MODE_SHIFT)
#  define GCR_PM_MODE_ACTIVE        (0 << GCR_PM_MODE_SHIFT) /* Active mode */
#  define GCR_PM_MODE_BACKUP        (4 << GCR_PM_MODE_SHIFT) /* Backup Low Power Mode */
#  define GCR_PM_MODE_SHUTDOWN      (6 << GCR_PM_MODE_SHIFT) /* Shutdown Mode */

#define GCR_PM_GPIOWKEN             (1 << 4)  /* Bit 4:  GPIO Wakeup Enable */
#define GCR_PM_RTCWKEN              (1 << 5)  /* Bit 5:  RTC Alarm Wakeup Enable */
#define GCR_PM_HFIOPD               (1 << 15) /* Bit 15: HFIO DEEPSLEEP Auto Off */

/* Peripheral Clocks Disable 0 */

#define GCR_PCLKDIS0_GPIO0D         (1 << 0)  /* Bit 0:  GPIO0 Port and Pad Logic Clock Disable */
#define GCR_PCLKDIS0_DMAD           (1 << 5)  /* Bit 5:  Standard DMA Clock Disable */
#define GCR_PCLKDIS0_SPI0D          (1 << 6)  /* Bit 6:  SPI0 Clock Disable */
#define GCR_PCLKDIS0_SPI1D          (1 << 7)  /* Bit 7:  SPI1 Clock Disable */
#define GCR_PCLKDIS0_UART0D         (1 << 9)  /* Bit 9:  UART0 Clock Disable */
#define GCR_PCLKDIS0_UART1D         (1 << 10) /* Bit 10: UART1 Clock Disable */
#define GCR_PCLKDIS0_I2C0D          (1 << 13) /* Bit 14: I2C0 Clock Disable */
#define GCR_PCLKDIS0_TMR0D          (1 << 15) /* Bit 15: Timer0 Clock Disable */
#define GCR_PCLKDIS0_TMR1D          (1 << 16) /* Bit 16: Timer1 Clock Disable */
#define GCR_PCLKDIS0_TMR2D          (1 << 17) /* Bit 17: Timer2 Clock Disable */
#define GCR_PCLKDIS0_I2C1D          (1 << 28) /* Bit 28: I2C1 Clock Disable */

/* Memory Clock Control */

#define GCR_MEMCTRL_FWS_SHIFT       (0)       /* Bits 0-2: Flash Wait States */
#define GCR_MEMCTRL_FWS_MASK        (7 << GCR_MEMCTRL_FWS_SHIFT)
#  define GCR_MEMCTRL_FWS(n)        ((uint32_t)(n) << GCR_MEMCTRL_FWS_SHIFT)
#define GCR_MEMCTRL_RAM0LS          (1 << 8)  /* Bit 8:  System RAM 0 Light Sleep Enable */
#define GCR_MEMCTRL_RAM1LS          (1 << 9)  /* Bit 9:  System RAM 1 Light Sleep Enable */
#define GCR_MEMCTRL_RAM2LS          (1 << 10) /* Bit 10: System RAM 2 Light Sleep Enable */
#define GCR_MEMCTRL_RAM3LS          (1 << 11) /* Bit 11: System RAM 3 Light Sleep Enable */
#define GCR_MEMCTRL_ICACHERET       (1 << 12) /* Bit 12: ICC0 Cache RAM Light Sleep Enable */

/* Memory Zeroize Register */

#define GCR_MEMZCTRL_SRAM_ZERO      (1 << 0)  /* Bit 0:  System Data RAM Zeroization */
#define GCR_MEMZCTRL_ICACHE_ZERO    (1 << 1)  /* Bit 1:  Internal Cache Data and Tag RAM Zeroization */

/* System Status Flags */

#define GCR_SYSSTAT_ICELOCK         (1 << 0)  /* Bit 0:  Arm Cortex-M4 with FPU ICE Lock Status Flag */

/* Reset Register 1 */

#define GCR_RST1_I2C1               (1 << 0)  /* Bit 0:  I2C1 Reset */

/* Peripheral Clocks Disable 1 */

#define GCR_PCLKDIS1_FLCD           (1 << 3)  /* Bit 3:  Flash Controller Disable */
#define GCR_PCLKDIS1_ICCD           (1 << 11) /* Bit 11: ICC Clock Disable */

/* Event Enable Register */

#define GCR_EVTEN_DMAEVENT          (1 << 0)  /* Bit 0:  DMA CTZ Event Wake-Up Enable */
#define GCR_EVTEN_RXEVT             (1 << 1)  /* Bit 1:  RX Event Enabled */

/* Revision Register */

#define GCR_REV_MASK                (0xffff)  /* Bits 0-15: Maxim Integrated Chip Revision */

/* System Status Interrupt Enable */

#define GCR_SYSIE_ICEULIE           (1 << 0)  /* Bit 0:  Arm ICE Unlocked Interrupt Enable */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_GCR_H */
