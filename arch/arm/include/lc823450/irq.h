/****************************************************************************
 * arch/arm/include/lc823450/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_LC823450_IRQ_H
#define __ARCH_ARM_INCLUDE_LC823450_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Processor Exceptions (vectors 0-15) */

#define LC823450_IRQ_RESERVED       (0) /* Reserved vector (only used with CONFIG_DEBUG) */
                                        /* Vector  0: Reset stack pointer value */
                                        /* Vector  1: Reset (not handler as an IRQ) */
#define LC823450_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define LC823450_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
#define LC823450_IRQ_MEMFAULT       (4) /* Vector  4: Memory management (MPU) */
#define LC823450_IRQ_BUSFAULT       (5) /* Vector  5: Bus fault */
#define LC823450_IRQ_USAGEFAULT     (6) /* Vector  6: Usage fault */
#define LC823450_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define LC823450_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
                                        /* Vector 13: Reserved */
#define LC823450_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define LC823450_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16).
 * These definitions are chip-specific
 */

#define LC823450_IRQ_INTERRUPTS    (16) /* Vector number of the first external interrupt */

#define LC823450_IRQ_CTXM3_00       (LC823450_IRQ_INTERRUPTS+0)   /* 16: CortexM3_00 interrupt */
#define LC823450_IRQ_CTXM3_01       (LC823450_IRQ_INTERRUPTS+1)   /* 17: CortexM3_01 interrupt */
#define LC823450_IRQ_CTXM3_02       (LC823450_IRQ_INTERRUPTS+2)   /* 18: CortexM3_02 interrupt */
#define LC823450_IRQ_CTXM3_03       (LC823450_IRQ_INTERRUPTS+3)   /* 19: CortexM3_03 interrupt */
#define LC823450_IRQ_CTXM3_10       (LC823450_IRQ_INTERRUPTS+4)   /* 20: CortexM3_00 interrupt */
#define LC823450_IRQ_CTXM3_11       (LC823450_IRQ_INTERRUPTS+5)   /* 21: CortexM3_01 interrupt */
#define LC823450_IRQ_CTXM3_12       (LC823450_IRQ_INTERRUPTS+6)   /* 22: CortexM3_02 interrupt */
#define LC823450_IRQ_CTXM3_13       (LC823450_IRQ_INTERRUPTS+7)   /* 23: CortexM3_03 interrupt */
#define LC823450_IRQ_LPDSP0         (LC823450_IRQ_INTERRUPTS+8)   /* 24: LPDSP0 interrupt */
#define LC823450_IRQ_LPDSP1         (LC823450_IRQ_INTERRUPTS+9)   /* 25: LPDSP1 interrupt */
#define LC823450_IRQ_LPDSP2         (LC823450_IRQ_INTERRUPTS+10)  /* 26: LPDSP2 interrupt */
#define LC823450_IRQ_LPDSP3         (LC823450_IRQ_INTERRUPTS+11)  /* 27: LPDSP3 interrupt */
#define LC823450_IRQ_WDT0           (LC823450_IRQ_INTERRUPTS+12)  /* 28: WatchDogTimer0 interrupt */
#define LC823450_IRQ_WDT1           (LC823450_IRQ_INTERRUPTS+13)  /* 29: WatchDogTimer1 interrupt */
#define LC823450_IRQ_WDT2           (LC823450_IRQ_INTERRUPTS+14)  /* 30: WatchDogTimer2 interrupt */
#define LC823450_IRQ_BTIMER0        (LC823450_IRQ_INTERRUPTS+15)  /* 31: BasicTimer0 interrupt */
#define LC823450_IRQ_BTIMER1        (LC823450_IRQ_INTERRUPTS+16)  /* 32: BasicTimer0 interrupt */
#define LC823450_IRQ_BTIMER2        (LC823450_IRQ_INTERRUPTS+17)  /* 33: BasicTimer0 interrupt */
#define LC823450_IRQ_MTIMER00       (LC823450_IRQ_INTERRUPTS+18)  /* 34: MultipleTimer00 interrupt */
#define LC823450_IRQ_MTIMER01       (LC823450_IRQ_INTERRUPTS+19)  /* 35: MultipleTimer01 interrupt */
#define LC823450_IRQ_MTIMER10       (LC823450_IRQ_INTERRUPTS+20)  /* 36: MultipleTimer10 interrupt */
#define LC823450_IRQ_MTIMER11       (LC823450_IRQ_INTERRUPTS+21)  /* 37: MultipleTimer11 interrupt */
#define LC823450_IRQ_MTIMER20       (LC823450_IRQ_INTERRUPTS+22)  /* 38: MultipleTimer20 interrupt */
#define LC823450_IRQ_MTIMER21       (LC823450_IRQ_INTERRUPTS+23)  /* 39: MultipleTimer21 interrupt */
#define LC823450_IRQ_MTIMER30       (LC823450_IRQ_INTERRUPTS+24)  /* 40: MultipleTimer30 interrupt */
#define LC823450_IRQ_MTIMER31       (LC823450_IRQ_INTERRUPTS+25)  /* 41: MultipleTimer31 interrupt */
#define LC823450_IRQ_EHCI           (LC823450_IRQ_INTERRUPTS+26)  /* 42: USB HOST EHCI interrupt */
#define LC823450_IRQ_OHCI           (LC823450_IRQ_INTERRUPTS+27)  /* 43: USB HOST OHCI interrupt */
#define LC823450_IRQ_SERFLASH       (LC823450_IRQ_INTERRUPTS+28)  /* 44: USB HOST OHCI interrupt */
#define LC823450_IRQ_DMAC           (LC823450_IRQ_INTERRUPTS+29)  /* 45: DMA Controller interrupt */
#define LC823450_IRQ_SDCSYNC0       (LC823450_IRQ_INTERRUPTS+30)  /* 46: SDCardSync0 interrupt */
#define LC823450_IRQ_SDCSYNC1       (LC823450_IRQ_INTERRUPTS+31)  /* 47: SDCardSync1 interrupt */
#define LC823450_IRQ_SDCSYNC2       (LC823450_IRQ_INTERRUPTS+32)  /* 48: SDCardSync2 interrupt */
#define LC823450_IRQ_SDCASYNC0      (LC823450_IRQ_INTERRUPTS+33)  /* 49: SDCardAsync0 interrupt */
#define LC823450_IRQ_SDCASYNC1      (LC823450_IRQ_INTERRUPTS+34)  /* 50: SDCardAsync1 interrupt */
#define LC823450_IRQ_SDCASYNC2      (LC823450_IRQ_INTERRUPTS+35)  /* 51: SDCardAsync2 interrupt */
#define LC823450_IRQ_MEMSTICK       (LC823450_IRQ_INTERRUPTS+36)  /* 52: MemoryStick interrupt */
#define LC823450_IRQ_MEMSTICKINS    (LC823450_IRQ_INTERRUPTS+37)  /* 53: MemoryStick ins interrupt */
#define LC823450_IRQ_DSPCMD         (LC823450_IRQ_INTERRUPTS+38)  /* 54: DSP cmd interface interrupt */
#define LC823450_IRQ_ADC            (LC823450_IRQ_INTERRUPTS+39)  /* 55: AD Converter interrupt */
#define LC823450_IRQ_SIO            (LC823450_IRQ_INTERRUPTS+40)  /* 56: SIO interrupt */
#define LC823450_IRQ_I2C0           (LC823450_IRQ_INTERRUPTS+41)  /* 57: I2C0 interrupt */
#define LC823450_IRQ_I2C1           (LC823450_IRQ_INTERRUPTS+42)  /* 58: I2C1 interrupt */
#define LC823450_IRQ_UART0          (LC823450_IRQ_INTERRUPTS+43)  /* 59: UART0 interrupt */
#define LC823450_IRQ_UART1          (LC823450_IRQ_INTERRUPTS+44)  /* 60: UART1 interrupt */
#define LC823450_IRQ_UART2          (LC823450_IRQ_INTERRUPTS+45)  /* 61: UART2 interrupt */
#define LC823450_IRQ_RTC            (LC823450_IRQ_INTERRUPTS+46)  /* 62: RTC interrupt */
#define LC823450_IRQ_RTCKEY         (LC823450_IRQ_INTERRUPTS+47)  /* 63: RTCKEY interrupt */
#define LC823450_IRQ_AUDIOBUF0      (LC823450_IRQ_INTERRUPTS+48)  /* 64: AudioBuffer0 interrupt */
#define LC823450_IRQ_AUDIOBUF1      (LC823450_IRQ_INTERRUPTS+49)  /* 65: AudioBuffer1 interrupt */
#define LC823450_IRQ_AUDIOBUF2      (LC823450_IRQ_INTERRUPTS+50)  /* 66: AudioBuffer2 interrupt */
#define LC823450_IRQ_AUDIOBUF3      (LC823450_IRQ_INTERRUPTS+51)  /* 67: AudioBuffer3 interrupt */
#define LC823450_IRQ_AUDIOSTAT0     (LC823450_IRQ_INTERRUPTS+52)  /* 68: AudioStatus0 interrupt */
#define LC823450_IRQ_AUDIOSTAT1     (LC823450_IRQ_INTERRUPTS+53)  /* 69: AudioStatus1 interrupt */
#define LC823450_IRQ_AUDIOSTAT2     (LC823450_IRQ_INTERRUPTS+54)  /* 70: AudioStatus2 interrupt */
#define LC823450_IRQ_AUDIOSTAT3     (LC823450_IRQ_INTERRUPTS+55)  /* 71: AudioStatus3 interrupt */
#define LC823450_IRQ_AUDIOTM0       (LC823450_IRQ_INTERRUPTS+56)  /* 72: AudioTimer0 interrupt */
#define LC823450_IRQ_AUDIOTM1       (LC823450_IRQ_INTERRUPTS+57)  /* 73: AudioTimer1 interrupt */
#define LC823450_IRQ_USBDEV         (LC823450_IRQ_INTERRUPTS+58)  /* 74: USB Device interrupt */
#define LC823450_IRQ_EXTINT0        (LC823450_IRQ_INTERRUPTS+59)  /* 75: ExternalINT0 interrupt */
#define LC823450_IRQ_EXTINT1        (LC823450_IRQ_INTERRUPTS+60)  /* 76: ExternalINT1 interrupt */
#define LC823450_IRQ_EXTINT2        (LC823450_IRQ_INTERRUPTS+61)  /* 77: ExternalINT2 interrupt */
#define LC823450_IRQ_EXTINT3        (LC823450_IRQ_INTERRUPTS+62)  /* 78: ExternalINT3 interrupt */
#define LC823450_IRQ_EXTINT4        (LC823450_IRQ_INTERRUPTS+63)  /* 79: ExternalINT4 interrupt */
#define LC823450_IRQ_EXTINT5        (LC823450_IRQ_INTERRUPTS+64)  /* 80: ExternalINT5 interrupt */

#define LC823450_IRQ_NEXTINT        (65)
#define LC823450_IRQ_NIRQS          (LC823450_IRQ_EXTINT5 + 1)

#define LC823450_IRQ_GPIO00         (LC823450_IRQ_NIRQS + 0)      /* 81: GPIO00 */
#define LC823450_IRQ_GPIO01         (LC823450_IRQ_NIRQS + 1)      /* 82: GPIO01 */
#define LC823450_IRQ_GPIO02         (LC823450_IRQ_NIRQS + 2)      /* 83: GPIO02 */
#define LC823450_IRQ_GPIO03         (LC823450_IRQ_NIRQS + 3)      /* 84: GPIO03 */
#define LC823450_IRQ_GPIO04         (LC823450_IRQ_NIRQS + 4)      /* 85: GPIO04 */
#define LC823450_IRQ_GPIO05         (LC823450_IRQ_NIRQS + 5)      /* 86: GPIO05 */
#define LC823450_IRQ_GPIO06         (LC823450_IRQ_NIRQS + 6)      /* 87: GPIO06 */
#define LC823450_IRQ_GPIO07         (LC823450_IRQ_NIRQS + 7)      /* 88: GPIO07 */
#define LC823450_IRQ_GPIO08         (LC823450_IRQ_NIRQS + 8)      /* 89: GPIO08 */
#define LC823450_IRQ_GPIO09         (LC823450_IRQ_NIRQS + 9)      /* 90: GPIO09 */
#define LC823450_IRQ_GPIO0A         (LC823450_IRQ_NIRQS + 10)     /* 91: GPIO0A */
#define LC823450_IRQ_GPIO0B         (LC823450_IRQ_NIRQS + 11)     /* 92: GPIO0B */
#define LC823450_IRQ_GPIO0C         (LC823450_IRQ_NIRQS + 12)     /* 93: GPIO0C */
#define LC823450_IRQ_GPIO0D         (LC823450_IRQ_NIRQS + 13)     /* 94: GPIO0D */
#define LC823450_IRQ_GPIO0E         (LC823450_IRQ_NIRQS + 14)     /* 95: GPIO0E */
#define LC823450_IRQ_GPIO0F         (LC823450_IRQ_NIRQS + 15)     /* 96: GPIO0F */
#define LC823450_IRQ_GPIO10         (LC823450_IRQ_NIRQS + 16)     /* 97: GPIO10 */
#define LC823450_IRQ_GPIO11         (LC823450_IRQ_NIRQS + 17)     /* 98: GPIO11 */
#define LC823450_IRQ_GPIO12         (LC823450_IRQ_NIRQS + 18)     /* 99: GPIO12 */
#define LC823450_IRQ_GPIO13         (LC823450_IRQ_NIRQS + 19)     /* 100: GPIO13 */
#define LC823450_IRQ_GPIO14         (LC823450_IRQ_NIRQS + 20)     /* 101: GPIO14 */
#define LC823450_IRQ_GPIO15         (LC823450_IRQ_NIRQS + 21)     /* 102: GPIO15 */
#define LC823450_IRQ_GPIO16         (LC823450_IRQ_NIRQS + 22)     /* 103: GPIO16 */
#define LC823450_IRQ_GPIO17         (LC823450_IRQ_NIRQS + 23)     /* 104: GPIO17 */
#define LC823450_IRQ_GPIO18         (LC823450_IRQ_NIRQS + 24)     /* 105: GPIO18 */
#define LC823450_IRQ_GPIO19         (LC823450_IRQ_NIRQS + 25)     /* 106: GPIO19 */
#define LC823450_IRQ_GPIO1A         (LC823450_IRQ_NIRQS + 26)     /* 107: GPIO1A */
#define LC823450_IRQ_GPIO1B         (LC823450_IRQ_NIRQS + 27)     /* 108: GPIO1B */
#define LC823450_IRQ_GPIO1C         (LC823450_IRQ_NIRQS + 28)     /* 109: GPIO1C */
#define LC823450_IRQ_GPIO1D         (LC823450_IRQ_NIRQS + 29)     /* 110: GPIO1D */
#define LC823450_IRQ_GPIO1E         (LC823450_IRQ_NIRQS + 30)     /* 111: GPIO1E */
#define LC823450_IRQ_GPIO1F         (LC823450_IRQ_NIRQS + 31)     /* 112: GPIO1F */
#define LC823450_IRQ_GPIO20         (LC823450_IRQ_NIRQS + 32)     /* 113: GPIO20 */
#define LC823450_IRQ_GPIO21         (LC823450_IRQ_NIRQS + 33)     /* 114: GPIO21 */
#define LC823450_IRQ_GPIO22         (LC823450_IRQ_NIRQS + 34)     /* 115: GPIO22 */
#define LC823450_IRQ_GPIO23         (LC823450_IRQ_NIRQS + 35)     /* 116: GPIO23 */
#define LC823450_IRQ_GPIO24         (LC823450_IRQ_NIRQS + 36)     /* 117: GPIO24 */
#define LC823450_IRQ_GPIO25         (LC823450_IRQ_NIRQS + 37)     /* 118: GPIO25 */
#define LC823450_IRQ_GPIO26         (LC823450_IRQ_NIRQS + 38)     /* 119: GPIO26 */
#define LC823450_IRQ_GPIO27         (LC823450_IRQ_NIRQS + 39)     /* 120: GPIO27 */
#define LC823450_IRQ_GPIO28         (LC823450_IRQ_NIRQS + 40)     /* 121: GPIO28 */
#define LC823450_IRQ_GPIO29         (LC823450_IRQ_NIRQS + 41)     /* 122: GPIO29 */
#define LC823450_IRQ_GPIO2A         (LC823450_IRQ_NIRQS + 42)     /* 123: GPIO2A */
#define LC823450_IRQ_GPIO2B         (LC823450_IRQ_NIRQS + 43)     /* 124: GPIO2B */
#define LC823450_IRQ_GPIO2C         (LC823450_IRQ_NIRQS + 44)     /* 125: GPIO2C */
#define LC823450_IRQ_GPIO2D         (LC823450_IRQ_NIRQS + 45)     /* 126: GPIO2D */
#define LC823450_IRQ_GPIO2E         (LC823450_IRQ_NIRQS + 46)     /* 127: GPIO2E */
#define LC823450_IRQ_GPIO2F         (LC823450_IRQ_NIRQS + 47)     /* 128: GPIO2F */
#define LC823450_IRQ_GPIO30         (LC823450_IRQ_NIRQS + 48)     /* 129: GPIO30 */
#define LC823450_IRQ_GPIO31         (LC823450_IRQ_NIRQS + 49)     /* 130: GPIO31 */
#define LC823450_IRQ_GPIO32         (LC823450_IRQ_NIRQS + 50)     /* 131: GPIO32 */
#define LC823450_IRQ_GPIO33         (LC823450_IRQ_NIRQS + 51)     /* 132: GPIO33 */
#define LC823450_IRQ_GPIO34         (LC823450_IRQ_NIRQS + 52)     /* 133: GPIO34 */
#define LC823450_IRQ_GPIO35         (LC823450_IRQ_NIRQS + 53)     /* 134: GPIO35 */
#define LC823450_IRQ_GPIO36         (LC823450_IRQ_NIRQS + 54)     /* 135: GPIO36 */
#define LC823450_IRQ_GPIO37         (LC823450_IRQ_NIRQS + 55)     /* 136: GPIO37 */
#define LC823450_IRQ_GPIO38         (LC823450_IRQ_NIRQS + 56)     /* 137: GPIO38 */
#define LC823450_IRQ_GPIO39         (LC823450_IRQ_NIRQS + 57)     /* 138: GPIO39 */
#define LC823450_IRQ_GPIO3A         (LC823450_IRQ_NIRQS + 58)     /* 139: GPIO3A */
#define LC823450_IRQ_GPIO3B         (LC823450_IRQ_NIRQS + 59)     /* 140: GPIO3B */
#define LC823450_IRQ_GPIO3C         (LC823450_IRQ_NIRQS + 60)     /* 141: GPIO3C */
#define LC823450_IRQ_GPIO3D         (LC823450_IRQ_NIRQS + 61)     /* 142: GPIO3D */
#define LC823450_IRQ_GPIO3E         (LC823450_IRQ_NIRQS + 62)     /* 143: GPIO3E */
#define LC823450_IRQ_GPIO3F         (LC823450_IRQ_NIRQS + 63)     /* 144: GPIO3F */
#define LC823450_IRQ_GPIO40         (LC823450_IRQ_NIRQS + 64)     /* 145: GPIO40 */
#define LC823450_IRQ_GPIO41         (LC823450_IRQ_NIRQS + 65)     /* 146: GPIO41 */
#define LC823450_IRQ_GPIO42         (LC823450_IRQ_NIRQS + 66)     /* 147: GPIO42 */
#define LC823450_IRQ_GPIO43         (LC823450_IRQ_NIRQS + 67)     /* 148: GPIO43 */
#define LC823450_IRQ_GPIO44         (LC823450_IRQ_NIRQS + 68)     /* 149: GPIO44 */
#define LC823450_IRQ_GPIO45         (LC823450_IRQ_NIRQS + 69)     /* 150: GPIO45 */
#define LC823450_IRQ_GPIO46         (LC823450_IRQ_NIRQS + 70)     /* 151: GPIO46 */
#define LC823450_IRQ_GPIO47         (LC823450_IRQ_NIRQS + 71)     /* 152: GPIO47 */
#define LC823450_IRQ_GPIO48         (LC823450_IRQ_NIRQS + 72)     /* 153: GPIO48 */
#define LC823450_IRQ_GPIO49         (LC823450_IRQ_NIRQS + 73)     /* 154: GPIO49 */
#define LC823450_IRQ_GPIO4A         (LC823450_IRQ_NIRQS + 74)     /* 155: GPIO4A */
#define LC823450_IRQ_GPIO4B         (LC823450_IRQ_NIRQS + 75)     /* 156: GPIO4B */
#define LC823450_IRQ_GPIO4C         (LC823450_IRQ_NIRQS + 76)     /* 157: GPIO4C */
#define LC823450_IRQ_GPIO4D         (LC823450_IRQ_NIRQS + 77)     /* 158: GPIO4D */
#define LC823450_IRQ_GPIO4E         (LC823450_IRQ_NIRQS + 78)     /* 159: GPIO4E */
#define LC823450_IRQ_GPIO4F         (LC823450_IRQ_NIRQS + 79)     /* 160: GPIO4F */
#define LC823450_IRQ_GPIO50         (LC823450_IRQ_NIRQS + 80)     /* 161: GPIO50 */
#define LC823450_IRQ_GPIO51         (LC823450_IRQ_NIRQS + 81)     /* 162: GPIO51 */
#define LC823450_IRQ_GPIO52         (LC823450_IRQ_NIRQS + 82)     /* 163: GPIO52 */
#define LC823450_IRQ_GPIO53         (LC823450_IRQ_NIRQS + 83)     /* 164: GPIO53 */
#define LC823450_IRQ_GPIO54         (LC823450_IRQ_NIRQS + 84)     /* 165: GPIO54 */
#define LC823450_IRQ_GPIO55         (LC823450_IRQ_NIRQS + 85)     /* 166: GPIO55 */
#define LC823450_IRQ_GPIO56         (LC823450_IRQ_NIRQS + 86)     /* 167: GPIO56 */
#define LC823450_IRQ_GPIO57         (LC823450_IRQ_NIRQS + 87)     /* 168: GPIO57 */
#define LC823450_IRQ_GPIO58         (LC823450_IRQ_NIRQS + 88)     /* 169: GPIO58 */
#define LC823450_IRQ_GPIO59         (LC823450_IRQ_NIRQS + 89)     /* 170: GPIO59 */

#define LC823450_IRQ_NGPIOIRQS      (90)

#ifdef CONFIG_LC823450_VIRQ
#define LC823450_IRQ_VIRTUAL        (LC823450_IRQ_NIRQS + LC823450_IRQ_NGPIOIRQS)
#define LC823450_IRQ_V00            (LC823450_IRQ_VIRTUAL + 0)    /* 171: V00 */
#define LC823450_IRQ_NVIRTUALIRQS   (1)
#else /* CONFIG_LC823450_VIRQ */
#define LC823450_IRQ_NVIRTUALIRQS   (0)
#endif /* CONFIG_LC823450_VIRQ */

#define NR_IRQS                     (LC823450_IRQ_NIRQS + LC823450_IRQ_NGPIOIRQS + \
                                     LC823450_IRQ_NVIRTUALIRQS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
enum lc823450_srctype_e
{
  SRCTYPE_FALLING = 0,  /* Falling edge */
  SRCTYPE_LOW,          /* Low level */
  SRCTYPE_RISING,       /* Rigsing edge */
  SRCTYPE_HIGH,         /* High level */
  SRCTYPE_MAX,
};
#ifdef CONFIG_LC823450_VIRQ
struct lc823450_irq_ops
{
  void (*enable)(int irq);
  void (*disable)(int irq);
  int (*srctype)(int irq, enum lc823450_srctype_e type);
};
#endif /* CONFIG_LC823450_VIRQ */

#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

EXTERN int lc823450_irq_srctype(int irq, enum lc823450_srctype_e srctype);
#ifdef CONFIG_LC823450_VIRQ
EXTERN int lc823450_irq_register(int irq, struct lc823450_irq_ops *ops);
#endif /* CONFIG_LC823450_VIRQ */

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_LC823450_IRQ_H */
