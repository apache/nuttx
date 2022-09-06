/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_pmu.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_PMU_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_PMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define GD32_PMU_CTL_OFFSET        0x0000            /* Power control register offset */
#define GD32_PMU_CS_OFFSET         0x0004            /* Power control and status register offset */

/* Register Addresses *******************************************************/

#define GD32_PMU_CTL               (GD32_PMU_BASE+GD32_PMU_CTL_OFFSET)       /* Power control register */
#define GD32_PMU_CS                (GD32_PMU_BASE+GD32_PMU_CS_OFFSET)        /* Power control and status register */

/* Register Bitfield Definitions ********************************************/

/* Power control register */

#define PMU_CTL_LDOLP              (1 << 0)          /* Bit 0: LDO low power mode */
#define PMU_CTL_STBMOD             (1 << 1)          /* Bit 1: Standby mode */
#define PMU_CTL_WURST              (1 << 2)          /* Bit 2: wakeup flag reset */
#define PMU_CTL_STBRST             (1 << 3)          /* Bit 3: Standby flag reset */
#define PMU_CTL_LVDEN              (1 << 4)          /* Bit 4: Low voltage detector enable */

#define PMU_CTL_LVDT_SHIFT         (5)               /* Bits 5-7: Low voltage detector threshold */
#define PMU_CTL_LVDT_MASK          (7 << PMU_CTL_LVDT_SHIFT)
#  define PMU_CTL_LVDT(n)          ((n) << PMU_CTL_LVDT_SHIFT)
#  define PMU_LVDT_0               PMU_CTL_LVDT(0)   /* Voltage threshold is 2.1V */
#  define PMU_LVDT_1               PMU_CTL_LVDT(1)   /* Voltage threshold is 2.3V */
#  define PMU_LVDT_2               PMU_CTL_LVDT(2)   /* Voltage threshold is 2.4V */
#  define PMU_LVDT_3               PMU_CTL_LVDT(3)   /* Voltage threshold is 2.6V */
#  define PMU_LVDT_4               PMU_CTL_LVDT(4)   /* Voltage threshold is 2.7V */
#  define PMU_LVDT_5               PMU_CTL_LVDT(5)   /* Voltage threshold is 2.9V */
#  define PMU_LVDT_6               PMU_CTL_LVDT(6)   /* Voltage threshold is 3.0V */
#  define PMU_LVDT_7               PMU_CTL_LVDT(7)   /* Voltage threshold is 3.1V */

#define PMU_CTL_BKPWEN             (1 << 8)          /* Bit 8: Backup domain write enable */
#define PMU_CTL_LDLP               (1 << 10)         /* Bit 10: Low-driver mode when use low power LDO */
#define PMU_CTL_LDNP               (1 << 11)         /* Bit 11: Low-driver mode when use normal power LDO */

#define PMU_CTL_LDOVS_SHIFT        (14)              /* Bit 14-15: LDO output voltage select */
#define PMU_CTL_LDOVS_MASK         (3 << PMU_CTL_LDOVS_SHIFT)
#  define PMU_CTL_LDOVS(n)         ((n) << PMU_CTL_LDOVS_SHIFT)
#  define PMU_LDOVS_LOW            PMU_CTL_LDOVS(1)  /* LDO output voltage low mode */
#  define PMU_LDOVS_MID            PMU_CTL_LDOVS(2)  /* LDO output voltage mid mode */
#  define PMU_LDOVS_HIGH           PMU_CTL_LDOVS(3)  /* LDO output voltage high mode */

#define PMU_CTL_HDEN               (1 << 16)         /* Bit 16: High-driver mode enable */
#define PMU_CTL_HDS                (1 << 17)         /* Bit 17: High-driver mode switch */

#define PMU_CTL_LDEN_SHIFT         (18)               /* Bit 18-19: LDO output voltage select */
#define PMU_CTL_LDEN_MASK          (3 << PMU_CTL_LDEN_SHIFT)
#  define PMU_CTL_LDEN(n)          ((n) << PMU_CTL_LDEN_SHIFT)
#  define PMU_LOWDRIVER_DISABLE    PMU_CTL_LDEN(0)   /* Low-driver mode disable in deep-sleep mode */
#  define PMU_LOWDRIVER_ENABLE     PMU_CTL_LDEN(3)   /* Low-driver mode enable in deep-sleep mode */

/* Power control and status register */

#define PMU_CS_WUF                 (1 << 0)          /* Bit 0:  Wakeup Flag */
#define PMU_CS_STBF                (1 << 1)          /* Bit 1:  Standby Flag */
#define PMU_CS_LVDF                (1 << 2)          /* Bit 2:  Low voltage detector status flag */
#define PMU_CS_BLDORF              (1 << 3)          /* Bit 3:  Backup SRAM LDO ready flag */
#define PMU_CS_WUPEN               (1 << 8)          /* Bit 8:  Wakeup pin enable */
#define PMU_CS_BLDOON              (1 << 9)          /* Bit 9:  Backup SRAM LDO on */
#define PMU_CS_LDOVSRF             (1 << 14)         /* Bit 14: LDO voltage select ready flag */
#define PMU_CS_HDRF                (1 << 16)         /* Bit 16: High-driver ready flag */
#define PMU_CS_HDSRF               (1 << 17)         /* Bit 17: High-driver switch ready flag */

#define PMU_CS_LDRF_SHIFT          (18)              /* Bit 18-19: Low-driver mode ready flag */
#define PMU_CS_LDRF_MASK           (3 << PMU_CS_LDRF_SHIFT)
#  define PMU_CS_LDRF(n)           ((n) << PMU_CS_LDRF_SHIFT)
#  define PMU_LDRF_NORMAL          PMU_CS_LDRF(0)    /* Normal driver in deep-sleep mode */
#  define PMU_LDRF_LOWDRIVER       PMU_CS_LDRF(3)    /* Low-driver mode in deep-sleep mode */

/* PMU low-driver mode when use low power LDO */
#define PMU_NORMALDR_LOWPWR        (0)               /* Normal driver when use low power LDO */
#define PMU_LOWDR_LOWPWR           PMU_CTL_LDLP      /* Low-driver mode enabled when LDEN is 11 and use low power LDO */

/* PMU low-driver mode when use normal power LDO */
#define PMU_NORMALDR_NORMALPWR     (0)               /* Normal driver when use normal power LDO */
#define PMU_LOWDR_NORMALPWR        PMU_CTL_LDNP      /* Low-driver mode enabled when LDEN is 11 and use normal power LDO */

/* PMU ldo definitions */
#define PMU_LDO_NORMAL             (0)               /* LDO normal work when PMU enter deep-sleep mode */
#define PMU_LDO_LOWPOWER           PMU_CTL_LDOLP     /* LDO work at low power status when PMU enter deep-sleep mode */

/* PMU flag reset definitions */
#define PMU_FLAG_RESET_WAKEUP      (0x00)            /* Wakeup flag reset */
#define PMU_FLAG_RESET_STANDBY     (0x01)            /* Standby flag reset */

/* PMU high-driver mode switch */
#define PMU_HIGHDR_SWITCH_NONE     (0)               /* No high-driver mode switch */
#define PMU_HIGHDR_SWITCH_EN       PMU_CTL_HDS       /* High-driver mode switch */

/* PMU command constants definitions */
#define WFI_CMD                    (0x00)            /* Use WFI command */
#define WFE_CMD                    (0x01)            /* Use WFE command */

/* PMU backup SRAM LDO on or off */
#define PMU_BLDOON_OFF             (0)               /* Backup SRAM LDO off */
#define PMU_BLDOON_ON              PMU_CS_BLDOON     /* The backup SRAM LDO on */

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_PMU_H */
