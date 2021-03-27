/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/hardware/lpc17_40_eeprom.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_EEPROM_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_EEPROM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/lpc17_40_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define LPC17_40_EEPROM_EECMD_OFFSET            0x0080 /* Command register */
#define LPC17_40_EEPROM_EEADDR_OFFSET           0x0084 /* Address register */
#define LPC17_40_EEPROM_EEWDATA_OFFSET          0x0088 /* Write Data register */
#define LPC17_40_EEPROM_EERDATA_OFFSET          0x008c /* Read Data register */
#define LPC17_40_EEPROM_EEWSTATE_OFFSET         0x0090 /* Wait state register */
#define LPC17_40_EEPROM_EECLKDIV_OFFSET         0x0094 /* Clock divider register */
#define LPC17_40_EEPROM_EEPWRDWN_OFFSET         0x0098 /* Power down register */

#define LPC17_40_EEPROM_INTSTAT_OFFSET          0x0fe0 /* Interrupt status */
#define LPC17_40_EEPROM_INTEN_OFFSET            0x0fe4 /* Interrupt enable */
#define LPC17_40_EEPROM_INTSTATCLR_OFFSET       0x0fe8 /* Interrupt status clear */
#define LPC17_40_EEPROM_INTENCLR_OFFSET         0x0fd8 /* Interrupt enable clear */
#define LPC17_40_EEPROM_INTENSET_OFFSET         0x0fdc /* Interrupt enable set */
#define LPC17_40_EEPROM_INTSTATSET_OFFSET       0x0fec /* Interrupt status set */

#define LPC17_40_EEPROM_EECMD                   (LPC17_40_EEPROM_BASE+LPC17_40_EEPROM_EECMD_OFFSET)
#define LPC17_40_EEPROM_EEADDR                  (LPC17_40_EEPROM_BASE+LPC17_40_EEPROM_EEADDR_OFFSET)
#define LPC17_40_EEPROM_EEWDATA                 (LPC17_40_EEPROM_BASE+LPC17_40_EEPROM_EEWDATA_OFFSET)
#define LPC17_40_EEPROM_EERDATA                 (LPC17_40_EEPROM_BASE+LPC17_40_EEPROM_EERDATA_OFFSET)
#define LPC17_40_EEPROM_EEWSTATE                (LPC17_40_EEPROM_BASE+LPC17_40_EEPROM_EEWSTATE_OFFSET)
#define LPC17_40_EEPROM_EECLKDIV                (LPC17_40_EEPROM_BASE+LPC17_40_EEPROM_EECLKDIV_OFFSET)
#define LPC17_40_EEPROM_EEPWRDWN                (LPC17_40_EEPROM_BASE+LPC17_40_EEPROM_EEPWRDWN_OFFSET)

#define LPC17_40_EEPROM_INTSTAT                 (LPC17_40_EEPROM_BASE+LPC17_40_EEPROM_INTSTAT_OFFSET)
#define LPC17_40_EEPROM_INTEN                   (LPC17_40_EEPROM_BASE+LPC17_40_EEPROM_INTEN_OFFSET)
#define LPC17_40_EEPROM_INTSTATCLR              (LPC17_40_EEPROM_BASE+LPC17_40_EEPROM_INTSTATCLR_OFFSET)
#define LPC17_40_EEPROM_INTENCLR                (LPC17_40_EEPROM_BASE+LPC17_40_EEPROM_INTENCLR_OFFSET)
#define LPC17_40_EEPROM_INTENSET                (LPC17_40_EEPROM_BASE+LPC17_40_EEPROM_INTENSET_OFFSET)
#define LPC17_40_EEPROM_INTSTATSET              (LPC17_40_EEPROM_BASE+LPC17_40_EEPROM_INTSTATSET_OFFSET)

/* EECMD - EEPROM Command Register */

#define EEPROM_CMD_SHIFT                        (0)       /* Bit 0-2: Command */
#define EEPROM_CMD_MASK                         (7 << EEPROM_CMD_SHIFT)
#   define EECMD_READ8                          (0)       /* 000: 8bit read */
#   define EECMD_READ16                         (1)       /* 001: 16bit read */
#   define EECMD_READ32                         (2)       /* 010: 32bit read */
#   define EECMD_WRITE8                         (3)       /* 011: 8bit write */
#   define EECMD_WRITE16                        (4)       /* 100: 16bit write */
#   define EECMD_WRITE32                        (5)       /* 101: 32bit write */
#   define EEMCD_ERASE                          (6)       /* 110: erase/program page */
                                                          /* 111: Reserved */
#define EEPROM_RDPREFETCH                       (1 << 3)  /* Bit 3: Read data prefetch bit */
                                                          /* Bits 4-31: Reserved */

/* EEADDR - EEPROM Address Register */

#define EEPROM_ADDR_SHIFT                       (0)       /* Bits 0-11: Address */
#define EEPROM_ADDR_MASK                        (0x7ff << EEPROM_EEADDR_ADDR_SHIFT)
                                                          /* Bits 12-31: Reserved */

/* EEPROM Read/Write Data Registers */

/* R/W registers has no bitfields, data read/write
 * must conforms to the expected sizes
 */

/* EEWSTATE - EEPROM Wait State Register */

#define EEPROM_WSTATE_PHASE3_SHIFT              (0)       /* Bits 0-7: Wait states 3 (minus 1 encoded) */
#define EEPROM_WSTATE_PHASE3_MASK               (0xff << EEWSTATE_PHASE3_SHIFT)
#define EEPROM_WSTATE_PHASE2_SHIFT              (8)       /* Bits 8-15: Wait states 2 (minus 1 encoded) */
#define EEPROM_WSTATE_PHASE2_MASK               (0xff << EEWSTATE_PHASE2_SHIFT)
#define EEPROM_WSTATE_PHASE1_SHIFT              (16)      /* Bits 16-23: Wait states 1 (minus 1 encoded) */
#define EEPROM_WSTATE_PHASE1_MASK               (0xff << EEWSTATE_PHASE1_SHIFT)
                                                          /* Bits 24-31: Reserved */

/* EECLKDIV - EEPROM Clock Divider Register */

#define EEPROM_CLKDIV_SHIFT                     (0)      /* Bits 0-15: Division factor (minus 1 encoded) */
#define EEPROM_CLKDIV_MASK                      (0xffff << EECLKDIV_CLKDIV_SHIFT)
                                                         /* Bits 16-31: Reserved */

/* EEPWRDWN - EEPROM Power Down Register */

#define EEPROM_PWRDWN                           (1)      /* Bit 0: Power down mode bit */
                                                         /* Bits 1-31: Reserved */

/* EEPROM Interrupt Registers ***********************************************/

/* INTEN - Interrupt Enable Register */

                                                  /* Bits 0-25: Reserved */
#define EEPROM_INTEN_RW_DONE                    (1 << 26) /* Bit 26: Read/Write finished interrupt bit */
                                                          /* Bit 27: Reserved */
#define EEPROM_INTEN_PROG_DONE                  (1 << 28) /* Bit 28: Program finished interrupt bit */

                                                  /* Bits 29-31: Reserved */

/* INTENCLR - Interrupt Enable Clear Register */

                                                  /* Bits 0-25: Reserved */
#define EEPROM_INTENCLR_RWCLR_EN                (1 << 26) /* Bit 26: Clear R/W interrupt enable bit */
                                                          /* Bits27: Reserved */
#define EEPROM_INTENCLR_PROG1CLR_EN             (1 << 28) /* Bit 28: Clear program interrupt bit */
                                                          /* Bits 29-31: Reserved */

/* INTENSET - Interrupt Enable Set Register */

                                                  /* Bits 0-25: Reserved */
#define EEPROM_INTENSET_RWSET_EN                (1 << 26) /* Bit 26: Set Read/Write finished interrupt bit */
                                                          /* Bit 27: Reserved */
#define EEPROM_INTENSET_PROG1SET_EN             (1 << 28) /* Bit 28: Set program interrupt bit */
                                                          /* Bits 29-31: Reserved */

/* INTSTAT - Interrupt Status Register */

                                                  /* Bits 0-25: Reserved */
#define EEPROM_INTSTAT_RW_END                   (1 << 26) /* Bit 26: Read/Write done status bit */
                                                          /* Bit 27: Reserved */
#define EEPROM_INTSTAT_PROG1_END                (1 << 28) /* Bit 28: Program done status bit */
                                                          /* Bits 29-31: Reserved */

/* INTSTATCLR - Interrupt Status Clear Register */

                                                  /* Bits 0-25: Reserved */
#define EEPROM_INTSTATCLR_RW_CLR                (1 << 26) /* Bit 26: Set Read/Write finished interrupt bit */
                                                          /* Bit 27: Reserved */
#define EEPROM_INTSTATCLR_PROG_1CLR             (1 << 28) /* Bit 28: Set program interrupt bit */
                                                          /* Bits 29-31: Reserved */

/* INTSTATSET - Interrupt Status Set Register */

                                                  /* Bits 0-25: Reserved */
#define EEPROM_INTSTATSET_RW_SET                (1 << 26) /* Bit 26: Read/Write done status bit */
                                                          /* Bit 27: Reserved */
#define EEPROM_INTSTATSET_PROG1_SET             (1 << 28) /* Bit 28: Program done status bit */
                                                          /* Bits 29-31: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_EEPROM_H */
