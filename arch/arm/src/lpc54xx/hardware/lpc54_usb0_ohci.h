/****************************************************************************
 * arch/arm/src/lpc54xx/hardware/lpc54_usb0_ohci.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_USB0_OHCI_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_USB0_OHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/ohci.h>

#include "chip.h"
#include "hardware/lpc54_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

/* USB Host Controller (OHCI).  See include/nuttx/usb/ohci.h */

/* Additional, non-standard register offsets */

#define LPC54_OHCI_PORTMODE_OFFSET    0x005c /* Port mode register */

/* Register addresses *******************************************************/

/* USB Host Controller (OHCI) */

/* Control and status registers (section 7.1) */

#define LPC54_OHCI_HCIREV             (LPC54_FSUSBHOST_BASE + OHCI_HCIREV_OFFSET)
#define LPC54_OHCI_CTRL               (LPC54_FSUSBHOST_BASE + OHCI_CTRL_OFFSET)
#define LPC54_OHCI_CMDST              (LPC54_FSUSBHOST_BASE + OHCI_CMDST_OFFSET)
#define LPC54_OHCI_INTST              (LPC54_FSUSBHOST_BASE + OHCI_INTST_OFFSET)
#define LPC54_OHCI_INTEN              (LPC54_FSUSBHOST_BASE + OHCI_INTEN_OFFSET)
#define LPC54_OHCI_INTDIS             (LPC54_FSUSBHOST_BASE + OHCI_INTDIS_OFFSET)

/* Memory pointers (section 7.2) */

#define LPC54_OHCI_HCCA               (LPC54_FSUSBHOST_BASE + OHCI_HCCA_OFFSET)
#define LPC54_OHCI_PERED              (LPC54_FSUSBHOST_BASE + OHCI_PERED_OFFSET)
#define LPC54_OHCI_CTRLHEADED         (LPC54_FSUSBHOST_BASE + OHCI_CTRLHEADED_OFFSET)
#define LPC54_OHCI_CTRLED             (LPC54_FSUSBHOST_BASE + OHCI_CTRLED_OFFSET)
#define LPC54_OHCI_BULKHEADED         (LPC54_FSUSBHOST_BASE + OHCI_BULKHEADED_OFFSET)
#define LPC54_OHCI_BULKED             (LPC54_FSUSBHOST_BASE + OHCI_BULKED_OFFSET)
#define LPC54_OHCI_DONEHEAD           (LPC54_FSUSBHOST_BASE + OHCI_DONEHEAD_OFFSET)

/* Frame counters (section 7.3) */

#define LPC54_OHCI_FMINT              (LPC54_FSUSBHOST_BASE + OHCI_FMINT_OFFSET)
#define LPC54_OHCI_FMREM              (LPC54_FSUSBHOST_BASE + OHCI_FMREM_OFFSET)
#define LPC54_OHCI_FMNO               (LPC54_FSUSBHOST_BASE + OHCI_FMNO_OFFSET)
#define LPC54_OHCI_PERSTART           (LPC54_FSUSBHOST_BASE + OHCI_PERSTART_OFFSET)

/* Root hub ports (section 7.4) */

#define LPC54_OHCI_LSTHRES            (LPC54_FSUSBHOST_BASE + OHCI_LSTHRES_OFFSET)
#define LPC54_OHCI_RHDESCA            (LPC54_FSUSBHOST_BASE + OHCI_RHDESCA_OFFSET)
#define LPC54_OHCI_RHDESCB            (LPC54_FSUSBHOST_BASE + OHCI_RHDESCB_OFFSET)
#define LPC54_OHCI_RHSTATUS           (LPC54_FSUSBHOST_BASE + OHCI_RHSTATUS_OFFSET)
#define LPC54_OHCI_RHPORTST1          (LPC54_FSUSBHOST_BASE + OHCI_RHPORTST1_OFFSET)
#define LPC54_OHCI_RHPORTST2          (LPC54_FSUSBHOST_BASE + OHCI_RHPORTST2_OFFSET)

/* Non-standard Registers */

#define LPC54_OHCI_PORTMODE           (LPC54_FSUSBHOST_BASE + LPC54_OHCI_PORTMODE_OFFSET)

/* Register bit definitions *************************************************/

/* USB Host Controller (OHCI). See include/nuttx/usb/ohci.h */

/* Port mode register */

#define OHCI_PORTMODE_ID              (1 << 0)  /* Bit 0:  Port ID pin value */
                                                /* Bits 1-7: Reserved */
#define OHCI_PORTMODE_IDEN            (1 << 8)  /* Bit 8:  Port ID pull-up enable */
                                                /* Bits 9-15: Reserved */
#define OHCI_PORTMODE_DEVENABLE       (1 << 16) /* Bit 16: Device mode enable */
                                                /* Bits 17-31: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_USB0_OHCI_H */
