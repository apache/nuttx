/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_ohci.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_OHCI_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_OHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/ohci.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SAMA5 supports 3 root hub ports */

#define SAM_OHCI_NRHPORT        3

/* Register offsets *********************************************************/

/* See nuttx/usb/ohci.h */

/* Register addresses *******************************************************/

#define SAM_USBHOST_HCIREV      (SAM_UHPOHCI_VSECTION+OHCI_HCIREV_OFFSET)
#define SAM_USBHOST_CTRL        (SAM_UHPOHCI_VSECTION+OHCI_CTRL_OFFSET)
#define SAM_USBHOST_CMDST       (SAM_UHPOHCI_VSECTION+OHCI_CMDST_OFFSET)
#define SAM_USBHOST_INTST       (SAM_UHPOHCI_VSECTION+OHCI_INTST_OFFSET)
#define SAM_USBHOST_INTEN       (SAM_UHPOHCI_VSECTION+OHCI_INTEN_OFFSET)
#define SAM_USBHOST_INTDIS      (SAM_UHPOHCI_VSECTION+OHCI_INTDIS_OFFSET)

/* Memory pointers (section 7.2) */

#define SAM_USBHOST_HCCA        (SAM_UHPOHCI_VSECTION+OHCI_HCCA_OFFSET)
#define SAM_USBHOST_PERED       (SAM_UHPOHCI_VSECTION+OHCI_PERED_OFFSET)
#define SAM_USBHOST_CTRLHEADED  (SAM_UHPOHCI_VSECTION+OHCI_CTRLHEADED_OFFSET)
#define SAM_USBHOST_CTRLED      (SAM_UHPOHCI_VSECTION+OHCI_CTRLED_OFFSET)
#define SAM_USBHOST_BULKHEADED  (SAM_UHPOHCI_VSECTION+OHCI_BULKHEADED_OFFSET)
#define SAM_USBHOST_BULKED      (SAM_UHPOHCI_VSECTION+OHCI_BULKED_OFFSET)
#define SAM_USBHOST_DONEHEAD    (SAM_UHPOHCI_VSECTION+OHCI_DONEHEAD_OFFSET)

/* Frame counters (section 7.3) */

#define SAM_USBHOST_FMINT       (SAM_UHPOHCI_VSECTION+OHCI_FMINT_OFFSET)
#define SAM_USBHOST_FMREM       (SAM_UHPOHCI_VSECTION+OHCI_FMREM_OFFSET)
#define SAM_USBHOST_FMNO        (SAM_UHPOHCI_VSECTION+OHCI_FMNO_OFFSET)
#define SAM_USBHOST_PERSTART    (SAM_UHPOHCI_VSECTION+OHCI_PERSTART_OFFSET)

/* Root hub ports (section 7.4) */

#define SAM_USBHOST_LSTHRES     (SAM_UHPOHCI_VSECTION+OHCI_LSTHRES_OFFSET)
#define SAM_USBHOST_RHDESCA     (SAM_UHPOHCI_VSECTION+OHCI_RHDESCA_OFFSET)
#define SAM_USBHOST_RHDESCB     (SAM_UHPOHCI_VSECTION+OHCI_RHDESCB_OFFSET)
#define SAM_USBHOST_RHSTATUS    (SAM_UHPOHCI_VSECTION+OHCI_RHSTATUS_OFFSET)

#define SAM_USBHOST_RHPORTST(n) (SAM_UHPOHCI_VSECTION+OHCI_RHPORTST_OFFSET(n))
#define SAM_USBHOST_RHPORTST1   (SAM_UHPOHCI_VSECTION+OHCI_RHPORTST1_OFFSET)
#define SAM_USBHOST_RHPORTST2   (SAM_UHPOHCI_VSECTION+OHCI_RHPORTST2_OFFSET)
#define SAM_USBHOST_RHPORTST3   (SAM_UHPOHCI_VSECTION+OHCI_RHPORTST3_OFFSET)

/* Register bit definitions *************************************************/

/* See include/nuttx/usb/ohci.h */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_OHCI_H */
