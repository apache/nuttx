/****************************************************************************
 * arch/mips/src/jz4780/jz4780_ohci.h
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

#ifndef __ARCH_MIPS_SRC_JZ4780_JZ4780_OHCI_H
#define __ARCH_MIPS_SRC_JZ4780_JZ4780_OHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/ohci.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define JZ_UHPOHCI_VSECTION   0xb34a0000 /* JZ4780 OHCI */

/* The JZ4780 supports 1 root hub port */

#define JZ_OHCI_NRHPORT        1

/* Register offsets *********************************************************/

/* See nuttx/usb/ohci.h */

/* Register addresses *******************************************************/

#define JZ_USBHOST_HCIREV      (JZ_UHPOHCI_VSECTION + OHCI_HCIREV_OFFSET)
#define JZ_USBHOST_CTRL        (JZ_UHPOHCI_VSECTION + OHCI_CTRL_OFFSET)
#define JZ_USBHOST_CMDST       (JZ_UHPOHCI_VSECTION + OHCI_CMDST_OFFSET)
#define JZ_USBHOST_INTST       (JZ_UHPOHCI_VSECTION + OHCI_INTST_OFFSET)
#define JZ_USBHOST_INTEN       (JZ_UHPOHCI_VSECTION + OHCI_INTEN_OFFSET)
#define JZ_USBHOST_INTDIS      (JZ_UHPOHCI_VSECTION + OHCI_INTDIS_OFFSET)

/* Memory pointers (section 7.2) */

#define JZ_USBHOST_HCCA        (JZ_UHPOHCI_VSECTION + OHCI_HCCA_OFFSET)
#define JZ_USBHOST_PERED       (JZ_UHPOHCI_VSECTION + OHCI_PERED_OFFSET)
#define JZ_USBHOST_CTRLHEADED  (JZ_UHPOHCI_VSECTION + OHCI_CTRLHEADED_OFFSET)
#define JZ_USBHOST_CTRLED      (JZ_UHPOHCI_VSECTION + OHCI_CTRLED_OFFSET)
#define JZ_USBHOST_BULKHEADED  (JZ_UHPOHCI_VSECTION + OHCI_BULKHEADED_OFFSET)
#define JZ_USBHOST_BULKED      (JZ_UHPOHCI_VSECTION + OHCI_BULKED_OFFSET)
#define JZ_USBHOST_DONEHEAD    (JZ_UHPOHCI_VSECTION + OHCI_DONEHEAD_OFFSET)

/* Frame counters (section 7.3) */

#define JZ_USBHOST_FMINT       (JZ_UHPOHCI_VSECTION + OHCI_FMINT_OFFSET)
#define JZ_USBHOST_FMREM       (JZ_UHPOHCI_VSECTION + OHCI_FMREM_OFFSET)
#define JZ_USBHOST_FMNO        (JZ_UHPOHCI_VSECTION + OHCI_FMNO_OFFSET)
#define JZ_USBHOST_PERSTART    (JZ_UHPOHCI_VSECTION + OHCI_PERSTART_OFFSET)

/* Root hub ports (section 7.4) */

#define JZ_USBHOST_LSTHRES     (JZ_UHPOHCI_VSECTION + OHCI_LSTHRES_OFFSET)
#define JZ_USBHOST_RHDESCA     (JZ_UHPOHCI_VSECTION + OHCI_RHDESCA_OFFSET)
#define JZ_USBHOST_RHDESCB     (JZ_UHPOHCI_VSECTION + OHCI_RHDESCB_OFFSET)
#define JZ_USBHOST_RHSTATUS    (JZ_UHPOHCI_VSECTION + OHCI_RHSTATUS_OFFSET)

#define JZ_USBHOST_RHPORTST(n) (JZ_UHPOHCI_VSECTION + OHCI_RHPORTST_OFFSET(n))
#define JZ_USBHOST_RHPORTST1   (JZ_UHPOHCI_VSECTION + OHCI_RHPORTST1_OFFSET)
#define JZ_USBHOST_RHPORTST2   (JZ_UHPOHCI_VSECTION + OHCI_RHPORTST2_OFFSET)
#define JZ_USBHOST_RHPORTST3   (JZ_UHPOHCI_VSECTION + OHCI_RHPORTST3_OFFSET)

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

#endif /* __ARCH_MIPS_SRC_JZ4780_JZ4780_OHCI_H */
