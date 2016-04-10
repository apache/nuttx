/****************************************************************************
 * arch/arm/src/sama5/chip/sam_ohci.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_OHCI_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_OHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/ohci.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

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
 * Public Functions
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_OHCI_H */
