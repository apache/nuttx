/****************************************************************************
 * include/nuttx/usb/storage.h
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

#ifndef __INCLUDE_NUTTX_USB_STORAGE_H
#define __INCLUDE_NUTTX_USB_STORAGE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Mass storage requests */

#define USBMSC_TYPE_SETUPIN  (USB_DIR_IN | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE)
#define USBMSC_TYPE_SETUPOUT (USB_DIR_OUT | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE)

#define USBMSC_REQ_MSRESET           (0xff) /* Reset mass storage device and interface */
#define USBMSC_REQ_GETMAXLUN         (0xfe) /* Return number LUNs supported */

/* Mass storage subclass codes */

#define USBMSC_SUBCLASS_RBC          (0x01) /* Reduced block commands (e.g., flash devices) */
#define USBMSC_SUBCLASS_SFF1         (0x02) /* SFF-8020i/MMC-2 (ATAPI) (e.g., C/DVD) */
#define USBMSC_SUBCLASS_QIC          (0x03) /* QIC-157 (e.g., tape device) */
#define USBMSC_SUBCLASS_UFI          (0x04) /* e.g. floppy device */
#define USBMSC_SUBCLASS_SFF2         (0x05) /* SFF-8070i (e.g. floppy disk) */
#define USBMSC_SUBCLASS_SCSI         (0x06) /* SCSI transparent */

/* Mass storage transport protocols */

#define USBMSC_PROTO_CBI0            (0x00) /* CBI transport with command completion interrupt */
#define USBMSC_PROTO_CBI1            (0x01) /* CBI transport without command completion interrupt */
#define USBMSC_PROTO_BULKONLY        (0x50) /* Bulk only transport */

/* Common Block Wrapper (CBW) */

#define USBMSC_CBW_SIZEOF            (31)
#define USBMSC_CBW_SIGNATURE         (0x43425355)   /*  Little endian USBC */
#define USBMSC_CBWFLAG_IN            (0x80)         /* Bit 7=1: Direction = IN */

#define USBMSC_MAXCDBLEN             (16)           /* Max length of SCSI Command Data Block */

/* Command Status Wrapper (CSW) */

#define USBMSC_CSW_SIZEOF            (13)
#define USBMSC_CSW_SIGNATURE         (0x53425355)   /* Little endian 'USBS' */
#define USBMSC_CSWSTATUS_PASS        (0)
#define USBMSC_CSWSTATUS_FAIL        (1)
#define USBMSC_CSWSTATUS_PHASEERROR  (2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Command Block Wrapper (CBW) */

struct usbmsc_cbw_s
{
  uint8_t signature[4];           /* 'USBC' = 0x43425355 */
  uint8_t tag[4];                 /* Depends on command id */
  uint8_t datlen[4];              /* Number of bytes that host expects to transfer */
  uint8_t flags;                  /* Bit 7: Direction=IN (other obsolete or reserved) */
  uint8_t lun;                    /* LUN (normally 0) */
  uint8_t cdblen;                 /* len of cdb[] */
  uint8_t cdb[USBMSC_MAXCDBLEN];  /* Command Data Block */
};

/* Command Status Wrapper (CSW) */

struct usbmsc_csw_s
{
  uint8_t signature[4];           /* 'USBS' = 0x53425355 */
  uint8_t tag[4];                 /* Same tag as original command */
  uint8_t residue[4];             /* Amount not transferred */
  uint8_t status;                 /* Status of transfer */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_USB_STORAGE_H */
