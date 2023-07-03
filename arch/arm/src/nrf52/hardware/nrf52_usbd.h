/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_usbd.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_USBD_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_USBD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "hardware/nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NRF52_USBD_INOUT_NENDPOINTS (7)

/* Register offsets *********************************************************/

#define NRF52_USBD_TASKS_STARTEPIN_OFFSET(n)  (0x0004 + (0x04 * n))
#define NRF52_USBD_TASKS_STARTISOIN_OFFSET    (0x0024)
#define NRF52_USBD_TASKS_STARTEPOUT_OFFSET(n) (0x0028 + (0x04 * n))
#define NRF52_USBD_TASKS_STARTISOOUT_OFFSET   (0x0048)
#define NRF52_USBD_TASKS_EP0RCVOUT_OFFSET     (0x004c)
#define NRF52_USBD_TASKS_EP0STATUS_OFFSET     (0x0050)
#define NRF52_USBD_TASKS_EP0STALL_OFFSET      (0x0054)
#define NRF52_USBD_TASKS_DPDMDRIVE_OFFSET     (0x0058)
#define NRF52_USBD_TASKS_PDPMNODRIVE_OFFSET   (0x005c)
#define NRF52_USBD_EVENTS_USBRESET_OFFSET     (0x0100)
#define NRF52_USBD_EVENTS_STARTED_OFFSET      (0x0104)
#define NRF52_USBD_EVENTS_ENDEPIN_OFFSET(n)   (0x0108 + (0x04 * n))
#define NRF52_USBD_EVENTS_EP0DATADONE_OFFSET  (0x0128)
#define NRF52_USBD_EVENTS_ENDISOIN_OFFSET     (0x012c)
#define NRF52_USBD_EVENTS_ENDEPOUT_OFFSET(n)  (0x0130 + (0x04 * n))
#define NRF52_USBD_EVENTS_ENDISOOUT_OFFSET    (0x0150)
#define NRF52_USBD_EVENTS_SOF_OFFSET          (0x0154)
#define NRF52_USBD_EVENTS_USBEVENT_OFFSET     (0x0158)
#define NRF52_USBD_EVENTS_EP0SETUP_OFFSET     (0x015c)
#define NRF52_USBD_EVENTS_EPDATA_OFFSET       (0x0160)
#define NRF52_USBD_SHORTS_OFFSET              (0x0200)
#define NRF52_USBD_INTEN_OFFSET               (0x0300)
#define NRF52_USBD_INTENSET_OFFSET            (0x0304)
#define NRF52_USBD_INTENCLR_OFFSET            (0x0308)
#define NRF52_USBD_EVENTCAUSE_OFFSET          (0x0400)
#define NRF52_USBD_HALTED_EPIN_OFFSET(n)      (0x0420 + (0x04 * n))
#define NRF52_USBD_HALTED_EPOUT_OFFSET(n)     (0x0444 + (0x04 * n))
#define NRF52_USBD_EPSTATUS_OFFSET            (0x0468)
#define NRF52_USBD_EPDATASTATUS_OFFSET        (0x046c)
#define NRF52_USBD_USBADDR_OFFSET             (0x0470)
#define NRF52_USBD_BMREQUESTTYPE_OFFSET       (0x0480)
#define NRF52_USBD_BREQUEST_OFFSET            (0x0484)
#define NRF52_USBD_WVALUEL_OFFSET             (0x0488)
#define NRF52_USBD_WVALUEH_OFFSET             (0x048c)
#define NRF52_USBD_WINDEXL_OFFSET             (0x0490)
#define NRF52_USBD_WINDEXH_OFFSET             (0x0494)
#define NRF52_USBD_WLENGTHL_OFFSET            (0x0498)
#define NRF52_USBD_WLENGTHH_OFFSET            (0x049c)
#define NRF52_USBD_SIZE_EPOUT_OFFSET(n)       (0x04A0 + (0x04 * n))
#define NRF52_USBD_SIZE_ISOOUT_OFFSET         (0x04c0)
#define NRF52_USBD_ENABLE_OFFSET              (0x0500)
#define NRF52_USBD_USBPULLUP_OFFSET           (0x0504)
#define NRF52_USBD_DPDMVALUE_OFFSET           (0x0508)
#define NRF52_USBD_DTOGGLE_OFFSET             (0x050C)
#define NRF52_USBD_EPINEN_OFFSET              (0x0510)
#define NRF52_USBD_EPOUTEN_OFFSET             (0x0514)
#define NRF52_USBD_EPSTALL_OFFSET             (0x0518)
#define NRF52_USBD_ISOSPLIT_OFFSET            (0x051c)
#define NRF52_USBD_FRAMECNTR_OFFSET           (0x0520)
#define NRF52_USBD_LOWPOWER_OFFSET            (0x052c)
#define NRF52_USBD_ISOINCONFIG_OFFSET         (0x0530)
#define NRF52_USBD_EPIN_PTR_OFFSET(n)         (0x0600 + (0x14 * n))
#define NRF52_USBD_EPIN_MAXCNT_OFFSET(n)      (0x0604 + (0x14 * n))
#define NRF52_USBD_EPIN_AMOUNT_OFFSET(n)      (0x0608 + (0x14 * n))
#define NRF52_USBD_ISOIN_PTR_OFFSET           (0x06a0)
#define NRF52_USBD_ISOIN_MAXCNT_OFFSET        (0x06a4)
#define NRF52_USBD_ISOIN_AMOUNT_OFFSET        (0x06a8)
#define NRF52_USBD_EPOUT_PTR_OFFSET(n)        (0x0700 + (0x14 * n))
#define NRF52_USBD_EPOUT_MAXCNT_OFFSET(n)     (0x0704 + (0x14 * n))
#define NRF52_USBD_EPOUT_AMOUNT_OFFSET(n)     (0x0708 + (0x14 * n))
#define NRF52_USBD_ISOOUT_PTR_OFFSET          (0x07a0)
#define NRF52_USBD_ISOOUT_MAXCNT_OFFSET       (0x07a4)
#define NRF52_USBD_ISOOUT_AMOUNT_OFFSET       (0x07a8)

/* Register addresses *******************************************************/

#define NRF52_USBD_TASKS_STARTEPIN(n)  (NRF52_USBD_BASE + NRF52_USBD_TASKS_STARTEPIN_OFFSET(n))
#define NRF52_USBD_TASKS_STARTISOIN    (NRF52_USBD_BASE + NRF52_USBD_TASKS_STARTISOIN_OFFSET)
#define NRF52_USBD_TASKS_STARTEPOUT(n) (NRF52_USBD_BASE + NRF52_USBD_TASKS_STARTEPOUT_OFFSET(n))
#define NRF52_USBD_TASKS_STARTISOOUT   (NRF52_USBD_BASE + NRF52_USBD_TASKS_STARTISOOUT_OFFSET)
#define NRF52_USBD_TASKS_EP0RCVOUT     (NRF52_USBD_BASE + NRF52_USBD_TASKS_EP0RCVOUT_OFFSET)
#define NRF52_USBD_TASKS_EP0STATUS     (NRF52_USBD_BASE + NRF52_USBD_TASKS_EP0STATUS_OFFSET)
#define NRF52_USBD_TASKS_EP0STALL      (NRF52_USBD_BASE + NRF52_USBD_TASKS_EP0STALL_OFFSET)
#define NRF52_USBD_TASKS_DPDMDRIVE     (NRF52_USBD_BASE + NRF52_USBD_TASKS_DPDMDRIVE_OFFSET)
#define NRF52_USBD_TASKS_PDPMNODRIVE   (NRF52_USBD_BASE + NRF52_USBD_TASKS_PDPMNODRIVE_OFFSET)
#define NRF52_USBD_EVENTS_USBRESET     (NRF52_USBD_BASE + NRF52_USBD_EVENTS_USBRESET_OFFSET)
#define NRF52_USBD_EVENTS_STARTED      (NRF52_USBD_BASE + NRF52_USBD_EVENTS_STARTED_OFFSET)
#define NRF52_USBD_EVENTS_ENDEPIN(n)   (NRF52_USBD_BASE + NRF52_USBD_EVENTS_ENDEPIN_OFFSET(n))
#define NRF52_USBD_EVENTS_EP0DATADONE  (NRF52_USBD_BASE + NRF52_USBD_EVENTS_EP0DATADONE_OFFSET)
#define NRF52_USBD_EVENTS_ENDISOIN     (NRF52_USBD_BASE + NRF52_USBD_EVENTS_ENDISOIN_OFFSEN)
#define NRF52_USBD_EVENTS_ENDEPOUT(n)  (NRF52_USBD_BASE + NRF52_USBD_EVENTS_ENDEPOUT_OFFSET(n))
#define NRF52_USBD_EVENTS_ENDISOOUT    (NRF52_USBD_BASE + NRF52_USBD_EVENTS_ENDISOOUT_OFFSET)
#define NRF52_USBD_EVENTS_SOF          (NRF52_USBD_BASE + NRF52_USBD_EVENTS_SOF_OFFSET)
#define NRF52_USBD_EVENTS_USBEVENT     (NRF52_USBD_BASE + NRF52_USBD_EVENTS_USBEVENT_OFFSET)
#define NRF52_USBD_EVENTS_EP0SETUP     (NRF52_USBD_BASE + NRF52_USBD_EVENTS_EP0SETUP_OFFSET)
#define NRF52_USBD_EVENTS_EPDATA       (NRF52_USBD_BASE + NRF52_USBD_EVENTS_EPDATA_OFFSET)
#define NRF52_USBD_SHORTS              (NRF52_USBD_BASE + NRF52_USBD_SHORTS_OFFSET)
#define NRF52_USBD_INTEN               (NRF52_USBD_BASE + NRF52_USBD_INTEN_OFFSET)
#define NRF52_USBD_INTENSET            (NRF52_USBD_BASE + NRF52_USBD_INTENSET_OFFSET)
#define NRF52_USBD_INTENCLR            (NRF52_USBD_BASE + NRF52_USBD_INTENCLR_OFFSET)
#define NRF52_USBD_EVENTCAUSE          (NRF52_USBD_BASE + NRF52_USBD_EVENTCAUSE_OFFSET)
#define NRF52_USBD_HALTED_EPIN(n)      (NRF52_USBD_BASE + NRF52_USBD_HALTED_EPIN_OFFSET(n))
#define NRF52_USBD_HALTED_EPOUT(n)     (NRF52_USBD_BASE + NRF52_USBD_HALTED_EPOUT_OFFSET(n))
#define NRF52_USBD_EPSTATUS            (NRF52_USBD_BASE + NRF52_USBD_EPSTATUS_OFFSET)
#define NRF52_USBD_EPDATASTATUS        (NRF52_USBD_BASE + NRF52_USBD_EPDATASTATUS_OFFSET)
#define NRF52_USBD_USBADDR             (NRF52_USBD_BASE + NRF52_USBD_USBADDR_OFFSET)
#define NRF52_USBD_BMREQUESTTYPE       (NRF52_USBD_BASE + NRF52_USBD_BMREQUESTTYPE_OFFSET)
#define NRF52_USBD_BREQUEST            (NRF52_USBD_BASE + NRF52_USBD_BREQUEST_OFFSET)
#define NRF52_USBD_WVALUEL             (NRF52_USBD_BASE + NRF52_USBD_WVALUEL_OFFSET)
#define NRF52_USBD_WVALUEH             (NRF52_USBD_BASE + NRF52_USBD_WVALUEH_OFFSET)
#define NRF52_USBD_WINDEXL             (NRF52_USBD_BASE + NRF52_USBD_WINDEXL_OFFSET)
#define NRF52_USBD_WINDEXH             (NRF52_USBD_BASE + NRF52_USBD_WINDEXH_OFFSET)
#define NRF52_USBD_WLENGTHL            (NRF52_USBD_BASE + NRF52_USBD_WLENGTHL_OFFSET)
#define NRF52_USBD_WLENGTHH            (NRF52_USBD_BASE + NRF52_USBD_WLENGTHH_OFFSET)
#define NRF52_USBD_SIZE_EPOUT(n)       (NRF52_USBD_BASE + NRF52_USBD_SIZE_EPOUT_OFFSET(n))
#define NRF52_USBD_SIZE_ISOOUT         (NRF52_USBD_BASE + NRF52_USBD_SIZE_ISOOUT_OFFSET)
#define NRF52_USBD_ENABLE              (NRF52_USBD_BASE + NRF52_USBD_ENABLE_OFFSET)
#define NRF52_USBD_USBPULLUP           (NRF52_USBD_BASE + NRF52_USBD_USBPULLUP_OFFSET)
#define NRF52_USBD_DPDMVALUE           (NRF52_USBD_BASE + NRF52_USBD_DPDMVALUE_OFFSET)
#define NRF52_USBD_DTOGGLE             (NRF52_USBD_BASE + NRF52_USBD_DTOGGLE_OFFSET)
#define NRF52_USBD_EPINEN              (NRF52_USBD_BASE + NRF52_USBD_EPINEN_OFFSET)
#define NRF52_USBD_EPOUTEN             (NRF52_USBD_BASE + NRF52_USBD_EPOUTEN_OFFSET)
#define NRF52_USBD_EPSTALL             (NRF52_USBD_BASE + NRF52_USBD_EPSTALL_OFFSET)
#define NRF52_USBD_ISOSPLIT            (NRF52_USBD_BASE + NRF52_USBD_ISOSPLIT_OFFSET)
#define NRF52_USBD_FRAMECNTR           (NRF52_USBD_BASE + NRF52_USBD_FRAMECNTR_OFFSET)
#define NRF52_USBD_LOWPOWER            (NRF52_USBD_BASE + NRF52_USBD_LOWPOWER_OFFSET)
#define NRF52_USBD_ISOINCONFIG         (NRF52_USBD_BASE + NRF52_USBD_ISOINCONFIG_OFFSET)
#define NRF52_USBD_EPIN_PTR(n)         (NRF52_USBD_BASE + NRF52_USBD_EPIN_PTR_OFFSET(n))
#define NRF52_USBD_EPIN_MAXCNT(n)      (NRF52_USBD_BASE + NRF52_USBD_EPIN_MAXCNT_OFFSET(n))
#define NRF52_USBD_EPIN_AMOUNT(n)      (NRF52_USBD_BASE + NRF52_USBD_EPIN_AMOUNT_OFFSET(n))
#define NRF52_USBD_ISOIN_PTR           (NRF52_USBD_BASE + NRF52_USBD_ISOIN_PTR_OFFSET)
#define NRF52_USBD_ISOIN_MAXCNT        (NRF52_USBD_BASE + NRF52_USBD_ISOIN_MAXCNT_OFFSET)
#define NRF52_USBD_ISOIN_AMOUNT        (NRF52_USBD_BASE + NRF52_USBD_ISOIN_AMOUNT_OFFSET)
#define NRF52_USBD_EPOUT_PTR(n)        (NRF52_USBD_BASE + NRF52_USBD_EPOUT_PTR_OFFSET(n))
#define NRF52_USBD_EPOUT_MAXCNT(n)     (NRF52_USBD_BASE + NRF52_USBD_EPOUT_MAXCNT_OFFSET(n))
#define NRF52_USBD_EPOUT_AMOUNT(n)     (NRF52_USBD_BASE + NRF52_USBD_EPOUT_AMOUNT_OFFSET(n))
#define NRF52_USBD_ISOOUT_PTR          (NRF52_USBD_BASE + NRF52_USBD_ISOOUT_PTR_OFFSET)
#define NRF52_USBD_ISOOUT_MAXCNT       (NRF52_USBD_BASE + NRF52_USBD_ISOOUT_MAXCNT_OFFSET)
#define NRF52_USBD_ISOOUT_AMOUNT       (NRF52_USBD_BASE + NRF52_USBD_ISOOUT_AMOUNT_OFFSET)

/* Register bit definitions *************************************************/

/* USBD tasks */

#define USBD_TASKS_TRIGGER                 (1)

/* USBD events */

#define USBD_EVENT_GENERATED               (1)

/* USBD shorts */

#define USBD_SHORTS_EP0DATADONE_STARTEPIN0  (1 << 0) /* Shortcut EP0DATADONE->TARTEPIN[0] */
#define USBD_SHORTS_EP0DATADONE_STARTEPOUT0 (1 << 1) /* Shortcut EP0DATADONE->STARTEPOUT[0] */
#define USBD_SHORTS_EP0DATADONE_EP0STATUS   (1 << 3) /* Shortcut EP0DATADONE->EP0STATUS */
#define USBD_SHORTS_ENDEPOUT0_EP0STATUS     (1 << 4) /* Shortcut ENDEPOUT0->EP0STATUS */
#define USBD_SHORTS_ENDEPOUT0_EP0RCVOUT     (1 << 5) /* Shortcut ENDEPOUT[0]->EP0RCVOUT */

/* USBD interrupts */

#define USBD_INT_USBRESET                   (1 << 0)        /* Enable/disable interrupt USBRESET */
#define USBD_INT_STARTED                    (1 << 1)        /* Enable/disable interrupt STARTED */
#define USBD_INT_ENDEPIN(n)                 (1 << (2 + n))  /* Enable/disable interrupt ENDEPIN[i] */
#define USBD_INT_EP0DATADONE                (1 << 10)       /* Enable/disable interrupt EP0DATADONE */
#define USBD_INT_ENDISOIN                   (1 << 11)       /* Enable/disable interrupt ENDISOIN */
#define USBD_INT_ENDEPOUT(n)                (1 << (12 + n)) /* Enable/disable interrupt ENDEPOUT[i] */
#define USBD_INT_ENDISOOUT                  (1 << 20)       /* Enable/disable interrupt ENDISOOUT */
#define USBD_INT_SOF                        (1 << 21)       /* Enable/disable interrupt SOF */
#define USBD_INT_USBEVENT                   (1 << 22)       /* Enable/disable interrupt USBEVENT */
#define USBD_INT_EP0SETUP                   (1 << 23)       /* Enable/disable interrupt EP0SETUP */
#define USBD_INT_EPDATA                     (1 << 24)       /* Enable/disable interrupt EPDATA */
#define USBD_INT_ALL                        (0x1fffffff)
#define USBD_INT_ALL_NUM                    (25)

/* EVENTCAUSE */

#define USBD_EVENTCAUSE_ISOOUTCRC           (1 << 0)        /* CRC error */
#define USBD_EVENTCAUSE_SUSPEND             (1 << 8)        /* Device suspended */
#define USBD_EVENTCAUSE_RESUME              (1 << 9)        /* Device resumed */
#define USBD_EVENTCAUSE_USBWUALLOWED        (1 << 10)       /* USB MAC woken up */
#define USBD_EVENTCAUSE_READY               (1 << 11)       /* USB device is ready */

/* HALTED.EPIN[n] and HALTED.EPOUT[n] */

#define USBD_HALTED_GETSTATUS               (1 << 0)        /* Endpoint halted status */

/* EPSTATUS */

#define USBD_EPSTATUS_EPIN(n)               (1 << (0 + n))  /* IN endpoint EasyDMA captured */
#define USBD_EPSTATUS_EPOUT(n)              (1 << (16 + n)) /* OUT endpoint EasyDMA captured */

/* EPDATASTATUS */

#define USBD_EPDATASTATUS_EPIN(n)           (1 << (0 + n))  /* IN endpoint ACK */
#define USBD_EPDATASTATUS_EPOUT(n)          (1 << (16 + n)) /* OUT endpoint ACK */

/* USBADDR */

#define USBD_USBADDR_MASK                    (0x7f)         /* Device USB address */

/* BMREQUESTTYPE */

#define USBD_BMREQUESTTYPE_RECIP_SHIFT       (0)            /* Data transfer type */
#define USBD_BMREQUESTTYPE_RECIP_MASK        (0x1f << USBD_BMREQUESTTYPE_RECIP_SHIFT)
#  define USBD_BMREQUESTTYPE_RECIP_DEVICE    (0 << USBD_BMREQUESTTYPE_RECIP_SHIFT)
#  define USBD_BMREQUESTTYPE_RECIP_INTERFACE (1 << USBD_BMREQUESTTYPE_RECIP_SHIFT)
#  define USBD_BMREQUESTTYPE_RECIP_ENDPOINT  (2 << USBD_BMREQUESTTYPE_RECIP_SHIFT)
#  define USBD_BMREQUESTTYPE_RECIP_OTHER     (3 << USBD_BMREQUESTTYPE_RECIP_SHIFT)
#define USBD_BMREQUESTTYPE_TYPE_SHIFT        (5)            /* Data transfer type */
#define USBD_BMREQUESTTYPE_TYPE_MASK         (0x3 << USBD_BMREQUESTTYPE_TYPE_SHIFT)
#  define USBD_BMREQUESTTYPE_TYPE_STANDARD   (0 << USBD_BMREQUESTTYPE_TYPE_SHIFT)
#  define USBD_BMREQUESTTYPE_TYPE_CLASS      (1 << USBD_BMREQUESTTYPE_TYPE_SHIFT)
#  define USBD_BMREQUESTTYPE_TYPE_VENDOR     (2 << USBD_BMREQUESTTYPE_TYPE_SHIFT)
#define USBD_BMREQUESTTYPE_DIR_HOST2DEV      (0 << 7)       /* Host-to-device */
#define USBD_BMREQUESTTYPE_DIR_DEV2HOST      (1 << 7)       /* Device-to-host */

/* BREQUEST */

#define USBD_BREQUEST_MASK                   (0xff)         /* SETUP data, byte 1, bRequest */
#define USBD_BREQUEST_STD_GET_STATUS         (0)
#define USBD_BREQUEST_STD_CLEAR_FEATURE      (1)
#define USBD_BREQUEST_STD_SET_FEATURE        (3)
#define USBD_BREQUEST_STD_SET_ADDRESS        (5)
#define USBD_BREQUEST_STD_GET_DESCRIPTOR     (6)
#define USBD_BREQUEST_STD_SET_DESCRIPTOR     (7)
#define USBD_BREQUEST_STD_GET_CONFIGURATION  (8)
#define USBD_BREQUEST_STD_SET_CONFIGURATION  (9)
#define USBD_BREQUEST_STD_GET_INTERFACE      (10)
#define USBD_BREQUEST_STD_SET_INTERFACE      (11)
#define USBD_BREQUEST_STD_SYNCH_FRAME        (12)

/* WVALUEL */

#define USBD_WVALUEL_MASK                    (0xff)         /* SETUP data, byte 2, LSB of wValue */

/* WVALUEH */

#define USBD_WVALUEH_MASK                    (0xff)         /* SETUP data, byte 3, MSB of wValue */

/* WINDEXL */

#define USBD_WINDEXL_MASK                    (0xff)         /* SETUP data, byte 4, LSB of wIndex */

/* WINDEXH */

#define USBD_WINDEXH_MASK                    (0xff)         /* SETUP data, byte 5, MSB of wIndex */

/* WLENGTHL */

#define USBD_WLENGTHL_MASK                   (0xff)         /* SETUP data, byte 6, LSB of wLength */

/* WLENGTHH */

#define USBD_WLENGTHH_MASK                   (0xff)         /* SETUP data, byte 7, MSB of wLength */

/* SIZE.EPOUT[n] */

#define USBD_SIZE_EPOUT_MASK                 (0x7f)         /* EP OUT last data size */

/* SIZE.ISOOUT */

#define USBD_SIZE_ISOOUT_MASK                (0x3ff)        /* ISO OUT last data size */

/* ENABLE */

#define USBD_ENABLE_DISABLE                  (0)            /* USB peripheral is disabled */
#define USBD_ENABLE_ENABLE                   (1)            /* USB peripheral is enabled */

/* USBPULLUP */

#define USBD_USBPULLUP_DISABLE               (0)            /* Pull-up is disconnected */
#define USBD_USBPULLUP_ENABLE                (1)            /* Pull-up is connected to D+ */

/* DPDMVALUE */

#define USBD_DPDMVALUE_STATE_SHIFT           (0)            /* State D+ and D- lines will be forced into by the DPDMDRIVE task */
#define USBD_DPDMVALUE_STATE_MASK            (0x1f << USBD_DPDMVALUE_STATE_SHIFT)
#  define USBD_DPDMVALUE_STATE_RESUME        (1 << USBD_DPDMVALUE_STATE_SHIFT)
#  define USBD_DPDMVALUE_STATE_J             (2 << USBD_DPDMVALUE_STATE_SHIFT)
#  define USBD_DPDMVALUE_STATE_K             (4 << USBD_DPDMVALUE_STATE_SHIFT)

/* DTOGGLE */

#define USBD_DTOGGLE_EP_SHIFT                (0)            /* Select bulk endpoint number */
#define USBD_DTOGGLE_EP_MASK                 (0x7 << USBD_DTOGGLE_EP_SHIFT)
#  define USBD_DTOGGLE_EP(n)                 ((n << USBD_DTOGGLE_EP_SHIFT) & USBD_DTOGGLE_EP_MASK)
#define USBD_DTOGGLE_IO_OUT                  (0 << 7)
#define USBD_DTOGGLE_IO_IN                   (1 << 7)
#define USBD_DTOGGLE_VALUE_SHIFT             (8)            /* Data toggle value */
#define USBD_DTOGGLE_VALUE_MASK              (0x3 << USBD_DTOGGLE_VALUE_SHIFT)
#  define USBD_DTOGGLE_VALUE_NOP             (0 << USBD_DTOGGLE_VALUE_SHIFT)
#  define USBD_DTOGGLE_VALUE_DATA0           (1 << USBD_DTOGGLE_VALUE_SHIFT)
#  define USBD_DTOGGLE_VALUE_DATA1           (2 << USBD_DTOGGLE_VALUE_SHIFT)

/* EPINEN */

#define USBD_EPINEN_IN(n)                    (1 << (0 + n)) /* Enable IN endpoint i */
#define USBD_EPINEN_ISOIN_DISABLE            (0 << 8)       /* Disable ISO IN endpoint */
#define USBD_EPINEN_ISOIN_ENABLE             (1 << 8)       /* Enable ISO IN endpoint */

/* EPOUTEN */

#define USBD_EPOUTEN_OUT(n)                  (1 << (0 + n)) /* Enable OUT endpoint i */
#define USBD_EPOUTEN_ISOOUT_DISABLE          (0 << 8)       /* Disable ISO OUT endpoint */
#define USBD_EPOUTEN_ISOOUT_ENABLE           (1 << 8)       /* Enable ISO OUT endpoint */

/* EPSTALL */

#define USBD_EPSTALL_EP_SHIFT                (0)            /* Select endpoint number */
#define USBD_EPSTALL_EP_MASK                 (0x7 << USBD_EPSTALL_EP_SHIFT)
#  define USBD_EPSTALL_EP(n)                 ((n << USBD_EPSTALL_EP_SHIFT) & USBD_EPSTALL_EP_MASK)
#define USBD_EPSTALL_IO_OUT                  (0 << 7)            /* Selects OUT endpoint */
#define USBD_EPSTALL_IO_IN                   (1 << 7)            /* Selects IN endpoint */
#define USBD_EPSTALL_IO_UNSTALL              (0 << 8)            /* Don't stall selected endpoint */
#define USBD_EPSTALL_IO_STALL                (1 << 8)            /* Stall selected endpoint */

/* ISOSPLIT */

#define USBD_ISOSPLIT_ONEDIR                 (0x0000)
#define USBD_ISOSPLIT_HALFIN                 (0x0080)

/* FRAMECNTR */

#define USBD_FRAMECNTR_MASK                  (0x7ff)

/* LOWPOWER */

#define USBD_LOWPOWER_NORMAL                 (0)
#define USBD_LOWPOWER_LOWPOWER               (1)

/* ISOINCONFIG */

#define USBD_ISOINCONFIG_NORESP              (0)
#define USBD_ISOINCONFIG_ZERODATA            (1)

/* EPIN[n].MAXCNT */

#define USBD_EPIN_MAXCNT_MASK                (0x7f)

/* EPIN[n].AMOUNT */

#define USBD_EPIN_AMOUNT_MASK                (0x7f)

/* ISOIN.MAXCNT */

#define USBD_ISOIN_MAXCNT_MASK               (0x3ff)

/* ISOIN.AMOUNT */

#define USBD_ISOIN_AMOUNT_MASK               (0x3ff)

/* EPOUT[n].MAXCNT */

#define USBD_EPOUT_MAXCNT_MASK               (0x7f)

/* EPOUT[n].AMOUNT */

#define USBD_EPOUT_AMOUNT_MASK               (0x7f)

/* ISOOUT.MAXCNT */

#define USBD_ISOOUT_MAXCNT_MASK              (0x3ff)

/* ISOOUT.AMOUNT */

#define USBD_ISOOUT_AMOUNT_MASK              (0x3ff)

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_USBD_H */
