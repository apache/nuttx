/****************************************************************************
 * include/nuttx/usb/max3421e.h
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

#ifndef __INCLUDE_NUTTX_USB_MAX3421E_H
#define __INCLUDE_NUTTX_USB_MAX3421E_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#ifdef CONFIG_USBHOST_MAX3421E

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* MAX3421E USB Host Driver Support
 *
 * Pre-requisites
 *
 *  CONFIG_USBHOST - Enable general USB host support
 *  CONFIG_USBHOST_MAX3421E - Enable the MAX3421E USB host support
 *  CONFIG_SCHED_LPWORK - Low priority work queue support is required.
 *
 * Options:
 *
 *   CONFIG_MAX3421E_DESCSIZE - Maximum size of a descriptor.  Default: 128
 *   CONFIG_MAX3421E_USBHOST_REGDEBUG - Enable very low-level register access
 *     debug.  Depends on CONFIG_DEBUG_USB_INFO.
 *   CONFIG_MAX3421E_USBHOST_PKTDUMP - Dump all incoming and outgoing USB
 *     packets. Depends on CONFIG_DEBUG_USB_INFO.
 */

/* Host Mode Register Addresses *********************************************/

/* The command byte contains the register address, a direction bit, and an
 * ACKSTAT bit:
 *
 *   Bits 3-7:  Command
 *   Bit 2:     Unused
 *   Bit 1:     Direction (read = 0, write = 1)
 *   Bit 0:     ACKSTAT
 */

#define MAX3421E_USBHOST_RCVFIFO        (1 << 3)
#define MAX3421E_USBHOST_SNDFIFO        (2 << 3)
#define MAX3421E_USBHOST_SUDFIFO        (4 << 3)
#define MAX3421E_USBHOST_RCVBC          (6 << 3)
#define MAX3421E_USBHOST_SNDBC          (7 << 3)
#define MAX3421E_USBHOST_USBIRQ         (13 << 3)
#define MAX3421E_USBHOST_USBIEN         (14 << 3)
#define MAX3421E_USBHOST_USBCTL         (15 << 3)
#define MAX3421E_USBHOST_CPUCTL         (16 << 3)
#define MAX3421E_USBHOST_PINCTL         (17 << 3)
#define MAX3421E_USBHOST_REVISION       (18 << 3)
#define MAX3421E_USBHOST_IOPINS1        (20 << 3)
#define MAX3421E_USBHOST_IOPINS2        (21 << 3)
#define MAX3421E_USBHOST_GPINIRQ        (22 << 3)
#define MAX3421E_USBHOST_GPINIEN        (23 << 3)
#define MAX3421E_USBHOST_GPINPOL        (24 << 3)
#define MAX3421E_USBHOST_HIRQ           (25 << 3)
#define MAX3421E_USBHOST_HIEN           (26 << 3)
#define MAX3421E_USBHOST_MODE           (27 << 3)
#define MAX3421E_USBHOST_PERADDR        (28 << 3)
#define MAX3421E_USBHOST_HCTL           (29 << 3)
#define MAX3421E_USBHOST_HXFR           (30 << 3)
#define MAX3421E_USBHOST_HRSL           (31 << 3)

/* Peripheral Mode Register Addresses ***************************************/

/* The command byte contains the register address, a direction bit, and an
 * ACKSTAT bit:
 *
 *   Bits 3-7:  Command
 *   Bit 2:     Unused
 *   Bit 1:     Direction (read = 0, write = 1)
 *   Bit 0:     ACKSTAT
 */

#define MAX3421E_USBDEV_EP0FIFO         (0 << 3)
#define MAX3421E_USBDEV_EP1OUTFIFO      (1 << 3)
#define MAX3421E_USBDEV_EP2INFIFO       (2 << 3)
#define MAX3421E_USBDEV_EP3INFIFO       (3 << 3)
#define MAX3421E_USBDEV_SUDFIFO         (4 << 3)
#define MAX3421E_USBDEV_EP0BC           (5 << 3)
#define MAX3421E_USBDEV_EP1OUTBC        (6 << 3)
#define MAX3421E_USBDEV_EP2INBC         (7 << 3)
#define MAX3421E_USBDEV_EP3INBC         (8 << 3)
#define MAX3421E_USBDEV_EPSTALLS        (9 << 3)
#define MAX3421E_USBDEV_CLRTOGS         (10 << 3)
#define MAX3421E_USBDEV_EPIRQ           (11 << 3)
#define MAX3421E_USBDEV_EPIEN           (12 << 3)
#define MAX3421E_USBDEV_USBIRQ          (13 << 3)
#define MAX3421E_USBDEV_USBIEN          (14 << 3)
#define MAX3421E_USBDEV_USBCTL          (15 << 3)
#define MAX3421E_USBDEV_CPUCTL          (16 << 3)
#define MAX3421E_USBDEV_PINCTL          (17 << 3)
#define MAX3421E_USBDEV_REVISION        (18 << 3)
#define MAX3421E_USBDEV_FNADDR          (19 << 3)
#define MAX3421E_USBDEV_IOPINS1         (20 << 3)
#define MAX3421E_USBDEV_IOPINS2         (21 << 3)
#define MAX3421E_USBDEV_GPINIRQ         (22 << 3)
#define MAX3421E_USBDEV_GPINIEN         (23 << 3)
#define MAX3421E_USBDEV_GPINPOL         (24 << 3)
#define MAX3421E_USBDEV_MODE            (27 << 3)

/* Host Mode Register Bit-Field Definitions *********************************/

#define USBHOST_RCVBC_MASK              0x7f
#define USBHOST_SNDBC_MASK              0x7f

#define USBHOST_USBIRQ_OSCOKIRQ         (1 << 0)
#define USBHOST_USBIRQ_NOVBUSIRQ        (1 << 5)
#define USBHOST_USBIRQ_VBUSIRQ          (1 << 6)

#define USBHOST_USBIEN_OSCOKIE          (1 << 0)
#define USBHOST_USBIEN_NOVBUSIE         (1 << 5)
#define USBHOST_USBIEN_VBUSIEN          (1 << 6)

#define USBHOST_USBCTL_PWRDOWN          (1 << 4)
#define USBHOST_USBCTL_CHIPRES          (1 << 5)

#define USBHOST_CPUCTL_IE               (1 << 0)
#define USBHOST_CPUCTL_PULSEWID_SHIFT   (6)       /* Bits 6-7:  INT Pulsewidth */
#define USBHOST_CPUCTL_PULSEWID_MASK    (3 << USBHOST_CPUCTL_PULSEWID_SHIFT)
#  define USBHOST_CPUCTL_PULSEWID0      (1 << 6)
#  define USBHOST_CPUCTL_PULSEWID1      (1 << 7)
#  define USBHOST_CPUCTL_PULSEWID_10p6US  (0 << USBHOST_CPUCTL_PULSEWID_SHIFT) /* 10.6 uS */
#  define USBHOST_CPUCTL_PULSEWID_5p3US   (1 << USBHOST_CPUCTL_PULSEWID_SHIFT) /* 5.3 uS */
#  define USBHOST_CPUCTL_PULSEWID_2p6US   (2 << USBHOST_CPUCTL_PULSEWID_SHIFT) /* 2.6 uS */
#  define USBHOST_CPUCTL_PULSEWID_1p3US   (4 << USBHOST_CPUCTL_PULSEWID_SHIFT) /* 1.3 uS */

#define USBHOST_PINCTL_PXA              (1 << 0)
#define USBHOST_PINCTL_GPXB             (1 << 1)
#define USBHOST_PINCTL_POSINT           (1 << 2)
#define USBHOST_PINCTL_INTLEVEL         (1 << 3)
#define USBHOST_PINCTL_FDUPSPI          (1 << 4)
#define USBHOST_PINCTL_EP0INAK          (1 << 5)
#define USBHOST_PINCTL_EP2INAK          (1 << 6)
#define USBHOST_PINCTL_EP3INAK          (1 << 7)

#define USBHOST_REVISION                0x13

#define USBHOST_IOPINS1_GPOUT(n)        (1 << (n))
#define USBHOST_IOPINS1_GPIN(n)         (1 << ((n) + 4))
#define USBHOST_IOPINS2_GPOUT(n)        (1 << ((n) - 4))
#define USBHOST_IOPINS2_GPIN(n)         (1 << (n))
#define USBHOST_GPINIRQ(n)              (1 << (n))
#define USBHOST_GPINIEN(n)              (1 << (n))
#define USBHOST_GPINPOL(n)              (1 << (n))

#define USBHOST_HIRQ_BUSEVENTIRQ        (1 << 0)
#define USBHOST_HIRQ_RSMREQIRQ          (1 << 1)
#define USBHOST_HIRQ_RCVDAVIRQ          (1 << 2)
#define USBHOST_HIRQ_SNDBAVIRQ          (1 << 3)
#define USBHOST_HIRQ_SUSDNIRQ           (1 << 4)
#define USBHOST_HIRQ_CONNIRQ            (1 << 5)
#define USBHOST_HIRQ_FRAMEIRQ           (1 << 6)
#define USBHOST_HIRQ_HXFRDNIRQ          (1 << 7)

#define USBHOST_HIEN_BUSEVENTIE         (1 << 0)
#define USBHOST_HIEN_RSMREQIE           (1 << 1)
#define USBHOST_HIEN_RCVDAVIE           (1 << 2)
#define USBHOST_HIEN_SNDBAVIE           (1 << 3)
#define USBHOST_HIEN_SUSDNIE            (1 << 4)
#define USBHOST_HIEN_CONNIE             (1 << 5)
#define USBHOST_HIEN_FRAMEIE            (1 << 6)
#define USBHOST_HIEN_HXFRDNIE           (1 << 7)

#define USBHOST_MODE_HOST               (1 << 0)
#define USBHOST_MODE_SPEED              (1 << 1)
#define USBHOST_MODE_HUBPRE             (1 << 2)
#define USBHOST_MODE_SOFKAENAB          (1 << 3)
#define USBHOST_MODE_SEPIRQ             (1 << 4)
#define USBHOST_MODE_NDELAYISO          (1 << 5)
#define USBHOST_MODE_DMPULLD            (1 << 6)
#define USBHOST_MODE_DPPULLDN           (1 << 7)

#define USBHOST_PERADDR_MASK            0x7f

#define USBHOST_HCTL_BUSRST             (1 << 0)
#define USBHOST_HCTL_FRMRST             (1 << 1)
#define USBHOST_HCTL_BUSSAMPLE          (1 << 2)
#define USBHOST_HCTL_SIGRSM             (1 << 3)
#define USBHOST_HCTL_TOGGLES_SHIFT      (4)       /* Bits 4-7: Data toggles */
#define USBHOST_HCTL_TOGGLES_MASK       (15 << USBHOST_HCTL_TOGGLES_SHIFT)
#  define USBHOST_HCTL_RCVTOG0          (1 << 4)
#  define USBHOST_HCTL_RCVTOG1          (1 << 5)
#  define USBHOST_HCTL_SNDTOG0          (1 << 6)
#  define USBHOST_HCTL_SNDTOG1          (1 << 7)

#define USBHOST_HXFR_EP_SHIFT           (0)      /* Bits 0-3:  Endpoint number */
#define USBHOST_HXFR_EP_MASK            (15 << USBHOST_HXFR_EP_SHIFT)
#  define USBHOST_HXFR_EP(n)            ((uint8_t)(n) << USBHOST_HXFR_EP_SHIFT)
#define USBHOST_HXFR_TOKEN_SHIFT        (4)      /* Bits 4-7:  Token */
#define USBHOST_HXFR_TOKEN_MASK         (15 << USBHOST_HXFR_EP_SHIFT)
#  define USBHOST_HXFR_SETUP            (1 << 4)
#  define USBHOST_HXFR_OUTNIN           (1 << 5)
#  define USBHOST_HXFR_ISO              (1 << 6)
#  define USBHOST_HXFR_HS               (1 << 7)
#  define USBHOST_HXFR_TOKEN_IN         (0)
#  define USBHOST_HXFR_TOKEN_SETUP      USBHOST_HXFR_SETUP
#  define USBHOST_HXFR_TOKEN_OUT        USBHOST_HXFR_OUTNIN
#  define USBHOST_HXFR_TOKEN_INHS       USBHOST_HXFR_HS
#  define USBHOST_HXFR_TOKEN_OUTHS      (USBHOST_HXFR_OUTNIN | USBHOST_HXFR_HS)
#  define USBHOST_HXFR_TOKEN_ISOIN      USBHOST_HXFR_ISO
#  define USBHOST_HXFR_TOKEN_ISOOUT     (USBHOST_HXFR_OUTNIN | USBHOST_HXFR_ISO)

#define USBHOST_HRSL_HRSLT_SHIFT        (0)       /* Bits 0-3: Host result error code */
#define USBHOST_HRSL_HRSLT_MASK         (15 << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_SUCCESS    (0  << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_BUSY       (1  << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_BADREQ     (2  << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_UNDEF      (3  << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_NAK        (4  << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_STALL      (5  << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_TOGERR     (6  << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_WRONGPID   (7  << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_BADBC      (8  << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_PIDERR     (9  << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_PKTERR     (10 << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_CRCERR     (11 << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_KERR       (12 << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_JERR       (13 << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_TIMEOUT    (14 << USBHOST_HRSL_HRSLT_SHIFT)
#  define USBHOST_HRSL_HRSLT_BABBLE     (15 << USBHOST_HRSL_HRSLT_SHIFT)
#define USBHOST_HRSL_RCVTOGRD           (1 << 4)
#define USBHOST_HRSL_SNDTOGRD           (1 << 5)
#define USBHOST_HRSL_KSTATUS            (1 << 6)
#define USBHOST_HRSL_JSTATUS            (1 << 7)

/* Peripheral Mode Register Bit-Field Definitions ***************************/

#define USBDEV_EP0BC_MASK               0x7f
#define USBDEV_EP1OUTBC_MASK            0x7f
#define USBDEV_EP2INBC_MASK             0x7f
#define USBDEV_EP3INBC_MASK             0x7f

#define USBDEV_EPSTALLS_STLEP0IN        (1 << 0)
#define USBDEV_EPSTALLS_STLEP0OUT       (1 << 1)
#define USBDEV_EPSTALLS_STLEP1OUT       (1 << 2)
#define USBDEV_EPSTALLS_STLEP2IN        (1 << 3)
#define USBDEV_EPSTALLS_STLEP3IN        (1 << 4)
#define USBDEV_EPSTALLS_STLSTAT         (1 << 5)
#define USBDEV_EPSTALLS_ACKSTAT         (1 << 6)

#define USBDEV_CLRTOGS_CTGEP1OUT        (1 << 2)
#define USBDEV_CLRTOGS_CTGEP2IN         (1 << 3)
#define USBDEV_CLRTOGS_CTGEP3IN         (1 << 4)
#define USBDEV_CLRTOGS_EP1DISAB         (1 << 5)
#define USBDEV_CLRTOGS_EP2DISAB         (1 << 6)
#define USBDEV_CLRTOGS_EP3DISAB         (1 << 7)

#define USBDEV_EPIRQ_IN0BAVIRQ          (1 << 0)
#define USBDEV_EPIRQ_OUT0DAVIRQ         (1 << 1)
#define USBDEV_EPIRQ_OUT1DAVIRQ         (1 << 2)
#define USBDEV_EPIRQ_IN2BAVIRQ          (1 << 3)
#define USBDEV_EPIRQ_IN3BAVIRQ          (1 << 4)
#define USBDEV_EPIRQ_SUDAVIRQ           (1 << 5)

#define USBDEV_EPIEN_IN0BAVIE           (1 << 0)
#define USBDEV_EPIEN_OUT0DAVIE          (1 << 1)
#define USBDEV_EPIEN_OUT1DAVIE          (1 << 2)
#define USBDEV_EPIEN_IN2BAVIE           (1 << 3)
#define USBDEV_EPIEN_IN3BAVIE           (1 << 4)
#define USBDEV_EPIEN_SUDAVIE            (1 << 5)

#define USBDEV_USBIRQ_OSCOKIRQ          (1 << 0)
#define USBDEV_USBIRQ_RWUDNIRQ          (1 << 1)
#define USBDEV_USBIRQ_BUSACTIRQ         (1 << 2)
#define USBDEV_USBIRQ_URESIRQ           (1 << 3)
#define USBDEV_USBIRQ_SUSPIRQ           (1 << 4)
#define USBDEV_USBIRQ_NOVBUSIRQ         (1 << 5)
#define USBDEV_USBIRQ_VBUSIRQ           (1 << 6)
#define USBDEV_USBIRQ_URESDNIRQ         (1 << 7)

#define USBDEV_USBIEN_OSCOKIE           (1 << 0)
#define USBDEV_USBIEN_RWUDNIE           (1 << 1)
#define USBDEV_USBIEN_BUSACTIE          (1 << 2)
#define USBDEV_USBIEN_URESIE            (1 << 3)
#define USBDEV_USBIEN_SUSPIE            (1 << 4)
#define USBDEV_USBIEN_NOVBUSIE          (1 << 5)
#define USBDEV_USBIEN_VBUSIEN           (1 << 6)
#define USBDEV_USBIEN_URESDNIE          (1 << 7)

#define USBDEV_USBCTL_SIGRWU            (1 << 2)
#define USBDEV_USBCTL_CONNECT           (1 << 3)
#define USBDEV_USBCTL_PWRDOWN           (1 << 4)
#define USBDEV_USBCTL_CHIPRES           (1 << 5)
#define USBDEV_USBCTL_VBGATE            (1 << 6)
#define USBDEV_USBCTL_HOSCSTEN          (1 << 7)

#define USBDEV_CPUCTL_IE                (1 << 0)
#define USBDEV_CPUCTL_PULSEWID0         (1 << 6)
#define USBDEV_CPUCTL_PULSEWID1         (1 << 7)

#define USBDEV_PINCTL_PXA               (1 << 0)
#define USBDEV_PINCTL_GPXB              (1 << 1)
#define USBDEV_PINCTL_POSINT            (1 << 2)
#define USBDEV_PINCTL_INTLEVEL          (1 << 3)
#define USBDEV_PINCTL_FDUPSPI           (1 << 4)
#define USBDEV_PINCTL_EP0INAK           (1 << 5)
#define USBDEV_PINCTL_EP2INAK           (1 << 6)
#define USBDEV_PINCTL_EP3INAK           (1 << 7)

#define USBDEV_REVISION                 0x13
#define USBDEV_FNADDR_MASK              0x7f

#define USBDEV_IOPINS1_GPOUT(n)         (1 << (n))
#define USBDEV_IOPINS1_GPIN(n)          (1 << ((n) + 4))
#define USBDEV_IOPINS2_GPOUT(n)         (1 << ((n) - 4))
#define USBDEV_IOPINS2_GPIN(n)          (1 << (n))
#define USBDEV_GPINIRQ(n)               (1 << (n))
#define USBDEV_GPINIEN(n)               (1 << (n))
#define USBDEV_GPINPOL(n)               (1 << (n))

#define USBDEV_MODE_SEPIRQ              (1 << 4)
#define USBDEV_MODE_HOST                (1 << 0)

/* Misc. Definitions ********************************************************/

/* The command byte contains the register address, a direction bit, and an
 * ACKSTAT bit:
 *
 *   Bits 3-7:  Command
 *   Bit 2:     Unused
 *   Bit 1:     Direction (read = 0, write = 1)
 *   Bit 0:     ACKSTAT
 *
 * The ACKSTAT bit sets the ACKSTAT bit in the EPSTALLS (R9) register
 * (peripheral mode only). The SPI master sets this bit to indicate that it
 * has finished servicing a CONTROL transfer.  The ACKSTAT bit is ignored in
 * host mode.
 */

/* Read/write access to a register */

#define MAX3421E_DIR_WRITE              0x02
#define MAX3421E_DIR_READ               0x00

#define MAX3421E_ACKSTAT_TRUE           0x01
#define MAX3421E_ACKSTAT_FALSE          0x00

/* Sizes and numbers of things -- Peripheral mode */

#define MAX3421E_NENDPOINTS             4       /* EP0-EP3 */
#define MAX3421E_ALLEP_SET              0x0f    /* EP0-EP3 */
#define MAX3421E_CONTROL_SET            0x01    /* EP0 is the only control EP */
#define MAX3421E_BULK_SET               0x0e    /* EP1-3 can be bulk EPs */
#define MAX3421E_INTERUPT_SET           0x0e    /* EP1-3 can be interrupt EPs */
#define MAX3421E_OUTEP_SET              0x02    /* EP1 is the only OUT endpoint */
#define MAX3421E_INEP_SET               0x0c    /* EP2-3 are IN endpoints */
#define MAX3421E_DBLBUF_SET             0x06    /* EP1-2 are double buffered */

#define MAX3421E_SETUPFIFO_SIZE         8

/* Sizes and numbers of things -- Host mode */

#define MAX3421E_NHOST_CHANNELS         16      /* Number of host channels */
#define MAX3421E_SNDFIFO_SIZE           64      /* Send FIFO, double-buffered */
#define MAX3421E_RCVFIFO_SIZE           64      /* Receive FIFO, double-buffered */
#define MAX3421E_SUDFIFO_SIZE           8       /* Setup FIFO */

/* Value of the MODE register HOST bit */

#define MAX3421E_MODE_PERIPH            0
#define MAX3421E_MODE_HOST              1

/* The MAX3421E SPI interface operates without adjustment in either SPI mode
 * (CPOL = 0, CPHA = 0) or (CPOL = 1, CPHA = 1).
 */

#define MAX3421E_SPIMODE                SPIDEV_MODE0

/* The SPI clock frequency can be between DC and 26MHz. */

#define MAX3421E_SPIFREQ_MAX            (26*1000*1000)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure defines the interface provided by the max3421e lower-half
 * driver.
 */

enum spi_mode_e;  /* Forward reference */
struct spi_dev_s; /* Forward reference */

struct max3421e_lowerhalf_s
{
  /* Device characterization.
   *
   * The interrupt configuration byte may have the following values:
   *
   * USBDEV_PINCTL_INTLEVEL=1 USBDEV_PINCTL_POSINT=xx (has no effect)
   *   Open-drain, low level active interrupt.  In this mode the INT pin is
   *   open-drain, so a pull-up resistor on the INT line is necessary.
   *
   * USBDEV_PINCTL_INTLEVEL=0 USBDEV_PINCTL_POSINT=0
   *   Push-pull, falling edge-sensitive.  When POSINT=0 (and INTLEVEL=0),
   *   the INT pin signals pending interrupts with a negative edge.
   *
   * USBDEV_PINCTL_INTLEVEL=0 USBDEV_PINCTL_POSINT=1
   *   Push-pull, rising edge-sensitive.  When POSINT=1 (and INTLEVEL=0),
   *   the INT pin signals pending interrupts with a positive edge.
   */

  FAR struct spi_dev_s *spi; /* SPI device instance */
  uint32_t frequency;        /* SPI frequency < 26MHz */
  enum spi_mode_e mode;      /* Either SPIDEV_MODE0 or SPIDEV_MODE3 */
  uint8_t devid;             /* Distinguishes multiple MAX3421E on SPI bus */
  uint8_t intconfig;         /* Interrupt configuration.  See notes above. */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
   * to isolate the driver from differences in GPIO interrupt handling
   * by varying boards and MCUs:
   *
   *   attach      - Attach the interrupt handler to the GPIO interrupt
   *   enable      - Enable or disable the GPIO interrupt
   *   acknowledge - Acknowledge/clear any pending GPIO interrupt
   *   power       - Enable or disable 5V VBUS power.  REVISIT:  Often a
   *                 GPIO from the MAX3421E is used to control VBUS power.
   *
   * REVISIT: A method may be necessary to sense the state of the GPX input.
   */

  CODE int (*attach)(FAR const struct max3421e_lowerhalf_s *lower,
                     xcpt_t isr, FAR void *arg);
  CODE void (*enable)(FAR const struct max3421e_lowerhalf_s *lower,
                      bool enable);
  CODE void (*acknowledge)(FAR const struct max3421e_lowerhalf_s *lower);
  CODE void (*power)(FAR const struct max3421e_lowerhalf_s *lower,
                     bool enable);

  /* Additional, driver-specific state data may follow */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct usbhost_connection_s; /* Forward reference */

/****************************************************************************
 * Name: max3421e_usbhost_initialize
 *
 * Description:
 *   Initialize MAX3421E as USB host controller.
 *
 * Input Parameters:
 *   lower - The interface to the lower half driver
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the device
 *   to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already connected.
 *
 ****************************************************************************/

FAR struct usbhost_connection_s *
max3421e_usbhost_initialize(FAR const struct max3421e_lowerhalf_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_USBHOST_MAX3421E */
#endif /* __INCLUDE_NUTTX_USB_MAX3421E_H */
