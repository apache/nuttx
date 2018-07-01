/****************************************************************************
 * include/nuttx/usb/max3421e.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
#define USBHOST_CPUCTL_PULSEWID0        (1 << 6)
#define USBHOST_CPUCTL_PULSEWID1        (1 << 7)

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
#define USBHOST_HCTL_RCVTOG0            (1 << 4)
#define USBHOST_HCTL_RCVTOG1            (1 << 5)
#define USBHOST_HCTL_SNDTOG0            (1 << 6)
#define USBHOST_HCTL_SNDTOG1            (1 << 7)

#define USBHOST_HXFR_EP0                (1 << 0)
#define USBHOST_HXFR_EP1                (1 << 1)
#define USBHOST_HXFR_EP2                (1 << 2)
#define USBHOST_HXFR_EP3                (1 << 3)
#define USBHOST_HXFR_SETUP              (1 << 4)
#define USBHOST_HXFR_OUTNIN             (1 << 5)
#define USBHOST_HXFR_ISO                (1 << 6)
#define USBHOST_HXFR_HS                 (1 << 7)

#define USBHOST_HRSL_HRSLT0             (1 << 0)
#define USBHOST_HRSL_HRSLT1             (1 << 1)
#define USBHOST_HRSL_HRSLT2             (1 << 2)
#define USBHOST_HRSL_HRSLT3             (1 << 3)
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
 */

/* Read/write access to a register */

#define MAX3421E_DIR_WRITE              0x02
#define MAX3421E_DIR_READ               0x00

#define MAX3421E_ACKSTAT_TRUE           0x01
#define MAX3421E_ACKSTAT_FALSE          0x00

/* Sizes and numbers of things */

#define MAX3421E_NENDPOINTS             4       /* EP0..EP3 */
#define MAX3421E_DBLBUF_SET             0x06    /* EP2, EP3 double buffered */

#define MAX3421E_SNDFIFO_SIZE           64
#define MAX3421E_RCVFIFO_SIZE           64
#define MAX3421E_SETUPFIFO_SIZE         8

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
  /* Device characterization */

  FAR struct spi_dev_s *spi; /* SPI device instance */
  uint32_t frequency;        /* SPI frequency < 26MHz */
  enum spi_mode_e mode;      /* Either SPIDEV_MODE0 or SPIDEV_MODE3 */
  uint8_t devid;             /* Distinguishes multiple MAX3421E on SPI bus */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
   * to isolate the driver from differences in GPIO interrupt handling
   * by varying boards and MCUs:
   *
   *   attach      - Attach the interrupt handler to the GPIO interrupt
   *   enable      - Enable or disable the GPIO interrupt
   *   acknowledge - Acknowledge/clear any pending GPIO interrupt
   */

  CODE int (*attach)(FAR struct max3421e_lowerhalf_s *lower, xcpt_t isr,
                     FAR void *arg);
  CODE void (*enable)(FAR const struct max3421e_lowerhalf_s *lower,
                      bool enable);
  CODE void (*acknowledge)(FAR const struct max3421e_lowerhalf_s *lower);

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
