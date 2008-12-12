/****************************************************************************
 * include/nuttx/mii.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __INCLUDE_NUTTX_MII_H
#define __INCLUDE_NUTTX_MII_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* MII register offsets */

#define MII_BMCR                   0x00     /* Basic mode control */
#define MII_BMSR                   0x01     /* Basic mode status */
#define MII_PHYSID1                0x02     /* PHYS ID 1 */
#define MII_PHYSID2                0x03     /* PHYS ID 2 */
#define MII_ADVERTISE              0x04     /* Advertisement control */
#define MII_LPA                    0x05     /* Link partner ability */
#define MII_EXPANSION              0x06     /* Expansion  */
#define MII_CTRL1000               0x09     /* 1000BASE-T control */
#define MII_STAT1000               0x0a     /* 1000BASE-T status */
#define MII_ESTATUS                0x0f     /* Extended Status */
#define MII_DCOUNTER               0x12     /* Disconnect counter */
#define MII_FCSCOUNTER             0x13     /* False carrier counter */
#define MII_NWAYTEST               0x14     /* N-way auto-neg test reg */
#define MII_RERRCOUNTER            0x15     /* Receive error counter */
#define MII_SREVISION              0x16     /* Silicon revision */
#define MII_LBRERROR               0x18     /* Lpback, rx, bypass error */
#define MII_PHYADDR                0x19     /* PHY address */
#define MII_TPISTATUS              0x1b     /* TPI status for 10mbps */
#define MII_NCONFIG                0x1c     /* Network interface config */

/* MII Control register bit definitions */

#define MII_BMCR_SPEED1000         0x0040  /* MSB of Speed (1000) */
#define MII_BMCR_CTST              0x0080  /* Collision test  */
#define MII_BMCR_FULLDPLX          0x0100  /* Full duplex */
#define MII_BMCR_ANRESTART         0x0200  /* Auto negotiation restart */
#define MII_BMCR_ISOLATE           0x0400  /* Disconnect DP83840 from MII */
#define MII_BMCR_PDOWN             0x0800  /* Powerdown the DP83840 */
#define MII_BMCR_ANENABLE          0x1000  /* Enable auto negotiation */
#define MII_BMCR_SPEED100          0x2000  /* Select 100Mbps */
#define MII_BMCR_LOOPBACK          0x4000  /* TXD loopback bits */
#define MII_BMCR_RESET             0x8000  /* Reset the DP83840 */

/* MII Status register bit definitions */

#define MII_BMSR_ERCAP              0x0001  /* Ext-reg capability */
#define MII_BMSR_JCD                0x0002  /* Jabber detected */
#define MII_BMSR_LSTATUS            0x0004  /* Link status */
#define MII_BMSR_ANEGCAPABLE        0x0008  /* Able to do auto-negotiation */
#define MII_BMSR_RFAULT             0x0010  /* Remote fault detected */
#define MII_BMSR_ANEGCOMPLETE       0x0020  /* Auto-negotiation complete */
#define MII_BMSR_ESTATEN            0x0100  /* Extended Status in R15 */
#define MII_BMSR_100FULL2           0x0200  /* Can do 100BASE-T2 HDX */
#define MII_BMSR_100HALF2           0x0400  /* Can do 100BASE-T2 FDX */
#define MII_BMSR_10HALF             0x0800  /* Can do 10mbps half-duplex */
#define MII_BMSR_10FULL             0x1000  /* Can do 10mbps full-duplex */
#define MII_BMSR_100HALF            0x2000  /* Can do 100mbps half-duplex */
#define MII_BMSR_100FULL            0x4000  /* Can do 100mbps full-duplex */
#define MII_BMSR_100BASE4           0x8000  /* Can do 100mbps 4k packets */

/* Advertisement control register bit definitions */

#define MII_ADVERTISE_SLCT          0x001f  /* Selector bits */
#define MII_ADVERTISE_CSMA          0x0001  /* Only selector supported */
#define MII_ADVERTISE_10HALF        0x0020  /* Try for 10mbps half-duplex */
#define MII_ADVERTISE_1000XFULL     0x0020  /* Try for 1000BASE-X full-duplex */
#define MII_ADVERTISE_10FULL        0x0040  /* Try for 10mbps full-duplex */
#define MII_ADVERTISE_1000XHALF     0x0040  /* Try for 1000BASE-X half-duplex */
#define MII_ADVERTISE_100HALF       0x0080  /* Try for 100mbps half-duplex */
#define MII_ADVERTISE_1000XPAUSE    0x0080  /* Try for 1000BASE-X pause */
#define MII_ADVERTISE_100FULL       0x0100  /* Try for 100mbps full-duplex */
#define MII_ADVERTISE_1000XPSE_ASYM 0x0100  /* Try for 1000BASE-X asym pause */
#define MII_ADVERTISE_100BASE4      0x0200  /* Try for 100mbps 4k packets */
#define MII_ADVERTISE_PAUSE_CAP     0x0400  /* Try for pause */
#define MII_ADVERTISE_PAUSE_ASYM    0x0800  /* Try for asymetric pause */
#define MII_ADVERTISE_RFAULT        0x2000  /* Say we can detect faults */
#define MII_ADVERTISE_LPACK         0x4000  /* Ack link partners response */
#define MII_ADVERTISE_NPAGE         0x8000  /* Next page bit */

/* Link partner ability register bit definitions */

#define MII_LPA_SLCT                0x001f  /* Same as advertise selector */
#define MII_LPA_10HALF              0x0020  /* Can do 10mbps half-duplex */
#define MII_LPA_1000XFULL           0x0020  /* Can do 1000BASE-X full-duplex */
#define MII_LPA_10FULL              0x0040  /* Can do 10mbps full-duplex */
#define MII_LPA_1000XHALF           0x0040  /* Can do 1000BASE-X half-duplex */
#define MII_LPA_100HALF             0x0080  /* Can do 100mbps half-duplex */
#define MII_LPA_1000XPAUSE          0x0080  /* Can do 1000BASE-X pause */
#define MII_LPA_100FULL             0x0100  /* Can do 100mbps full-duplex */
#define MII_LPA_1000XPAUSE_ASYM     0x0100  /* Can do 1000BASE-X pause asym*/
#define MII_LPA_100BASE4            0x0200  /* Can do 100mbps 4k packets */
#define MII_LPA_PAUSE_CAP           0x0400  /* Can pause */
#define MII_LPA_PAUSE_ASYM          0x0800  /* Can pause asymetrically */
#define MII_LPA_RFAULT              0x2000  /* Link partner faulted */
#define MII_LPA_LPACK               0x4000  /* Link partner acked us */
#define MII_LPA_NPAGE               0x8000  /* Next page bit */

/* MII Auto-negotiation expansion register bit definitions */

#define MII_EXPANSION_NWAY          0x0001  /* Can do N-way auto-nego */
#define MII_EXPANSION_LCWP          0x0002  /* Got new RX page code word */
#define MII_EXPANSION_ENABLENPAGE   0x0004  /* This enables npage words */
#define MII_EXPANSION_NPCAPABLE     0x0008  /* Link partner supports npage */
#define MII_EXPANSION_MFAULTS       0x0010  /* Multiple faults detected */

/* MII ID2 register bits */

#define MII_ID2_OUI_LO              0xfc00  /* Low bits of OUI mask */
#define MII_ID2_MODEL               0x03f0  /* Model number */
#define MII_ID2_REV                 0x000f  /* Model number */

/* MII PHYADDR register bit definitions */

#define MII_PHYADDR_DUPLEX          0x0080
#define MII_PHYADDR_SPEED           0x0040

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_MII_H */
