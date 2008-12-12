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

/* MII register offsets *****************************************************/

/* Apparently common registers */

#define MII_MCR                     0x00    /* MII management control */
#define MII_MSR                     0x01    /* MII management status */
#define MII_PHYID1                  0x02    /* PHY ID 1 */
#define MII_PHYID2                  0x03    /* PHY ID 2 */
#define MII_ADVERTISE               0x04    /* Auto-negotiation advertisement */
#define MII_LPA                     0x05    /* Auto-negotiation link partner ability */
#define MII_EXPANSION               0x06    /* Auto-negotiation expansion register*/

/* DP83840: 0x07-0x11, 0x14, 0x1a, 0x1d-0x1f reserved */

#define MII_DCOUNTER                0x12    /* Disconnect counter */
#define MII_FCSCOUNTER              0x13    /* False carrier sense counter */
#define MII_NWAYTEST                0x14    /* N-way auto-neg test reg */
#define MII_RERRCOUNTER             0x15    /* Receive error counter */
#define MII_SREVISION               0x16    /* Silicon revision */
#define MII_LBRERROR                0x18    /* Loopback, bypass and receiver error */
#define MII_PHYADDR                 0x19    /* PHY address */
#define MII_10BTSR                  0x1b    /* 10BASE-T status register */
#define MII_10BTCR                  0x1c    /* 10BASE-T configuration register */

/* Am79c874: 0x08-0x0f, 0x14, 0x16, 0x19-0x1f reserved */

#define MII_NPADVERTISE             0x07    /* Auto-negotiation next page advertisement */
#define MII_MISCFEATURES            0x10    /* Miscellaneous features reg */
#define MII_INTCS                   0x11    /* Interrupt control/status */
#define MII_DIAGNOSTIC              0x12    /* Diagnostic register */
#define MII_LOOPBACK                0x13    /* Power management/loopback register */
#define MII_MODEC                   0x15    /* Mode control register */
#define MII_DISCONNECT              0x17    /* Disconnect counter */
#define MII_RCVERROR                0x18    /* Receive error counter */

/* */

#define MII_CTRL1000                0x09    /* 1000BASE-T control */
#define MII_STAT1000                0x0a    /* 1000BASE-T status */
#define MII_ESTATUS                 0x0f    /* Extended Status */
#define MII_NCONFIG                 0x1c    /* Network interface config */

/* MII register bit settings ************************************************/

/* MII Control register bit definitions */

#define MII_MCR_SPEED1000           0x0040  /* Bit 6: MSB of Speed (1000 reserved on 10/100) */
#define MII_MCR_CTST                0x0080  /* Bit 7: Enable ollision test  */
#define MII_MCR_FULLDPLX            0x0100  /* Bit 8: Full duplex */
#define MII_MCR_ANRESTART           0x0200  /* Bit 9: Restart auto negotiation */
#define MII_MCR_ISOLATE             0x0400  /* Bit 10: Electronically isolate PHY from MII */
#define MII_MCR_PDOWN               0x0800  /* Bit 11: Powerdown the PHY */
#define MII_MCR_ANENABLE            0x1000  /* Bit 12: Enable auto negotiation */
#define MII_MCR_SPEED100            0x2000  /* Bit 13: Select 100Mbps */
#define MII_MCR_LOOPBACK            0x4000  /* Bit 14: Enable loopback mode */
#define MII_MCR_RESET               0x8000  /* Bit 15: PHY reset */

/* MII Status register bit definitions */

#define MII_MSR_EXTCAP              0x0001  /* Bit 0: Extended register capability */
#define MII_MSR_JABBERDETECT        0x0002  /* Bit 1: Jabber detect */
#define MII_MSR_LINKSTATUS          0x0004  /* Bit 2: Link status */
#define MII_MSR_ANEGABLE            0x0008  /* Bit 3: Auto-negotiation able */
#define MII_MSR_RFAULT              0x0010  /* Bit 4: Remote fault */
#define MII_MSR_ANEGCOMPLETE        0x0020  /* Bit 5: Auto-negotiation complete */
#define MII_MSR_MFRAMESUPPRESS      0x0040  /* Bit 6: Management frame suppression */
#define MII_MSR_ESTATEN             0x0100  /* Bit 8: Extended Status in R15 */
#define MII_MSR_100BASET2FULL       0x0200  /* Bit 9: 100BASE-T2 half duplex able */
#define MII_MSR_100BASET2HALF       0x0400  /* Bit 10: 100BASE-T2 full duplex able */
#define MII_MSR_10BASETXHALF        0x0800  /* Bit 11: 10BASE-TX half duplex able */
#define MII_MSR_10BASETXFULL        0x1000  /* Bit 12: 10BASE-TX full duplex able */
#define MII_MSR_100BASETXHALF       0x2000  /* Bit 13: 100BASE-TX half duplex able */
#define MII_MSR_100BASETXFULL       0x4000  /* Bit 14: 100BASE-TX full duplex able */
#define MII_MSR_100BASET4           0x8000  /* Bit 15: 100BASE-T4 able */

/* MII ID2 register bits */

#define MII_PHYID2_OUI              0xfc00  /* OUI mask */
#define MII_PHYID2_MODEL            0x03f0  /* Model number mask */
#define MII_PHYID2_REV              0x000f  /* Revision number mask */

/* Advertisement control register bit definitions */

#define MII_ADVERTISE_SELECT        0x001f  /* Bits 0-4: Selector field */
#define MII_ADVERTISE_CSMA          0x0001  /*        CSMA */
#define MII_ADVERTISE_10BASETXHALF  0x0020  /* Bit 5: Try 10BASE-TX half duplex */
#define MII_ADVERTISE_1000XFULL     0x0020  /* Bit 5: Try 1000BASE-X full duplex */
#define MII_ADVERTISE_10BASETXFULL  0x0040  /* Bit 6: Try 10BASE-TX full duplex */
#define MII_ADVERTISE_1000XHALF     0x0040  /* Bit 6: Try 1000BASE-X half duplex */
#define MII_ADVERTISE_100BASETXHALF 0x0080  /* Bit 7: Try 100BASE-TX half duplex */
#define MII_ADVERTISE_1000XPAUSE    0x0080  /* Bit 7: Try 1000BASE-X pause */
#define MII_ADVERTISE_100BASETXFULL 0x0100  /* Bit 8: Try 100BASE-TX full duplex*/
#define MII_ADVERTISE_1000XASYMPAU  0x0100  /* Bit 8: Try 1000BASE-X asym pause */
#define MII_ADVERTISE_100BASET4     0x0200  /* Bit 9: Try 100BASE-T4 */
#define MII_ADVERTISE_FDXPAUSE      0x0400  /* Bit 10: Try full duplex flow control */
#define MII_ADVERTISE_ASYMPAUSE     0x0800  /* Bit 11: Try asymetric pause */
#define MII_ADVERTISE_RFAULT        0x2000  /* Bit 13: Remote fault supported */
#define MII_ADVERTISE_LPACK         0x4000  /* Bit 14: Ack link partners response */
#define MII_ADVERTISE_NXTPAGE       0x8000  /* Bit 15: Next page enabled */

/* Link partner ability register bit definitions */

#define MII_LPA_SELECT              0x001f  /* Bits 0-4: Link partner selector field */
#define MII_LPA_10BASETXHALF        0x0020  /* Bit 5: 10BASE-TX half duplex able */
#define MII_LPA_1000XFULL           0x0020  /* Bit 5: 1000BASE-X full-duplex able */
#define MII_LPA_10BASETXFULL        0x0040  /* Bit 6: 10BASE-TX full duplex able */
#define MII_LPA_1000XHALF           0x0040  /* Bit 6: 1000BASE-X half-duplex */
#define MII_LPA_100BASETXHALF       0x0080  /* Bit 7: 100BASE-TX half duplex able */
#define MII_LPA_1000XPAUSE          0x0080  /* Bit 7: 1000BASE-X pause able */
#define MII_LPA_100BASETXFULL       0x0100  /* Bit 8: 100BASE-TX full duplex able */
#define MII_LPA_1000XASYMPAU        0x0100  /* Bit 8: 1000BASE-X asym pause able */
#define MII_LPA_100BASET4           0x0200  /* Bit 9: 100BASE-T4 able */
#define MII_LPA_FDXPAUSE            0x0400  /* Bit 10: Full duplex flow control able */
#define MII_LPA_ASYMPAUSE           0x0800  /* Bit 11: Asynchronous pause able */
#define MII_LPA_RFAULT              0x2000  /* Bit 13: Link partner remote fault request */
#define MII_LPA_LPACK               0x4000  /* Bit 14: Link partner acknowledgement */
#define MII_LPA_NXTPAGE             0x8000  /* Bit 15: Next page requested */

/* Link partnter ability in next page format */

#define MII_LPANP_MESSAGE           0x07ff  /* Bits 0-10: Link partner's message code */
#define MII_LPANP_TOGGLE            0x0800  /* Bit 11: Link partner toggle */
#define MII_LPANP_LACK2             0x1000  /* Bit 12: Link partner can comply ACK */
#define MII_LPANP_MSGPAGE           0x2000  /* Bit 13: Link partner message page request */
#define MII_LPANP_LPACK             0x4000  /* Bit 14: Link partner acknowledgement */
#define MII_LPANP_NXTPAGE           0x8000  /* Bit 15: Next page requested */

/* MII Auto-negotiation expansion register bit definitions */

#define MII_EXPANSION_ANEGABLE      0x0001  /* Bit 0: Link partner is auto-negotion able */
#define MII_EXPANSION_PAGERECVD     0x0002  /* Bit 1: New link code word in LPA ability reg */
#define MII_EXPANSION_ENABLENPAGE   0x0004  /* Bit 2: This enables npage words */
#define MII_EXPANSION_NXTPAGEABLE   0x0008  /* Bit 3: Link partner supports next page */
#define MII_EXPANSION_PARFAULTS     0x0010  /* Bit 4: Fault detected by parallel logic */

/* Auto-negotiation next page advertisement */

#define MII_NPADVERTISE_CODE        0x07ff  /* Bits 0-10: message/un-formated code field */
#define MII_NPADVERTISE_TOGGLE      0x0800  /* Bit 11: Toggle */
#define MII_NPADVERTISE_ACK2        0x1000  /* Bit 12: Acknowledgement 2 */
#define MII_NPADVERTISE_MSGPAGE     0x2000  /* Bit 13: Message page */
#define MII_NPADVERTISE_NXTPAGE     0x8000  /* Bit 15: Next page indication */

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
