/****************************************************************************
 * arch/arm/src/cxd56xx/hardware/cxd56_rtc.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_RTC_H
#define __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/cxd5602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RTC_WRREGPOSTCNT                (0x0)
#define RTC_WRREGPRECNT                 (0x4)
#define RTC_WRREGREQ                    (0x8)
#define RTC_WRINTPOSTCNT                (0x10)
#define RTC_WRINTPRECNT                 (0x14)
#define RTC_WRINTCTRL                   (0x18)
#define RTC_WRINTCLR                    (0x1c)
#define RTC_OFFSETVAL                   (0x20)
#define RTC_OFFSETREQ                   (0x24)
#define RTC_RDREQ                       (0x30)
#define RTC_RDPOSTCNT                   (0x34)
#define RTC_RDPRECNT                    (0x38)
#define RTC_RTPOSTCNT                   (0x40)
#define RTC_RTPRECNT                    (0x44)
#define RTC_SETALMPOSTCNT(id)           (0x50 + ((id) * 0x8))
#define RTC_SETALMPRECNT(id)            (0x54 + ((id) * 0x8))
#define RTC_SETALMPOSTCNT0              (0x50)
#define RTC_SETALMPRECNT0               (0x54)
#define RTC_SETALMPOSTCNT1              (0x58)
#define RTC_SETALMPRECNT1               (0x5c)
#define RTC_SETALMPOSTCNT2              (0x60)
#define RTC_SETALMPRECNT2               (0x64)
#define RTC_ALMCLR                      (0x70)
#define RTC_ALMOUTEN(id)                (0x74 + ((id) * 0x4))
#define RTC_ALMOUTEN0                   (0x74)
#define RTC_ALMOUTEN1                   (0x78)
#define RTC_ALMOUTEN2                   (0x7c)
#define RTC_ALMFLG                      (0x80)
#define RTC_DBGSETALMPOSTCNT0           (0x90)
#define RTC_DBGSETALMPRECNT0            (0x94)
#define RTC_DBGSETALMPOSTCNT1           (0x98)
#define RTC_DBGSETALMPRECNT1            (0x9c)
#define RTC_DBGSETALMPOSTCNT2           (0xa0)
#define RTC_DBGSETALMPRECNT2            (0xa4)

/* Register Addresses *******************************************************/

#define CXD56_RTC0_WRREGPOSTCNT         (CXD56_RTC0_BASE + RTC_WRREGPOSTCNT)
#define CXD56_RTC0_WRREGPRECNT          (CXD56_RTC0_BASE + RTC_WRREGPRECNT)
#define CXD56_RTC0_WRREGREQ             (CXD56_RTC0_BASE + RTC_WRREGREQ)
#define CXD56_RTC0_WRINTPOSTCNT         (CXD56_RTC0_BASE + RTC_WRINTPOSTCNT)
#define CXD56_RTC0_WRINTPRECNT          (CXD56_RTC0_BASE + RTC_WRINTPRECNT)
#define CXD56_RTC0_WRINTCTRL            (CXD56_RTC0_BASE + RTC_WRINTCTRL)
#define CXD56_RTC0_WRINTCLR             (CXD56_RTC0_BASE + RTC_WRINTCLR)
#define CXD56_RTC0_OFFSETVAL            (CXD56_RTC0_BASE + RTC_OFFSETVAL)
#define CXD56_RTC0_OFFSETREQ            (CXD56_RTC0_BASE + RTC_OFFSETREQ)
#define CXD56_RTC0_RDREQ                (CXD56_RTC0_BASE + RTC_RDREQ)
#define CXD56_RTC0_RDPOSTCNT            (CXD56_RTC0_BASE + RTC_RDPOSTCNT)
#define CXD56_RTC0_RDPRECNT             (CXD56_RTC0_BASE + RTC_RDPRECNT)
#define CXD56_RTC0_RTPOSTCNT            (CXD56_RTC0_BASE + RTC_RTPOSTCNT)
#define CXD56_RTC0_RTPRECNT             (CXD56_RTC0_BASE + RTC_RTPRECNT)
#define CXD56_RTC0_SETALMPOSTCNT(id)    (CXD56_RTC0_BASE + RTC_SETALMPOSTCNT(id))
#define CXD56_RTC0_SETALMPRECNT(id)     (CXD56_RTC0_BASE + RTC_SETALMPRECNT(id))
#define CXD56_RTC0_ALMCLR               (CXD56_RTC0_BASE + RTC_ALMCLR)
#define CXD56_RTC0_ALMOUTEN(id)         (CXD56_RTC0_BASE + RTC_ALMOUTEN(id))
#define CXD56_RTC0_ALMFLG               (CXD56_RTC0_BASE + RTC_ALMFLG)

/* Register bit definitions *************************************************/

/* Flag/Clear Register */

#define RTCREG_ALM0_FLAG_MASK           (1u << 0)
#define RTCREG_ALM1_FLAG_MASK           (1u << 1)
#define RTCREG_ALM2_FLAG_MASK           (1u << 2)
#define RTCREG_ALM0_ERR_FLAG_MASK       (1u << 16)
#define RTCREG_ALM1_ERR_FLAG_MASK       (1u << 17)

#define RTCREG_ALM0_MASK        (RTCREG_ALM0_FLAG_MASK | RTCREG_ALM0_ERR_FLAG_MASK)
#define RTCREG_ALM1_MASK        (RTCREG_ALM1_FLAG_MASK | RTCREG_ALM1_ERR_FLAG_MASK)
#define RTCREG_ALM2_MASK        (RTCREG_ALM2_FLAG_MASK)

/* Write Request */

#define RTCREG_WREQ_BUSYA_MASK          (1u << 0)
#define RTCREG_WREQ_BUSYB_MASK          (1u << 1)

/* Alarm Request */

#define RTCREG_ASET_BUSY_MASK           (1u << 16)

/* Alarm Enable */

#define RTCREG_ALM_EN_MASK              (1u << 0)
#define RTCREG_ALM_BUSY_MASK            (1u << 8)
#define RTCREG_ALM_DBG_MASK             (1u << 15)
#define RTCREG_ALM_ERREN_MASK           (1u << 16)
#define RTCREG_ALM_ERRDBG_MASK          (1u << 31)

#endif /* __ARCH_ARM_SRC_CXD56XX_HARDWARE_CXD56_RTC_H */
