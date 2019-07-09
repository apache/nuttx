/********************************************************************************************
 * arch/arm/src/samd2l2/hardware/samd_tc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Matt Thompson <matt@extent3d.com>
 *
 * References:
 *   "Microchip SAMD21 datasheet"
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_TC_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_TC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAMD21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* TC register offsets *********************************************************************/

#define SAM_TC_CTRLA_OFFSET          0x0000  /* Control A register */
#define SAM_TC_READREQ_OFFSET        0x0002  /* Read request register */
#define SAM_TC_CTRLBCLR_OFFSET       0x0004  /* Control B clear register */
#define SAM_TC_CTRLBSET_OFFSET       0x0005  /* Control B clear register */
#define SAM_TC_CTRLC_OFFSET          0x0006  /* Control C register */
#define SAM_TC_DBGCTRL_OFFSET        0x0008  /* Debug control register */
#define SAM_TC_EVCTRL_OFFSET         0x000A  /* Event control register */
#define SAM_TC_INTENCLR_OFFSET       0x000C  /* Interrupt enable clear register */
#define SAM_TC_INTENSET_OFFSET       0x000D  /* Interrupt enable set register */
#define SAM_TC_INTFLAG_OFFSET        0x000E  /* Interrupt flag register */
#define SAM_TC_STATUS_OFFSET         0x000F  /* Status register */
#define SAM_TC_COUNT_OFFSET          0x0010  /* Count register */
#define SAM_TC_CC0_OFFSET            0x0018  /* Capture Compare 0 register */
#define SAM_TC_CC1_OFFSET            0x001C  /* Capture Compare 1 register */

/* TC register addresses *******************************************************************/

#define SAM_TC3_CTRLA                 (SAM_TC3_BASE+SAM_TC_CTRLA_OFFSET)
#define SAM_TC3_READREQ               (SAM_TC3_BASE+SAM_TC_READREQ_OFFSET)
#define SAM_TC3_CTRLBCLR              (SAM_TC3_BASE+SAM_TC_CTRLBCLR_OFFSET)
#define SAM_TC3_CTRLBSET              (SAM_TC3_BASE+SAM_TC_CTRLBSET_OFFSET)
#define SAM_TC3_CTRLC                 (SAM_TC3_BASE+SAM_TC_CTRLC_OFFSET)
#define SAM_TC3_DBGCTRL               (SAM_TC3_BASE+SAM_TC_DBGCTRL_OFFSET)
#define SAM_TC3_EVCTRL                (SAM_TC3_BASE+SAM_TC_EVCTRL_OFFSET)
#define SAM_TC3_INTENCLR              (SAM_TC3_BASE+SAM_TC_INTENCLR_OFFSET)
#define SAM_TC3_INTENSET              (SAM_TC3_BASE+SAM_TC_INTENSET_OFFSET)
#define SAM_TC3_INTFLAG               (SAM_TC3_BASE+SAM_TC_INTFLAG_OFFSET)
#define SAM_TC3_STATUS                (SAM_TC3_BASE+SAM_TC_STATUS_OFFSET)
#define SAM_TC3_COUNT                 (SAM_TC3_BASE+SAM_TC_COUNT_OFFSET)
#define SAM_TC3_CC0                   (SAM_TC3_BASE+SAM_TC_CC0_OFFSET)
#define SAM_TC3_CC1                   (SAM_TC3_BASE+SAM_TC_CC1_OFFSET)

#define SAM_TC4_CTRLA                 (SAM_TC4_BASE+SAM_TC_CTRLA_OFFSET)
#define SAM_TC4_READREQ               (SAM_TC4_BASE+SAM_TC_READREQ_OFFSET)
#define SAM_TC4_CTRLBCLR              (SAM_TC4_BASE+SAM_TC_CTRLBCLR_OFFSET)
#define SAM_TC4_CTRLBSET              (SAM_TC4_BASE+SAM_TC_CTRLBSET_OFFSET)
#define SAM_TC4_CTRLC                 (SAM_TC4_BASE+SAM_TC_CTRLC_OFFSET)
#define SAM_TC4_DBGCTRL               (SAM_TC4_BASE+SAM_TC_DBGCTRL_OFFSET)
#define SAM_TC4_EVCTRL                (SAM_TC4_BASE+SAM_TC_EVCTRL_OFFSET)
#define SAM_TC4_INTENCLR              (SAM_TC4_BASE+SAM_TC_INTENCLR_OFFSET)
#define SAM_TC4_INTENSET              (SAM_TC4_BASE+SAM_TC_INTENSET_OFFSET)
#define SAM_TC4_INTFLAG               (SAM_TC4_BASE+SAM_TC_INTFLAG_OFFSET)
#define SAM_TC4_STATUS                (SAM_TC4_BASE+SAM_TC_STATUS_OFFSET)
#define SAM_TC4_COUNT                 (SAM_TC4_BASE+SAM_TC_COUNT_OFFSET)
#define SAM_TC4_CC0                   (SAM_TC4_BASE+SAM_TC_CC0_OFFSET)
#define SAM_TC4_CC1                   (SAM_TC4_BASE+SAM_TC_CC1_OFFSET)

#define SAM_TC5_CTRLA                 (SAM_TC5_BASE+SAM_TC_CTRLA_OFFSET)
#define SAM_TC5_READREQ               (SAM_TC5_BASE+SAM_TC_READREQ_OFFSET)
#define SAM_TC5_CTRLBCLR              (SAM_TC5_BASE+SAM_TC_CTRLBCLR_OFFSET)
#define SAM_TC5_CTRLBSET              (SAM_TC5_BASE+SAM_TC_CTRLBSET_OFFSET)
#define SAM_TC5_CTRLC                 (SAM_TC5_BASE+SAM_TC_CTRLC_OFFSET)
#define SAM_TC5_DBGCTRL               (SAM_TC5_BASE+SAM_TC_DBGCTRL_OFFSET)
#define SAM_TC5_EVCTRL                (SAM_TC5_BASE+SAM_TC_EVCTRL_OFFSET)
#define SAM_TC5_INTENCLR              (SAM_TC5_BASE+SAM_TC_INTENCLR_OFFSET)
#define SAM_TC5_INTENSET              (SAM_TC5_BASE+SAM_TC_INTENSET_OFFSET)
#define SAM_TC5_INTFLAG               (SAM_TC5_BASE+SAM_TC_INTFLAG_OFFSET)
#define SAM_TC5_STATUS                (SAM_TC5_BASE+SAM_TC_STATUS_OFFSET)
#define SAM_TC5_COUNT                 (SAM_TC5_BASE+SAM_TC_COUNT_OFFSET)
#define SAM_TC5_CC0                   (SAM_TC5_BASE+SAM_TC_CC0_OFFSET)
#define SAM_TC5_CC1                   (SAM_TC5_BASE+SAM_TC_CC1_OFFSET)

#define SAM_TC6_CTRLA                 (SAM_TC6_BASE+SAM_TC_CTRLA_OFFSET)
#define SAM_TC6_READREQ               (SAM_TC6_BASE+SAM_TC_READREQ_OFFSET)
#define SAM_TC6_CTRLBCLR              (SAM_TC6_BASE+SAM_TC_CTRLBCLR_OFFSET)
#define SAM_TC6_CTRLBSET              (SAM_TC6_BASE+SAM_TC_CTRLBSET_OFFSET)
#define SAM_TC6_CTRLC                 (SAM_TC6_BASE+SAM_TC_CTRLC_OFFSET)
#define SAM_TC6_DBGCTRL               (SAM_TC6_BASE+SAM_TC_DBGCTRL_OFFSET)
#define SAM_TC6_EVCTRL                (SAM_TC6_BASE+SAM_TC_EVCTRL_OFFSET)
#define SAM_TC6_INTENCLR              (SAM_TC6_BASE+SAM_TC_INTENCLR_OFFSET)
#define SAM_TC6_INTENSET              (SAM_TC6_BASE+SAM_TC_INTENSET_OFFSET)
#define SAM_TC6_INTFLAG               (SAM_TC6_BASE+SAM_TC_INTFLAG_OFFSET)
#define SAM_TC6_STATUS                (SAM_TC6_BASE+SAM_TC_STATUS_OFFSET)
#define SAM_TC6_COUNT                 (SAM_TC6_BASE+SAM_TC_COUNT_OFFSET)
#define SAM_TC6_CC0                   (SAM_TC6_BASE+SAM_TC_CC0_OFFSET)
#define SAM_TC6_CC1                   (SAM_TC6_BASE+SAM_TC_CC1_OFFSET)

#define SAM_TC7_CTRLA                 (SAM_TC7_BASE+SAM_TC_CTRLA_OFFSET)
#define SAM_TC7_READREQ               (SAM_TC7_BASE+SAM_TC_READREQ_OFFSET)
#define SAM_TC7_CTRLBCLR              (SAM_TC7_BASE+SAM_TC_CTRLBCLR_OFFSET)
#define SAM_TC7_CTRLBSET              (SAM_TC7_BASE+SAM_TC_CTRLBSET_OFFSET)
#define SAM_TC7_CTRLC                 (SAM_TC7_BASE+SAM_TC_CTRLC_OFFSET)
#define SAM_TC7_DBGCTRL               (SAM_TC7_BASE+SAM_TC_DBGCTRL_OFFSET)
#define SAM_TC7_EVCTRL                (SAM_TC7_BASE+SAM_TC_EVCTRL_OFFSET)
#define SAM_TC7_INTENCLR              (SAM_TC7_BASE+SAM_TC_INTENCLR_OFFSET)
#define SAM_TC7_INTENSET              (SAM_TC7_BASE+SAM_TC_INTENSET_OFFSET)
#define SAM_TC7_INTFLAG               (SAM_TC7_BASE+SAM_TC_INTFLAG_OFFSET)
#define SAM_TC7_STATUS                (SAM_TC7_BASE+SAM_TC_STATUS_OFFSET)
#define SAM_TC7_COUNT                 (SAM_TC7_BASE+SAM_TC_COUNT_OFFSET)
#define SAM_TC7_CC0                   (SAM_TC7_BASE+SAM_TC_CC0_OFFSET)
#define SAM_TC7_CC1                   (SAM_TC7_BASE+SAM_TC_CC1_OFFSET)

/* TC register bit definitions *************************************************************/

/* Control A register */

#define TC_CTRLA_SWRST               (1 << 0)  /* Bit 0:  Software reset */
#define TC_CTRLA_ENABLE              (1 << 1)  /* Bit 1:  Enable */
#define TC_CTRLA_MODE_SHIFT          (2)
#define TC_CTRLA_MODE_MASK           (3 << TC_CTRLA_MODE_SHIFT)
#  define TC_CTRLA_MODE_COUNT16      (0 << TC_CTRLA_MODE_SHIFT)
#  define TC_CTRLA_MODE_COUNT8       (1 << TC_CTRLA_MODE_SHIFT)
#  define TC_CTRLA_MODE_COUNT32      (2 << TC_CTRLA_MODE_SHIFT)
#define TC_CTRLA_WAVEGEN_SHIFT       (5)
#define TC_CTRLA_WAVEGEN_MASK        (3 << TC_CTRLA_WAVEGEN_SHIFT)
#  define TC_CTRLA_WAVEGEN_NFRQ      (0 << TC_CTRLA_WAVEGEN_SHIFT)
#  define TC_CTRLA_WAVEGEN_MFRQ      (1 << TC_CTRLA_WAVEGEN_SHIFT)
#  define TC_CTRLA_WAVEGEN_NPWM      (2 << TC_CTRLA_WAVEGEN_SHIFT)
#  define TC_CTRLA_WAVEGEN_MPWM      (3 << TC_CTRLA_WAVEGEN_SHIFT)
#define TC_CTRLA_PRESCALER_SHIFT     (8)
#define TC_CTRLA_PRESCALER_MASK      (7 << TC_CTRLA_PRESCALER_SHIFT)
#  define TC_CTRLA_PRESCALER_DIV1    (0 << TC_CTRLA_PRESCALER_SHIFT)
#  define TC_CTRLA_PRESCALER_DIV2    (1 << TC_CTRLA_PRESCALER_SHIFT)
#  define TC_CTRLA_PRESCALER_DIV4    (2 << TC_CTRLA_PRESCALER_SHIFT)
#  define TC_CTRLA_PRESCALER_DIV8    (3 << TC_CTRLA_PRESCALER_SHIFT)
#  define TC_CTRLA_PRESCALER_DIV16   (4 << TC_CTRLA_PRESCALER_SHIFT)
#  define TC_CTRLA_PRESCALER_DIV64   (5 << TC_CTRLA_PRESCALER_SHIFT)
#  define TC_CTRLA_PRESCALER_DIV256  (6 << TC_CTRLA_PRESCALER_SHIFT)
#  define TC_CTRLA_PRESCALER_DIV1024 (7 << TC_CTRLA_PRESCALER_SHIFT)
#define TC_CTRLA_RUNSTDBY            (1 << 11)
#define TC_CTRLA_PRESCSYNC_SHIFT     (12)
#define TC_CTRLA_PRESCSYNC_MASK      (3 << TC_CTRLA_PRESCSYNC_SHIFT)
#  define TC_CTRLA_PRESCSYNC_GCLK    (0 << TC_CTRLA_PRESCSYNC_SHIFT)
#  define TC_CTRLA_PRESCSYNC_PRESC   (1 << TC_CTRLA_PRESCSYNC_SHIFT)
#  define TC_CTRLA_PRESCSYNC_RESYNC  (2 << TC_CTRLA_PRESCSYNC_SHIFT)

/* Read Request register */

#define TC_READREQ_ADDR_SHIFT        (0)
#define TC_READREQ_ADDR_MASK         (0x1F << TC_READREQ_ADDR_SHIFT)
#define TC_READREQ_RCONT             (1 << 14)
#define TC_READREQ_RREQ              (1 << 15)

/* Control B Set/Clear register */

#define TC_CTRLB_DIR                 (1 << 0)
#define TC_CTRLB_ONESHOT             (1 << 2)
#define TC_CTRLB_CMD_SHIFT           (6)
#define TC_CTRLB_CMD_MASK            (3 << TC_CTRLB_CMD_SHIFT)
#  define TC_CTRLB_CMD_NONE          (0 << TC_CTRLB_CMD_SHIFT)
#  define TC_CTRLB_CMD_RETRIGGER     (1 << TC_CTRLB_CMD_SHIFT)
#  define TC_CTRLB_CMD_STOP          (2 << TC_CTRLB_CMD_SHIFT)

/* Control C register */

#define TC_CTRLC_INVEN0              (1 << 0)
#define TC_CTRLC_INVEN1              (1 << 1)
#define TC_CTRLC_CPTEN0              (1 << 4)
#define TC_CTRLC_CPTEN1              (1 << 5)

/* Debug control register */

#define TC_DBGCTRL_DBGRUN            (1 << 0)

/* Event control register */

#define TC_EVCTRL_EVACT_SHIFT        (0)
#define TC_EVCTRL_EVACT_MASK         (7 << TC_EVCTRL_EVACT_SHIFT)
#  define TC_EVCTRL_EVACT_OFF        (0 << TC_EVCTRL_EVACT_SHIFT)
#  define TC_EVCTRL_EVACT_RETRIGGER  (1 << TC_EVCTRL_EVACT_SHIFT)
#  define TC_EVCTRL_EVACT_COUNT      (2 << TC_EVCTRL_EVACT_SHIFT)
#  define TC_EVCTRL_EVACT_START      (3 << TC_EVCTRL_EVACT_SHIFT)
#  define TC_EVCTRL_EVACT_PPW        (5 << TC_EVCTRL_EVACT_SHIFT)
#  define TC_EVCTRL_EVACT_PWP        (6 << TC_EVCTRL_EVACT_SHIFT)
#define TC_EVCTRL_TCINV              (1 << 4)
#define TC_EVCTRL_TCEI               (1 << 5)
#define TC_EVCTRL_OVFEO              (1 << 8)
#define TC_EVCTRL_MCEO0              (1 << 12)
#define TC_EVCTRL_MCEO1              (1 << 13)

/* Interrupt register bits */

#define TC_INT_OVF                   (1 << 0)
#define TC_INT_ERR                   (1 << 1)
#define TC_INT_SYNCRDY               (1 << 3)
#define TC_INT_MC0                   (1 << 4)
#define TC_INT_MC1                   (1 << 5)

/* Status register */

#define TC_STATUS_STOP               (1 << 3)
#define TC_STATUS_SLAVE              (1 << 4)
#define TC_STATUS_SYNCBUSY           (1 << 7)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAMD21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_TC_H */
