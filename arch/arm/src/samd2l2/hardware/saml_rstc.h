/********************************************************************************************
 * arch/arm/src/samd2l2/hardware/saml_rstc.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Atmel SAM L21E / SAM L21G / SAM L21J Smart ARM-Based Microcontroller
 *   Datasheet", Atmel-42385C-SAML21_Datasheet_Preliminary-03/20/15
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

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_RSTC_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_RSTC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* RSTC register offsets *********************************************************************/

#define SAM_RSTC_RCAUSE_OFFSET         0x0000  /* Reset cause */
#define SAM_RSTC_BKUPEXIT_OFFSET       0x0002  /* Backup exit source */
#define SAM_RSTC_WKDBCONF_OFFSET       0x0004  /* Wakeup debounce count */
#define SAM_RSTC_WKPOL_OFFSET          0x0008  /* Wakeup polarity */
#define SAM_RSTC_WKEN_OFFSET           0x000c  /* Wakeup enable */
#define SAM_RSTC_WKCAUSE_OFFSET        0x0010  /* Wakeup cause */

/* RSTC register addresses *******************************************************************/

#define SAM_RSTC_RCAUSE                (SAM_RSTC_BASE+SAM_RSTC_RCAUSE_OFFSET)
#define SAM_RSTC_BKUPEXIT              (SAM_RSTC_BASE+SAM_RSTC_BKUPEXIT_OFFSET)
#define SAM_RSTC_WKDBCONF              (SAM_RSTC_BASE+SAM_RSTC_WKDBCONF_OFFSET)
#define SAM_RSTC_WKPOL                 (SAM_RSTC_BASE+SAM_RSTC_WKPOL_OFFSET)
#define SAM_RSTC_WKEN                  (SAM_RSTC_BASE+SAM_RSTC_WKEN_OFFSET)
#define SAM_RSTC_WKCAUSE               (SAM_RSTC_BASE+SAM_RSTC_WKCAUSE_OFFSET)

/* RSTC register bit definitions *************************************************************/

/* Reset cause */

#define RSTC_RCAUSE_POR                (1 << 0)  /* Bit 0: Power on reset */
#define RSTC_RCAUSE_BOD12              (1 << 1)  /* Bit 1: Brown out 12 detector reset */
#define RSTC_RCAUSE_BOD33              (1 << 2)  /* Bit 2: Brown out 33 detector reset */
#define RSTC_RCAUSE_EXT                (1 << 4)  /* Bit 4: External reset */
#define RSTC_RCAUSE_WDT                (1 << 5)  /* Bit 5: Watchdog reset */
#define RSTC_RCAUSE_SYST               (1 << 6)  /* Bit 6: System reset request */
#define RSTC_RCAUSE_BACKUP             (1 << 7)  /* Bit 7: Backup reset*/

/* Backup exit source */

#define RSTC_BKUPEXIT_EXTWAKE          (1 << 0)  /* Bit 0: External wakeup */
#define RSTC_BKUPEXIT_RTC              (1 << 1)  /* Bit 1: Real time counter interrupt */
#define RSTC_BKUPEXIT_BBPS             (1 << 2)  /* Bit 2: Battery backup power switch */

/* Wakeup debounce count */

#define RSTC_WKDBCONF_MASK             (0x1f)

/* Wakeup polarity */

#define RSTC_WKPOL_PIN(n)               (1 << (n)) /* Bit n:Input pin n active high */
#  define RSTC_WKPOL_PIN0               (1 << 0)   /* Bit 0: Input pin 0 active high */
#  define RSTC_WKPOL_PIN1               (1 << 1)   /* Bit 1: Input pin 1 active high */
#  define RSTC_WKPOL_PIN2               (1 << 2)   /* Bit 2: Input pin 2 active high */
#  define RSTC_WKPOL_PIN3               (1 << 3)   /* Bit 3: Input pin 3 active high */
#  define RSTC_WKPOL_PIN4               (1 << 4)   /* Bit 4: Input pin 4 active high */
#  define RSTC_WKPOL_PIN5               (1 << 5)   /* Bit 5: Input pin 5 active high */
#  define RSTC_WKPOL_PIN6               (1 << 6)   /* Bit 6: Input pin 6 active high */
#  define RSTC_WKPOL_PIN7               (1 << 7)   /* Bit 7: Input pin 7 active high */

/* Wakeup enable */

#define RSTC_WKEN_PIN(n)                (1 << (n)) /* Bit n: Wakeup input pin n enabled */
#  define RSTC_WKEN_PIN0                (1 << 0)   /* Bit 0: Wakeup input pin 0 enabled */
#  define RSTC_WKEN_PIN1                (1 << 1)   /* Bit 1: Wakeup input pin 1 enabled */
#  define RSTC_WKEN_PIN2                (1 << 2)   /* Bit 2: Wakeup input pin 2 enabled */
#  define RSTC_WKEN_PIN3                (1 << 3)   /* Bit 3: Wakeup input pin 3 enabled */
#  define RSTC_WKEN_PIN4                (1 << 4)   /* Bit 4: Wakeup input pin 4 enabled */
#  define RSTC_WKEN_PIN5                (1 << 5)   /* Bit 5: Wakeup input pin 5 enabled */
#  define RSTC_WKEN_PIN6                (1 << 6)   /* Bit 6: Wakeup input pin 6 enabled */
#  define RSTC_WKEN_PIN7                (1 << 7)   /* Bit 7: Wakeup input pin 7 enabled */

/* Wakeup cause */

#define RSTC_WKCAUSE_PIN(n)             (1 << (n)) /* Bit n: WKCAUSE n active and enabled */
#  define RSTC_WKCAUSE_PIN0             (1 << 0)   /* Bit 0: WKCAUSE 0 active and enabled */
#  define RSTC_WKCAUSE_PIN1             (1 << 1)   /* Bit 1: WKCAUSE 1 active and enabled */
#  define RSTC_WKCAUSE_PIN2             (1 << 2)   /* Bit 2: WKCAUSE 2 active and enabled */
#  define RSTC_WKCAUSE_PIN3             (1 << 3)   /* Bit 3: WKCAUSE 3 active and enabled */
#  define RSTC_WKCAUSE_PIN4             (1 << 4)   /* Bit 4: WKCAUSE 4 active and enabled */
#  define RSTC_WKCAUSE_PIN5             (1 << 5)   /* Bit 5: WKCAUSE 5 active and enabled */
#  define RSTC_WKCAUSE_PIN6             (1 << 6)   /* Bit 6: WKCAUSE 6 active and enabled */
#  define RSTC_WKCAUSE_PIN7             (1 << 7)   /* Bit 7: WKCAUSE 7 active and enabled */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_RSTC_H */
