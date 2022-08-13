/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_adc.h
 *
 * Generated from rp2040.svd originally provided by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_ADC_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP2040_ADC_CS_OFFSET           0x000000  /* ADC Control and Status register */
#define RP2040_ADC_RESULT_OFFSET       0x000004  /* ADC Results register */
#define RP2040_ADC_FCS_OFFSET          0x000008  /* ADC FIFO Control and Status register */
#define RP2040_ADC_FIFO_OFFSET         0x00000c  /* ADC Result FIFO */
#define RP2040_ADC_DIV_OFFSET          0x000010  /* ADC Clock Divider register */
#define RP2040_ADC_INTR_OFFSET         0x000014  /* ADC Raw Interrupts register */
#define RP2040_ADC_INTE_OFFSET         0x000018  /* ADC Interrupt Enable register */
#define RP2040_ADC_INTF_OFFSET         0x00001c  /* ADC Interrupt Force register */
#define RP2040_ADC_INTS_OFFSET         0x000020  /* ADC Interrupt Status register */

/* Register definitions *****************************************************/

#define  RP2040_ADC_CS                 (RP2040_ADC_BASE + RP2040_ADC_CS_OFFSET)
#define  RP2040_ADC_RESULT             (RP2040_ADC_BASE + RP2040_ADC_RESULT_OFFSET)
#define  RP2040_ADC_FCS                (RP2040_ADC_BASE + RP2040_ADC_FCS_OFFSET)
#define  RP2040_ADC_FIFO               (RP2040_ADC_BASE + RP2040_ADC_FIFO_OFFSET)
#define  RP2040_ADC_DIV                (RP2040_ADC_BASE + RP2040_ADC_DIV_OFFSET)
#define  RP2040_ADC_INTR               (RP2040_ADC_BASE + RP2040_ADC_INTR_OFFSET)
#define  RP2040_ADC_INTE               (RP2040_ADC_BASE + RP2040_ADC_INTE_OFFSET)
#define  RP2040_ADC_INTF               (RP2040_ADC_BASE + RP2040_ADC_INTF_OFFSET)
#define  RP2040_ADC_INTS               (RP2040_ADC_BASE + RP2040_ADC_INTS_OFFSET)

/* Register bit definitions *************************************************/

#define RP2040_ADC_CS_RROBIN_SHIFT     (16)
#define RP2040_ADC_CS_RROBIN_MASK      (0x001fl << RPC2040_ADC_CS_RROBIN_SHIFT)
#define RP2040_ADC_CS_AINSEL_SHIFT     (12)
#define RP2040_ADC_CS_AINSEL_MASK      (0x0007l << RPC2040_ADC_CS_AINSEL_SHIFT)
#define RP2040_ADC_CS_ERR_STICKY       (1 << 10)
#define RP2040_ADC_CS_ERR              (1 <<  9)
#define RP2040_ADC_CS_READY            (1 <<  8)
#define RP2040_ADC_CS_START_MANY       (1 <<  3)
#define RP2040_ADC_CS_START_ONCE       (1 <<  2)
#define RP2040_ADC_CS_TS_ENA           (1 <<  1)
#define RP2040_ADC_CS_EN               (1 <<  0)

#define RP2040_ADC_RESULT_VAL_MASK     (0x00000fffl)

#define RP2040_ADC_FCS_THRESH_SHIFT    (24)
#define RP2040_ADC_FCS_THRESH_MASK     (0x000fl << RP2040_ADC_FCS_THRESH_SHIFT)
#define RP2040_ADC_FCS_LEVEL_SHIFT     (16)
#define RP2040_ADC_FCS_LEVEL_MASK      (0x000fl << RP2040_ADC_FCS_LEVEL_SHIFT)
#define RP2040_ADC_FCS_OVER            (1 << 11)
#define RP2040_ADC_FCS_UNDER           (1 << 10)
#define RP2040_ADC_FCS_FULL            (1 <<  9)
#define RP2040_ADC_FCS_EMPTY           (1 <<  8)
#define RP2040_ADC_FCS_DREQ_EN         (1 <<  3)
#define RP2040_ADC_FCS_ERR             (1 <<  2)
#define RP2040_ADC_FCS_SHIFT           (1 <<  1)
#define RP2040_ADC_FCS_EN              (1 <<  0)

#define RP2040_ADC_FIFO_ERR            (1 << 15)
#define RP2040_ADC_FIFO_VAL_MASK       (0x0fffl)

#define RP2040_ADC_DIV_INT_SHIFT       (8)
#define RP2040_ADC_DIV_INT_MASK        (0x0fffl << RP2040_ADC_DIV_INT_SHIFT)
#define RP2040_ADC_DIV_FRAC_SHIFT      (0)
#define RP2040_ADC_DIV_FRAC_MASK       (0x00ffl << RP2040_ADC_DIV_FRAC_MASK)

#define RP2040_ADC_INTR_FIFO           (1 << 0)  /* Raw interrupt status */

#define RP2040_ADC_INTE_FIFO           (1 << 0)  /* Set to pass interrupts */

#define RP2040_ADC_INTF_FIFO           (1 << 0)  /* Write 1 to force the interrupt. */

#define RP2040_ADC_INTS_FIFO           (1 << 0)  /* Masked interrupt status */

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_ADC_H */
