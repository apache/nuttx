/****************************************************************************
 * arch/arm/src/armv8-m/itm.h
 *
 *   Copyright (c) 2009 - 2013 ARM LIMITED
 *
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ARM nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS
 *  AND CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *   Copyright (C) 2014 Pierre-noel Bouteville . All rights reserved.
 *   Author: Pierre-noel Bouteville <pnb990@gmail.com>
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

#ifndef __ARCH_ARM_SRC_ARMV8_M_ITM_H
#define __ARCH_ARM_SRC_ARMV8_M_ITM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Instrumentation Trace Macrocell Register (ITM) Definitions ***************/

/* ITM Register Base Address ************************************************/

#define ITM_BASE                 (0xe0000000ul)

/* ITM Register Addresses ***************************************************/

#define ITM_PORT(i)              (ITM_BASE + (i * 4)) /* Stimulus Port 32-bit */
#define ITM_TER                  (ITM_BASE + 0x0e00)  /* Trace Enable Register */
#define ITM_TPR                  (ITM_BASE + 0x0e40)  /* Trace Privilege Register */
#define ITM_TCR                  (ITM_BASE + 0x0e80)  /* Trace Control Register */
#define ITM_IWR                  (ITM_BASE + 0x0ef8)  /* Integration Write Register */
#define ITM_IRR                  (ITM_BASE + 0x0efc)  /* Integration Read Register */
#define ITM_IMCR                 (ITM_BASE + 0x0f00)  /* Integration Mode Control Register */
#define ITM_LAR                  (ITM_BASE + 0x0fb0)  /* Lock Access Register */
#define ITM_LSR                  (ITM_BASE + 0x0fb4)  /* Lock Status Register */
#define ITM_PID4                 (ITM_BASE + 0x0fd0)  /* Peripheral Identification Register #4 */
#define ITM_PID5                 (ITM_BASE + 0x0fd4)  /* Peripheral Identification Register #5 */
#define ITM_PID6                 (ITM_BASE + 0x0fd8)  /* Peripheral Identification Register #6 */
#define ITM_PID7                 (ITM_BASE + 0x0fdc)  /* Peripheral Identification Register #7 */
#define ITM_PID0                 (ITM_BASE + 0x0fe0)  /* Peripheral Identification Register #0 */
#define ITM_PID1                 (ITM_BASE + 0x0fe4)  /* Peripheral Identification Register #1 */
#define ITM_PID2                 (ITM_BASE + 0x0fe8)  /* Peripheral Identification Register #2 */
#define ITM_PID3                 (ITM_BASE + 0x0fec)  /* Peripheral Identification Register #3 */
#define ITM_CID0                 (ITM_BASE + 0x0ff0)  /* Component  Identification Register #0 */
#define ITM_CID1                 (ITM_BASE + 0x0ff4)  /* Component  Identification Register #1 */
#define ITM_CID2                 (ITM_BASE + 0x0ff8)  /* Component  Identification Register #2 */
#define ITM_CID3                 (ITM_BASE + 0x0ffc)  /* Component  Identification Register #3 */

/* ITM Register Bit Field Definitions ***************************************/

/* ITM TPR */

#define ITM_TPR_PRIVMASK_SHIFT   0
#define ITM_TPR_PRIVMASK_MASK    (0xful << ITM_TPR_PRIVMASK_SHIFT)

/* ITM TCR */

#define ITM_TCR_BUSY_SHIFT       23
#define ITM_TCR_BUSY_MASK        (1ul << ITM_TCR_BUSY_SHIFT)
#define ITM_TCR_TraceBusID_SHIFT 16
#define ITM_TCR_TraceBusID_MASK  (0x7ful << ITM_TCR_TraceBusID_SHIFT)
#define ITM_TCR_GTSFREQ_SHIFT    10
#define ITM_TCR_GTSFREQ_MASK     (3ul << ITM_TCR_GTSFREQ_SHIFT)
#define ITM_TCR_TSPrescale_SHIFT 8
#define ITM_TCR_TSPrescale_MASK  (3ul << ITM_TCR_TSPrescale_SHIFT)
#define ITM_TCR_SWOENA_SHIFT     4
#define ITM_TCR_SWOENA_MASK      (1ul << ITM_TCR_SWOENA_SHIFT)
#define ITM_TCR_DWTENA_SHIFT     3
#define ITM_TCR_DWTENA_MASK      (1ul << ITM_TCR_DWTENA_SHIFT)
#define ITM_TCR_SYNCENA_SHIFT    2
#define ITM_TCR_SYNCENA_MASK     (1ul << ITM_TCR_SYNCENA_SHIFT)
#define ITM_TCR_TSENA_SHIFT      1
#define ITM_TCR_TSENA_MASK       (1ul << ITM_TCR_TSENA_SHIFT)
#define ITM_TCR_ITMENA_SHIFT     0
#define ITM_TCR_ITMENA_MASK      (1ul << ITM_TCR_ITMENA_SHIFT)

/* ITM IWR */

#define ITM_IWR_ATVALIDM_SHIFT   0
#define ITM_IWR_ATVALIDM_MASK    (1ul << ITM_IWR_ATVALIDM_SHIFT)

/* ITM IRR */

#define ITM_IRR_ATREADYM_SHIFT   0
#define ITM_IRR_ATREADYM_MASK    (1ul << ITM_IRR_ATREADYM_SHIFT)

/* ITM IMCR */

#define ITM_IMCR_INTEGRATION_SHIFT 0
#define ITM_IMCR_INTEGRATION_MASK  (1ul << ITM_IMCR_INTEGRATION_SHIFT)

/* ITM LSR */

#define ITM_LSR_ByteAcc_SHIFT    2
#define ITM_LSR_ByteAcc_MASK     (1ul << ITM_LSR_ByteAcc_SHIFT)
#define ITM_LSR_Access_SHIFT     1
#define ITM_LSR_Access_MASK      (1ul << ITM_LSR_Access_SHIFT)
#define ITM_LSR_Present_SHIFT    0
#define ITM_LSR_Present_MASK     (1ul << ITM_LSR_Present_SHIFT)

/* Value identifying g_itm_rxbuffer is ready for next character. */

#define ITM_RXBUFFER_EMPTY       0x5aa55aa5

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

extern volatile int32_t g_itm_rxbuffer; /* External variable to receive characters. */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

uint32_t itm_sendchar(uint32_t ch);
int32_t itm_receivechar(void);
int32_t itm_checkchar(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_ARMV8_M_ITM_H */
