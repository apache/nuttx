/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_pwm.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_PWM_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Register offsets *********************************************************/

#define NRF54L_PWM_TASKS_STOP_OFFSET                  0x0004              /* Stop PWM */
#define NRF54L_PWM_TASKS_NEXTSTEP_OFFSET              0x0008              /* Steps by one value in the current sequence */
#define NRF54L_PWM_TASKS_DMA_SEQ_START_OFFSET(n)      (0x0010 + 8 * (n))  /* Start sequence n */
#define NRF54L_PWM_SUBSCRIBE_STOP_OFFSET              0x0084              /* Subscribe to STOP */
#define NRF54L_PWM_SUBSCRIBE_NEXTSTEP_OFFSET          0x0088              /* Subscribe to NEXTSTEP */
#define NRF54L_PWM_SUBSCRIBE_DMA_SEQ_START_OFFSET(n)  (0x0090 + 8 * (n))  /* Subscribe to sequence n START */
#define NRF54L_PWM_EVENTS_STOPPED_OFFSET              0x0104              /* STOP event */
#define NRF54L_PWM_EVENTS_SEQSTARTED0_OFFSET          0x0108              /* Sequence 0 started event */
#define NRF54L_PWM_EVENTS_SEQSTARTED1_OFFSET          0x010c              /* Sequence 1 started event */
#define NRF54L_PWM_EVENTS_SEQEND0_OFFSET              0x0110              /* Sequence 0 end event */
#define NRF54L_PWM_EVENTS_SEQEND1_OFFSET              0x0114              /* Sequence 1 end event */
#define NRF54L_PWM_EVENTS_PWMPERIODEND_OFFSET         0x0118              /* PWM period end event */
#define NRF54L_PWM_EVENTS_LOOPSDONE_OFFSET            0x011c              /* Loop done event */
#define NRF54L_PWM_EVENTS_RAMUNDERFLOW_OFFSET         0x0120              /* RAM underflow */
#define NRF54L_PWM_EVENTS_DMA_SEQ_END_OFFSET(n)       (0x0124 + 12 * (n)) /* Sequence n DMA buffer completed */
#define NRF54L_PWM_EVENTS_DMA_SEQ_READY_OFFSET(n)     (0x0128 + 12 * (n)) /* Sequence n DMA buffer prepared */
#define NRF54L_PWM_EVENTS_DMA_SEQ_BUSERROR_OFFSET(n)  (0x012c + 12 * (n)) /* Sequence n DMA bus error */
#define NRF54L_PWM_EVENTS_COMPAREMATCH_OFFSET(n)      (0x013c + 4 * (n))  /* Compare match on PWM channel n */
#define NRF54L_PWM_PUBLISH_STOPPED_OFFSET             0x0184              /* Publish STOPPED */
#define NRF54L_PWM_PUBLISH_SEQSTARTED_OFFSET(n)       (0x0188 + 4 * (n))  /* Publish sequence n started event */
#define NRF54L_PWM_PUBLISH_SEQEND_OFFSET(n)           (0x0190 + 4 * (n))  /* Publish sequence n end event */
#define NRF54L_PWM_PUBLISH_PWMPERIODEND_OFFSET        0x0198              /* Publish PWMPERIODEND */
#define NRF54L_PWM_PUBLISH_LOOPSDONE_OFFSET           0x019c              /* Publish LOOPSDONE */
#define NRF54L_PWM_PUBLISH_RAMUNDERFLOW_OFFSET        0x01a0              /* Publish RAMUNDERFLOW */
#define NRF54L_PWM_PUBLISH_DMA_SEQ_END_OFFSET(n)      (0x01a4 + 12 * (n)) /* Publish sequence n DMA END event */
#define NRF54L_PWM_PUBLISH_DMA_SEQ_READY_OFFSET(n)    (0x01a8 + 12 * (n)) /* Publish sequence n DMA READY event */
#define NRF54L_PWM_PUBLISH_DMA_SEQ_BUSERROR_OFFSET(n) (0x01ac + 12 * (n)) /* Publish sequence n DMA BUSERROR event */
#define NRF54L_PWM_PUBLISH_COMPAREMATCH_OFFSET(n)     (0x01bc + 4 * (n))  /* Publish channel n COMPAREMATCH event */
#define NRF54L_PWM_SHORTS_OFFSET                      0x0200              /* Shortcut register */
#define NRF54L_PWM_INTEN_OFFSET                       0x0300              /* Enable or disable interrupt */
#define NRF54L_PWM_INTENSET_OFFSET                    0x0304              /* Enable interrupt */
#define NRF54L_PWM_INTENCLR_OFFSET                    0x0308              /* Disable interrupt */
#define NRF54L_PWM_INTPEND_OFFSET                     0x030c              /* Pending interrupts */
#define NRF54L_PWM_ENABLE_OFFSET                      0x0500              /* PWM enable */
#define NRF54L_PWM_MODE_OFFSET                        0x0504              /* Wave counter mode */
#define NRF54L_PWM_COUNTERTOP_OFFSET                  0x0508              /* Counter max value */
#define NRF54L_PWM_PRESCALER_OFFSET                   0x050c              /* Prescaler configuration */
#define NRF54L_PWM_DECODER_OFFSET                     0x0510              /* Configuration of the decoder */
#define NRF54L_PWM_LOOP_OFFSET                        0x0514              /* Amount of playback of a loop */
#define NRF54L_PWM_IDLEOUT_OFFSET                     0x0518              /* Idle output levels */
#define NRF54L_PWM_SEQ0REFRESH_OFFSET                 0x0528              /* Sequence 0 additional periods */
#define NRF54L_PWM_SEQ0ENDDELAY_OFFSET                0x052c              /* Time added after sequence 0 */
#define NRF54L_PWM_SEQ1REFRESH_OFFSET                 0x0548              /* Sequence 1 additional periods */
#define NRF54L_PWM_SEQ1ENDDELAY_OFFSET                0x054c              /* Time added after sequence 1 */
#define NRF54L_PWM_PSEL0_OFFSET                       0x0560              /* Output pin select for PWM chan 0 */
#define NRF54L_PWM_PSEL1_OFFSET                       0x0564              /* Output pin select for PWM chan 1 */
#define NRF54L_PWM_PSEL2_OFFSET                       0x0568              /* Output pin select for PWM chan 2 */
#define NRF54L_PWM_PSEL3_OFFSET                       0x056c              /* Output pin select for PWM chan 3 */

#define NRF54L_PWM_DMA_SEQ_PTR_OFFSET(n)                 (0x0704 + 36 * (n)) /* Sequence n buffer address */
#define NRF54L_PWM_DMA_SEQ_MAXCNT_OFFSET(n)              (0x0708 + 36 * (n)) /* Sequence n buffer size in bytes */
#define NRF54L_PWM_DMA_SEQ_AMOUNT_OFFSET(n)              (0x070c + 36 * (n)) /* Sequence n completed transfer byte count */
#define NRF54L_PWM_DMA_SEQ_CURRENTAMOUNT_OFFSET(n)       (0x0710 + 36 * (n)) /* Sequence n current transfer byte count */
#define NRF54L_PWM_DMA_SEQ_TERMINATEONBUSERROR_OFFSET(n) (0x071c + 36 * (n)) /* Terminate sequence n DMA on bus error */
#define NRF54L_PWM_DMA_SEQ_BUSERRORADDRESS_OFFSET(n)     (0x0720 + 36 * (n)) /* Address of last sequence n DMA bus error */

/* Register Bitfield Definitions ********************************************/

/* TASKS_STOP Register */

#define PWM_TASKS_STOP                 (1 << 0) /* Bit 0: Stop PWM */

/* TASKS_DMA.SEQ[n].START Register */

#define PWM_TASKS_DMA_SEQ_START        (1 << 0) /* Bit 0: Start sequence */

/* TASKS_NEXTSTEP Register */

#define PWM_TASKS_NEXTSTEP             (1 << 0) /* Bit 0: Next step */

/* SHORTS Register */

#define PWM_SHORTS_SEQEND0_STOP             (1 << 0) /* Bit 0: Shortcut between event SEQEND[0] and task STOP */
#define PWM_SHORTS_SEQEND1_STOP             (1 << 1) /* Bit 1: Shortcut between event SEQEND[1] and task STOP */
#define PWM_SHORTS_LOOPSDONE_DMA_SEQ0_START (1 << 2) /* Bit 2: Shortcut between event LOOPSDONE and task DMA.SEQ[0].START */
#define PWM_SHORTS_LOOPSDONE_DMA_SEQ1_START (1 << 3) /* Bit 3: Shortcut between event LOOPSDONE and task DMA.SEQ[1].START */
#define PWM_SHORTS_LOOPSDONE_STOP           (1 << 4) /* Bit 4: Shortcut between event LOOPSDONE and task STOP */
#define PWM_SHORTS_RAMUNDERFLOW_STOP        (1 << 5) /* Stop on RAM underflow */
#define PWM_SHORTS_DMA_SEQ0_BUSERROR_STOP   (1 << 6) /* Bit 6: Shortcut between event DMA.SEQ[0].BUSERROR and task STOP */
#define PWM_SHORTS_DMA_SEQ1_BUSERROR_STOP   (1 << 7) /* Bit 7: Shortcut between event DMA.SEQ[1].BUSERROR and task STOP */

/* INTEN/INTENSET/INTENCLR Register */

#define PWM_INT_STOPPED                (1 << 1)              /* Interrupt for STOPPED */
#define PWM_INT_SEQSTARTED0            (1 << 2)              /* Interrupt for SEQSTARTED0 */
#define PWM_INT_SEQSTARTED1            (1 << 3)              /* Interrupt for SEQSTARTED1 */
#define PWM_INT_SEQEND0                (1 << 4)              /* Interrupt for SEQEND0 */
#define PWM_INT_SEQEND1                (1 << 5)              /* Interrupt for SEQEND1 */
#define PWM_INT_PWMPERIODEND           (1 << 6)              /* Interrupt for PWMPERIODEND */
#define PWM_INT_LOOPSDONE              (1 << 7)              /* Interrupt for LOOPSDONE */
#define PWM_INT_RAMUNDERFLOW           (1 << 8)              /* Interrupt for RAMUNDERFLOW */
#define PWM_INT_DMA_SEQ_END(n)         (1 << (9 + 3 * (n)))  /* Interrupt for sequence n DMA END */
#define PWM_INT_DMA_SEQ_READY(n)       (1 << (10 + 3 * (n))) /* Interrupt for sequence n DMA READY */
#define PWM_INT_DMA_SEQ_BUSERROR(n)    (1 << (11 + 3 * (n))) /* Interrupt for sequence n DMA BUSERROR */
#define PWM_INT_COMPAREMATCH(n)        (1 << (15 + (n)))     /* Interrupt for channel n COMPAREMATCH */

/* ENABLE Register */

#define PWM_ENABLE_ENABLE              (1 << 0) /* Bit 0: Enable PWM module */
#define PWM_ENABLE_DISABLE             (0 << 0) /* Bit 0: Disable PWM module */

/* MODE Register */

#define PWM_MODE_UP                    (0 << 0) /* Bit 0: Up counter, edge-aligned PWM */
#define PWM_MODE_UPDOWN                (1 << 0) /* Bit 0: Up and down counter, center-aligned PWM */

/* COUNTERTOP Register */

#define PWM_COUNTERTOP_MASK            (0x7fff)

/* PRESCALER Register */

#define PWM_PRESCALER_SHIFT            (0)
#define PWM_PRESCALER_MASK             (7 << PWM_PRESCALER_SHIFT)
#  define PWM_PRESCALER_16MHZ          (0 << PWM_PRESCALER_SHIFT)
#  define PWM_PRESCALER_8MHZ           (1 << PWM_PRESCALER_SHIFT)
#  define PWM_PRESCALER_4MHZ           (2 << PWM_PRESCALER_SHIFT)
#  define PWM_PRESCALER_2MHZ           (3 << PWM_PRESCALER_SHIFT)
#  define PWM_PRESCALER_1MHZ           (4 << PWM_PRESCALER_SHIFT)
#  define PWM_PRESCALER_500KHZ         (5 << PWM_PRESCALER_SHIFT)
#  define PWM_PRESCALER_250KHZ         (6 << PWM_PRESCALER_SHIFT)
#  define PWM_PRESCALER_125KHZ         (7 << PWM_PRESCALER_SHIFT)

/* DECODER Register */

#define PWM_DECODER_LOAD_SHIFT         (0) /* Bits 0-1: How a sequence is read from RAM */
#define PWM_DECODER_LOAD_MASK          (3 << PWM_DECODER_LOAD_SHIFT)
#  define PWM_DECODER_LOAD_COMMON      (0 << PWM_DECODER_LOAD_SHIFT)
#  define PWM_DECODER_LOAD_GROUPED     (1 << PWM_DECODER_LOAD_SHIFT)
#  define PWM_DECODER_LOAD_INDIVIDUAL  (2 << PWM_DECODER_LOAD_SHIFT)
#  define PWM_DECODER_LOAD_WAVEFORM    (3 << PWM_DECODER_LOAD_SHIFT)

#define PWM_DECODER_MODE_REFRESH       (0 << 8) /* Load using REFRESH */
#define PWM_DECODER_MODE_NEXTSTEP      (1 << 8) /* Load using NEXTSTEP */

/* LOOP Register */

#define PWM_LOOP_MASK                  (0xffff)

/* DMA.SEQ[n].MAXCNT Register */

#define PWM_DMA_MAXCNT_MASK            (0x7fff)
#define PWM_DMA_TERMINATEONBUSERROR    (1 << 0)

/* IDLEOUT Register */

#define PWM_IDLEOUT_VAL(n)             (1 << (n))

/* SUBSCRIBE and PUBLISH Registers */

#define PWM_SUBSCRIBE_CHIDX_SHIFT      (0)
#define PWM_SUBSCRIBE_CHIDX_MASK       (0xff << PWM_SUBSCRIBE_CHIDX_SHIFT)
#define PWM_SUBSCRIBE_EN               (1 << 31)
#define PWM_PUBLISH_CHIDX_SHIFT        (0)
#define PWM_PUBLISH_CHIDX_MASK         (0xff << PWM_PUBLISH_CHIDX_SHIFT)
#define PWM_PUBLISH_EN                 (1 << 31)

/* SEQ[n]REFRESH Register */

#define PWM_SEQREFRESH_MASK            (0xffffff)

/* SEQ[n]ENDDELAY Register */

#define PWM_SEQENDDELAY_MASK           (0xffffff)

/* PSEL[x] Register */

#define PWM_PSEL_PIN_SHIFT             (0)        /* Bits 0-4: OUT pin number */
#define PWM_PSEL_PIN_MASK              (0x1f << PWM_PSEL_PIN_SHIFT)
#define PWM_PSEL_PORT_SHIFT            (5)        /* Bits 5-7: OUT port number */
#define PWM_PSEL_PORT_MASK             (0x7 << PWM_PSEL_PORT_SHIFT)
#define PWM_PSEL_CONNECTED             (0 << 31)  /* Bit 31: Connected */
#define PWM_PSEL_DISCONNECTED          (1 << 31)  /* Bit 31: Disconnected */
#define PWM_PSEL_RESET                 (0xffffffff)

/* Decoder data */

#define PWM_DECODER_COMPARE_SHIFT     (0)
#define PWM_DECODER_COMPARE_MASK      (0x7fff)
#define PWM_DECODER_POL_RISING        (0 << 15)
#define PWM_DECODER_POL_FALLING       (1 << 15)

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_PWM_H */
