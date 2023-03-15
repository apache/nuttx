/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_pwm.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_PWM_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Register offsets *********************************************************/

#define NRF53_PWM_TASKS_STOP_OFFSET          0x0004                 /* Stop PWM */
#define NRF53_PWM_TASKS_SEQSTART0_OFFSET     0x0008                 /* Sequence 0 start */
#define NRF53_PWM_TASKS_SEQSTART1_OFFSET     0x000c                 /* Sequence 1 start  */
#define NRF53_PWM_TASKS_NEXTSTEP_OFFSET      0x0010                 /* Steps by one value in the current sequence */
                                                                    /* TODO: 0x084-0x090 */
#define NRF53_PWM_EVENTS_STOPPED_OFFSET      0x0104                 /* STOP event */
#define NRF53_PWM_EVENTS_SEQSTARTED0_OFFSET  0x0108                 /* Sequence 0 started event */
#define NRF53_PWM_EVENTS_SEQSTARTED1_OFFSET  0x010c                 /* Sequence 1 started event */
#define NRF53_PWM_EVENTS_SEQEND0_OFFSET      0x0110                 /* Sequence 0 end event */
#define NRF53_PWM_EVENTS_SEQEND1_OFFSET      0x0114                 /* Sequence 1 end event */
#define NRF53_PWM_EVENTS_PWMPERIODEN_OFFSET  0x0118                 /* PWM period end event */
#define NRF53_PWM_EVENTS_LOOPSDONE_OFFSET    0x011c                 /* Loop done event */
                                                                    /* TODO: 0x184-0x19c */
#define NRF53_PWM_SHORTS_OFFSET              0x0200                 /* Shourtcut register */
#define NRF53_PWM_INTEN_OFFSET               0x0300                 /* Enable or disable interrupt */
#define NRF53_PWM_INTENSET_OFFSET            0x0304                 /* Enable interrupt */
#define NRF53_PWM_INTENCLR_OFFSET            0x0308                 /* Disable interrupt */
#define NRF53_PWM_ENABLE_OFFSET              0x0500                 /* PWM enable */
#define NRF53_PWM_MODE_OFFSET                0x0504                 /* Wave counter mode */
#define NRF53_PWM_COUNTERTOP_OFFSET          0x0508                 /* Counter max value */
#define NRF53_PWM_PRESCALER_OFFSET           0x050c                 /* Prescaler configuration */
#define NRF53_PWM_DECODER_OFFSET             0x0510                 /* Configuration of the decoder */
#define NRF53_PWM_LOOP_OFFSET                0x0514                 /* Amount of playback of a loop */
#define NRF53_PWM_SEQ0PTR_OFFSET             0x0520                 /* Sequence 0 beginning address */
#define NRF53_PWM_SEQ0CNT_OFFSET             0x0524                 /* Sequence 0 length */
#define NRF53_PWM_SEQ0REFRESH_OFFSET         0x0528                 /* Sequence 0 additional periods */
#define NRF53_PWM_SEQ0ENDDELAY_OFFSET        0x052c                 /* Time added after sequence 0 */
#define NRF53_PWM_SEQ1PTR_OFFSET             0x0540                 /* Sequence 1 beginning address */
#define NRF53_PWM_SEQ1CNT_OFFSET             0x0544                 /* Sequence 1 length */
#define NRF53_PWM_SEQ1REFRESH_OFFSET         0x0548                 /* Sequence 0 additional periods */
#define NRF53_PWM_SEQ1ENDDELAY_OFFSET        0x054c                 /* Time added after sequence 1 */
#define NRF53_PWM_PSEL0_OFFSET               0x0560                 /* Output pin select for PWM chan 0 */
#define NRF53_PWM_PSEL1_OFFSET               0x0564                 /* Output pin select for PWM chan 1 */
#define NRF53_PWM_PSEL2_OFFSET               0x0568                 /* Output pin select for PWM chan 2 */
#define NRF53_PWM_PSEL3_OFFSET               0x056c                 /* Output pin select for PWM chan 3 */

/* Register Bitfield Definitions ********************************************/

/* TASKS_STOP Register */

#define PWM_TASKS_STOP                 (1 << 0) /* Bit 0: Stop PWM */

/* TASKS_SEQSTART[n] Register */

#define PWM_TASKS_SEQSTART             (1 << 0) /* Bit 0: Start sequence */

/* TASKS_NEXTSTEP Register */

#define PWM_TASKS_NEXTSTEP             (1 << 0) /* Bit 0: Next step */

/* SHORTS Register */

#define PWM_SHORTS_SEQEND0_STOP        (1 << 0) /* Bit 0: Shortcut between event SEQEND[0] and task STOP */
#define PWM_SHORTS_SEQEND1_STOP        (1 << 1) /* Bit 1: Shortcut between event SEQEND[1] and task STOP */
#define PWM_SHORTS_LOOPSDONE_SEQSTART0 (1 << 2) /* Bit 2: Shortcut between event LOOPSDONE and task SEQSTART[0] */
#define PWM_SHORTS_LOOPSDONE_SEQSTART1 (1 << 3) /* Bit 3: Shortcut between event LOOPSDONE and task SEQSTART[1] */
#define PWM_SHORTS_LOOPSDONE_STOP      (1 << 4) /* Bit 4: Shortcut between event LOOPSDONE and task STOP */

/* INTEN/INTENSET/INTENCLR Register */

#define PWM_INT_STOPPED                (1 << 0) /* Bit 0: Interrupt for event STOPPED */
#define PWM_INT_SEQSTARTED0            (1 << 1) /* Bit 1: Interrupt for event SEQSTARTED0 */
#define PWM_INT_SEQSTARTED1            (1 << 2) /* Bit 2: Interrupt for event SEQSTARTED2 */
#define PWM_INT_SEQEND0                (1 << 3) /* Bit 3: Interrupt for event SEQEND0 */
#define PWM_INT_SEQEND1                (1 << 4) /* Bit 4: Interrupt for event SEQEND1 */
#define PWM_INT_PWMPERIODEND           (1 << 5) /* Bit 5: Interrupt for event PWMPERIODEND */
#define PWM_INT_LOOPSDONE              (1 << 6) /* Bit 6: Interrupt for event LOOPSDONE */

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

#define PWM_DECODER_MODE_REFRESH       (8 << 0) /* Bit 8: */
#define PWM_DECODER_MODE_NEXTSTEP      (8 << 1) /* Bit 8: */

/* LOOP Register */

#define PWM_LOOP_MASK                  (0xffff)

/* SEQ[n]CNT Register */

#define PWM_SEQCNT_MASK                (0x7fff)

/* SEQ[n]REFRESH Register */

#define PWM_SEQREFRESH_MASK            (0xffffff)

/* SEQ[n]ENDDELAY Register */

#define PWM_SEQENDDELAY_MASK           (0xffffff)

/* PSEL[x] Register */

#define PWM_PSEL_PIN_SHIFT             (0)        /* Bits 0-4: OUT pin number */
#define PWM_PSEL_PIN_MASK              (0x1f << PWM_PSELSDA_PIN_SHIFT)
#define PWM_PSEL_PORT_SHIFT            (5)        /* Bit 5: PUT port number */
#define PWM_PSEL_PORT_MASK             (0x1 << PWM_PSELSDA_PORT_SHIFT)
#define PWM_PSEL_CONNECTED             (1 << 31)  /* Bit 31: Connection */
#define PWM_PSEL_RESET                 (0xffffffff)

/* Decoder data */

#define PWM_DECODER_COMPARE_SHIFT     (0)
#define PWM_DECODER_COMPARE_MASK      (0x7fff)
#define PWM_DECODER_POL_RISING        (0 << 15)
#define PWM_DECODER_POL_FALLING       (1 << 15)

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_PWM_H */
