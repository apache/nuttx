/****************************************************************************
 * arch/arm/src/lpc214x/lpc214x_timer.h
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

#ifndef __ARCH_ARM_SRC_LPC214X_LPC214X_TIMER_H
#define __ARCH_ARM_SRC_LPC214X_LPC214X_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer registers are 8-, 16-bit and 32-bits wide */

/* Timer Interrupt Register Bit Definitions (8-bit) */

#define LPC214X_TMR_IR_MR0I          (1 << 0)  /* Interrupt flag for match channel 0 */
#define LPC214X_TMR_IR_MR1I          (1 << 1)  /* Interrupt flag for match channel 1 */
#define LPC214X_TMR_IR_MR2I          (1 << 2)  /* Interrupt flag for match channel 2 */
#define LPC214X_TMR_IR_MR3I          (1 << 3)  /* Interrupt flag for match channel 3 */
#define LPC214X_TMR_IR_CR0I          (1 << 4)  /* Interrupt flag for capture channel 0 event */
#define LPC214X_TMR_IR_CR1I          (1 << 5)  /* Interrupt flag for capture channel 1 event */
#define LPC214X_TMR_IR_CR2I          (1 << 6)  /* Interrupt flag for capture channel 2 event */
#define LPC214X_TMR_IR_CR3I          (1 << 7)  /* Interrupt flag for capture channel 3 event */
#define LPC214X_TMR_IR_ALLI          (0xff)    /* All timer interrupts */

/* Timer Control Register Bit Definitions (8-bits) */

#define LPC214X_TMR_CR_ENABLE        (1 << 0)  /* Counter Enable */
#define LPC214X_TMR_CR_RESET         (1 << 1)  /* Countger Reset */

/* Timer Counter (32-bits, no bit fields) */

/* Timer Prescale Register Bit Definitions (32-bits, no bit fields) */

/* Timer Prescale Counter Register Bit Definitions */

/* Timer Match Control Register Bit Definitions (16-bit) */

#define LPC214X_TMR_MCR_MR0I         (1 <<  0) /* Enable Interrupt when MR0 matches TC */
#define LPC214X_TMR_MCR_MR0R         (1 <<  1) /* Enable Reset of TC upon MR0 match */
#define LPC214X_TMR_MCR_MR0S         (1 <<  2) /* Enable Stop of TC upon MR0 match */
#define LPC214X_TMR_MCR_MR1I         (1 <<  3) /* Enable Interrupt when MR1 matches TC */
#define LPC214X_TMR_MCR_MR1R         (1 <<  4) /* Enable Reset of TC upon MR1 match */
#define LPC214X_TMR_MCR_MR1S         (1 <<  5) /* Enable Stop of TC upon MR1 match */
#define LPC214X_TMR_MCR_MR2I         (1 <<  6) /* Enable Interrupt when MR2 matches TC */
#define LPC214X_TMR_MCR_MR2R         (1 <<  7) /* Enable Reset of TC upon MR2 match */
#define LPC214X_TMR_MCR_MR2S         (1 <<  8) /* Enable Stop of TC upon MR2 match */
#define LPC214X_TMR_MCR_MR3I         (1 <<  9) /* Enable Interrupt when MR3 matches TC */
#define LPC214X_TMR_MCR_MR3R         (1 << 10) /* Enable Reset of TC upon MR3 match */
#define LPC214X_TMR_MCR_MR3S         (1 << 11) /* Enable Stop of TC upon MR3 match */

/* Timer Match Register 0/1/2/3 (32-bits, no bit fields) */

/* Timer Capture Control Register Bit Definitions */

#define LPC214X_TMR_CCR_CAP0RE       (1 <<  0) /* Enable Rising edge on CAPn.0 will load TC to CR0 */
#define LPC214X_TMR_CCR_CAP0FE       (1 <<  1) /* Enable Falling edge on CAPn.0 will load TC to CR0 */
#define LPC214X_TMR_CCR_CAP0I        (1 <<  2) /* Enable Interrupt on load of CR0 */
#define LPC214X_TMR_CCR_CAP1RE       (1 <<  3) /* Enable Rising edge on CAPn.1 will load TC to CR1 */
#define LPC214X_TMR_CCR_CAP1FE       (1 <<  4) /* Enable Falling edge on CAPn.1 will load TC to CR1 */
#define LPC214X_TMR_CCR_CAP1I        (1 <<  5) /* Enable Interrupt on load of CR1 */
#define LPC214X_TMR_CCR_CAP2RE       (1 <<  6) /* Enable Rising edge on CAPn.2 will load TC to CR2 */
#define LPC214X_TMR_CCR_CAP2FE       (1 <<  7) /* Enable Falling edge on CAPn.2 will load TC to CR2 */
#define LPC214X_TMR_CCR_CAP2I        (1 <<  8) /* Enable Interrupt on load of CR2 */
#define LPC214X_TMR_CCR_CAP3RE       (1 <<  9) /* Enable Rising edge on CAPn.3 will load TC to CR3 */
#define LPC214X_TMR_CCR_CAP3FE       (1 << 10) /* Enable Falling edge on CAPn.3 will load TC to CR3 */
#define LPC214X_TMR_CCR_CAP3I        (1 << 11) /* Enable Interrupt on load of CR3 */

/* Timer Capture Register 0/1/2/3 (32-bits, no bit fields) */

/* Timer External Match Register Bit Definitions */

#define LPC214X_TMR_EMR_EM0          (1 <<  0) /* External Match 0 */
#define LPC214X_TMR_EMR_EM1          (1 <<  1) /* External Match 1 */
#define LPC214X_TMR_EMR_EM2          (1 <<  2) /* External Match 2 */
#define LPC214X_TMR_EMR_EM3          (1 <<  3) /* External Match 3 */

#define LPC214X_TMR_EMR_EMC0(b)      ((b) <<  4)  /* External match control 0 (see below) */
#define LPC214X_TMR_EMR_EMC1(b)      ((b) <<  6)  /* External match control 1 (see below) */
#define LPC214X_TMR_EMR_EMC2(b)      ((b) <<  8)  /* External match control 2 (see below) */
#define LPC214X_TMR_EMR_EMC3(b)      ((b) << 10)  /* External match control 3 (see below) */

/* EMR External Match Control (EMCn) Field Falues */

#define LPC214X_TMR_EMR_MASK         (3)       /* Mask for all bits */
#define LPC214X_TMR_EMR_NOOP         (0)       /* Do nothing */
#define LPC214X_TMR_EMR_CLEAR        (1)       /* Clear corresponding EMn bit/output to 0 */
#define LPC214X_TMR_EMR_SET          (2)       /* Set corresponding EMn bit/output to 1 */
#define LPC214X_TMR_EMR_TOGGLE       (3)       /* Toggle corresponding EMn bit/output */

/* Timer Count Control Register Bit Definitions (8-bit) */

#define LPC214X_TMR_
#define LPC214X_TMR_CTCR_MODE_MASK   (3 << 0) /* Counter/Timer Mode */
#define   LPC214X_TMR_CTCR_PCLK      (0 << 0) /*   Rising edge of PCLK */
#define   LPC214X_TMR_CTCR_RISING    (1 << 0) /*   Rising edge of CAP input */
#define   LPC214X_TMR_CTDR_FALLING   (2 << 0) /*   Failing edge of CAP input */
#define   LPC214X_TMR_CTCR_BOTH      (3 << 0) /*   Both edges of CAP input */
#define LPC214X_TMR_CTCR_INPUT_MASK  (3 << 2) /* Counter Input Select */
#define   LPC214X_TMR_CTCR_CR0       (0 << 2) /*   CAPn.0 */
#define   LPC214X_TMR_CTCR_CR1       (1 << 2) /*   CAPn.1 */
#define   LPC214X_TMR_CTCR_CR2       (2 << 2) /*   CAPn.2 */
#define   LPC214X_TMR_CTCR_CR3       (3 << 2) /*   CAPn.3 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC214X_LPC214X_TIMER_H */
