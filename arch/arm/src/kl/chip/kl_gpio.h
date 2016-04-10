/************************************************************************************
 * arch/arm/src/kl/chip/kl_gpio.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KL_CHIP_KL_GPIO_H
#define __ARCH_ARM_SRC_KL_CHIP_KL_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define KL_GPIO_NPORTS          5

/* Register Offsets *****************************************************************/

#define KL_GPIO_PDOR_OFFSET     0x0000 /* Port Data Output Register */
#define KL_GPIO_PSOR_OFFSET     0x0004 /* Port Set Output Register */
#define KL_GPIO_PCOR_OFFSET     0x0008 /* Port Clear Output Register */
#define KL_GPIO_PTOR_OFFSET     0x000c /* Port Toggle Output Register */
#define KL_GPIO_PDIR_OFFSET     0x0010 /* Port Data Input Register */
#define KL_GPIO_PDDR_OFFSET     0x0014 /* Port Data Direction Register */

/* Register Addresses ***************************************************************/

#define KL_GPIO_PDOR(n)         (KL_GPIO_BASE(n)+KL_GPIO_PDOR_OFFSET)
#define KL_GPIO_PSOR(n)         (KL_GPIO_BASE(n)+KL_GPIO_PSOR_OFFSET)
#define KL_GPIO_PCOR(n)         (KL_GPIO_BASE(n)+KL_GPIO_PCOR_OFFSET)
#define KL_GPIO_PTOR(n)         (KL_GPIO_BASE(n)+KL_GPIO_PTOR_OFFSET)
#define KL_GPIO_PDIR(n)         (KL_GPIO_BASE(n)+KL_GPIO_PDIR_OFFSET)
#define KL_GPIO_PDDR(n)         (KL_GPIO_BASE(n)+KL_GPIO_PDDR_OFFSET)

#define KL_GPIOA_PDOR           (KL_GPIOA_BASE+KL_GPIO_PDOR_OFFSET)
#define KL_GPIOA_PSOR           (KL_GPIOA_BASE+KL_GPIO_PSOR_OFFSET)
#define KL_GPIOA_PCOR           (KL_GPIOA_BASE+KL_GPIO_PCOR_OFFSET)
#define KL_GPIOA_PTOR           (KL_GPIOA_BASE+KL_GPIO_PTOR_OFFSET)
#define KL_GPIOA_PDIR           (KL_GPIOA_BASE+KL_GPIO_PDIR_OFFSET)
#define KL_GPIOA_PDDR           (KL_GPIOA_BASE+KL_GPIO_PDDR_OFFSET)

#define KL_GPIOB_PDOR           (KL_GPIOB_BASE+KL_GPIO_PDOR_OFFSET)
#define KL_GPIOB_PSOR           (KL_GPIOB_BASE+KL_GPIO_PSOR_OFFSET)
#define KL_GPIOB_PCOR           (KL_GPIOB_BASE+KL_GPIO_PCOR_OFFSET)
#define KL_GPIOB_PTOR           (KL_GPIOB_BASE+KL_GPIO_PTOR_OFFSET)
#define KL_GPIOB_PDIR           (KL_GPIOB_BASE+KL_GPIO_PDIR_OFFSET)
#define KL_GPIOB_PDDR           (KL_GPIOB_BASE+KL_GPIO_PDDR_OFFSET)

#define KL_GPIOC_PDOR           (KL_GPIOC_BASE+KL_GPIO_PDOR_OFFSET)
#define KL_GPIOC_PSOR           (KL_GPIOC_BASE+KL_GPIO_PSOR_OFFSET)
#define KL_GPIOC_PCOR           (KL_GPIOC_BASE+KL_GPIO_PCOR_OFFSET)
#define KL_GPIOC_PTOR           (KL_GPIOC_BASE+KL_GPIO_PTOR_OFFSET)
#define KL_GPIOC_PDIR           (KL_GPIOC_BASE+KL_GPIO_PDIR_OFFSET)
#define KL_GPIOC_PDDR           (KL_GPIOC_BASE+KL_GPIO_PDDR_OFFSET)

#define KL_GPIOD_PDOR           (KL_GPIOD_BASE+KL_GPIO_PDOR_OFFSET)
#define KL_GPIOD_PSOR           (KL_GPIOD_BASE+KL_GPIO_PSOR_OFFSET)
#define KL_GPIOD_PCOR           (KL_GPIOD_BASE+KL_GPIO_PCOR_OFFSET)
#define KL_GPIOD_PTOR           (KL_GPIOD_BASE+KL_GPIO_PTOR_OFFSET)
#define KL_GPIOD_PDIR           (KL_GPIOD_BASE+KL_GPIO_PDIR_OFFSET)
#define KL_GPIOD_PDDR           (KL_GPIOD_BASE+KL_GPIO_PDDR_OFFSET)

#define KL_GPIOE_PDOR           (KL_GPIOE_BASE+KL_GPIO_PDOR_OFFSET)
#define KL_GPIOE_PSOR           (KL_GPIOE_BASE+KL_GPIO_PSOR_OFFSET)
#define KL_GPIOE_PCOR           (KL_GPIOE_BASE+KL_GPIO_PCOR_OFFSET)
#define KL_GPIOE_PTOR           (KL_GPIOE_BASE+KL_GPIO_PTOR_OFFSET)
#define KL_GPIOE_PDIR           (KL_GPIOE_BASE+KL_GPIO_PDIR_OFFSET)
#define KL_GPIOE_PDDR           (KL_GPIOE_BASE+KL_GPIO_PDDR_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Port Data Output Register */

#define GPIO_PDOR(n)            (1 << (n))

/* Port Set Output Register */

#define GPIO_PSOR(n)            (1 << (n))

/* Port Clear Output Register */

#define GPIO_PCOR(n)            (1 << (n))

/* Port Toggle Output Register */

#define GPIO_PTOR(n)            (1 << (n))

/* Port Data Input Register */

#define GPIO_PDIR(n)            (1 << (n))

/* Port Data Direction Register */

#define GPIO_PDDR(n)            (1 << (n))

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_KL_CHIP_KL_GPIO_H */
