/****************************************************************************
 * arch/arm/src/kl/hardware/kl_gpio.h
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

#ifndef __ARCH_ARM_SRC_KL_HARDWARE_KL_GPIO_H
#define __ARCH_ARM_SRC_KL_HARDWARE_KL_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define KL_GPIO_NPORTS          5

/* Register Offsets *********************************************************/

#define KL_GPIO_PDOR_OFFSET     0x0000 /* Port Data Output Register */
#define KL_GPIO_PSOR_OFFSET     0x0004 /* Port Set Output Register */
#define KL_GPIO_PCOR_OFFSET     0x0008 /* Port Clear Output Register */
#define KL_GPIO_PTOR_OFFSET     0x000c /* Port Toggle Output Register */
#define KL_GPIO_PDIR_OFFSET     0x0010 /* Port Data Input Register */
#define KL_GPIO_PDDR_OFFSET     0x0014 /* Port Data Direction Register */

/* Register Addresses *******************************************************/

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

/* Register Bit Definitions *************************************************/

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

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_KL_HARDWARE_KL_GPIO_H */
