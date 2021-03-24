/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_gpio.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_GPIO_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define KINETIS_GPIO_PDOR_OFFSET 0x0000 /* Port Data Output Register */
#define KINETIS_GPIO_PSOR_OFFSET 0x0004 /* Port Set Output Register */
#define KINETIS_GPIO_PCOR_OFFSET 0x0008 /* Port Clear Output Register */
#define KINETIS_GPIO_PTOR_OFFSET 0x000c /* Port Toggle Output Register */
#define KINETIS_GPIO_PDIR_OFFSET 0x0010 /* Port Data Input Register */
#define KINETIS_GPIO_PDDR_OFFSET 0x0014 /* Port Data Direction Register */

/* Register Addresses *******************************************************/

#define KINETIS_GPIO_PDOR(n)    (KINETIS_GPIO_BASE(n)+KINETIS_GPIO_PDOR_OFFSET)
#define KINETIS_GPIO_PSOR(n)    (KINETIS_GPIO_BASE(n)+KINETIS_GPIO_PSOR_OFFSET)
#define KINETIS_GPIO_PCOR(n)    (KINETIS_GPIO_BASE(n)+KINETIS_GPIO_PCOR_OFFSET)
#define KINETIS_GPIO_PTOR(n)    (KINETIS_GPIO_BASE(n)+KINETIS_GPIO_PTOR_OFFSET)
#define KINETIS_GPIO_PDIR(n)    (KINETIS_GPIO_BASE(n)+KINETIS_GPIO_PDIR_OFFSET)
#define KINETIS_GPIO_PDDR(n)    (KINETIS_GPIO_BASE(n)+KINETIS_GPIO_PDDR_OFFSET)

#define KINETIS_GPIOA_PDOR      (KINETIS_GPIOA_BASE+KINETIS_GPIO_PDOR_OFFSET)
#define KINETIS_GPIOA_PSOR      (KINETIS_GPIOA_BASE+KINETIS_GPIO_PSOR_OFFSET)
#define KINETIS_GPIOA_PCOR      (KINETIS_GPIOA_BASE+KINETIS_GPIO_PCOR_OFFSET)
#define KINETIS_GPIOA_PTOR      (KINETIS_GPIOA_BASE+KINETIS_GPIO_PTOR_OFFSET)
#define KINETIS_GPIOA_PDIR      (KINETIS_GPIOA_BASE+KINETIS_GPIO_PDIR_OFFSET)
#define KINETIS_GPIOA_PDDR      (KINETIS_GPIOA_BASE+KINETIS_GPIO_PDDR_OFFSET)

#define KINETIS_GPIOB_PDOR      (KINETIS_GPIOB_BASE+KINETIS_GPIO_PDOR_OFFSET)
#define KINETIS_GPIOB_PSOR      (KINETIS_GPIOB_BASE+KINETIS_GPIO_PSOR_OFFSET)
#define KINETIS_GPIOB_PCOR      (KINETIS_GPIOB_BASE+KINETIS_GPIO_PCOR_OFFSET)
#define KINETIS_GPIOB_PTOR      (KINETIS_GPIOB_BASE+KINETIS_GPIO_PTOR_OFFSET)
#define KINETIS_GPIOB_PDIR      (KINETIS_GPIOB_BASE+KINETIS_GPIO_PDIR_OFFSET)
#define KINETIS_GPIOB_PDDR      (KINETIS_GPIOB_BASE+KINETIS_GPIO_PDDR_OFFSET)

#define KINETIS_GPIOC_PDOR      (KINETIS_GPIOC_BASE+KINETIS_GPIO_PDOR_OFFSET)
#define KINETIS_GPIOC_PSOR      (KINETIS_GPIOC_BASE+KINETIS_GPIO_PSOR_OFFSET)
#define KINETIS_GPIOC_PCOR      (KINETIS_GPIOC_BASE+KINETIS_GPIO_PCOR_OFFSET)
#define KINETIS_GPIOC_PTOR      (KINETIS_GPIOC_BASE+KINETIS_GPIO_PTOR_OFFSET)
#define KINETIS_GPIOC_PDIR      (KINETIS_GPIOC_BASE+KINETIS_GPIO_PDIR_OFFSET)
#define KINETIS_GPIOC_PDDR      (KINETIS_GPIOC_BASE+KINETIS_GPIO_PDDR_OFFSET)

#define KINETIS_GPIOD_PDOR      (KINETIS_GPIOD_BASE+KINETIS_GPIO_PDOR_OFFSET)
#define KINETIS_GPIOD_PSOR      (KINETIS_GPIOD_BASE+KINETIS_GPIO_PSOR_OFFSET)
#define KINETIS_GPIOD_PCOR      (KINETIS_GPIOD_BASE+KINETIS_GPIO_PCOR_OFFSET)
#define KINETIS_GPIOD_PTOR      (KINETIS_GPIOD_BASE+KINETIS_GPIO_PTOR_OFFSET)
#define KINETIS_GPIOD_PDIR      (KINETIS_GPIOD_BASE+KINETIS_GPIO_PDIR_OFFSET)
#define KINETIS_GPIOD_PDDR      (KINETIS_GPIOD_BASE+KINETIS_GPIO_PDDR_OFFSET)

#define KINETIS_GPIOE_PDOR      (KINETIS_GPIOE_BASE+KINETIS_GPIO_PDOR_OFFSET)
#define KINETIS_GPIOE_PSOR      (KINETIS_GPIOE_BASE+KINETIS_GPIO_PSOR_OFFSET)
#define KINETIS_GPIOE_PCOR      (KINETIS_GPIOE_BASE+KINETIS_GPIO_PCOR_OFFSET)
#define KINETIS_GPIOE_PTOR      (KINETIS_GPIOE_BASE+KINETIS_GPIO_PTOR_OFFSET)
#define KINETIS_GPIOE_PDIR      (KINETIS_GPIOE_BASE+KINETIS_GPIO_PDIR_OFFSET)
#define KINETIS_GPIOE_PDDR      (KINETIS_GPIOE_BASE+KINETIS_GPIO_PDDR_OFFSET)

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
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_GPIO_H */
