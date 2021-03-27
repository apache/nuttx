/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_gpio.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_GPIO_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define S32K1XX_GPIOA               (0)
#define S32K1XX_GPIOB               (1)
#define S32K1XX_GPIOC               (2)
#define S32K1XX_GPIOD               (3)
#define S32K1XX_GPIOE               (4)
#define S32K1XX_NGPIO               (5)

/* GPIO Register Offsets ****************************************************/

#define S32K1XX_GPIO_PDOR_OFFSET    0x0000  /* Port Data Output Register */
#define S32K1XX_GPIO_PSOR_OFFSET    0x0004  /* Port Set Output Register */
#define S32K1XX_GPIO_PCOR_OFFSET    0x0008  /* Port Clear Output Register */
#define S32K1XX_GPIO_PTOR_OFFSET    0x000c  /* Port Toggle Output Register */
#define S32K1XX_GPIO_PDIR_OFFSET    0x0010  /* Port Data Input Register */
#define S32K1XX_GPIO_PDDR_OFFSET    0x0014  /* Port Data Direction Register */
#define S32K1XX_GPIO_PIDR_OFFSET    0x0018  /* Port Input Disable Register */

/* GPIO Register Addresses **************************************************/

#define S32K1XX_GPIO_PDOR(g)        (S32K1XX_GPIO_BASE(g) + S32K1XX_GPIO_PDOR_OFFSET)
#define S32K1XX_GPIO_PSOR(g)        (S32K1XX_GPIO_BASE(g) + S32K1XX_GPIO_PSOR_OFFSET)
#define S32K1XX_GPIO_PCOR(g)        (S32K1XX_GPIO_BASE(g) + S32K1XX_GPIO_PCOR_OFFSET)
#define S32K1XX_GPIO_PTOR(g)        (S32K1XX_GPIO_BASE(g) + S32K1XX_GPIO_PTOR_OFFSET)
#define S32K1XX_GPIO_PDIR(g)        (S32K1XX_GPIO_BASE(g) + S32K1XX_GPIO_PDIR_OFFSET)
#define S32K1XX_GPIO_PDDR(g)        (S32K1XX_GPIO_BASE(g) + S32K1XX_GPIO_PDDR_OFFSET)
#define S32K1XX_GPIO_PIDR(g)        (S32K1XX_GPIO_BASE(g) + S32K1XX_GPIO_PIDR_OFFSET)

#define S32K1XX_GPIOA_PDOR          (S32K1XX_GPIOA_BASE + S32K1XX_GPIO_PDOR_OFFSET)
#define S32K1XX_GPIOA_PSOR          (S32K1XX_GPIOA_BASE + S32K1XX_GPIO_PSOR_OFFSET)
#define S32K1XX_GPIOA_PCOR          (S32K1XX_GPIOA_BASE + S32K1XX_GPIO_PCOR_OFFSET)
#define S32K1XX_GPIOA_PTOR          (S32K1XX_GPIOA_BASE + S32K1XX_GPIO_PTOR_OFFSET)
#define S32K1XX_GPIOA_PDIR          (S32K1XX_GPIOA_BASE + S32K1XX_GPIO_PDIR_OFFSET)
#define S32K1XX_GPIOA_PDDR          (S32K1XX_GPIOA_BASE + S32K1XX_GPIO_PDDR_OFFSET)
#define S32K1XX_GPIOA_PIDR          (S32K1XX_GPIOA_BASE + S32K1XX_GPIO_PIDR_OFFSET)

#define S32K1XX_GPIOB_PDOR          (S32K1XX_GPIOB_BASE + S32K1XX_GPIO_PDOR_OFFSET)
#define S32K1XX_GPIOB_PSOR          (S32K1XX_GPIOB_BASE + S32K1XX_GPIO_PSOR_OFFSET)
#define S32K1XX_GPIOB_PCOR          (S32K1XX_GPIOB_BASE + S32K1XX_GPIO_PCOR_OFFSET)
#define S32K1XX_GPIOB_PTOR          (S32K1XX_GPIOB_BASE + S32K1XX_GPIO_PTOR_OFFSET)
#define S32K1XX_GPIOB_PDIR          (S32K1XX_GPIOB_BASE + S32K1XX_GPIO_PDIR_OFFSET)
#define S32K1XX_GPIOB_PDDR          (S32K1XX_GPIOB_BASE + S32K1XX_GPIO_PDDR_OFFSET)
#define S32K1XX_GPIOB_PIDR          (S32K1XX_GPIOB_BASE + S32K1XX_GPIO_PIDR_OFFSET)

#define S32K1XX_GPIOC_PDOR          (S32K1XX_GPIOC_BASE + S32K1XX_GPIO_PDOR_OFFSET)
#define S32K1XX_GPIOC_PSOR          (S32K1XX_GPIOC_BASE + S32K1XX_GPIO_PSOR_OFFSET)
#define S32K1XX_GPIOC_PCOR          (S32K1XX_GPIOC_BASE + S32K1XX_GPIO_PCOR_OFFSET)
#define S32K1XX_GPIOC_PTOR          (S32K1XX_GPIOC_BASE + S32K1XX_GPIO_PTOR_OFFSET)
#define S32K1XX_GPIOC_PDIR          (S32K1XX_GPIOC_BASE + S32K1XX_GPIO_PDIR_OFFSET)
#define S32K1XX_GPIOC_PDDR          (S32K1XX_GPIOC_BASE + S32K1XX_GPIO_PDDR_OFFSET)
#define S32K1XX_GPIOC_PIDR          (S32K1XX_GPIOC_BASE + S32K1XX_GPIO_PIDR_OFFSET)

#define S32K1XX_GPIOD_PDOR          (S32K1XX_GPIOD_BASE + S32K1XX_GPIO_PDOR_OFFSET)
#define S32K1XX_GPIOD_PSOR          (S32K1XX_GPIOD_BASE + S32K1XX_GPIO_PSOR_OFFSET)
#define S32K1XX_GPIOD_PCOR          (S32K1XX_GPIOD_BASE + S32K1XX_GPIO_PCOR_OFFSET)
#define S32K1XX_GPIOD_PTOR          (S32K1XX_GPIOD_BASE + S32K1XX_GPIO_PTOR_OFFSET)
#define S32K1XX_GPIOD_PDIR          (S32K1XX_GPIOD_BASE + S32K1XX_GPIO_PDIR_OFFSET)
#define S32K1XX_GPIOD_PDDR          (S32K1XX_GPIOD_BASE + S32K1XX_GPIO_PDDR_OFFSET)
#define S32K1XX_GPIOD_PIDR          (S32K1XX_GPIOD_BASE + S32K1XX_GPIO_PIDR_OFFSET)

#define S32K1XX_GPIOE_PDOR          (S32K1XX_GPIOE_BASE + S32K1XX_GPIO_PDOR_OFFSET)
#define S32K1XX_GPIOE_PSOR          (S32K1XX_GPIOE_BASE + S32K1XX_GPIO_PSOR_OFFSET)
#define S32K1XX_GPIOE_PCOR          (S32K1XX_GPIOE_BASE + S32K1XX_GPIO_PCOR_OFFSET)
#define S32K1XX_GPIOE_PTOR          (S32K1XX_GPIOE_BASE + S32K1XX_GPIO_PTOR_OFFSET)
#define S32K1XX_GPIOE_PDIR          (S32K1XX_GPIOE_BASE + S32K1XX_GPIO_PDIR_OFFSET)
#define S32K1XX_GPIOE_PDDR          (S32K1XX_GPIOE_BASE + S32K1XX_GPIO_PDDR_OFFSET)
#define S32K1XX_GPIOE_PIDR          (S32K1XX_GPIOE_BASE + S32K1XX_GPIO_PIDR_OFFSET)

/* GPIO Register Bitfield Definitions ***************************************/

/* Port Data Output Register */

#define GPIO_PDOR(n)                (1 << (n))  /* Pin n data output, n=0..31 */

/* Port Set Output Register */

#define GPIO_PSOR(n)                (1 << (n))  /* Pin n set output, n=0..31 */

/* Port Clear Output Register */

#define GPIO_PCOR(n)                (1 << (n))  /* Pin n clear output, n=0..31 */

/* Port Toggle Output Register */

#define GPIO_PTOR(n)                (1 << (n))  /* Pin n toggle output, n=0..31 */

/* Port Data Input Register */

#define GPIO_PDIR(n)                (1 << (n))  /* Pin n data input, n=0..31 */

/* Port Data Direction Register */

#define GPIO_PDDR(n)                (1 << (n))  /* Pin n data direction, n=0..31 */

/* Port Input Disable Register */

#define GPIO_PIDR(n)                (1 << (n))  /* Pin n input disable, n=0..31 */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_GPIO_H */
