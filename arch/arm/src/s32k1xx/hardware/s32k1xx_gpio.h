/************************************************************************************
 * arch/arm/src/s32k1xx/chip/s32k1xx_gpio.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_GPIO_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define S32K1XX_GPIOA               (0)
#define S32K1XX_GPIOB               (1)
#define S32K1XX_GPIOC               (2)
#define S32K1XX_GPIOD               (3)
#define S32K1XX_GPIOE               (4)
#define S32K1XX_NGPIO               (5)

/* GPIO Register Offsets *************************************************************/

#define S32K1XX_GPIO_PDOR_OFFSET    0x0000  /* Port Data Output Register */
#define S32K1XX_GPIO_PSOR_OFFSET    0x0004  /* Port Set Output Register */
#define S32K1XX_GPIO_PCOR_OFFSET    0x0008  /* Port Clear Output Register */
#define S32K1XX_GPIO_PTOR_OFFSET    0x000c  /* Port Toggle Output Register */
#define S32K1XX_GPIO_PDIR_OFFSET    0x0010  /* Port Data Input Register */
#define S32K1XX_GPIO_PDDR_OFFSET    0x0014  /* Port Data Direction Register */
#define S32K1XX_GPIO_PIDR_OFFSET    0x0018  /* Port Input Disable Register */

/* GPIO Register Addresses ***********************************************************/

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

/* GPIO Register Bitfield Definitions ************************************************/

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
