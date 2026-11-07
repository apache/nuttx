/****************************************************************************
 * arch/rh850/include/u2c/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_RH850_INCLUDE_U2C_IRQ_H
#define __ARCH_RH850_INCLUDE_U2C_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers are the EIINT channel numbers of the U2C8 INTC */

#define NR_IRQS                 630

/* OSTM0 TINT */

#define RH850_IRQ_OSTM0         63

/* RLIN311 channels */

#define RH850_IRQ_RLIN311_LIN   493
#define RH850_IRQ_RLIN311_TX    494     /* UR0 */
#define RH850_IRQ_RLIN311_RX    495     /* UR1 */
#define RH850_IRQ_RLIN311_ERR   496     /* UR2 */

#endif /* __ARCH_RH850_INCLUDE_U2C_IRQ_H */
