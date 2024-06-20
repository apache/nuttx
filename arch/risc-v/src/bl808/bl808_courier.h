/****************************************************************************
 * arch/risc-v/src/bl808/bl808_courier.h
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

#ifndef __ARCH_RISC_V_SRC_BL808_BL808_COURIER_H
#define __ARCH_RISC_V_SRC_BL808_BL808_COURIER_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BL808_COURIER_IRQN_MASK 0xff
#define BL808_INT_SIG_SHIFT 8
#define BL808_INT_EN_SHIFT 9

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bl808_courier_req_irq_enable
 *
 * Description:
 *   Sends an IPC message to M0 core to enable m0_extirq.
 *
 ****************************************************************************/

void bl808_courier_req_irq_enable(int m0_extirq);

/****************************************************************************
 * Name: bl808_courier_req_irq_disable
 *
 * Description:
 *   Sends an IPC message to M0 core to disable m0_extirq.
 *
 ****************************************************************************/

void bl808_courier_req_irq_disable(int m0_extirq);

/****************************************************************************
 * Name: bl808_courier_init
 *
 * Description:
 *   Enables the IPC interrupt on D0 core and attaches its handler.
 *
 ****************************************************************************/

int bl808_courier_init(void);

#endif /* __ARCH_RISC_V_SRC_BL808_BL808_COURIER_H */
