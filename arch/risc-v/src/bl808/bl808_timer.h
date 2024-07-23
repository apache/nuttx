/****************************************************************************
 * arch/risc-v/src/bl808/bl808_timer.h
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

#ifndef __ARCH_RISC_V_SRC_BL808_BL808_TIMER_H
#define __ARCH_RISC_V_SRC_BL808_BL808_TIMER_H

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bl808_timer_init
 *
 * Description:
 *   Initialize timer hardware and register character drivers
 *   for enabled timer channels.
 *
 ****************************************************************************/

int bl808_timer_init(void);

#endif /* __ARCH_RISC_V_SRC_BL808_BL808_TIMER_H */
