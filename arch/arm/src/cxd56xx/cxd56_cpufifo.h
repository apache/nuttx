/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cpufifo.h
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

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_CPUFIFO_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_CPUFIFO_H

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

typedef int (*cpufifo_handler_t)(int cpuid, uint32_t data[2]);

int  cxd56_cfinitialize(void);
int  cxd56_cfpush(uint32_t data[2]);
int  cxd56_cfpull(uint32_t data[2]);
int  cxd56_cfregrxhandler(cpufifo_handler_t handler);
void cxd56_cfunregrxhandler(void);

#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_CPUFIFO_H */
