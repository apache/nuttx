/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_timersvc.h
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

#ifndef __ARCH_RISCV_SRC_RV32M1_RV32M1_TIMERSVC_H
#define __ARCH_RISCV_SRC_RV32M1_RV32M1_TIMERSVC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: rv32m1_timersvc_up
 ****************************************************************************/

EXTERN bool rv32m1_timersvc_up(void);

/****************************************************************************
 * Name: rv32m1_timersvc_freq
 ****************************************************************************/

EXTERN uint32_t rv32m1_timersvc_freq(void);

/****************************************************************************
 * Name: rv32m1_timersvc_period
 ****************************************************************************/

EXTERN uint32_t rv32m1_timersvc_period(void);

/****************************************************************************
 * Name: rv32m1_timersvc_value
 ****************************************************************************/

EXTERN uint32_t rv32m1_timersvc_value(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_RV32M1_RV32M1_TIMERSVC_H */
