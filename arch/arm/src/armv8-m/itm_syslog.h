/****************************************************************************
 * arch/arm/src/armv8-m/itm_syslog.h
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

#ifndef __ARCH_ARM_SRC_ARMV8_M_ITM_SYSLOG_H
#define __ARCH_ARM_SRC_ARMV8_M_ITM_SYSLOG_H

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: itm_syslog_initialize
 *
 * Description:
 *   Performs ARM-specific initialize for the ITM SYSLOG functions.
 *   Additional, board specific logic may be required to:
 *
 *   - Enable/configured serial wire output pins
 *   - Enable debug clocking.
 *
 *   Those operations must be performed by MCU-specific logic before this
 *   function is called.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV8M_ITMSYSLOG
void itm_syslog_initialize(void);
#else
#  define itm_syslog_initialize()
#endif

#endif /* __ARCH_ARM_SRC_ARMV8_M_ITM_SYSLOG_H */
