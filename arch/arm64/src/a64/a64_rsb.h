/****************************************************************************
 * arch/arm64/src/a64/a64_rsb.h
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

#ifndef __ARCH_ARM64_SRC_A64_A64_RSB_H
#define __ARCH_ARM64_SRC_A64_A64_RSB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: a64_rsb_read
 *
 * Description:
 *   Read a byte from a device on the Reduced Serial Bus.
 *
 * Input Parameters:
 *   rt_addr  - Run-Time Address of RSB Device
 *   reg_addr - Register Address of RSB Device
 *
 * Returned Value:
 *   Byte read from the RSB Device; ERROR if the read failed or timed out.
 *
 ****************************************************************************/

int a64_rsb_read(uint8_t rt_addr, uint8_t reg_addr);

/****************************************************************************
 * Name: a64_rsb_write
 *
 * Description:
 *   Write a byte to a device on the Reduced Serial Bus.
 *
 * Input Parameters:
 *   rt_addr  - Run-Time Address of RSB Device
 *   reg_addr - Register Address of RSB Device
 *   value    - Byte to be written
 *
 * Returned Value:
 *   Zero (OK) on success; ERROR if the write failed or timed out.
 *
 ****************************************************************************/

int a64_rsb_write(uint8_t rt_addr, uint8_t reg_addr, uint8_t value);

#endif /* __ARCH_ARM64_SRC_A64_A64_RSB_H */
