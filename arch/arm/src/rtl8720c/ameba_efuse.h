/****************************************************************************
 * arch/arm/src/rtl8720c/ameba_efuse.h
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

#ifndef __ARCH_ARM_SRC_RTL8720C_AMEBA_EFUSE_H
#define __ARCH_ARM_SRC_RTL8720C_AMEBA_EFUSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* #include "device.h" */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int ameba_efuse_logical_read(uint16_t laddr, uint16_t size, uint8_t *pbuf);
int ameba_efuse_logical_write(uint16_t addr, uint16_t cnts, uint8_t *data);
int ameba_efuse_fw_verify_enable(void);
int ameba_efuse_fw_verify_check(void);
int ameba_efuse_boot_message_disable(void);
int ameba_efuse_boot_message_enable(void);
#endif /* __ARCH_ARM_SRC_RTL8720C_AMEBA_EFUSE_H */
