/****************************************************************************
 * arch/xtensa/src/esp32/hardware/esp32_aes.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_AES_H
#define __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_AES_H

/* AES acceleration registers */

#define DR_REG_AES_BASE         (0x3ff01000)

#define AES_START_REG           ((DR_REG_AES_BASE) + 0x00)
#define AES_IDLE_REG            ((DR_REG_AES_BASE) + 0x04)
#define AES_MODE_REG            ((DR_REG_AES_BASE) + 0x08)
#define AES_KEY_BASE            ((DR_REG_AES_BASE) + 0x10)
#define AES_TEXT_BASE           ((DR_REG_AES_BASE) + 0x30)
#define AES_ENDIAN              ((DR_REG_AES_BASE) + 0x40)

/* AES start register */

#define AES_START_OPT           (BIT(0))

/* AES idle register */

#define AES_IDLE_STATE          (BIT(0))

/* AES mode register */

#define AES_MODE_DECRYPT        (BIT(2))

#endif /* __ARCH_XTENSA_SRC_ESP32_HARDWARE_ESP32_AES_H */
