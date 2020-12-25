/****************************************************************************
 * arch/xtensa/src/esp32/rom/esp32_efuse.h
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

#ifndef __XTENSA_SRC_ESP32_ROM_ESP32_EFUSE_H
#define __XTENSA_SRC_ESP32_ROM_ESP32_EFUSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ets_efuse_read_op
 ****************************************************************************/

void ets_efuse_read_op(void);

/****************************************************************************
 * Name: ets_efuse_program_op
 ****************************************************************************/

void ets_efuse_program_op(void);

/****************************************************************************
 * Name: ets_efuse_get_8m_clock
 ****************************************************************************/

uint32_t ets_efuse_get_8m_clock(void);

/****************************************************************************
 * name: ets_efuse_get_spiconfig
 ****************************************************************************/

uint32_t ets_efuse_get_spiconfig(void);

#define EFUSE_SPICFG_SPI_DEFAULTS      0
#define EFUSE_SPICFG_HSPI_DEFAULTS     1

#define EFUSE_SPICFG_RET_SPICLK_MASK   0x3f
#define EFUSE_SPICFG_RET_SPICLK_SHIFT  0
#define EFUSE_SPICFG_RET_SPICLK(ret)   (((ret) >> EFUSE_SPICFG_RET_SPICLK_SHIFT) & EFUSE_SPICFG_RET_SPICLK_MASK)

#define EFUSE_SPICFG_RET_SPIQ_MASK     0x3f
#define EFUSE_SPICFG_RET_SPIQ_SHIFT    6
#define EFUSE_SPICFG_RET_SPIQ(ret)     (((ret) >> EFUSE_SPICFG_RET_SPIQ_SHIFT) & EFUSE_SPICFG_RET_SPIQ_MASK)

#define EFUSE_SPICFG_RET_SPID_MASK     0x3f
#define EFUSE_SPICFG_RET_SPID_SHIFT    12
#define EFUSE_SPICFG_RET_SPID(ret)     (((ret) >> EFUSE_SPICFG_RET_SPID_SHIFT) & EFUSE_SPICFG_RET_SPID_MASK)

#define EFUSE_SPICFG_RET_SPICS0_MASK   0x3f
#define EFUSE_SPICFG_RET_SPICS0_SHIFT  18
#define EFUSE_SPICFG_RET_SPICS0(ret)   (((ret) >> EFUSE_SPICFG_RET_SPICS0_SHIFT) & EFUSE_SPICFG_RET_SPICS0_MASK)

#define EFUSE_SPICFG_RET_SPIHD_MASK    0x3f
#define EFUSE_SPICFG_RET_SPIHD_SHIFT   24
#define EFUSE_SPICFG_RET_SPIHD(ret)    (((ret) >> EFUSE_SPICFG_RET_SPIHD_SHIFT) & EFUSE_SPICFG_RET_SPIHD_MASK)

/****************************************************************************
 * name: eps_crc8
 ****************************************************************************/

unsigned char esp_crc8(unsigned char const *p, unsigned int len);

#ifdef __cplusplus
}
#endif

#endif /* __XTENSA_SRC_ESP32_ROM_ESP32_EFUSE_H */
