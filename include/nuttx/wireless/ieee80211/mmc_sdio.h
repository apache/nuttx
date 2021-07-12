/****************************************************************************
 * include/nuttx/wireless/ieee80211/mmc_sdio.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE80211_MMC_SDIO_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE80211_MMC_SDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <nuttx/sdio.h>

#ifndef caca
#define caca

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

int sdio_probe(FAR struct sdio_dev_s *dev);

int sdio_set_wide_bus(struct sdio_dev_s *dev);

int sdio_set_blocksize(FAR struct sdio_dev_s *dev, uint8_t function,
                       uint16_t blocksize);

int sdio_enable_function(FAR struct sdio_dev_s *dev, uint8_t function);

int sdio_enable_interrupt(FAR struct sdio_dev_s *dev, uint8_t function);

int sdio_sendcmdpoll(FAR struct sdio_dev_s *dev,
                     uint32_t cmd, uint32_t arg);

int sdio_io_rw_direct(FAR struct sdio_dev_s *dev, bool write,
                      uint8_t function, uint32_t address,
                      uint8_t inb, uint8_t *outb);

int sdio_io_rw_extended(FAR struct sdio_dev_s *dev, bool write,
                        uint8_t function, uint32_t address,
                        bool inc_addr, uint8_t *buf,
                        unsigned int blocklen, unsigned int nblocks);

#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE80211_MMC_SDIO_H */
