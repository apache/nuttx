/****************************************************************************
 * include/nuttx/wireless/ieee80211/mmc_sdio.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
