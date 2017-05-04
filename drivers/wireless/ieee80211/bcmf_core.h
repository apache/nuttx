/****************************************************************************
 * drivers/wireless/ieee80211/bcmf_core.h
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

#ifndef __DRIVERS_WIRELESS_IEEE80211_BCMF_CORE_H
#define __DRIVERS_WIRELESS_IEEE80211_BCMF_CORE_H

#include <stdint.h>
#include <stdbool.h>

#include "bcmf_sdio.h"

int bcmf_read_sbreg(FAR struct bcmf_sdio_dev_s *sbus, uint32_t address,
                          uint8_t *reg, unsigned int len);

int bcmf_write_sbreg(FAR struct bcmf_sdio_dev_s *sbus, uint32_t address,
                          uint8_t *reg, unsigned int len);

bool bcmf_core_isup(FAR struct bcmf_sdio_dev_s *sbus, unsigned int core);

void bcmf_core_disable(FAR struct bcmf_sdio_dev_s *sbus, unsigned int core);

void bcmf_core_reset(FAR struct bcmf_sdio_dev_s *sbus, unsigned int core);

int bcmf_core_upload_firmware(FAR struct bcmf_sdio_dev_s *sbus);

static inline int bcmf_read_sbregb(FAR struct bcmf_sdio_dev_s *sbus,
                                   uint32_t address, uint8_t *reg)
{
    return bcmf_read_sbreg(sbus, address, reg, 1);
}

static inline int bcmf_read_sbregw(FAR struct bcmf_sdio_dev_s *sbus,
                                   uint32_t address, uint32_t *reg)
{
    return bcmf_read_sbreg(sbus, address, (uint8_t *)reg, 4);
}

static inline int bcmf_write_sbregb(FAR struct bcmf_sdio_dev_s *sbus,
                                    uint32_t address, uint8_t reg)
{
    return bcmf_write_sbreg(sbus, address, &reg, 1);
}

static inline int bcmf_write_sbregw(FAR struct bcmf_sdio_dev_s *sbus,
                                    uint32_t address, uint32_t reg)
{
    return bcmf_write_sbreg(sbus, address, (uint8_t *)&reg, 4);
}

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCMF_CORE_H */
