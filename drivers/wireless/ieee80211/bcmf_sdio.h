#ifndef __DRIVERS_WIRELESS_IEEE80211_BCMF_SDIO_H
#define __DRIVERS_WIRELESS_IEEE80211_BCMF_SDIO_H

#include "bcmf_sdio_regs.h"
#include "bcmf_sdio_core.h"

#include "bcmf_driver.h"
#include <stdint.h>
#include <stdbool.h>

int bcmf_transfer_bytes(FAR struct bcmf_dev_s *priv, bool write,
                         uint8_t function, uint32_t address,
                         uint8_t *buf, unsigned int len);

int bcmf_read_reg(FAR struct bcmf_dev_s *priv, uint8_t function,
                  uint32_t address, uint8_t *reg);

int bcmf_write_reg(FAR struct bcmf_dev_s *priv, uint8_t function,
                   uint32_t address, uint8_t reg);

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCMF_SDIO_H */