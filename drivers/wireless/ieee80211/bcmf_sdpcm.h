#ifndef __DRIVERS_WIRELESS_IEEE80211_BCMF_SDPCM_H
#define __DRIVERS_WIRELESS_IEEE80211_BCMF_SDPCM_H

#include "bcmf_driver.h"
#include <stdint.h>

struct bcmf_sdpcm_header {
    uint16_t frametag[2];
    uint8_t  sequence;
    uint8_t  channel_flags;
    uint8_t  next_length;
    uint8_t  data_offset;
    uint8_t  flow_control;
    uint8_t  credit;
    uint16_t padding;
};

struct bcmf_sdpcm_cdc_dcmd {
    struct bcmf_sdpcm_header header;
    uint32_t cmd;    /* dongle command value */
    uint32_t len;    /* lower 16: output buflen;
                      * upper 16: input buflen (excludes header) */
    uint32_t flags;  /* flag defns given below */
    uint32_t status; /* status code returned from the device */
};

int bcmf_sdpcm_process_header(FAR struct bcmf_dev_s *priv,
                              struct bcmf_sdpcm_header *header);

int bcmf_sdpcm_readframe(FAR struct bcmf_dev_s *priv);

int bcmf_sdpcm_iovar_data_get(FAR struct bcmf_dev_s *priv, char *name,
                              void *data, unsigned int len);

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCMF_SDPCM_H */