#ifndef __DRIVERS_WIRELESS_IEEE80211_BCMF_SDPCM_H
#define __DRIVERS_WIRELESS_IEEE80211_BCMF_SDPCM_H

#include "bcmf_driver.h"

int bcmf_sdpcm_readframe(FAR struct bcmf_dev_s *priv);

int bcmf_sdpcm_sendframe(FAR struct bcmf_dev_s *priv);

int bcmf_sdpcm_queue_frame(FAR struct bcmf_dev_s *priv,
                           struct bcmf_frame_s *frame);

struct bcmf_frame_s* bcmf_sdpcm_allocate_frame(FAR struct bcmf_dev_s *priv,
                                unsigned int len, bool control, bool block);

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCMF_SDPCM_H */