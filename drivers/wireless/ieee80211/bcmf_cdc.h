#ifndef __DRIVERS_WIRELESS_IEEE80211_BCMF_CDC_H
#define __DRIVERS_WIRELESS_IEEE80211_BCMF_CDC_H

#include "bcmf_driver.h"
#include <stdbool.h>
#include <stdint.h>

int bcmf_cdc_iovar_request(FAR struct bcmf_dev_s *priv, uint32_t ifidx,
                           bool set, char *name, uint8_t *data, uint32_t *len);

int bcmf_cdc_ioctl(FAR struct bcmf_dev_s *priv, uint32_t ifidx, bool set,
                   uint32_t cmd, uint8_t *data, uint32_t *len);

int bcmf_cdc_process_control_frame(FAR struct bcmf_dev_s *priv,
								   struct bcmf_frame_s *frame);

int bcmf_cdc_process_data_frame(FAR struct bcmf_dev_s *priv,
								struct bcmf_frame_s *frame);

int bcmf_cdc_process_event_frame(FAR struct bcmf_dev_s *priv,
								 struct bcmf_frame_s *frame);

#endif /* __DRIVERS_WIRELESS_IEEE80211_BCMF_CDC_H */