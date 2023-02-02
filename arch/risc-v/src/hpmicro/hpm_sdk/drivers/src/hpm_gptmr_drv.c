/*
 * Copyright (c) 2021 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "hpm_gptmr_drv.h"

void gptmr_channel_get_default_config(GPTMR_Type *ptr, gptmr_channel_config_t *config)
{
    config->mode = gptmr_work_mode_no_capture;
    config->dma_request_event = gptmr_dma_request_disabled;
    config->synci_edge = gptmr_synci_edge_none;
    for (uint8_t i = 0; i < GPTMR_CH_CMP_COUNT; i++) {
        config->cmp[i] = 0;
    }
    config->reload = 0xFFFFFFFFUL;
    config->cmp_initial_polarity_high = true;
    config->enable_cmp_output = true;
    config->enable_sync_follow_previous_channel = false;
    config->enable_software_sync = false;
    config->debug_mode = true;
}

hpm_stat_t gptmr_channel_config(GPTMR_Type *ptr,
                         uint8_t ch_index,
                         gptmr_channel_config_t *config,
                         bool enable)
{
    uint32_t v = 0;
    if ((config->enable_sync_follow_previous_channel && !ch_index) || (config->reload == 0)) {
        return status_invalid_argument;
    }

    if (config->dma_request_event != gptmr_dma_request_disabled) {
        v |= GPTMR_CHANNEL_CR_DMAEN_MASK
            | GPTMR_CHANNEL_CR_DMASEL_SET(config->dma_request_event);
    }
    v |= GPTMR_CHANNEL_CR_CAPMODE_SET(config->mode)
        | GPTMR_CHANNEL_CR_DBGPAUSE_SET(config->debug_mode)
        | GPTMR_CHANNEL_CR_SWSYNCIEN_SET(config->enable_software_sync)
        | GPTMR_CHANNEL_CR_CMPINIT_SET(config->cmp_initial_polarity_high)
        | GPTMR_CHANNEL_CR_SYNCFLW_SET(config->enable_sync_follow_previous_channel)
        | GPTMR_CHANNEL_CR_CMPEN_SET(config->enable_cmp_output)
        | GPTMR_CHANNEL_CR_CEN_SET(enable)
        | config->synci_edge;

    for (uint8_t i = 0; i < GPTMR_CH_CMP_COUNT; i++) {
        ptr->CHANNEL[ch_index].CMP[i] = GPTMR_CHANNEL_CMP_CMP_SET(config->cmp[i]);
    }
    ptr->CHANNEL[ch_index].RLD = GPTMR_CHANNEL_RLD_RLD_SET(config->reload - 1);
    ptr->CHANNEL[ch_index].CR = v;
    return status_success;
}
