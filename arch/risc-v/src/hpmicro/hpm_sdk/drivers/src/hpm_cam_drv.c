/*
 * Copyright (c) 2021 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "hpm_cam_drv.h"

#define CAM_RX_FIFO_THRESHOLD (6U)

void cam_get_default_config(CAM_Type *ptr, cam_config_t *config, display_pixel_format_t pixel_format)
{
    config->width = 320;
    config->height = 240;
    config->pixclk_sampling_falling = false;
    config->hsync_active_low = false;
    config->vsync_active_low = false;
    config->color_ext = false;
    config->data_pack_msb = false;
    config->enable_buffer2 = false;
    config->data_store_mode = CAM_DATA_STORE_MODE_NORMAL;
    config->color_format = pixel_format;
    config->sensor_bitwidth = CAM_SENSOR_BITWIDTH_10BITS;

    switch (pixel_format) {
    case display_pixel_format_yuv422:
        config->csc_config.enable = true;
        config->csc_config.ycbcr_mode = false;
        config->csc_config.yuv2rgb_coef.c0 = 0x100;
        config->csc_config.yuv2rgb_coef.uv_offset = 0;
        config->csc_config.yuv2rgb_coef.y_offset = 0;
        config->csc_config.yuv2rgb_coef.c1 = 0x123;
        config->csc_config.yuv2rgb_coef.c2 = 0x76B;
        config->csc_config.yuv2rgb_coef.c3 = 0x79C;
        config->csc_config.yuv2rgb_coef.c4 = 0x208;
        break;
    case display_pixel_format_ycbcr422:
        config->csc_config.enable = true;
        config->csc_config.ycbcr_mode = true;
        config->csc_config.yuv2rgb_coef.c0 = 0x12A;
        config->csc_config.yuv2rgb_coef.uv_offset = 0x180;
        config->csc_config.yuv2rgb_coef.y_offset = 0x1F0;
        config->csc_config.yuv2rgb_coef.c1 = 0x198;
        config->csc_config.yuv2rgb_coef.c2 = 0x730;
        config->csc_config.yuv2rgb_coef.c3 = 0x79C;
        config->csc_config.yuv2rgb_coef.c4 = 0x204;
        break;
    default:
        config->csc_config.enable = false;
        config->csc_config.ycbcr_mode = false;
        config->csc_config.yuv2rgb_coef.c0 = 0;
        config->csc_config.yuv2rgb_coef.uv_offset = 0;
        config->csc_config.yuv2rgb_coef.y_offset = 0;
        config->csc_config.yuv2rgb_coef.c1 = 0;
        config->csc_config.yuv2rgb_coef.c2 = 0;
        config->csc_config.yuv2rgb_coef.c3 = 0;
        config->csc_config.yuv2rgb_coef.c4 = 0;
        break;
    }
}

void cam_reset(CAM_Type *ptr)
{
    cam_stop(ptr);
    ptr->CR1 = CAM_CR1_ASYNC_RXFIFO_CLR_MASK;
    ptr->INT_EN = 0;
    ptr->CR2 = CAM_CR2_FRMCNT_RST_MASK;
    ptr->STA = 0xFFFFFFFF;
    ptr->CR20 = 0;
}

hpm_stat_t cam_init(CAM_Type *ptr, cam_config_t *config)
{
    hpm_stat_t stat = status_success;

    cam_reset(ptr);

    ptr->CR1 = CAM_CR1_INV_PIXCLK_SET(config->pixclk_sampling_falling)
        | CAM_CR1_INV_HSYNC_SET(config->hsync_active_low)
        | CAM_CR1_INV_VSYNC_SET(config->vsync_active_low)
        | CAM_CR1_RESTART_BUSPTR_MASK
        | CAM_CR1_COLOR_EXT_SET(config->color_ext)
        | CAM_CR1_PACK_DIR_SET(config->data_pack_msb)
        | config->data_store_mode
        | config->color_format
        | config->sensor_bitwidth;

    ptr->IDEAL_WN_SIZE = CAM_IDEAL_WN_SIZE_HEIGHT_SET(config->height)
        | CAM_IDEAL_WN_SIZE_WIDTH_SET(config->width);

    ptr->MAX_WN_CYCLE = CAM_MAX_WN_CYCLE_ROW_SET(1200)
        | CAM_MAX_WN_CYCLE_COL_SET(2090);

    ptr->CR2 = CAM_CR2_DMA_REQ_EN_RFF_MASK
        | CAM_CR2_RXFF_LEVEL_SET(CAM_RX_FIFO_THRESHOLD);
    ptr->DMASA_FB1 = config->buffer1;
    if (config->enable_buffer2) {
        ptr->DMASA_FB2 = config->buffer2;
    }

    ptr->CSC_COEF0 = CAM_CSC_COEF0_ENABLE_SET(config->csc_config.enable)
                    | CAM_CSC_COEF0_YCBCR_MODE_SET(config->csc_config.ycbcr_mode)
                    | CAM_CSC_COEF0_C0_SET(config->csc_config.yuv2rgb_coef.c0)
                    | CAM_CSC_COEF0_UV_OFFSET_SET(config->csc_config.yuv2rgb_coef.uv_offset)
                    | CAM_CSC_COEF0_Y_OFFSET_SET(config->csc_config.yuv2rgb_coef.y_offset);
    ptr->CSC_COEF1 = CAM_CSC_COEF1_C1_SET(config->csc_config.yuv2rgb_coef.c1)
                    | CAM_CSC_COEF1_C4_SET(config->csc_config.yuv2rgb_coef.c4);
    ptr->CSC_COEF2 = CAM_CSC_COEF2_C2_SET(config->csc_config.yuv2rgb_coef.c2)
                    | CAM_CSC_COEF2_C3_SET(config->csc_config.yuv2rgb_coef.c3);

    return stat;
}

void cam_update_buffer(CAM_Type *ptr, uint32_t buffer)
{
    ptr->DMASA_FB1 = buffer;
}

void cam_stop(CAM_Type *ptr)
{
    ptr->CR18 &= ~CAM_CR18_CAM_ENABLE_MASK;
}

void cam_start(CAM_Type *ptr)
{
    ptr->CR18 |= CAM_CR18_CAM_ENABLE_MASK;
}

