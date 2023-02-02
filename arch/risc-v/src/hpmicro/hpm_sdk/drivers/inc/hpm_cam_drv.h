/*
 * Copyright (c) 2021 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef HPM_CAM_DRV_H
#define HPM_CAM_DRV_H

#include "hpm_common.h"
#include "hpm_display_common.h"
#include "hpm_cam_regs.h"

/**
 * @brief CAM driver APIs
 * @defgroup cam_interface CAM driver APIs
 * @ingroup io_interfaces
 * @{
 */

/**
 * @brief CAM data store mode
 */
#define CAM_DATA_STORE_MODE_NORMAL (0U)
#define CAM_DATA_STORE_MODE_Y_UV_PLANES (CAM_CR1_STORAGE_MODE_SET(1))
#define CAM_DATA_STORE_MODE_Y_ONLY (CAM_CR1_STORAGE_MODE_SET(2))
#define CAM_DATA_STORE_MODE_BINARY (CAM_CR1_STORAGE_MODE_SET(3))

/**
 * @brief CAM sensor bitwidth
 */
#define CAM_SENSOR_BITWIDTH_8BITS (CAM_CR1_SENSOR_BIT_WIDTH_SET(0))
#define CAM_SENSOR_BITWIDTH_10BITS (CAM_CR1_SENSOR_BIT_WIDTH_SET(1))

/**
 * @brief CAM IRQ flag
 */
#define CAM_IRQ_UNSUPPORTED_CONFIGURATION (CAM_INT_EN_ERR_CL_BWID_CFG_INT_EN_MASK)
#define CAM_IRQ_HIST_CALCULATION_DONE (CAM_INT_EN_HIST_DONE_INT_EN_MASK)
#define CAM_IRQ_HRESPONSE_ERROR (CAM_INT_EN_HRESP_ERR_EN_MASK)
#define CAM_IRQ_END_OF_FRAME (CAM_INT_EN_EOF_INT_EN_MASK)
#define CAM_IRQ_RX_FIFO_OVERRUN (CAM_INT_EN_RF_OR_INTEN_MASK)
#define CAM_IRQ_FB2_DMA_TRANSFER_DONE (CAM_INT_EN_FB2_DMA_DONE_INTEN_MASK)
#define CAM_IRQ_FB1_DMA_TRANSFER_DONE (CAM_INT_EN_FB1_DMA_DONE_INTEN_MASK)
#define CAM_IRQ_START_OF_FRAME (CAM_INT_EN_SOF_INT_EN_MASK)

/**
 * @brief CAM status flag
 */
#define CAM_STATUS_UNSUPPORTED_CONFIGURATION (CAM_STA_ERR_CL_BWID_CFG_MASK)
#define CAM_STATUS_HIST_CALCULATION_DONE (CAM_STA_HIST_DONE_MASK)
#define CAM_STATUS_RX_FIFO_OVERRUN (CAM_STA_RF_OR_INT_MASK)
#define CAM_STATUS_FB2_DMA_TRANSFER_DONE (CAM_STA_DMA_TSF_DONE_FB2_MASK)
#define CAM_STATUS_FB1_DMA_TRANSFER_DONE (CAM_STA_DMA_TSF_DONE_FB1_MASK)
#define CAM_STATUS_END_OF_FRAME (CAM_STA_EOF_INT_MASK)
#define CAM_STATUS_START_OF_FRAME (CAM_STA_SOF_INT_MASK)
#define CAM_STATUS_HRESPONSE_ERROR (CAM_STA_HRESP_ERR_INT_MASK)

/**
 * @brief CAM input color format
 */
#define CAM_COLOR_FORMAT_RGB888 (CAM_CR1_COLOR_FORMATS_SET(2))
#define CAM_COLOR_FORMAT_RGB565 (CAM_CR1_COLOR_FORMATS_SET(4))
#define CAM_COLOR_FORMAT_RGB555 (CAM_CR1_COLOR_FORMATS_SET(6))
#define CAM_COLOR_FORMAT_YCBCR422 (CAM_CR1_COLOR_FORMATS_SET(7))
#define CAM_COLOR_FORMAT_YUV444 (CAM_CR1_COLOR_FORMATS_SET(8))

/**
 * @brief CAM config
 */
typedef struct {
    uint32_t width;
    uint32_t height;
    bool pixclk_sampling_falling;
    bool hsync_active_low;
    bool vsync_active_low;
    bool color_ext;
    bool data_pack_msb;
    bool enable_buffer2;
    uint16_t data_store_mode;
    uint8_t color_format;
    uint8_t sensor_bitwidth;
    uint32_t buffer1;
    uint32_t buffer2;
    display_yuv2rgb_config_t csc_config;
} cam_config_t;

/**
 * @brief cam input pixel byte order
 */
typedef enum {
    cam_input_pixel_yuv444 = 0,    /* Y[23:16] U[15:8] V[7:0] */
    cam_input_pixel_yvu444 = 1,    /* Y[23:16] V[15:8] U[7:0] */
    cam_input_pixel_uyv444 = 2,    /* U[23:16] Y[15:8] V[7:0] */
    cam_input_pixel_vyu444 = 3,    /* V[23:16] Y[15:8] U[7:0] */
    cam_input_pixel_uvy444 = 4,    /* U[23:16] V[15:8] Y[7:0] */
    cam_input_pixel_vuy444 = 5,    /* V[23:16] U[15:8] Y[7:0] */
    cam_input_pixel_yuyv422 = 0,   /* Y0[31:24] U0[23:16] Y1[15:8] V0[7:0] */
    cam_input_pixel_yvyu422 = 1,   /* Y0[31:24] V0[23:16] Y1[15:8] U0[7:0] */
    cam_input_pixel_uyvy422 = 2,   /* U0[31:24] Y0[23:16] V0[15:8] Y1[7:0] */
    cam_input_pixel_vyuy422 = 3,   /* V0[31:24] Y0[23:16] U0[15:8] Y1[7:0] */
    cam_input_pixel_rgb565 = 0,    /* R[15:11] G[10:8] G[7:5] B[4:0] */
    cam_input_pixel_bgr565 = 1,    /* B[15:11] G[10:8] G[7:5] R[4:0] */
    cam_input_pixel_gbr888 = 0,    /* G[23:16] B[15:8] R[7:0] */
    cam_input_pixel_grb888 = 1,    /* G[23:16] R[15:8] B[7:0] */
    cam_input_pixel_bgr888 = 2,    /* B[23:16] G[15:8] R[7:0] */
    cam_input_pixel_rgb888 = 3,    /* R[23:16] G[15:8] B[7:0] */
    cam_input_pixel_brg888 = 4,    /* B[23:16] R[15:8] G[7:0] */
    cam_input_pixel_rbg888 = 5,    /* R[23:16] B[15:8] G[7:0] */
} cam_input_pixel_byte_order_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CAM set high and low limits of color key
 *
 * @param [in] ptr CAM base address
 * @param [in] high color key high limits
 * @param [in] low color key low limits
 */
static inline void cam_set_color_key(CAM_Type *ptr, uint32_t high, uint32_t low)
{
    ptr->CLRKEY_LOW = CAM_CLRKEY_LOW_LIMIT_SET(low);
    ptr->CLRKEY_HIGH = CAM_CLRKEY_HIGH_LIMIT_SET(high);
}

/**
 * @brief CAM get default config
 *
 * @param [in] ptr CAM base address
 * @param [out] config cam_config_t
 * @param [in] pixel_format display_pixel_format_t
 */
void cam_get_default_config(CAM_Type *ptr, cam_config_t *config, display_pixel_format_t pixel_format);

/**
 * @brief CAM init
 *
 * @param [in] ptr CAM base address
 * @param [in] config cam_config_t
 *
 * @retval hpm_stat_t status_invalid_argument or status_success
 */
hpm_stat_t cam_init(CAM_Type *ptr, cam_config_t *config);

/**
 * @brief CAM start
 *
 * @param [in] ptr CAM base address
 */
void cam_start(CAM_Type *ptr);

/**
 * @brief CAM stop
 *
 * @param [in] ptr CAM base address
 */
void cam_stop(CAM_Type *ptr);

void cam_update_buffer(CAM_Type *ptr, uint32_t buffer);

/**
 * @brief CAM enable binary output
 *
 * This function is used to enable CAM binary output after
 * the CAM is initialized by the cam_init.
 *
 * @param [in] ptr CAM base address
 */
static inline void cam_enable_binary_output(CAM_Type *ptr)
{
    ptr->CR20 |= CAM_CR20_BINARY_EN_MASK;
}

/**
 * @brief CAM disable binary output
 *
 * @param [in] ptr CAM base address
 */
static inline void cam_disable_binary_output(CAM_Type *ptr)
{
    ptr->CR20 &= ~CAM_CR20_BINARY_EN_MASK;
}

/**
 * @brief CAM set binary threshold
 *
 * @param [in] ptr CAM base address
 * @param [in] threshold threshold value of binary output
 */
static inline void cam_set_binary_threshold(CAM_Type *ptr, uint8_t threshold)
{
    ptr->CR20 = (ptr->CR20 & (~CAM_CR20_THRESHOLD_MASK)) | CAM_CR20_THRESHOLD_SET(threshold);
}

/**
 * @brief CAM enable argb8888 output
 *
 * This function is used to enable CAM argb8888 pixel output after the CAM is initialized by
 * the cam_init and input pixel byte order is configured by the cam_set_input_pixel_byte_order.
 *
 * @param [in] ptr CAM base address
 */
static inline void cam_enable_argb8888_output(CAM_Type *ptr)
{
    ptr->CR1 |= CAM_CR1_COLOR_EXT_MASK;
}

/**
 * @brief CAM disable argb8888 output
 *
 * @param [in] ptr CAM base address
 */
static inline void cam_disable_argb8888_output(CAM_Type *ptr)
{
    ptr->CR1 &= ~CAM_CR1_COLOR_EXT_MASK;
}

/**
 * @brief CAM set input pixel byte order
 *
 * @param [in] ptr CAM base address
 * @param [in] order cam_input_pixel_byte_order_t
 */
static inline void cam_set_input_pixel_byte_order(CAM_Type *ptr, cam_input_pixel_byte_order_t order)
{
    ptr->CR2 = (ptr->CR2 & (~CAM_CR2_CLRBITFORMAT_MASK)) | CAM_CR2_CLRBITFORMAT_SET(order);
}

/**
 * @}
 *
 */

#ifdef __cplusplus
}
#endif

#endif /* HPM_CAM_DRV_H */


