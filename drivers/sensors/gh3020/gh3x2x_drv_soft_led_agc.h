/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_soft_led_agc.h
 *
 * @brief   gh3x2x led agc process functions
 *
 * @version ref gh3x2x_drv_version.h
 *
 */


#ifndef _GH3X2X_DRV_SOFT_LED_AGC_H_
#define _GH3X2X_DRV_SOFT_LED_AGC_H_


#include "gh3x2x_drv_config.h"
#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"

#define  GH3X2X_LED_AGC_DISABLE             (0)
#define  GH3X2X_LED_AGC_ENABLE              (1)

/**
 * @brief soft lead result
 */
typedef struct
{
    uint8_t uchAmbSampleEn  : 2;                /**< AMB sampling enable */
    uint8_t uchAmbSlot      : 3;                /**< slot of amb sampling */
    uint8_t uchRes          : 3;                /**< reserved */
} STAmbSlotCtrl;

 /**
 * @brief gain limit
 */
typedef struct
{
    uint8_t uchGainLimitBg32uA  : 4;          /**< gain limit bg current */
    uint8_t uchGainLimitBg64uA  : 4;
    uint8_t uchGainLimitBg128uA : 4;
    uint8_t uchGainLimitBg256uA : 4;
} STSoftAgcGainLimit;

 /**
  * @brief soft agc parameter
  */
typedef struct
{
    uint8_t uchSlotxSoftAgcAdjEn;              /**< soft agc enable */
    uint8_t uchSlotxSoftBgAdjEn;               /**< soft bg cancel adjust enable */
    STAmbSlotCtrl stAmbSlotCtrl;           /**< amb slot ctrl */
    uint8_t uchRes0;                           /**< reserved */
    uint8_t uchRes1;
    uint8_t uchRes2;
    STSoftAgcGainLimit stSoftAgcGainLimit; /**< soft gain limit */
    uint32_t unAgcTrigThd_H;                   /**< trig threshold(high) of soft agc */
    uint32_t unAgcTrigThd_L;                   /**< trig threshold(low) of soft agc */
    uint32_t unAgcRestrainThd_H;               /**< restrain threshold(high) of soft agc */
    uint32_t unAgcRestrainThd_L;               /**< restrain threshold(low) of soft agc */
} STSoftAgcCfg;

/**
 * @fn  void GH3X2X_LedAgcReset(void)
 *
 * @brief   reset agc parameter
 *
 * @attention    None
 *
 * @param[in]    None
 * @param[out]   None
 *
 * @return  None
 */
void GH3X2X_LedAgcReset(void);

/**
 * @fn     void GH3X2X_LedAgcPramWrite(uint16_t usRegAddr, uint16_t usRegData)
 *
 * @brief    write AGC parameter
 *
 * @attention     None
 *
 * @param[in]     usRegAddr           reg addr
 * @param[in]     usRegData           reg data
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_LedAgcPramWrite(uint16_t usRegAddr, uint16_t usRegData);

/**
 * @fn     static void GH3X2X_LedAgcInit(void)
 *
 * @brief  init led agc
 *
 * @attention   just use to show function support
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_LedAgcInit(void);

/**
 * @fn     void GH3X2X_LedAgc_Close(void)
 *
 * @brief  close agc
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_LedAgc_Close(void);

/**
 * @fn     void GH3X2X_LedAgc_Open(void)
 *
 * @brief  open agc
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_LedAgc_Open(void);

#endif  // _GH3X2X_DRV_SOFT_LED_AGC_H_

/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/
