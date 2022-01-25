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

#define  GH3X2X_LED_AGC_DISABLE             (0)
#define  GH3X2X_LED_AGC_ENABLE              (1)

/**
 * @brief soft lead result
 */
typedef struct
{
    GU8 uchAmbSampleEn  : 2;                /**< AMB sampling enable */
    GU8 uchAmbSlot      : 3;                /**< slot of amb sampling */
    GU8 uchRes          : 3;                /**< reserved */
} STAmbSlotCtrl;

 /**
 * @brief gain limit
 */
typedef struct
{
    GU8 uchGainLimitBg32uA  : 4;          /**< gain limit bg current */
    GU8 uchGainLimitBg64uA  : 4;          
    GU8 uchGainLimitBg128uA : 4;        
    GU8 uchGainLimitBg256uA : 4;
} STSoftAgcGainLimit;

 /**
  * @brief soft agc parameter
  */
typedef struct
{
    GU8 uchSlotxSoftAgcAdjEn;              /**< soft agc enable */
    GU8 uchSlotxSoftBgAdjEn;               /**< soft bg cancel adjust enable */
    STAmbSlotCtrl stAmbSlotCtrl;           /**< amb slot ctrl */
    GU8 uchRes0;                           /**< reserved */
    GU8 uchRes1;
    GU8 uchRes2;
    STSoftAgcGainLimit stSoftAgcGainLimit; /**< soft gain limit */
    GU32 unAgcTrigThd_H;                   /**< trig threshold(high) of soft agc */
    GU32 unAgcTrigThd_L;                   /**< trig threshold(low) of soft agc */
    GU32 unAgcRestrainThd_H;               /**< restrain threshold(high) of soft agc */
    GU32 unAgcRestrainThd_L;               /**< restrain threshold(low) of soft agc */
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
 * @fn     void GH3X2X_LedAgcPramWrite(GU16 usRegAddr, GU16 usRegData)
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
void GH3X2X_LedAgcPramWrite(GU16 usRegAddr, GU16 usRegData);

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
