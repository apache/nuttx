/**
 * @copyright (c) 2003 - 2020, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_control_ex.h
 *
 * @brief   gh3x2x extension control define & declaration, using function map like evk
 *
 * @version ref gh3x2x_drv_version.h
 *
 */

#ifndef _GH3X2X_DRV_CONTROL_EX_H_
#define _GH3X2X_DRV_CONTROL_EX_H_


#include "gh3x2x_drv_config.h"
#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"



/* channel map cnt max define */
#if GH3X2X_EX_CHANNEL_CNT_FOLLOW_ALGORITHMS

/// hba func channel max cnt support
#define   GH3X2X_HBA_CHANNEL_MAP_MAX_CNT            (HBA_PPG_CHANNEL_NUM)

/// hrv func channel max cnt support
#define   GH3X2X_HRV_CHANNEL_MAP_MAX_CNT            (HRV_PPG_CHANNEL_NUM)

/// spo2 func channel max cnt support
#define   GH3X2X_SPO2_CHANNEL_MAP_MAX_CNT           (SPO2_PPG_CHANNEL_NUM)

/// ecg func channel max cnt support
#define   GH3X2X_ECG_CHANNEL_MAP_MAX_CNT            (ECG_CHANNEL_NUM)

/// test1 func channel max cnt support
#define   GH3X2X_TEST1_CHANNEL_MAP_MAX_CNT          (GH3X2X_CHANNEL_MAP_MAX_CH)

/// test2 func channel max cnt support
#define   GH3X2X_TEST2_CHANNEL_MAP_MAX_CNT          (GH3X2X_CHANNEL_MAP_MAX_CH)

/// adt func channel max cnt support
#define   GH3X2X_ADT_CHANNEL_MAP_MAX_CNT            (4u)

/// hsm func channel max cnt support
#define   GH3X2X_HSM_CHANNEL_MAP_MAX_CNT            (GH3X2X_CHANNEL_MAP_MAX_CH)

/// bt func channel max cnt support
#define   GH3X2X_BT_CHANNEL_MAP_MAX_CNT             (1u)

/// resp func channel max cnt support
#define   GH3X2X_RESP_CHANNEL_MAP_MAX_CNT           (GH3X2X_CHANNEL_MAP_MAX_CH)

/// af func channel max cnt support
#define   GH3X2X_AF_CHANNEL_MAP_MAX_CNT             (GH3X2X_CHANNEL_MAP_MAX_CH)

/// fpbp func channel max cnt support
#define   GH3X2X_FPBP_CHANNEL_MAP_MAX_CNT           (GH3X2X_CHANNEL_MAP_MAX_CH)

/// pwa func channel max cnt support
#define   GH3X2X_PWA_CHANNEL_MAP_MAX_CNT            (GH3X2X_CHANNEL_MAP_MAX_CH)


#else

/// hba func channel max cnt support
#define   GH3X2X_HBA_CHANNEL_MAP_MAX_CNT            (GH3X2X_CHANNEL_MAP_MAX_CH)

/// hrv func channel max cnt support
#define   GH3X2X_HRV_CHANNEL_MAP_MAX_CNT            (GH3X2X_CHANNEL_MAP_MAX_CH)

/// spo2 func channel max cnt support
#define   GH3X2X_SPO2_CHANNEL_MAP_MAX_CNT           (GH3X2X_CHANNEL_MAP_MAX_CH)

/// ecg func channel max cnt support
#define   GH3X2X_ECG_CHANNEL_MAP_MAX_CNT            (ECG_CHANNEL_NUM)

/// adt func channel max cnt support
#define   GH3X2X_ADT_CHANNEL_MAP_MAX_CNT            (4u)

/// hsm func channel max cnt support
#define   GH3X2X_HSM_CHANNEL_MAP_MAX_CNT            (GH3X2X_CHANNEL_MAP_MAX_CH)

/// bt func channel max cnt support
#define   GH3X2X_BT_CHANNEL_MAP_MAX_CNT             (1u)

/// resp func channel max cnt support
#define   GH3X2X_RESP_CHANNEL_MAP_MAX_CNT           (GH3X2X_CHANNEL_MAP_MAX_CH)

/// af func channel max cnt support
#define   GH3X2X_AF_CHANNEL_MAP_MAX_CNT             (GH3X2X_CHANNEL_MAP_MAX_CH)

/// fpbp func channel max cnt support
#define   GH3X2X_FPBP_CHANNEL_MAP_MAX_CNT           (GH3X2X_CHANNEL_MAP_MAX_CH)

/// pwa func channel max cnt support
#define   GH3X2X_PWA_CHANNEL_MAP_MAX_CNT            (GH3X2X_CHANNEL_MAP_MAX_CH)

/// test1 func channel max cnt support
#define   GH3X2X_TEST1_CHANNEL_MAP_MAX_CNT          (GH3X2X_CHANNEL_MAP_MAX_CH)

/// test2 func channel max cnt support
#define   GH3X2X_TEST2_CHANNEL_MAP_MAX_CNT          (GH3X2X_CHANNEL_MAP_MAX_CH)



#endif

//G202008031001 wanpeng START
#define SOFT_EVENT_NEED_FORCE_READ_FIFO             ((uint8_t)1)
#define SOFT_EVENT_WEAR_OFF                         ((uint8_t)2)
//G202008031001 wanpeng END




extern uint8_t g_uchAdtChannelMapArr[];
extern uint8_t g_uchAdtChannelMapCnt;

extern uint8_t g_uchHbaChannelMapArr[];
extern uint8_t g_uchHbaChannelMapCnt;

extern uint8_t g_uchSpo2ChannelMapArr[];
extern uint8_t g_uchSpo2ChannelMapCnt;

extern uint8_t g_uchHrvChannelMapArr[];
extern uint8_t g_uchHrvChannelMapCnt;

extern uint8_t g_uchEcgChannelMap;
extern uint8_t g_uchEcgChannelMapCnt;






/**
 * @fn     int8_t GH3X2X_FuncStartedBitSet(uint16_t usFuncStartedBitmapVal)
 *
 * @brief  Set started bitmap
 *
 * @attention   None
 *
 * @param[out]  None
 * @param[in]   usFuncStartedBitmapVal      bitmap val
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return algorithm run successfully
 */
int8_t GH3X2X_FuncStartedBitSet(uint16_t usFuncStartedBitmapVal);

/**
 * @fn     int8_t GH3X2X_FuncStartedBitClear(uint16_t usFuncStartedBitmapVal)
 *
 * @brief  Clear started bitmap
 *
 * @attention   None
 *
 * @param[out]  None
 * @param[in]   usFuncStartedBitmapVal      bitmap val
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return algorithm run successfully
 */
int8_t GH3X2X_FuncStartedBitClear(uint16_t usFuncStartedBitmapVal);

/**
 * @fn     int8_t GH3X2X_SoftAdtFuncStart(void)
 *
 * @brief  Soft adt function start
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return func start error
 * @retval  #GH3X2X_RET_NO_INITED_ERROR         gh3x2x has not inited
 */
int8_t GH3X2X_SoftAdtFuncStart(void);

/**
 * @fn     int8_t GH3X2X_SoftAdtFuncStop(void)
 *
 * @brief  Soft adt function stop
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return func stop error
 * @retval  #GH3X2X_RET_NO_INITED_ERROR         gh3x2x has not inited
 */
int8_t GH3X2X_SoftAdtFuncStop(void);








#endif /* _GH3X2X_DRV_CONTROL_EX_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/
