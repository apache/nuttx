/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_control_ex.c
 *
 * @brief   gh3x2x extension control functions, using function map like evk
 *
 * @version ref gh3x2x_drv_version.h
 *
 */


#include <stdio.h>
#include "gh3x2x_drv_version.h"
#include "gh3x2x_drv_common.h"
#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"
#include "gh3x2x_drv_zip.h"
#include "gh3x2x_drv_control.h"
#include "gh3x2x_drv_control_ex.h"
#include "gh3x2x_drv_soft_led_agc.h"
#include "gh3x2x_drv_dump.h"
#include "gh3x2x_drv_soft_adt.h"


/// get bitmap variable
#define  GH3X2X_GET_FUNC_START_BITMAP                   (g_unFuncStartedBitmap)

/// check bitmap is all off
#define  GH3X2X_IS_FUNC_START_BITMAP_ALL_OFF()          (g_unFuncStartedBitmap == GH3X2X_NO_FUNCTION)

/// fixed fifo thr variable @ stop
#define  GH3X2X_FIXED_FIFO_WATERMARK_THR()              do { g_usFuncLastFifoThrVal = g_usFuncFifoThrVal; } while (0)

#if (!GH3X2X_EX_SLOT_EN_FROM_CONFIG)
/// slot enable bits map
const uint16_t g_usSlotEnableBitsMapArr[] =
{
    GH3X2X_SLOT_INDEX_0, GH3X2X_SLOT_INDEX_1, GH3X2X_SLOT_INDEX_2, GH3X2X_SLOT_INDEX_3,
    GH3X2X_SLOT_INDEX_4, GH3X2X_SLOT_INDEX_5, GH3X2X_SLOT_INDEX_6, GH3X2X_SLOT_INDEX_7,
};
#endif

/// info: reg config array version
extern uint16_t g_usInfoConfigArrVer;

/// info: reg config create tool version
extern uint16_t g_usInfoConfigToolVer;

/// info: project id
extern uint16_t g_usInfoProjectId;

/// info: reg config created timestamp
extern uint32_t g_unInfoCreatedTimestamp;

/// slot enable for funcs,
extern uint16_t g_usSlotEnableBitsForFuncsArr[GH3020_SLOT_NUM_MAX];

/// function last fifo thr val
extern uint16_t g_usFuncLastFifoThrVal;

/// function the fifo thr val
extern uint16_t g_usFuncFifoThrVal;





/// G sensor enable flag
extern uint8_t g_uchGsensorEnable;

/// soft event
//extern uint8_t gubSoftEvent ;  //G202008231001 wanpeng

/// if support algorithm run simultaneously
//extern uint8_t g_uchAlgoRunMode;

/// enable gs movement detect or not
extern uint8_t g_uchAdtWithConfirmEnable;

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
int8_t GH3X2X_FuncStartedBitSet(uint16_t usFuncStartedBitmapVal)
{
    GH3X2X_VAL_SET_BIT(g_unFuncStartedBitmap, usFuncStartedBitmapVal);
    return GH3X2X_RET_OK;
}

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
int8_t GH3X2X_FuncStartedBitClear(uint16_t usFuncStartedBitmapVal)
{
    GH3X2X_VAL_CLEAR_BIT(g_unFuncStartedBitmap, usFuncStartedBitmapVal);
    return GH3X2X_RET_OK;
}

/**
 * @fn     int8_t GH3X2X_CustomizeFuncStartedBitSet(void)
 *
 * @brief  Set Customize started bitmap
 *
 * @attention   None
 *
 * @param[out]  None
 * @param[in]   None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return algorithm run successfully
 */
int8_t GH3X2X_CustomizeFuncStartedBitSet(void)
{
    return GH3X2X_FuncStartedBitSet(GH3X2X_STARTED_BITMAP_CUSTOMIZE);
}

/**
 * @fn     int8_t GH3X2X_CustomizeFuncStartedBitClear(void)
 *
 * @brief  Clear Customize started bitmap
 *
 * @attention   None
 *
 * @param[out]  None
 * @param[in]   None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return algorithm run successfully
 */
int8_t GH3X2X_CustomizeFuncStartedBitClear(void)
{
    return GH3X2X_FuncStartedBitClear(GH3X2X_STARTED_BITMAP_CUSTOMIZE);
}

/**
 * @fn     int8_t GH3X2X_SlotEnableEx(uint16_t usSlotEnableConfig, uint16_t usStartBitmap)
 *
 * @brief  Slot enable config
 *
 * @attention  Set slot enable and set slot map started bitmap
 *
 * @param[in]  usSlotEnableConfig         slot enable index , @ref GH3X2X_SLOT_INDEX_0 ... GH3X2X_SLOT_INDEX_ALL
 * @param[in]  usStartBitmap              start bitmap
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK                  return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR     return param error
 */
int8_t GH3X2X_SlotEnableEx(uint16_t usSlotEnableConfig, uint16_t usStartBitmap)
{
    int8_t chRet = GH3X2X_RET_OK;
    uint8_t uchSlotIndex = 0;
    uint16_t usSlotEnableConfigTmp = usSlotEnableConfig;

    chRet = GH3X2X_SlotEnableConfig(usSlotEnableConfigTmp, GH3X2X_SET_SLOT_ENABLE);
    if (chRet == GH3X2X_RET_OK)
    {
        for (uchSlotIndex = 0; uchSlotIndex < GH3020_SLOT_NUM_MAX; uchSlotIndex++)
        {
            if (GH3X2X_CHECK_LSB_SET(usSlotEnableConfigTmp))
            {
                GH3X2X_VAL_SET_BIT(g_usSlotEnableBitsForFuncsArr[uchSlotIndex], usStartBitmap);
            }
            GH3X2X_SET_VAL_RIGTH_SHIFT_1BIT(usSlotEnableConfigTmp);
        }
    }
    return chRet;
}

/**
 * @fn     int8_t GH3X2X_SlotDisableEx(uint16_t usSlotEnableConfig, uint16_t usStartBitmap)
 *
 * @brief  Slot disable config
 *
 * @attention  Set slot disable and set slot map started bitmap
 *
 * @param[in]  usSlotDisableConfig        slot disable index , @ref GH3X2X_SLOT_INDEX_0 ... GH3X2X_SLOT_INDEX_ALL
 * @param[in]  usStartBitmap              start bitmap
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK                  return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR     return param error
 */
int8_t GH3X2X_SlotDisableEx(uint16_t usSlotDisableConfig, uint16_t usStartBitmap)
{
    uint8_t uchSlotIndex = 0;
    uint16_t usSlotEnableConfigCalcTmp = usSlotDisableConfig;
    uint16_t usSlotEnableConfigReal = usSlotDisableConfig;
    int8_t chRet = GH3X2X_RET_OK;

    /* calc slot map funcs hasn't all off, slot shouldn't off */
    for (uchSlotIndex = 0; uchSlotIndex < GH3020_SLOT_NUM_MAX; uchSlotIndex++)
    {
        if (usSlotEnableConfigCalcTmp)
        {
            GH3X2X_VAL_CLEAR_BIT(g_usSlotEnableBitsForFuncsArr[uchSlotIndex], usStartBitmap);
            if (g_usSlotEnableBitsForFuncsArr[uchSlotIndex] != GH3X2X_STARTED_BITMAP_ALL_OFF)
            {
                GH3X2X_VAL_CLEAR_BIT(usSlotEnableConfigReal, GH3X2X_GET_LEFT_SHIFT_VAL(uchSlotIndex));
            }
        }
        GH3X2X_SET_VAL_RIGTH_SHIFT_1BIT(usSlotEnableConfigCalcTmp);
    }
    if (usSlotEnableConfigReal != 0)
    {
        chRet = GH3X2X_SlotEnableConfig(usSlotEnableConfigReal, GH3X2X_SET_SLOT_DISABLE);
    }
    return chRet;
}

/**
 * @fn     int8_t GH3X2X_FifoWatermarkThrConfigEx(uint16_t usFifoWatermarkThr)
 *
 * @brief  Fifo water mark threshold config
 *
 * @attention   Watermark threshold val must in (2, 800]. if val <= 2, will set 3, if val > 800, set 800;
 *              Be careful that fifo_use_cnt greater than val, gh3x2x willn't generate fifo_watermark interrupt after!
 *              Please configure according to usage, effect @GH3X2X_GetRawdataForAlgorithmEx last fifo Watermark irq!
 *
 * @param[in]   usFifoWatermarkThr         watermark threshold val
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 */
int8_t GH3X2X_FifoWatermarkThrConfigEx(uint16_t usFifoWatermarkThr)
{
    uint16_t usFifoWatermarkThrVal = usFifoWatermarkThr;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if (usFifoWatermarkThrVal < GH3X2X_FIFO_WATERMARK_THR_MIN)
    {
        GH3X2X_DEBUG_LOG("fifo wateramrk thr param less than min!\r\n");
        usFifoWatermarkThrVal = GH3X2X_FIFO_WATERMARK_THR_MIN;
    }
    else if (usFifoWatermarkThrVal > GH3X2X_FIFO_WATERMARK_THR_MAX)
    {
        GH3X2X_DEBUG_LOG("fifo wateramrk thr param greater than max!\r\n");
        usFifoWatermarkThrVal = GH3X2X_FIFO_WATERMARK_THR_MAX;
    }
    g_usFuncFifoThrVal = usFifoWatermarkThrVal;
    return GH3X2X_RET_OK;
}
#if 0
/**
 * @fn     int8_t GH3X2X_GetRawdataForAlgorithmEx(uint8_t *puchPackageDataBuffer)
 *
 * @brief  Get rawdata package for algorithm
 *
 * @attention   buffer size must define greater or equal to ((fifo_thr + n) * 4 + 5)
 *
 * @param[in]   None
 * @param[out]  puchPackageDataBuffer   pointer to package data output
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_RESOURCE_ERROR   resource error
 */
int8_t GH3X2X_GetRawdataForAlgorithmEx(uint8_t *puchPackageDataBuffer ,uint16_t uchFifoReadByteNum)
{
    int8_t chRet = GH3X2X_RET_OK;
    chRet = GH3X2X_GetRawdataForAlgorithm(puchPackageDataBuffer,uchFifoReadByteNum);
    GH3X2X_RET_ERROR_CHECK(chRet);

    if (g_usFuncLastFifoThrVal != g_usFuncFifoThrVal)
    {
        GH3X2X_FifoWatermarkThrConfig(g_usFuncFifoThrVal);
        g_usFuncLastFifoThrVal = g_usFuncFifoThrVal;
    }
    return chRet;
}
#endif








/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/
