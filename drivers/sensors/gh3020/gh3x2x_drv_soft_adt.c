/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_adt.c
 *
 * @brief   gh3x2x adt functions
 *
 * @version ref gh3x2x_drv_version.h
 *
 */


#include <stdio.h>
#include "gh3x2x_drv_version.h"
#include "gh3x2x_drv_common.h"
#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"
#include "gh3x2x_drv_control.h"
#include "gh3x2x_drv_control_ex.h"
#include "gh3x2x_drv_soft_adt.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>


#define debug_printf(...)
//extern void SlaverRttLog(const char * lpsbLog, ...);
extern void GH3X2X_AdtFuncStartWithConfirmHook(void);
extern void Gh3x2x_SoftAdtAlgorithmResultReport(STGh3x2xAlgoResult * pstAlgoResult, uint8_t uchDetectColor, uint32_t lubFrameId);

/**********佩戴检测全局变量**********/
int32_t glConfig[3] = { 0 };
int32_t ghbinfo[12] = { 0 };
int32_t glCapValue[2] = { 0 };
int32_t gnRawDataArr[15] = { 0 };
int32_t ghbrelsult[10];
bool gbFirst = NADTCAP_BYTE_TRUE;
uint8_t gbUnWear = NADTCAP_BYTE_FALSE;
int32_t g_lHbCount = 0;
uint8_t gbase_col = 0;
#define WATCH_NUM     (3)

#if (WATCH_NUM == 1)
int32_t lWearCap1 = 16400;
int32_t lUnwearCap1 = 12400;
int32_t lNoise1 = 300;
int32_t lWearCap2 = 0;
int32_t lUnwearCap2 = 0;
int32_t lNoise2 = 0;
#elif (WATCH_NUM == 2)
int32_t lWearCap1 = 20000;
int32_t lUnwearCap1 = 16000;
int32_t lNoise1 = 200;
int32_t lWearCap2 = 3700;
int32_t lUnwearCap2 = 1200;
int32_t lNoise2 = 200;
#elif (WATCH_NUM == 3)
int32_t lWearCap1 = 42500;
int32_t lUnwearCap1 = 39500;
int32_t lNoise1 = 150;
int32_t lWearCap2 = 42500;
int32_t lUnwearCap2 = 39500;
int32_t lNoise2 = 150;
#endif




/// movement status of g sensor
uint8_t g_uchGsensorStatus = GH3X2X_SENSOR_IS_NOT_MOVING;

/// sum of square of g sensor x,y,z value
uint32_t g_unGsDataSumOfSquare = 0;

/// sum of square threshold of g sensor data that can be judged as movement
uint32_t g_unGsMoveThreshold = 0;

/// count how many times of movement detected by g sensor
uint16_t g_usGsMoveDetectCnt = 0;

/// count how many times of not movement detected by g sensor
uint16_t g_usGsNotMoveDetectCnt = 0;

/// count threshold of continuous g sensor movement
uint16_t g_unGsMoveCntThreshold = 0;

/// count threshold of continuous g sensor not movement
uint16_t g_unGsNotMoveCntThreshold = 0;

/// enable gs movement detect or not
uint8_t g_uchAdtWithConfirmEnable = 0;

/// the first time of G sensor move detection
uint8_t g_uchGsMoveDetectFirstTime = 0;

uint32_t g_unNadtIrDefaultTimeOutSecondsThrd = 0;

typedef struct
{
    int16_t sXAxisVal;     /**< x-axis value */
    int16_t sYAxisVal;     /**< y-axis value */
    int16_t sZAxisVal;     /**< z-axis value */
} STGsAccRawdata;

STGsAccRawdata g_LastPointAdtGsenorRawdata;


uint32_t g_unLastAdtGsensorPeakPosi = 0;
uint32_t g_unLastLastAdtGsensorPeakPosi = 0;
uint32_t g_unCurrentAdtGsensorPeakPosi = 0;

uint8_t  g_uchSoftAdtChannl0Color;   // 0: green   1: ir


/// function started bitmap, use for sampling control
extern uint32_t g_unFuncStartedBitmap;
extern uint8_t  g_uchGh3x2xSleepFlag;
uint8_t g_uchDecByCapResult = 0;


#if 1


void GH3X2X_MoveDetectByCapData(STCapRawdata* pusCapData, uint16_t usCapDataCnt)
{
    if( 0 == GH3X2X_GetCapEnableFlag())
    {
        g_uchDecByCapResult = 1;
        return;
    }
}
#endif
/**
 * @fn      void GH3X2X_MoveDetectByGsData(STGsensorRawdata* stGsData, uint16_t usGsDataCnt, uint8_t uchCheckWindowSize)

 *
 * @brief  Move detection by software.
 *
 * @attention   None
 *
 * @param[in]   stGsData        gsensor data buffer
 * @param[in]   usGsDataCnt        gsensor data count
 * @param[in]   uchCheckWindowSize        the window size of cheak moving peak num
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_MoveDetectByGsData(int16_t* pusGsData, uint16_t usGsDataCnt)
{
    int16_t sXAxisVal = 0;
    int16_t sYAxisVal = 0;
    int16_t sZAxisVal = 0;
    uint8_t  uchGyroEnable = g_uchGyroEnable;
    uint32_t unSumOfSquare = 0;
    uint16_t  usGsDataIndex = 0;
    if ((GH3X2X_PTR_NULL == pusGsData) || (0 == usGsDataCnt))
    {
        return;
    }

    if (g_uchAdtWithConfirmEnable)
    {
        if(g_uchGsMoveDetectFirstTime)
        {
            g_LastPointAdtGsenorRawdata.sXAxisVal = pusGsData[0];
            g_LastPointAdtGsenorRawdata.sYAxisVal = pusGsData[0+1];
            g_LastPointAdtGsenorRawdata.sZAxisVal = pusGsData[0+2];
            g_uchGsMoveDetectFirstTime = 0;
        }
        //calculate average of x, y, z
        for (usGsDataIndex = 0; usGsDataIndex < usGsDataCnt; usGsDataIndex++)
        {
            sXAxisVal = pusGsData[usGsDataIndex*(3+3*uchGyroEnable)] - g_LastPointAdtGsenorRawdata.sXAxisVal;
            sYAxisVal = pusGsData[usGsDataIndex*(3+3*uchGyroEnable) + 1] - g_LastPointAdtGsenorRawdata.sYAxisVal;
            sZAxisVal = pusGsData[usGsDataIndex*(3+3*uchGyroEnable) + 2] - g_LastPointAdtGsenorRawdata.sZAxisVal;

            g_LastPointAdtGsenorRawdata.sXAxisVal = pusGsData[usGsDataIndex*(3+3*uchGyroEnable)];
            g_LastPointAdtGsenorRawdata.sYAxisVal = pusGsData[usGsDataIndex*(3+3*uchGyroEnable) + 1];
            g_LastPointAdtGsenorRawdata.sZAxisVal = pusGsData[usGsDataIndex*(3+3*uchGyroEnable) + 2];


            unSumOfSquare = (uint32_t)((int32_t)sXAxisVal * sXAxisVal) + (uint32_t)((int32_t)sYAxisVal * sYAxisVal) + (uint32_t)((int32_t)sZAxisVal * sZAxisVal);
            if(unSumOfSquare > g_unGsMoveThreshold)
            {
                g_usGsMoveDetectCnt ++;
                GH3X2X_DEBUG_LOG_PARAM("g_usGsMoveDetectCnt = %d \r\n", g_usGsMoveDetectCnt);
                g_usGsNotMoveDetectCnt = 0;
                if (g_usGsMoveDetectCnt > g_unGsMoveCntThreshold && g_uchDecByCapResult == 1)
                {
                    GH3X2X_DEBUG_LOG_PARAM("over g_unGsMoveCntThreshold!!! \r\n");
                    g_uchDecByCapResult = 0;
                    g_uchGsensorStatus = GH3X2X_SENSOR_IS_MOVING;
                    GH3X2X_DEBUG_LOG_PARAM("Move detect effect!!! \r\n");
                    if (GH3X2X_CHECK_BIT_NOT_SET(g_unFuncStartedBitmap, GH3X2X_FUNCTION_ADT))
                    {
                        GH3X2X_DEBUG_LOG_PARAM("start adt!!! \r\n");
                        GH3X2X_StartHardAdtAndResetGsDetect();
                    }
                }
            }
            else
            {
                if (g_usGsMoveDetectCnt != 0)
                {
                    g_usGsNotMoveDetectCnt ++;
                    GH3X2X_DEBUG_LOG_PARAM("g_usGsNotMoveDetectCnt = %d \r\n", g_usGsNotMoveDetectCnt);

                    if (g_usGsNotMoveDetectCnt >= g_unGsNotMoveCntThreshold)
                    {
                        GH3X2X_DEBUG_LOG_PARAM("over g_unGsNotMoveCntThreshold!!! \r\n");
                        g_usGsMoveDetectCnt = 0;
                        g_uchGsensorStatus = GH3X2X_SENSOR_IS_NOT_MOVING;
                        GH3X2X_DEBUG_LOG_PARAM("Move detect is not effect!!! \r\n");
                    }
                }
            }

            /*if(unSumOfSquare > g_unGsMoveThreshold)
            {
                if(0 == g_unGsMoveCntThreshold)
                {

                GH3X2X_DEBUG_LOG_PARAM("DIFF of peak = %d, abs of unSumOfSquare = %d \r\n",(g_unCurrentAdtGsensorPeakPosi - g_unCurrentAdtGsensorPeakPosi), unSumOfSquare);
                    break;
                }
                else if ((1 == g_unGsMoveCntThreshold)&&(0 != g_unLastAdtGsensorPeakPosi))
                {
                    if((g_unCurrentAdtGsensorPeakPosi - g_unLastAdtGsensorPeakPosi) <= (uint32_t)uchCheckWindowSize)
                    {
                        GH3X2X_DEBUG_LOG_PARAM("DIFF of peak = %d, abs of unSumOfSquare = %d \r\n",(g_unCurrentAdtGsensorPeakPosi - g_unLastAdtGsensorPeakPosi), unSumOfSquare);

                        break;
                    }
                }
                else if ((1 < g_unGsMoveCntThreshold)&&(0 != g_unLastLastAdtGsensorPeakPosi))
                {
                    if((g_unCurrentAdtGsensorPeakPosi - g_unLastLastAdtGsensorPeakPosi) <= (uint32_t)uchCheckWindowSize)
                    {

                    GH3X2X_DEBUG_LOG_PARAM("DIFF of peak = %d, abs of unSumOfSquare = %d \r\n",(g_unCurrentAdtGsensorPeakPosi - g_unLastLastAdtGsensorPeakPosi), unSumOfSquare);
                        break;
                    }
                }

                g_unLastLastAdtGsensorPeakPosi = g_unLastAdtGsensorPeakPosi;
                g_unLastAdtGsensorPeakPosi = g_unCurrentAdtGsensorPeakPosi;

            }*/

            //g_unCurrentAdtGsensorPeakPosi ++;

        }
        g_uchDecByCapResult = 0;

        //sXAxisVal = sXAxisVal / usGsDataCnt;
        //sYAxisVal = sYAxisVal / usGsDataCnt;
        //sZAxisVal = sZAxisVal / usGsDataCnt;

        //calculate square sum of x,y,z

        /*
        if (g_uchGsMoveDetectFirstTime)
        {
            g_uchGsMoveDetectFirstTime = 0;
            g_unGsDataSumOfSquare = unSumOfSquare;
            return;
        }
        else
        */
        //if (GH3X2X_VAL_ABS((int32_t)unSumOfSquare - (int32_t)g_unGsDataSumOfSquare) > g_unGsMoveThreshold)
    }
}

/**
 * @fn     void Gh3x2x_ResetMoveDetectByGsData(void)
 *
 * @brief  Reset movement detection by gsensor data
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  1: enabled, 0:disabled
 */

void Gh3x2x_ResetMoveDetectByGsData(void)
{
    g_uchGsensorStatus  = GH3X2X_SENSOR_IS_NOT_MOVING;
    g_uchGsMoveDetectFirstTime = 1;
    g_uchAdtWithConfirmEnable  = 1;

    g_usGsMoveDetectCnt = 0;
    g_usGsNotMoveDetectCnt = 0;

    //g_unLastAdtGsensorPeakPosi = 0;
    //g_unLastLastAdtGsensorPeakPosi = 0;
    //g_unCurrentAdtGsensorPeakPosi = 0;

    GH3X2X_DEBUG_LOG_PARAM("Gh3x2x_ResetMoveDetectByGsData g_uchAdtWithConfirmEnable = %x",g_uchAdtWithConfirmEnable);
}
/**
 * @fn     int8_t GH3X2X_AdtFuncStartWithConfirm(void)
 *
 * @brief  Start adt function with move detect confirm
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_AdtFuncStartWithConfirm(void)
{
    if (GH3X2X_GetSoftWearOffDetEn())
    {
        GH3X2X_DEBUG_LOG_PARAM("[%s]\r\n", __FUNCTION__);
        Gh3x2x_ResetMoveDetectByGsData();
        GH3X2X_AdtFuncStartWithConfirmHook();
        if (GH3X2X_CHECK_BIT_SET(g_unFuncStartedBitmap, GH3X2X_FUNCTION_ADT))
        {
            GH3X2X_StopHardAdtAndStartGsDetect();
        }
        GH3X2X_DEBUG_LOG_PARAM("GH3X2X_AdtFuncStartWithConfirm g_uchAdtWithConfirmEnable = %x",g_uchAdtWithConfirmEnable);
    }
}

/**
 * @fn     int8_t GH3X2X_AdtFuncStopWithConfirm(void)
 *
 * @brief  stop adt function with move detect confirm
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_AdtFuncStopWithConfirm(void)
{
    if (GH3X2X_GetSoftWearOffDetEn())
    {
        GH3X2X_DEBUG_LOG_PARAM("[%s]\r\n", __FUNCTION__);
        Gh3x2x_ResetMoveDetectByGsData();
        g_uchAdtWithConfirmEnable = 0;
    }
}

/**
 * @fn     uint8_t Gh3x2x_GetAdtConfirmStatus(void)
 *
 * @brief  Get status of movement detected
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  GH3X2X_SENSOR_IS_NOT_MOVING/GH3X2X_SENSOR_IS_MOVING
 */
uint8_t Gh3x2x_GetAdtConfirmStatus(void)
{
    return g_uchGsensorStatus;
}


/**
 * @fn     void Gh3x2x_SetAdtConfirmPara(uint8_t uchMoveThreshold, uint16_t usMoveCntThreshold)
 *
 * @brief  Set some parameter of adt confirm
 *
 * @attention   None
 *
 * @param[in]   usMoveThreshold            threshold of gsensor data(sqrt(sum of square)) that can be judged as movement
 * @param[in]   usMoveCntThreshold        how many times of continuous overtake usMoveThreshold that can trig sampling
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_SetAdtConfirmPara(uint8_t uchMoveThreshold, uint16_t usMoveCntThreshold, uint16_t usNotMoveCntThreshold, uint32_t unIRTimeoutSecondsThreshold)
{
    g_unGsMoveThreshold = uchMoveThreshold * uchMoveThreshold;
    g_unGsMoveCntThreshold = usMoveCntThreshold;
    g_unGsNotMoveCntThreshold = usNotMoveCntThreshold;
    g_unNadtIrDefaultTimeOutSecondsThrd = unIRTimeoutSecondsThreshold;
}


/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/



uint8_t g_SoftWearQuality = 0;

// config type
#define NADT_CONFIG_SOFT_AUTOLED_TYPE       (0)
#define NADT_CONFIG_TURNLIGH_TYPE           (1)
#define NADT_CONFIG_UNWEAR_THR_TYPE         (2)
#define NADT_CONFIG_NOSIE_TIMEOUT_TYPE      (3)
#define NADT_CONFIG_MULTI_SAMPLE_RATE_TYPE  (4)
#define NADT_CONFIG_SLEEP_TYPE              (5)

#define NADT_CONFIG_ADT_UNWEAR_THR_TYPE            (2)
#define NADT_CONFIG_SAMPLE_RATE_TYPE               (4)
#define NADT_CONFIG_CAP1_INFO_TYPE                 (110)
#define NADT_CONFIG_CAP2_INFO_TYPE                 (111)
#define NADT_CONFIG_CAP_TRIG_CNT_TYPE              (112)
#define NADT_CONFIG_CAP_CONFIRM_CNT_TYPE           (113)
#define NADT_CONFIG_ADT_CONFIRM_CNT_TYPE           (114)
#define NADT_CONFIG_GSENSOR_THR_TYPE               (115)
#define NADT_CONFIG_GSENSOR_CONFIRM_CNT_TYPE       (116)
#define NADT_CONFIG_WEAR_CAP_UPDATE_TIME_TYPE      (117)
#define NADT_CONFIG_UNWEAR_CAP_UPDATE_TIME_TYPE    (118)

#define NADTCAP_BYTE_TRUE       (uint8_t)1
#define NADTCAP_BYTE_FALSE      (uint8_t)0

EMWearRecordType g_emSoftWearOffStatus = STATUS_DEFAULT;
uint32_t g_unNadtIrDefaultTimeOutRecord = 0;

uint8_t GH3X2X_GetSoftWearOffDetEn(void)
{
    return g_stAdtModuleCfg.stAdtCfgByte0.uchSoftWearOffDetEn;
}


void GH3x2xCapNadtResultPro(const struct gh3020_frameinfo_s * const pstFrameInfo,int32_t  pnSoftWearOffDetResult[])
{
    if (pnSoftWearOffDetResult[3])            // 电容更新标记,更新佩戴检测阈值
    {
        pstFrameInfo->pstAlgoResult->uchUpdateFlag = 1;
        lUnwearCap1 = pnSoftWearOffDetResult[4];
        lWearCap1 = pnSoftWearOffDetResult[5];
        lUnwearCap2 = pnSoftWearOffDetResult[6];
        lWearCap2 = pnSoftWearOffDetResult[7];
    }
    if(g_SoftWearQuality != pnSoftWearOffDetResult[2])
    {
        pstFrameInfo->pstAlgoResult->uchUpdateFlag = 1;
        g_SoftWearQuality = pnSoftWearOffDetResult[2];
    }
    if(pstFrameInfo->punFrameCnt == 0)
    {
        pstFrameInfo->pstAlgoResult->uchUpdateFlag = 1;
    }
    if(pstFrameInfo->pstAlgoResult->uchUpdateFlag == 1)
    {
        pstFrameInfo->pstAlgoResult->uchResultNum  = 7;
        pstFrameInfo->pstAlgoResult->snResult[0] = pnSoftWearOffDetResult[0]; // (bit0-bit1)：佩戴状态（0默认，1佩戴，2脱落，3非活体）； (bit2)：疑似脱落标记（0正常，1疑似脱落）；
        pstFrameInfo->pstAlgoResult->snResult[1] = pnSoftWearOffDetResult[1]; //活体检测置信度
        pstFrameInfo->pstAlgoResult->snResult[2] = pnSoftWearOffDetResult[2]; //佩戴质量（1紧佩戴，2松佩戴，3极度松佩戴，4脱落）；
        if(pstFrameInfo->punFrameCnt == 0)
        {
            pstFrameInfo->pstAlgoResult->snResult[3] = lUnwearCap1; //脱落电容值1；
            pstFrameInfo->pstAlgoResult->snResult[4] = lWearCap1; //佩戴电容值1；
            pstFrameInfo->pstAlgoResult->snResult[5] = lUnwearCap2; //脱落电容值2；
            pstFrameInfo->pstAlgoResult->snResult[6] = lWearCap2; //佩戴电容值2；
        }
        else
        {
            pstFrameInfo->pstAlgoResult->snResult[3] = pnSoftWearOffDetResult[4]; //脱落电容值1；
            pstFrameInfo->pstAlgoResult->snResult[4] = pnSoftWearOffDetResult[5]; //佩戴电容值1；
            pstFrameInfo->pstAlgoResult->snResult[5] = pnSoftWearOffDetResult[6]; //脱落电容值2；
            pstFrameInfo->pstAlgoResult->snResult[6] = pnSoftWearOffDetResult[7]; //佩戴电容值2；
        }

    }

}




uint8_t Gh3x2xCheckSoftAdtTimeOut(const struct gh3020_frameinfo_s * const pstFrameInfo, uint32_t unSpecialAngleTimeSec, uint32_t unMovelessTimeSec)
{
    return 0;
}






void GH3X2X_SetSoftWearStatus(EMWearRecordType emStatus)
{
    if(GH3X2X_GetSoftWearOffDetEn())
    {
        g_emSoftWearOffStatus = emStatus;
    }
}

uint8_t GH3X2X_UpdateSoftWearStatus(void)
{
    if(GH3X2X_GetSoftWearOffDetEn())
    {
        if(STATUS_LVING_UNWEAR == g_emSoftWearOffStatus)
        {
            GH3X2X_DEBUG_LOG_PARAM("[%s]:Soft wear off detect: wear off !!!\r\n", __FUNCTION__);
            g_emSoftWearOffStatus = STATUS_UNWEAR_REPORTED;
            return STATUS_LVING_UNWEAR;
        }
        else if(STATUS_LVING_WEAR == g_emSoftWearOffStatus)
        {
            GH3X2X_DEBUG_LOG_PARAM("[%s]:Soft wear off detect: wear on !!!\r\n", __FUNCTION__);
            g_emSoftWearOffStatus = STATUS_WEAR_REPORTED;
            return STATUS_LVING_WEAR;
        }
    }
    return STATUS_DEFAULT;
}

extern  void hal_gh3x2x_write_cap_to_flash(int32_t WearCap1,int32_t UnwearCap1,int32_t WearCap2,int32_t UnwearCap2);
extern  void hal_gh3x2x_read_cap_from_flash(int32_t* WearCap1,int32_t* UnwearCap1,int32_t* WearCap2,int32_t* UnwearCap2);

void GH3x2xWriteCapToFlash(void)
{
    hal_gh3x2x_write_cap_to_flash(lWearCap1,lUnwearCap1,lWearCap2,lUnwearCap2);
}

void GH3x2xReadCapFromFlash(void)
{
    hal_gh3x2x_read_cap_from_flash(&lWearCap1,&lUnwearCap1,&lWearCap2,&lUnwearCap2);
}



