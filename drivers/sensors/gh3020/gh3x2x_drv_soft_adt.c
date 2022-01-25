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
#include "gh3x2x_drv_interface.h"
#include "gh3x2x_drv_control.h"
#include "gh3x2x_drv_uprotocol.h"
#include "gh3x2x_drv_control_ex.h"
#include "gh3x2x_drv_soft_adt.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>


#define debug_printf(...) 
//extern void SlaverRttLog(const char * lpsbLog, ...);
extern void GH3X2X_AdtFuncStartWithConfirmHook(void);
extern void Gh3x2x_SoftAdtAlgorithmResultReport(STGh3x2xAlgoResult * pstAlgoResult, GU8 uchDetectColor, GU32 lubFrameId);

/**********佩戴检测全局变量**********/
GS32 glConfig[3] = { 0 };
GS32 ghbinfo[12] = { 0 };
GS32 glCapValue[2] = { 0 };
GS32 gnRawDataArr[15] = { 0 };
GS32 ghbrelsult[10];
GBOOL gbFirst = NADTCAP_BYTE_TRUE;
GU8 gbUnWear = NADTCAP_BYTE_FALSE;
GS32 g_lHbCount = 0;
GU8 gbase_col = 0;
#define WATCH_NUM     (3)

#if (WATCH_NUM == 1)
GS32 lWearCap1 = 16400;
GS32 lUnwearCap1 = 12400;
GS32 lNoise1 = 300;
GS32 lWearCap2 = 0;
GS32 lUnwearCap2 = 0;
GS32 lNoise2 = 0;
#elif (WATCH_NUM == 2)
GS32 lWearCap1 = 20000;
GS32 lUnwearCap1 = 16000;
GS32 lNoise1 = 200;
GS32 lWearCap2 = 3700;
GS32 lUnwearCap2 = 1200;
GS32 lNoise2 = 200;
#elif (WATCH_NUM == 3)
GS32 lWearCap1 = 42500;
GS32 lUnwearCap1 = 39500;
GS32 lNoise1 = 150;
GS32 lWearCap2 = 42500;
GS32 lUnwearCap2 = 39500;
GS32 lNoise2 = 150;
#endif




/// movement status of g sensor
GU8 g_uchGsensorStatus = GH3X2X_SENSOR_IS_NOT_MOVING;

/// sum of square of g sensor x,y,z value
GU32 g_unGsDataSumOfSquare = 0;

/// sum of square threshold of g sensor data that can be judged as movement
GU32 g_unGsMoveThreshold = 0;

/// count how many times of movement detected by g sensor 
GU16 g_usGsMoveDetectCnt = 0;

/// count how many times of not movement detected by g sensor 
GU16 g_usGsNotMoveDetectCnt = 0;

/// count threshold of continuous g sensor movement
GU16 g_unGsMoveCntThreshold = 0;

/// count threshold of continuous g sensor not movement
GU16 g_unGsNotMoveCntThreshold = 0;

/// enable gs movement detect or not
GU8 g_uchAdtWithConfirmEnable = 0;

/// the first time of G sensor move detection
GU8 g_uchGsMoveDetectFirstTime = 0;

GU32 g_unNadtIrDefaultTimeOutSecondsThrd = 0;

typedef struct
{
    GS16 sXAxisVal;     /**< x-axis value */
    GS16 sYAxisVal;     /**< y-axis value */
    GS16 sZAxisVal;     /**< z-axis value */
} STGsAccRawdata;

STGsAccRawdata g_LastPointAdtGsenorRawdata;


GU32 g_unLastAdtGsensorPeakPosi = 0;
GU32 g_unLastLastAdtGsensorPeakPosi = 0;
GU32 g_unCurrentAdtGsensorPeakPosi = 0;

GU8  g_uchSoftAdtChannl0Color;   // 0: green   1: ir


/// function started bitmap, use for sampling control
extern GU32 g_unFuncStartedBitmap;
extern GU8  g_uchGh3x2xSleepFlag;
GU8 g_uchDecByCapResult = 0;


#if 1


void GH3X2X_MoveDetectByCapData(STCapRawdata* pusCapData, GU16 usCapDataCnt)
{
    if( 0 == GH3X2X_GetCapEnableFlag())
    {
        g_uchDecByCapResult = 1;
        return;
    }
}
#endif
/**
 * @fn      void GH3X2X_MoveDetectByGsData(STGsensorRawdata* stGsData, GU16 usGsDataCnt, GU8 uchCheckWindowSize)
 
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
void GH3X2X_MoveDetectByGsData(GS16* pusGsData, GU16 usGsDataCnt)
{
    GS16 sXAxisVal = 0;
    GS16 sYAxisVal = 0;
    GS16 sZAxisVal = 0;
    GU8  uchGyroEnable = g_uchGyroEnable;
    GU32 unSumOfSquare = 0;    
    GU16  usGsDataIndex = 0;
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


            unSumOfSquare = (GU32)((GS32)sXAxisVal * sXAxisVal) + (GU32)((GS32)sYAxisVal * sYAxisVal) + (GU32)((GS32)sZAxisVal * sZAxisVal);
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
                    if((g_unCurrentAdtGsensorPeakPosi - g_unLastAdtGsensorPeakPosi) <= (GU32)uchCheckWindowSize)
                    {
                        GH3X2X_DEBUG_LOG_PARAM("DIFF of peak = %d, abs of unSumOfSquare = %d \r\n",(g_unCurrentAdtGsensorPeakPosi - g_unLastAdtGsensorPeakPosi), unSumOfSquare);

                        break;
                    }
                }
                else if ((1 < g_unGsMoveCntThreshold)&&(0 != g_unLastLastAdtGsensorPeakPosi))
                {
                    if((g_unCurrentAdtGsensorPeakPosi - g_unLastLastAdtGsensorPeakPosi) <= (GU32)uchCheckWindowSize)
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
        //if (GH3X2X_VAL_ABS((GS32)unSumOfSquare - (GS32)g_unGsDataSumOfSquare) > g_unGsMoveThreshold)
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
 * @fn     GS8 GH3X2X_AdtFuncStartWithConfirm(void)
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
 * @fn     GS8 GH3X2X_AdtFuncStopWithConfirm(void)
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
 * @fn     GU8 Gh3x2x_GetAdtConfirmStatus(void)
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
GU8 Gh3x2x_GetAdtConfirmStatus(void)
{
    return g_uchGsensorStatus;
}


/**
 * @fn     void Gh3x2x_SetAdtConfirmPara(GU8 uchMoveThreshold, GU16 usMoveCntThreshold)
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
void Gh3x2x_SetAdtConfirmPara(GU8 uchMoveThreshold, GU16 usMoveCntThreshold, GU16 usNotMoveCntThreshold, GU32 unIRTimeoutSecondsThreshold)
{
    g_unGsMoveThreshold = uchMoveThreshold * uchMoveThreshold;
    g_unGsMoveCntThreshold = usMoveCntThreshold;
    g_unGsNotMoveCntThreshold = usNotMoveCntThreshold;
    g_unNadtIrDefaultTimeOutSecondsThrd = unIRTimeoutSecondsThreshold;
}


/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/

 

GU8 g_SoftWearQuality = 0;

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

#define NADTCAP_BYTE_TRUE       (GU8)1
#define NADTCAP_BYTE_FALSE      (GU8)0

EMWearRecordType g_emSoftWearOffStatus = STATUS_DEFAULT;
GU32 g_unNadtIrDefaultTimeOutRecord = 0;

GU8 GH3X2X_GetSoftWearOffDetEn(void)
{
    return g_stAdtModuleCfg.stAdtCfgByte0.uchSoftWearOffDetEn;
}


void GH3x2xCapNadtResultPro(const STGh3x2xFrameInfo * const pstFrameInfo,GS32  pnSoftWearOffDetResult[])
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




GU8 Gh3x2xCheckSoftAdtTimeOut(const STGh3x2xFrameInfo * const pstFrameInfo, GU32 unSpecialAngleTimeSec, GU32 unMovelessTimeSec)
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

GU8 GH3X2X_UpdateSoftWearStatus(void)
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

extern  void hal_gh3x2x_write_cap_to_flash(GS32 WearCap1,GS32 UnwearCap1,GS32 WearCap2,GS32 UnwearCap2);
extern  void hal_gh3x2x_read_cap_from_flash(GS32* WearCap1,GS32* UnwearCap1,GS32* WearCap2,GS32* UnwearCap2);

void GH3x2xWriteCapToFlash(void)
{
    hal_gh3x2x_write_cap_to_flash(lWearCap1,lUnwearCap1,lWearCap2,lUnwearCap2);
}

void GH3x2xReadCapFromFlash(void)
{
    hal_gh3x2x_read_cap_from_flash(&lWearCap1,&lUnwearCap1,&lWearCap2,&lUnwearCap2);
}



