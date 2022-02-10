/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_demo_algorithm.c
 *
 * @brief   gh3x2x driver lib demo code for algorithm
 *
 * @author  Gooidx Iot Team
 *
 */

#include "gh3x2x_inner.h"

/**
 * @fn     void Gh3x2xDemoAlgorithmCalculate(uint8_t* puchReadFifoBuffer, STGsensorRawdata *pstGsAxisValueArr,
 *                                              uint16_t usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *
 * @brief  Algorithm calculate.This function will be called in Gh3x2xDemoInterruptProcess()
 *
 * @attention   None
 *
 * @param[in]   puchReadFifoBuffer      point to gh3x2x fifo data buffer
 * @param[in]   pstGsAxisValueArr       point to gsensor data buffer
 * @param[in]   usGsDataNum             gsensor data cnt
 * @param[in]   emGsSensitivity         sensitivity of gsensor
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_DemoAlgorithmCalculate(uint8_t* puchReadFifoBuffer, uint16_t usFifoBuffLen,STGsensorRawdata *pstGsAxisValueArr, uint16_t usGsDataNum,
                                   STCapRawdata* pstCapValueArr,uint16_t usCapDataNum,STTempRawdata* pstTempValueArr,uint16_t usTempDataNum)
{
    for(uint8_t uchFunCnt = 0; uchFunCnt < GH3X2X_FUNC_OFFSET_MAX; uchFunCnt ++)
    {
        if(g_pstGh3x2xFrameInfo[uchFunCnt])
        {
            if(g_unDemoFuncMode & (((uint32_t)1)<< uchFunCnt))
            {
                GH3x2xFunctionProcess(puchReadFifoBuffer,usFifoBuffLen,(int16_t*)pstGsAxisValueArr,usGsDataNum,
                                        pstCapValueArr,usCapDataNum,pstTempValueArr,usTempDataNum,
                                        g_pstGh3x2xFrameInfo[uchFunCnt]);
            }
        }
    }
}
