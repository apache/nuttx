/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_grv_zip.c
 *
 * @brief   gh3x2x extension control functions, using function map like evk
 *
 * @version ref gh3x2x_grv_ziphandle.h
 *
 */

#include <stdio.h>
#include "gh3x2x_drv_version.h"
#include "gh3x2x_drv_common.h"
#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"
#include "gh3x2x_drv_uprotocol.h"
#include "gh3x2x_drv_control.h"
#include "gh3x2x_drv_dump.h"
#include "gh3x2x_drv_soft_led_agc.h"
#include "gh3x2x_drv_soft_adt.h"
#include "gh3x2x_drv_zip.h"

// Zipmode Tempdata
STZipLastDataStruct g_stZipLastData;

uint32_t g_unLastFrameFlag2 = 0;

//  Zip Package Num Even/Odd Flag
extern uint8_t g_uchOddEvenChangeFlag;
extern uint16_t g_pusDrvCurrentRecord[];
extern uint8_t g_uchPacketMaxLenSupport;
extern uint16_t g_usGh3x2xDataIndexInBuf;
extern uint16_t g_usRawdataByteIndexArr[GH3X2X_CHANNEL_MAP_MAX_CH];
extern float g_fGsDataIndexInBuf;
extern uint16_t g_usAlgoResIndex;
extern uint32_t g_unLastIncompleteMark;
extern uint8_t g_uchFifoPackageID;
extern uint8_t g_uchFifoPackageMode;
extern void Gh2x2xPackPakcageHeader(uint8_t *puchNeedContinue, uint8_t uchFunctionIDForMaster, uint8_t *puchUploadBuf, uint8_t *puchTagArray,
                                            uint8_t uchGsEnable, uint8_t uchAlgoResFlag, uint8_t uchAgcEnable, uint8_t uchAmbEnable, uint8_t uchGsGyroEnable,uint8_t uchCapEnable,uint8_t uchTempEnable,
                                            uint8_t uchZipEnableFlag, uint8_t uchOddEvenChangeFlag);
extern void GH3x2xSendFunctionInfo(const struct gh3020_frameinfo_s * const pstFrameInfo);

/**
 * @fn     void GH3X2X_GetRawDataDiff(uint32_t *punRawData, uint8_t *puchRawdataTagTempArr,
                                       uint8_t uchChannelMapCnt, uint8_t *puchZipDataArr, uint8_t *puchZipDataArrSize)
 *
 * @brief  Pack and Generate Zip RawData Data.
 *
 * @attention   None
 *
 * @param[out]  puchZipDataArr          Zip Paclage Data.
 * @param[out]  puchZipDataArrSize      Zip Package Size.
 * @param[in]   punRawData              Current Gsensor Data.
 * @param[in]   puchRawdataTagTempArr   Current Gsensor Data.
 * @param[in]   uchChannelMapCnt        Current Gsensor Data.

 * @return  None
 */
void GH3X2X_GetRawDataDiff(uint32_t *punRawData, uint8_t *puchRawdataTagTempArr,
                            uint8_t uchChannelMapCnt, uint8_t *puchZipArr, uint8_t *puchZipDataArrSize,
                            uint8_t *uchFifoLastRawdataTagArr, uint32_t *unFifoLastRawdataArr)
{
    uint8_t uchDiffunittemp;
    int8_t chNum;
    uint8_t uchChnum;
    uint32_t unChnum;
    uint8_t uchDifftemp;
    uint8_t uchDiffarrcnt8bit = 0;
    uint8_t uchDiffarrcnt4bit = 0;
    uint8_t uchDataType = 0;

    uint8_t puchZipDataArr[255];
    GH3X2X_Memset(puchZipDataArr, 0, 255);

    if (puchRawdataTagTempArr[0] != uchFifoLastRawdataTagArr[0])
    {
        uchDiffarrcnt8bit++;
        puchZipDataArr[uchDiffarrcnt8bit] = RAWDATA_DIFF_ODD;
        GH3X2X_Memcpy(&puchZipDataArr[uchDiffarrcnt8bit+RAWDATA_DIFF_ODD], puchRawdataTagTempArr, uchChannelMapCnt);
        uchDiffarrcnt8bit += uchChannelMapCnt;
        uchDiffarrcnt4bit += (uint8_t)(uchChannelMapCnt*RAWDATA_DIFF_EVEN);
    }
    else
    {
        uchDiffarrcnt8bit++;
        puchZipDataArr[uchDiffarrcnt8bit] = 0;
    }

    for (uchChnum = 0 ; uchChnum < uchChannelMapCnt ; uchChnum++)
    {
        if (punRawData[uchChnum] >= unFifoLastRawdataArr[uchChnum])
        {
            unChnum = punRawData[uchChnum] - unFifoLastRawdataArr[uchChnum];
        }
        else
        {
            unChnum = unFifoLastRawdataArr[uchChnum] - punRawData[uchChnum];
        }

        for (chNum = RAWDATA_DIFF_SIZE ; chNum >= 0 ; chNum--)
        {
            if (unChnum == 0)
            {
                uchDataType = 0;
                break;
            }
            uchDifftemp = ((uint8_t)(GH3X2X_GET_FIRST_4BITS((unChnum) >> (chNum*RAWDATA_DIFF_BYTE_SIZE))));
            if (uchDifftemp != 0)
            {
                uchDataType = chNum * RAWDATA_DIFF_EVEN + GH3X2X_VAL_DIFF_STA(punRawData[uchChnum], \
                    unFifoLastRawdataArr[uchChnum]);
                break;
            }
        }

        if (uchDiffarrcnt4bit%RAWDATA_DIFF_EVEN)
        {
            puchZipDataArr[uchDiffarrcnt8bit] |= uchDataType;
        }

        else
        {
            uchDiffarrcnt8bit++;
            puchZipDataArr[uchDiffarrcnt8bit] |= (uint8_t)(uchDataType << RAWDATA_DIFF_BYTE_SIZE);
        }
        uchDiffarrcnt4bit++;

        for (chNum = (uint8_t)(uchDataType/RAWDATA_DIFF_EVEN) ; chNum >= 0 ; chNum--)
        {
            uchDiffunittemp = (uint8_t)(GH3X2X_GET_FIRST_4BITS((unChnum) >> (chNum*RAWDATA_DIFF_BYTE_SIZE)));
            if (uchDiffarrcnt4bit%RAWDATA_DIFF_EVEN)
            {
                puchZipDataArr[uchDiffarrcnt8bit] |= uchDiffunittemp;
            }
            else
            {
                uchDiffarrcnt8bit++;
                puchZipDataArr[uchDiffarrcnt8bit] |= (uint8_t)(uchDiffunittemp << RAWDATA_DIFF_BYTE_SIZE);
            }
            uchDiffarrcnt4bit++;
        }
    }// end of (for uchChnum=0;uchChnum<uchChannelMapCnt;uchChnum++)

    uchDiffarrcnt8bit = (uchDiffarrcnt4bit + RAWDATA_DIFF_ODD) / RAWDATA_DIFF_EVEN + RAWDATA_DIFF_ODD;
    puchZipDataArr[0] = uchDiffarrcnt8bit;
    *puchZipDataArrSize = (uint8_t)(uchDiffarrcnt8bit + RAWDATA_DIFF_ODD);
    GH3X2X_Memcpy(puchZipArr, puchZipDataArr, uchDiffarrcnt8bit + 1);
}

/**
 * @fn     void GH3X2X_GetDataDiff(uint32_t *punData, uint8_t uchChannelMapCnt, uint8_t *puchZipDataArr,
                            uint8_t *puchZipDataArrSize, uint32_t *punZipTempData)
 *
 * @brief  Pack and Generate Zip RawData Data.
 *
 * @attention   None
 *
 * @param[out]  puchZipDataArr          Zip Paclage Data.
 * @param[out]  puchZipDataArrSize      Zip Package Size.
 * @param[in]   punData              Current Gsensor Data.
 * @param[in]   punZipTempData   Current Gsensor Data.
 * @param[in]   uchChannelMapCnt        Current Gsensor Data.

 * @return  None
 */
void GH3X2X_GetDataDiff(uint32_t *punData, uint8_t uchChannelMapCnt, uint8_t *puchZipDataArr,
                        uint8_t *puchZipDataArrSize, uint32_t *punZipTempData)
{
    uint8_t uchDiffunittemp;
    int8_t chNum;
    uint8_t uchChnum;
    uint32_t unChnum;
    uint8_t uchDifftemp;
    uint8_t uchDiffarrcnt8bit = 0;
    uint8_t uchDiffarrcnt4bit = 0;
    uint8_t uchDataType = 0;

    //uint8_t puchZipDataArr[255];
    //GH3X2X_Memset(puchZipDataArr, 0, 255);

    for (uchChnum = 0 ; uchChnum < uchChannelMapCnt ; uchChnum++)
    {
        if (punData[uchChnum] >= punZipTempData[uchChnum])
        {
            unChnum = punData[uchChnum] - punZipTempData[uchChnum];
        }
        else
        {
            unChnum = punZipTempData[uchChnum] - punData[uchChnum];
        }

        for (chNum = RAWDATA_DIFF_SIZE ; chNum >= 0 ; chNum--)
        {
            if (unChnum == 0)
            {
                uchDataType = 0;
                break;
            }
            uchDifftemp = ((uint8_t)(GH3X2X_GET_FIRST_4BITS((unChnum) >> (chNum*RAWDATA_DIFF_BYTE_SIZE))));
            if (uchDifftemp != 0)
            {
                uchDataType = chNum * RAWDATA_DIFF_EVEN + GH3X2X_VAL_DIFF_STA(punData[uchChnum], \
                    punZipTempData[uchChnum]);
                break;
            }
        }

        if (uchDiffarrcnt4bit%RAWDATA_DIFF_EVEN)
        {
            puchZipDataArr[uchDiffarrcnt8bit] |= uchDataType;
        }

        else
        {
            uchDiffarrcnt8bit++;
            puchZipDataArr[uchDiffarrcnt8bit] |= (uint8_t)(uchDataType << RAWDATA_DIFF_BYTE_SIZE);
        }
        uchDiffarrcnt4bit++;

        for (chNum = (int8_t)(uchDataType/RAWDATA_DIFF_EVEN) ; chNum >= 0 ; chNum--)
        {
            uchDiffunittemp = (uint8_t)(GH3X2X_GET_FIRST_4BITS((unChnum) >> (chNum*RAWDATA_DIFF_BYTE_SIZE)));
            if (uchDiffarrcnt4bit%RAWDATA_DIFF_EVEN)
            {
                puchZipDataArr[uchDiffarrcnt8bit] |= uchDiffunittemp;
            }
            else
            {
                uchDiffarrcnt8bit++;
                puchZipDataArr[uchDiffarrcnt8bit] |= (uint8_t)(uchDiffunittemp << RAWDATA_DIFF_BYTE_SIZE);
            }
            uchDiffarrcnt4bit++;
        }
    }// end of (for uchChnum=0;uchChnum<uchChannelMapCnt;uchChnum++)

    uchDiffarrcnt8bit = (uchDiffarrcnt4bit + RAWDATA_DIFF_ODD) / RAWDATA_DIFF_EVEN + RAWDATA_DIFF_ODD;
    puchZipDataArr[0] = uchDiffarrcnt8bit;
    *puchZipDataArrSize = (uint8_t)(uchDiffarrcnt8bit + RAWDATA_DIFF_ODD);
//    GH3X2X_Memcpy(puchZipArr, puchZipDataArr, uchDiffarrcnt8bit + 1);
}

void GH3X2X_ZipFillLastRawdataBuffer(const struct gh3020_frameinfo_s * const pstFrameInfo, uint32_t unFunctionID,
                                            uint8_t uchChnlNum, uint8_t *uchFifoLastRawdataTagArr, uint32_t *unFifoLastRawdataArr)
{
    for(uint8_t uchChnlCnt = 0; uchChnlCnt < uchChnlNum; uchChnlCnt ++)
    {
        //led adj flag 0
        if(pstFrameInfo->punFrameFlag[0]&((((uint32_t)1)) << uchChnlCnt))
        {
            pstFrameInfo->punFrameRawdata[uchChnlCnt] |= ((((uint32_t)1)) << 26);
        }
        // fast recovery
        if ((GH3X2X_FUNCTION_ECG == unFunctionID)||(GH3X2X_FUNCTION_PWTT == unFunctionID))
        {
            if(0 == uchChnlCnt)
            {
                if(pstFrameInfo->punFrameFlag[2]&((((uint32_t)1)) << 0))
                {
                    pstFrameInfo->punFrameRawdata[uchChnlCnt] |= ((((uint32_t)1)) << 26);
                }
            }
        }
        //led adj flag 1
        if(pstFrameInfo->punFrameFlag[1]&((((uint32_t)1)) << uchChnlCnt))
        {
            pstFrameInfo->punFrameRawdata[uchChnlCnt] |= ((((uint32_t)1)) << 25);
        }

        uchFifoLastRawdataTagArr[uchChnlCnt] = (pstFrameInfo->punFrameRawdata[uchChnlCnt] >> 24)&0x000000FF;
        unFifoLastRawdataArr[uchChnlCnt] = pstFrameInfo->punFrameRawdata[uchChnlCnt]&0x00FFFFFF;

        pstFrameInfo->punFrameRawdata[uchChnlCnt] &= 0x00FFFFFF;
    }
}

void GH3X2X_ZipFillLastDataBuffer(uint32_t *unCurrentData, uint8_t uchChnlNum, uint32_t *unLastdata)
{
    for(uint8_t uchChnlCnt = 0; uchChnlCnt < uchChnlNum; uchChnlCnt ++)
    {
        unLastdata[uchChnlCnt] = unCurrentData[uchChnlCnt];
    }
}


extern uint8_t g_uchGh3x2xUploadStatus;
extern uint8_t g_uchGh3x2xUploadBufUse;
uint8_t g_uchCurrentFunctionID = 0;

