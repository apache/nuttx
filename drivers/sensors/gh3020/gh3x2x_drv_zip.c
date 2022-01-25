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
#include "gh3x2x_drv_uprotocol.h"
#include "gh3x2x_drv_interface.h"
#include "gh3x2x_drv_control.h"
#include"gh3x2x_drv_dump.h"
#include "gh3x2x_drv_soft_led_agc.h"
#include "gh3x2x_drv_soft_adt.h"
#include "gh3x2x_drv_zip.h"

// Zipmode Tempdata
STZipLastDataStruct g_stZipLastData;

GU32 g_unLastFrameFlag2 = 0;

//  Zip Package Num Even/Odd Flag
extern GU8 g_uchOddEvenChangeFlag;
extern GU8  g_puchGainBgCancelRecord[];
extern GU16 g_pusDrvCurrentRecord[];
extern GU8 g_uchPacketMaxLenSupport;
extern GU16 g_usGh3x2xDataIndexInBuf;
extern GU16 g_usRawdataByteIndexArr[GH3X2X_CHANNEL_MAP_MAX_CH];
extern GF32 g_fGsDataIndexInBuf;
extern GU16 g_usAlgoResIndex;
extern GU32 g_unLastIncompleteMark;
extern GU8 g_uchFifoPackageID;
extern GU8 g_uchFifoPackageMode;
extern void Gh2x2xPackPakcageHeader(GU8 *puchNeedContinue, GU8 uchFunctionIDForMaster, GU8 *puchUploadBuf, GU8 *puchTagArray,
                                            GU8 uchGsEnable, GU8 uchAlgoResFlag, GU8 uchAgcEnable, GU8 uchAmbEnable, GU8 uchGsGyroEnable,GU8 uchCapEnable,GU8 uchTempEnable,
                                            GU8 uchZipEnableFlag, GU8 uchOddEvenChangeFlag);
extern void GH3x2xSendFunctionInfo(const STGh3x2xFrameInfo * const pstFrameInfo);

/**
 * @fn     void GH3X2X_GetRawDataDiff(GU32 *punRawData, GU8 *puchRawdataTagTempArr, 
                                       GU8 uchChannelMapCnt, GU8 *puchZipDataArr, GU8 *puchZipDataArrSize)
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
void GH3X2X_GetRawDataDiff(GU32 *punRawData, GU8 *puchRawdataTagTempArr, 
                            GU8 uchChannelMapCnt, GU8 *puchZipArr, GU8 *puchZipDataArrSize, 
                            GU8 *uchFifoLastRawdataTagArr, GU32 *unFifoLastRawdataArr)
{
    GU8 uchDiffunittemp;
    GS8 chNum;
    GU8 uchChnum;
    GU32 unChnum;
    GU8 uchDifftemp;
    GU8 uchDiffarrcnt8bit = 0;
    GU8 uchDiffarrcnt4bit = 0;
    GU8 uchDataType = 0;

    GU8 puchZipDataArr[255];
    GH3X2X_Memset(puchZipDataArr, 0, 255);

    if (puchRawdataTagTempArr[0] != uchFifoLastRawdataTagArr[0])
    {
        uchDiffarrcnt8bit++;
        puchZipDataArr[uchDiffarrcnt8bit] = RAWDATA_DIFF_ODD;
        GH3X2X_Memcpy(&puchZipDataArr[uchDiffarrcnt8bit+RAWDATA_DIFF_ODD], puchRawdataTagTempArr, uchChannelMapCnt);
        uchDiffarrcnt8bit += uchChannelMapCnt;
        uchDiffarrcnt4bit += (GU8)(uchChannelMapCnt*RAWDATA_DIFF_EVEN);
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
            uchDifftemp = ((GU8)(GH3X2X_GET_FIRST_4BITS((unChnum) >> (chNum*RAWDATA_DIFF_BYTE_SIZE))));
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
            puchZipDataArr[uchDiffarrcnt8bit] |= (GU8)(uchDataType << RAWDATA_DIFF_BYTE_SIZE);
        }
        uchDiffarrcnt4bit++;
        
        for (chNum = (GU8)(uchDataType/RAWDATA_DIFF_EVEN) ; chNum >= 0 ; chNum--)
        {
            uchDiffunittemp = (GU8)(GH3X2X_GET_FIRST_4BITS((unChnum) >> (chNum*RAWDATA_DIFF_BYTE_SIZE)));
            if (uchDiffarrcnt4bit%RAWDATA_DIFF_EVEN)
            {
                puchZipDataArr[uchDiffarrcnt8bit] |= uchDiffunittemp;
            }
            else
            {
                uchDiffarrcnt8bit++;
                puchZipDataArr[uchDiffarrcnt8bit] |= (GU8)(uchDiffunittemp << RAWDATA_DIFF_BYTE_SIZE);
            }
            uchDiffarrcnt4bit++;
        }
    }// end of (for uchChnum=0;uchChnum<uchChannelMapCnt;uchChnum++)
    
    uchDiffarrcnt8bit = (uchDiffarrcnt4bit + RAWDATA_DIFF_ODD) / RAWDATA_DIFF_EVEN + RAWDATA_DIFF_ODD;
    puchZipDataArr[0] = uchDiffarrcnt8bit;
    *puchZipDataArrSize = (GU8)(uchDiffarrcnt8bit + RAWDATA_DIFF_ODD);
    GH3X2X_Memcpy(puchZipArr, puchZipDataArr, uchDiffarrcnt8bit + 1);
}

/**
 * @fn     void GH3X2X_GetDataDiff(GU32 *punData, GU8 uchChannelMapCnt, GU8 *puchZipDataArr, 
                            GU8 *puchZipDataArrSize, GU32 *punZipTempData)
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
void GH3X2X_GetDataDiff(GU32 *punData, GU8 uchChannelMapCnt, GU8 *puchZipDataArr, 
                        GU8 *puchZipDataArrSize, GU32 *punZipTempData)
{
    GU8 uchDiffunittemp;
    GS8 chNum;
    GU8 uchChnum;
    GU32 unChnum;
    GU8 uchDifftemp;
    GU8 uchDiffarrcnt8bit = 0;
    GU8 uchDiffarrcnt4bit = 0;
    GU8 uchDataType = 0;

    //GU8 puchZipDataArr[255];
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
            uchDifftemp = ((GU8)(GH3X2X_GET_FIRST_4BITS((unChnum) >> (chNum*RAWDATA_DIFF_BYTE_SIZE))));
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
            puchZipDataArr[uchDiffarrcnt8bit] |= (GU8)(uchDataType << RAWDATA_DIFF_BYTE_SIZE);
        }
        uchDiffarrcnt4bit++;
        
        for (chNum = (GS8)(uchDataType/RAWDATA_DIFF_EVEN) ; chNum >= 0 ; chNum--)
        {
            uchDiffunittemp = (GU8)(GH3X2X_GET_FIRST_4BITS((unChnum) >> (chNum*RAWDATA_DIFF_BYTE_SIZE)));
            if (uchDiffarrcnt4bit%RAWDATA_DIFF_EVEN)
            {
                puchZipDataArr[uchDiffarrcnt8bit] |= uchDiffunittemp;
            }
            else
            {
                uchDiffarrcnt8bit++;
                puchZipDataArr[uchDiffarrcnt8bit] |= (GU8)(uchDiffunittemp << RAWDATA_DIFF_BYTE_SIZE);
            }
            uchDiffarrcnt4bit++;
        }
    }// end of (for uchChnum=0;uchChnum<uchChannelMapCnt;uchChnum++)
    
    uchDiffarrcnt8bit = (uchDiffarrcnt4bit + RAWDATA_DIFF_ODD) / RAWDATA_DIFF_EVEN + RAWDATA_DIFF_ODD;
    puchZipDataArr[0] = uchDiffarrcnt8bit;
    *puchZipDataArrSize = (GU8)(uchDiffarrcnt8bit + RAWDATA_DIFF_ODD);
//    GH3X2X_Memcpy(puchZipArr, puchZipDataArr, uchDiffarrcnt8bit + 1);
}

void GH3X2X_ZipFillLastRawdataBuffer(const STGh3x2xFrameInfo * const pstFrameInfo, GU32 unFunctionID, 
                                            GU8 uchChnlNum, GU8 *uchFifoLastRawdataTagArr, GU32 *unFifoLastRawdataArr)
{
    for(GU8 uchChnlCnt = 0; uchChnlCnt < uchChnlNum; uchChnlCnt ++)
    {
        //led adj flag 0 
        if(pstFrameInfo->punFrameFlag[0]&((((GU32)1)) << uchChnlCnt))
        {
            pstFrameInfo->punFrameRawdata[uchChnlCnt] |= ((((GU32)1)) << 26);
        }
        // fast recovery
        if ((GH3X2X_FUNCTION_ECG == unFunctionID)||(GH3X2X_FUNCTION_PWTT == unFunctionID))
        {
            if(0 == uchChnlCnt)
            {
                if(pstFrameInfo->punFrameFlag[2]&((((GU32)1)) << 0))
                {
                    pstFrameInfo->punFrameRawdata[uchChnlCnt] |= ((((GU32)1)) << 26);
                }
            }
        }
        //led adj flag 1 
        if(pstFrameInfo->punFrameFlag[1]&((((GU32)1)) << uchChnlCnt))
        {
            pstFrameInfo->punFrameRawdata[uchChnlCnt] |= ((((GU32)1)) << 25);
        }
        
        uchFifoLastRawdataTagArr[uchChnlCnt] = (pstFrameInfo->punFrameRawdata[uchChnlCnt] >> 24)&0x000000FF;
        unFifoLastRawdataArr[uchChnlCnt] = pstFrameInfo->punFrameRawdata[uchChnlCnt]&0x00FFFFFF;

        pstFrameInfo->punFrameRawdata[uchChnlCnt] &= 0x00FFFFFF;
    }
}

void GH3X2X_ZipFillLastDataBuffer(GU32 *unCurrentData, GU8 uchChnlNum, GU32 *unLastdata)
{
    for(GU8 uchChnlCnt = 0; uchChnlCnt < uchChnlNum; uchChnlCnt ++)
    {
        unLastdata[uchChnlCnt] = unCurrentData[uchChnlCnt];
    }
}


extern GU8 g_uchGh3x2xUploadStatus;
extern GU8 g_uchGh3x2xUploadBufUse;
GU8 g_uchCurrentFunctionID = 0;
__weak void Gh2x2xUploadZipDataToMaster(const STGh3x2xFrameInfo * const pstFrameInfo, GU16 usFrameCnt, GU16 usFrameNum, GU8* puchTagArray)
{
    GU8  uchChnlNum;
    GU32 unFunctionID;
    GU8  uchFunctionIDForMaster = 0xFF;
    GU8  uchGsEnable;
    GU8  uchCapEnable;
    GU8  uchTempEnable;
    GU8  uchGsGyroEnable;
    GU8  uchAgcEnable;
    GU8  uchAmbEnable;
    GU8  uchAlgoResFlag;
    GU16 uchOneFrameByteNum = 0;
    GU8  uchReslutByteNum;
    GU8  uchNeedContinue;
    GU8  *puchUploadBuf = g_pstGh3x2xProtocolData->puchPacketPayloadArr + 4;
    GU8 puchTempFrameDataBuf[2 * (GH3X2X_UPROTOCOL_PAYLOAD_LEN_MAX + 8)];
    GU16 usRespondLen;
    GU8 uchFramePosi;
    GU8 uchEvenFirstFrameFlag = 0;

    GU8 uchZipRawdataArr[GH3X2X_UPROTOCOL_PAYLOAD_LEN_MAX] = {0};
    GU8 uchZipRawdataArrSize = 0;
    GU8 uchZipAgcdataArr[GH3X2X_UPROTOCOL_PAYLOAD_LEN_MAX] = {0};
    GU8 uchZipAgcdataArrSize = 0;
    GU8 uchFifoLastRawdataTagArr[CHANNEL_MAP_ID_NUM] = {0};
    GU32 unFrameRawdataArr[CHANNEL_MAP_ID_NUM] = {0};
    GU32 unAgcdataArr[CHANNEL_MAP_ID_NUM] = {0};

    GU8 uchFirstFrameLen = 0;
    GU8 uchFillLastFrameDataFlag = 0;
    if(0 == pstFrameInfo)
    {
        return;
    }
    unFunctionID = pstFrameInfo->unFunctionID;
    uchChnlNum = pstFrameInfo->pstFunctionInfo->uchChnlNum;
    uchFramePosi = 0;
    if(0 == usFrameCnt)
    {
        uchFramePosi |= GH3X2X_FRAME_POSI_HEAD;
    }
    if((usFrameNum - 1) == usFrameCnt)
    {
        uchFramePosi |= GH3X2X_FRAME_POSI_TAIL;
    }
    /*
    if(GH3X2X_FUNCTION_SOFT_ADT_IR == unFunctionID)
    {
        unFunctionID = GH3X2X_FUNCTION_SOFT_ADT_GREEN;  //adt ir and adt green store in one csv file
    }
    */

    //function id transform
    for (int uchOffsetCnt = 0 ; uchOffsetCnt < GH3X2X_FUNC_OFFSET_MAX ; uchOffsetCnt ++)
    {
        if ((unFunctionID >> uchOffsetCnt) & 0x1)
        {
            uchFunctionIDForMaster = uchOffsetCnt;
        }
    }

    
    if (g_uchCurrentFunctionID != uchFunctionIDForMaster)
    {
        g_uchCurrentFunctionID = uchFunctionIDForMaster;
        g_uchOddEvenChangeFlag = 1;
    }

    if(0xFF == uchFunctionIDForMaster)  //invalid function id
    {
        return;
    }
    uchGsEnable = g_uchGsensorEnable;
    uchGsGyroEnable = (g_uchGyroEnable & uchGsEnable);
    uchCapEnable =  GH3X2X_GetCapEnableFlag();
    uchTempEnable = GH3X2X_GetTempEnableFlag();
    uchAgcEnable = GH3X2X_GetLedAgcState();
    if(unFunctionID == GH3X2X_FUNC_OFFSET_ECG)
    {
        uchAgcEnable = 0;
    }
    uchAmbEnable = 0;
    uchAlgoResFlag = 1;


    /********  calculate how many bytes in one frame  ******/
    //result byte num
    uchReslutByteNum = 0;
    uchReslutByteNum += 1;
    //FLAG 2
    uchReslutByteNum += 5;
    //FLAG 3 (need upload on only first frame)
    if(uchFramePosi&GH3X2X_FRAME_POSI_HEAD)
    {
        uchReslutByteNum += 5;
    }
    
    //section FRAME_ID
    uchFirstFrameLen += 2;
    uchFirstFrameLen += 1;
    //section GSENSOR_DATA
    if(uchGsEnable)
    {
        uchFirstFrameLen += 6;
        if(uchGsGyroEnable)
        {
            uchFirstFrameLen += 6;
        }
    }
    //section Cap and temp
    if(uchCapEnable)
    {
        uchFirstFrameLen += 12;
    }
    if(uchTempEnable)
    {
        uchFirstFrameLen += 12;
    }
    
    if (uchAlgoResFlag && 1 == pstFrameInfo->pstAlgoResult->uchUpdateFlag)
    {
        uchOneFrameByteNum += (5*pstFrameInfo->pstAlgoResult->uchResultNum);
        uchReslutByteNum += (5*pstFrameInfo->pstAlgoResult->uchResultNum);
    }

    GH3x2xSendFunctionInfo(pstFrameInfo); //send function info when first frame
    
    uchNeedContinue = 1;
    while(uchNeedContinue)
    {
        if(GH3X2X_UPLOAD_STATUS_NULL == g_uchGh3x2xUploadStatus)
        {
            Gh2x2xPackPakcageHeader(&uchNeedContinue, uchFunctionIDForMaster,\
                            puchUploadBuf, puchTagArray, uchGsEnable, uchAlgoResFlag,\
                            uchAgcEnable, uchAmbEnable, uchGsGyroEnable, uchCapEnable,uchTempEnable,1, g_uchOddEvenChangeFlag);
        }

        
        if(GH3X2X_UPLOAD_STATUS_HAVE_STORE_DATA == g_uchGh3x2xUploadStatus && uchFillLastFrameDataFlag == 0)
        {
            
            uchNeedContinue = 0;
            uchOneFrameByteNum = 0;
            
            //section FRAME_ID
            puchTempFrameDataBuf[uchOneFrameByteNum] = ((*(pstFrameInfo->punFrameCnt))&0x000000FF);
        
            
            uchOneFrameByteNum ++;
            //section GSENSOR_DATA
            if(uchGsEnable)
            {
                GS16 *psGsensorData;
                psGsensorData = pstFrameInfo->pusFrameGsensordata;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(psGsensorData[0]); //gsx
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_LOW_BYTE_FROM_WORD(psGsensorData[0]);
                uchOneFrameByteNum += 1;
                
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(psGsensorData[1]); //gsy
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_LOW_BYTE_FROM_WORD(psGsensorData[1]);
                uchOneFrameByteNum += 1;
                
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(psGsensorData[2]); //gsz
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_LOW_BYTE_FROM_WORD(psGsensorData[2]);
                uchOneFrameByteNum += 1;
                if(uchGsGyroEnable)
                {
                    puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(psGsensorData[3]); //gyrox
                    uchOneFrameByteNum += 1;
                    puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_LOW_BYTE_FROM_WORD(psGsensorData[3]);
                    uchOneFrameByteNum += 1;
                    
                    puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(psGsensorData[4]); //gyroy
                    uchOneFrameByteNum += 1;
                    puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_LOW_BYTE_FROM_WORD(psGsensorData[4]);
                    uchOneFrameByteNum += 1;
                    
                    puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(psGsensorData[5]); //gyroz
                    uchOneFrameByteNum += 1;
                    puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_LOW_BYTE_FROM_WORD(psGsensorData[5]);
                    uchOneFrameByteNum += 1;
                }
            }
            if(uchCapEnable)
            {
             
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE2_FROM_DWORD(pstFrameInfo->pstFrameCapdata->unCapData[0]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE1_FROM_DWORD(pstFrameInfo->pstFrameCapdata->unCapData[0]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE0_FROM_DWORD(pstFrameInfo->pstFrameCapdata->unCapData[0]); 
                uchOneFrameByteNum += 1;

                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE2_FROM_DWORD(pstFrameInfo->pstFrameCapdata->unCapData[1]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE1_FROM_DWORD(pstFrameInfo->pstFrameCapdata->unCapData[1]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE0_FROM_DWORD(pstFrameInfo->pstFrameCapdata->unCapData[1]); 
                uchOneFrameByteNum += 1;

                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE2_FROM_DWORD(pstFrameInfo->pstFrameCapdata->unCapData[2]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE1_FROM_DWORD(pstFrameInfo->pstFrameCapdata->unCapData[2]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE0_FROM_DWORD(pstFrameInfo->pstFrameCapdata->unCapData[2]); 
                uchOneFrameByteNum += 1;

                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE2_FROM_DWORD(pstFrameInfo->pstFrameCapdata->unCapData[3]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE1_FROM_DWORD(pstFrameInfo->pstFrameCapdata->unCapData[3]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE0_FROM_DWORD(pstFrameInfo->pstFrameCapdata->unCapData[3]); 
                uchOneFrameByteNum += 1;

            }
            if(uchTempEnable)
            {
            
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE2_FROM_DWORD(pstFrameInfo->pstFrameTempdata->unTempData[0]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE1_FROM_DWORD(pstFrameInfo->pstFrameTempdata->unTempData[0]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE0_FROM_DWORD(pstFrameInfo->pstFrameTempdata->unTempData[0]); 
                uchOneFrameByteNum += 1;
                
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE2_FROM_DWORD(pstFrameInfo->pstFrameTempdata->unTempData[1]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE1_FROM_DWORD(pstFrameInfo->pstFrameTempdata->unTempData[1]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE0_FROM_DWORD(pstFrameInfo->pstFrameTempdata->unTempData[1]); 
                uchOneFrameByteNum += 1;
            
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE2_FROM_DWORD(pstFrameInfo->pstFrameTempdata->unTempData[2]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE1_FROM_DWORD(pstFrameInfo->pstFrameTempdata->unTempData[2]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE0_FROM_DWORD(pstFrameInfo->pstFrameTempdata->unTempData[2]); 
                uchOneFrameByteNum += 1;
            
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE2_FROM_DWORD(pstFrameInfo->pstFrameTempdata->unTempData[3]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE1_FROM_DWORD(pstFrameInfo->pstFrameTempdata->unTempData[3]); 
                uchOneFrameByteNum += 1;
                puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE0_FROM_DWORD(pstFrameInfo->pstFrameTempdata->unTempData[3]); 
                uchOneFrameByteNum += 1;
            
            }
            //section RAWDATA
            //set flag ,copy rawdata, clear flag
            
            if (GH3X2X_GetFifoPackageMode() == 1)
            {
                if (g_uchOddEvenChangeFlag && g_uchGh3x2xUploadBufUse <= GH3X2X_UPLOAD_PACKAGE_HEAD_SIZE)
                {
                    uchEvenFirstFrameFlag = 1;
                }
                puchTempFrameDataBuf[uchOneFrameByteNum] = g_uchFifoPackageID;
                uchOneFrameByteNum ++;
            }
            else
            {
                if (g_uchOddEvenChangeFlag && g_uchGh3x2xUploadBufUse <= GH3X2X_UPLOAD_PACKAGE_HEAD_SIZE)
                {
                    uchEvenFirstFrameFlag = 1;
                    for(GU8 uchChnlCnt = 0; uchChnlCnt < uchChnlNum; uchChnlCnt ++)
                    {
                        //led adj flag 0 
                        if(pstFrameInfo->punFrameFlag[0]&((((GU32)1)) << uchChnlCnt))
                        {
                            pstFrameInfo->punFrameRawdata[uchChnlCnt] |= ((((GU32)1)) << 26);
                        }
                        // fast recovery
                        if ((GH3X2X_FUNCTION_ECG == unFunctionID)||(GH3X2X_FUNCTION_PWTT == unFunctionID))
                        {
                            if(0 == uchChnlCnt)
                            {
                                if(pstFrameInfo->punFrameFlag[2]&((((GU32)1)) << 0))
                                {
                                    pstFrameInfo->punFrameRawdata[uchChnlCnt] |= ((((GU32)1)) << 26);
                                }
                            }
                        }
                        //led adj flag 1 
                        if(pstFrameInfo->punFrameFlag[1]&((((GU32)1)) << uchChnlCnt))
                        {
                            pstFrameInfo->punFrameRawdata[uchChnlCnt] |= ((((GU32)1)) << 25);
                        }

                        puchTempFrameDataBuf[uchOneFrameByteNum] = (pstFrameInfo->punFrameRawdata[uchChnlCnt] >> 24)&0x000000FF;
                        uchOneFrameByteNum ++;
                        
                        puchTempFrameDataBuf[uchOneFrameByteNum] = (pstFrameInfo->punFrameRawdata[uchChnlCnt] >> 16)&0x000000FF;
                        uchOneFrameByteNum ++;
                        
                        puchTempFrameDataBuf[uchOneFrameByteNum] = (pstFrameInfo->punFrameRawdata[uchChnlCnt] >> 8)&0x000000FF;
                        uchOneFrameByteNum ++;
                        
                        puchTempFrameDataBuf[uchOneFrameByteNum] = (pstFrameInfo->punFrameRawdata[uchChnlCnt] >> 0)&0x000000FF;
                        uchOneFrameByteNum ++;

                        g_stZipLastData.uchFifoLastRawdataTagArr[uchFunctionIDForMaster][uchChnlCnt] = (pstFrameInfo->punFrameRawdata[uchChnlCnt] >> 24)&0x000000FF;
                        g_stZipLastData.unFifoLastRawdataArr[uchChnlCnt] = pstFrameInfo->punFrameRawdata[uchChnlCnt]&0x00FFFFFF;

                        pstFrameInfo->punFrameRawdata[uchChnlCnt] &= 0x00FFFFFF;
                    }
                }
                else
                {
                    GH3X2X_ZipFillLastRawdataBuffer(pstFrameInfo, unFunctionID, uchChnlNum, uchFifoLastRawdataTagArr, unFrameRawdataArr);
                    GH3X2X_GetRawDataDiff(unFrameRawdataArr, uchFifoLastRawdataTagArr,\
                        uchChnlNum, uchZipRawdataArr, &uchZipRawdataArrSize,\
                        g_stZipLastData.uchFifoLastRawdataTagArr[uchFunctionIDForMaster], g_stZipLastData.unFifoLastRawdataArr);
                    GH3X2X_Memcpy(&puchTempFrameDataBuf[uchOneFrameByteNum], uchZipRawdataArr, uchZipRawdataArrSize);
                                    uchOneFrameByteNum += uchZipRawdataArrSize;
                }
            }
            //section AGC_INFO
            if(uchAgcEnable)
            {
                if (uchEvenFirstFrameFlag)
                {
                    GH3X2X_ZipFillLastDataBuffer(pstFrameInfo->punFrameAgcInfo, uchChnlNum, g_stZipLastData.unFifoLastAgcdataArr);
                    GH3X2X_Memcpy((GU8*)&(puchTempFrameDataBuf[uchOneFrameByteNum]), (GU8*)(pstFrameInfo->punFrameAgcInfo), uchChnlNum*4);
                    uchOneFrameByteNum += uchChnlNum*4;
                }
                else
                {
                    GH3X2X_ZipFillLastDataBuffer(pstFrameInfo->punFrameAgcInfo, uchChnlNum, unAgcdataArr);
                    GH3X2X_GetDataDiff(unAgcdataArr, uchChnlNum, uchZipAgcdataArr, 
                                    &uchZipAgcdataArrSize, g_stZipLastData.unFifoLastAgcdataArr);
                    GH3X2X_Memcpy(&puchTempFrameDataBuf[uchOneFrameByteNum], uchZipAgcdataArr, uchZipAgcdataArrSize);
                    uchOneFrameByteNum += uchZipAgcdataArrSize;
                }
            }

            //section AmbData
            if(uchAmbEnable)
            {
                GH3X2X_Memcpy((GU8*)&(puchUploadBuf[uchOneFrameByteNum]), (GU8*)(pstFrameInfo->punFrameAgcInfo), uchChnlNum*3);
                uchOneFrameByteNum += uchChnlNum*3;
            }
            
            //section RESLUT_DATA
            //result byte num
            //puchTempFrameDataBuf[g_uchGh3x2xUploadBufUse] = 0;
            GU8 uchRealReslutByteNum = uchReslutByteNum;
            uchOneFrameByteNum ++;
            //flag2
            if ((*(pstFrameInfo->punFrameCnt)) == 0 || pstFrameInfo->punFrameFlag[2] != g_unLastFrameFlag2)
            {
                puchTempFrameDataBuf[uchOneFrameByteNum] = 2;
                uchOneFrameByteNum ++;        
                GH3X2X_Memcpy((GU8*)&(puchTempFrameDataBuf[uchOneFrameByteNum]), (GU8*)(&(pstFrameInfo->punFrameFlag[2])), 4);
                uchOneFrameByteNum += 4;
                g_unLastFrameFlag2 = pstFrameInfo->punFrameFlag[2];
            }
            else
            {
                uchRealReslutByteNum -= 5;
            }
            //flag3
            if(uchFramePosi&GH3X2X_FRAME_POSI_HEAD)
            {
                puchTempFrameDataBuf[uchOneFrameByteNum] = 3;
                uchOneFrameByteNum ++;     
                GH3X2X_Memcpy((GU8*)&(puchTempFrameDataBuf[uchOneFrameByteNum]), (GU8*)(&(pstFrameInfo->punFrameFlag[3])), 4);
                uchOneFrameByteNum += 4;    
            }

            //alg result

            if (1 == uchAlgoResFlag)
            {
                if (1 == pstFrameInfo->pstAlgoResult->uchUpdateFlag)
                {                                                                             
                    GU8 uchResTagData = GH3X2X_GET_LEFT_SHIFT_VAL(UPROTOCOL_RESULT_TYPE_BIT_FIELD);
                    for(GU8 uchResultCnt = 0;uchResultCnt < pstFrameInfo->pstAlgoResult->uchResultNum;uchResultCnt++)
                    {
                        puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_SET_BIT(uchResTagData, uchResultCnt);
                        uchOneFrameByteNum ++ ;
                        puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE0_FROM_DWORD(pstFrameInfo->pstAlgoResult->snResult[uchResultCnt]);
                        uchOneFrameByteNum ++ ;
                        puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE1_FROM_DWORD(pstFrameInfo->pstAlgoResult->snResult[uchResultCnt]);
                        uchOneFrameByteNum ++ ;
                        puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE2_FROM_DWORD(pstFrameInfo->pstAlgoResult->snResult[uchResultCnt]);
                        uchOneFrameByteNum ++ ;
                        puchTempFrameDataBuf[uchOneFrameByteNum] = GH3X2X_GET_BYTE3_FROM_DWORD(pstFrameInfo->pstAlgoResult->snResult[uchResultCnt]);
                        uchOneFrameByteNum ++ ;
                    }
                } // end of if (1 == uchAlgoResFlag)
            }
            
            //modify uchReslutByteNum
            puchTempFrameDataBuf[uchOneFrameByteNum - uchRealReslutByteNum] = uchRealReslutByteNum - 1;

            GU8 uchSendFlag = 0; 
                
            if(((GU16)g_uchGh3x2xUploadBufUse + uchOneFrameByteNum) >= GH3X2X_UPROTOCOL_GET_PAYLOAD_LEN_SUPPORT(g_uchPacketMaxLenSupport))  //can not store more data
            {
                if (g_uchGh3x2xUploadBufUse <= GH3X2X_UPLOAD_PACKAGE_HEAD_SIZE)
                {
                    uchSendFlag = GH3X2X_SEND_STATUS_SPLIC_PACK_MODE;
                }
                else
                {
                    uchSendFlag = GH3X2X_SEND_STATUS_FULL_MODE;
                }
            }
            else if (uchFramePosi&GH3X2X_FRAME_POSI_TAIL)
            {
                uchSendFlag = GH3X2X_SEND_STATUS_TAIL_MODE;
            }

            if (uchSendFlag == GH3X2X_SEND_STATUS_SPLIC_PACK_MODE)
            {
                GU16 usMaxPackLen = GH3X2X_UPROTOCOL_GET_PAYLOAD_LEN_SUPPORT(g_uchPacketMaxLenSupport) - g_uchGh3x2xUploadBufUse;
                GU16 usPackNum = uchOneFrameByteNum / usMaxPackLen;
                GU16 usPackByteCnt = 0;
                GU16 usExternPackLen = uchOneFrameByteNum % usMaxPackLen;
                GU16 usPackCnt = 1;
                
                for (usPackCnt = 1 ; usPackCnt < usPackNum + 1 ; usPackCnt ++)
                {
                    puchUploadBuf[UPROTOCOL_RAWDATA_LEN_INDEX] = g_uchGh3x2xUploadBufUse + usMaxPackLen;
                    puchUploadBuf[UPROTOCOL_PACKAGE_TYPE_INDEX] &= 0x7;
                    puchUploadBuf[UPROTOCOL_PACKAGE_TYPE_INDEX] |= GH3X2X_VAL_LEFT_SHIFT((GU8)usPackCnt, UPROTOCOL_SPLIC_PACK_CNT_FIELD);
                    GH3X2X_Memcpy(&puchUploadBuf[g_uchGh3x2xUploadBufUse], &puchTempFrameDataBuf[usPackByteCnt], usMaxPackLen);
                    usPackByteCnt += usMaxPackLen;
                    usRespondLen = GH3X2X_UprotocolPacketFormat(GH3X2X_UPROTOCOL_CMD_NEW_STRUCT_RAWDATA, puchUploadBuf - 4,
                                        puchUploadBuf, g_uchGh3x2xUploadBufUse + usMaxPackLen);
                    Gh3x2xDemoSendProtocolData(puchUploadBuf - 4,usRespondLen);
                }
                puchUploadBuf[UPROTOCOL_RAWDATA_LEN_INDEX] = g_uchGh3x2xUploadBufUse + usExternPackLen;
                puchUploadBuf[UPROTOCOL_PACKAGE_TYPE_INDEX] &= 0x7;
                puchUploadBuf[UPROTOCOL_PACKAGE_TYPE_INDEX] |= GH3X2X_VAL_LEFT_SHIFT((GU8)usPackCnt, UPROTOCOL_SPLIC_PACK_CNT_FIELD);
                puchUploadBuf[UPROTOCOL_PACKAGE_TYPE_INDEX] |= GH3X2X_VAL_LEFT_SHIFT(1, UPROTOCOL_SPLIC_PACK_OVER_FIELD);
                GH3X2X_Memcpy(&puchUploadBuf[g_uchGh3x2xUploadBufUse], &puchTempFrameDataBuf[usPackByteCnt], usExternPackLen);
                usPackByteCnt += usExternPackLen;
                usRespondLen = GH3X2X_UprotocolPacketFormat(GH3X2X_UPROTOCOL_CMD_NEW_STRUCT_RAWDATA, puchUploadBuf - 4,
                                    puchUploadBuf, g_uchGh3x2xUploadBufUse + usExternPackLen);
                Gh3x2xDemoSendProtocolData(puchUploadBuf - 4,usRespondLen);
                
                g_uchOddEvenChangeFlag =! g_uchOddEvenChangeFlag;
                g_uchGh3x2xUploadStatus = GH3X2X_UPLOAD_STATUS_NULL;
                return;
            }

            if(((GU16)g_uchGh3x2xUploadBufUse + uchOneFrameByteNum) >= GH3X2X_UPROTOCOL_GET_PAYLOAD_LEN_SUPPORT(g_uchPacketMaxLenSupport))  //can not store more data
            {
                //send data
                usRespondLen = GH3X2X_UprotocolPacketFormat(GH3X2X_UPROTOCOL_CMD_NEW_STRUCT_RAWDATA, puchUploadBuf - 4,
                                    puchUploadBuf, g_uchGh3x2xUploadBufUse);
                Gh3x2xDemoSendProtocolData(puchUploadBuf - 4,usRespondLen);
                g_uchOddEvenChangeFlag = !g_uchOddEvenChangeFlag;
                g_uchGh3x2xUploadStatus = GH3X2X_UPLOAD_STATUS_NULL;
                uchNeedContinue = 1;
                continue;
            }
            
            GH3X2X_Memcpy(&puchUploadBuf[g_uchGh3x2xUploadBufUse], puchTempFrameDataBuf, uchOneFrameByteNum);
            g_uchGh3x2xUploadBufUse += uchOneFrameByteNum;
            if (uchEvenFirstFrameFlag != 1)
            {
                GH3X2X_Memcpy(g_stZipLastData.unFifoLastRawdataArr, unFrameRawdataArr, uchChnlNum*4);
                GH3X2X_Memcpy(g_stZipLastData.uchFifoLastRawdataTagArr[uchFunctionIDForMaster], uchFifoLastRawdataTagArr, uchChnlNum);
                GH3X2X_Memcpy(g_stZipLastData.unFifoLastAgcdataArr, unAgcdataArr, uchChnlNum*4);
            }
            puchUploadBuf[UPROTOCOL_RAWDATA_LEN_INDEX] += uchOneFrameByteNum;

            if(uchFramePosi&GH3X2X_FRAME_POSI_TAIL)
            {
                //send data
                usRespondLen = GH3X2X_UprotocolPacketFormat(GH3X2X_UPROTOCOL_CMD_NEW_STRUCT_RAWDATA, puchUploadBuf - 4,
                                    puchUploadBuf, g_uchGh3x2xUploadBufUse);
                Gh3x2xDemoSendProtocolData(puchUploadBuf - 4,usRespondLen);
                g_uchOddEvenChangeFlag = !g_uchOddEvenChangeFlag;
                g_uchGh3x2xUploadStatus = GH3X2X_UPLOAD_STATUS_NULL;
            }
        }
    }
}

