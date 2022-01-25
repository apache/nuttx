/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_grv_zip.h
 *
 * @brief   gh3x2x universal protocol define & declaration
 *
 * @version ref gh3x2x_grv_zip.h
 *
 */
#ifndef _GH3X2X_DRV_ZIPHANDLE_H_
#define _GH3X2X_DRV_ZIPHANDLE_H_


/**
 * @brief ZipMode Lastdata Temp
 */
typedef struct
{
    GU8 uchFifoLastRawdataTagArr[GH3X2X_FUNC_OFFSET_MAX][GH3X2X_CHANNEL_MAP_MAX_CH];
    GU32 unFifoLastRawdataArr[GH3X2X_CHANNEL_MAP_MAX_CH];
    GU32 unFifoLastAgcdataArr[GH3X2X_CHANNEL_MAP_MAX_CH];
} STZipLastDataStruct;




extern void GH3X2X_GetRawDataDiff(GU32 *punRawData, GU8 *puchRawdataTagTempArr, 
                            GU8 uchChannelMapCnt, GU8 *puchZipDataArr, GU8 *puchZipDataArrSize, 
                            GU8 *uchFifoLastRawdataTagArr, GU32 *unFifoLastRawdataArr);

extern void GH3X2X_GetDataDiff(GU32 *punData, GU8 uchChannelMapCnt, GU8 *puchZipDataArr, 
                        GU8 *puchZipDataArrSize, GU32 *punZipTempData);

extern void GH3X2X_ZipmodeInit(void);

#endif

