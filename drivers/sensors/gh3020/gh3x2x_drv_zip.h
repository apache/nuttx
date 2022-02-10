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
    uint8_t uchFifoLastRawdataTagArr[GH3X2X_FUNC_OFFSET_MAX][GH3X2X_CHANNEL_MAP_MAX_CH];
    uint32_t unFifoLastRawdataArr[GH3X2X_CHANNEL_MAP_MAX_CH];
    uint32_t unFifoLastAgcdataArr[GH3X2X_CHANNEL_MAP_MAX_CH];
} STZipLastDataStruct;




extern void GH3X2X_GetRawDataDiff(uint32_t *punRawData, uint8_t *puchRawdataTagTempArr,
                            uint8_t uchChannelMapCnt, uint8_t *puchZipDataArr, uint8_t *puchZipDataArrSize,
                            uint8_t *uchFifoLastRawdataTagArr, uint32_t *unFifoLastRawdataArr);

extern void GH3X2X_GetDataDiff(uint32_t *punData, uint8_t uchChannelMapCnt, uint8_t *puchZipDataArr,
                        uint8_t *puchZipDataArrSize, uint32_t *punZipTempData);

extern void GH3X2X_ZipmodeInit(void);

#endif

