/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_adt_.h
 *
 * @brief   gh3x2x adt
 *
 * @version ref gh3x2x_drv_version.h
 *
 */


#ifndef _GH3X2X_DRV_ADT_H_
#define _GH3X2X_DRV_ADT_H_


#include "gh3x2x_drv_config.h"
#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"



typedef struct
{
    uint8_t uchElectrodeAdtEn          : 1;  //0:disable electrode adt  1:enable electrode adt
    uint8_t uchElectrodeWearRevCntEn   : 1;  //elctrode wear state reverse counter dis/en    0: disable  1: enable
    uint8_t uchSoftWearOffDetEn        : 1;  //software wear off detction enable   0: disable  1: enable
    uint8_t uchElectrodeSoftWearColor  : 5;  //software wear led color             0:green     1:ir
}STAdtCfgByte0;



typedef struct t_adt_cfg
{
    uint8_t uchAdtSampleCtrlMode;     // 0: wear state do not affect sample  1:  wear off only open adt slot     2: wear on/off need switch cfg
    STAdtCfgByte0 stAdtCfgByte0;
    uint8_t ubResByte2;
    uint8_t ubResByte3;
}STAdtCfg;


/// wear on event en set
#define  WEAR_ON_EVENT_EN_CONFIG(en)        do {\
                                                gh3020_spi_writebits(GH3020_REG_INT_CR2, \
                                                    GH3020_WEAR_ON_EVENT_EN_LSB, GH3020_WEAR_ON_EVENT_EN_MSB, (en)); \
                                            } while (0)

/// wear off event en set
#define  WEAR_OFF_EVENT_EN_CONFIG(en)        do {\
                                                gh3020_spi_writebits(GH3020_REG_INT_CR2, \
                                                    GH3020_WEAR_OFF_EVENT_EN_LSB, GH3020_WEAR_OFF_EVENT_EN_MSB, (en)); \
                                            } while (0)



#define SOFT_WEAR_INPUT_DATA_CHNL0_INDEX   (0)
#define SOFT_WEAR_INPUT_DATA_ACCX_INDEX    (1)
#define SOFT_WEAR_INPUT_DATA_ACCY_INDEX    (2)
#define SOFT_WEAR_INPUT_DATA_ACCZ_INDEX    (3)
#define SOFT_WEAR_INPUT_COLOR_INDEX        (4)
#define SOFT_WEAR_INPUT_DATA_ADT_INDEX     (5) //ir  adt data
#define SOFT_WEAR_INPUT_DATA_BG_INDEX      (6)
#define SOFT_WEAR_INPUT_DATA_CAP0_INDEX    (7)
#define SOFT_WEAR_INPUT_DATA_CAP1_INDEX    (8)
#define SOFT_WEAR_INPUT_DATA_CAP2_INDEX    (9)
#define SOFT_WEAR_INPUT_DATA_CAP3_INDEX    (10)


#define SOFT_WEAR_INPUT_DATA_INDEX_TOTAL (11)
#define SOFT_WEAR_OUTPUT_DATA_INDEX_TOTAL (8)


#define SOFT_WEAR_OUTPUT_WEAR_STATE_BIT  (3)


#define NADTCAP_BYTE_TRUE       (uint8_t)1
#define NADTCAP_BYTE_FALSE      (uint8_t)0



#define NADT_SAMPLE_FREQ  25
#define NADT_WWD_IIR_LEN  4

#define NADT_WEAR_LIVE_DETECTION
#ifdef  NADT_WEAR_LIVE_DETECTION // this is wear live dection, full wear dection,can distinguish body and other thing; more Ram
#define NADT_PPG_BUFF_LEN (4) //4
#else
#define NADT_PPG_BUFF_LEN (2) //lite
#endif // NADT_WEAR_LIVE_DETECTION

#define NADT_WWD_FILTER_BUFF_LEN (NADT_PPG_BUFF_LEN*NADT_SAMPLE_FREQ)
#define NADT_MATH_ROUND(v) round(v)
#define NADT_MATH_ABSF(v)  fabs(v)
#define NADT_MATH_ABS(v)   abs(v)
#define NADT_MATH_ROUNDF(v) roundf(v)
#define NADT_ABS_GS32(v) abs((int32_t)(v))
#define NADT_MIN(a,b) ((a) > (b) ? (b) : (a))



#define NADT_PV_PEAK (1)
#define NADT_PV_VALLEY (2)
#define NADT_PV_PV_INTER (4)
#define NADT_ACC_TRUNCATE (5)

typedef struct
{
    int32_t *pData;
    int32_t nLength;
    int32_t nPos;
}NADT_QUEUE_BUFFER_S32;

typedef struct
{
    int16_t *pData;
    int32_t nLength;
    int32_t nPos;
}NADT_QUEUE_BUFFER_S16;

typedef struct
{
    const float *pfCoefB;
    const float *pfCoefA;
    float *pfXbuff;
    float *pfYbuff;
    int32_t lLen;
    float fThr;
}NADT_ST_IIR_PARAM;

typedef struct
{
    int32_t *pfFftValue;
    int16_t *psPeakIndex;
    int16_t *psValleyIndex;
    int16_t *psPeakValleyIndex;
    int32_t lPeakValleyLen;
    int32_t lPeakLen;
    int32_t lValleyLen;
    int32_t lDataLen;
}NADT_ST_PEAK_VALLEY_INFO;

typedef struct
{
    bool bSearchMax;
    int32_t fExtremumValue;
    int32_t lPreExtremumPos;
    int32_t lExtremumPos;
    int32_t lDataCount;
}NADT_ST_PEAKX_DET;

//#ifdef __cplusplus
//extern "C" {
//#endif


extern STAdtCfg g_stAdtModuleCfg;
extern uint8_t g_uchElectrodeWearRevCnt;
extern uint8_t  g_uchSoftAdtChannl0Color;

#if 1
    //void NADTControl(int32_t lOptTye, int32_t lConfigValue[]);
#endif

extern uint8_t NADTProc(int32_t lPacketInfoArr[], int32_t lResult[]);
extern int8_t* GetNadtVersion(void);

    //bool HBA_NADTProc(int32_t lPacketInfoArr[], uint8_t lResult[1]);

//#ifdef __cplusplus
//}
//#endif







/**
 * @fn     void GH3X2X_AdtPramInit(void)
 *
 * @brief  GH3X2X_AdtPramInit
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_AdtPramInit(void);


/**
 * @fn     void GH3X2X_AdtPramSet(uint16_t usRegVal, uint8_t uchRegPosi)
 *
 * @brief  GH3X2X_AdtPramSet
 *
 * @attention   None
 *
 * @param[in]   usRegVal
 * @param[in]   uchRegPosi
 * @param[out]  None
 *
 * @return  None
 */

void GH3X2X_AdtPramSet(uint16_t usRegVal, uint8_t uchRegPosi);


/**
 * @fn     uint8_t GH3X2X_GetAndClearElectrodeWearRevCnt(void)
 *
 * @brief  GH3X2X_AddElectrodeWearRevCnt
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  ElectrodeWearRevCnt
 */
extern uint8_t GH3X2X_GetAndClearElectrodeWearRevCnt(void);











#endif /* _GH3X2X_DRV_SOFT_LEAD_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/

