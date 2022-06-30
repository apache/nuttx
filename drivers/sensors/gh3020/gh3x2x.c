/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_demo.c
 *
 * @brief   gh3x2x driver example
 *
 * @author  Gooidx Iot Team
 *
 */

/* includes */
#include "stdint.h"
#include "string.h"
#include "gh3x2x.h"
#include "gh3x2x_config.h"
#include "gh3x2x_inner.h"
#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"

#define GH3X2X_INT_PROCESS_SUCCESS              (0)         /**< gh3x2x interrupt process success */
#define GH3X2X_INT_PROCESS_REPEAT               (1)         /**< need process gh3x2x interrupt process again */


#if (__SUPPORT_HARD_ADT_CONFIG__)
#define __GH3X2X_ADT_EVENT_MASK__               (GH3X2X_IRQ_MSK_WEAR_ON_BIT | GH3X2X_IRQ_MSK_WEAR_OFF_BIT)
#else
#define __GH3X2X_ADT_EVENT_MASK__               (0)
#endif

#if (__FUNC_TYPE_ECG_ENABLE__)
#define __GH3X2X_ECG_EVENT_MASK__               (GH3X2X_IRQ_MSK_LEAD_ON_DET_BIT)
#else
#define __GH3X2X_ECG_EVENT_MASK__               (0)
#endif

/* GH3X2X event that need to process */
#define __GH3X2X_EVENT_PROCESS_MASK__   (\
                                            __GH3X2X_ADT_EVENT_MASK__ | __GH3X2X_ECG_EVENT_MASK__ | \
                                            GH3X2X_IRQ_MSK_FIFO_WATERMARK_BIT | GH3X2X_IRQ_MSK_FIFO_FULL_BIT | \
                                            GH3X2X_IRQ_MSK_TUNNING_FAIL_BIT | GH3X2X_IRQ_MSK_TUNNING_DONE_BIT | \
                                            GH3X2X_IRQ_MSK_CHIP_RESET_BIT \
                                        )
#define DRV_LIB_REG_CFG_EMPTY           (0x00)    //there is no reg config array loaded into driver lib
#define DRV_LIB_REG_CFG_MIN_LEN         (0x5)     //minimum length of reg config array
#define DRV_LIB_DEMO_VERSION_HEAD       "GH(M)3X2X_DRV_LIB_DEMO"
#define DRV_LIB_DEMO_VERSION_NUM        "1.0"
#define DRV_LIB_DEMO_VERSION            (DRV_LIB_DEMO_VERSION_HEAD "_V" DRV_LIB_DEMO_VERSION_NUM)

#if _GS_GYRO_ENABLE_
const uint8_t g_uchGyroEnable = 1;
#else
const uint8_t g_uchGyroEnable = 0;
#endif
/// gsensor fifo index
static uint16_t gsensor_soft_fifo_buffer_index = 0;

/// gsensor fifo
static STGsensorRawdata gsensor_soft_fifo_buffer[__GSENSOR_DATA_BUFFER_SIZE__ + __GS_EXTRA_BUF_LEN__];
#if __CAP_ENABLE__
static uint16_t cap_soft_fifo_buffer_index = 0;
static STCapRawdata cap_soft_fifo_buffer[__CAP_DATA_BUFFER_SIZE__];
#else
static uint16_t cap_soft_fifo_buffer_index = 0;
static STCapRawdata* cap_soft_fifo_buffer = 0;
#endif
#if __TEMP_ENABLE__
static uint16_t temp_soft_fifo_buffer_index = 0;
static STTempRawdata temp_soft_fifo_buffer[__TEMP_DATA_BUFFER_SIZE__];
#else
static uint16_t temp_soft_fifo_buffer_index = 0;
static STTempRawdata* temp_soft_fifo_buffer = 0;
#endif

uint32_t g_unDemoFuncMode = 0;
uint8_t g_uchDemoWorkMode = GH3X2X_DEMO_WORK_MODE_MCU_ONLINE;
uint8_t g_uchDemoAlgoEnableFlag = 1;
#if __SLOT_SEQUENCE_DYNAMIC_ADJUST_EN__
uint16_t g_usGh3x2xSlotTime[8];
#endif
#if (__FUNC_TYPE_SOFT_ADT_ENABLE__)
uint8_t g_uchHardAdtFuncStatus = 0;    //0: stop   1: start
uint8_t g_uchVirtualAdtTimerCtrlStatus = 0;  //0: stop  1: running
#endif

#if (__SUPPORT_ELECTRODE_WEAR_STATUS_DUMP__)
uint8_t g_uchGh3x2xElectrodeWearStatus;
#endif

EMWearDetectType g_uchWearDetectStatus = WEAR_DETECT_WEAR_ON;


#if __ALGO_RUN_SIMULTANEOUSLY_SUPPORT__
#if __FUNC_TYPE_HR_ENABLE__
uint32_t g_pun_HR_FrameIncompleteRawdata[GH3X2X_HR_CHNL_NUM];
uint32_t g_un_HR_FrameIncompleteChnlMapBit;
uint8_t g_puch_HR_FrameLastGain[GH3X2X_HR_CHNL_NUM];
uint32_t g_un_HR_TimeStamp;
STGh3x2xDownSampleInfo g_stGh3x2xDownSampleInfo_HR;
#endif
#if __FUNC_TYPE_HRV_ENABLE__
uint32_t g_pun_HRV_FrameIncompleteRawdata[GH3X2X_HRV_CHNL_NUM];
uint32_t g_un_HRV_FrameIncompleteChnlMapBit;
uint8_t g_puch_HRV_FrameLastGain[GH3X2X_HRV_CHNL_NUM];
uint32_t g_un_HRV_TimeStamp;
STGh3x2xDownSampleInfo g_stGh3x2xDownSampleInfo_HRV;
#endif
#if __FUNC_TYPE_HSM_ENABLE__
uint32_t g_pun_HSM_FrameIncompleteRawdata[GH3X2X_HSM_CHNL_NUM];
uint32_t g_un_HSM_FrameIncompleteChnlMapBit;
uint8_t g_puch_HSM_FrameLastGain[GH3X2X_HSM_CHNL_NUM];
uint32_t g_un_HSM_TimeStamp;
STGh3x2xDownSampleInfo g_stGh3x2xDownSampleInfo_HSM;
#endif
#if __FUNC_TYPE_SPO2_ENABLE__
uint32_t g_pun_SPO2_FrameIncompleteRawdata[GH3X2X_SPO2_CHNL_NUM];
uint32_t g_un_SPO2_FrameIncompleteChnlMapBit;
uint8_t g_puch_SPO2_FrameLastGain[GH3X2X_SPO2_CHNL_NUM];
uint32_t g_un_SPO2_TimeStamp;
STGh3x2xDownSampleInfo g_stGh3x2xDownSampleInfo_SPO2;
#endif
#if __FUNC_TYPE_PWTT_ENABLE__
uint32_t g_pun_PWTT_FrameIncompleteRawdata[GH3X2X_PWTT_CHNL_NUM];
uint32_t g_un_PWTT_FrameIncompleteChnlMapBit;
uint8_t g_puch_PWTT_FrameLastGain[GH3X2X_PWTT_CHNL_NUM];
uint32_t g_un_PWTT_TimeStamp;
STGh3x2xDownSampleInfo g_stGh3x2xDownSampleInfo_PWTT;
#endif
#if __FUNC_TYPE_BT_ENABLE__
uint32_t g_pun_BT_FrameIncompleteRawdata[GH3X2X_BT_CHNL_NUM];
uint32_t g_un_BT_FrameIncompleteChnlMapBit;
uint8_t g_puch_BT_FrameLastGain[GH3X2X_BT_CHNL_NUM];
uint32_t g_un_BT_TimeStamp;
STGh3x2xDownSampleInfo g_stGh3x2xDownSampleInfo_BT;
#endif
#if __FUNC_TYPE_RESP_ENABLE__
uint32_t g_pun_RESP_FrameIncompleteRawdata[GH3X2X_RESP_CHNL_NUM];
uint32_t g_un_RESP_FrameIncompleteChnlMapBit;
uint8_t g_puch_RESP_FrameLastGain[GH3X2X_RESP_CHNL_NUM];
uint32_t g_un_RESP_TimeStamp;
STGh3x2xDownSampleInfo g_stGh3x2xDownSampleInfo_RESP;
#endif
#if __FUNC_TYPE_AF_ENABLE__
uint32_t g_pun_AF_FrameIncompleteRawdata[GH3X2X_AF_CHNL_NUM];
uint32_t g_un_AF_FrameIncompleteChnlMapBit;
uint8_t g_puch_AF_FrameLastGain[GH3X2X_AF_CHNL_NUM];
uint32_t g_un_AF_TimeStamp;
STGh3x2xDownSampleInfo g_stGh3x2xDownSampleInfo_AF;
#endif
#if __FUNC_TYPE_BP_ENABLE__
uint32_t g_pun_BP_FrameIncompleteRawdata[GH3X2X_BP_CHNL_NUM];
uint32_t g_un_BP_FrameIncompleteChnlMapBit;
uint8_t g_puch_BP_FrameLastGain[GH3X2X_BP_CHNL_NUM];
uint32_t g_un_BP_TimeStamp;
STGh3x2xDownSampleInfo g_stGh3x2xDownSampleInfo_BP;
#endif
#if __FUNC_TYPE_TEST_ENABLE__
uint32_t g_pun_TEST1_FrameIncompleteRawdata[GH3X2X_TEST_CHNL_NUM];
uint32_t g_un_TEST1_FrameIncompleteChnlMapBit;
uint8_t g_puch_TEST1_FrameLastGain[GH3X2X_TEST_CHNL_NUM];
uint32_t g_un_TEST1_TimeStamp;
STGh3x2xDownSampleInfo g_stGh3x2xDownSampleInfo_TEST1;
uint32_t g_pun_TEST2_FrameIncompleteRawdata[GH3X2X_TEST_CHNL_NUM];
uint32_t g_un_TEST2_FrameIncompleteChnlMapBit;
uint8_t g_puch_TEST2_FrameLastGain[GH3X2X_TEST_CHNL_NUM];
uint32_t g_un_TEST2_TimeStamp;
STGh3x2xDownSampleInfo g_stGh3x2xDownSampleInfo_TEST2;
#endif
#else
uint32_t g_pun_MAIN_FUNC_FrameIncompleteRawdata[GH3X2X_MAIN_FUNC_CHNL_NUM];
uint32_t g_un_MAIN_FUNC_FrameIncompleteChnlMapBit;
uint8_t g_puch_MAIN_FUNC_FrameLastGain[GH3X2X_MAIN_FUNC_CHNL_NUM];
uint32_t g_un_MAIN_FUNC_TimeStamp;
STGh3x2xDownSampleInfo g_stGh3x2xDownSampleInfo_MAIN_FUNC;
#endif
#if __SUPPORT_HARD_ADT_CONFIG__
uint32_t g_pun_ADT_FrameIncompleteRawdata[GH3X2X_ADT_CHNL_NUM];
uint32_t g_un_ADT_FrameIncompleteChnlMapBit;
uint8_t g_puch_ADT_FrameLastGain[GH3X2X_ADT_CHNL_NUM];
uint32_t g_un_ADT_TimeStamp;
STGh3x2xDownSampleInfo g_stGh3x2xDownSampleInfo_ADT;
#endif
#if __FUNC_TYPE_SOFT_ADT_ENABLE__
uint32_t g_pun_SOFT_ADT_GREEN_FrameIncompleteRawdata[GH3X2X_SOFT_ADT_CHNL_NUM];
uint32_t g_un_SOFT_ADT_GREEN_FrameIncompleteChnlMapBit;
uint8_t g_puch_SOFT_ADT_GREEN_FrameLastGain[GH3X2X_SOFT_ADT_CHNL_NUM];
uint32_t g_un_SOFT_ADT_GREEN_TimeStamp;
STGh3x2xDownSampleInfo g_stGh3x2xDownSampleInfo_SOFT_ADT_GREEN;
#endif

#if __FUNC_TYPE_HR_ENABLE__
uint8_t g_pch_HR_ChnlMap[GH3X2X_HR_CHNL_NUM];
STGh3x2xFunctionInfo g_st_HR_FuncitonInfo;
STGh3x2xAlgoResult g_st_HR_AlgoRecordResult;
#endif
#if __FUNC_TYPE_HRV_ENABLE__
uint8_t g_pch_HRV_ChnlMap[GH3X2X_HRV_CHNL_NUM];
STGh3x2xFunctionInfo g_st_HRV_FuncitonInfo;
STGh3x2xAlgoResult g_st_HRV_AlgoRecordResult;
#endif
#if __FUNC_TYPE_HSM_ENABLE__
uint8_t g_pch_HSM_ChnlMap[GH3X2X_HSM_CHNL_NUM];
STGh3x2xFunctionInfo g_st_HSM_FuncitonInfo;
#endif
#if __FUNC_TYPE_SPO2_ENABLE__
uint8_t g_pch_SPO2_ChnlMap[GH3X2X_SPO2_CHNL_NUM];
STGh3x2xFunctionInfo g_st_SPO2_FuncitonInfo;
STGh3x2xAlgoResult g_st_SPO2_AlgoRecordResult;
#endif
#if __FUNC_TYPE_PWTT_ENABLE__
uint8_t g_pch_PWTT_ChnlMap[GH3X2X_PWTT_CHNL_NUM];
STGh3x2xFunctionInfo g_st_PWTT_FuncitonInfo;
#endif
#if __FUNC_TYPE_BT_ENABLE__
uint8_t g_pch_BT_ChnlMap[GH3X2X_BT_CHNL_NUM];
STGh3x2xFunctionInfo g_st_BT_FuncitonInfo;
#endif
#if __FUNC_TYPE_RESP_ENABLE__
uint8_t g_pch_RESP_ChnlMap[GH3X2X_RESP_CHNL_NUM];
STGh3x2xFunctionInfo g_st_RESP_FuncitonInfo;
#endif
#if __FUNC_TYPE_AF_ENABLE__
uint8_t g_pch_AF_ChnlMap[GH3X2X_AF_CHNL_NUM];
STGh3x2xFunctionInfo g_st_AF_FuncitonInfo;
#endif
#if __SUPPORT_HARD_ADT_CONFIG__
uint8_t g_pch_ADT_ChnlMap[GH3X2X_ADT_CHNL_NUM];
STGh3x2xFunctionInfo g_st_ADT_FuncitonInfo;
#endif
#if __FUNC_TYPE_SOFT_ADT_ENABLE__
uint8_t g_pch_SOFT_ADT_GREEN_ChnlMap[GH3X2X_SOFT_ADT_CHNL_NUM];
STGh3x2xFunctionInfo g_st_SOFT_ADT_GREEN_FuncitonInfo;
#endif
#if __FUNC_TYPE_SOFT_ADT_ENABLE__
uint8_t g_pch_SOFT_ADT_IR_ChnlMap[GH3X2X_SOFT_ADT_CHNL_NUM];
STGh3x2xFunctionInfo g_st_SOFT_ADT_IR_FuncitonInfo;
#endif
#if __FUNC_TYPE_BP_ENABLE__
uint8_t g_pch_BP_ChnlMap[GH3X2X_BP_CHNL_NUM];
STGh3x2xFunctionInfo g_st_BP_FuncitonInfo;
#endif
#if __FUNC_TYPE_TEST_ENABLE__
uint8_t g_pch_TEST1_ChnlMap[GH3X2X_TEST_CHNL_NUM];
STGh3x2xFunctionInfo g_st_TEST1_FuncitonInfo;
uint8_t g_pch_TEST2_ChnlMap[GH3X2X_TEST_CHNL_NUM];
STGh3x2xFunctionInfo g_st_TEST2_FuncitonInfo;
#endif

uint32_t g_punGh3x2xFrameRawdata[GH3X2X_FUNC_CHNL_NUM_MAX];
#if _GS_GYRO_ENABLE_
int16_t g_psGh3x2xFrameGsensorData[6];
#else
int16_t g_psGh3x2xFrameGsensorData[3];
#endif
uint32_t g_punGh3x2xFrameAgcInfo[GH3X2X_FUNC_CHNL_NUM_MAX];
uint32_t g_punGh3x2xFrameFlag[8];
STGh3x2xAlgoResult g_stGh3x2xAlgoResult;

STCapRawdata g_pstGh3x2xFrameCapData;
STTempRawdata g_pstGh3x2xFrameTempData;

/* Move from old gh3x2x_protocol.c here */

STGh3x2xProtocolData * const g_pstGh3x2xProtocolData = 0;


//HR data info
#if __FUNC_TYPE_HR_ENABLE__
const struct gh3020_frameinfo_s g_stHR_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_HR,
    .pstFunctionInfo = &g_st_HR_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_HR_CHNL_NUM,
    .pchChnlMap = g_pch_HR_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .pstAlgoRecordResult = &g_st_HR_AlgoRecordResult,
#if !__ALGO_RUN_SIMULTANEOUSLY_SUPPORT__
    .punIncompleteRawdata = g_pun_MAIN_FUNC_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_MAIN_FUNC_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_MAIN_FUNC_FrameLastGain,
    .punFrameCnt = &g_un_MAIN_FUNC_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_MAIN_FUNC,
#else
    .punIncompleteRawdata = g_pun_HR_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_HR_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_HR_FrameLastGain,
    .punFrameCnt = &g_un_HR_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_HR,
#endif
};
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xHR_FrameInfo = &g_stHR_FrameInfo;
#else
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xHR_FrameInfo = 0;
#endif
//HRV data info
#if __FUNC_TYPE_HRV_ENABLE__
const struct gh3020_frameinfo_s g_stHRV_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_HRV,
    .pstFunctionInfo = &g_st_HRV_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_HRV_CHNL_NUM,
    .pchChnlMap = g_pch_HRV_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .pstAlgoRecordResult = &g_st_HRV_AlgoRecordResult,
#if !__ALGO_RUN_SIMULTANEOUSLY_SUPPORT__
    .punIncompleteRawdata = g_pun_MAIN_FUNC_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_MAIN_FUNC_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_MAIN_FUNC_FrameLastGain,
    .punFrameCnt = &g_un_MAIN_FUNC_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_MAIN_FUNC,
#else
    .punIncompleteRawdata = g_pun_HRV_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_HRV_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_HRV_FrameLastGain,
    .punFrameCnt = &g_un_HRV_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_HRV,
#endif
};
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xHRV_FrameInfo = &g_stHRV_FrameInfo;
#else
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xHRV_FrameInfo = 0;
#endif
//HSM data info
#if __FUNC_TYPE_HSM_ENABLE__
const struct gh3020_frameinfo_s g_stHSM_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_HSM,
    .pstFunctionInfo = &g_st_HSM_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_HSM_CHNL_NUM,
    .pchChnlMap = g_pch_HSM_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .pstAlgoRecordResult = 0,
#if !__ALGO_RUN_SIMULTANEOUSLY_SUPPORT__
    .punIncompleteRawdata = g_pun_MAIN_FUNC_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_MAIN_FUNC_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_MAIN_FUNC_FrameLastGain,
    .punFrameCnt = &g_un_MAIN_FUNC_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_MAIN_FUNC,
#else
    .punIncompleteRawdata = g_pun_HSM_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_HSM_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_HSM_FrameLastGain,
    .punFrameCnt = &g_un_HSM_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_HSM,
#endif
};
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xHSM_FrameInfo = &g_stHSM_FrameInfo;
#else
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xHSM_FrameInfo = 0;
#endif
//SPO2 data info
#if __FUNC_TYPE_SPO2_ENABLE__
const struct gh3020_frameinfo_s g_stSPO2_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_SPO2,
    .pstFunctionInfo = &g_st_SPO2_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_SPO2_CHNL_NUM,
    .pchChnlMap = g_pch_SPO2_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .pstAlgoRecordResult = &g_st_SPO2_AlgoRecordResult,
#if !__ALGO_RUN_SIMULTANEOUSLY_SUPPORT__
    .punIncompleteRawdata = g_pun_MAIN_FUNC_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_MAIN_FUNC_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_MAIN_FUNC_FrameLastGain,
    .punFrameCnt = &g_un_MAIN_FUNC_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_MAIN_FUNC,
#else
    .punIncompleteRawdata = g_pun_SPO2_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_SPO2_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_SPO2_FrameLastGain,
    .punFrameCnt = &g_un_SPO2_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_SPO2,
#endif
};
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xSPO2_FrameInfo = &g_stSPO2_FrameInfo;
#else
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xSPO2_FrameInfo = 0;
#endif

#if __FUNC_TYPE_PWTT_ENABLE__
const struct gh3020_frameinfo_s g_stPWTT_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_PWTT,
    .pstFunctionInfo = &g_st_PWTT_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_PWTT_CHNL_NUM,
    .pchChnlMap = g_pch_PWTT_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .pstAlgoRecordResult = 0,
#if !__ALGO_RUN_SIMULTANEOUSLY_SUPPORT__
    .punIncompleteRawdata = g_pun_MAIN_FUNC_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_MAIN_FUNC_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_MAIN_FUNC_FrameLastGain,
    .punFrameCnt = &g_un_MAIN_FUNC_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_MAIN_FUNC,
#else
    .punIncompleteRawdata = g_pun_PWTT_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_PWTT_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_PWTT_FrameLastGain,
    .punFrameCnt = &g_un_PWTT_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_PWTT,
#endif
};
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xECG_FrameInfo = &g_stECG_FrameInfo;
#else
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xECG_FrameInfo = 0;
#endif
//BT data info
#if __FUNC_TYPE_BT_ENABLE__
const struct gh3020_frameinfo_s g_stBT_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_BT,
    .pstFunctionInfo = &g_st_BT_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_BT_CHNL_NUM,
    .pchChnlMap = g_pch_BT_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .pstAlgoRecordResult = 0,
#if !__ALGO_RUN_SIMULTANEOUSLY_SUPPORT__
    .punIncompleteRawdata = g_pun_MAIN_FUNC_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_MAIN_FUNC_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_MAIN_FUNC_FrameLastGain,
    .punFrameCnt = &g_un_MAIN_FUNC_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_MAIN_FUNC,
#else
    .punIncompleteRawdata = g_pun_BT_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_BT_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_BT_FrameLastGain,
    .punFrameCnt = &g_un_BT_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_BT,
#endif
};
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xBT_FrameInfo = &g_stBT_FrameInfo;
#else
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xBT_FrameInfo = 0;
#endif
//RESP data info
#if __FUNC_TYPE_RESP_ENABLE__
const struct gh3020_frameinfo_s g_stRESP_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_RESP,
    .pstFunctionInfo = &g_st_RESP_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_RESP_CHNL_NUM,
    .pchChnlMap = g_pch_RESP_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .pstAlgoRecordResult = 0,
#if !__ALGO_RUN_SIMULTANEOUSLY_SUPPORT__
    .punIncompleteRawdata = g_pun_MAIN_FUNC_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_MAIN_FUNC_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_MAIN_FUNC_FrameLastGain,
    .punFrameCnt = &g_un_MAIN_FUNC_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_MAIN_FUNC,
#else
    .punIncompleteRawdata = g_pun_RESP_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_RESP_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_RESP_FrameLastGain,
    .punFrameCnt = &g_un_RESP_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_RESP,
#endif
};
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xRESP_FrameInfo = &g_stRESP_FrameInfo;
#else
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xRESP_FrameInfo = 0;
#endif
//AF data info
#if __FUNC_TYPE_AF_ENABLE__
const struct gh3020_frameinfo_s g_stAF_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_AF,
    .pstFunctionInfo = &g_st_AF_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_AF_CHNL_NUM,
    .pchChnlMap = g_pch_AF_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .pstAlgoRecordResult = 0,
#if !__ALGO_RUN_SIMULTANEOUSLY_SUPPORT__
    .punIncompleteRawdata = g_pun_MAIN_FUNC_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_MAIN_FUNC_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_MAIN_FUNC_FrameLastGain,
    .punFrameCnt = &g_un_MAIN_FUNC_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_MAIN_FUNC,
#else
    .punIncompleteRawdata = g_pun_AF_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_AF_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_AF_FrameLastGain,
    .punFrameCnt = &g_un_AF_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_AF,
#endif
};
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xAF_FrameInfo = &g_stAF_FrameInfo;
#else
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xAF_FrameInfo = 0;
#endif
#if __FUNC_TYPE_BP_ENABLE__
const struct gh3020_frameinfo_s g_stFPBP_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_FPBP,
    .pstFunctionInfo = &g_st_BP_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_BP_CHNL_NUM,
    .pchChnlMap = g_pch_BP_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .pstAlgoRecordResult = 0,
#if !__ALGO_RUN_SIMULTANEOUSLY_SUPPORT__
    .punIncompleteRawdata = g_pun_MAIN_FUNC_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_MAIN_FUNC_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_MAIN_FUNC_FrameLastGain,
    .punFrameCnt = &g_un_MAIN_FUNC_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_MAIN_FUNC,
#else
    .punIncompleteRawdata = g_pun_BP_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_BP_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_BP_FrameLastGain,
    .punFrameCnt = &g_un_BP_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_BP,
#endif
};
const struct gh3020_frameinfo_s g_stPWA_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_PWA,
    .pstFunctionInfo = &g_st_BP_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_PWA_CHNL_NUM,
    .pchChnlMap = g_pch_BP_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .pstAlgoRecordResult = 0,
#if !__ALGO_RUN_SIMULTANEOUSLY_SUPPORT__
    .punIncompleteRawdata = g_pun_MAIN_FUNC_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_MAIN_FUNC_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_MAIN_FUNC_FrameLastGain,
    .punFrameCnt = &g_un_MAIN_FUNC_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_MAIN_FUNC,
#else
    .punIncompleteRawdata = g_pun_BP_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_BP_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_BP_FrameLastGain,
    .punFrameCnt = &g_un_BP_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_BP,
#endif
};
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xFPBP_FrameInfo = &g_stFPBP_FrameInfo;
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xPWA_FrameInfo = &g_stPWA_FrameInfo;
#else
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xFPBP_FrameInfo = 0;
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xPWA_FrameInfo = 0;
#endif
#if __FUNC_TYPE_TEST_ENABLE__
const struct gh3020_frameinfo_s g_stTEST1_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_TEST1,
    .pstFunctionInfo = &g_st_TEST1_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_TEST_CHNL_NUM,
    .pchChnlMap = g_pch_TEST1_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .pstAlgoRecordResult = 0,
#if !__ALGO_RUN_SIMULTANEOUSLY_SUPPORT__
    .punIncompleteRawdata = g_pun_MAIN_FUNC_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_MAIN_FUNC_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_MAIN_FUNC_FrameLastGain,
    .punFrameCnt = &g_un_MAIN_FUNC_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_MAIN_FUNC,
#else
    .punIncompleteRawdata = g_pun_TEST1_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_TEST1_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_TEST1_FrameLastGain,
    .punFrameCnt = &g_un_TEST1_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_TEST1,
#endif
};
const struct gh3020_frameinfo_s g_stTEST2_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_TEST2,
    .pstFunctionInfo = &g_st_TEST2_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_TEST_CHNL_NUM,
    .pchChnlMap = g_pch_TEST2_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .pstAlgoRecordResult = 0,
#if !__ALGO_RUN_SIMULTANEOUSLY_SUPPORT__
    .punIncompleteRawdata = g_pun_MAIN_FUNC_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_MAIN_FUNC_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_MAIN_FUNC_FrameLastGain,
    .punFrameCnt = &g_un_MAIN_FUNC_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_MAIN_FUNC,
#else
    .punIncompleteRawdata = g_pun_TEST2_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_TEST2_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_TEST2_FrameLastGain,
    .punFrameCnt = &g_un_TEST2_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_TEST2,
#endif
};
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xTEST1_FrameInfo = &g_stTEST1_FrameInfo;
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xTEST2_FrameInfo = &g_stTEST2_FrameInfo;
#else
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xTEST1_FrameInfo = 0;
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xTEST2_FrameInfo = 0;
#endif
//ADT data info
#if __SUPPORT_HARD_ADT_CONFIG__
const struct gh3020_frameinfo_s g_stADT_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_ADT,
    .pstFunctionInfo = &g_st_ADT_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_ADT_CHNL_NUM,
    .pchChnlMap = g_pch_ADT_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .punIncompleteRawdata = g_pun_ADT_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_ADT_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_ADT_FrameLastGain,
    .punFrameCnt = &g_un_ADT_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_ADT,
    .pstAlgoRecordResult = 0,
};
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xADT_FrameInfo = &g_stADT_FrameInfo;
#else
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xADT_FrameInfo = 0;
#endif
//SOFT ADT data info
#if __FUNC_TYPE_SOFT_ADT_ENABLE__
const struct gh3020_frameinfo_s g_stSOFT_ADT_GREEN_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_SOFT_ADT_GREEN,
    .pstFunctionInfo = &g_st_SOFT_ADT_GREEN_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_SOFT_ADT_CHNL_NUM,
    .pchChnlMap = g_pch_SOFT_ADT_GREEN_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .punIncompleteRawdata = g_pun_SOFT_ADT_GREEN_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_SOFT_ADT_GREEN_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_SOFT_ADT_GREEN_FrameLastGain,
    .punFrameCnt = &g_un_SOFT_ADT_GREEN_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_SOFT_ADT_GREEN,
    .pstAlgoRecordResult = 0,
};
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xSOFT_ADT_GREEN_FrameInfo = &g_stSOFT_ADT_GREEN_FrameInfo;
#else
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xSOFT_ADT_GREEN_FrameInfo = 0;
#endif
#if __FUNC_TYPE_SOFT_ADT_ENABLE__
const struct gh3020_frameinfo_s g_stSOFT_ADT_IR_FrameInfo =
{
    .unFunctionID = GH3X2X_FUNCTION_SOFT_ADT_IR,
    .pstFunctionInfo = &g_st_SOFT_ADT_IR_FuncitonInfo,
    .pstAlgoResult   = &g_stGh3x2xAlgoResult,
    .uchFuntionChnlLimit = GH3X2X_SOFT_ADT_CHNL_NUM,
    .pchChnlMap = g_pch_SOFT_ADT_IR_ChnlMap,
    .punFrameRawdata = g_punGh3x2xFrameRawdata,
    .pusFrameGsensordata = g_psGh3x2xFrameGsensorData,
    .pstFrameCapdata = &g_pstGh3x2xFrameCapData,
    .pstFrameTempdata = &g_pstGh3x2xFrameTempData,
    .punFrameAgcInfo = g_punGh3x2xFrameAgcInfo,
    .punFrameFlag = g_punGh3x2xFrameFlag,
    .punIncompleteRawdata = g_pun_SOFT_ADT_GREEN_FrameIncompleteRawdata,
    .punIncompleteChnlMapBit = &g_un_SOFT_ADT_GREEN_FrameIncompleteChnlMapBit,
    .puchFrameLastGain = g_puch_SOFT_ADT_GREEN_FrameLastGain,
    .punFrameCnt = &g_un_SOFT_ADT_GREEN_TimeStamp,
    .pstDownSampleInfo = &g_stGh3x2xDownSampleInfo_SOFT_ADT_GREEN,
    .pstAlgoRecordResult = 0,
};
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xSOFT_ADT_IR_FrameInfo = &g_stSOFT_ADT_IR_FrameInfo;
#else
//const struct gh3020_frameinfo_s * const  g_pstGh3x2xSOFT_ADT_IR_FrameInfo = 0;
#endif


//Function data info struct
const struct gh3020_frameinfo_s * const  g_pstGh3x2xFrameInfo[GH3X2X_FUNC_OFFSET_MAX] =
{
//adt
#if __SUPPORT_HARD_ADT_CONFIG__
    &g_stADT_FrameInfo,
#else
    0,
#endif
//hr
#if __FUNC_TYPE_HR_ENABLE__
    &g_stHR_FrameInfo,
#else
    0,
#endif
//hrv
#if __FUNC_TYPE_HRV_ENABLE__
    &g_stHRV_FrameInfo,
#else
    0,
#endif
//hsm
#if __FUNC_TYPE_HSM_ENABLE__
    &g_stHSM_FrameInfo,
#else
    0,
#endif
//fpbp
//pwa
#if __FUNC_TYPE_BP_ENABLE__
    &g_stFPBP_FrameInfo,
    &g_stPWA_FrameInfo,
#else
    0,
    0,
#endif
//spo2
#if __FUNC_TYPE_SPO2_ENABLE__
    &g_stSPO2_FrameInfo,
#else
    0,
#endif
//ecg
#if __FUNC_TYPE_ECG_ENABLE__
    &g_stECG_FrameInfo,
#else
    0,
#endif
//pwtt
#if __FUNC_TYPE_PWTT_ENABLE__
    &g_stPWTT_FrameInfo,
#else
    0,
#endif
//soft adt green
#if __FUNC_TYPE_SOFT_ADT_ENABLE__
    &g_stSOFT_ADT_GREEN_FrameInfo,
#else
    0,
#endif
//bt
#if __FUNC_TYPE_BT_ENABLE__
    &g_stBT_FrameInfo,
#else
    0,
#endif
//resp
#if __FUNC_TYPE_RESP_ENABLE__
    &g_stRESP_FrameInfo,
#else
    0,
#endif
//af
#if __FUNC_TYPE_AF_ENABLE__
    &g_stAF_FrameInfo,
#else
    0,
#endif
#if __FUNC_TYPE_TEST_ENABLE__
    &g_stTEST1_FrameInfo,
    &g_stTEST2_FrameInfo,
#else
    0,
    0,
#endif
//soft adt ir
#if __FUNC_TYPE_SOFT_ADT_ENABLE__
    &g_stSOFT_ADT_IR_FrameInfo,
#else
    0,
#endif
};

/// read data buffer
static uint8_t g_uchGh3x2xReadRawdataBuffer[__GH3X2X_RAWDATA_BUFFER_SIZE__ + __GH3X2X_RAWDATA_BUFFER_CRC_SIZE__];

/// read data buffer len
uint16_t g_usGh3x2xReadRawdataLen = 0;


extern void Gh3x2xDemoStopSamplingInner(uint32_t unFuncMode);
extern void Gh3x2xDemoStartSamplingInner(uint32_t unFuncMode);



#if (__DRIVER_LIB_MODE__==__DRV_LIB_WITHOUT_ALGO__)
/// data buffer for algorithm
uint32_t g_uchGh3x2xDemoBufferForAlgo[__GH3X2X_RAWDATA_BUFFER_SIZE__ / 4];

#if (__FUNC_TYPE_HR_ENABLE__)
/// hba incomplete data mark
uint32_t g_unDemoHbaIncompleteChMark = 0;

/// hba incomplete data last time
uint32_t g_unDemoHbaIncompleteRawDataArr[32] = {0};
#endif

#if (__FUNC_TYPE_SPO2_ENABLE__)
/// spo2 incomplete data mark
uint32_t g_unDemoSpo2IncompleteChMark = 0;

/// spo2 incomplete data last time
uint32_t g_unDemoSpo2IncompleteRawDataArr[32] = {0};
#endif

#if (__FUNC_TYPE_HRV_ENABLE__)
/// hrv incomplete data mark
uint32_t g_unDemoHrvIncompleteChMark = 0;

/// hba incomplete data last time
uint32_t g_unDemoHrvIncompleteRawDataArr[32] = {0};
#endif
#endif

/// driver lib demo init flag
static uint8_t g_uchGh3x2xInitFlag = 0;

#if (__NORMAL_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__ || __MIX_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__)
//hal_gh3x2x_int_handler_call_back is called, this value is 1
uint8_t g_uchGh3x2xIntCallBackIsCalled = 0;
#endif

#ifdef CONFIG_FACTEST_SENSORS_GH3020
uint8_t g_uchEngineeringModeStatus = 0;  // 0: is not in engineering mode   1: is in engineering mode
FAR struct gh3020_factestmode_param_s *g_pstSampleParaGroup = 0;
uint8_t g_uchEngineeringModeSampleParaGroupNum = 0;
#endif

/// current gh3x2x reg config array index
uint8_t g_uchGh3x2xRegCfgArrIndex = DRV_LIB_REG_CFG_EMPTY;

/// int mode
uint8_t g_uchGh3x2xIntMode = __INTERRUPT_PROCESS_MODE__;

#if __SUPPORT_SOFT_AGC_CONFIG__
//main chnl
STGh3x2xNewAgcMainChnlInfo  g_pstGh3x2xNewAgcMainChnlInfoEntity[GH3X2X_NEW_AGC_SLOT_NUM_LIMIT];
STGh3x2xNewAgcMainChnlInfo * const g_pstGh3x2xNewAgcMainChnlInfo = g_pstGh3x2xNewAgcMainChnlInfoEntity;
const uint8_t g_uchNewAgcSlotNumLimit = GH3X2X_NEW_AGC_SLOT_NUM_LIMIT;
//sub chnl
#if GH3X2X_NEW_AGC_SUB_CHNL_NUM_LIMIT > 0
STGh3x2xNewAgcSubChnlSlotInfo  g_pstGh3x2xNewAgcSubChnlSlotInfoEntity[GH3X2X_NEW_AGC_SLOT_NUM_LIMIT];
STGh3x2xNewAgcSubChnlRxInfo    g_pstGh3x2xNewAgcSubChnlRxInfoEntity[GH3X2X_NEW_AGC_SUB_CHNL_NUM_LIMIT];
STGh3x2xNewAgcSubChnlSlotInfo * const g_pstGh3x2xNewAgcSubChnlSlotInfo = g_pstGh3x2xNewAgcSubChnlSlotInfoEntity;
STGh3x2xNewAgcSubChnlRxInfo  * const g_pstGh3x2xNewAgcSubChnlRxInfo = g_pstGh3x2xNewAgcSubChnlRxInfoEntity;
#else
STGh3x2xNewAgcSubChnlSlotInfo  * const g_pstGh3x2xNewAgcSubChnlSlotInfo = 0;
STGh3x2xNewAgcSubChnlRxInfo  * const g_pstGh3x2xNewAgcSubChnlRxInfo = 0;
#endif
const uint8_t g_uchNewAgcSubChnlNumLimit = GH3X2X_NEW_AGC_SUB_CHNL_NUM_LIMIT;
#else
STGh3x2xNewAgcMainChnlInfo * const g_pstGh3x2xNewAgcMainChnlInfo = 0;
STGh3x2xNewAgcSubChnlSlotInfo * const g_pstGh3x2xNewAgcSubChnlSlotInfo = 0;
STGh3x2xNewAgcSubChnlRxInfo * const g_pstGh3x2xNewAgcSubChnlRxInfo = 0;
const uint8_t g_uchNewAgcSlotNumLimit = 0;
const uint8_t g_uchNewAgcSubChnlNumLimit = 0;
#endif









#if __GS_NONSYNC_READ_EN__
uint8_t g_uchGh3x2xGsenosrExtralBufLen = __GS_EXTRA_BUF_LEN__;
int16_t g_sGh3x2xGsenorCurrentRemainPointNum;
uint16_t g_usGsenorRefFunction = 0;
float g_fGh3x2xNonSyncGsensorStep;


void Gh3x2xNonSyncGsensorInit(void)
{
    g_sGh3x2xGsenorCurrentRemainPointNum = 0;
    EXAMPLE_LOG("[Non-Sync Gsensor]non-sync gsensor init.\r\n");
}

uint16_t Gh3x2xGetCurrentRefFunction(void)
{
    uint16_t usSampleRateMax = 0;
    uint16_t usRefFucntion = 0;
    //find max sample rate function in opened function
    for(uint8_t uchFunCnt = 0; uchFunCnt < GH3X2X_FUNC_OFFSET_MAX; uchFunCnt ++)
    {
        if(g_pstGh3x2xFrameInfo[uchFunCnt])
        {
            if(g_unDemoFuncMode & (((uint32_t)1)<< uchFunCnt))
            {
                if(usSampleRateMax < g_pstGh3x2xFrameInfo[uchFunCnt]->pstFunctionInfo->usSampleRate)
                {
                    usSampleRateMax = g_pstGh3x2xFrameInfo[uchFunCnt]->pstFunctionInfo->usSampleRate;
                    usRefFucntion = g_pstGh3x2xFrameInfo[uchFunCnt]->unFunctionID;
                }
            }
        }
    }

    EXAMPLE_LOG("[Non-Sync Gsensor]usRefFucntion = 0x%X.\r\n",usRefFucntion);
    //calculate step
    if(usSampleRateMax)
    {
        g_fGh3x2xNonSyncGsensorStep = __GS_SAMPLE_RATE_HZ__ / usSampleRateMax;
    }
    else
    {
        g_fGh3x2xNonSyncGsensorStep = 0;
        EXAMPLE_LOG("[Non-Sync Gsensor]Error !!! usSampleRateMax = 0.\r\n");
    }
    EXAMPLE_LOG("[Non-Sync Gsensor]g_fGh3x2xNonSyncGsensorStep = %.3f.\r\n",g_fGh3x2xNonSyncGsensorStep);

    EXAMPLE_LOG("[Non-Sync Gsensor]usRefFucntion = 0x%d,usSampleRateMax = %d.\r\n",usRefFucntion,usSampleRateMax);
    return usRefFucntion;
}

uint16_t Gh3x2xGetGsensorNeedPointNum(uint16_t usCurrentGsenosrPoint,uint8_t* puchReadFifoBuffer, uint16_t usFifoBuffLen)
{
    uint8_t uchFunctionOfst = 0xFF;
    uint16_t usFrameNum = 0;
    uint16_t usTempGsensorPointNum;
    for(uint8_t uchFunCnt = 0; uchFunCnt < GH3X2X_FUNC_OFFSET_MAX; uchFunCnt ++)
    {
        if (g_usGsenorRefFunction == (((uint32_t)1)<< uchFunCnt))
        {
            uchFunctionOfst = uchFunCnt;
            break;
        }
    }
    if(0xFF == uchFunctionOfst)
    {
        return 0;
    }

    //get ref function frame num in this fifo data
    if(g_pstGh3x2xFrameInfo[uchFunctionOfst])
    {
        usFrameNum = GH3x2xGetFrameNum(puchReadFifoBuffer,usFifoBuffLen,g_pstGh3x2xFrameInfo[uchFunctionOfst]);
    }



    usTempGsensorPointNum = ((uint16_t)((float)usFrameNum * g_fGh3x2xNonSyncGsensorStep));
    if(usTempGsensorPointNum > (__GS_EXTRA_BUF_LEN__ + usCurrentGsenosrPoint))
    {
        usTempGsensorPointNum = __GS_EXTRA_BUF_LEN__ + usCurrentGsenosrPoint;
    }


    EXAMPLE_LOG("[Non-Sync Gsensor]GsensorNeedPointNum = %d.\r\n",usTempGsensorPointNum);

    return usTempGsensorPointNum;
}

int16_t Gh3x2xGetGsensorHeadPos(uint16_t usCurrentGsenosrPoint,uint16_t usNeedPoinNum)
{
    int16_t sPos1;
    int16_t sPos2;
    EXAMPLE_LOG("[Non-Sync Gsensor]usCurrentGsenosrPoint = %d.\r\n",usCurrentGsenosrPoint);
    sPos1 = (int16_t)usCurrentGsenosrPoint - (int16_t)usNeedPoinNum;
    sPos2= (int16_t)0 - g_sGh3x2xGsenorCurrentRemainPointNum;

    if(sPos1 < sPos2)
    {
        int16_t sOldestPosi;
        uint8_t  uchExistOldData = 1;
        int16_t sDesPosi = sPos1;
        if(g_sGh3x2xGsenorCurrentRemainPointNum) //exist remain data
        {
            sOldestPosi = (int16_t)0 - (int16_t)g_sGh3x2xGsenorCurrentRemainPointNum;
        }
        else if(usCurrentGsenosrPoint)   //use new gsensor data
        {
            sOldestPosi = 0;
        }
        else
        {
            uchExistOldData = 0;
        }
        EXAMPLE_LOG("[Non-Sync Gsensor]Gsensor data is not enough,add oldest point num = %d, uchExistOldData = %d.****\r\n",sPos2 - sPos1,uchExistOldData);
        EXAMPLE_LOG("[Non-Sync Gsensor]GsensorHeadPos = %d.\r\n",sPos1);
        if(uchExistOldData)
        {
            for(sDesPosi = sPos1; sDesPosi < sOldestPosi;  sDesPosi++)  //copy oldest point value
            {
                memcpy((uint8_t*)(gsensor_soft_fifo_buffer + __GS_EXTRA_BUF_LEN__ + sDesPosi),(uint8_t*)(gsensor_soft_fifo_buffer + __GS_EXTRA_BUF_LEN__ + sOldestPosi),sizeof(STGsensorRawdata));
            }
        }
        else  //copy 0
        {
            memset((uint8_t*)gsensor_soft_fifo_buffer+ __GS_EXTRA_BUF_LEN__ + sDesPosi,0, usNeedPoinNum * sizeof(STGsensorRawdata));
        }

        return sPos1;
    }
    EXAMPLE_LOG("[Non-Sync Gsensor]GsensorHeadPos = %d.\r\n",sPos2);
    return sPos2;
}

void Gh3x2xNonSyncGsenosrPostProcess(uint16_t usCurrentGsenosrPoint,uint16_t usNeedPoinNum ,int16_t sHeadPos)
{
    int16_t sRemainPointSrcStartPosi;
    int16_t sRemainPointDesStartPosi;
    uint16_t usRemainPoinNum;
    sRemainPointSrcStartPosi = (int16_t)usNeedPoinNum + sHeadPos;
    usRemainPoinNum = usCurrentGsenosrPoint - sRemainPointSrcStartPosi;
    EXAMPLE_LOG("[Non-Sync Gsensor]raw usRemainPoinNum = %d.\r\n",usRemainPoinNum);
    if(usRemainPoinNum > __GS_EXTRA_BUF_LEN__)
    {

        EXAMPLE_LOG("[Non-Sync Gsensor]******Gsensor data is too much, give up oldest point num = %d.****************\r\n",(usRemainPoinNum - __GS_EXTRA_BUF_LEN__));
        sRemainPointSrcStartPosi += (usRemainPoinNum - __GS_EXTRA_BUF_LEN__);  //give up oldest data
        usRemainPoinNum = __GS_EXTRA_BUF_LEN__;
    }
    sRemainPointDesStartPosi = ((int16_t)0) - usRemainPoinNum;
    //copy remain data to extral buf
    if(usRemainPoinNum)
    {
        memcpy((uint8_t*)(gsensor_soft_fifo_buffer + __GS_EXTRA_BUF_LEN__ + sRemainPointDesStartPosi),(uint8_t*)(gsensor_soft_fifo_buffer + __GS_EXTRA_BUF_LEN__ + sRemainPointSrcStartPosi),usRemainPoinNum*sizeof(STGsensorRawdata));
    }
    //update g_sGh3x2xGsenorCurrentRemainPointNum
    g_sGh3x2xGsenorCurrentRemainPointNum = usRemainPoinNum;
    EXAMPLE_LOG("[Non-Sync Gsensor]GsenosrPostProcess: RemainPointNum= %d.\r\n",g_sGh3x2xGsenorCurrentRemainPointNum);
}
#endif

#if (__SUPPORT_HARD_ADT_CONFIG__)
/**
 * @fn     void Gh3x2xDemoWearEventProcess(u16* usEvent,EMWearDetectForceSwitch emForceSwitch)
 *
 * @brief  Process wear on/off event
 *
 * @attention   None
 *
 * @param[in]   usEvent             gh3x2x irq status
 * @param[in]   emForceSwitch       switch wear detect status by force or not
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2xDemoWearEventProcess(uint16_t* usEvent,EMWearDetectForceSwitch emForceSwitch)
{
#if __FUNC_TYPE_SOFT_ADT_ENABLE__
    if (g_uchHardAdtFuncStatus == 0)
    {
        *usEvent &= ~(GH3X2X_IRQ_MSK_WEAR_OFF_BIT|GH3X2X_IRQ_MSK_WEAR_ON_BIT);
        return;
    }
#endif
    if ((g_unDemoFuncMode & GH3X2X_FUNCTION_ADT) == GH3X2X_FUNCTION_ADT)
    {
        if((0 != (*usEvent & GH3X2X_IRQ_MSK_WEAR_ON_BIT)) || (0 != (*usEvent & GH3X2X_IRQ_MSK_WEAR_OFF_BIT)))
        {
            if(0 != (*usEvent & GH3X2X_IRQ_MSK_WEAR_ON_BIT))
            {
                g_uchWearDetectStatus = WEAR_DETECT_WEAR_OFF;
                if(GH3X2X_RET_OK != GH3X2X_WearDetectSwitchTo(WEAR_DETECT_WEAR_OFF,emForceSwitch))
                {
                    *usEvent &= ~GH3X2X_IRQ_MSK_WEAR_ON_BIT;
                }
                else
                {
                    EXAMPLE_LOG("[%s]:switch to wear off\r\n", __FUNCTION__);
                }
            }
            else
            {
                g_uchWearDetectStatus = WEAR_DETECT_WEAR_ON;
                if(GH3X2X_RET_OK != GH3X2X_WearDetectSwitchTo(WEAR_DETECT_WEAR_ON,emForceSwitch))
                {
                    *usEvent &= ~GH3X2X_IRQ_MSK_WEAR_OFF_BIT;
                }
                else
                {
                    EXAMPLE_LOG("[%s]:switch to wear on\r\n", __FUNCTION__);
                }

            }
        }
    }
    else
    {
        *usEvent &= ~(GH3X2X_IRQ_MSK_WEAR_OFF_BIT|GH3X2X_IRQ_MSK_WEAR_ON_BIT);
    }
}
#endif

#if (__SUPPORT_PROTOCOL_ANALYZE__)
char* GetDrvLibDemoVersion(void)
{
#ifdef GOODIX_DEMO_PLANFORM
    return GetEvkVersion();
#else
    return DRV_LIB_DEMO_VERSION;
#endif
}
#endif

#if (__SUPPORT_PROTOCOL_ANALYZE__)
char* GetEvkBootloaderVersion(void)
{
#ifdef GOODIX_DEMO_PLANFORM
    return GetBootloaderVersion();
#else
    return DRV_LIB_DEMO_VERSION;
#endif
}
#endif

/**
 * @fn     void Gh3x2x_NormalizeGsensorSensitivity(STGsensorRawdata gsensor_buffer[], uint16_t gsensor_point_num)
 *
 * @brief  normalize gsensor sensitivity to  512LSB/g
 *
 * @attention   None
 *
 * @param[in]    gsensor_buffer          gsensor rawdata buffer
 * @param[in]   gsensor_point_num     gsensor rawdata point num
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_NormalizeGsensorSensitivity(STGsensorRawdata gsensor_buffer[], uint16_t gsensor_point_num)
{
    uint8_t uchRightShift;
    uchRightShift = __GS_SENSITIVITY_CONFIG__;
    for(uint16_t uchCnt = 0; uchCnt < gsensor_point_num; uchCnt++)
    {
        gsensor_buffer[uchCnt].sXAxisVal >>= uchRightShift;
        gsensor_buffer[uchCnt].sYAxisVal >>= uchRightShift;
        gsensor_buffer[uchCnt].sZAxisVal >>= uchRightShift;
    }
}


void Gh3x2xFunctionCtrlModuleInit(void)
{
    g_unDemoFuncMode = 0;
#if (__FUNC_TYPE_SOFT_ADT_ENABLE__)
    g_uchHardAdtFuncStatus = 0;
#endif
}
/**
 * @fn     void gh3020_samplerate_set(uint32_t unFunctionID,  uint16_t usSampleRate)
 *
 * @brief
 *
 * @attention   None
 *
 * @param[in]   unFunctionID   such as: GH3X2X_FUNCTION_HR   or  GH3X2X_FUNCTION_HR|GH3X2X_FUNCTION_HRV
 * @param[in]   usSampleRate   only 25Hz times supported  (25/50/75...)
 * @param[out]  None
 *
 * @return  None
 */
#if __SUPPORT_FUNCTION_SAMPLE_RATE_MODIFY__
void gh3020_samplerate_set(uint32_t unFunctionID,  uint16_t usSampleRate)
{
    if((0 == usSampleRate)||(0 !=(usSampleRate%25)))
    {
        EXAMPLE_LOG("%s : usSampleRate is invalid !!!\r\n", __FUNCTION__);
        return;
    }

    for(uint8_t uchFunctionCnt = 0; uchFunctionCnt < GH3X2X_FUNC_OFFSET_MAX; uchFunctionCnt++)
    {
        if(unFunctionID&(((uint32_t)1)<<uchFunctionCnt))
        {
            if(g_pstGh3x2xFrameInfo[uchFunctionCnt])
            {
                g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstFunctionInfo->usSampleRateForUserSetting = usSampleRate;
            }
        }
    }
}
#endif



/**
 * @fn      void Gh3x2xDemoFunctionChannelEnSet(uint32_t unFunctionID,  uint32_t unChnlEn)
 *
 * @brief
 *
 * @attention   None
 *
 * @param[in]   unFunctionID   such as: GH3X2X_FUNCTION_HR   or  GH3X2X_FUNCTION_HR|GH3X2X_FUNCTION_HRV
 * @param[in]   unChnlEn   BitN = 0: channel N is disable        BitN = 1: channel N is enable
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2xDemoFunctionChannelEnSet(uint32_t unFunctionID,  uint32_t unChnlEn)
{
    for(uint8_t uchFunctionCnt = 0; uchFunctionCnt < GH3X2X_FUNC_OFFSET_MAX; uchFunctionCnt++)
    {
        if(unFunctionID&(((uint32_t)1)<<uchFunctionCnt))
        {
            if(g_pstGh3x2xFrameInfo[uchFunctionCnt])
            {
                g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstFunctionInfo->unChnlEnForUserSetting = unChnlEn;
            }
        }
    }
}

/**
 * @fn     int gh3020_init(void)
 *
 * @brief  Init GH3x2x module
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  error code
 */
int gh3020_init(void)
{
    int8_t schret = GH3X2X_RET_OK;
    EXAMPLE_LOG("%s : Lib Version : %s\r\n", __FUNCTION__, GH3X2X_GetDriverLibVersion());
    #if (__SUPPORT_PROTOCOL_ANALYZE__)
    EXAMPLE_LOG("%s : Protocol Version : %s\r\n", __FUNCTION__, GH3X2X_GetProtocolVersion());
    #endif

    /* Step 2: Reset gh3x2x chip */

    GH3X2X_RegisterResetPinControlFunc(hal_gh3x2x_reset_pin_ctrl);
    GH3X2X_HardReset();

    Gh3x2x_BspDelayMs(30);
    GH3X2X_ExitLowPowerMode();

    /* Step 3: Init gh3x2x chip */
    schret = GH3X2X_Init(&gh3020_initcfg_list[0]);
    if (GH3X2X_RET_OK == schret)
    {
        g_uchGh3x2xRegCfgArrIndex = 0;
        EXAMPLE_LOG("gh3020_init:init success\r\n");
    }
    else
    {
        g_uchGh3x2xRegCfgArrIndex = DRV_LIB_REG_CFG_EMPTY;
        EXAMPLE_LOG("gh3020_init:init fail, error code: %d\r\n", schret);
        return schret;
    }
    GH3X2X_SetMaxNumWhenReadFifo(__GH3X2X_RAWDATA_BUFFER_SIZE__);
    GH3X2X_EnterLowPowerMode();

    /* Step 4: setup EX INT for GH3X2X INT pin */
    #if (__NORMAL_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__ || __MIX_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__)
    if (g_uchGh3x2xIntMode == __NORMAL_INT_PROCESS_MODE__)
    {
        g_uchGh3x2xIntCallBackIsCalled = 0;
        if(0 == g_uchGh3x2xInitFlag)
        {
            hal_gh3x2x_int_init();
        }
    }
    #endif

    /* Step 5: init memory information for driver lib */
    #if ((__DRIVER_LIB_MODE__ == __DRV_LIB_WITH_ALGO__) || __USE_GOODIX_SOFT_ADT_ALGORITHM__)
    Gh3x2x_DemoAlgorithmMemConfig();
    #endif

    #if (__SUPPORT_PROTOCOL_ANALYZE__)
    Gh3x2x_HalSerialFifoInit();
    GH3X2X_UprotocolPacketMaxLenConfig(__PROTOCOL_DATA_LEN__);
    GH3X2X_RegisterGetFirmwareVersionFunc(GetDrvLibDemoVersion);
    GH3X2X_RegisterGetBootloaderVersionFunc(GetEvkBootloaderVersion);
    #endif

    /* Step 6: Soft ADT init */
    #if (__FUNC_TYPE_SOFT_ADT_ENABLE__)
    {
        Gh3x2x_SetAdtConfirmPara(__GSENSOR_MOVE_THRESHOLD__, __GSENSOR_MOVE_CNT_THRESHOLD__, __GSENSOR_NOT_MOVE_CNT_THRESHOLD__, __SOFT_ADT_IR_DETECT_TIMEOUT__);
    #if (__USE_POLLING_TIMER_AS_ADT_TIMER__)&&\
        (__POLLING_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__ || __MIX_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__)
    #else
        if (g_uchGh3x2xIntMode == __NORMAL_INT_PROCESS_MODE__)
        {
            Gh3x2xCreateAdtConfirmTimer();
        }
        #endif
    }
    #endif

    Gh3x2xFunctionInfoForUserInit();
    Gh3x2xFunctionCtrlModuleInit();

    g_uchGh3x2xInitFlag = 1;
    #if __GH3X2X_MEM_POOL_CHECK_EN__
    g_uchGh3x2xMemPollHaveGetCheckSum = 0;
    #endif

    return (int)schret;
}

/**
 * @fn     void gh3020_rdmode_switch(uint8_t uchIntModeType)
 *
 * @brief  change Interrupt process mode of GH3x2x.
 *
 * @attention   do not input 2
 *
 * @param[in]   emIntModeType 0:int pin 1: polling
 * @param[out]  None
 *
 * @return  None
 */
void gh3020_rdmode_switch(uint8_t uchIntModeType)
{
#if (__MIX_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__)
    EXAMPLE_LOG("[%s]:int process switch to = %d\r\n", __FUNCTION__, uchIntModeType);
    g_uchGh3x2xIntMode = uchIntModeType;
    g_uchGh3x2xInitFlag = 0;
#else
    EXAMPLE_LOG("[%s]:do not allow int mode switch!!!\r\n", __FUNCTION__);
#endif
}

uint8_t Gh3x2xGetInterruptMode(void)
{
    return g_uchGh3x2xIntMode;
}

/**
 * @fn     uint8_t gh3020_fifo_process(void)
 *
 * @brief  Interrupt process of GH3x2x.
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void gh3020_fifo_process(void)
{
  if(0 == g_uchGh3x2xInitFlag)
    {
      EXAMPLE_LOG("[%s]:gh3x2x is not init!!!\r\n", __FUNCTION__);
      return;
    }

  uint8_t uchRet;
  uint16_t usGotEvent;

#if __FUNC_TYPE_SOFT_ADT_ENABLE__
  if (g_uchGh3x2xIntMode == __POLLING_INT_PROCESS_MODE__ && 1 == g_uchVirtualAdtTimerCtrlStatus && GH3X2X_GetSoftWearOffDetEn())
    {
      Gh3x2xDemoMoveDetectTimerHandler();
    }
#endif

    //check "Gh3x2xDemoInterruptProcess" call is legal or not
  if (g_uchGh3x2xIntMode == __NORMAL_INT_PROCESS_MODE__)
    {
      if((0 == GH3X2X_GetSoftEvent())&&(0 == g_uchGh3x2xIntCallBackIsCalled))
        {
          EXAMPLE_LOG("InterruptProcess:invalid call, do not process.\r\n");
          return;
        }
    }

  if (g_uchGh3x2xIntMode == __NORMAL_INT_PROCESS_MODE__)
    {
      g_uchGh3x2xIntCallBackIsCalled = 0;
    }

  uint16_t current_fifowtm = GH3X2X_GetCurrentFifoWaterLine() *
                             GH3020_BYTES_PER_DATA;
  uint16_t readcnt = 0;
  do
    {
      uchRet = GH3X2X_INT_PROCESS_SUCCESS;
      usGotEvent = 0;

      /* Step 1: exit low power mode, and read irq status */

      if ((GH3x2x_GetNeedWakeUpGh3x2xFlag()) || (GH3X2X_GetSoftEvent()))
        {
          GH3X2X_ExitLowPowerMode();
        }

      usGotEvent = GH3X2X_GetIrqStatus();
      usGotEvent &= __GH3X2X_EVENT_PROCESS_MASK__;

      /* Step 2: if hard adt enabled, do wear event process */

    #if (__SUPPORT_HARD_ADT_CONFIG__)
      Gh3x2xDemoWearEventProcess(&usGotEvent, WEAR_DETECT_DONT_FORCE_SWITCH);

    #if __EXAMPLE_LOG_CONFIG__
      if(0 != (usGotEvent & GH3X2X_IRQ_MSK_WEAR_ON_BIT))
        {
            EXAMPLE_LOG("Got hardware wear on !!! \r\n");
        }
        if(0 != (usGotEvent & GH3X2X_IRQ_MSK_WEAR_OFF_BIT))
        {
            EXAMPLE_LOG("Got hardware wear off !!! \r\n");
        }
    #endif
    #endif

        /* Step 3: if it's FIFO interrupt, read gh3x2x fifo data, read gsensor data*/

      if (g_uchGh3x2xIntMode == __POLLING_INT_PROCESS_MODE__)
        {
          usGotEvent |= GH3X2X_IRQ_MSK_FIFO_WATERMARK_BIT;
        }

      if (GH3X2X_GetSoftEvent() == 0 && usGotEvent == 0)
        {
            GH3X2X_EnterLowPowerMode();
            EXAMPLE_LOG("no event need to handle!!!\r\n");
            return;
        }

      EXAMPLE_LOG("usGotEvent = 0x%x\n", usGotEvent);
      if ((usGotEvent & (GH3X2X_IRQ_MSK_FIFO_WATERMARK_BIT | GH3X2X_IRQ_MSK_FIFO_FULL_BIT)) \
          || (GH3X2X_GetSoftEvent() & (GH3X2X_SOFT_EVENT_NEED_FORCE_READ_FIFO|GH3X2X_SOFT_EVENT_NEED_TRY_READ_FIFO)))
        {
          uint16_t usFifoByteNum = ((uint16_t)4)*gh3020_spi_readreg(GH3020_REG_INT_FIFO_UR); //read fifo use
          GH3020_DEBUG_LOG("[gh3020] fifo byte=%u\n", usFifoByteNum);

          if (g_uchGh3x2xIntMode == __NORMAL_INT_PROCESS_MODE__)
            {
              uint8_t uchIsForceRead = (GH3X2X_GetSoftEvent() & GH3X2X_SOFT_EVENT_NEED_FORCE_READ_FIFO);
              if (readcnt < current_fifowtm)
                {
                    if (current_fifowtm - readcnt < usFifoByteNum)
                      {
                          usFifoByteNum = current_fifowtm - readcnt;
                      }
                }
              else if(0 == uchIsForceRead)
                {
                  if (usFifoByteNum < ((uint16_t)4)*GH3X2X_GetCurrentFifoWaterLine())  //data is too less
                    {
                      usFifoByteNum = 0;   //no need read
                    }
                }
            }

          GH3X2X_ClearSoftEvent(GH3X2X_SOFT_EVENT_NEED_FORCE_READ_FIFO|GH3X2X_SOFT_EVENT_NEED_TRY_READ_FIFO);
          if (usFifoByteNum != 0)
          {
              GH3020_DEBUG_LOG("[gh3020] fifo read %uB\n", usFifoByteNum);
              if (GH3X2X_RET_READ_FIFO_CONTINUE ==
                  GH3X2X_ReadFifodata(g_uchGh3x2xReadRawdataBuffer,
                                      &g_usGh3x2xReadRawdataLen, usFifoByteNum))
                {
                  GH3X2X_SetSoftEvent(GH3X2X_SOFT_EVENT_NEED_TRY_READ_FIFO);
                }

              if (usFifoByteNum > __GH3X2X_RAWDATA_BUFFER_SIZE__)
                {
                  readcnt += __GH3X2X_RAWDATA_BUFFER_SIZE__;
                }
              else
                {
                  readcnt += usFifoByteNum;
                }
          }

        #if ((__SUPPORT_PROTOCOL_ANALYZE__)||(__SUPPORT_ALGO_INPUT_OUTPUT_DATA_HOOK_CONFIG__))
        #if (__SUPPORT_ELECTRODE_WEAR_STATUS_DUMP__)
          GH3X2X_ReadElectrodeWearDumpData();
        #endif
          GH3X2X_RecordDumpData();
        #endif
          GH3X2X_RecordTiaGainInfo();
        }

        /* Step 5: do soft agc process*/

        Gh3x2x_UserHandleCurrentInfo();
    #ifdef CONFIG_FACTEST_SENSORS_GH3020
      if(0 == g_uchEngineeringModeStatus)  //in engineering mode,  disable soft agc function
    #endif
        {
            if(0 != (usGotEvent & (GH3X2X_IRQ_MSK_TUNNING_FAIL_BIT | GH3X2X_IRQ_MSK_TUNNING_DONE_BIT)))
            {
                GH3X2X_SoftLedADJAutoADJInt();
            }
        #if (__SUPPORT_SAMPLE_DEBUG_MODE__)
            if((g_usDumpMode & 0x3) != 0)
            {
                GH3X2X_LedAgcProcessExDump(g_uchGh3x2xReadRawdataBuffer);
            }
            else
        #endif
            {
                GH3X2X_LedAgcProcess(g_uchGh3x2xReadRawdataBuffer, g_usGh3x2xReadRawdataLen);
            }
        }

        /* Extern Step: send fifo*/
    #if (__FIFO_PACKAGE_SEND_ENABLE__)
        if (GH3X2X_GetFifoPackageMode())
        {
            GH3X2X_SendRawdataFifoPackage(g_uchGh3x2xReadRawdataBuffer, g_usGh3x2xReadRawdataLen);
        }
    #endif

        /* Step 6: chip reset event process*/
        if(0 != (usGotEvent & GH3X2X_IRQ_MSK_CHIP_RESET_BIT))
        {
            if(0 == GH3x2x_GetChipResetRecoveringFlag())  //do nothing in recover period
            {
                EXAMPLE_LOG("Got chip reset event !!! ActiveChipResetFlag = %d, g_unDemoFuncMode = %d, recovering flag = %d\r\n",(int)GH3x2x_GetActiveChipResetFlag(), (int)g_unDemoFuncMode, (int)GH3x2x_GetChipResetRecoveringFlag());
                if((GH3x2x_GetActiveChipResetFlag())||(0 == g_unDemoFuncMode))
                {
                    Gh3x2xDemoSamplingControl(0xFFFFFFFF, UPROTOCOL_CMD_STOP);
                    GH3X2X_ExitLowPowerMode();
                    GH3X2X_Init(GH3X2X_PTR_NULL);
                }
                else
                {
                    //need recovery for chip reset
                    int8_t schret = GH3X2X_RET_GENERIC_ERROR;
                    uint32_t unDemoFuncModeBeforeChipReset;
                    EXAMPLE_LOG("gh3020_fifo_process:sart chip reset recovery...\r\n");
                    GH3x2x_SetChipResetRecoveringFlag(1);
                #if __SUPPORT_HARD_RESET_CONFIG__
                    GH3X2X_HardReset();
                #else
                    GH3X2X_SoftReset();
                #endif
                    Gh3x2x_BspDelayMs(30);
                    unDemoFuncModeBeforeChipReset = g_unDemoFuncMode;
                    gh3020_stop_sampling(g_unDemoFuncMode);
                    GH3X2X_ExitLowPowerMode();

                    schret = GH3X2X_Init(gh3020_initcfg_list+g_uchGh3x2xRegCfgArrIndex);

                    if (GH3X2X_RET_OK == schret)
                    {
                        EXAMPLE_LOG("gh3020_fifo_process:recovery init success\r\n");
                    }
                    else
                    {
                        g_uchGh3x2xRegCfgArrIndex = DRV_LIB_REG_CFG_EMPTY;
                        EXAMPLE_LOG("gh3020_fifo_process:recovery init fail, error code: %d\r\n", schret);
                    }
                #ifdef CONFIG_FACTEST_SENSORS_GH3020
                    if(1 == g_uchEngineeringModeStatus)
                    {
                        gh3020_start_sampling_factest(unDemoFuncModeBeforeChipReset, g_pstSampleParaGroup, g_uchEngineeringModeSampleParaGroupNum);
                    }
                    else
                #endif
                    {
                        gh3020_start_sampling(unDemoFuncModeBeforeChipReset);
                    }
                    GH3x2x_SetChipResetRecoveringFlag(0);

                }
        }
    }


        GH3X2X_EnterLowPowerModeWithoutWaiting();

        /* Step 7: report event */
        if(GH3X2X_GetSoftEvent() & GH3X2X_SOFT_EVENT_WEAR_OFF)
        {
            usGotEvent |= GH3X2X_IRQ_MSK_WEAR_OFF_BIT;
            GH3X2X_ClearSoftEvent(GH3X2X_SOFT_EVENT_WEAR_OFF);
        }

        if ((usGotEvent & (GH3X2X_IRQ_MSK_FIFO_WATERMARK_BIT |
                           GH3X2X_IRQ_MSK_FIFO_FULL_BIT)) ||
            (GH3X2X_GetSoftEvent() & (GH3X2X_SOFT_EVENT_NEED_FORCE_READ_FIFO |
                                      GH3X2X_SOFT_EVENT_NEED_TRY_READ_FIFO))
            || readcnt == current_fifowtm)
        {
        #if __GS_NONSYNC_READ_EN__
            uint16_t usGsensorNeedPointNum;
            int16_t sGsensorHeadPosi;
        #endif
            if (GH3X2X_GetGsensorEnableFlag())
            {
            #if (__FUNC_TYPE_SOFT_ADT_ENABLE__)
                if (g_uchVirtualAdtTimerCtrlStatus == 0)
            #endif
                {
                    Gsensor_drv_get_fifo_data(gsensor_soft_fifo_buffer + __GS_EXTRA_BUF_LEN__, &gsensor_soft_fifo_buffer_index);
                    Gh3x2x_NormalizeGsensorSensitivity(gsensor_soft_fifo_buffer + __GS_EXTRA_BUF_LEN__ ,gsensor_soft_fifo_buffer_index);
                }

            }
            /*extra step: get cap and temp data from other sensor*/
            #if __CAP_ENABLE__
            if (GH3X2X_GetCapEnableFlag())
            {
                Cap_drv_get_fifo_data(cap_soft_fifo_buffer, &cap_soft_fifo_buffer_index);
            }
            #endif
            #if __TEMP_ENABLE__
            if (GH3X2X_GetTempEnableFlag())
            {
                Temp_drv_get_fifo_data(temp_soft_fifo_buffer, &temp_soft_fifo_buffer_index);
            }
            #endif
            /* Step 8: algorithm process*/
            #if __GH3X2X_MEM_POOL_CHECK_EN__
            Gh3x2xCheckMemPollBeforeAlgoCal();
            #endif



        #ifdef GOODIX_DEMO_PLANFORM
            if (((GH3X2X_DEMO_WORK_MODE_MCU_ONLINE == g_uchDemoWorkMode) ||\
                        (GH3X2X_DEMO_WORK_MODE_PASS_THROUGH == g_uchDemoWorkMode) ||\
                        (GH3X2X_DEMO_WORK_MODE_APP == g_uchDemoWorkMode) ||\
                        (GH3X2X_DEMO_WORK_MODE_EVK == g_uchDemoWorkMode)) &&\
                        ((g_usDumpMode & 0x3) == 0))
        #endif
            {
                //#if (__DRIVER_LIB_MODE__==__DRV_LIB_WITH_ALGO__)
            #if __GS_NONSYNC_READ_EN__
                usGsensorNeedPointNum = Gh3x2xGetGsensorNeedPointNum(gsensor_soft_fifo_buffer_index, g_uchGh3x2xReadRawdataBuffer, g_usGh3x2xReadRawdataLen);
                sGsensorHeadPosi = Gh3x2xGetGsensorHeadPos(gsensor_soft_fifo_buffer_index,usGsensorNeedPointNum);
                Gh3x2x_DemoAlgorithmCalculate(g_uchGh3x2xReadRawdataBuffer,g_usGh3x2xReadRawdataLen, gsensor_soft_fifo_buffer + __GS_EXTRA_BUF_LEN__ + sGsensorHeadPosi, usGsensorNeedPointNum,
                                                cap_soft_fifo_buffer,cap_soft_fifo_buffer_index,temp_soft_fifo_buffer,temp_soft_fifo_buffer_index);


                for(uint16_t usPointCnt = 0; usPointCnt < usGsensorNeedPointNum; usPointCnt++)
                {
                    STGsensorRawdata * temp_gsensor = gsensor_soft_fifo_buffer + __GS_EXTRA_BUF_LEN__ + sGsensorHeadPosi;
                    EXAMPLE_LOG("[Gsensor show]x = %d,y = %d, z = %d\r\n",temp_gsensor[usPointCnt].sXAxisVal,temp_gsensor[usPointCnt].sYAxisVal,temp_gsensor[usPointCnt].sZAxisVal);
                }


                Gh3x2xNonSyncGsenosrPostProcess(gsensor_soft_fifo_buffer_index, usGsensorNeedPointNum, sGsensorHeadPosi);
            #else
                Gh3x2x_DemoAlgorithmCalculate(g_uchGh3x2xReadRawdataBuffer,g_usGh3x2xReadRawdataLen, gsensor_soft_fifo_buffer + __GS_EXTRA_BUF_LEN__, gsensor_soft_fifo_buffer_index,
                                                cap_soft_fifo_buffer,cap_soft_fifo_buffer_index,temp_soft_fifo_buffer,temp_soft_fifo_buffer_index);
            #endif
            }
            #if __GH3X2X_MEM_POOL_CHECK_EN__
            Gh3x2xUpdataMemPollChkSumAfterAlgoCal();
            #endif
        }

        ///* Step 9: event hook */

    #if (__SUPPORT_HARD_ADT_CONFIG__)
        if (usGotEvent & GH3X2X_IRQ_MSK_WEAR_ON_BIT)
        {
            // Gh3x2x_WearEventHook(usGotEvent, uchEventEx);
            gh3020_start_sampling(GH3X2X_FUNCTION_SOFT_ADT_IR);
            GOODIX_PLANFROM_WEAR_ON_EVENT();
            EXAMPLE_LOG("Wear on, object !!!\r\n");
        }
    #endif


        if (GH3X2X_GetSoftEvent())  //have new soft event
        {
            uchRet = GH3X2X_INT_PROCESS_REPEAT;
            EXAMPLE_LOG("InterruptProcess: process need repeat.\r\n");
        }

    }while(GH3X2X_INT_PROCESS_REPEAT == uchRet);
}

/**
 * @fn     int8_t GH3x2xDemoSearchCfgListByFunc(uint8_t* puchCfgIndex, uint32_t unFuncMode, uint32_t unFuncModeFilter)
 *
 * @brief  Search reg config array by function mode
 *
 * @attention   None
 *
 * @param[in]   unFuncMode              function mode
 * @param[in]   unFuncModeFilter        ignored function mode
 * @param[out]  puchCfgIndex            reg config array index
 *
 * @return  GH3X2X_RET_OK:switch application mode success
 * @return  GH3X2X_RET_RESOURCE_ERROR:can't find corresponding reg config array
 */
int8_t GH3x2xDemoSearchCfgListByFunc(uint8_t* puchCfgIndex, uint32_t unFuncMode, uint32_t unFuncModeFilter)
{
    int8_t  chRet = GH3X2X_RET_OK;
    uint8_t  uchIndex = 0;
    uint32_t unFuncModeTemp = GH3X2X_NO_FUNCTION;
    //find reg config array that match unFuncMode
    EXAMPLE_LOG("target app mode:0x%lx\r\n",unFuncMode);
    for (uchIndex = 0; uchIndex < __GH3X2X_CFG_LIST_MAX_NUM__; uchIndex++)
    {
        if (((gh3020_initcfg_list+uchIndex)->arrlen) > DRV_LIB_REG_CFG_MIN_LEN)
        {
            GH3X2X_DecodeRegCfgArr(&unFuncModeTemp,(gh3020_initcfg_list+uchIndex)->pregarr, \
                                (gh3020_initcfg_list+uchIndex)->arrlen);


            EXAMPLE_LOG("search cfg app mode:0x%lx\r\n",unFuncModeTemp);

            if((unFuncMode | unFuncModeFilter) == ((unFuncModeTemp&unFuncMode) | unFuncModeFilter))
            {
                (*puchCfgIndex) = uchIndex;
                EXAMPLE_LOG("Got right cfg ,index = %d\r\n",uchIndex);
                break;
            }
        }
    }

    if (uchIndex >= __GH3X2X_CFG_LIST_MAX_NUM__)
    {
        chRet = GH3X2X_RET_RESOURCE_ERROR;
    }
    return chRet;
}


#if __GH3X2X_ARRAY_CFG_MANUAL_SWITCH_EN__

/**
 * @fn     int8_t Gh3x2xDemoArrayCfgSwitch(uint8_t uchArrayCfgIndex)
 *
 * @brief  array cfg switch (call by user)
 *
 * @attention   If __GH3X2X_ARRAY_CFG_MANUAL_SWITCH_EN__ is 1, you should switch array cfg manually before calling Gh3x2xDemoStartSampling
 *
 * @param[in]   uchArrayCfgIndex    0: gh3020_reglist_normal    1: gh3020_reglist_factest ...
 * @param[out]  None
 *
 * @return  GH3X2X_RET_OK:switch application mode success
 * @return  GH3X2X_RET_RESOURCE_ERROR:can't find corresponding reg config array
 */
int8_t Gh3x2xDemoArrayCfgSwitch(uint8_t uchArrayCfgIndex)
{
    int8_t  chRet = GH3X2X_RET_OK;

    if(g_uchGh3x2xRegCfgArrIndex == uchArrayCfgIndex)
    {
        EXAMPLE_LOG("Current cfg is already cfg%d, no need switch.\r\n",(int)uchArrayCfgIndex);
    }
    else if(uchArrayCfgIndex < __GH3X2X_CFG_LIST_MAX_NUM__)
    {
        Gh3x2xDemoSamplingControl(0xFFFFFFFF, UPROTOCOL_CMD_STOP);
        #if __SUPPORT_HARD_RESET_CONFIG__
        GH3X2X_HardReset();
        #else
        GH3X2X_SoftReset();
        #endif
        Gh3x2x_BspDelayMs(15);
        GH3X2X_ExitLowPowerMode();
        if (GH3X2X_RET_OK == GH3X2X_Init(gh3020_initcfg_list + uchArrayCfgIndex))
        {
            g_uchGh3x2xRegCfgArrIndex = uchArrayCfgIndex;
            EXAMPLE_LOG("cfg%d switch success !!!.\r\n",(int)uchArrayCfgIndex);
        }
        else
        {
            g_uchGh3x2xRegCfgArrIndex = DRV_LIB_REG_CFG_EMPTY;
            chRet = GH3X2X_RET_RESOURCE_ERROR;
            EXAMPLE_LOG("Error ! cfg%d switch fail !!!\r\n",(int)uchArrayCfgIndex);
        }
        GH3X2X_EnterLowPowerMode();

    }
    else
    {
        EXAMPLE_LOG("Error! uchArrayCfgIndex(%d) is invalid !!!\r\n",(int)uchArrayCfgIndex);
        chRet = GH3X2X_RET_RESOURCE_ERROR;
    }
    return chRet;
}

#endif


void Gh3x2xSetCurrentSlotEnReg(uint32_t unFuncMode)
{
    uint8_t uchSlotEn = 0;

    #if __SLOT_SEQUENCE_DYNAMIC_ADJUST_EN__
    uint8_t  uchSlotSeq[8];
    uint8_t  uchValidSlotNum = 0;
    uint16_t usStartReg;
    uint8_t uchSlotSrDiver[8];
    uint8_t uchSlotCfgHaveSortedFlag[8] = {0};
    uint8_t uchSlotCfgIndexFast2Low[8];
    #endif
    for(uint8_t uchFunCnt = 0; uchFunCnt < GH3X2X_FUNC_OFFSET_MAX; uchFunCnt ++)
    {
        if(g_pstGh3x2xFrameInfo[uchFunCnt])
        {

            if(g_unDemoFuncMode & (((uint32_t)1)<< uchFunCnt))
            {
                #if (__FUNC_TYPE_SOFT_ADT_ENABLE__)
                if((GH3X2X_FUNC_OFFSET_ADT != uchFunCnt)||(1 == g_uchHardAdtFuncStatus))
                #endif
                {
                    uchSlotEn |= g_pstGh3x2xFrameInfo[uchFunCnt]->pstFunctionInfo->uchSlotBit;
                }
            }
        }
    }
    #if __SLOT_SEQUENCE_DYNAMIC_ADJUST_EN__
    for(uint8_t uchSlotCnt = 0; uchSlotCnt < 8; uchSlotCnt ++)
    {
        //get diver for every slot cfg
        uchSlotSrDiver[uchSlotCnt] = GH3x2xGetAgcReg(GH3X2X_AGC_REG_SR_MULIPLIPER, uchSlotCnt);
    }

    //EXAMPLE_LOG("%s : Slot cfg Sort fast to slow: ", __FUNCTION__);
    for(uint8_t uchSlotCnt = 0; uchSlotCnt < 8; uchSlotCnt ++)
    {
        //find min diver
        uint16_t usCurrentMinDiver = 0xFFFF;
        uint8_t uchCurrentMinDiverCfgIndex = 0xFF;
        for(uint8_t uchSearchCnt = 0; uchSearchCnt < 8; uchSearchCnt ++)
        {
            if(0 == uchSlotCfgHaveSortedFlag[uchSearchCnt])
            {
                if(usCurrentMinDiver > uchSlotSrDiver[uchSearchCnt])
                {
                    usCurrentMinDiver = uchSlotSrDiver[uchSearchCnt];
                    uchCurrentMinDiverCfgIndex = uchSearchCnt;
                }
            }
        }
        if(0xFF != uchCurrentMinDiverCfgIndex)
        {
            uchSlotCfgHaveSortedFlag[uchCurrentMinDiverCfgIndex] = 1;
            uchSlotCfgIndexFast2Low[uchSlotCnt] = uchCurrentMinDiverCfgIndex;

            //EXAMPLE_LOG("%d ", uchCurrentMinDiverCfgIndex);
        }
    }
    //EXAMPLE_LOG("\r\n");


    //find slot cfg from fastest to lowest
    for(uint8_t uchSlotCnt = 0; uchSlotCnt < 8; uchSlotCnt ++)
    {
        uint8_t uchCurrentFastSlotCfgIndex = uchSlotCfgIndexFast2Low[uchSlotCnt];
        for(uint8_t uchSlotSearchCnt = 0; uchSlotSearchCnt < 8; uchSlotSearchCnt ++)
        {
            if(uchSlotEn&(1<<uchSlotSearchCnt))
            {
                if(uchSlotSearchCnt == uchCurrentFastSlotCfgIndex)
                {
                    uchSlotSeq[uchValidSlotNum] = uchSlotSearchCnt;
                    uchValidSlotNum ++;
                }
            }
        }
    }
    if(uchValidSlotNum < 8)
    {
        uchSlotSeq[uchValidSlotNum] = 8;
        uchValidSlotNum ++;
    }
    else
    {
        uchValidSlotNum = 8;
    }


    usStartReg = gh3020_spi_readreg(0x0000);  //bak start reg
    gh3020_spi_writereg(0x0000,usStartReg&(0xFFFE));   //stop sample
    //write 0x0100~0x0106 and 0x01EC
    for(uint8_t uchSlotCnt = 0; uchSlotCnt < uchValidSlotNum; uchSlotCnt ++)
    {
        if(0 == (uchSlotCnt&0x01)) //0,2,4,6
        {
            gh3020_spi_writebits(0x0100 + (uchSlotCnt&0xFE), 0,3, uchSlotSeq[uchSlotCnt]);
        }
        else  //1,3,5,7
        {
            gh3020_spi_writebits(0x0100 + (uchSlotCnt&0xFE), 8,11,  uchSlotSeq[uchSlotCnt]);
        }
        if(uchSlotSeq[uchSlotCnt] < 8)
        {
            gh3020_spi_writereg(0x01EC + uchSlotCnt*2, g_usGh3x2xSlotTime[uchSlotSeq[uchSlotCnt]]);
            //EXAMPLE_LOG("uchSlotCnt = %d, uchSlotSeq = %d, slottime = %d\r\n",uchSlotCnt,uchSlotSeq[uchSlotCnt],g_usGh3x2xSlotTime[uchSlotSeq[uchSlotCnt]]);
        }
    }
    gh3020_spi_writereg(0x0000,usStartReg);   //recover sample reg
    #endif
    GH3X2X_SlotEnRegSet(uchSlotEn);
    EXAMPLE_LOG("Current uchSlotEn= 0x%x\r\n",uchSlotEn);
}

#define GH3X2X_ECG_CTRL_FLAG_START 1
#define GH3X2X_ECG_CTRL_FLAG_STOP  2

#define GH3X2X_HARD_ADT_CTRL_FLAG_START 1
#define GH3X2X_HARD_ADT_CTRL_FLAG_STOP  2
#if __SUPPORT_FUNCTION_SAMPLE_RATE_MODIFY__
void Gh3x2xSampleRateUpdate(uint32_t unFuncMode)
{
    uint16_t usLastSampleRate[8] = {0};  // last sample rate for every slot
    uint16_t usTargetSampleRate[8] = {0};
    uint8_t  uchHaveStopSample = 0;
    uint16_t usStartReg;


    //read sample rate reg
    for(uint8_t uchSlotCnt = 0; uchSlotCnt < 8; uchSlotCnt ++)
    {
        usLastSampleRate[uchSlotCnt] = 1000/(GH3x2xGetAgcReg(GH3X2X_AGC_REG_SR_MULIPLIPER, uchSlotCnt) + 1);
    }



    //get every slot target sample rate
    for(uint8_t uchFunctionCnt = 0; uchFunctionCnt < GH3X2X_FUNC_OFFSET_MAX; uchFunctionCnt++)
    {
        if((g_pstGh3x2xFrameInfo[uchFunctionCnt])&&(unFuncMode&((uint32_t)1<<uchFunctionCnt)))     //function is valid and opened
        {
            uint16_t usTempTargetSampleRate = g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstFunctionInfo->usSampleRateForUserSetting;
            if(0 == usTempTargetSampleRate)
            {
                usTempTargetSampleRate = g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstFunctionInfo->usSampleRate;
            }
            uint8_t uchTempSlotBit = g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstFunctionInfo->uchSlotBit;
            EXAMPLE_LOG("[Gh3x2xSampleRateUpdate]FunctionId = 0x%lX, slot bit = 0x%X,usSampleRateUser = %d\r\n",g_pstGh3x2xFrameInfo[uchFunctionCnt]->unFunctionID,uchTempSlotBit, usTempTargetSampleRate);
            for(uint8_t uchSlotCnt = 0; uchSlotCnt < 8; uchSlotCnt ++)
            {
                if(uchTempSlotBit&(1<<uchSlotCnt))
                {
                    if(usTempTargetSampleRate > usTargetSampleRate[uchSlotCnt])
                    {
                        usTargetSampleRate[uchSlotCnt] = usTempTargetSampleRate;   //store faster sample rate
                    }
                }
            }
        }
    }


    //update sample rate registers
    for(uint8_t uchSlotCnt = 0; uchSlotCnt < 8; uchSlotCnt ++)
    {
        if((usTargetSampleRate[uchSlotCnt] != usLastSampleRate[uchSlotCnt])&&(0 != usTargetSampleRate[uchSlotCnt]))
        {
            if(0 == uchHaveStopSample)
            {
                usStartReg = gh3020_spi_readreg(0x0000);  //bak start reg
                gh3020_spi_writereg(0x0000,usStartReg&(0xFFFE));   //stop sample
                uchHaveStopSample = 1;
            }
            GH3x2xSetAgcReg(GH3X2X_AGC_REG_SR_MULIPLIPER, uchSlotCnt, (1000/usTargetSampleRate[uchSlotCnt]) - 1);
            EXAMPLE_LOG("[Gh3x2xSampleRateUpdate]Slot%d sample rate need change: Last = %d, Target = %d\r\n",uchSlotCnt, usLastSampleRate[uchSlotCnt], usTargetSampleRate[uchSlotCnt]);
        }
    }
    if(uchHaveStopSample)
    {
        gh3020_spi_writereg(0x0000,usStartReg);   //recover sample reg
    }



    //update usSampleRate value,and down sample check
    for(uint8_t uchFunctionCnt = 0; uchFunctionCnt < GH3X2X_FUNC_OFFSET_MAX; uchFunctionCnt++)
    {
        uint16_t usSlowestSlotSampleRate = 0xFFFF;

        if((g_pstGh3x2xFrameInfo[uchFunctionCnt])&&(unFuncMode&((uint32_t)1<<uchFunctionCnt)))     //function is valid and opened
        {
            uint8_t uchTempSlotBit = g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstFunctionInfo->uchSlotBit;
            //update update usSampleRate value
            if(g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstFunctionInfo->usSampleRateForUserSetting)
            {
                g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstFunctionInfo->usSampleRate = g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstFunctionInfo->usSampleRateForUserSetting;
            }
            //get slowest slot in  function (target sample rate)
            for(uint8_t uchSlotCnt = 0; uchSlotCnt < 8; uchSlotCnt ++)
            {
                if(uchTempSlotBit&(1<<uchSlotCnt))
                {
                    if(usSlowestSlotSampleRate > usTargetSampleRate[uchSlotCnt])
                    {
                        usSlowestSlotSampleRate = usTargetSampleRate[uchSlotCnt];
                    }
                }
            }
            if(usSlowestSlotSampleRate > g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstFunctionInfo->usSampleRate)   //need down sample
            {
                g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstDownSampleInfo->uchDownSampleFactor = (usSlowestSlotSampleRate/g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstFunctionInfo->usSampleRate) - 1;
            }
            else
            {
                g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstDownSampleInfo->uchDownSampleFactor = 0;
            }
            g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstDownSampleInfo->uchDownSampleCnt = 0;
            EXAMPLE_LOG("[Gh3x2xSampleRateUpdate]Down sample: FunctionID = 0x%lX,: SlowestSlotSR = %d, Factor = %d\r\n",g_pstGh3x2xFrameInfo[uchFunctionCnt]->unFunctionID, usSlowestSlotSampleRate, g_pstGh3x2xFrameInfo[uchFunctionCnt]->pstDownSampleInfo->uchDownSampleFactor);

        }

    }

}
#endif

void GH3x2xMaserControlFunctionLog(uint32_t unFuncMode, EMUprotocolParseCmdType emCmdType)
{
    if(UPROTOCOL_CMD_START == emCmdType)
    {
        EXAMPLE_LOG("Master start function = 0x%lx\r\n",unFuncMode);
    }
    else
    {
        EXAMPLE_LOG("Master stop function = 0x%lx\r\n",unFuncMode);
    }
}




/**
 * @fn     void Gh3x2xDemoSamplingControl(EMUprotocolParseCmdType emSwitch)
 *
 * @brief  Start/stop sampling of gh3x2x and sensor
 *
 * @attention   None
 *
 * @param[in]   unFuncMode      function that will be started or stopped
 * @param[in]   emSwitch        stop/start sampling
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2xDemoSamplingControl(uint32_t unFuncMode, EMUprotocolParseCmdType emSwitch)
{
    #if ((__SUPPORT_PROTOCOL_ANALYZE__)||(__SUPPORT_HARD_ADT_CONFIG__))
    uint16_t usGotEvent = 0;
    #endif
    #if __GS_NONSYNC_READ_EN__

    uint16_t usTempFunction;
    #endif

    EXAMPLE_LOG("[%s]:emSwitch = %d\r\n",__FUNCTION__, emSwitch);

    GH3X2X_ExitLowPowerMode();
    Gh3x2x_BspDelayUs(500);

    for(uint8_t uchFunCnt = 0; uchFunCnt < GH3X2X_FUNC_OFFSET_MAX; uchFunCnt ++)
    {
        if(g_pstGh3x2xFrameInfo[uchFunCnt])
        {
            if(unFuncMode & (((uint32_t)1)<< uchFunCnt))
            {
                if((UPROTOCOL_CMD_START == emSwitch)&&((g_unDemoFuncMode & (((uint32_t)1)<< uchFunCnt)) != (((uint32_t)1)<< uchFunCnt)))
                {
                    if(GH3X2X_FUNC_OFFSET_ADT == uchFunCnt)
                    {
                    #if (__FUNC_TYPE_SOFT_ADT_ENABLE__)
                        if (GH3X2X_GetSoftWearOffDetEn())
                        {
                            GH3X2X_AdtFuncStartWithConfirm();
                            GH3X2X_START_ADT_TIMER();
                        }
                        else
                        {
                            GH3X2X_FunctionStart(g_pstGh3x2xFrameInfo[uchFunCnt]);
                            g_uchHardAdtFuncStatus = 1;
                        }
                    #else
                        GH3X2X_FunctionStart(g_pstGh3x2xFrameInfo[uchFunCnt]);
                    #endif
                    }
                    else
                    {
                        GH3X2X_FunctionStart(g_pstGh3x2xFrameInfo[uchFunCnt]);
                    }
                }
                else if ((UPROTOCOL_CMD_STOP == emSwitch)&&((g_unDemoFuncMode & (((uint32_t)1)<< uchFunCnt)) == (((uint32_t)1)<< uchFunCnt)))
                {
                    if(GH3X2X_FUNC_OFFSET_ADT == uchFunCnt)
                    {
                    #if (__SUPPORT_HARD_ADT_CONFIG__)
                    #if (__FUNC_TYPE_SOFT_ADT_ENABLE__)
                        if (GH3X2X_GetSoftWearOffDetEn())
                        {
                            Gh3x2x_ResetMoveDetectByGsData();
                            GH3X2X_STOP_ADT_TIMER();
                        }
                        g_uchHardAdtFuncStatus = 0;
                    #endif
                        GH3X2X_FunctionStop(g_pstGh3x2xFrameInfo[uchFunCnt]);
                        usGotEvent |= GH3X2X_IRQ_MSK_WEAR_OFF_BIT;
                        Gh3x2xDemoWearEventProcess(&usGotEvent,WEAR_DETECT_FORCE_SWITCH);
                    #endif
                    #if (__SUPPORT_PROTOCOL_ANALYZE__)
                        Gh3x2xDemoReportEvent(usGotEvent,0);
                    #endif
                    }
                    else
                    {
                        GH3X2X_FunctionStop(g_pstGh3x2xFrameInfo[uchFunCnt]);
                    #if __FUNC_TYPE_ECG_ENABLE__
                        if(GH3X2X_FUNC_OFFSET_ECG == uchFunCnt)
                        {
                            uchEcgCtrlFlag = GH3X2X_ECG_CTRL_FLAG_STOP;
                        }
                    #endif
                    }
                }
            }
        }
    }

    /* Enable gsensor sampling */
    if (UPROTOCOL_CMD_START == emSwitch)
    {
        if (GH3X2X_GetGsensorEnableFlag() && (GH3X2X_NO_FUNCTION == g_unDemoFuncMode))
        {
            Gsensor_start_cache_data();
        }
        if (GH3X2X_GetCapEnableFlag() && (GH3X2X_NO_FUNCTION == g_unDemoFuncMode))
        {
        #if __FUNC_TYPE_SOFT_ADT_ENABLE__
            GH3x2xReadCapFromFlash();
            Cap_start_cache_data();
        #endif
        }
        if (GH3X2X_GetTempEnableFlag() && (GH3X2X_NO_FUNCTION == g_unDemoFuncMode))
        {
        #if __FUNC_TYPE_BT_ENABLE__
            Temp_start_cache_data();
        #endif
        }
        g_unDemoFuncMode |= unFuncMode;
    }
    else
    {
        g_unDemoFuncMode &= ~unFuncMode;
        if (GH3X2X_GetGsensorEnableFlag()  && (GH3X2X_NO_FUNCTION == g_unDemoFuncMode))
        {
            Gsensor_stop_cache_data();
        }
        if (GH3X2X_GetCapEnableFlag() && (GH3X2X_NO_FUNCTION == g_unDemoFuncMode))
        {
        #if __FUNC_TYPE_SOFT_ADT_ENABLE__
            GH3x2xWriteCapToFlash();
            Cap_stop_cache_data();
        #endif
        }
        if (GH3X2X_GetTempEnableFlag() && (GH3X2X_NO_FUNCTION == g_unDemoFuncMode))
        {
        #if __FUNC_TYPE_BT_ENABLE__
            Temp_stop_cache_data();
        #endif
        }
        GH3X2X_SetSoftEvent(GH3X2X_SOFT_EVENT_NEED_FORCE_READ_FIFO);
    }



#if __SUPPORT_FUNCTION_SAMPLE_RATE_MODIFY__
    Gh3x2xSampleRateUpdate(g_unDemoFuncMode);
#endif
    Gh3x2xSetCurrentSlotEnReg(g_unDemoFuncMode);
    EXAMPLE_LOG("Current g_unDemoFuncMode= 0x%lx\r\n",g_unDemoFuncMode);

    GH3X2X_EnterLowPowerMode();

#if __GS_NONSYNC_READ_EN__
    usTempFunction = Gh3x2xGetCurrentRefFunction();
    if(usTempFunction != g_usGsenorRefFunction)
    {
        g_usGsenorRefFunction = usTempFunction;
        Gh3x2xNonSyncGsensorInit();
    }
#endif
#if __GH3X2X_MEM_POOL_CHECK_EN__
    Gh3x2xUpdataMemPollChkSumAfterAlgoCal();
#endif
    if(GH3X2X_GetSoftEvent())
    {
        gh3020_fifo_process();
    }
}


void GH3X2X_StartHardAdtAndResetGsDetect(void)
{
    EXAMPLE_LOG("[%s]\r\n", __FUNCTION__);
    GH3X2X_ExitLowPowerMode();
    GH3X2X_FunctionStart(g_pstGh3x2xFrameInfo[GH3X2X_FUNC_OFFSET_ADT]);
#if (__FUNC_TYPE_SOFT_ADT_ENABLE__)
    g_uchHardAdtFuncStatus = 1;
    Gh3x2x_ResetMoveDetectByGsData();
#endif
    Gh3x2xSetCurrentSlotEnReg(GH3X2X_FUNCTION_ADT);
    GH3X2X_EnterLowPowerMode();
}

void GH3X2X_StopHardAdtAndStartGsDetect(void)
{
    EXAMPLE_LOG("[%s]\r\n", __FUNCTION__);
    GH3X2X_ExitLowPowerMode();
    GH3X2X_FunctionStop(g_pstGh3x2xFrameInfo[GH3X2X_FUNC_OFFSET_ADT]);
    uint16_t usGotEvent = 0;
    usGotEvent |= GH3X2X_IRQ_MSK_WEAR_OFF_BIT;
#if (__SUPPORT_HARD_ADT_CONFIG__)
    Gh3x2xDemoWearEventProcess(&usGotEvent,WEAR_DETECT_FORCE_SWITCH);
#endif
#if (__SUPPORT_PROTOCOL_ANALYZE__)
    Gh3x2xDemoReportEvent(usGotEvent,0);
#endif
#if (__FUNC_TYPE_SOFT_ADT_ENABLE__)
    g_uchHardAdtFuncStatus = 0;
    Gh3x2x_ResetMoveDetectByGsData();
#endif
    Gh3x2xSetCurrentSlotEnReg(g_unDemoFuncMode&(~GH3X2X_FUNCTION_ADT));
#if (__FUNC_TYPE_SOFT_ADT_ENABLE__)
    GH3X2X_START_ADT_TIMER();
#endif
    GH3X2X_EnterLowPowerMode();
}



void Gh3x2xFunctionSlotBitInit(void)
{
    for(uint8_t uchFunctionCnt = 0; uchFunctionCnt < GH3X2X_FUNC_OFFSET_MAX; uchFunctionCnt++)
    {
        if(g_pstGh3x2xFrameInfo[uchFunctionCnt])
        {
            GH3x2xCalFunctionSlotBit(g_pstGh3x2xFrameInfo[uchFunctionCnt]);
        }
    }
}

/**
 * @fn     void Gh3x2xDemoStartSampling()
 *
 * @brief  Start sampling of GH3x2x
 *
 * @attention   None
 *
 * @param[in]   unFuncMode      function mode that will start
 *                              GH3X2X_FUNCTION_ADT/GH3X2X_FUNCTION_HR/GH3X2X_FUNCTION_SPO2/GH3X2X_FUNCTION_ECG and etc.
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2xDemoStartSamplingInner(uint32_t unFuncMode)
{
  uint32_t unFuncModeTemp1 = GH3X2X_NO_FUNCTION;
  uint32_t unFuncModeTemp2 = GH3X2X_NO_FUNCTION;
  uint8_t  uchIndex = 0;
  int8_t  chRet = GH3X2X_RET_OK;

  EXAMPLE_LOG("[Gh3x2xDemoStartSampling] unFuncMode = 0x%X\r\n",(int)unFuncMode);

#if (__SUPPORT_HARD_ADT_CONFIG__)
  if (GH3X2X_FUNCTION_ADT == unFuncMode)
    {
      unFuncModeTemp2 = unFuncMode;
    }
  else
    {
      unFuncModeTemp2 = unFuncMode & (~GH3X2X_FUNCTION_ADT);
    }
#else
  unFuncModeTemp2 = unFuncMode;
#endif

  /* Decode functions in current reg config array */

  GH3X2X_DecodeRegCfgArr(&unFuncModeTemp1, (gh3020_initcfg_list + g_uchGh3x2xRegCfgArrIndex)->pregarr, \
                            (gh3020_initcfg_list + g_uchGh3x2xRegCfgArrIndex)->arrlen);

  /* If this function not exist in current reg cfg array, need switch reg cfg array */

  if ((unFuncModeTemp2 & unFuncModeTemp1) != unFuncModeTemp2)
    {
      chRet = GH3x2xDemoSearchCfgListByFunc(&uchIndex, unFuncMode, GH3X2X_FUNCTION_ADT);
      if (GH3X2X_RET_OK == chRet)
        {
          #if (__SUPPORT_HARD_ADT_CONFIG__)
          uint32_t unDemoFuncModeBeforeReset = g_unDemoFuncMode;
          EMWearDetectType  uchWearDetectStatusBeforeReset = g_uchWearDetectStatus;
          EXAMPLE_LOG("unDemoFuncModeBeforeReset = %ld\r\n",unDemoFuncModeBeforeReset);
          #endif
          Gh3x2xDemoSamplingControl(0xffffffff, UPROTOCOL_CMD_STOP);

          GH3X2X_HardReset();

          Gh3x2x_BspDelayMs(15);
          GH3X2X_ExitLowPowerMode();
          if (GH3X2X_RET_OK == GH3X2X_Init(gh3020_initcfg_list+uchIndex))
            {
              g_uchGh3x2xRegCfgArrIndex = uchIndex;
            }
          else
            {
              g_uchGh3x2xRegCfgArrIndex = DRV_LIB_REG_CFG_EMPTY;
            }

          Gh3x2xFunctionCtrlModuleInit();

        #if (__SUPPORT_HARD_ADT_CONFIG__)

          /* if ADT is running, recover ADT status */

          EXAMPLE_LOG("temp unDemoFuncModeBeforeReset = %ld\r\n",(unDemoFuncModeBeforeReset & GH3X2X_FUNCTION_ADT));
          if (GH3X2X_FUNCTION_ADT == (unDemoFuncModeBeforeReset & GH3X2X_FUNCTION_ADT))
            {
              EXAMPLE_LOG("g_uchWearDetectStatus = %d\r\n",g_uchWearDetectStatus);
              g_uchWearDetectStatus = uchWearDetectStatusBeforeReset;
              unFuncMode |= GH3X2X_FUNCTION_ADT;
              if(WEAR_DETECT_WEAR_OFF == g_uchWearDetectStatus)
                {
                  GH3X2X_FunctionStart(g_pstGh3x2xFrameInfo[GH3X2X_FUNC_OFFSET_ADT]);
                  #if __FUNC_TYPE_SOFT_ADT_ENABLE__
                  g_uchHardAdtFuncStatus = 1;
                  #endif
                  g_uchWearDetectStatus = WEAR_DETECT_WEAR_OFF;
                  GH3X2X_WearDetectSwitchTo(WEAR_DETECT_WEAR_OFF,WEAR_DETECT_FORCE_SWITCH);
                }
            }
        #endif
        }
      else
        {
          EXAMPLE_LOG("APP Mode Switch Error:Reg cfg file not exist!!!APP mode:0x%lx\r\n",unFuncMode);
        }
    }

    /* function slot bit init */

    Gh3x2xFunctionSlotBitInit();

#ifdef CONFIG_FACTEST_SENSORS_GH3020
  if(1 == g_uchEngineeringModeStatus)
    {
      GH3X2X_ExitLowPowerMode();
      GH3x2x_ChangeSampleParmForEngineeringMode(g_pstGh3x2xFrameInfo, unFuncMode, g_pstSampleParaGroup, g_uchEngineeringModeSampleParaGroupNum);
      GH3X2X_EnterLowPowerMode();
    }
#endif

  Gh3x2xDemoSamplingControl(unFuncMode, UPROTOCOL_CMD_START);
}



void gh3020_start_sampling(uint32_t unFuncMode)
{
    Gh3x2xDemoStartSamplingInner(unFuncMode);
}



#ifdef CONFIG_FACTEST_SENSORS_GH3020

/**
 * @fn     void gh3020_start_sampling_factest()
 *
 * @brief  Start sampling of GH3x2x for engineering mode
 *
 * @attention   None
 *
 * @param[in]   unFuncMode      function mode that will start
 *                              GH3X2X_FUNCTION_ADT/GH3X2X_FUNCTION_HR/GH3X2X_FUNCTION_SPO2/GH3X2X_FUNCTION_ECG and etc.
 * @param[out]  None
 *
 * @return  None
 */
void gh3020_start_sampling_factest(uint32_t unFuncMode, FAR struct gh3020_factestmode_param_s *pstSampleParaGroup , uint8_t uchSampleParaGroupNum)
{
    g_pstSampleParaGroup = pstSampleParaGroup;
    g_uchEngineeringModeSampleParaGroupNum = uchSampleParaGroupNum;
    g_uchEngineeringModeStatus = 1;
    Gh3x2xDemoStartSamplingInner(unFuncMode);
    g_unDemoFuncMode = GH3X2X_FUNCTION_TEST1;
}
#endif



/**
 * @fn     void Gh3x2xDemoStopSampling(uint32_t unFuncMode)
 *
 * @brief  Stop sampling of GH3x2x
 *
 * @attention   None
 *
 * @param[in]   unFuncMode      function mode that will start
 *                              GH3X2X_FUNCTION_ADT/GH3X2X_FUNCTION_HR/GH3X2X_FUNCTION_SPO2/GH3X2X_FUNCTION_ECG and etc.
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2xDemoStopSamplingInner(uint32_t unFuncMode)
{
    EXAMPLE_LOG("[Gh3x2xDemoStopSampling] unFuncMode = 0x%X\r\n",(int)unFuncMode);
    Gh3x2xDemoSamplingControl(unFuncMode, UPROTOCOL_CMD_STOP);
}

void gh3020_stop_sampling(uint32_t unFuncMode)
{
    Gh3x2xDemoStopSamplingInner(unFuncMode);
}


#ifdef CONFIG_FACTEST_SENSORS_GH3020

/**
 * @fn     void gh3020_stop_sampling_factest(void)
 *
 * @brief  Stop sampling of GH3x2x for engineering mode
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void gh3020_stop_sampling_factest(void)
{
    Gh3x2xDemoStopSamplingInner(g_unDemoFuncMode);

    Gh3x2xDemoSamplingControl(0xFFFFFFFF, UPROTOCOL_CMD_STOP);

    #if __SUPPORT_HARD_RESET_CONFIG__
    GH3X2X_HardReset();
    #else
    GH3X2X_SoftReset();
    #endif

    Gh3x2x_BspDelayMs(15);
    GH3X2X_ExitLowPowerMode();
    if (GH3X2X_RET_OK == GH3X2X_Init(&gh3020_initcfg_list[0]))
    {
        g_uchGh3x2xRegCfgArrIndex = 0;
    }
    else
    {
        g_uchGh3x2xRegCfgArrIndex = DRV_LIB_REG_CFG_EMPTY;
    }

    Gh3x2xFunctionCtrlModuleInit();

    g_uchEngineeringModeStatus = 0;
}
#endif


/**
 * @fn     void Gh3x2xDemoMoveDetectTimerHandler(void)
 *
 * @brief  Move detect timer handler callback
 *
 * @attention   if use adt with confirm mode,must call this function to detect movement
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2xDemoMoveDetectTimerHandler(void)
{
    #if (__FUNC_TYPE_SOFT_ADT_ENABLE__)
    //0.check CAP
    #if (__CAP_ENABLE__)
    Cap_drv_get_fifo_data(cap_soft_fifo_buffer, &cap_soft_fifo_buffer_index);
    #endif
    GH3X2X_MoveDetectByCapData(cap_soft_fifo_buffer, cap_soft_fifo_buffer_index);
    //1.check GS
    Gsensor_drv_get_fifo_data(gsensor_soft_fifo_buffer + __GS_EXTRA_BUF_LEN__, &gsensor_soft_fifo_buffer_index);
    Gh3x2x_NormalizeGsensorSensitivity(gsensor_soft_fifo_buffer + __GS_EXTRA_BUF_LEN__,gsensor_soft_fifo_buffer_index);
    GH3X2X_MoveDetectByGsData((int16_t*)(gsensor_soft_fifo_buffer + __GS_EXTRA_BUF_LEN__), gsensor_soft_fifo_buffer_index);
    if (GH3X2X_GetSoftWearOffDetEn())
    {
        EXAMPLE_LOG("move detecting...\r\n");
    }
    #endif
}

/**
 * @fn     void Gh3x2xDemoSetFuncionFrequency(uint8_t uchFunctionID, uint16_t usFrequencyValue)
 *
 * @brief  Set function frequency
 *
 * @attention   None
 *
 * @param[in]   uchFunctionID
 * @param[in]   usFrequencyValue
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2xDemoSetFuncionFrequency(uint8_t uchFunctionID, uint16_t usFrequencyValue)
{
    uint32_t unFuncModeTemp = 0;

    GH3X2X_DecodeRegCfgArr(&unFuncModeTemp, (gh3020_initcfg_list + g_uchGh3x2xRegCfgArrIndex)->pregarr, \
                            (gh3020_initcfg_list + g_uchGh3x2xRegCfgArrIndex)->arrlen);

    if (((1 << uchFunctionID) & unFuncModeTemp) == 0)
    {
        EXAMPLE_LOG("[%s]:Current cfg cannot support !!! cfgfunc = 0x%x, selectfuncs = 0x%x\r\n", __FUNCTION__, (int)unFuncModeTemp, (1 << uchFunctionID));
        return;
    }

    if (usFrequencyValue % 25 || usFrequencyValue > 1000)
    {
        EXAMPLE_LOG("[%s]:The SR is not a multiple of 25!!!srvalue = %d\r\n", __FUNCTION__, usFrequencyValue);
        return;
    }

    uint32_t unDemoFuncModeBeforeChipReset = g_unDemoFuncMode;

    if (unDemoFuncModeBeforeChipReset != 0)
    {
        gh3020_stop_sampling(unDemoFuncModeBeforeChipReset);
        EXAMPLE_LOG("[%s]:Stop.", __FUNCTION__);
        Gh3x2x_BspDelayMs(30);
    }

    GH3X2X_ExitLowPowerMode();
    GH3X2X_ModifyFunctionFrequency(uchFunctionID, usFrequencyValue);
    EXAMPLE_LOG("[%s]:Change fs !!! func = 0x%x, fs = %d\r\n", __FUNCTION__, (1 << uchFunctionID), usFrequencyValue);

    if (unDemoFuncModeBeforeChipReset != 0)
    {
        Gh3x2x_BspDelayMs(30);
        gh3020_start_sampling(unDemoFuncModeBeforeChipReset);
        EXAMPLE_LOG("[%s]:Restart.", __FUNCTION__);
    }
}

/**
 * @fn     void Gh3x2xDemoSetFuncionLedCurrent(uint8_t uchFunctionID, uint16_t usLedDrv0Current, uint16_t usLedDrv1Current)
 *
 * @brief  Set function led current
 *
 * @attention   None
 *
 * @param[in]   uchFunctionID               function offset
 * @param[in]   usLedDrv0Current         0mA-200mA
 * @param[in]   usLedDrv1Current         0mA-200mA
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2xDemoSetFuncionLedCurrent(uint8_t uchFunctionID, uint16_t usLedDrv0Current, uint16_t usLedDrv1Current)
{
    uint32_t unFuncModeTemp = 0;

    GH3X2X_DecodeRegCfgArr(&unFuncModeTemp, (gh3020_initcfg_list + g_uchGh3x2xRegCfgArrIndex)->pregarr, \
                            (gh3020_initcfg_list + g_uchGh3x2xRegCfgArrIndex)->arrlen);

    if (((1 << uchFunctionID) & unFuncModeTemp) == 0)
    {
        EXAMPLE_LOG("[%s]:Current cfg cannot support !!! cfgfunc = 0x%x, selectfuncs = 0x%x\r\n", __FUNCTION__, (int)unFuncModeTemp, (1 << uchFunctionID));
        return;
    }

    GH3X2X_ExitLowPowerMode();
    GH3X2X_ModifyFunctionLedCurrent(uchFunctionID, usLedDrv0Current, usLedDrv1Current);
    EXAMPLE_LOG("[%s]:Change led current !!! func = 0x%x, drv0 current = %d, drv1 current = %d\r\n",  __FUNCTION__, (1 << uchFunctionID), (int)usLedDrv0Current, (int)usLedDrv1Current);
}


#if (__SUPPORT_ELECTRODE_WEAR_STATUS_DUMP__)
void GH3X2X_ReadElectrodeWearDumpData(void)
{
    g_uchGh3x2xElectrodeWearStatus = gh3020_spi_readbits(0x050C, 1, 1);
}
#endif

void Gh3x2xSetFrameFlag2(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    if(0 == (*(pstFrameInfo->punFrameCnt)))
    {
        pstFrameInfo->punFrameFlag[2] |= 0x02;
    }
    #if (__SUPPORT_ELECTRODE_WEAR_STATUS_DUMP__)
    if(g_uchGh3x2xElectrodeWearStatus)
    {
        pstFrameInfo->punFrameFlag[2] |= 0x04;
    }
    #endif
}

#if    __SUPPORT_ALGO_INPUT_OUTPUT_DATA_HOOK_CONFIG__
STGh3x2xAlgoInfoRecordData g_StAlgoRecordData;
const STGh3x2xAlgoInfoRecordData * const g_pStAlgoRecordData = &g_StAlgoRecordData;
#else
const STGh3x2xAlgoInfoRecordData * const g_pStAlgoRecordData = (STGh3x2xAlgoInfoRecordData*)0;
#endif


#if 0 == __FUNC_TYPE_ECG_ENABLE__
void GH3X2X_EcgSampleHookHandle(uint8_t uchEventType, uint8_t uchEventInfo){}
void GH3X2X_LeadDetEnInHardAdt(uint8_t uchEn){}
void GH3X2X_LeadDetEnControl(uint8_t uchEventInfo){}
void GH3X2X_SlaverSoftLeadPramSet(uint16_t usRegVal, uint8_t uchRegPosi){}
void GH3X2X_SlaverSoftLeadPramInit(void){}
#else
    #if (__SUPPORT_ECG_LEAD_OFF_DET_800HZ__ == 0)
    int32_t GH3X2X_LeadOffDetect2Init(void){  GH3X2X_LeadOffDetectInit(); return 0;}
    uint8_t GH3X2X_LeadOffDetect2(uint8_t* puchFifoBuffer, uint16_t* pusFifoBufferLen){ return GH3X2X_LeadOffDetect(puchFifoBuffer, *pusFifoBufferLen);}
    #endif
#endif



#if (0 == __SUPPORT_SOFT_AGC_CONFIG__)
void GH3X2X_LedAgcPramWrite(uint16_t usRegAddr, uint16_t usRegData){}
void GH3X2X_LedAgcReset(void){}
void GH3X2X_LedAgcInit(void){}
void GH3X2X_LedAgcProcess(uint8_t* puchReadFifoBuffer, uint16_t usFifoLen){}
uint8_t GH3X2X_GetLedAgcState(void)
{
    return   0;
}
void GH3X2X_SoftLedADJAutoADJInt(void){}
#endif


#if (__NORMAL_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__ || __MIX_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__)
void Gh3x2xPollingModePro(void){}
#endif


#if 0 == GH3X2X_NEW_AGC_SUB_CHNL_NUM_LIMIT
void Gh3x2xNewAgcSubChnlModuleReset(void){}
uint8_t GH3x2xNewAgcSetNewSubChnl(uint8_t uchSlotNo, uint8_t uchRxEn){return 0;}
void GH3X2X_NewAgcSubChnlGainAdj(uint8_t* puchReadFifoBuffer, uint16_t usFifoLen){};
void GH3x2xNewAgcSubChnlClearAnalysisCntAndSetGain(uint8_t uchSlotNo, uint8_t uchRxEn ,uint8_t uchNextGain){};
void GH3x2xNewAgcSetSubChnlSlotInfo(uint8_t uchSlotCnt, uint16_t usSampleRate){};
#endif

void GH3x2xSlotTimeInfo(void)
{
#if 1 == __SLOT_SEQUENCE_DYNAMIC_ADJUST_EN__
    EXAMPLE_LOG("%s : Slot time: ", __FUNCTION__);
    for(uint8_t uchSlotCnt = 0; uchSlotCnt < 8; uchSlotCnt ++)
    {
        uint8_t uchCfgIndex;
        //get slot time infomation
        if(0 == (uchSlotCnt&0x01)) //0,2,4,6
        {
            uchCfgIndex = gh3020_spi_readbits(0x0100 + (uchSlotCnt&0xFE), 0,3);
        }
        else  //1,3,5,7
        {
            uchCfgIndex = gh3020_spi_readbits(0x0100 + (uchSlotCnt&0xFE), 8,11);
        }
        if(uchCfgIndex < 8)
        {
            g_usGh3x2xSlotTime[uchCfgIndex] = gh3020_spi_readreg(0x01EC + uchCfgIndex *2);
            EXAMPLE_LOG("[%d,%d] ", uchCfgIndex,g_usGh3x2xSlotTime[uchCfgIndex]);
        }
    }
    EXAMPLE_LOG("\r\n");
#endif
}

//function __weak api
#if (0 == __USE_GOODIX_HR_ALGORITHM__)
void GH3X2X_SetHbAlgoOutMaxCnt(uint16_t usAlgoOutMaxCnt){}
void GH3X2X_WriteHrAlgConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue){}
#endif
#if (0 == __USE_GOODIX_HRV_ALGORITHM__)
void GH3X2X_SetHrvAlgoOutMaxCnt(uint16_t usAlgoOutMaxCnt){}
void GH3X2X_WriteHrvAlgConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue){}
#endif
#if (0 == __USE_GOODIX_SPO2_ALGORITHM__)
void GH3X2X_SetSpo2AlgoOutMaxCnt(uint16_t usAlgoOutMaxCnt){}
void GH3X2X_WriteSpo2AlgConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue){}
#endif
#if (0 == __USE_GOODIX_ECG_ALGORITHM__)
uint8_t  GH3x2xEcgAlgoClassification(void){return GH3X2X_RET_OK;}
void GH3X2X_SetEcgAlgoOutMaxCnt(uint16_t usAlgoOutMaxCnt){}
#endif
#if (0 == __FUNC_TYPE_ECG_ENABLE__)
void GH3X2X_WriteEcgDrvConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue){}
#endif
#if ((0 == __USE_GOODIX_SOFT_ADT_ALGORITHM__) || (0 == __FUNC_TYPE_SOFT_ADT_ENABLE__))
#endif
#if (0 == __USE_GOODIX_HSM_ALGORITHM__)
#endif
#if (0 == __USE_GOODIX_BT_ALGORITHM__)
void GH3X2X_WriteBtAlgConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue){}
#endif
#if (0 == __USE_GOODIX_RESP_ALGORITHM__)
#endif
#if (0 == __USE_GOODIX_AF_ALGORITHM__)
void GH3X2X_WriteAfAlgConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue){}
#endif
#if (0 == __USE_GOODIX_BP_ALGORITHM__)
void GH3X2X_WriteBpAlgConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue){}
void Gh3x2x_BpAlgorithmPersonalInfoInput(EMBpModeType EMBpMode, STGh3x2xBpAlgoInput *pstAlgoInput){}
int8_t GH3X2X_BpParameterOperationFunc(int8_t (*pBpParameterWriteFunc)(int32_t *buffer), int8_t (*pBpParameterReadFunc)(int32_t *buffer)){return 0;}
#endif
#if (0 == __FIFO_PACKAGE_SEND_ENABLE__)
uint8_t GH3X2X_GetFifoPackageMode(void){return 0;}
void GH3X2X_SendRawdataFifoPackage(uint8_t *puchGh3x2xReadFifoData, uint16_t usFifoReadByteNum){}
#endif
#if (0 == __CAP_ENABLE__)
uint8_t GH3X2X_GetCapEnableFlag(void)
{
    return 0;
}
#endif
#if (0 == __TEMP_ENABLE__)
uint8_t GH3X2X_GetTempEnableFlag(void)
{
    return 0;
}
#endif

/*__weak void GH3X2X_Log(char *log_string)
{
}*/





/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/
