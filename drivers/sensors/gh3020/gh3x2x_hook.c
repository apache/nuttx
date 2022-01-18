/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_demo_hook.c
 *
 * @brief   gh3x2x driver lib demo code for hook
 *
 * @author  Gooidx Iot Team
 *
 */

#include "gh3x2x_config.h"
#include "gh3x2x_inner.h"
#include "gh3020_bridge.h"

/* hook functions */

/**
 * @fn     void gh3x2x_init_hook_func(void)
 *
 * @brief  gh3x2x init hook
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void gh3x2x_init_hook_func(void)
{

    /* code implement by user */

}

/**
 * @fn     void gh3x2x_sampling_start_hook_func(void)
 *
 * @brief  gh3x2x start hook
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void gh3x2x_sampling_start_hook_func(void)
{



    GOODIX_PLANFROM_SAMPLING_START_HOOK_ENTITY();

}

/**
 * @fn     void gh3x2x_sampling_stop_hook_func(void)
 *
 * @brief  gh3x2x stop hook
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void gh3x2x_sampling_stop_hook_func(void)
{

  GOODIX_PLANFROM_SAMPLING_STOP_HOOK_ENTITY();
}

/**
 * @fn     void gh3x2x_get_rawdata_hook_func(GU8 *read_buffer_ptr, GU16 length)
 *
 * @brief  gh3x2x get rawdata hook
 *
 * @attention   None
 *
 * @param[in]   read_buffer_ptr     pointer to read rawdata buffer
 * @param[in]   length              length
 * @param[out]  None
 *
 * @return  None
 */
 typedef struct
{
    GU32 uiAdcCode;                      //sampling rawdata of ADC
    GU8  ubSlotNo;                       //slot number
    GU8  ubAdcNo;                        //adc number
    GU8  ubFlagLedAdjIsAgc_EcgRecover;   //adj flag of ppg data or fast recover flag of ecg data
    GU8  ubFlagLedAdjAgcUp;                 //adj down flag of ppg data   0: down  1:up
}StFifoDataInformation;

void gh3x2x_get_rawdata_hook_func(GU8 *read_buffer_ptr, GU16 length)
{
    /* code implement by user */
    /****************** FOLLOWING CODE IS EXAMPLE **********************************/
#if (__SUPPORT_ENGINEERING_MODE__)
    struct sensor_event_ppgq ppg[4];
    StFifoDataInformation stTempFifoInfo = {0};
    if(length/4 > 0)
    {
        EXAMPLE_LOG("length = %d \r\n", length/4);
        GU32 temp = 0;
        GS32 rawdata = 0;
        for(int i = 0;i < length;i += 4)
        {
            // big endian to little endian
            temp =  ((GU32)read_buffer_ptr[i+ 0]) << 24;
            temp += ((GU32)read_buffer_ptr[i+ 1])<< 16;
            temp += ((GU32)read_buffer_ptr[i+ 2]) << 8;
            temp += ((GU32)read_buffer_ptr[i+ 3]);
            // pick rawdata and flag
            stTempFifoInfo.uiAdcCode = ((temp >> 0) & 0x00FFFFFF);
            stTempFifoInfo.ubSlotNo =  ((temp >> 29) & 0x00000007);
            stTempFifoInfo.ubAdcNo = ((temp >> 27) & 0x00000003);
            stTempFifoInfo.ubFlagLedAdjIsAgc_EcgRecover = ((temp >> 26) & 0x00000001);
            stTempFifoInfo.ubFlagLedAdjAgcUp = ((temp >> 25) & 0x00000001);
            rawdata = (GS32)(stTempFifoInfo.uiAdcCode) - 0x800000;
            if(rawdata < 0)
            {
                rawdata = 0;
            }
            if(GH3X2X_FUNCTION_TEST1 == g_unDemoFuncMode)
            {
                switch(stTempFifoInfo.ubSlotNo)
                {
                    case 0x00:  //slot0-dark
                        ppg[3].ppg[stTempFifoInfo.ubAdcNo] = (uint32_t)rawdata;
                        break;
                    case 0x01:  //slot1-green
                        ppg[0].ppg[stTempFifoInfo.ubAdcNo] = (uint32_t)rawdata;
                        break;
                    case 0x02:  //slot2-red
                        ppg[1].ppg[stTempFifoInfo.ubAdcNo] = (uint32_t)rawdata;
                        break;
                    case 0x03:  //slot3-ir
                        ppg[2].ppg[stTempFifoInfo.ubAdcNo] = (uint32_t)rawdata;
                        break;
                    default :
                        break;
                }
            }
        }

        if(GH3X2X_FUNCTION_TEST1 == g_unDemoFuncMode)
        {
            gh3020_transdata(&ppg[0], 0);
            gh3020_transdata(&ppg[1], 1);
            gh3020_transdata(&ppg[2], 2);
            gh3020_transdata(&ppg[3], 3);
        }
    }
#endif
}


#if (__FUNC_TYPE_ECG_ENABLE__)
/**
 * @fn     void Gh3x2x_LeadOnEventHook(void)
 *
 * @brief  Lead on event hook
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_LeadOnEventHook(void)
{
        GOODIX_PLANFROM_LEAD_ON_EVENT();
}

/**
 * @fn     void Gh3x2x_LeadOffEventHook(void)
 *
 * @brief  Lead off event hook
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_LeadOffEventHook(void)
{
        GOODIX_PLANFROM_LEAD_OFF_EVENT();
}
#endif


#if (__SUPPORT_HARD_ADT_CONFIG__)

/**
 * @fn     extern void Gh3x2x_WearEventHook(GU16 usGotEvent, GU8 uchWearOffType, GU8 uchSoftAdtFlag);
 *
 * @brief  Wear event hook
 *
 * @attention   None
 *
 * @param[in]   wear event
 * @param[in]   wear off type  0: no object  1: nonliving object    wear on type   0: object     1: living object
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_WearEventHook(GU16 usGotEvent, GU8 uchType,GU8 uchSoftAdtFlag, GU8 uchLivingConfi)
{


    EXAMPLE_LOG("uchSoftAdtFlag = 0x%X\r\n",  uchSoftAdtFlag);

    if(uchSoftAdtFlag&0x01)  // soft adt time out
    {
        EXAMPLE_LOG("Soft adt time out !!!\r\n");
    }

    if(uchSoftAdtFlag&0x02)  // maybe wear off
    {
        EXAMPLE_LOG("Soft adt maybe wear off\r\n");
    }


    if (usGotEvent & GH3X2X_IRQ_MSK_WEAR_OFF_BIT)
    {
        GOODIX_PLANFROM_WEAR_OFF_EVENT();
        if(0 == uchType)
        {
            EXAMPLE_LOG("Wear off, no object!!!\r\n");
        }
        else if(1 == uchType)
        {
            EXAMPLE_LOG("Wear off, nonliving object!!!\r\n");
        }

    }
    else if (usGotEvent & GH3X2X_IRQ_MSK_WEAR_ON_BIT)
    {
        if(0 == uchType)
        {
            GOODIX_PLANFROM_WEAR_ON_EVENT();
            EXAMPLE_LOG("Wear on, object !!!\r\n");
        }
        else if(1 == uchType)
        {
            EXAMPLE_LOG("Wear on, living object !!!\r\n");
        }
    }
}
#endif


/**
 * @fn      void gh3x2x_algorithm_get_io_data_hook_func(const STGh3x2xFrameInfo * const pstFrameInfo)
 *
 * @brief  get algorithm input and output data
 *
 * @attention   None
 *
 * @param[in]   pstFrameInfo
 * @param[out]  None
 *
 * @return  None
 */
#if    __SUPPORT_ALGO_INPUT_OUTPUT_DATA_HOOK_CONFIG__
void gh3x2x_algorithm_get_io_data_hook_func(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    /****************** FOLLOWING CODE IS EXAMPLE **********************************/
#if 0
    //function id and channel num
    EXAMPLE_LOG("[IO_DATA]Function ID: 0x%X, channel num = %d, frame cnt = %d\r\n",(int)(pstFrameInfo->unFunctionID),(int)(pstFrameInfo->pstFunctionInfo->uchChnlNum),(int)(pstFrameInfo->punFrameCnt[0]));
    //gsensor data
    EXAMPLE_LOG("[IO_DATA]Gsensor: x = %d, y = %d, z = %d\r\n",\
                        (int)(pstFrameInfo->pusFrameGsensordata[0]),\
                        (int)(pstFrameInfo->pusFrameGsensordata[1]),\
                        (int)(pstFrameInfo->pusFrameGsensordata[2])\
                );
    //rawdata
    for(GU8 uchChnlCnt = 0; uchChnlCnt < pstFrameInfo->pstFunctionInfo->uchChnlNum; uchChnlCnt ++)
    {
        EXAMPLE_LOG("[IO_DATA]Ch%d rawdata= %d\r\n",(int)(uchChnlCnt),(int)(pstFrameInfo->punFrameRawdata[uchChnlCnt]));
    }
    //algorithm result
    if(pstFrameInfo->pstAlgoResult->uchUpdateFlag)
    {
        EXAMPLE_LOG("[IO_DATA]algorithm result0= %d\r\n",(int)pstFrameInfo->pstAlgoResult->snResult[0]);
    }
#endif
}
#endif

