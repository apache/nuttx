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
#include "gh3x2x.h"
#include "gh3020_bridge.h"

static const uint16_t gh3x2x_gain_list[13] = {
    10, 25, 50, 75, 100, 250, 500, 750, 1000, 1250, 1500, 1750, 2000
};

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
    GU8  ubFlagLedAdjAgcUp;              //adj down flag of ppg data   0: down  1:up
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
void Gh3x2x_WearEventHook(GU16 usGotEvent, GU8 uchExentEx)
{
    if (usGotEvent & GH3X2X_IRQ_MSK_WEAR_OFF_BIT)
    {
        //Gh3x2xDemoStopSampling(g_unDemoFuncMode & (~GH3X2X_FUNCTION_ADT));
        GOODIX_PLANFROM_WEAR_OFF_EVENT();
        EXAMPLE_LOG("Wear off, no object!!!\r\n");
    }
    else if (usGotEvent & GH3X2X_IRQ_MSK_WEAR_ON_BIT)
    {
        gh3020_start_sampling(GH3X2X_FUNCTION_SOFT_ADT_IR);
        GOODIX_PLANFROM_WEAR_ON_EVENT();
        EXAMPLE_LOG("Wear on, object !!!\r\n");
    }
}
#endif

typedef struct{
    GS32 ppg_gr1_raw[__HR_ALGORITHM_SUPPORT_CHNL_NUM__];
    GS32 ppg_rd1_raw[__SPO2_ALGORITHM_SUPPORT_CHNL_NUM__];
    GS32 ppg_ir1_raw[__SPO2_ALGORITHM_SUPPORT_CHNL_NUM__];
    GS32 amb_raw[__SPO2_ALGORITHM_SUPPORT_CHNL_NUM__];

    /*
    ppg_status数据格式：
    [15:0]   Gain,数据范围：0~12，分别表示[10,25,50,75,100,250,500,750,1000,1250,1500,1750,2000kΩ]
    [31:16]  led电流，数据范围：0~400mA;
    */
    GU32 ppg_gr1_status[__HR_ALGORITHM_SUPPORT_CHNL_NUM__];
    GU32 ppg_rd1_status[__SPO2_ALGORITHM_SUPPORT_CHNL_NUM__];
    GU32 ppg_ir1_status[__SPO2_ALGORITHM_SUPPORT_CHNL_NUM__];

}af_sensor_ppg_t;
af_sensor_ppg_t ppg_data = {0};

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
    struct sensor_event_ppgq ppg;
    GS32 raw_temp = 0;
    GU32 gain_temp = 0;
    GU32 current_temp = 0;

    //green
    if((int)(pstFrameInfo->unFunctionID) & GH3X2X_FUNCTION_HR)
    {
        for(int i = 0;i < __HR_ALGORITHM_SUPPORT_CHNL_NUM__;i ++)
        {
            raw_temp = (GS32)(pstFrameInfo->punFrameRawdata[i]) - 0x800000;

            //[15:0]:gain,0~12;[10,25,50,75,100,250,500,750,1000,1250,1500,1750,2000kΩ].
            //[31:16]:LED current,0~200mA.
            gain_temp = (GU32)pstFrameInfo->punFrameAgcInfo[i] & 0x0000000F;
            current_temp = ((((GU32)pstFrameInfo->punFrameAgcInfo[i] & 0x0000FF00) >> 8) * 200 / 255) +
                        ((((GU32)pstFrameInfo->punFrameAgcInfo[i] & 0x00FF0000) >> 16) * 200 / 255);
            ppg.gain[i] = gh3x2x_gain_list[gain_temp];
            if (raw_temp > 0)
            {
                ppg.ppg[i] = (uint32_t)raw_temp;
            }
            else
            {
                ppg.ppg[i] = 0;
            }
        }
        ppg.current = current_temp * 1000;
        gh3020_transdata(&ppg, GH3020_PPG0_SENSOR_IDX);
    }

    //red
    if((int)(pstFrameInfo->unFunctionID) & GH3X2X_FUNCTION_SPO2)
    {
        for(int i = 0;i < __SPO2_ALGORITHM_SUPPORT_CHNL_NUM__;i ++)
        {
            raw_temp = (GS32)(pstFrameInfo->punFrameRawdata[i]) - 0x800000;

            //[15:0]:gain,0~12;[10,25,50,75,100,250,500,750,1000,1250,1500,1750,2000kΩ].
            //[31:16]:LED current,0~200mA.
            gain_temp = (GU32)pstFrameInfo->punFrameAgcInfo[i] & 0x0000000F;
            current_temp = ((((GU32)pstFrameInfo->punFrameAgcInfo[i] & 0x0000FF00) >> 8) * 200 / 255) +
                        ((((GU32)pstFrameInfo->punFrameAgcInfo[i] & 0x00FF0000) >> 16) * 200 / 255);
            ppg.gain[i] = gh3x2x_gain_list[gain_temp];
            if (raw_temp > 0)
            {
                ppg.ppg[i] = (uint32_t)raw_temp;
            }
            else
            {
                ppg.ppg[i] = 0;
            }
        }
        ppg.current = current_temp * 1000;
        gh3020_transdata(&ppg, GH3020_PPG1_SENSOR_IDX);
    }

    //ir
    if((int)(pstFrameInfo->unFunctionID) & GH3X2X_FUNCTION_HRV)
    {
        for(int i = 0;i < __SPO2_ALGORITHM_SUPPORT_CHNL_NUM__;i ++)
        {
            raw_temp = (GS32)(pstFrameInfo->punFrameRawdata[i]) - 0x800000;

            //[15:0]:gain,0~12;[10,25,50,75,100,250,500,750,1000,1250,1500,1750,2000kΩ].
            //[31:16]:LED current,0~200mA.
            gain_temp = (GU32)pstFrameInfo->punFrameAgcInfo[i] & 0x0000000F;
            current_temp = ((((GU32)pstFrameInfo->punFrameAgcInfo[i] & 0x0000FF00) >> 8) * 200 / 255) +
                        ((((GU32)pstFrameInfo->punFrameAgcInfo[i] & 0x00FF0000) >> 16) * 200 / 255);
            ppg.gain[i] = gh3x2x_gain_list[gain_temp];
            if (raw_temp > 0)
            {
                ppg.ppg[i] = (uint32_t)raw_temp;
            }
            else
            {
                ppg.ppg[i] = 0;
            }
        }
        ppg.current = current_temp * 1000;
        gh3020_transdata(&ppg, GH3020_PPG2_SENSOR_IDX);
    }

    //awb
    if((int)(pstFrameInfo->unFunctionID) & GH3X2X_FUNCTION_RESP)
    {
        for(int i = 0;i < __SPO2_ALGORITHM_SUPPORT_CHNL_NUM__;i ++)
        {
            gain_temp = (GU32)pstFrameInfo->punFrameAgcInfo[i] & 0x0000000F;
            raw_temp = (GS32)(pstFrameInfo->punFrameRawdata[i]) - 0x800000;
            ppg.gain[i] = gh3x2x_gain_list[gain_temp];
            if (raw_temp > 0)
            {
                ppg.ppg[i] = (uint32_t)raw_temp;
            }
            else
            {
                ppg.ppg[i] = 0;
            }
        }
        ppg.current = 0;
        gh3020_transdata(&ppg, GH3020_PPG3_SENSOR_IDX);
    }
}
#endif

void gh3x2x_active_reset_hook(void)
{
    Gh3x2x_BspDelayMs(20);
    gh3020_fifo_process();
    EXAMPLE_LOG("[%s]:handle protocol reset\r\n", __FUNCTION__);
}

void gh3x2x_protocol_ctrl_timer_handle(EMUprotocolParseCmdType emCmdType)
{
    if (UPROTOCOL_CMD_START == emCmdType)
    {
        EXAMPLE_LOG("[%s]:start polling timer\r\n", __FUNCTION__);
        //start polling timer
    }
    else if (UPROTOCOL_CMD_STOP == emCmdType)
    {
        EXAMPLE_LOG("[%s]:stop polling timer\r\n", __FUNCTION__);
        //stop polling timer
    }
}
