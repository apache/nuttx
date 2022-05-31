/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_demo_user.c
 *
 * @brief   gh3x2x driver lib demo code that user defined
 *
 * @author
 *
 */

/* includes */
#include <stdint.h>
#include <string.h>

#include "gh3x2x_drv.h"
#include "gh3x2x_config.h"
#include "gh3x2x_inner.h"
#include "gh3x2x.h"
#include "gh3020_bridge.h"
#if (__GH3X2X_MP_MODE__)
#include "gh3x2x_mp_common.h"
#endif


/**
 * @fn     void hal_gh3x2x_reset_pin_ctrl(uint8_t pin_level)
 *
 * @brief  hal reset pin ctrl for gh3x2x
 *
 * @attention   pin level set 1 [high level] or 0 [low level]
 *
 * @param[in]   pin_level     reset pin level
 * @param[out]  None
 *
 * @return  None
 */

void hal_gh3x2x_reset_pin_ctrl(uint8_t pin_level)
{
  gh3020_rstctrl((uint8_t)pin_level);
}


#if (__NORMAL_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__ || __MIX_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__)

/**
 * @fn     void hal_gh3x2x_int_init(void)
 *
 * @brief  gh3x2x int init
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void hal_gh3x2x_int_init(void)
{
    GOODIX_PLANFROM_INT_INIT_ENTITY();
}


/**
 * @fn     void gh3x2x_int_handler_call_back(void)
 *
 * @brief  call back of gh3x2x int handler
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void gh3x2x_int_handler_call_back(void)
{
    if (Gh3x2xGetInterruptMode() == __NORMAL_INT_PROCESS_MODE__)
    {
        GH3x2x_SetNeedWakeUpGh3x2xFlag(GH3x2x_GetNeedWakeUpGh3x2xFlagBeforeInt());  //not need wake up gh3x2x when process int event
        g_uchGh3x2xIntCallBackIsCalled = 1;
#if (__GH3X2X_MP_MODE__)
        GH3X2X_MP_SET_INT_FLAG();  //gh3x2x mp test must call it
#endif
        GOODIX_PLANFROM_INT_HANDLER_CALL_BACK_ENTITY();
    }
}
#endif

/**
 * @fn     void Gsensor_start_cache_data(void)
 *
 * @brief  Start cache gsensor data for gh3x2x
 *
 * @attention   This function will be called when start gh3x2x sampling.
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void Gsensor_start_cache_data(void)
{
    GOODIX_PLANFROM_INT_GS_START_CACHE_ENTITY();
}

/**
 * @fn     void Gsensor_stop_cache_data(void)
 *
 * @brief  Stop cache gsensor data for gh3x2x
 *
 * @attention   This function will be called when stop gh3x2x sampling.
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void Gsensor_stop_cache_data(void)
{

    GOODIX_PLANFROM_INT_GS_STOP_CACHE_ENTITY();
}

void Cap_start_cache_data(void){GOODIX_PLANFROM_INT_CAP_START_CACHE_ENTITY();}
void Cap_stop_cache_data(void){GOODIX_PLANFROM_INT_CAP_STOP_CACHE_ENTITY();}
void Temp_start_cache_data(void){GOODIX_PLANFROM_INT_TEMP_START_CACHE_ENTITY();}
void Temp_stop_cache_data(void){GOODIX_PLANFROM_INT_TEMP_STOP_CACHE_ENTITY() ;}

void hal_gh3x2x_write_cap_to_flash(int32_t WearCap1,int32_t UnwearCap1,int32_t WearCap2,int32_t UnwearCap2)
{
    GOODIX_PLANFROM_WRITE_CAP_TO_FLASH_ENTITY();
}
void hal_gh3x2x_read_cap_from_flash(int32_t* WearCap1,int32_t* UnwearCap1,int32_t* WearCap2,int32_t* UnwearCap2)
{
    GOODIX_PLANFROM_READ_CAP_FROM_FLASH_ENTITY();
}

/**
 * @fn     void Gsensor_drv_get_fifo_data(STGsensorRawdata gsensor_buffer[], uint16_t *gsensor_buffer_index)
 *
 * @brief  get gsensor fifo data
 *
 * @attention   When read fifo data of GH3x2x, will call this function to get corresponding cached gsensor data.
 *
 * @param[in]   None
 * @param[out]  gsensor_buffer          pointer to gsensor data buffer
 * @param[out]  gsensor_buffer_index    pointer to number of gsensor data(every gsensor data include x,y,z axis data)
 *
 * @return  None
 */
void Gsensor_drv_get_fifo_data(STGsensorRawdata gsensor_buffer[], uint16_t *gsensor_buffer_index)
{
/**************************** WARNNING  START***************************************************/
/*  (*gsensor_buffer_index) can not be allowed bigger than __GSENSOR_DATA_BUFFER_SIZE__  ****************/
/* Be care for copying data to gsensor_buffer, length of gsensor_buffer is __GSENSOR_DATA_BUFFER_SIZE__ *****/
/**************************** WARNNING END*****************************************************/

    GOODIX_PLANFROM_INT_GET_GS_DATA_ENTITY();



/**************************** WARNNING: DO NOT REMOVE OR MODIFY THIS CODE   ---START***************************************************/
    if((*gsensor_buffer_index) > (__GSENSOR_DATA_BUFFER_SIZE__))
    {
        while(1);   // Fatal error !!!
    }
/**************************** WARNNING: DO NOT REMOVE OR MODIFY THIS CODE   ---END***************************************************/
}

void Cap_drv_get_fifo_data(STCapRawdata cap_data_buffer[], uint16_t *cap_buffer_index)
{
    GOODIX_PLANFROM_INT_GET_CAP_DATA_ENTITY()  ;
    if((*cap_buffer_index) > (__CAP_DATA_BUFFER_SIZE__))
    {
        while(1);   // Fatal error !!!
    }
}
void Temp_drv_get_fifo_data(STTempRawdata temp_data_buffer[], uint16_t *temp_buffer_index)
{
    GOODIX_PLANFROM_INT_GET_TEMP_DATA_ENTITY();
    if((*temp_buffer_index) > (__TEMP_DATA_BUFFER_SIZE__))
    {
        while(1);   // Fatal error !!!
    }
}

#if (__EXAMPLE_LOG_CONFIG__)
/**
 * @fn     void GH3X2X_Log(char *log_string)
 *
 * @brief  for debug version, log
 *
 * @attention   this function must define that use debug version lib
 *
 * @param[in]   log_string      pointer to log string
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_Log(char *log_string)
{
//   syslog(LOG_DEBUG, log_string);
}
#endif

/**
 * @fn     void Gh3x2x_BspDelayUs(uint16_t usUsec)
 *
 * @brief  Delay in us,user should register this function into driver lib
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */

void Gh3x2x_BspDelayUs(uint16_t usUsec)
{
  usleep(usUsec);
}

void GH3X2X_AdtFuncStartWithConfirmHook(void)
{
    GOODIX_PLANFROM_START_WITH_CONFIRM_HOOK_ENTITY();
}

/**
 * @fn     void Gh3x2x_BspDelayMs(uint16_t usMsec)
 *
 * @brief  Delay in ms
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_BspDelayMs(uint16_t usMsec)
{
  usleep(usMsec * 1000);
}

#if (__FUNC_TYPE_SOFT_ADT_ENABLE__)
/**
 * @fn     void Gh3x2x_CreateAdtConfirmTimer(void)
 *
 * @brief  Create a timer for adt confirm which will read gsensor data periodically
 *
 * @attention   Period of timer can be set by
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
#if (__USE_POLLING_TIMER_AS_ADT_TIMER__)&&\
    (__POLLING_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__ || __MIX_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__)
#else
void Gh3x2xCreateAdtConfirmTimer(void)
{
    GOODIX_PLANFROM_CREAT_ADT_CONFIRM_ENTITY();
}
#endif

/**
 * @fn     void Gh3x2x_StartAdtConfirmTimer(void)
 *
 * @brief  Start time of adt confirm to get g sensor
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
#if __FUNC_TYPE_SOFT_ADT_ENABLE__
#if (__USE_POLLING_TIMER_AS_ADT_TIMER__)&&\
    (__POLLING_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__ || __MIX_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__)
#else
void Gh3x2x_StartAdtConfirmTimer(void)
{
    GOODIX_PLANFROM_START_TIMER_ENTITY();
}
#endif
#endif

/**
 * @fn     void Gh3x2x_StopAdtConfirmTimer(void)
 *
 * @brief  Stop time of adt confirm
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
#if __FUNC_TYPE_SOFT_ADT_ENABLE__
#if (__USE_POLLING_TIMER_AS_ADT_TIMER__)&&\
    (__POLLING_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__ || __MIX_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__)
#else
void Gh3x2x_StopAdtConfirmTimer(void)
{
    GOODIX_PLANFROM_STOP_TIMER_ENTITY();
}
#endif
#endif
#endif

/**
 * @fn     void gh3020_set_fifowtm
 *
 * @brief  set FIFO watermark, unit is data (4 bytes per data, 4 data per sample)
 *
 * @attention   None
 *
 * @param[in]   fifowtm: desired FIFO watermark
 * @param[out]  None
 *
 * @return  None
 */

void gh3020_set_fifowtm(uint16_t fifowtm)
{
  GH3X2X_FifoWatermarkThrConfig(fifowtm);
}

/**
 * @fn     void Gh3x2x_UserHandleCurrentInfo(void)
 *
 * @brief  handle gh3x2x chip current information for user
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_UserHandleCurrentInfo(void)
{
    //read or write  slotx current

    //GH3X2X_GetSlotLedCurrent(0,0); //read slot 0  drv 0
    //GH3X2X_GetSlotLedCurrent(1,0); // read  slot 1  drv 0

    //GH3X2X_SlotLedCurrentConfig(0,0,50);  //set slot0 drv0 50 LSB
    //GH3X2X_SlotLedCurrentConfig(1,0,50);  //set slot1 drv0 50 LSB
}

//hr
#if (0 == __USE_GOODIX_HR_ALGORITHM__)
int8_t GH3x2xHrAlgoInit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
int8_t GH3x2xHrAlgoDeinit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xHrAlgoExe(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
}
#endif

//hrv
#if (0 == __USE_GOODIX_HRV_ALGORITHM__)
int8_t GH3x2xHrvAlgoInit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
int8_t GH3x2xHrvAlgoDeinit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xHrvAlgoExe(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
}
#endif

//spo2
#if (0 == __USE_GOODIX_SPO2_ALGORITHM__)
int8_t GH3x2xSpo2AlgoInit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
int8_t GH3x2xSpo2AlgoDeinit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xSpo2AlgoExe(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
}
#endif

//ecg
#if (0 == __USE_GOODIX_ECG_ALGORITHM__)
int8_t GH3x2xEcgAlgoInit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
int8_t GH3x2xEcgAlgoDeinit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xEcgAlgoExe(const struct gh3020_frameinfo_s * const pstFrameInfo)
{

}
#endif

//soft adt
#if ((0 == __USE_GOODIX_SOFT_ADT_ALGORITHM__) || (0 == __FUNC_TYPE_SOFT_ADT_ENABLE__))
int8_t GH3x2xSoftAdtAlgoInit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
int8_t GH3x2xSoftAdtAlgoDeinit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xSoftAdtAlgoExe(const struct gh3020_frameinfo_s * const pstFrameInfo)
{

}
#endif

//hsm
#if (0 == __USE_GOODIX_HSM_ALGORITHM__)
int8_t GH3x2xHsmAlgoInit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
int8_t GH3x2xHsmAlgoDeinit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xHsmAlgoExe(const struct gh3020_frameinfo_s * const pstFrameInfo)
{

}
#endif

//bt
#if (0 == __USE_GOODIX_BT_ALGORITHM__)
int8_t GH3x2xBtAlgoInit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
int8_t GH3x2xBtAlgoDeinit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xBtAlgoExe(const struct gh3020_frameinfo_s * const pstFrameInfo)
{

}
#endif

//resp
#if (0 == __USE_GOODIX_RESP_ALGORITHM__)
int8_t GH3x2xRespAlgoInit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
int8_t GH3x2xRespAlgoDeinit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xRespAlgoExe(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
}
#endif

//af
#if (0 == __USE_GOODIX_AF_ALGORITHM__)
int8_t GH3x2xAfAlgoInit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
int8_t GH3x2xAfAlgoDeinit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xAfAlgoExe(const struct gh3020_frameinfo_s * const pstFrameInfo)
{

}
#endif

//bp
#if (0 == __USE_GOODIX_BP_ALGORITHM__)
int8_t GH3x2xBpAlgoInit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
int8_t GH3x2xBpAlgoDeinit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xBpAlgoExe(const struct gh3020_frameinfo_s * const pstFrameInfo)
{

}
#endif


/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/
