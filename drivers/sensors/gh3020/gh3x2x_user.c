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

#if ( __GH3X2X_INTERFACE__ == __GH3X2X_INTERFACE_I2C__ )

/* i2c interface */
/**
 * @fn     void hal_gh3x2x_i2c_init(void)
 *
 * @brief  hal i2c init for gh3x2x
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void hal_gh3x2x_i2c_init(void)
{

    /* code implement by user */
    GOODIX_PLANFROM_I2C_INIT_ENTITY();

}

/**
 * @fn     uint8_t hal_gh3x2x_i2c_write(uint8_t device_id, const uint8_t write_buffer[], uint16_t length)
 *
 * @brief  hal i2c write for gh3x2x
 *
 * @attention   device_id is 8bits, if platform i2c use 7bits addr, should use (device_id >> 1)
 *
 * @param[in]   device_id       device addr
 * @param[in]   write_buffer    write data buffer
 * @param[in]   length          write data len
 * @param[out]  None
 *
 * @return  status
 * @retval  #1      return successfully
 * @retval  #0      return error
 */
GU8 hal_gh3x2x_i2c_write(GU8 device_id, const GU8 write_buffer[], GU16 length)
{
    uint8_t ret = 1;

    /* code implement by user */

    GOODIX_PLANFROM_I2C_WRITE_ENTITY(device_id, write_buffer,length);
    return ret;
}

/**
 * @fn     uint8_t hal_gh3x2x_i2c_read(uint8_t device_id, const uint8_t write_buffer[], uint16_t write_length,
 *                            uint8_t read_buffer[], uint16_t read_length)
 *
 * @brief  hal i2c read for gh3x2x
 *
 * @attention   device_id is 8bits, if platform i2c use 7bits addr, should use (device_id >> 1)
 *
 * @param[in]   device_id       device addr
 * @param[in]   write_buffer    write data buffer
 * @param[in]   write_length    write data len
 * @param[in]   read_length     read data len
 * @param[out]  read_buffer     pointer to read buffer
 *
 * @return  status
 * @retval  #1      return successfully
 * @retval  #0      return error
 */
GU8 hal_gh3x2x_i2c_read(GU8 device_id, const GU8 write_buffer[], GU16 write_length, GU8 read_buffer[], GU16 read_length)
{
    uint8_t ret = 1;

    /* code implement by user */

    GOODIX_PLANFROM_I2C_READ_ENTITY(device_id, write_buffer, write_length, read_buffer, read_length);
    return ret;
}

#else // __GH3X2X_INTERFACE__ == __GH3X2X_INTERFACE_SPI__

/* spi interface */
/**
 * @fn     void hal_gh3x2x_spi_init(void)
 *
 * @brief  hal spi init for gh3x2x
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */

void hal_gh3x2x_spi_init(void)
{
    /* SPI and CS pin have benn iinit spi and cs pin */
    return;
}

/**
 * @fn     GU8 hal_gh3x2x_spi_write(GU8 write_buffer[], GU16 length)
 *
 * @brief  hal spi write for gh3x2x
 *
 * @attention   if __GH3X2X_SPI_TYPE__ == __GH3X2X_SPI_TYPE_SOFTWARE_CS__  , user need generate timming: write_buf[0](W) + write_buf[1](W) + ...
 * @attention   if __GH3X2X_SPI_TYPE__ == __GH3X2X_SPI_TYPE_HARDWARE_CS__  , user need generate timming: CS LOW  + write_buf[0](W) + write_buf[1](W) + ... + CS HIGH
 *
 * @param[in]   write_buffer    write data buffer
 * @param[in]   length          write data len
 * @param[out]  None
 *
 * @return  status
 * @retval  #1      return successfully
 * @retval  #0      return error
 */
GU8 hal_gh3x2x_spi_write(GU8 write_buffer[], GU16 length)
{
  if (gh3020_spiwrite(write_buffer, (uint16_t)length) == 0)
    {
      return 1;
    }
  else
    {
      return 0;
    }
}


#if (__GH3X2X_SPI_TYPE__ == __GH3X2X_SPI_TYPE_SOFTWARE_CS__)
/**
 * @fn     GU8 hal_gh3x2x_spi_read(GU8 read_buffer[], GU16 length)
 *
 * @brief  hal spi read for gh3x2x
 *
 * @attention   user need generate timming: read_buf[0](R) + write_buf[1](R) + ...
 *
 * @param[in]   read_length     read data len
 * @param[out]  read_buffer     pointer to read buffer
 *
 * @return  status
 * @retval  #1      return successfully
 * @retval  #0      return error
 */
GU8 hal_gh3x2x_spi_read(GU8 read_buffer[], GU16 length)
{
  if (gh3020_spiread(read_buffer, (uint16_t)length) == 0)
    {
      return 1;
    }
  else
    {
      return 0;
    }
}
#elif (__GH3X2X_SPI_TYPE__ == __GH3X2X_SPI_TYPE_HARDWARE_CS__)
/**
 * @fn     GU8 hal_gh3x2x_spi_write_F1_and_read(GU8 read_buffer[], GU16 length)
 *
 * @brief  hal spi write F1 and read for gh3x2x
 *
 * @attention    user need generate timming: CS LOW + F1(W) + read_buf[0](R) + read_buf[1](R) + ... + CS HIGH
 *
 * @param[in]   write_buf     write data
 * @param[in]   length     write data len
 *
 * @return  status
 * @retval  #1      return successfully
 * @retval  #0      return error
 */
GU8 hal_gh3x2x_spi_write_F1_and_read(GU8 read_buffer[], GU16 length)
{
    GU8 ret = 1;

    GOODIX_PLANFROM_SPI_READ_ENTITY();

    return ret;
}
#endif

/**
 * @fn     void hal_gh3x2x_spi_cs_ctrl(GU8 cs_pin_level)
 *
 * @brief  hal spi cs pin ctrl for gh3x2x
 *
 * @attention   pin level set 1 [high level] or 0 [low level]
 *
 * @param[in]   cs_pin_level     spi cs pin level
 * @param[out]  None
 *
 * @return  None
 */
#if (__GH3X2X_SPI_TYPE__ == __GH3X2X_SPI_TYPE_SOFTWARE_CS__)
void hal_gh3x2x_spi_cs_ctrl(GU8 cs_pin_level)
{
  gh3020_spi_csctrl((uint8_t)cs_pin_level);
}
#endif

#endif

#if __SUPPORT_HARD_RESET_CONFIG__

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



void hal_gh3x2x_reset_pin_init(void)
{
  return;
}

/**
 * @fn     void hal_gh3x2x_reset_pin_ctrl(GU8 pin_level)
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

void hal_gh3x2x_reset_pin_ctrl(GU8 pin_level)
{
  gh3020_rstctrl((uint8_t)pin_level);
}

#endif
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

void hal_gh3x2x_write_cap_to_flash(GS32 WearCap1,GS32 UnwearCap1,GS32 WearCap2,GS32 UnwearCap2)
{
    GOODIX_PLANFROM_WRITE_CAP_TO_FLASH_ENTITY();
}
void hal_gh3x2x_read_cap_from_flash(GS32* WearCap1,GS32* UnwearCap1,GS32* WearCap2,GS32* UnwearCap2)
{
    GOODIX_PLANFROM_READ_CAP_FROM_FLASH_ENTITY();
}

/**
 * @fn     void Gsensor_drv_get_fifo_data(STGsensorRawdata gsensor_buffer[], GU16 *gsensor_buffer_index)
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
void Gsensor_drv_get_fifo_data(STGsensorRawdata gsensor_buffer[], GU16 *gsensor_buffer_index)
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

void Cap_drv_get_fifo_data(STCapRawdata cap_data_buffer[], GU16 *cap_buffer_index)
{
    GOODIX_PLANFROM_INT_GET_CAP_DATA_ENTITY()  ;
    if((*cap_buffer_index) > (__CAP_DATA_BUFFER_SIZE__))
    {
        while(1);   // Fatal error !!!
    }
}
void Temp_drv_get_fifo_data(STTempRawdata temp_data_buffer[], GU16 *temp_buffer_index)
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
void GH3X2X_Log(GCHAR *log_string)
{
  //syslog(LOG_DEBUG, log_string);
}
#endif

/**
 * @fn     void Gh3x2x_BspDelayUs(GU16 usUsec)
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

void Gh3x2x_BspDelayUs(GU16 usUsec)
{
  up_udelay(usUsec);
}

void GH3X2X_AdtFuncStartWithConfirmHook(void)
{
    GOODIX_PLANFROM_START_WITH_CONFIRM_HOOK_ENTITY();
}

/**
 * @fn     void Gh3x2x_BspDelayMs(GU16 usMsec)
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
void Gh3x2x_BspDelayMs(GU16 usMsec)
{
  up_mdelay(usMsec);
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
GS8 GH3x2xHrAlgoInit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
GS8 GH3x2xHrAlgoDeinit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xHrAlgoExe(const STGh3x2xFrameInfo * const pstFrameInfo)
{
}
#endif

//hrv
#if (0 == __USE_GOODIX_HRV_ALGORITHM__)
GS8 GH3x2xHrvAlgoInit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
GS8 GH3x2xHrvAlgoDeinit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xHrvAlgoExe(const STGh3x2xFrameInfo * const pstFrameInfo)
{
}
#endif

//spo2
#if (0 == __USE_GOODIX_SPO2_ALGORITHM__)
GS8 GH3x2xSpo2AlgoInit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
GS8 GH3x2xSpo2AlgoDeinit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xSpo2AlgoExe(const STGh3x2xFrameInfo * const pstFrameInfo)
{
}
#endif

//ecg
#if (0 == __USE_GOODIX_ECG_ALGORITHM__)
GS8 GH3x2xEcgAlgoInit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
GS8 GH3x2xEcgAlgoDeinit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xEcgAlgoExe(const STGh3x2xFrameInfo * const pstFrameInfo)
{

}
#endif

//soft adt
#if ((0 == __USE_GOODIX_SOFT_ADT_ALGORITHM__) || (0 == __FUNC_TYPE_SOFT_ADT_ENABLE__))
GS8 GH3x2xSoftAdtAlgoInit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
GS8 GH3x2xSoftAdtAlgoDeinit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xSoftAdtAlgoExe(const STGh3x2xFrameInfo * const pstFrameInfo)
{

}
#endif

//hsm
#if (0 == __USE_GOODIX_HSM_ALGORITHM__)
GS8 GH3x2xHsmAlgoInit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
GS8 GH3x2xHsmAlgoDeinit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xHsmAlgoExe(const STGh3x2xFrameInfo * const pstFrameInfo)
{

}
#endif

//bt
#if (0 == __USE_GOODIX_BT_ALGORITHM__)
GS8 GH3x2xBtAlgoInit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
GS8 GH3x2xBtAlgoDeinit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xBtAlgoExe(const STGh3x2xFrameInfo * const pstFrameInfo)
{

}
#endif

//resp
#if (0 == __USE_GOODIX_RESP_ALGORITHM__)
GS8 GH3x2xRespAlgoInit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
GS8 GH3x2xRespAlgoDeinit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xRespAlgoExe(const STGh3x2xFrameInfo * const pstFrameInfo)
{
}
#endif

//af
#if (0 == __USE_GOODIX_AF_ALGORITHM__)
GS8 GH3x2xAfAlgoInit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
GS8 GH3x2xAfAlgoDeinit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xAfAlgoExe(const STGh3x2xFrameInfo * const pstFrameInfo)
{

}
#endif

//bp
#if (0 == __USE_GOODIX_BP_ALGORITHM__)
GS8 GH3x2xBpAlgoInit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
GS8 GH3x2xBpAlgoDeinit(const STGh3x2xFrameInfo * const pstFrameInfo)
{
    return GH3X2X_RET_OK;
}
void  GH3x2xBpAlgoExe(const STGh3x2xFrameInfo * const pstFrameInfo)
{

}
#endif

#if (__DRIVER_LIB_MODE__==__DRV_LIB_WITH_ALGO__)
/**
 * @fn     void Gh3x2x_HrAlgorithmResultReport(STHbAlgoResult stHbAlgoRes[], GU16 pusAlgoResIndexArr[], usAlgoResCnt)
 *
 * @brief  This function will be called after calculate hb algorithm.
 *
 * @attention   None
 *
 * @param[in]   stHbAlgoRes             hb algorithm result array
 * @param[in]   pusAlgoResIndexArr      frame index of every algo result
 * @param[in]   usAlgoResCnt            number of algo result
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_HrAlgorithmResultReport(STGh3x2xAlgoResult * pstAlgoResult, GU32 lubFrameId)
{
#if (__USE_GOODIX_HR_ALGORITHM__)
    #if __GH3X2X_HR_OUTPUT_VALUE_STRATEGY_EN__
    Gh3x2xHrOutputValueStrategyPro(pstAlgoResult,lubFrameId);
    #endif
    /* code implement by user */
    //GOODIX_PLANFROM_HR_RESULT_REPORT_ENTITY();
#endif
}

/**
 * @fn     void Gh3x2x_Spo2AlgorithmResultReport(STGh3x2xAlgoResult * pstAlgoResult)
 *
 *
 * @brief  This function will be called after calculate spo2 algorithm.
 *
 * @attention   None
 *
 * @param[in]   stSpo2AlgoRes           spo2 algorithm result array
 * @param[in]   pusAlgoResIndexArr      frame index of every algo result
 * @param[in]   usAlgoResCnt            number of algo result
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_Spo2AlgorithmResultReport(STGh3x2xAlgoResult * pstAlgoResult, GU32 lubFrameId)
{
#if (__USE_GOODIX_SPO2_ALGORITHM__)

    /* code implement by user */
    //GOODIX_PLANFROM_SPO2_RESULT_REPORT_ENTITY();
#endif
}

/**
 * @fn     void Gh3x2x_HrvAlgorithmResultReport(STHrvAlgoResult stHrvAlgoRes[], GU16 pusAlgoResIndexArr[], usAlgoResCnt)
 *
 * @brief  This function will be called after calculate hrv algorithm.
 *
 * @attention   None
 *
 * @param[in]   stHrvAlgoRes            hrv algorithm result array
 * @param[in]   pusAlgoResIndexArr      frame index of every algo result
 * @param[in]   usAlgoResCnt            number of algo result
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_HrvAlgorithmResultReport(STGh3x2xAlgoResult * pstAlgoResult, GU32 lubFrameId)
{
#if (__USE_GOODIX_HRV_ALGORITHM__)
    /* code implement by user */
    //SLAVER_LOG("snHrvOut0=%d,snHrvOut1=%d,snHrvOut2=%d,snHrvOut3=%d,snHrvNum=%d,snHrvConfi=%d",
    //          stHrvAlgoRes[0].snHrvOut0,stHrvAlgoRes[0].snHrvOut1,stHrvAlgoRes[0].snHrvOut2,stHrvAlgoRes[0].snHrvOut3,stHrvAlgoRes[0].snHrvNum,stHrvAlgoRes[0].snHrvNum);
    //GOODIX_PLANFROM_HRV_RESULT_REPORT_ENTITY();
#endif
}

/**
 * @fn     void Gh3x2x_HrvAlgorithmResultReport(STHrvAlgoResult stHrvAlgoRes[], GU16 pusAlgoResIndexArr[], usAlgoResCnt)
 *
 * @brief  This function will be called after calculate hrv algorithm.
 *
 * @attention   None
 *
 * @param[in]   stHrvAlgoRes            hrv algorithm result array
 * @param[in]   pusAlgoResIndexArr      frame index of every algo result
 * @param[in]   usAlgoResCnt            number of algo result
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_BpAlgorithmResultReport(STGh3x2xBpAlgoOutput * pstAlgoResult)
{
#if (__USE_GOODIX_BP_ALGORITHM__)
    /* code implement by user */
    if (pstAlgoResult->uchResultUpdateFlag & BP_OUTPUT_RESULT)
    {
        /* BP result */
    }
    if (pstAlgoResult->uchResultUpdateFlag & BP_OUTPUT_CALI_VALUE)
    {
        /* BP cali value */
    }
    if (pstAlgoResult->uchResultUpdateFlag & BP_OUTPUT_PREESURE)
    {
        /* BP preesure value */
    }
    if (pstAlgoResult->uchResultUpdateFlag & BP_OUTPUT_OFFET)
    {
        /* BP preesure signal offset value */
    }
#endif
}

/**
 * @fn     void Gh3x2x_EcgAlgorithmResultReport(STEcgAlgoResult stEcgAlgoRes[], GU16 pusAlgoResIndexArr[], usAlgoResCnt)
 *
 * @brief  This function will be called after calculate ecg algorithm.
 *
 * @attention   None
 *
 * @param[in]   stEcgAlgoRes            ecg algorithm result array
 * @param[in]   pusAlgoResIndexArr      frame index of every algo result
 * @param[in]   usAlgoResCnt            number of algo result
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_EcgAlgorithmResultReport(STGh3x2xAlgoResult * pstAlgoResult, GU32 lubFrameId)
{
#if (__USE_GOODIX_ECG_ALGORITHM__)
    /* code implement by user */
    //GOODIX_PLANFROM_ECG_RESULT_REPORT_ENTITY();
#endif
}

/**
 * @fn     void Gh3x2x_SoftAdtAlgorithmResultReport(STGh3x2xAlgoResult * pstAlgoResult, GU8 uchDetectColor, GU32 lubFrameId)
 *
 * @brief  This function will be called after calculate ecg algorithm.
 *
 * @attention   None
 *
 * @param[in]   pstAlgoResult
 * @param[in]   pstAlgoResult
 * @param[in]   lubFrameId
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_SoftAdtAlgorithmResultReport(STGh3x2xAlgoResult * pstAlgoResult, GU8 uchDetectColor, GU32 lubFrameId)
{
#if (__USE_GOODIX_SOFT_ADT_ALGORITHM__)
    //EXAMPLE_LOG("[%s]:color = %d, result = %d,%d\r\n", __FUNCTION__, uchDetectColor, pstAlgoResult->snResult[0], pstAlgoResult->snResult[1]);
    /* code implement by user */
    if (uchDetectColor == 1)//ir
    {
        if (pstAlgoResult->snResult[0] == 0x1)
        {
            EXAMPLE_LOG("softadt ir detect wear on!!!\n");
            //Gh3x2xDemoStopSampling(GH3X2X_FUNCTION_SOFT_ADT_IR);
            //Gh3x2xDemoStartSampling(GH3X2X_FUNCTION_HR|GH3X2X_FUNCTION_SOFT_ADT_GREEN);
        }
        else if (pstAlgoResult->snResult[0] & 0x2)
        {
            EXAMPLE_LOG("softadt ir detect wear off!!!\n");
        }
    }
    else if (uchDetectColor == 0)//color
    {
        if (pstAlgoResult->snResult[0] == 0x1)
        {
            EXAMPLE_LOG("softadt green detect wear on!!!\n");
            //Gh3x2xDemoStopSampling(GH3X2X_FUNCTION_SOFT_ADT_GREEN);
            //Gh3x2xDemoStartSampling(GH3X2X_FUNCTION_HR|GH3X2X_FUNCTION_SOFT_ADT_GREEN);
        }
        else if (pstAlgoResult->snResult[0] & 0x2)
        {
            EXAMPLE_LOG("softadt green detect wear off!!!\n");
        }
    }
    GOODIX_PLANFROM_NADT_RESULT_HANDLE_ENTITY();
#endif
}

#endif // end else #if (__DRIVER_LIB_MODE__==__DRV_LIB_WITHOUT_ALGO__)




#if (__SUPPORT_PROTOCOL_ANALYZE__)
/**
 * @fn     void Gh3x2x_HalSerialSendData(GU8* uchTxDataBuf, GU16 usBufLen)
 *
 * @brief  Serial send data
 *
 * @attention   None
 *
 * @param[in]   uchTxDataBuf        pointer to data buffer to be transmitted
 * @param[in]   usBufLen            data buffer length
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_HalSerialSendData(GU8* uchTxDataBuf, GU16 usBufLen)
{
    GOODIX_PLANFROM_SERIAL_SEND_ENTITY();
}


/**
 * @fn      void Gh3x2xSerialSendTimerInit(GU8 uchPeriodMs)
 *
 * @brief  Gh3x2xSerialSendTimerInit
 *
 * @attention   None
 *
 * @param[in]   uchPeriodMs    timer period (ms)
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2xSerialSendTimerInit(GU8 uchPeriodMs)
{

}


/**
 * @fn     void Gh3x2xSerialSendTimerStop(void)
 *
 * @brief  Serial send timer stop
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2xSerialSendTimerStop(void)
{
    GOODIX_PLANFROM_SERIAL_TIMER_STOP_ENTITY();
}



/**
 * @fn     void Gh3x2xSerialSendTimerStart(void)
 *
 * @brief  Serial send timer start
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2xSerialSendTimerStart(void)
{
    GOODIX_PLANFROM_SERIAL_TIMER_START_ENTITY();
}

#endif


#if __USER_DYNAMIC_ALGO_MEM_EN__
void* Gh3x2xAlgoMemApply(GU32 unMemSize)
{
    //return malloc(unMemSize);
    return 0;
}

void Gh3x2xAlgoMemFree(void* MemAddr)
{
    //free(MemAddr);
}
#endif


/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/
