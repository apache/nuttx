/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_demo_inner.h
 *
 * @brief   gh3x2x driver lib demo inner header file
 *
 * @author  Gooidx Iot Team
 *
 */

#ifndef _GH3X2X_INNER_H_
#define _GH3X2X_INNER_H_
#include "gh3x2x_config.h"


#ifdef GOODIX_DEMO_PLANFORM
#include "gh3x2x_demo_goodix_planfrom.h"
#else
#define GOODIX_PLANFROM_SAMPLING_START_HOOK_ENTITY()
#define GOODIX_PLANFROM_SAMPLING_STOP_HOOK_ENTITY()
#define GOODIX_PLANFROM_SPI_WRITE_ENTITY(write_buffer, length)
#define GOODIX_PLANFROM_PROTOCOL_ANALYZE()
#define GOODIX_PLANFROM_REPORT_EVRENT_ENTITY()
#define GOODIX_PLANFROM_SPI_READ_ENTITY()
#define GOODIX_PLANFROM_SPI_CS_CTRL_ENTITY()
#define GOODIX_PLANFROM_SPI_RESET_PIN_ENTITY()
#define GOODIX_PLANFROM_INT_HANDLER_CALL_BACK_ENTITY()
#define GOODIX_PLANFROM_INT_GS_START_CACHE_ENTITY()
#define GOODIX_PLANFROM_INT_GS_STOP_CACHE_ENTITY()
#define GOODIX_PLANFROM_INT_GET_GS_DATA_ENTITY()
#define GOODIX_PLANFROM_LOG_ENTITY()
#define GOODIX_PLANFROM_DELAY_US_ENTITY()
#define GOODIX_PLANFROM_DELAY_MS_ENTITY()
#define GOODIX_PLANFROM_DELAY_MS_ENTITY()
#define GOODIX_PLANFROM_SERIAL_SEND_ENTITY()
#define GOODIX_PLANFROM_INT_INIT_ENTITY()
#define GOODIX_PLANFROM_RESET_PIN_INIT_ENTITY()
#define GOODIX_PLANFROM_SPI_INIT_ENTITY()
#define GOODIX_PLANFROM_CREAT_ADT_CONFIRM_ENTITY()
#define GOODIX_PLANFROM_START_TIMER_ENTITY()
#define GOODIX_PLANFROM_STOP_TIMER_ENTITY()
#define GOODIX_PLANFROM_WEAR_ON_EVENT()
#define GOODIX_PLANFROM_WEAR_OFF_EVENT()
#define GOODIX_PLANFROM_LEAD_ON_EVENT()
#define GOODIX_PLANFROM_LEAD_OFF_EVENT()
#define GOODIX_PLANFROM_HR_RESULT_REPORT_ENTITY()
#define GOODIX_PLANFROM_SPO2_RESULT_REPORT_ENTITY()
#define GOODIX_PLANFROM_SERIAL_TIMER_STOP_ENTITY()
#define GOODIX_PLANFROM_SERIAL_TIMER_START_ENTITY()
#define GOODIX_PLANFROM_I2C_WRITE_ENTITY(device_id, write_buffer,length)
#define GOODIX_PLANFROM_I2C_INIT_ENTITY()
#define GOODIX_PLANFROM_I2C_READ_ENTITY(device_id, write_buffer, write_length, read_buffer, read_length)
#define GOODIX_PLANFROM_INT_CAP_START_CACHE_ENTITY()
#define GOODIX_PLANFROM_INT_CAP_STOP_CACHE_ENTITY()
#define GOODIX_PLANFROM_INT_GET_CAP_DATA_ENTITY()
#define GOODIX_PLANFROM_INT_TEMP_START_CACHE_ENTITY()
#define GOODIX_PLANFROM_INT_TEMP_STOP_CACHE_ENTITY()
#define GOODIX_PLANFROM_INT_GET_TEMP_DATA_ENTITY()
#define GOODIX_PLANFROM_WRITE_CAP_TO_FLASH_ENTITY()
#define GOODIX_PLANFROM_READ_CAP_FROM_FLASH_ENTITY()
#define GOODIX_PLANFROM_BEFORE_WAKE_UP_HOOK_ENTITY()
#define GOODIX_PLANFROM_WAKE_UP_HOOK_ENTITY()
#define GOODIX_PLANFROM_PT_CONFIG_HOOK_ENTITY()
#define GOODIX_PLANFROM_START_WITH_CONFIRM_HOOK_ENTITY()
#define GOODIX_PLANFROM_PRESSURE_PARAS_READ_ENTITY()
#define GOODIX_PLANFROM_PRESSURE_PARAS_WRITE_ENTITY()
#define GOODIX_PLANFROM_PRINTF_ENTITY()
#define GOODIX_PLANFROM_MALLOC_USER_ENTITY()
#define GOODIX_PLANFROM_FREE_USER_ENTITY()
#define GOODIX_PLANFROM_NADT_RESULT_HANDLE_ENTITY()



#endif

typedef struct
{
    int16_t sXAxisVal; // X-Axis Value
    int16_t sYAxisVal; // Y-Axis Value
    int16_t sZAxisVal; // Z-Axis Value
#if _GS_GYRO_ENABLE_
    int16_t sXGyroVal;
    int16_t sYGyroVal;
    int16_t sZGyroVal;
#endif

}STGsensorRawdata;


#define __PROTOCOL_DATA_LEN__       235

/// save function opened of driver lib
extern uint32_t g_unDemoFuncMode;

/* config list array */
extern const struct gh3020_initcfg_s gh3020_initcfg_list[];

#if (__DRIVER_LIB_MODE__==__DRV_LIB_WITHOUT_ALGO__)
/// data buffer for algorithm
extern uint32_t g_uchGh3x2xDemoBufferForAlgo[];

#if (__FUNC_TYPE_HR_ENABLE__)
/// hba algorithm channel map cnt
extern uint8_t g_uchHbaChannelMapCnt;

/// hba incomplete data mark
extern uint32_t g_unDemoHbaIncompleteChMark;

/// hba incomplete data last time
extern uint32_t g_unDemoHbaIncompleteRawDataArr[];
#endif

#if (__FUNC_TYPE_SPO2_ENABLE__)
/// spo2 algorithm channel map cnt
extern uint8_t g_uchSpo2ChannelMapCnt;

/// spo2 incomplete data mark
extern uint32_t g_unDemoSpo2IncompleteChMark;

/// spo2 incomplete data last time
extern uint32_t g_unDemoSpo2IncompleteRawDataArr[];
#endif

#if (__FUNC_TYPE_ECG_ENABLE__)
/// ecg incomplete data last time
extern uint32_t g_unDemoEcgIncompleteChMark;

/// ecg incomplete data last time
extern uint32_t g_unDemoEcgIncompleteRawDataArr[];
#endif

#if (__FUNC_TYPE_HRV_ENABLE__)
/// hrv algorithm channel map cnt
extern uint8_t g_uchHrvChannelMapCnt;

/// hrv incomplete data mark
extern uint32_t g_unDemoHrvIncompleteChMark;

/// hrv incomplete data last time
extern uint32_t g_unDemoHrvIncompleteRawDataArr[];
#endif

#endif

#if __GH3X2X_MEM_POOL_CHECK_EN__
extern uint8_t  g_uchGh3x2xMemPollHaveGetCheckSum;
#endif



extern uint8_t g_uchDemoWorkMode;



#if (__NORMAL_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__ || __MIX_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__)
//hal_gh3x2x_int_handler_call_back is called, this value is 1
extern uint8_t g_uchGh3x2xIntCallBackIsCalled;
#endif




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
extern void hal_gh3x2x_i2c_init(void);

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
uint8_t hal_gh3x2x_i2c_write(uint8_t device_id, const uint8_t write_buffer[], uint16_t length);

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
uint8_t hal_gh3x2x_i2c_read(uint8_t device_id, const uint8_t write_buffer[], uint16_t write_length, uint8_t read_buffer[], uint16_t read_length);

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
extern void hal_gh3x2x_spi_init(void);

/**
 * @fn     uint8_t hal_gh3x2x_spi_write(uint8_t write_buffer[], uint16_t length)
 *
 * @brief  hal spi write for gh3x2x
 *
 * @attention   None
 *
 * @param[in]   write_buffer    write data buffer
 * @param[in]   length          write data len
 * @param[out]  None
 *
 * @return  status
 * @retval  #1      return successfully
 * @retval  #0      return error
 */
extern uint8_t hal_gh3x2x_spi_write(uint8_t write_buffer[], uint16_t length);

/**
 * @fn     uint8_t hal_gh3x2x_spi_read(uint8_t read_buffer[], uint16_t length)
 *
 * @brief  hal spi read for gh3x2x
 *
 * @attention   None
 *
 * @param[in]   read_length     read data len
 * @param[out]  read_buffer     pointer to read buffer
 *
 * @return  status
 * @retval  #1      return successfully
 * @retval  #0      return error
 */
extern uint8_t hal_gh3x2x_spi_read(uint8_t read_buffer[], uint16_t length);

extern uint8_t hal_gh3x2x_spi_write_F1_and_read(uint8_t read_buf[], uint16_t length);


/**
 * @fn     void hal_gh3x2x_spi_cs_ctrl(uint8_t cs_pin_level)
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
extern void hal_gh3x2x_spi_cs_ctrl(uint8_t cs_pin_level);


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
extern void hal_gh3x2x_reset_pin_ctrl(uint8_t pin_level);

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
extern void hal_gh3x2x_int_init(void);

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
extern void Gsensor_start_cache_data(void);

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
extern void Gsensor_stop_cache_data(void);
extern void Cap_start_cache_data(void);
extern void Cap_stop_cache_data(void);
extern void Temp_start_cache_data(void);
extern void Temp_stop_cache_data(void);
/**
 * @fn     void Gsensor_drv_get_fifo_data(STGsensorRawdata gsensor_buffer[], uint16_t *gsensor_buffer_index)
 *
 * @brief  get gsensor fifo data
 *
 * @attention   When read fifo data of GH3x2x,user should call this function to get gsensor data.And user must
 *              assure that these gsensor data is corresponding to GH3x2x fifo data
 *
 * @param[in]   None
 * @param[out]  gsensor_buffer          pointer to gsensor data buffer
 * @param[out]  gsensor_buffer_index    pointer to number of gsensor data
 *
 * @return  None
 */
extern void Gsensor_drv_get_fifo_data(STGsensorRawdata gsensor_buffer[], uint16_t *gsensor_buffer_index);
extern void Cap_drv_get_fifo_data(STCapRawdata cap_data_buffer[], uint16_t *cap_buffer_index);
extern void Temp_drv_get_fifo_data(STTempRawdata Temp_data_buffer[], uint16_t *temp_buffer_index);
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
extern void GH3X2X_Log(char *log_string);

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
extern void Gh3x2x_BspDelayUs(uint16_t usUsec);

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
extern void Gh3x2x_BspDelayMs(uint16_t usMsec);

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
extern void Gh3x2x_LeadOnEventHook(void);

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
extern void Gh3x2x_LeadOffEventHook(void);

/**
 * @fn     void Gh3x2x_DemoAlgorithmMemConfig(void)
 *
 * @brief  Config memory for algorithm
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */



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
void Gh3x2x_UserHandleCurrentInfo(void);

extern void Gh3x2x_DemoAlgorithmMemConfig(void);

/**
 * @fn     void Gh3x2x_Spo2AlgorithmResultHook(STSpo2AlgoResult stSpo2AlgoRes[], uint16_t pusAlgoResIndexArr[],
 *                                              usAlgoResCnt)
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
void Gh3x2x_Spo2AlgorithmResultHook(STGh3x2xAlgoResult * pstAlgoResult, uint8_t lubFrameId);

/**
 * @fn     void Gh3x2x_HrAlgorithmResultHook(STHbAlgoResult stHbAlgoRes[], uint16_t pusAlgoResIndexArr[], usAlgoResCnt)
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
void Gh3x2x_HrAlgorithmResultHook(STGh3x2xAlgoResult * pstAlgoResult, uint8_t lubFrameId);

/**
 * @fn     void Gh3x2x_HrvAlgorithmResultHook(STHbAlgoResult stHbAlgoRes[], uint16_t pusAlgoResIndexArr[], usAlgoResCnt)
 *
 * @brief  This function will be called after calculate hrv algorithm.
 *
 * @attention   None
 *
 * @param[in]   stHbAlgoRes             hrv algorithm result array
 * @param[in]   pusAlgoResIndexArr      frame index of every algo result
 * @param[in]   usAlgoResCnt            number of algo result
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_HrvAlgorithmResultHook(STGh3x2xAlgoResult * pstAlgoResult, uint8_t lubFrameId);

/**
 * @fn     void Gh3x2x_EcgAlgorithmResultHook(STEcgAlgoResult stEcgAlgoRes[], uint16_t pusAlgoResIndexArr[], usAlgoResCnt)
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
void Gh3x2x_EcgAlgorithmResultHook(STGh3x2xAlgoResult * pstAlgoResult, uint8_t lubFrameId);

/**
 * @fn     void Gh3x2xDemoAlgorithmCalculate(uint8_t* puchReadFifoBuffer, STGsensorRawdata *pstGsAxisValueArr,
 *                                              uint16_t usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *
 * @brief  Algorithm calculate.This function will be called in Gh3x2xDemoInterruptProcess()
 *
 * @attention   None
 *
 * @param[in]   puchReadFifoBuffer      point to gh3x2x fifo data buffer
 * @param[in]   pstGsAxisValueArr       point to gsensor data buffer
 * @param[in]   usGsDataNum             gsensor data cnt
 * @param[in]   emGsSensitivity         sensitivity of gsensor
 * @param[out]  None
 *
 * @return  None
 */

void Gh3x2x_DemoAlgorithmCalculate(uint8_t* puchReadFifoBuffer, uint16_t usFifoBuffLen,STGsensorRawdata *pstGsAxisValueArr, uint16_t usGsDataNum,
                                   STCapRawdata* pstCapValueArr,uint16_t usCapDataNum,STTempRawdata* pstTempValueArr,uint16_t usTempDataNum);

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
extern void Gh3x2xDemoSamplingControl(uint32_t unFuncMode, EMUprotocolParseCmdType emSwitch);


/// dump mode
extern uint16_t g_usDumpMode;
extern void GH3X2X_LedAgcProcessExDump(uint8_t* puchReadFifoBuffer);

extern void GH3X2X_ReadElectrodeWearDumpData(void);

extern void GH3X2X_RecordDumpData(void);

extern void gh3x2x_init_hook_func(void);

extern void gh3x2x_sampling_start_hook_func(void);

extern void gh3x2x_sampling_stop_hook_func(void);



/**
 * @fn     void Gh3x2xDemoReportEvent(uint16_t usEvent,uint8_t uchWearType)
 *
 * @brief  Report event to APP/EVK
 *
 * @attention   None
 *
 * @param[in]   usEvent      GH3x2x event
 * @param[out]  None
 *
 * @return  None
 */
extern void Gh3x2xDemoReportEvent(uint16_t usEvent,uint8_t uchWearType);

#if (__DRIVER_LIB_MODE__==__DRV_LIB_WITH_ALGO__)
/**
 * @fn     void Gh3x2xDemoReportEvent(uint16_t usEvent,uint8_t uchWearType)
 *
 * @brief  Report event to APP/EVK
 *
 * @attention   None
 *
 * @param[in]   puchFifoReadData        gh3x2x fifo data
 * @param[in]   pstGsAxisValueArr       pointer to gsensor data buf
 * @param[in]   usGsDataNum             gsensor data number
 * @param[in]   emGsSensitivity         gsensor sensitivity
 * @param[out]  None
 *
 * @return  None
 */
extern void Gh3x2xDemoSendRawdataPkg(uint8_t* puchFifoReadData, STGsensorRawdata *pstGsAxisValueArr,
                                            uint16_t usGsDataNum, EMGsensorSensitivity emGsSensitivity);
#else
/**
 * @fn     void Gh3x2xDemoSendRawdataPkg(uint8_t* puchFifoReadData, uint16_t usFifoDataLen, STGsensorRawdata *pstGsAxisValueArr,
                                uint16_t usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *
 * @brief  Report event to APP/EVK
 *
 * @attention   None
 *
 * @param[in]   puchFifoReadData        gh3x2x fifo data
 * @param[in]   usFifoDataLen           gh3x2x fifo data len in byte
 * @param[in]   pstGsAxisValueArr       pointer to gsensor data buf
 * @param[in]   usGsDataNum             gsensor data number
 * @param[in]   emGsSensitivity         gsensor sensitivity
 * @param[out]  None
 *
 * @return  None
 */
extern void Gh3x2xDemoSendRawdataPkg(uint8_t* puchFifoReadData, uint16_t usFifoDataLen, STGsensorRawdata *pstGsAxisValueArr,
                                uint16_t usGsDataNum, EMGsensorSensitivity emGsSensitivity);
#endif

/**
 * @fn     void Gh3x2x_HalSerialSendData(uint8_t* uchTxDataBuf, uint16_t usBufLen)
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
extern void Gh3x2x_HalSerialSendData(uint8_t* uchTxDataBuf, uint16_t usBufLen);

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


extern void Gh3x2xSerialSendTimerStop(void);

extern void Gh3x2xSerialSendTimerStart(void);

extern void Gh3x2x_StartAdtConfirmTimer(void);

extern void Gh3x2xSerialSendTimerInit(uint8_t uchPeriodMs);


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
extern void Gh3x2x_StopAdtConfirmTimer(void);

extern void Gh3x2xDemoSendWearOrLeadStateToUI(uint8_t state);

extern void Gh3x2x_HalSerialFifoInit(void);


extern void Gh3x2xMemPoolCheckSumWrite(uint8_t *lunMemPool);
extern uint16_t Gh3x2xMemPoolCheckSumCheck(uint8_t *lunMemPool);
extern void Gh3x2xCheckMemPollBeforeAlgoCal(void);
extern void Gh3x2xUpdataMemPollChkSumAfterAlgoCal(void);
extern void Gh3x2xHrOutputValueStrategyPro(STGh3x2xAlgoResult * pstAlgoResult, uint32_t lubFrameId);
extern void GH3x2xMaserControlFunctionLog(uint32_t unFuncMode, EMUprotocolParseCmdType emCmdType);


#if __FUNC_TYPE_SOFT_ADT_ENABLE__
extern uint8_t g_uchVirtualAdtTimerCtrlStatus;
#if (__USE_POLLING_TIMER_AS_ADT_TIMER__)&&\
    (__POLLING_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__ || __MIX_INT_PROCESS_MODE__ == __INTERRUPT_PROCESS_MODE__)
#define GH3X2X_START_ADT_TIMER()   g_uchVirtualAdtTimerCtrlStatus = 1;
#define GH3X2X_STOP_ADT_TIMER()    g_uchVirtualAdtTimerCtrlStatus = 0;
#else
#define GH3X2X_START_ADT_TIMER()    if (GH3X2X_GetSoftWearOffDetEn()){Gh3x2x_StartAdtConfirmTimer();g_uchVirtualAdtTimerCtrlStatus = 1;}
#define GH3X2X_STOP_ADT_TIMER()    if (GH3X2X_GetSoftWearOffDetEn()){Gh3x2x_StopAdtConfirmTimer();g_uchVirtualAdtTimerCtrlStatus = 0;}
#endif
#endif
#endif /* _GH3X2X_DEMO_INNER_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/
