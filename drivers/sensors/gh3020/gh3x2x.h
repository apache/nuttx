/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_demo.h
 *
 * @brief   header file of gh3x2x driver lib demo
 *
 * @version GH(M)3X2X_DRV_LIB_DEMO_1.0
 *
 * @author  Gooidx Iot Team
 *
 */


#ifndef _GH3X2X_H_
#define _GH3X2X_H_

#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"

/**
 * @fn     int gh3020_init(void)
 *
 * @brief  Init GH3x2x module.
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
int gh3020_init(void);

/**
 * @fn     GU8 gh3020_fifo_process(void)
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
void gh3020_fifo_process(void);

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
void gh3x2x_int_handler_call_back(void);


/**
 * @fn     GS8 Gh3x2xDemoArrayCfgSwitch(GU8 uchArrayCfgIndex)
 *
 * @brief  array cfg switch (call by user)
 *
 * @attention   If __GH3X2X_ARRAY_CFG_MANUAL_SWITCH_EN__ is 1, you should switch array cfg manually before calling gh3020_start_sampling
 *
 * @param[in]   uchArrayCfgIndex    0: gh3x2x_reg_list0    1: gh3x2x_reg_list0 ...
 * @param[out]  None
 *
 * @return  GH3X2X_RET_OK:switch application mode success
 * @return  GH3X2X_RET_RESOURCE_ERROR:can't find corresponding reg config array
 */
GS8 Gh3x2xDemoArrayCfgSwitch(GU8 uchArrayCfgIndex);


/**
 * @fn     void gh3020_start_sampling()
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
void gh3020_start_sampling(GU32 unFuncMode);

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
void gh3020_start_sampling_factest(GU32 unFuncMode, STGh3x2xEngineeringModeSampleParam *pstSampleParaGroup , GU8 uchSampleParaGroupNum);


/**
 * @fn     void gh3020_stop_sampling(GU32 unFuncMode)
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
void gh3020_stop_sampling(GU32 unFuncMode);



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
void gh3020_stop_sampling_factest(void);



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
void Gh3x2xCreateAdtConfirmTimer(void);

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
void Gh3x2xDemoMoveDetectTimerHandler(void);

/**
 * @fn     void Gh3x2xDemoProtocolProcess(GU8* puchProtocolDataBuffer, GU16 usRecvLen)
 *
 * @brief  Analyze protocol about GH3x2x,and pack protocol data to reply
 *
 * @attention   None
 *
 * @param[in]  puchProtocolDataBuffer   pointer to received protocol data buffer
 * @param[in]  usRecvLen                protocol data buffer length
 *
 * @return  None
 */
void Gh3x2xDemoProtocolProcess(GU8* puchProtocolDataBuffer, GU16 usRecvLen);

/**
 * @fn     void gh3020_samplerate_set(GU32 unFunctionID,  GU16 usSampleRate)
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
void gh3020_samplerate_set(GU32 unFunctionID,  GU16 usSampleRate);




/**
 * @fn     void Gh3x2xDemoSetFuncionLedCurrent(GU8 uchFunctionID, GU16 usLedDrv0Current, GU16 usLedDrv1Current)
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
void Gh3x2xDemoSetFuncionLedCurrent(GU8 uchFunctionID, GU16 usLedDrv0Current, GU16 usLedDrv1Current);







/**
 * @fn      void Gh3x2xDemoFunctionChannelEnSet(GU32 unFunctionID,  GU32 unChnlEn)
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
void Gh3x2xDemoFunctionChannelEnSet(GU32 unFunctionID,  GU32 unChnlEn);

/**
 * @fn     void gh3020_rdmode_switch(EMInterruptModeType emIntModeType)
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
void gh3020_rdmode_switch(GU8 uchIntModeType);
GU8 Gh3x2xGetInterruptMode(void);
void Gh3x2xSerialSendTimerHandle(void);
void gh3020_set_fifowtm(uint16_t fifowtm);


#endif /* _GH3X2X_DEMO_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/
