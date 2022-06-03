/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_control.c
 *
 * @brief   gh3x2x control functions
 *
 * @version ref gh3x2x_drv_version.h
 *
 */


#include <stdio.h>
#include "gh3x2x_drv_version.h"
#include "gh3x2x_drv_common.h"
#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"
#include "gh3x2x.h"
#include "gh3x2x_drv_control.h"
#include "gh3x2x_drv_control_ex.h"
#include "gh3x2x_drv_uprotocol.h"
#include "gh3x2x_drv_dump.h"
#include "gh3x2x_drv_soft_led_agc.h"
#include "gh3x2x_drv_config.h"
#include "gh3x2x_drv_soft_adt.h"
#include "gh3x2x_drv_zip.h"


uint8_t gubGh3x2xUseZipProtocol = 1;
uint8_t g_uchFifoPackageID = 0xFF;
uint8_t g_uchFifoPackageMode = 0;
//uint8_t *gpuchSendFifoPackage = GH3X2X_PTR_NULL;
uint32_t g_unSendFifoPackageCnt = 0;


uint8_t g_uchAlgoEnableFlag = 1;

uint32_t * gpunFrameRawdata;

STAdtCfg g_stAdtModuleCfg;

uint8_t g_uchElectrodeWearRevCnt;

uint8_t g_uchOddEvenChangeFlag = 1;

/// config array translate key code. ascii array from "GooidxIOTConfigArrKeyCode"
const uint8_t g_uchConfigArrKeyCodeArr[] =
{
    0x47, 0x6F, 0x6F, 0x69, 0x64, 0x78, 0x49, 0x4F, 0x54, 0x43, 0x6F, 0x6E, 0x66,
    0x69, 0x67, 0x41, 0x72, 0x72, 0x4B, 0x65, 0x79, 0x43, 0x6F, 0x64, 0x65,
};

/// xor val index @g_uchConfigArrKeyCodeArr
static uint8_t g_uchEncryptRawdataXorIndex = GH3X2X_ENCRYPT_ARRAY_START_INDEX;



/// soft event
uint8_t gubSoftEvent = 0;  //G202008231001 wanpeng
/// G sensor enable flag
uint8_t g_uchGsensorEnable = 0;

/// Cap enable flag
uint8_t g_uchCapEnable = 0;

/// Temp enable flag
uint8_t g_uchTempEnable = 0;

uint8_t g_uchResetFromProtocolFlag = 0;

/// gh3x2x status
uint8_t g_uchGh3x2xStatus = GH3X2X_STATUS_NO_INIT;

/// current fifo water line
uint16_t g_usCurrentFiFoWaterLine = 0;

/// read int pin status func ptr
pfnReadPinStatus g_pGh3x2xReadIntPinStatusFunc = GH3X2X_PTR_NULL;

/// read reset pin status func ptr
pfnReadPinStatus g_pGh3x2xReadResetPinStatusFunc = GH3X2X_PTR_NULL;

/// read spcs pin status func ptr
pfnReadPinStatus g_pGh3x2xReadSpcsPinStatusFunc = GH3X2X_PTR_NULL;

/// read spdo pin status func ptr
pfnReadPinStatus g_pGh3x2xReadSpdoPinStatusFunc = GH3X2X_PTR_NULL;

/// set reset pin level spi
pfnSetPinLevel g_pGh3x2xResetPinLevelControlFunc = GH3X2X_PTR_NULL;

#if     GH3X2X_PMU_FIFO_POWER_CTRL_ENABLED

/// fifo power ctrl fifo control reg cache var
uint16_t g_usPmuFifoModuleCtrlVal = 0x0000;

/// for fifo power ctrl cache status
uint8_t g_uchPmuFifoModuleCtrlCacheStatus = 0;

/// max rawdata num read from fifo every time
uint16_t g_usMaxNumReadFromFifo = GH3X2X_FIFO_DATA_BYTES_MAX_LEN;

#endif


uint8_t g_uchActiveChipResetFlag = 0;   //1: user have done chip reset(soft reset and hardwear reset) actively   gh3x2x_init will clear it
uint8_t g_uchChipResetRecoveringFlag = 0;  //0: is not in chip reset recovering flag  1: is in chip reset recovering


uint8_t g_uchNeedWakeUpGh3x2x = 1; // 0: do not need wake up gh3x2x  1: need wake up gh3x2x
uint8_t g_uchNeedWakeUpGh3x2xBeforeInt = 0; // 0: do not need wake up gh3x2x  1: need wake up gh3x2x

uint8_t g_uchAlgInitFlag[GH3X2X_FUNC_OFFSET_MAX] = {0};
uint8_t g_uchGh3x2xSleepFlag = 0;

extern void GH3X2X_RawdataCode(uint32_t *punRawdata, uint16_t usChnlNum);

/**
 * @fn     int8_t GH3X2X_ExitLowPowerMode(void)
 *
 * @brief  Exit lowpower mode, in this mode, can read&write reg with gh3x2x
 *
 * @attention   Power consumption bigger than lowpower mode, detaile ref gh3x2x datasheet
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR    exit low power mode error
 */
int8_t GH3X2X_ExitLowPowerMode(void)
{
    int8_t chRet = GH3X2X_RET_GENERIC_ERROR;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    GH3X2X_CHIP_WAKEUP(chRet);
    GH3x2x_SetNeedWakeUpGh3x2xFlag(1);
    return chRet;
}

/**
 * @fn     int8_t GH3X2X_EnterLowPowerMode(void)
 *
 * @brief  Enter lowpower mode, in this mode, can't read&write reg with gh3x2x.
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR    enter low power mode error
 */
int8_t GH3X2X_EnterLowPowerMode(void)
{
    int8_t chRet = GH3X2X_RET_GENERIC_ERROR;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    GH3X2X_CHIP_SLEEP(chRet);
    return chRet;
}

int8_t GH3X2X_EnterLowPowerModeWithoutWaiting(void)
{
    int8_t chRet = GH3X2X_RET_GENERIC_ERROR;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    GH3X2X_CHIP_SLEEP_NOWAIT(chRet);
    return chRet;
}

/**
 * @fn     int8_t GH3X2X_SoftReset(void)
 *
 * @brief  Gh3x2x softreset via i2c/spi, can read&write reg with gh3x2x after reset
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR    exit low power mode error
 */
int8_t GH3X2X_SoftReset(void)
{
    int8_t chRet = GH3X2X_RET_GENERIC_ERROR;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    GH3X2X_CHIP_RESET(chRet);
    g_uchActiveChipResetFlag = 1;
    if (g_uchResetFromProtocolFlag == 1)
    {
        g_uchResetFromProtocolFlag = 0;
        usleep(20000);
        gh3020_fifo_process();
    }
    return chRet;
}


#if defined(GH3X2X_LOG_DEBUG) && (GH3X2X_LOG_DEBUG > 0) // debug level > 0
/**
 * @fn     void GH3X2X_LogConfigArr(struct gh3020_reg_s *pstRegConfigArr, uint16_t usRegConfigLen)
 *
 * @brief  log gh3x2x reg config array
 *
 * @attention   Only use for dbg version;
 *
 * @param[in]   pstRegConfigArr       pointer to the reg struct array
 * @param[in]   usRegConfigLen        reg struct array length
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_LogConfigArr(FAR struct gh3020_reg_s *pstRegConfigArr, uint16_t usRegConfigLen)
{
    uint16_t usIndex = 0;
    GH3X2X_DEBUG_LOG("Log reg config arr:\r\n");
    if ((pstRegConfigArr != GH3X2X_PTR_NULL) && (usRegConfigLen != 0))
    {
        for (usIndex = 0; usIndex < usRegConfigLen; usIndex++)
        {
            GH3X2X_DEBUG_LOG_PARAM("addr:0x%.4x, val:0x%.4x\r\n", pstRegConfigArr[usIndex].regaddr,
                                                                pstRegConfigArr[usIndex].regval);
        }
    }
}
#endif

/**
 * @fn     int8_t GH3X2X_DumpRegs(struct gh3020_reg_s *pstDumpRegsArr, uint16_t usDumpRegsStartAddr, uint16_t usDumpRegsLen)
 *
 * @brief  Dump gh3x2x regs
 *
 * @attention   usDumpRegsStartAddr only allow even address, if param set odd address val that val & 0xFFFE in api;
 *              If address greater than reg max addres 0xFFFE, it will return GH3X2X_RET_GENERIC_ERROR.
 *
 * @param[out]  pstDumpRegsArr           pointer to the dump reg struct output
 * @param[in]   usDumpRegsStartAddr     dump reg address
 * @param[in]   usDumpRegsLen           dump reg length
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR    dump gh3x2x address out of bounds
 */
int8_t GH3X2X_DumpRegs(FAR struct gh3020_reg_s *pstDumpRegsArr, uint16_t usDumpRegsStartAddr, uint16_t usDumpRegsLen)
{
    int8_t chRet = GH3X2X_RET_OK;
    uint16_t usIndex = 0;
    uint16_t usDumpRegsAddr = usDumpRegsStartAddr & GH3020_REG_ADDR_EVEN_FIXED; // just allow even addr val

    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if ((pstDumpRegsArr != GH3X2X_PTR_NULL) && (usDumpRegsLen != 0))
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        for (usIndex = 0; usIndex < usDumpRegsLen; usIndex++) // read GH3X2X reg.
        {
            pstDumpRegsArr[usIndex].regaddr = usDumpRegsAddr;
            pstDumpRegsArr[usIndex].regval = gh3020_spi_readreg(usDumpRegsAddr);
            if (usDumpRegsAddr < GH3020_REG_ADDR_MAX) // last reg addr : 0xfffe
            {
                usDumpRegsAddr += GH3020_REG_ADDR_SIZE;
            }
            else
            {
                GH3X2X_DEBUG_LOG("Dump reg address out of bounds!\r\n");
                chRet = GH3X2X_RET_GENERIC_ERROR;
                break;
            }
        }
        GH3X2X_WAIT_CHIP_DSLP();
    }
    return chRet;
}

/**
 * @fn     int8_t GH3X2X_CommunicateConfirm(void)
 *
 * @brief  Communication operation interface confirm
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_COMM_ERROR       gh3x2x communicate error
 */
int8_t GH3X2X_CommunicateConfirm(void)
{
    uint8_t uchIndex = GH3X2X_COMMUNICATE_CONFIRM_MAX_CNT;
    int8_t chRet = GH3X2X_RET_COMM_ERROR;
    uint16_t uchReadData;
    uint16_t uchWriteData;

    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    do
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        if (gh3020_spi_readreg(GH3020_REG_CHIP_READY_CODE) == GH3020_REGVAL_CHIP_READY)
        {
            uchReadData = gh3020_spi_readreg(GH3020_REG_SLOT_TMR0);
            GH3X2X_DEBUG_LOG_PARAM("slot_time0 reg = %d\r\n", (int)uchReadData);
            uchWriteData = ~uchReadData;
            gh3020_spi_writereg(GH3020_REG_SLOT_TMR0, uchWriteData);
            uchReadData = gh3020_spi_readreg(GH3020_REG_SLOT_TMR0);
            GH3X2X_DEBUG_LOG_PARAM("slot_time0 reg = %d\r\n", (int)uchReadData);
            if (uchWriteData == uchReadData)
            {
                chRet = GH3X2X_RET_OK;
                uchWriteData = ~uchReadData;
                gh3020_spi_writereg(GH3020_REG_SLOT_TMR0, uchWriteData);
                break;
            }
        }
        else
        {
            GH3X2X_DEBUG_LOG_PARAM("ready code reg is error !!! %s\r\n", __FUNCTION__);
        }
        uchIndex--;
    } while (uchIndex > 0);
    GH3X2X_WAIT_CHIP_DSLP();

    return chRet;
}

/**
 * @fn     int8_t GH3X2X_Init(const struct gh3020_initcfg_s *pstGh3x2xInitConfigParam)
 *
 * @brief  Init gh3x2x with configure parameters
 *
 * @attention   None
 *
 * @param[in]   pstGh3x2xInitConfigParam      pointer to gh3x2x init config param
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_COMM_ERROR       gh3x2x communicate error
 */
int8_t GH3X2X_Init(FAR const struct gh3020_initcfg_s *pstGh3x2xInitConfigParam)
{
    g_uchGh3x2xStatus = GH3X2X_STATUS_NO_INIT;

    GH3X2X_DEBUG_LOG_PARAM("%s : Lib Version : %s\r\n", __FUNCTION__, GH3X2X_GetDriverLibVersion());
    if (GH3X2X_CommunicateConfirm() != GH3X2X_RET_OK) /* confirm GH3X2X communicate. */
    {
        GH3X2X_DEBUG_LOG("communicate confirm error!\r\n");
        return GH3X2X_RET_COMM_ERROR;
    }

    /* clear int status. */
    GH3X2X_WAIT_CHIP_WAKEUP();
    gh3020_spi_writereg(GH3020_REG_INT_STR, GH3020_MSK_INT_STR_ALL_BIT);
    GH3X2X_WAIT_CHIP_DSLP();

    if (pstGh3x2xInitConfigParam != GH3X2X_PTR_NULL)
    {
        /* load config */
        if (GH3X2X_LoadNewRegConfigArr(pstGh3x2xInitConfigParam->pregarr,
            pstGh3x2xInitConfigParam->arrlen) != GH3X2X_RET_OK)
        {
            return GH3X2X_RET_COMM_ERROR;
        }
    }
    else
    {
        GH3X2X_DEBUG_LOG("gh3x2x init param is null!\r\n");
    }

    uint16_t efuse = gh3020_get_efuse();
    GH3X2X_SetDrvEcode((int8_t)(efuse & GH3020_MSK_EFUSE_CTRL_RDATA1_LED));
    GH3X2X_SetConfigDrvEcode(pstGh3x2xInitConfigParam);

    /* @ fix chip error code below here.
     * e.g. otp clk err
     */

    /* @end of fix chip error. */

    /* call hook */
    GH3X2X_WAIT_CHIP_WAKEUP();
    GH3X2X_WAIT_CHIP_DSLP();

    /* set chip status inited. */
    GH3X2X_SET_CHIP_INIED();
    g_uchGh3x2xStatus = GH3X2X_STATUS_INITED;
    g_uchActiveChipResetFlag = 0;
    return GH3X2X_RET_OK;
}

/**
 * @fn     int8_t GH3X2X_StartSampling(void)
 *
 * @brief  Start gh3x2x sampling
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK                  return successfully
 * @retval  #GH3X2X_RET_NO_INITED_ERROR     gh3x2x has not inited
 * @retval  #GH3X2X_RET_GENERIC_ERROR       gh3x2x has started, need restart should call GH3X2X_StopSampling then start
 */
int8_t GH3X2X_StartSampling(void)
{
    int8_t chRet = GH3X2X_RET_GENERIC_ERROR;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if (g_uchGh3x2xStatus != GH3X2X_STATUS_STARTED)
    {
        g_uchFifoPackageID = 0xFF;
        if (GH3X2X_IS_CHIP_INIED() && (g_uchGh3x2xStatus == GH3X2X_STATUS_INITED))
        {
            uint16_t usRegVal = 0;
            GH3X2X_WAIT_CHIP_WAKEUP();
            g_usCurrentFiFoWaterLine = gh3020_spi_readreg(GH3020_REG_FIFO_WATERLINE);  //record water line
            usRegVal = gh3020_spi_readreg(GH3020_REG_CARDIFF_CTRL);
            gh3020_spi_writereg(GH3020_REG_CARDIFF_CTRL, GH3X2X_SET_BIT(usRegVal, GH3020_MSK_CARDIFF_CTRL_START));
            #if GH3X2X_ALGORITHM_ECG_SUPPORT
            GH3X2X_LeadDetEnControl(ECG_SAMPLE_EVENT_INFO_SAMPLE_START);
            #endif

            for (int uchSlotIdx=0; uchSlotIdx < 8; uchSlotIdx++)
            {
                for (int nAdcCnt=0; nAdcCnt < 4; nAdcCnt++)
                {
                    uint8_t uchBgCancel=GH3X2X_GetSlotLedTiaGain(uchSlotIdx, nAdcCnt);
                    g_puchTiaGainAfterSoftAgc[uchSlotIdx*4 + nAdcCnt] = uchBgCancel;
                }
            }

            GH3X2X_LedAgcInit();
        #if GH3X2X_DUMP_MODE_EN
            GH3X2X_DumpInit();
        #endif
            g_uchGh3x2xStatus = GH3X2X_STATUS_STARTED;
            GH3X2X_WAIT_CHIP_DSLP(); //  GH3X2X_SEND_DSLEEP_CMD(); // enter sleep mode in application code
            chRet = GH3X2X_RET_OK;
        }
        else
        {
            g_uchGh3x2xStatus = GH3X2X_STATUS_NO_INIT;
            chRet = GH3X2X_RET_NO_INITED_ERROR;
            GH3X2X_DEBUG_LOG("gh3x2x start error that don't inited!\r\n");
        }
    }
    else
    {
        GH3X2X_DEBUG_LOG("gh3x2x started,restart call stop->start!\r\n");
    }

    return chRet;
}

/**
 * @fn     int8_t GH3X2X_StopSampling(void)
 *
 * @brief  Stop gh3x2x sampling
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK                  return successfully
 * @retval  #GH3X2X_RET_NO_INITED_ERROR     gh3x2x has not inited
 */
int8_t GH3X2X_StopSampling(void)
{
    int8_t chRet = GH3X2X_RET_NO_INITED_ERROR;

    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if ((g_uchGh3x2xStatus == GH3X2X_STATUS_INITED) || (g_uchGh3x2xStatus == GH3X2X_STATUS_STARTED))
    {
        uint16_t usRegVal = 0;
        GH3X2X_WAIT_CHIP_WAKEUP();
        g_uchGh3x2xStatus = GH3X2X_STATUS_INITED;
        usRegVal = gh3020_spi_readreg(GH3020_REG_CARDIFF_CTRL);
        gh3020_spi_writereg(GH3020_REG_CARDIFF_CTRL, GH3X2X_CLEAR_BIT(usRegVal, GH3020_MSK_CARDIFF_CTRL_START));
        #if GH3X2X_ALGORITHM_ECG_SUPPORT
        GH3X2X_LeadDetEnControl(ECG_SAMPLE_EVENT_INFO_SAMPLE_STOP);
        #endif
        GH3X2X_WAIT_CHIP_DSLP();
        chRet = GH3X2X_RET_OK;
    }
    else
    {
        GH3X2X_DEBUG_LOG("gh3x2x stop error that don't inited!\r\n");
    }

    return chRet;
}

/**
 * @fn     uint16_t GH3X2X_GetIrqStatus(void)
 *
 * @brief  Get irq status reg val
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  irq status val, ref irq status mask
 */
uint16_t GH3X2X_GetIrqStatus(void)
{
    uint16_t usIrqRegVal = 0;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    GH3X2X_WAIT_CHIP_WAKEUP();
    usIrqRegVal = gh3020_spi_readreg(GH3020_REG_INT_STR);
    gh3020_spi_writereg(GH3020_REG_INT_STR, usIrqRegVal);
    GH3X2X_WAIT_CHIP_DSLP();
    return (uint16_t)(usIrqRegVal & (uint16_t)GH3020_MSK_INT_STR_ALL_BIT);
}

/**
 * @fn     void GH3X2X_UnpackRawdataPackage(STGh3x2xSlotRawdata *pstSlotRawdataArr,
 *                                    uint8_t *puchReadRawdataBuffer, uint16_t usReadRawdataLen)
 *
 * @brief  Unpack to 8 slot rawdata from read fifo data buffer
 *
 * @attention   This function should use in get rawdata hook;
 *              param pstSlotRawdataArr size must equal 8, define like stSlotRawdataArr[8]. If not define 8,
 *              that mcu will occur array out of bounds error, careful!!!
 *
 * @param[out]  pstSlotRawdataArr       pointer to 8 slot rawdata struct output
 * @param[in]   puchReadRawdataBuffer   pointer to read fifo buffer
 * @param[in]   usReadRawdataLen        read fifo rawdata data len (bytes)
 *
 * @return  None
 */
void GH3X2X_UnpackRawdataPackage(STGh3x2xSlotRawdata *pstSlotRawdataArr,
                                    uint8_t *puchReadRawdataBuffer, uint16_t usReadRawdataLen)
{
    uint16_t usIndex = 0;
    uint8_t  uchSlotNum = 0;
    uint8_t  uchAdcNum = 0;
    uint8_t  uchLastSlotNum = 0;
    uint8_t  uchLastAdcNum = 0;
    uint16_t usRawdataIndex = 0;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    for (usIndex = 0; usIndex < GH3020_SLOT_NUM_MAX; usIndex ++)
    {
        pstSlotRawdataArr[usIndex].usRawdataCnt = 0; // clear cnt
    }
    for (usIndex = 0; usIndex < usReadRawdataLen; usIndex += GH3X2X_FIFO_RAWDATA_SIZE)
    {
        uchSlotNum = GH3X2X_BYTE_RAWDATA_GET_SLOT_NUM(puchReadRawdataBuffer[usIndex]);
        uchAdcNum = GH3X2X_BYTE_RAWDATA_GET_ADC_NUM(puchReadRawdataBuffer[usIndex]);
        if (usIndex == 0) // fixed started
        {
            uchLastSlotNum = uchSlotNum;
            uchLastAdcNum = uchAdcNum;
        }
        if ((uchLastSlotNum != uchSlotNum)
            || ((uchLastSlotNum == uchSlotNum) && (uchAdcNum <= uchLastAdcNum) && (usIndex != 0)))
        {
            pstSlotRawdataArr[uchLastSlotNum].usRawdataCnt++;
        }
        usRawdataIndex = pstSlotRawdataArr[uchSlotNum].usRawdataCnt;
        if (pstSlotRawdataArr[uchSlotNum].punRawdata != GH3X2X_PTR_NULL)
        {
            pstSlotRawdataArr[uchSlotNum].punRawdata[usRawdataIndex][uchAdcNum] = \
                                                        GH3X2X_RAWDATA_CLEAR_SLOT_ADC_NUM(\
                                                            GH3X2X_MAKEUP_DWORD(puchReadRawdataBuffer[usIndex],
                                                                                puchReadRawdataBuffer[usIndex + 1],
                                                                                puchReadRawdataBuffer[usIndex + 2],
                                                                                puchReadRawdataBuffer[usIndex + 3]));
        }
        uchLastSlotNum = uchSlotNum;
        uchLastAdcNum = uchAdcNum;
        if ((usIndex + GH3X2X_FIFO_RAWDATA_SIZE) == usReadRawdataLen)
        {
            pstSlotRawdataArr[uchSlotNum].usRawdataCnt++;
        }
    } // for (usIndex = 0; usIndex < usReadRawdataLen; usIndex += 4)
}

/**
 * @fn     int16_t GH3X2X_GetRawdata(STGh3x2xRawdata *pstGh3x2xRawdata, uint16_t* usFifoLength)
 *
 * @brief  Get rawdata from fifo
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  pstGh3x2xRawdata     pointer to rawdata struct output
 * @param[out]  usFifoLength         pointer to read fifo bytes
 *
 * @return  error code
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return param error
 * @retval  #GH3X2X_RET_READ_FIFO_CONTINUE      return fifo is not empty
 */
int16_t GH3X2X_GetRawdata(STGh3x2xRawdata *pstGh3x2xRawdata, uint16_t* usFifoLength)
{
    uint16_t usIndex = 0;
    int16_t sRet = GH3X2X_RET_PARAMETER_ERROR;

    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if (pstGh3x2xRawdata != GH3X2X_PTR_NULL)
    {
        if (pstGh3x2xRawdata->puchReadBuffer != GH3X2X_PTR_NULL)
        {
            GH3X2X_WAIT_CHIP_WAKEUP();
            *usFifoLength = GH3X2X_FIFO_CNT_CALC(gh3020_spi_readreg( GH3020_REG_INT_FIFO_UR));

            if (*usFifoLength > g_usMaxNumReadFromFifo)
            {
                *usFifoLength = g_usMaxNumReadFromFifo;
                sRet = GH3X2X_RET_READ_FIFO_CONTINUE;
            }

            if ((*usFifoLength > 0) && (*usFifoLength <= GH3X2X_FIFO_DATA_BYTES_MAX_LEN))
            {
                gh3020_spi_readfifo(pstGh3x2xRawdata->puchReadBuffer, *usFifoLength);
            }
            else
            {
                if (*usFifoLength > GH3X2X_FIFO_DATA_BYTES_MAX_LEN)
                {
                    *usFifoLength = 0;
                    GH3X2X_DEBUG_LOG("get rawdata fifo len greater than max, pelease check i2c/spi!\r\n");
                }
                else
                {
                    GH3X2X_DEBUG_LOG("get rawdata fifo equl 0!\r\n");
                }
            }

            /* call hook */
#ifdef CONFIG_FACTEST_SENSORS_GH3020
            gh3020_get_rawdata(pstGh3x2xRawdata->puchReadBuffer, *usFifoLength);
#endif
            GH3X2X_WAIT_CHIP_DSLP();
            GH3X2X_UnpackRawdataPackage(pstGh3x2xRawdata->stSlotRawdataArr,
                                        pstGh3x2xRawdata->puchReadBuffer, *usFifoLength);
        }// if (pstGh3x2xRawdata->puchReadBuffer != GH3X2X_PTR_NULL)
        else // fixed clear cnt if readbuffer ptr is null
        {
            GH3X2X_DEBUG_LOG("get rawdata error that readbuffer is null!\r\n");
            for (usIndex = 0; usIndex < GH3020_SLOT_NUM_MAX; usIndex ++)
            {
                pstGh3x2xRawdata->stSlotRawdataArr[usIndex].usRawdataCnt = 0; // clear cnt
            }
        }
    }// if (pstGh3x2xRawdata != GH3X2X_PTR_NULL)

    return sRet;
}


void GH3X2X_CalRawdataBuf(uint8_t *puchRawdata, uint16_t usRawdataLen)
{

    uint16_t usIndex = usRawdataLen;
    uint8_t uchXorNum = g_uchConfigArrKeyCodeArr[g_uchEncryptRawdataXorIndex];
    uint8_t uchXorXorNum = GH3X2X_CRYP_XOR_XOR_VAL ^ uchXorNum;
    uint8_t uchCntH = GH3X2X_GET_HIGH_BYTE_FROM_WORD(usRawdataLen) ^ uchXorNum;
    uint8_t uchCntL = GH3X2X_GET_LOW_BYTE_FROM_WORD(usRawdataLen) ^ uchXorNum;
    uint32_t unSum = 0;






    puchRawdata[usIndex + 0] = GH3X2X_GET_HIGH_4BITS(uchCntH) | GH3X2X_GET_LOW_4BITS(uchXorNum);
    puchRawdata[usIndex + 1] = GH3X2X_GET_HIGH_4BITS(uchCntL) | GH3X2X_GET_LOW_4BITS(uchCntH);
    puchRawdata[usIndex + 2] = GH3X2X_GET_HIGH_4BITS(uchXorXorNum) | \
                                            GH3X2X_GET_LOW_4BITS(uchCntL);
    puchRawdata[usIndex + 3] = GH3X2X_GET_HIGH_4BITS(uchXorNum) | \
                                            GH3X2X_GET_LOW_4BITS(uchXorXorNum);

    for(uint16_t usByteCnt = 0; usByteCnt < usRawdataLen; usByteCnt += 4)
    {
        unSum += ((uint32_t)(puchRawdata[usByteCnt + 0]^uchXorNum)) << 24;
        unSum += ((uint32_t)(puchRawdata[usByteCnt + 1]^uchXorXorNum)) << 16;
        unSum += ((uint32_t)(puchRawdata[usByteCnt + 2]^uchXorNum)) << 8;
        unSum += ((uint32_t)(puchRawdata[usByteCnt + 3]^uchXorXorNum)) << 0;
    }
    *((uint32_t*)(puchRawdata + usRawdataLen + 4)) = unSum;






    g_uchEncryptRawdataXorIndex = (g_uchEncryptRawdataXorIndex + 1) % (sizeof(g_uchConfigArrKeyCodeArr));
}



uint8_t GH3X2X_CheckRawdataBuf(uint8_t *puchRawdata, uint16_t usRawdataLen)
{
    uint16_t usIndex = usRawdataLen;

    uint8_t uchXorNum = GH3X2X_GET_HIGH_4BITS(puchRawdata[usIndex + 3])
                    | GH3X2X_GET_LOW_4BITS(puchRawdata[usIndex + 0]);
    uint8_t uchCntH = (GH3X2X_GET_HIGH_4BITS(puchRawdata[usIndex + 0])
                | GH3X2X_GET_LOW_4BITS(puchRawdata[usIndex + 1])) ^ uchXorNum;
    uint8_t uchCntL = (GH3X2X_GET_HIGH_4BITS(puchRawdata[usIndex + 1])
                | GH3X2X_GET_LOW_4BITS(puchRawdata[usIndex + 2])) ^ uchXorNum;
    uint8_t uchXorXorNum = GH3X2X_GET_HIGH_4BITS(puchRawdata[usIndex + 2])
                    | GH3X2X_GET_LOW_4BITS(puchRawdata[usIndex + 3]);

    uint32_t unSum = 0;





    if(usRawdataLen != GH3X2X_MAKEUP_WORD(uchCntH, uchCntL))
    {
        GH3X2X_DEBUG_LOG_PARAM("rawdata buffer check fail [code0]!!!\r\n");
        return 0;
    }
    for(uint16_t usByteCnt = 0; usByteCnt < usRawdataLen; usByteCnt += 4)
    {
        unSum += ((uint32_t)(puchRawdata[usByteCnt + 0]^uchXorNum)) << 24;
        unSum += ((uint32_t)(puchRawdata[usByteCnt + 1]^uchXorXorNum)) << 16;
        unSum += ((uint32_t)(puchRawdata[usByteCnt + 2]^uchXorNum)) << 8;
        unSum += ((uint32_t)(puchRawdata[usByteCnt + 3]^uchXorXorNum)) << 0;
    }
    if((0 == usRawdataLen)||(unSum == (*((uint32_t*)(puchRawdata + usRawdataLen + 4)))))
    {
        return 1;
    }
    else
    {
        GH3X2X_DEBUG_LOG_PARAM("rawdata buffer check fail [code1]!!!\r\n");
        return 0;
    }
}
#if 0
void GH3X2X_SetFifoPackageMode(uint8_t uchMode, uint8_t *puchFifoBuffer)
{
    g_uchFifoPackageMode = uchMode;
    gpuchSendFifoPackage = puchFifoBuffer;
}
#endif

__weak uint8_t GH3X2X_GetFifoPackageMode(void)
{
    return g_uchFifoPackageMode;
}

__weak void GH3X2X_SendRawdataFifoPackage(uint8_t *puchGh3x2xReadFifoData, uint16_t usFifoReadByteNum)
{
    if (usFifoReadByteNum == 0)
    {
        return;
    }
    g_uchFifoPackageID ++;
    //SlaverRttLog("[%s]:bytenum = %d, packid = %d\r\n", __FUNCTION__, usFifoReadByteNum, g_uchFifoPackageID);
    uint8_t unRawdataFifoBuffer[220] = {0};
    uint32_t uRawdataBufCnt = 0;
    uint8_t uchIdChangeFlag = 0;
    uint16_t usPackageNum = usFifoReadByteNum / 200;
    uint16_t usLastRawdataFifoBufNum = usFifoReadByteNum % 200;
    if (usLastRawdataFifoBufNum == 0)
    {
        usLastRawdataFifoBufNum = 200;
        usPackageNum--;
    }
    for (uRawdataBufCnt = 0 ; uRawdataBufCnt < usPackageNum ; uRawdataBufCnt ++)
    {
        unRawdataFifoBuffer[0] = g_uchFifoPackageID;
        unRawdataFifoBuffer[1] = 200;
        unRawdataFifoBuffer[2] = uchIdChangeFlag;
        uchIdChangeFlag ++;
        GH3X2X_Memcpy(&unRawdataFifoBuffer[5], &puchGh3x2xReadFifoData[uRawdataBufCnt * 200], (uint32_t)200);
    }
    unRawdataFifoBuffer[0] = g_uchFifoPackageID;
    unRawdataFifoBuffer[1] = (uint8_t)(usLastRawdataFifoBufNum);
    unRawdataFifoBuffer[2] = 0x10 | uchIdChangeFlag;
    GH3X2X_Memcpy(&unRawdataFifoBuffer[5], &puchGh3x2xReadFifoData[usPackageNum * 200], (uint32_t)usLastRawdataFifoBufNum);
}
#if 0
void GH3X2X_PackRawdataFifoPackage(uint8_t *puchGh3x2xReadFifoData, uint16_t usFifoReadByteNum)
{
    if (usFifoReadByteNum == 0)
    {
        return;
    }

    if (usFifoReadByteNum + g_unSendFifoPackageCnt >= GH3X2X_RAWDATA_BUFFER_SIZE)
    {
        uint32_t usFifoReadExternByteNum = GH3X2X_RAWDATA_BUFFER_SIZE - g_unSendFifoPackageCnt;
        GH3X2X_Memcpy(&gpuchSendFifoPackage[g_unSendFifoPackageCnt], puchGh3x2xReadFifoData, usFifoReadExternByteNum);
        g_unSendFifoPackageCnt += usFifoReadExternByteNum;

        GH3X2X_SendRawdataFifoPackage(gpuchSendFifoPackage, g_unSendFifoPackageCnt);

        g_unSendFifoPackageCnt = 0;

        GH3X2X_Memcpy(&gpuchSendFifoPackage[g_unSendFifoPackageCnt],\
            &puchGh3x2xReadFifoData[usFifoReadByteNum - (GH3X2X_RAWDATA_BUFFER_SIZE - g_unSendFifoPackageCnt)],\
            usFifoReadByteNum + g_unSendFifoPackageCnt - GH3X2X_RAWDATA_BUFFER_SIZE);
        g_unSendFifoPackageCnt += usFifoReadByteNum + g_unSendFifoPackageCnt - GH3X2X_RAWDATA_BUFFER_SIZE;
    }
    else
    {
        GH3X2X_Memcpy(&gpuchSendFifoPackage[g_unSendFifoPackageCnt], puchGh3x2xReadFifoData, usFifoReadByteNum);
        g_unSendFifoPackageCnt += usFifoReadByteNum;
    }
}
#endif
/**
 * @fn     int16_t GH3X2X_ReadFifodata(uint8_t *puchGh3x2xReadFifoData, uint16_t* pusReadFifoDataLen)
 *
 * @brief  Read Gh3x2x Fifo Data
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  puchGh3x2xReadFifoData      pointer to Gh3x2x fifo data
 * @param[out]  pusReadFifoDataLen          pointer to Gh3x2x fifo data length in byte
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return read fifo successful
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return param error
 * @retval  #GH3X2X_RET_READ_FIFO_CONTINUE      return fifo is not empty
 */
int16_t GH3X2X_ReadFifodata(uint8_t *puchGh3x2xReadFifoData, uint16_t* pusReadFifoDataLen ,uint16_t usFifoReadByteNum)
{
    //uint16_t usFifoLength = 0;
    int16_t sRet = GH3X2X_RET_OK;

    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if ((puchGh3x2xReadFifoData != GH3X2X_PTR_NULL) && (pusReadFifoDataLen != GH3X2X_PTR_NULL))
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        /*
        usFifoLength = GH3X2X_FIFO_CNT_CALC(gh3020_spi_readreg( GH3020_REG_INT_FIFO_UR));
        */
        if (usFifoReadByteNum > g_usMaxNumReadFromFifo)
        {
            usFifoReadByteNum = g_usMaxNumReadFromFifo;
            sRet = GH3X2X_RET_READ_FIFO_CONTINUE;
        }



        if ((usFifoReadByteNum > 0) && (usFifoReadByteNum <= GH3X2X_FIFO_DATA_BYTES_MAX_LEN))
        {
            gh3020_spi_readfifo(puchGh3x2xReadFifoData, usFifoReadByteNum);
            GH3X2X_CalRawdataBuf(puchGh3x2xReadFifoData, usFifoReadByteNum);
        }
        else
        {
            if (usFifoReadByteNum > GH3X2X_FIFO_DATA_BYTES_MAX_LEN)
            {
                usFifoReadByteNum = 0;
                GH3X2X_DEBUG_LOG("get rawdata fifo len greater than max, pelease check i2c/spi!\r\n");
            }
            else
            {
                GH3X2X_DEBUG_LOG("get rawdata fifo equl 0!\r\n");
            }
        }

        /* call hook */
#ifdef CONFIG_FACTEST_SENSORS_GH3020
        gh3020_get_rawdata(puchGh3x2xReadFifoData, usFifoReadByteNum);
#endif
        GH3X2X_WAIT_CHIP_DSLP();
        (*pusReadFifoDataLen) = usFifoReadByteNum;
    }// if (pstGh3x2xRawdata->puchReadBuffer != GH3X2X_PTR_NULL)
    else
    {
        GH3X2X_DEBUG_LOG("get rawdata error that pointer is null!\r\n");
        sRet = GH3X2X_RET_PARAMETER_ERROR;
    }

    //SET_VAL_VIA_PTR(usFifoReadByteNum, usFifoLength);
    return sRet;
}

/**
 * @fn     uint16_t GH3X2X_FindGu16MinVal(uint16_t *pusBuffer, uint8_t uchLen)
 *
 * @brief  Find min val
 *
 * @attention   len must > 0
 *
 * @param[in]   pusBuffer        pointer to buffer
 * @param[in]   uchLen           buffer length
 * @param[out]  None
 *
 * @return  min val, if len = 0, return 0
 */
uint16_t GH3X2X_FindGu16MinVal(uint16_t *pusBuffer, uint8_t uchLen)
{
    uint16_t usMinVal = pusBuffer[0];
    uint8_t uchIndex = 1;

    for (uchIndex = 1; uchIndex < uchLen; uchIndex++)
    {
        if (pusBuffer[uchIndex] < usMinVal)
        {
            usMinVal = pusBuffer[uchIndex];
        }
    }
    return usMinVal;
}

/**
 * @fn     uint16_t GH3X2X_FindGu16MaxVal(uint16_t *pusBuffer, uint8_t uchLen)
 *
 * @brief  Find max val
 *
 * @attention   len must > 0
 *
 * @param[in]   pusBuffer        pointer to buffer
 * @param[in]   uchLen           buffer length
 * @param[out]  None
 *
 * @return  max val, if len = 0, return 0
 */
uint16_t GH3X2X_FindGu16MaxVal(uint16_t *pusBuffer, uint8_t uchLen)
{
    uint16_t usMaxVal = pusBuffer[0];
    uint8_t uchIndex = 1;

    for (uchIndex = 1; uchIndex < uchLen; uchIndex++)
    {
        if (pusBuffer[uchIndex] > usMaxVal)
        {
            usMaxVal = pusBuffer[uchIndex];
        }
    }
    return usMaxVal;
}

/**
 * @fn     void GH3X2X_FindGu16MaxMinVal(uint16_t *pusMaxVal, uint16_t *pusMinVal, uint16_t *pusBuffer, uint8_t uchLen)
 *
 * @brief  Find min val & max val
 *
 * @attention   len must > 0, ptr not null
 *
 * @param[in]   pusBuffer        pointer to buffer
 * @param[in]   uchLen           buffer length
 * @param[out]  pusMaxVal        pointer to max val
 * @param[out]  pusMinVal        pointer to min val
 *
 * @return  None
 */
void GH3X2X_FindGu16MaxMinVal(uint16_t *pusMaxVal, uint16_t *pusMinVal, uint16_t *pusBuffer, uint8_t uchLen)
{
    uint16_t usMaxVal = pusBuffer[0];
    uint16_t usMinVal = pusBuffer[0];
    uint8_t uchIndex = 1;

    for (uchIndex = 1; uchIndex < uchLen; uchIndex++)
    {
        if (pusBuffer[uchIndex] > usMaxVal)
        {
            usMaxVal = pusBuffer[uchIndex];
        }
        if (pusBuffer[uchIndex] < usMinVal)
        {
            usMinVal = pusBuffer[uchIndex];
        }
    }
    SET_VAL_VIA_PTR(pusMaxVal, usMaxVal);
    SET_VAL_VIA_PTR(pusMinVal, usMinVal);
}

/**
 * @fn     int8_t GH3X2X_UnpackRawdataWithChannelMap(STGh3x2xChannelRawdata *pstGh3x2xChannelRawdata,
 *               uint8_t *puchReadRawdataBuffer, uint16_t usReadRawdataLen, uint8_t uchChannelMapCnt, uint8_t *puchChannelMapArr)
 *
 * @brief  Unpack to channel rawdata from read fifo data buffer;
 *         if last channel rawdata incomplete, should change fifo watermark
 *
 * @attention   This function should use in get rawdata hook
 *
 * @param[in]   puchReadRawdataBuffer       pointer to read data buffer
 * @param[in]   usReadRawdataLen            read data length
 * @param[in]   uchChannelMapCnt            channel map array cnt, max:32
 * @param[in]   puchChannelMapArr           pointer to channel map array
 * @param[out]  pstGh3x2xChannelRawdata     pointer to channel rawdata struct
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return param error
 */
int8_t GH3X2X_UnpackRawdataWithChannelMap(STGh3x2xChannelRawdata *pstGh3x2xChannelRawdata,
                uint8_t *puchReadRawdataBuffer, uint16_t usReadRawdataLen, uint8_t uchChannelMapCnt, uint8_t *puchChannelMapArr)
{
    uint8_t  uchChCntIndex = 0;
    uint16_t usRawdataIndex = 0;
    uint16_t usRawdataCnt = 0;
    uint16_t usRawdataCntMin = 0;
    uint16_t usRawdataCntMax = 0;
    uint16_t usRawdataIndexArr[GH3X2X_CHANNEL_MAP_MAX_CH] = {0}; // use for rawdata cnt & rawdata bytes index
    uint16_t usRawdataByteIndexTmp = 0;
    uint16_t usRawdataBaseIndex = 0;

    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);

    /* check pram */
    if ((pstGh3x2xChannelRawdata->punIncompleteChRawdataArr == GH3X2X_PTR_NULL)
        || (pstGh3x2xChannelRawdata->punChRawdataArr == GH3X2X_PTR_NULL) || (puchReadRawdataBuffer == GH3X2X_PTR_NULL)
        || (pstGh3x2xChannelRawdata == GH3X2X_PTR_NULL) || (puchChannelMapArr == GH3X2X_PTR_NULL))
    {
        GH3X2X_DEBUG_LOG("pointer param set null error!\r\n");
        return GH3X2X_RET_PARAMETER_ERROR;
    }
    for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)
    {
        if (GH3X2X_CLEAR_BIT(puchChannelMapArr[uchChCntIndex], GH3X2X_SLOT_ADC_NUM_BITS) != 0)
        {
            GH3X2X_DEBUG_LOG("channel map error! set @ref GH3X2X_CHANNEL_MAP!\r\n");
            return GH3X2X_RET_PARAMETER_ERROR;
        }
    }
    if (uchChannelMapCnt > GH3X2X_CHANNEL_MAP_MAX_CH)
    {
        GH3X2X_DEBUG_LOG("channel cnt greater than max!\r\n");
        return GH3X2X_RET_PARAMETER_ERROR;
    }

    /* calc rawdata len */
    for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)
    {
        if (GH3X2X_CHECK_LEFT_BIT_SET(pstGh3x2xChannelRawdata->unIncompleteChMark, uchChCntIndex))
        {
            usRawdataIndexArr[uchChCntIndex]++;
        }
        for (usRawdataIndex = 0; usRawdataIndex < usReadRawdataLen; usRawdataIndex += GH3X2X_FIFO_RAWDATA_SIZE)
        {
            if (puchChannelMapArr[uchChCntIndex] == GH3X2X_CHANNEL_MAP_GET(puchReadRawdataBuffer[usRawdataIndex]))
            {
                usRawdataIndexArr[uchChCntIndex]++; // if rawdata channel map equal config(included slot&adc)
            }
        }
    }
    GH3X2X_FindGu16MaxMinVal(&usRawdataCntMax, &usRawdataCntMin, usRawdataIndexArr, uchChannelMapCnt); // calc min&max
    if (usRawdataCntMax == usRawdataCntMin)
    {
        usRawdataCnt = usRawdataCntMax; // whatever last data complete or incomplete
    }
    else
    {
        if ((usRawdataCntMax - usRawdataCntMin) > 1)
        {
            GH3X2X_DEBUG_LOG("rawdata doesn't correspond to channel map error!\r\n");
            usRawdataCnt = 0;
            pstGh3x2xChannelRawdata->unIncompleteChMark = 0;
            GH3X2X_Memset(pstGh3x2xChannelRawdata->punIncompleteChRawdataArr, 0, uchChannelMapCnt * sizeof(uint32_t));
        }
        else // cnt tolerable deviation is 1
        {
            usRawdataCnt = GH3X2X_FindGu16MinVal(usRawdataIndexArr, uchChannelMapCnt);
        }
    } // end of if (usRawdataCntMax == usRawdataCntMin)
    GH3X2X_Memset(usRawdataIndexArr, 0, uchChannelMapCnt * sizeof(uint16_t)); // clear index array, before next use

    if (usRawdataCnt != 0)
    {
        for (usRawdataIndex = 0; usRawdataIndex < usRawdataCnt; usRawdataIndex++)
        {
            usRawdataBaseIndex = (uint16_t)(usRawdataIndex * uchChannelMapCnt);
            for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++) // search each channel
            {
                if (GH3X2X_CHECK_LEFT_BIT_SET(pstGh3x2xChannelRawdata->unIncompleteChMark, uchChCntIndex))
                {
                    pstGh3x2xChannelRawdata->punChRawdataArr[usRawdataBaseIndex + uchChCntIndex] = \
                                                    pstGh3x2xChannelRawdata->punIncompleteChRawdataArr[uchChCntIndex];
                    pstGh3x2xChannelRawdata->punIncompleteChRawdataArr[uchChCntIndex] = 0;
                    GH3X2X_VAL_CLEAR_LEFT_BIT(pstGh3x2xChannelRawdata->unIncompleteChMark, uchChCntIndex);
                }
                else
                {
                    usRawdataByteIndexTmp = usRawdataIndexArr[uchChCntIndex];
                    while (usRawdataByteIndexTmp < usReadRawdataLen) // search each
                    {
                        if (GH3X2X_CHANNEL_MAP_GET(puchReadRawdataBuffer[usRawdataByteIndexTmp]) == \
                                                                    puchChannelMapArr[uchChCntIndex]) // if map equal
                        {
                            pstGh3x2xChannelRawdata->punChRawdataArr[usRawdataBaseIndex + uchChCntIndex] = \
                                                GH3X2X_RAWDATA_CLEAR_SLOT_ADC_NUM(
                                                    GH3X2X_MAKEUP_DWORD(puchReadRawdataBuffer[usRawdataByteIndexTmp],
                                                                    puchReadRawdataBuffer[usRawdataByteIndexTmp + 1],
                                                                    puchReadRawdataBuffer[usRawdataByteIndexTmp + 2],
                                                                    puchReadRawdataBuffer[usRawdataByteIndexTmp + 3]));
                            usRawdataByteIndexTmp += GH3X2X_FIFO_RAWDATA_SIZE;
                            break;
                        }
                        usRawdataByteIndexTmp += GH3X2X_FIFO_RAWDATA_SIZE;
                    }
                    usRawdataIndexArr[uchChCntIndex] = usRawdataByteIndexTmp; // write back byte index
                }
            } // end of for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)
        } // end of for (usRawdataIndex = 0; usRawdataIndex < usRawdataCnt; usRawdataIndex++)
        if (usRawdataCntMax != usRawdataCnt)
        {
            for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++) // search each channel
            {
                usRawdataByteIndexTmp = usRawdataIndexArr[uchChCntIndex];
                while (usRawdataByteIndexTmp < usReadRawdataLen) // search each
                {
                    if (GH3X2X_CHANNEL_MAP_GET(puchReadRawdataBuffer[usRawdataByteIndexTmp]) == \
                                                                puchChannelMapArr[uchChCntIndex]) // if map equal
                    {
                        pstGh3x2xChannelRawdata->punIncompleteChRawdataArr[uchChCntIndex] = \
                                            GH3X2X_RAWDATA_CLEAR_SLOT_ADC_NUM(
                                                GH3X2X_MAKEUP_DWORD(puchReadRawdataBuffer[usRawdataByteIndexTmp],
                                                                puchReadRawdataBuffer[usRawdataByteIndexTmp + 1],
                                                                puchReadRawdataBuffer[usRawdataByteIndexTmp + 2],
                                                                puchReadRawdataBuffer[usRawdataByteIndexTmp + 3]));
                        pstGh3x2xChannelRawdata->unIncompleteChMark |= GH3X2X_GET_LEFT_SHIFT_VAL(uchChCntIndex);
                        break;
                    }
                    usRawdataByteIndexTmp += GH3X2X_FIFO_RAWDATA_SIZE;
                }
                if (GH3X2X_CHECK_LEFT_BIT_NOT_SET(pstGh3x2xChannelRawdata->unIncompleteChMark, uchChCntIndex))
                {
                    pstGh3x2xChannelRawdata->punIncompleteChRawdataArr[uchChCntIndex] = 0;
                }
            }
        } // end of if (usRawdataCntMax != usRawdataCnt)
    } // end of if (usRawdataCnt != 0)
    pstGh3x2xChannelRawdata->usChRawdataCnt = usRawdataCnt;
    return GH3X2X_RET_OK;
}

/**
 * @fn     int16_t GH3X2X_GetRawdataWithChannelMap(STGh3x2xChannelRawdata *pstGh3x2xChannelRawdata,
 *                                       uint8_t *puchReadRawdataBuffer, uint16_t* usFifoLength, uint8_t uchChannelMapCnt,
 *                                       uint8_t *puchChannelMapArr)
 *
 * @brief  Get rawdata from fifo with channel map
 *
 * @attention   None
 *
 * @param[in]   puchReadRawdataBuffer       pointer to read data buffer
 * @param[in]   uchChannelMapCnt            channel map array cnt, max:32
 * @param[in]   puchChannelMapArr           pointer to channel map array
 * @param[out]  pstGh3x2xChannelRawdata     pointer to channel rawdata struct output
 * @param[out]  usFifoLength                pointer to read fifo data length in bytes
 *
 * @return  GH3X2X_RET_OK                       return read fifo successful
 * @retval  #GH3X2X_RET_READ_FIFO_CONTINUE      return fifo is not empty
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return param error
 */
int16_t GH3X2X_GetRawdataWithChannelMap(STGh3x2xChannelRawdata *pstGh3x2xChannelRawdata,
                                        uint8_t *puchReadRawdataBuffer, uint16_t* usFifoLength, uint8_t uchChannelMapCnt,
                                        uint8_t *puchChannelMapArr)
{
    int16_t sRet = GH3X2X_RET_OK;

    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);

    if ((puchReadRawdataBuffer != GH3X2X_PTR_NULL) && (usFifoLength != GH3X2X_PTR_NULL))
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        *usFifoLength = GH3X2X_FIFO_CNT_CALC(gh3020_spi_readreg( GH3020_REG_INT_FIFO_UR));
        if (*usFifoLength > g_usMaxNumReadFromFifo)
        {
            *usFifoLength = g_usMaxNumReadFromFifo;
            sRet = GH3X2X_RET_READ_FIFO_CONTINUE;
        }
        if ((*usFifoLength > 0) && (*usFifoLength <= GH3X2X_FIFO_DATA_BYTES_MAX_LEN))
        {
            gh3020_spi_readfifo(puchReadRawdataBuffer, *usFifoLength);
        }
        else
        {
            if (*usFifoLength > GH3X2X_FIFO_DATA_BYTES_MAX_LEN)
            {
                *usFifoLength = 0;
                GH3X2X_DEBUG_LOG("get rawdata fifo len greater than max, pelease check i2c/spi!\r\n");
            }
            else
            {
                GH3X2X_DEBUG_LOG("get rawdata fifo equl 0!\r\n");
            }
        }

        /* call hook */
#ifdef CONFIG_FACTEST_SENSORS_GH3020
        gh3020_get_rawdata(puchReadRawdataBuffer, *usFifoLength);
#endif
        GH3X2X_WAIT_CHIP_DSLP();
        sRet = GH3X2X_UnpackRawdataWithChannelMap(pstGh3x2xChannelRawdata, puchReadRawdataBuffer, *usFifoLength,
                                                    uchChannelMapCnt, puchChannelMapArr);
    }
    else // fixed clear cnt if readbuffer ptr is null
    {
        GH3X2X_DEBUG_LOG("get rawdata error that readbuffer is null!\r\n");
        sRet = GH3X2X_RET_PARAMETER_ERROR;
    }

    return sRet;
}

/**
 * @fn     int8_t GH3X2X_ChannelMapRawdataClear(STGh3x2xChannelRawdata *pstGh3x2xChannelRawdata)
 *
 * @brief  clear channel map rawdata struct
 *
 * @attention   This function must use before get rawdata by onne kind channel map
 *              e.g. channel map start/stop need clear
 *
 * @param[in]   pstGh3x2xChannelRawdata     pointer to channel rawdata struct
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return param error
 */
int8_t GH3X2X_ChannelMapRawdataClear(STGh3x2xChannelRawdata *pstGh3x2xChannelRawdata)
{
    int8_t chRet = GH3X2X_RET_PARAMETER_ERROR;

    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if (pstGh3x2xChannelRawdata != GH3X2X_PTR_NULL)
    {
        pstGh3x2xChannelRawdata->unIncompleteChMark = 0;
        pstGh3x2xChannelRawdata->usChRawdataCnt = 0;
        chRet = GH3X2X_RET_OK;
    }
    else
    {
        GH3X2X_DEBUG_LOG("channel rawdata param is null!\r\n");
    }
    return chRet;
}

/**
 * @fn     int8_t GH3X2X_SlotEnableConfig(uint16_t usSlotEnableConfig, EMSlotEnableConfigType emSlotEnableConfigType)
 *
 * @brief  Slot enable config
 *
 * @attention  Set slot enable or disable, if just need enable slot by enable config param
 *
 * @param[in]  usSlotEnableConfig         slot enable index , @ref GH3X2X_SLOT_INDEX_0 ... GH3X2X_SLOT_INDEX_ALL
 * @param[in]  emSlotEnableConfigType     slot config type, @ref EMSlotEnableConfigType
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK                  return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR     return param error
 */
int8_t GH3X2X_SlotEnableConfig(uint16_t usSlotEnableConfig, EMSlotEnableConfigType emSlotEnableConfigType)
{
    int8_t chRet = GH3X2X_RET_OK;
    uint16_t usSlotEnableCfgRegVal = 0;

    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if (GH3X2X_CHECK_BIT_SET(GH3X2X_SLOT_INDEX_ALL, usSlotEnableConfig))
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        switch (emSlotEnableConfigType)
        {
        case GH3X2X_SET_SLOT_ENABLE:
            usSlotEnableCfgRegVal = gh3020_spi_readreg(GH3020_REG_SLOT_ENABLE_CFG);
            gh3020_spi_writereg(GH3020_REG_SLOT_ENABLE_CFG,
                            GH3X2X_SET_BIT(usSlotEnableCfgRegVal, usSlotEnableConfig) & GH3X2X_SLOT_INDEX_ALL);
            break;
        case GH3X2X_SET_SLOT_DISABLE:
            usSlotEnableCfgRegVal = gh3020_spi_readreg(GH3020_REG_SLOT_ENABLE_CFG);
            gh3020_spi_writereg(GH3020_REG_SLOT_ENABLE_CFG,
                            GH3X2X_CLEAR_BIT(usSlotEnableCfgRegVal, usSlotEnableConfig) & GH3X2X_SLOT_INDEX_ALL);
            break;
        default: // include GH3X2X_SET_SLOT_DIRECT_ENABLE mode
            gh3020_spi_writereg(GH3020_REG_SLOT_ENABLE_CFG, usSlotEnableConfig & GH3X2X_SLOT_INDEX_ALL);
            break;
        }
        GH3X2X_WAIT_CHIP_DSLP();
    }
    else
    {
        GH3X2X_DEBUG_LOG("slot enable config error!\r\n");
        chRet = GH3X2X_RET_PARAMETER_ERROR;
    }
    return chRet;
}

/**
 * @fn     void GH3X2X_SlotEnRegSet(uint8_t uchSetValue)
 *
 * @brief  Slot enable reg set
 *
 * @attention  None
 *
 * @param[in]  set value
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_SlotEnRegSet(uint8_t uchSetValue)
{
    GH3X2X_DEBUG_LOG_PARAM("[SlotEnRegSet] set slot en: 0x%X\r\n",uchSetValue);
    gh3020_spi_writereg(GH3020_REG_SLOT_ENABLE_CFG,
                (uint16_t)uchSetValue);
}



/**
 * @fn     int8_t GH3X2X_FifoWatermarkThrConfig(uint16_t usFifoWatermarkThr)
 *
 * @brief  Fifo water mark threshold config
 *
 * @attention   Watermark threshold val must in (2, 800]. if val <= 2, will set 3, if val > 800, set 800;
 *              Be careful that fifo_use_cnt greater than val, gh3x2x willn't generate fifo_watermark interrupt after!
 *
 * @param[in]   usFifoWatermarkThr         watermark threshold val
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 */
int8_t GH3X2X_FifoWatermarkThrConfig(uint16_t usFifoWatermarkThr)
{
    uint16_t usFifoWatermarkThrVal = usFifoWatermarkThr;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if (usFifoWatermarkThrVal < GH3X2X_FIFO_WATERMARK_THR_MIN)
    {
        GH3X2X_DEBUG_LOG("fifo wateramrk thr param less than min!\r\n");
        usFifoWatermarkThrVal = GH3X2X_FIFO_WATERMARK_THR_MIN;
    }
    else if (usFifoWatermarkThrVal > GH3X2X_FIFO_WATERMARK_THR_MAX)
    {
        GH3X2X_DEBUG_LOG("fifo wateramrk thr param greater than max!\r\n");
        usFifoWatermarkThrVal = GH3X2X_FIFO_WATERMARK_THR_MAX;
    }
    GH3X2X_WAIT_CHIP_WAKEUP();
    gh3020_spi_writereg(GH3020_REG_FIFO_WATERLINE, usFifoWatermarkThrVal);
    GH3X2X_WAIT_CHIP_DSLP();
    return GH3X2X_RET_OK;
}

/**
 * @fn     uint16_t GH3X2X_GetFifoWatermarkThr(void)
 *
 * @brief  Get fifo water mark threshold
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  fifo water mark threshold
 */
uint16_t GH3X2X_GetFifoWatermarkThr(void)
{
    uint16_t usFifoWatermarkThrVal = 0;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    GH3X2X_WAIT_CHIP_WAKEUP();
    usFifoWatermarkThrVal = gh3020_spi_readreg(GH3020_REG_FIFO_WATERLINE);
    GH3X2X_WAIT_CHIP_DSLP();
    return usFifoWatermarkThrVal;
}

/**
 * @fn     int8_t GH3X2X_SlotLedCurrentConfig(uint8_t uchSlotIndex, uint8_t uchDrvIndex, uint8_t uchCurrentVal)
 *
 * @brief  Slot led current config
 *
 * @attention   None
 *
 * @param[in]   uchSlotIndex        slot index [0-7]
 * @param[in]   uchDrvIndex         driver index @ref GH3X2X_LED_DRV_INDEX_0 ... GH3X2X_LED_DRV_INDEX_1
 * @param[in]   uchCurrentVal       current val
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR     return param error
 */
int8_t GH3X2X_SlotLedCurrentConfig(uint8_t uchSlotIndex, uint8_t uchDrvIndex, uint8_t uchCurrentVal)
{
    uint16_t usCurrentRegVal = 0;
    uint16_t usSlotDrvRegAddr = 0;
    int8_t chRet = GH3X2X_RET_PARAMETER_ERROR;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if ((uchSlotIndex < GH3020_SLOT_NUM_MAX) && (uchDrvIndex < GH3020_SLOT_LED_DRV_NUM_MAX))
    {
        usSlotDrvRegAddr = GH3020_REG_SLOT0_CTRL_10 + (uchSlotIndex * GH3020_OFFSET_SLOT_CTRL)\
                             + (uchDrvIndex * GH3020_REG_ADDR_SIZE);
        GH3X2X_WAIT_CHIP_WAKEUP();
        usCurrentRegVal = gh3020_spi_readreg(usSlotDrvRegAddr);
        GH3X2X_VAL_CLEAR_BIT(usCurrentRegVal, (uint16_t)GH3020_MSK_SLOT_LED_CURRENT_CLEAR);
        GH3X2X_VAL_SET_BIT(usCurrentRegVal, (uint16_t)uchCurrentVal);
        gh3020_spi_writereg(usSlotDrvRegAddr, usCurrentRegVal);
        GH3X2X_WAIT_CHIP_DSLP();
        chRet = GH3X2X_RET_OK;
    }
    else
    {
        if (uchSlotIndex >= GH3020_SLOT_NUM_MAX)
        {
            GH3X2X_DEBUG_LOG("slot index param greater than max!\r\n");
        }
        else // uchDrvIndex >= GH3020_SLOT_LED_DRV_NUM_MAX
        {
            GH3X2X_DEBUG_LOG("drv index param greater than max!\r\n");
        }
    }
    return chRet;
}

/**
 * @fn     uint8_t GH3X2X_GetSlotLedCurrent(uint8_t uchSlotIndex, uint8_t uchDrvIndex)
 *
 * @brief  Get slot led current config
 *
 * @attention   None
 *
 * @param[in]   uchSlotIndex        slot index [0-7]
 * @param[in]   uchDrvIndex         driver index @ref GH3X2X_LED_DRV_INDEX_0 ... GH3X2X_LED_DRV_INDEX_1
 * @param[out]  None
 *
 * @return  current val. if param error, return val always equal 0
 */
__weak uint8_t GH3X2X_GetSlotLedCurrent(uint8_t uchSlotIndex, uint8_t uchDrvIndex)
{
    uint16_t usCurrentRegVal = 0;
    uint16_t usSlotDrvRegAddr = 0;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if ((uchSlotIndex < GH3020_SLOT_NUM_MAX) && (uchDrvIndex < GH3020_SLOT_LED_DRV_NUM_MAX))
    {
        usSlotDrvRegAddr = GH3020_REG_SLOT0_CTRL_10 + (uchSlotIndex * GH3020_OFFSET_SLOT_CTRL)\
                             + (uchDrvIndex * GH3020_REG_ADDR_SIZE);
        GH3X2X_WAIT_CHIP_WAKEUP();
        usCurrentRegVal = gh3020_spi_readreg(usSlotDrvRegAddr);
        GH3X2X_WAIT_CHIP_DSLP();
    }
    else
    {
        if (uchSlotIndex >= GH3020_SLOT_NUM_MAX)
        {
            GH3X2X_DEBUG_LOG("slot index param greater than max!\r\n");
        }
        else // uchDrvIndex >= GH3020_SLOT_LED_DRV_NUM_MAX
        {
            GH3X2X_DEBUG_LOG("drv index param greater than max!\r\n");
        }
    }
    return GH3X2X_GET_LOW_BYTE_FROM_WORD(usCurrentRegVal);
}

/**
 * @fn     int8_t GH3X2X_SlotLedTiaGainConfig(uint8_t uchSlotIndex, uint8_t uchAdcIndex, uint8_t uchGainVal)
 *
 * @brief  Slot gain config
 *
 * @attention   None
 *
 * @param[in]   uchSlotIndex        slot index [0-7]
 * @param[in]   uchAdcIndex         adc index [0-3]
 * @param[in]   uchGainVal          tia gain val [0-12]
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK                  return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR     return param error
 */
int8_t GH3X2X_SlotLedTiaGainConfig(uint8_t uchSlotIndex, uint8_t uchAdcIndex, uint8_t uchGainVal)
{
    uint16_t usGainRegVal = 0;
    uint16_t usSlotTiaGainRegAddr = 0;
    int8_t chRet = GH3X2X_RET_PARAMETER_ERROR;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if ((uchSlotIndex < GH3020_SLOT_NUM_MAX) && (uchAdcIndex < GH3020_SLOT_TIA_GAIN_NUM_MAX)
        && (uchGainVal < GH3020_SLOT_TIA_GAIN_VAL_MAX))
    {
        usSlotTiaGainRegAddr = GH3020_REG_SLOT0_CTRL_4 + (uchSlotIndex * GH3020_OFFSET_SLOT_CTRL);
        GH3X2X_WAIT_CHIP_WAKEUP();
        usGainRegVal = gh3020_spi_readreg(usSlotTiaGainRegAddr);
        GH3X2X_VAL_CLEAR_BIT(usGainRegVal,
                                (GH3020_MSK_SLOT_TIA_GAIN << (GH3020_SLOT_TIA_GAIN_BITS_SIZE * uchAdcIndex)));
        GH3X2X_VAL_SET_BIT(usGainRegVal, (((uint16_t)uchGainVal) << (GH3020_SLOT_TIA_GAIN_BITS_SIZE * uchAdcIndex)));
        gh3020_spi_writereg(usSlotTiaGainRegAddr, usGainRegVal);
        GH3X2X_WAIT_CHIP_DSLP();
        chRet = GH3X2X_RET_OK;
    }
    else
    {
        if (uchSlotIndex >= GH3020_SLOT_NUM_MAX)
        {
            GH3X2X_DEBUG_LOG("slot index param greater than max!\r\n");
        }
        else if (uchAdcIndex >= GH3020_SLOT_TIA_GAIN_NUM_MAX)
        {
            GH3X2X_DEBUG_LOG("adc index param greater than max!\r\n");
        }
        else
        {
            GH3X2X_DEBUG_LOG("gain val param greater than max!\r\n");
        }
    }
    return chRet;
}

/**
 * @fn     uint8_t GH3X2X_GetSlotLedTiaGain(uint8_t uchSlotIndex, uint8_t uchAdcIndex)
 *
 * @brief  Get slot gain config
 *
 * @attention   None
 *
 * @param[in]   uchSlotIndex        slot index [0-7]
 * @param[in]   uchAdcIndex         adc index [0-3]
 * @param[out]  None
 *
 * @return  tia gain val. if param error, return val always equal 0
 */
uint8_t GH3X2X_GetSlotLedTiaGain(uint8_t uchSlotIndex, uint8_t uchAdcIndex)
{
    uint16_t usGainRegVal = 0;
    uint16_t usSlotTiaGainRegAddr = 0;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if ((uchSlotIndex < GH3020_SLOT_NUM_MAX) && (uchAdcIndex < GH3020_SLOT_TIA_GAIN_NUM_MAX))
    {
        usSlotTiaGainRegAddr = GH3020_REG_SLOT0_CTRL_4 + (uchSlotIndex * GH3020_OFFSET_SLOT_CTRL);
        GH3X2X_WAIT_CHIP_WAKEUP();
        usGainRegVal = gh3020_spi_readreg(usSlotTiaGainRegAddr);
        GH3X2X_WAIT_CHIP_DSLP();
    }
    else
    {
        if (uchSlotIndex >= GH3020_SLOT_NUM_MAX)
        {
            GH3X2X_DEBUG_LOG("slot index param greater than max!\r\n");
        }
        else // uchAdcIndex >= GH3020_SLOT_TIA_GAIN_NUM_MAX
        {
            GH3X2X_DEBUG_LOG("adc index param greater than max!\r\n");
        }
    }
    return (uint8_t)(((usGainRegVal) >> (GH3020_SLOT_TIA_GAIN_BITS_SIZE * uchAdcIndex)) & GH3020_MSK_SLOT_TIA_GAIN);
}

/**
 * @fn     int8_t GH3X2X_AdcBgcThrConfig(uint8_t uchAdcIndex, uint8_t uchBgcThrVal)
 *
 * @brief  adc bgc thr config
 *
 * @attention   None
 *
 * @param[in]   uchAdcIndex         adc index [0-3]
 * @param[in]   uchBgcThrVal        bgc thr val [0-7]
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK                  return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR     return param error
 */
int8_t GH3X2X_AdcBgcThrConfig(uint8_t uchAdcIndex, uint8_t uchBgcThrVal)
{
    uint16_t usBgcThregVal = 0;
    int8_t chRet = GH3X2X_RET_PARAMETER_ERROR;
    uint8_t uchKdcCh = (uint8_t)(GH3020_SLOT_KDC_THR_ADC_INDEX_MAX - uchAdcIndex);
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if ((uchAdcIndex < GH3020_SLOT_TIA_GAIN_NUM_MAX) && (uchBgcThrVal < GH3020_SLOT_KDC_THR_VAL_MAX))
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        usBgcThregVal = gh3020_spi_readreg(GH3020_REG_PPG_DAC_AD_REG1);
        GH3X2X_VAL_CLEAR_BIT(usBgcThregVal,
                                (GH3020_MSK_SLOT_KDC_THR << (GH3020_SLOT_KDC_THR_BITS_SIZE * uchKdcCh)));
        GH3X2X_VAL_SET_BIT(usBgcThregVal, (((uint16_t)uchBgcThrVal) << (GH3020_SLOT_KDC_THR_BITS_SIZE * uchKdcCh)));
        gh3020_spi_writereg(GH3020_REG_PPG_DAC_AD_REG1, usBgcThregVal);
        GH3X2X_WAIT_CHIP_DSLP();
        chRet = GH3X2X_RET_OK;
    }
    else
    {
        if (uchAdcIndex >= GH3020_SLOT_TIA_GAIN_NUM_MAX)
        {
            GH3X2X_DEBUG_LOG("adc index param greater than max!\r\n");
        }
        else
        {
            GH3X2X_DEBUG_LOG("bgc thr val param greater than max!\r\n");
        }
    }
    return chRet;
}

/**
 * @fn     uint8_t GH3X2X_GetAdcBgcThr(uint8_t uchAdcIndex)
 *
 * @brief  Get adc bgc thr config
 *
 * @attention   None
 *
 * @param[in]   uchAdcIndex         adc index [0-3]
 * @param[out]  None
 *
 * @return  tia gain val. if param error, return val always equal 0
 */
uint8_t GH3X2X_GetAdcBgcThr(uint8_t uchAdcIndex)
{
    uint16_t usBgcThregVal = 0;
    uint8_t uchKdcCh = 0;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if (uchAdcIndex < GH3020_SLOT_TIA_GAIN_NUM_MAX)
    {
        uchKdcCh = (uint8_t)(GH3020_SLOT_KDC_THR_ADC_INDEX_MAX - uchAdcIndex);
        GH3X2X_WAIT_CHIP_WAKEUP();
        usBgcThregVal = gh3020_spi_readreg(GH3020_REG_PPG_DAC_AD_REG1);
        GH3X2X_WAIT_CHIP_DSLP();
    }
    else
    {
        GH3X2X_DEBUG_LOG("adc index param greater than max!\r\n");
    }
    return (uint8_t)(((usBgcThregVal) >> (GH3020_SLOT_KDC_THR_BITS_SIZE * uchKdcCh)) & GH3020_MSK_SLOT_KDC_THR);
}

/**
 * @fn     int8_t GH3X2X_IrqWidthConfig(uint16_t usIrqPulseWidth, uint16_t usIrqColdWidth)
 *
 * @brief  Irq width config
 *
 * @attention   Irq val must in [0, 2047]. real_width = (val + 1) * low_clk_tick;
 *              low_clk default 32k, if set ext clk enable that use low_ext_clk;
 *
 * @param[in]   usIrqPulseWidth         irq pulse width val
 * @param[in]   usIrqColdWidth          irq cold width val
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 */
int8_t GH3X2X_IrqWidthConfig(uint16_t usIrqPulseWidth, uint16_t usIrqColdWidth)
{
    uint16_t usIrqPulseWidthVal = usIrqPulseWidth;
    uint16_t usIrqColdWidthVal = usIrqColdWidth;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if (usIrqPulseWidthVal > GH3X2X_IRQ_WIDTH_VAL_MAX)
    {
        GH3X2X_DEBUG_LOG("irq pulse width param greater than max!\r\n");
        usIrqPulseWidthVal = GH3X2X_IRQ_WIDTH_VAL_MAX;
    }
    if (usIrqColdWidthVal > GH3X2X_IRQ_WIDTH_VAL_MAX)
    {
        GH3X2X_DEBUG_LOG("irq cold width param greater than max!\r\n");
        usIrqColdWidthVal = GH3X2X_IRQ_WIDTH_VAL_MAX;
    }
    GH3X2X_WAIT_CHIP_WAKEUP();
    gh3020_spi_writereg(GH3020_REG_INT_PWR, usIrqPulseWidthVal);
    gh3020_spi_writereg(GH3020_REG_INT_CTR, usIrqColdWidthVal);
    GH3X2X_WAIT_CHIP_DSLP();
    return GH3X2X_RET_OK;
}

/**
 * @fn     int8_t GH3X2X_IrqModeConfig(EMIrqModeConfig emIrqMode)
 *
 * @brief  Irq mode config
 *
 * @attention   None
 *
 * @param[in]   emIrqMode         irq mode config @ref EMIrqModeConfig
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 */
int8_t GH3X2X_IrqModeConfig(EMIrqModeConfig emIrqMode)
{
    uint16_t usIrqCtrlRegVal = 0;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    GH3X2X_WAIT_CHIP_WAKEUP();
    usIrqCtrlRegVal = gh3020_spi_readreg(GH3020_REG_INT_CR);
    GH3X2X_VAL_CLEAR_BIT(usIrqCtrlRegVal, (uint16_t)GH3020_MSK_INT_CTRL_MODE_BIT);
    GH3X2X_VAL_SET_BIT(usIrqCtrlRegVal, ((uint16_t)emIrqMode) & GH3020_MSK_INT_CTRL_MODE_BIT);
    gh3020_spi_writereg(GH3020_REG_INT_CR, usIrqCtrlRegVal);
    GH3X2X_WAIT_CHIP_DSLP();
    return GH3X2X_RET_OK;
}

/**
 * @fn     void GH3X2X_RegisterResetPinControlFunc(void (*pResetPinLevelControlFunc)(uint8_t uchPinLevel))
 *
 * @brief  Register reset pin level control function
 *
 * @attention   Pin level set val should define as 1 [high level] or 0 [low level];
 *
 * @param[in]   pResetPinLevelControlFunc       pointer to set reset pin level function
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_RegisterResetPinControlFunc(void (*pResetPinLevelControlFunc)(uint8_t uchPinLevel))
{
    g_pGh3x2xResetPinLevelControlFunc = pResetPinLevelControlFunc;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
}

/**
 * @fn     int8_t GH3X2X_HardReset(void)
 *
 * @brief  Gh3x2x softreset via i2c/spi, can read&write reg with gh3x2x after reset
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR    reset pin control function not register
 */
int8_t GH3X2X_HardReset(void)
{
    int8_t chRet = GH3X2X_RET_GENERIC_ERROR;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if (g_pGh3x2xResetPinLevelControlFunc != GH3X2X_PTR_NULL)
    {
        g_pGh3x2xResetPinLevelControlFunc(0);
        usleep(GH3X2X_HARD_RESET_DELAY_X_US); /* hard reset delay 20us. */
        g_pGh3x2xResetPinLevelControlFunc(1);
        chRet = GH3X2X_RET_OK;
    }
    else
    {
        GH3X2X_DEBUG_LOG("reset pin ctrl func has not register!\r\n");
    }
    g_uchActiveChipResetFlag = 1;
    if (g_uchResetFromProtocolFlag == 1)
    {
        g_uchResetFromProtocolFlag = 0;
        usleep(20000);
        gh3020_fifo_process();
    }
    return chRet;
}

/**
 * @fn     void GH3X2X_WearDetectEnable(EMWearDetectEnableType emWearDetectEnable)
 *
 * @brief  enable or disable wear detect
 *
 * @attention   None
 *
 * @param[in]   emWearDetectEnable      enable or disable @ref EMWearDetectEnableType
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_WearDetectEnable(EMWearDetectEnableType emWearDetectEnable)
{
    uint16_t usWearStatus = 0;

    GH3X2X_WAIT_CHIP_WAKEUP();
    usWearStatus = gh3020_spi_readreg(GH3020_REG_ADT_WEARON_CR);

    if (WEAR_DETECT_ENABLE == emWearDetectEnable)
    {
        GH3X2X_VAL_SET_BIT(usWearStatus, GH3020_MSK_ADT_WEAR_DET_EN);
    }
    else
    {
        GH3X2X_VAL_CLEAR_BIT(usWearStatus, (uint16_t)GH3020_MSK_ADT_WEAR_DET_EN);
    }

    gh3020_spi_writereg(GH3020_REG_ADT_WEARON_CR, usWearStatus);
    GH3X2X_WAIT_CHIP_DSLP();
}

/**
 * @fn     int8_t GH3X2X_WearDetectSwitchTo(EMWearDetectType emWearDetectType, EMWearDetectForceSwitch emForceSwitch)
 *
 * @brief  Gh3x2x switch to detect wear on/off type
 *
 * @attention   Should follow that use WEAR_DETECT_DONT_FORCE_SWITCH @wear on/off irq status process.
 *              If want force switch to detect on/off, should use WEAR_DETECT_FORCE_SWITCH. careful!!!
 *
 * @param[in]   emWearDetectType        switch to detect type @ref EMWearDetectType
 * @param[in]   emForceSwitch           force switch type @ref EMWearDetectForceSwitch
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK                  return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR       detect type equal last type, don't need to switch
 * @retval  #GH3X2X_RET_PARAMETER_ERROR     return param error
 */
int8_t GH3X2X_WearDetectSwitchTo(EMWearDetectType emWearDetectType, EMWearDetectForceSwitch emForceSwitch)
{
    int8_t chRet = GH3X2X_RET_GENERIC_ERROR;
    uint16_t usWearStatus = 0;
    uint16_t usWearLogicSel = 0;

    GH3X2X_DEBUG_LOG_PARAM("%s:type = %d\r\n", __FUNCTION__, emWearDetectType);
    if (emWearDetectType > WEAR_DETECT_WEAR_OFF)
    {
        GH3X2X_DEBUG_LOG("param set error! @ref EMWearDetectType\r\n");
        chRet = GH3X2X_RET_PARAMETER_ERROR;
    }
    else
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        usWearStatus = gh3020_spi_readreg(GH3020_REG_ADT_WEARON_CR);
        GH3X2X_DEBUG_LOG_PARAM("[GH3X2X_WearDetectSwitchTo] usWearStatus = %d\n",(int)usWearStatus);
        if (emWearDetectType == WEAR_DETECT_WEAR_ON) // switch to detect wear on (got wear off evt)
        {
            if ((GH3X2X_CHECK_BIT_SET(usWearStatus, GH3020_MSK_ADT_WEAR_STATUS)) \
                || (emForceSwitch != WEAR_DETECT_DONT_FORCE_SWITCH)) // system is wear on
            {
                GH3X2X_DEBUG_LOG_PARAM("[GH3X2X_WearDetectSwitchTo] WEAR_DETECT_WEAR_ON\n");
#if (GH3X2X_WEAR_ON_FORCE_SWITCH_WITH_LOGIC_SEL) // wear on force switch with adt logic sel
                GH3X2X_VAL_CLEAR_BIT(usWearStatus, (uint16_t)GH3020_MSK_ADT_WEAR_STATUS);
                if (emForceSwitch != WEAR_DETECT_DONT_FORCE_SWITCH)
                {
                    usWearLogicSel = gh3020_spi_readreg(GH3020_REG_ADT_WEARON_LOGIC_SEL);
                    gh3020_spi_writereg(GH3020_REG_ADT_WEARON_LOGIC_SEL, 0); // clear all logic
                    gh3020_spi_writereg(GH3020_REG_ADT_WEARON_CR, usWearStatus); // set system into wear off
                    gh3020_spi_writereg(GH3020_REG_ADT_WEARON_CR, \
                                    GH3X2X_SET_BIT(usWearStatus, GH3020_REGVAL_ADT_WEAR_CR_WEAR_ON));
                    usleep(GH3X2X_WEAR_DETECT_SWITCH_WAIT_X_US);
                    gh3020_spi_writereg(GH3020_REG_ADT_WEARON_LOGIC_SEL, usWearLogicSel); // re-sel wear on logic
                }
                else
                {
                    gh3020_spi_writereg(GH3020_REG_ADT_WEARON_CR, usWearStatus); // set system into wear off
                }
#else
                GH3X2X_VAL_CLEAR_BIT(usWearStatus, (uint16_t)GH3020_MSK_ADT_WEAR_STATUS);
                if ((GH3X2X_CHECK_BIT_SET(usWearStatus, GH3020_MSK_ADT_WEAR_DET_EN))
                    && (emForceSwitch != WEAR_DETECT_DONT_FORCE_SWITCH))
                {
                    gh3020_spi_writereg(GH3020_REG_ADT_WEARON_CR, \
                                        GH3X2X_CLEAR_BIT(usWearStatus, GH3020_MSK_ADT_WEAR_DET_EN));

                              GH3X2X_DEBUG_LOG_PARAM("[GH3X2X_WearDetectSwitchTo] write det en to 0 !!!\n");
                }
                gh3020_spi_writereg(GH3020_REG_ADT_WEARON_CR, usWearStatus); // set system into wear off

                        GH3X2X_DEBUG_LOG_PARAM("[GH3X2X_WearDetectSwitchTo] usWearStatus = %d\n",(int)usWearStatus);


#endif
                chRet = GH3X2X_RET_OK;
            } // end of if (GH3X2X_CHECK_BIT_SET(usWearStatus, GH3020_MSK_ADT_WEAR_STATUS))
        } // end of if (emWearDetectType == WEAR_DETECT_WEAR_ON)
        else // switch to detect wear off (got wear on evt)
        {
            if (GH3X2X_CHECK_BIT_NOT_SET(usWearStatus, GH3020_MSK_ADT_WEAR_STATUS) \
                || (emForceSwitch != WEAR_DETECT_DONT_FORCE_SWITCH)) // system is wear off
            {
                GH3X2X_DEBUG_LOG_PARAM("[GH3X2X_WearDetectSwitchTo] WEAR_DETECT_WEAR_OFF\n");
                GH3X2X_VAL_SET_BIT(usWearStatus, GH3020_MSK_ADT_WEAR_STATUS);
                if (emForceSwitch != WEAR_DETECT_DONT_FORCE_SWITCH)
                {
                    usWearLogicSel = gh3020_spi_readreg(GH3020_REG_ADT_WEAROFF_LOGIC_SEL);
                    gh3020_spi_writereg(GH3020_REG_ADT_WEAROFF_LOGIC_SEL, 0); // clear all logic
                    gh3020_spi_writereg(GH3020_REG_ADT_WEARON_CR, usWearStatus); // set system into wear on
                    gh3020_spi_writereg(GH3020_REG_ADT_WEARON_CR, \
                                    GH3X2X_SET_BIT(usWearStatus, GH3020_REGVAL_ADT_WEAR_CR_WEAR_OFF));
                    usleep(GH3X2X_WEAR_DETECT_SWITCH_WAIT_X_US);
                    gh3020_spi_writereg(GH3020_REG_ADT_WEAROFF_LOGIC_SEL, usWearLogicSel); // re-sel wear off logic
                }
                else
                {
                    gh3020_spi_writereg(GH3020_REG_ADT_WEARON_CR, usWearStatus); // set system into wear on
                }
                chRet = GH3X2X_RET_OK;
            }
        }
        GH3X2X_WAIT_CHIP_DSLP();
    } // end of else if (emWearDetectType > WEAR_DETECT_WEAR_OFF)
    return chRet;
}

/**
 * @fn     EMSkinColorStatusType GH3X2X_GetSkinStatus(void)
 *
 * @brief  Get gh3x2x skin color status
 *
 * @attention   Must using when chip hard wear detect module enable, if not always return 0
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  skin color status @ref EMSkinColorStatusType
 */
EMSkinColorStatusType GH3X2X_GetSkinStatus(void)
{
    uint16_t usSkinStatus = 0;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    GH3X2X_WAIT_CHIP_WAKEUP();
    usSkinStatus = gh3020_spi_readreg(GH3020_REG_SKIN_STR);
    GH3X2X_WAIT_CHIP_DSLP();
    return ((EMSkinColorStatusType) GH3X2X_VAL_GET_BIT(usSkinStatus, GH3020_MSK_SKIN_COLOR_STATUS));
}

/**
 * @fn     int8_t GH3X2X_HsiClockCalibration(void)
 *
 * @brief  Calibration Hsi(high-speed internal) clock
 *
 * @attention   must use 4M clock input @OSCIN pin
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK                  return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR       input clock is unstable, or sampling was started
 */
int8_t GH3X2X_HsiClockCalibration(void)
{
    int8_t chRet = GH3X2X_RET_GENERIC_ERROR;

    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if (g_uchGh3x2xStatus < GH3X2X_STATUS_STARTED)
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        gh3020_spi_writereg(GH3020_REG_OSC_THR, GH3X2X_HSI_CALI_THR_CNT_VAL);
        gh3020_spi_writereg(GH3020_REG_OSC_CR, GH3X2X_HSI_CALI_CTRL_EN_VAL);
        usleep(GH3X2X_HSI_CALI_DELAY_VAL);
        if (GH3X2X_CHECK_BIT_SET (gh3020_spi_readreg(GH3020_REG_OSC_FLAG), GH3X2X_OSC_CALI_CODE_LOACKED))
        {
            chRet = GH3X2X_RET_OK;
        }
        GH3X2X_DEBUG_LOG_PARAM("Hsi: 0x%.4x\r\n", gh3020_spi_readreg(GH3020_REG_OSC13M_TUNE));
        gh3020_spi_writereg(GH3020_REG_OSC_CR, GH3X2X_OSC_CALI_CTRL_DIS_VAL);
        GH3X2X_WAIT_CHIP_DSLP();
    }

    return chRet;
}

/**
 * @fn     int8_t GH3X2X_LsiClockCalibration(void)
 *
 * @brief  Calibration Lsi(low-speed internal) clock
 *
 * @attention   must use 4M clock input @OSCIN pin, take case that will consume about 100ms
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK                  return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR       input clock is unstable, or sampling was started
 */
int8_t GH3X2X_LsiClockCalibration(void)
{
    int8_t chRet = GH3X2X_RET_GENERIC_ERROR;
    uint16_t usCaliVal = 0;
    int16_t sFreqErrVal = 0;
    int16_t sLastFreqErrVal = GH3X2X_LSI_CALI_ERR_MAX_VAL;
    uint8_t uchIndex = 0;
    uint8_t uchMinVal = 0;
    uint8_t uchMaxVal = GH3X2X_LSI_CALI_FINE_VAL_MAX;
    uint8_t uchCalcVal = 0;
    uint8_t uchLastCalcVal = GH3X2X_LSI_CALI_FINE_VAL_MAX;

    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if (g_uchGh3x2xStatus < GH3X2X_STATUS_STARTED)
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        gh3020_spi_writereg(GH3020_REG_OSC_THR, GH3X2X_LSI_CALI_THR_CNT_VAL);
        gh3020_spi_writereg(GH3020_REG_OSC_CR, GH3X2X_LSI_CALI_CTRL_C_EN_VAL);
        usleep(GH3X2X_LSI_COR_CALI_DELAY_VAL);
        if (GH3X2X_CHECK_BIT_SET (gh3020_spi_readreg(GH3020_REG_OSC_FLAG), GH3X2X_OSC_CALI_CODE_LOACKED))
        {
            usCaliVal = gh3020_spi_readreg(GH3020_REG_OSC32K_TUNE);
            uchCalcVal = GH3X2X_GET_HIGH_BYTE_FROM_WORD(usCaliVal);
            for (uchIndex = 0; uchIndex < GH3X2X_LSI_CALI_FINE_TUNE_MAX; uchIndex++)
            {
                usCaliVal = GH3X2X_MAKEUP_WORD(uchCalcVal, GH3X2X_GET_LOW_BYTE_FROM_WORD(usCaliVal));
                gh3020_spi_writereg(GH3020_REG_OSC32K_TUNE, usCaliVal);
                gh3020_spi_writereg(GH3020_REG_OSC_CR, GH3X2X_LSI_CALI_CTRL_F_EN_VAL);
                usleep(GH3X2X_LSI_FINE_CALI_DELAY_VAL);
                sFreqErrVal = gh3020_spi_readreg(GH3020_REG_OSC_FREQ_ERR_UR);
                if (sFreqErrVal > GH3X2X_LSI_CALI_ERR_MAX_VAL)
                {
                    sFreqErrVal = (int16_t)(GH3X2X_LSI_CALI_ERR_FIXED_VAL - sFreqErrVal);
                    uchMinVal = uchCalcVal;
                }
                else
                {
                    uchMaxVal = uchCalcVal;
                }
                uchCalcVal = ((uint16_t)uchMaxVal + (uint16_t)uchMinVal) / GH3X2X_LSI_CALI_FINE_DIV_NUM;
                if (uchLastCalcVal == uchCalcVal)
                {
                    chRet = GH3X2X_RET_OK;
                    if (sLastFreqErrVal < sFreqErrVal)
                    {
                        usCaliVal = GH3X2X_MAKEUP_WORD(uchMaxVal, GH3X2X_GET_LOW_BYTE_FROM_WORD(usCaliVal));
                        gh3020_spi_writereg(GH3020_REG_OSC32K_TUNE, usCaliVal);
                    }
                    break;
                }
                uchLastCalcVal = uchCalcVal;
                if (sLastFreqErrVal > sFreqErrVal)
                {
                    sLastFreqErrVal = sFreqErrVal;
                }
            } // end of for (uchIndex = 0; uchIndex < GH3X2X_LSI_CALI_FINE_TUNE_MAX; uchIndex++)
        } // end of if (GH3X2X_CHECK_BIT_SET (gh3020_spi_readreg(GH3020_REG_OSC_FLAG), GH3X2X_OSC_CALI_CODE_LOACKED))
        GH3X2X_DEBUG_LOG_PARAM("Lsi: 0x%.4x\r\n", gh3020_spi_readreg(GH3020_REG_OSC32K_TUNE));
        gh3020_spi_writereg(GH3020_REG_OSC_CR, GH3X2X_OSC_CALI_CTRL_DIS_VAL);
        GH3X2X_WAIT_CHIP_DSLP();
    } // end of if (g_uchGh3x2xStatus < GH3X2X_STATUS_STARTED)
    return chRet;
}

/**
 * @fn     void GH3X2X_RegisterReadPinStatusFunc(uint8_t (*pReadIntPinStatusFunc)(void),
 *                                     uint8_t (*pReadResetPinStatusFunc)(void),
 *                                     uint8_t (*pReadSpcsPinStatusFunc)(void),
 *                                     uint8_t (*pReadSpdoPinStatusFunc)(void))
 *
 * @brief  Register read pin status function
 *
 * @attention   Only use for debug, read gh3x2x some pin status can debug some hardware errors;
 *              Pin status return should define as 1 [high level] or 0 [low level];
 *
 * @param[in]   pReadIntPinStatusFunc       pointer to read int pin staus function
 * @param[in]   pReadResetPinStatusFunc     pointer to read reset pin staus function
 * @param[in]   pReadSpcsPinStatusFunc      pointer to read spcs pin staus function
 * @param[in]   pReadSpdoPinStatusFunc      pointer to read spdo pin staus function
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_RegisterReadPinStatusFunc(uint8_t (*pReadIntPinStatusFunc)(void),
                                      uint8_t (*pReadResetPinStatusFunc)(void),
                                      uint8_t (*pReadSpcsPinStatusFunc)(void),
                                      uint8_t (*pReadSpdoPinStatusFunc)(void))
{
    g_pGh3x2xReadIntPinStatusFunc = pReadIntPinStatusFunc;
    g_pGh3x2xReadResetPinStatusFunc = pReadResetPinStatusFunc;
    g_pGh3x2xReadSpcsPinStatusFunc = pReadSpcsPinStatusFunc;
    g_pGh3x2xReadSpdoPinStatusFunc = pReadSpdoPinStatusFunc;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
}

/**
 * @fn     char *GH3X2X_GetDriverLibVersion(void)
 *
 * @brief  Get driver version
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  library version string
 */
char *GH3X2X_GetDriverLibVersion(void)
{
    return (char *)GH3X2X_VERSION_STRING;
}

/**
 * @fn     char *GH3X2X_GetDriverLibFuncSupport(void)
 *
 * @brief  Get driver function support
 *
 * @attention   just use to show function support
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  library version string
 */
char *GH3X2X_GetDriverLibFuncSupport(void)
{
    return (char *)GH3X2X_ALGO_FNC_SUPPORT_STRING;
}

/**
 * @fn     void GH3X2X_SetMaxNumWhenReadFifo(uint16_t usMaxNum)
 *
 * @brief  Set max number of rawdata read from fifo every time
 *
 * @attention   None.
 *
 * @param[in]   usMaxNum        max number of rawdata read from fifo
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_SetMaxNumWhenReadFifo(uint16_t usMaxNum)
{
    if (usMaxNum <= GH3X2X_FIFO_DATA_BYTES_MAX_LEN)
    {
        g_usMaxNumReadFromFifo = (usMaxNum/4) * 4;
    }
}


/**
 * @fn     uint16_t GH3X2X_GetCurrentFifoWaterLine(void)
 *
 * @brief  get current fifo water line setting
 *
 * @attention   None.
 *
 * @param[in]   None
 * @param[out]  fifo water line
 *
 * @return  None
 */
uint16_t GH3X2X_GetCurrentFifoWaterLine(void)
{
    return g_usCurrentFiFoWaterLine;
}



const uint8_t g_uchLedCurrentFullScal[4] = {25,50,100,200};




/**
 * @fn      uint8_t  Gh3x2x_GetLedCurrentFullScal(void)

 *
 * @brief
 *
 * @attention   None.
 *
 * @param[in]   None
 * @param[out]  Drv 0/1Current fs (mA)
 *
 * @return  None
 */

void  GH3x2x_GetLedCurrentFullScal(uint8_t *puchDrv0Fs, uint8_t *puchDrv1Fs)
{
    (*puchDrv0Fs)= gh3020_spi_readbits(GH3020_REG_LED_DRV_AD_REG,GH3020_DRV0_FULL_SCAL_CURRENT_LSB,GH3020_DRV0_FULL_SCAL_CURRENT_MSB);
    (*puchDrv1Fs)= gh3020_spi_readbits(GH3020_REG_LED_DRV_AD_REG,GH3020_DRV1_FULL_SCAL_CURRENT_LSB,GH3020_DRV1_FULL_SCAL_CURRENT_MSB);


    (*puchDrv0Fs) = g_uchLedCurrentFullScal[*puchDrv0Fs];
    (*puchDrv1Fs) = g_uchLedCurrentFullScal[*puchDrv1Fs];

}



void GH3x2x_DisableHardwareAgc(void)
{
    for(uint8_t uchSlotCnt = 0; uchSlotCnt < 8; uchSlotCnt ++)
    {
        gh3020_spi_writebits(GH3020_REG_SLOT0_CTRL_3 + GH3020_OFFSET_SLOT_CTRL*uchSlotCnt,GH3020_SLOT_AGC_EN_LSB,GH3020_SLOT_AGC_EN_MSB, 0);
    }
}


void GH3x2x_GetBgLevel(uint8_t *puchBglevel)
{
    for(uint8_t uchSlotCnt = 0; uchSlotCnt < 8; uchSlotCnt ++)
    {
        puchBglevel[uchSlotCnt] = gh3020_spi_readbits(GH3020_REG_SLOT0_CTRL_3 + GH3020_OFFSET_SLOT_CTRL*uchSlotCnt,GH3020_SLOT_BG_LEVEL_LSB,GH3020_SLOT_BG_LEVEL_MSB);
    }
}

void GH3x2x_GetBgCancel(uint8_t *puchBgCancel)
{
    for(uint8_t uchSlotCnt = 0; uchSlotCnt < 8; uchSlotCnt ++)
    {
        puchBgCancel[uchSlotCnt] = gh3020_spi_readbits(GH3020_REG_SLOT0_CTRL_3 + GH3020_OFFSET_SLOT_CTRL*uchSlotCnt,GH3020_SLOT_BG_CANCEL_LSB,GH3020_SLOT_BG_CANCEL_MSB);
    }
}



void GH3x2x_SetIntTime(uint8_t uchSlotNo, uint8_t uchIntTimeIndex)
{
    gh3020_spi_writebits(GH3020_REG_SLOT0_CTRL_3 + GH3020_OFFSET_SLOT_CTRL*uchSlotNo,GH3020_SLOT_ADC_INT_TIME_LSB,GH3020_SLOT_ADC_INT_TIME_MSB, uchIntTimeIndex);
}

void GH3x2x_SetSlotTime(uint8_t uchSlotNo, uint16_t usSlotTime)
{
    gh3020_spi_writereg(GH3020_REG_SLOT_TMR0 + 2*uchSlotNo, usSlotTime);
}


void GH3x2x_SetSlotSampleRate(uint8_t uchSlotNo, uint16_t usSampleRate)
{
    uint16_t usDivReg;
    if(usSampleRate > 1000)
    {
        usSampleRate = 1000;
    }
    if(usSampleRate < 5)
    {
        usSampleRate = 5;
    }

    usDivReg = (1000/usSampleRate) - 1;


    gh3020_spi_writebits(GH3020_REG_SLOT0_CTRL_0 + GH3020_OFFSET_SLOT_CTRL*uchSlotNo,GH3020_SLOT_SR_LSB,GH3020_SLOT_SR_MSB, usDivReg);


    GH3X2X_DEBUG_LOG_PARAM("GH3x2x_SetSlotSampleRate: uchSlotNo = %d, usSampleRate = %d, usDivReg = %d \r\n",(int)uchSlotNo,(int)usSampleRate,(int)usDivReg);
}





const uint16_t gusSlotTimeList[3][7] = {
{26,    36,    46,    56,    95,    174,    331},
{48,    67,    87,    107,    186,    343,    658},
{69,    99,    128,    158,    276,    512,    985}
};


uint16_t GH3x2x_CalSlotTime(uint8_t uchBgLevel, uint8_t uchBgCancel ,uint8_t uchIntTimeIndex)
{
    uint16_t usSlotTime;
    if(uchBgLevel > 2)
    {
        uchBgLevel = 2;
    }
    usSlotTime = gusSlotTimeList[uchBgLevel][uchIntTimeIndex];
    if(uchBgCancel > 0)
    {
        usSlotTime += 104;
    }

	usSlotTime += 80;  //fix pd remain problem

    GH3X2X_DEBUG_LOG_PARAM("GH3x2x_CalSlotTime: bg level = %d, bg cancel = %d, IntTimeIndex = %d, TimeSlot = %d \r\n",(int)uchBgLevel,(int)uchBgCancel,(int)uchIntTimeIndex,(int)usSlotTime);

    return usSlotTime;
}

int8_t g_chDrvECode = 0;

void GH3X2X_SetDrvEcode(int8_t chECode)
{
    g_chDrvECode = chECode;
}

uint16_t GH3X2X_SetLedDrvCurrent(float current_mA, uint8_t uchDrvNum, int8_t chECODE)
{
    float fEr;
    float fY;
    uint8_t  uchN;
    uint16_t usDrvCode;
    uint16_t usDrvScale;

    if(uchDrvNum == 0)
    {
        fY = 0.1f;
        usDrvScale =
          gh3020_spi_readbits(GH3020_REG_LED_DRV_AD_REG,
                              GH3020_DRV0_FULL_SCAL_CURRENT_LSB,
                              GH3020_DRV0_FULL_SCAL_CURRENT_MSB);
    }
    else
    {
        fY = 0.01f;
        usDrvScale =
          gh3020_spi_readbits(GH3020_REG_LED_DRV_AD_REG,
                              GH3020_DRV1_FULL_SCAL_CURRENT_LSB,
                              GH3020_DRV1_FULL_SCAL_CURRENT_MSB);
    }
    uchN = 1 << (3 - usDrvScale);

    fEr = 1.0f + (float)chECODE / 256.0f;
    usDrvCode = (uint16_t)(0.5f + (fEr* 255.0f *(float)uchN) / (225.0f / current_mA - fY));

    return usDrvCode;
}

/**
 * @fn      void GH3x2x_ChangeSampleParmForEngineeringMode(uint32_t unFunctionMode, struct gh3020_factestmode_param_s *pstSampleParaGroup , uint8_t uchSampleParaGroupNum)

 *
 * @brief
 *
 * @attention   None.
 *
 * @param[in]   unFunctionMode: function group exp: GH3X2X_FUNCTION_ADT|GH3X2X_FUNCTION_HR
 * @param[in]   pstSampleParaGroup   the pointer of engineering mode sample param group struct array base
 * @param[in]   uchSampleParaGroupNum   engineering mode sample param group number
 * @param[out]  None
 *
 * @return  None
 */
void GH3x2x_ChangeSampleParmForEngineeringMode(const struct gh3020_frameinfo_s * const  pstGh3x2xFrameInfo[], uint32_t unFunctionMode, FAR struct gh3020_factestmode_param_s *pstSampleParaGroup , uint8_t uchSampleParaGroupNum)
{
    uint32_t unFunctionID;
    uint8_t uchFunctionCnt;
    uint8_t *puchChnlMap;
    uint8_t uchChnlNum;
    uint8_t uchDr0Fs;
    uint8_t uchDr1Fs;
    uint8_t puchBglevel[8];    //slot0 ~ slot1 bg level
    uint8_t puchBgCancel[8];   //slot0 ~ slot1 bg cancel

    if(0 == pstSampleParaGroup) //is the point is invalid, do nothing
    {
        return ;
    }

    //read current full scale
    GH3x2x_GetLedCurrentFullScal(&uchDr0Fs, &uchDr1Fs);

    //read bg level
    GH3x2x_GetBgLevel(puchBglevel);

    //read bg cancel
    GH3x2x_GetBgCancel(puchBgCancel);

    //disable all hardware age slot
    GH3x2x_DisableHardwareAgc();



    //process every sample parm group
    for(uint8_t uchSampleParaGroupCnt = 0; uchSampleParaGroupCnt < uchSampleParaGroupNum; uchSampleParaGroupCnt ++)
    {
        FAR struct gh3020_factestmode_param_s *pstTempSampleParam = pstSampleParaGroup + uchSampleParaGroupCnt;
        unFunctionID = pstTempSampleParam->channelmode;
        if(0 == unFunctionID)
        {
            continue;
        }
        puchChnlMap = 0;
        uchChnlNum = 0;

        if(unFunctionID == (unFunctionMode&unFunctionID))   //unFunctionMode have cover unFunctionID
        {
            // get channl map and chnl num
            for (uchFunctionCnt = 0 ; uchFunctionCnt < GH3X2X_FUNC_OFFSET_MAX ; uchFunctionCnt ++)
            {
                if(pstGh3x2xFrameInfo[uchFunctionCnt])
                {
                    if(unFunctionID == (((uint32_t)1) << uchFunctionCnt))
                    {
                        break;
                    }
                }
            }
            if(GH3X2X_FUNC_OFFSET_MAX == uchFunctionCnt)
            {
                GH3X2X_DEBUG_LOG_PARAM("ChangeSampleParmForEngineeringMode: can not find valid pstGh3x2xFrameInfo !!! unFunctionID = %d\r\n", (int)unFunctionID);
                continue;
            }
            GH3X2X_DEBUG_LOG_PARAM("ChangeSampleParmForEngineeringMode: pstGh3x2xFrameInfo uchFunctionCnt = %d\r\n", (int)uchFunctionCnt);
            puchChnlMap = pstGh3x2xFrameInfo[uchFunctionCnt]->pchChnlMap;
            uchChnlNum = pstGh3x2xFrameInfo[uchFunctionCnt]->pstFunctionInfo->uchChnlNum;

            if((0 != puchChnlMap)&&(0 != uchChnlNum)) //channel map and channel num is all valid
            {

                GH3X2X_DEBUG_LOG_PARAM("ChangeSampleParmForEngineeringMode: ParaGroupCnt = %d, uchFunctionID = %d, uchChnlNum = %d\r\n", (int)uchSampleParaGroupCnt, (int)unFunctionID, (int)uchChnlNum);
                for(uint8_t uchChnlCnt = 0; uchChnlCnt < uchChnlNum; uchChnlCnt ++)
                {
                    uint8_t  uchTempSlotNo = GH3X2X_BYTE_RAWDATA_GET_SLOT_NUM(puchChnlMap[uchChnlCnt]);
                    uint8_t  uchTempAdtNo = GH3X2X_BYTE_RAWDATA_GET_ADC_NUM(puchChnlMap[uchChnlCnt]);
                    if(pstTempSampleParam->int_time_change_en)
                    {
                        uint16_t usTimeSlot;
                        GH3X2X_DEBUG_LOG_PARAM("ChangeSampleParm[int time]: need change int time. \r\n");
                        usTimeSlot = GH3x2x_CalSlotTime(puchBglevel[uchTempSlotNo],puchBgCancel[uchTempSlotNo], pstTempSampleParam->int_time);
                        GH3x2x_SetIntTime(uchTempSlotNo,pstTempSampleParam->int_time);
                        GH3x2x_SetSlotTime(uchTempSlotNo,usTimeSlot);
                    }
                    if(pstTempSampleParam->sample_rate_change_en)
                    {
                        GH3X2X_DEBUG_LOG_PARAM("ChangeSampleParm[sample rate]\r\n");

                        GH3x2x_SetSlotSampleRate(uchTempSlotNo,pstTempSampleParam->sample_rate);
                    }
                    if(pstTempSampleParam->tia_gain_change_en)
                    {
                        GH3X2X_SlotLedTiaGainConfig(uchTempSlotNo, uchTempAdtNo, pstTempSampleParam->tia_gain[uchChnlCnt]);

                        GH3X2X_DEBUG_LOG_PARAM("ChangeSampleParm[tia gain]: uchSlotNo = %d, uchTempAdtNo = %d, TiaGain = %d \r\n",(int)uchTempSlotNo,(int)uchTempAdtNo,(int)pstTempSampleParam->uchTiaGain[uchChnlCnt]);
                    }
                    if(pstTempSampleParam->led_current_change_en)
                    {

                        uint16_t usDrv0Reg;

                        uint16_t usDrv1Reg;

                        GH3X2X_DEBUG_LOG_PARAM("ChangeSampleParm[led current]\r\n");

                        usDrv0Reg =
                          GH3X2X_SetLedDrvCurrent(
                              (float)pstTempSampleParam->led_drv0_current[uchChnlCnt],
                              0, (int8_t)g_chDrvECode);
                        if(usDrv0Reg > 255)
                        {
                            usDrv0Reg = 255;
                        }
                        usDrv1Reg =
                          GH3X2X_SetLedDrvCurrent(
                              (float)pstTempSampleParam->led_drv1_current[uchChnlCnt],
                              1, (int8_t)g_chDrvECode);
                        if(usDrv1Reg > 255)
                        {
                            usDrv1Reg = 255;
                        }

                        GH3X2X_SlotLedCurrentConfig(uchTempSlotNo, 0, usDrv0Reg);
                        GH3X2X_SlotLedCurrentConfig(uchTempSlotNo, 1, usDrv1Reg);

                        GH3X2X_DEBUG_LOG_PARAM("ChangeSampleParmForEngineeringMode: uchSlotNo = %d, uchTempAdtNo = %d, Drv0 = %d mA, Drv1 = %d mA \r\n",(int)uchTempSlotNo,(int)uchTempAdtNo,(int)pstTempSampleParam->uchLedDrv0Current[uchChnlCnt],(int)pstTempSampleParam->uchLedDrv1Current[uchChnlCnt]);
                        GH3X2X_DEBUG_LOG_PARAM("ChangeSampleParmForEngineeringMode: Drv0_reg = %d, Drv1_reg = %d \r\n",(int)usDrv0Reg,(int)usDrv1Reg);
                    }
                }
            }
        }
    }
}

void GH3X2X_SetConfigDrvEcode(const FAR struct gh3020_initcfg_s *pstGh3x2xInitConfigParam)
{
    uint16_t usDrvScale;
    uint8_t uchN;
    float current;

    if (pstGh3x2xInitConfigParam == GH3X2X_PTR_NULL)
    {
        return;
    }

    for (uint16_t usArrCnt = 0; usArrCnt < pstGh3x2xInitConfigParam->arrlen; usArrCnt ++)
    {
        uint16_t usRegData = pstGh3x2xInitConfigParam->pregarr[usArrCnt].regval;
        uint16_t usRegAddr = pstGh3x2xInitConfigParam->pregarr[usArrCnt].regaddr;
        if ((pstGh3x2xInitConfigParam->pregarr[usArrCnt].regaddr & 0xF000) == 0)
        {
            for (int uchSlotIndex = 0; uchSlotIndex < 8; uchSlotIndex++)
            {
                if ((pstGh3x2xInitConfigParam->pregarr[usArrCnt].regaddr) == 0x011E + (uchSlotIndex * 0x001C) + (0 * 0x0002))
                {
                    usDrvScale =
                      gh3020_spi_readbits(GH3020_REG_LED_DRV_AD_REG,
                                          GH3020_DRV0_FULL_SCAL_CURRENT_LSB,
                                          GH3020_DRV0_FULL_SCAL_CURRENT_MSB);
                    uchN = 1 << (3 - usDrvScale);
                    current = 225.0f / (255.0f * uchN / (float)(usRegData&0xFF) + 0.1f);
                    uint16_t usDrv0Reg = GH3X2X_SetLedDrvCurrent(current, 0, g_chDrvECode);
                    GH3X2X_SlotLedCurrentConfig(uchSlotIndex, 0, usDrv0Reg);
                    if (usDrv0Reg != (GH3X2X_ReadReg(usRegAddr)&0xFF))
                    {
                        GH3X2X_SAMPLE_LOG_PARAM("[%s]:reg verify error!!!\r\n", __FUNCTION__);
                    }
                }
                else if ((pstGh3x2xInitConfigParam->pregarr[usArrCnt].regaddr) == 0x011E + (uchSlotIndex * 0x001C) + (1 * 0x0002))
                {
                    usDrvScale =
                      gh3020_spi_readbits(GH3020_REG_LED_DRV_AD_REG,
                                          GH3020_DRV1_FULL_SCAL_CURRENT_LSB,
                                          GH3020_DRV1_FULL_SCAL_CURRENT_MSB);
                    uchN = 1 << (3 - usDrvScale);
                    current = 225.0f / (255.0f * uchN / (float)(usRegData&0xFF) + 0.01f);
                    uint16_t usDrv1Reg = GH3X2X_SetLedDrvCurrent(current, 1, g_chDrvECode);
                    GH3X2X_SlotLedCurrentConfig(uchSlotIndex, 1, usDrv1Reg);
                    if (usDrv1Reg != (GH3X2X_ReadReg(usRegAddr)&0xFF))
                    {
                        GH3X2X_SAMPLE_LOG_PARAM("[%s]:reg verify error!!!\r\n", __FUNCTION__);
                    }
                }
            }
        }
    }
}

/**
 * @fn       uint8_t GH3x2x_GetActiveChipResetFlag(void)


 *
 * @brief
 *
 * @attention   None.
 *
 * @param[in]   None
 * @param[out]  ActiveChipResetFlag  1: user have done chip reset(soft reset and hardwear reset) actively    gh3x2x_init will clear it
 *
 * @return  None
 */
uint8_t GH3x2x_GetActiveChipResetFlag(void)
{
    return g_uchActiveChipResetFlag;
}


/**
 * @fn       uint8_t GH3x2x_GetChipResetRecoveringFlag(void)


 *
 * @brief
 *
 * @attention   None.
 *
 * @param[in]   None
 * @param[out]  chip reset recovering flag  0: is not in chip reset recovering flag  1: is in chip reset recovering
 *
 * @return  None
 */
uint8_t GH3x2x_GetChipResetRecoveringFlag(void)
{
    return g_uchChipResetRecoveringFlag;
}



/**
 * @fn       uint8_t GH3x2x_SetChipResetRecoveringFlag(void)


 *
 * @brief
 *
 * @attention   None.
 *
 * @param[in]   chip reset recovering flag  0: is not in chip reset recovering flag  1: is in chip reset recovering
 * @param[out]  None
 *
 * @return  None
 */
void GH3x2x_SetChipResetRecoveringFlag(uint8_t uchChipResetRecoeringFlag)
{
    g_uchChipResetRecoveringFlag = uchChipResetRecoeringFlag;
}




/**
 * @fn       void GH3x2x_GetNeedWakeUpGh3x2xFlag(void)
 *
 * @brief
 *
 * @attention   None.
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  NeedWakeUpGh3x2xFlag  0: do not need wake up gh3x2x  1: need wake up gh3x2x
 */
uint8_t GH3x2x_GetNeedWakeUpGh3x2xFlag(void)
{
    return g_uchNeedWakeUpGh3x2x;
}

/**
 * @fn       void GH3x2x_GetNeedWakeUpGh3x2xFlagBeforeInt(void)
 *
 * @brief
 *
 * @attention   None.
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  NeedWakeUpGh3x2xFlag  0: do not need wake up gh3x2x  1: need wake up gh3x2x
 */
uint8_t GH3x2x_GetNeedWakeUpGh3x2xFlagBeforeInt(void)
{
    return g_uchNeedWakeUpGh3x2xBeforeInt;
}

/**
 * @fn        void GH3x2x_SetNeedWakeUpGh3x2xFlag(uint8_t uchFlag)

 *
 * @brief
 *
 * @attention   None.
 *
 * @param[in]   set flag  0: do not need wake up gh3x2x  1: need wake up gh3x2x
 * @param[out]  None
 *
 * @return  None
 */
void GH3x2x_SetNeedWakeUpGh3x2xFlag(uint8_t uchFlag)
{
    g_uchNeedWakeUpGh3x2x = uchFlag;
}

/**
 * @fn        void GH3x2x_SetNeedWakeUpGh3x2xFlagBeforeInt(uint8_t uchFlag)

 *
 * @brief
 *
 * @attention   None.
 *
 * @param[in]   set flag  0: do not need wake up gh3x2x  1: need wake up gh3x2x
 * @param[out]  None
 *
 * @return  None
 */
void GH3x2x_SetNeedWakeUpGh3x2xFlagBeforeInt(uint8_t uchFlag)
{
    g_uchNeedWakeUpGh3x2xBeforeInt = uchFlag;
}



void GH3x2xCreatTagArray(uint8_t *puchTagArray, uint16_t usArrayLen,uint32_t *unCompeletMask, const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    uint8_t  uchChnlNum;
    uint8_t *puchChnlMap;

    uchChnlNum = pstFrameInfo->pstFunctionInfo->uchChnlNum;
    puchChnlMap = pstFrameInfo->pchChnlMap;

    //create tag array
    GH3X2X_Memset(puchTagArray,0xFF,usArrayLen);

    for(uint8_t uchChnlCnt = 0; uchChnlCnt < uchChnlNum; uchChnlCnt ++)
    {
        uint8_t uchTag;
        uchTag = puchChnlMap[uchChnlCnt] >> 3;
        puchTagArray[uchTag] = uchChnlCnt;
    }
    //crete compelet mask
    (*unCompeletMask) = ((((long long)1) << uchChnlNum) - 1)&pstFrameInfo->pstFunctionInfo->unChnlEnForUserSetting;
}






uint16_t GH3x2xGetFrameNum(uint8_t *puchRawdataBuf, uint16_t usRawDataByteLen, const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    uint32_t unTempIncompeletFlag;
    uint8_t  puchTagArray[32];
    uint32_t unCompeletMask;
    uint16_t usFrameNum = 0;


    unTempIncompeletFlag = *(pstFrameInfo->punIncompleteChnlMapBit);

    GH3x2xCreatTagArray(puchTagArray, 32,&unCompeletMask, pstFrameInfo);

    //set incomplete flag
    for (uint16_t usSearchStartPosi = 0; usSearchStartPosi < usRawDataByteLen; usSearchStartPosi += GH3X2X_FIFO_RAWDATA_SIZE)
    {
        uint8_t uchTag;
        uint8_t uchChnlId;
        uchTag = puchRawdataBuf[usSearchStartPosi] >> 3;
        uchChnlId =  puchTagArray[uchTag];
        if(0xFF != uchChnlId)
        {
            unTempIncompeletFlag |= (1 << uchChnlId);
        }

        if(unCompeletMask == (unCompeletMask&unTempIncompeletFlag))  //got one frame
        {
            unTempIncompeletFlag = 0;
            usFrameNum ++;
        }

    }

    return usFrameNum;

}


float GH3x2xCalGsensorStep(uint16_t usGsDataNum, uint16_t usFrameNum)
{
    float fGsensorStep = 0;

    if ((usFrameNum > 1) && (usGsDataNum > 0)) // calc gs index
    {
        fGsensorStep = ((float)(usGsDataNum - 1)) / (usFrameNum - 1);
    }

    return  fGsensorStep;


}

__weak void GH3x2xSendAllVersion(void)
{
    uint8_t uchPacketPayloadArr[240] = {0};
    uint8_t uchCmdArr[7 + GH3X2X_FUNC_OFFSET_MAX] = {(0x01),(0x0B),(0x0C),(0x0E),(0x0F),(0x10),(0x11)};
    uint8_t uchCmdLen = 7;

    if (g_uchAlgoEnableFlag)
    {
        for (int nCnt = 0 ; nCnt < GH3X2X_FUNC_OFFSET_MAX ; nCnt++)
        {
            uchCmdArr[7 + nCnt] = 0x12 + nCnt;
        }
        uchCmdLen = 7 + GH3X2X_FUNC_OFFSET_MAX;
    }

    for (uint8_t uchOffset = 0 ; uchOffset < uchCmdLen ; uchOffset ++)
    {
        uint8_t uchGetVersionType = UPROTOCOL_GET_VER_TYPE_ALGO_VER + uchOffset;
        char *pszVersionString = GH3X2X_GetVersion(uchCmdArr[uchOffset]);
        uint8_t uchVersionStringLen = (uint8_t)GH3X2X_Strlen(pszVersionString);
        uchPacketPayloadArr[UPROTOCOL_GET_VER_TYPE_INDEX + 4] = uchGetVersionType;
        uchPacketPayloadArr[UPROTOCOL_GET_VER_STRING_LEN_INDEX + 4] = uchVersionStringLen;
        GH3X2X_Memcpy((char *)&uchPacketPayloadArr[UPROTOCOL_GET_VER_STRING_INDEX + 4], pszVersionString, uchVersionStringLen);
    }
}


uint8_t Gh3x2xFunctionID2Offset(uint32_t unFunctionID)
{
    for(uint8_t uchOffset = 0; uchOffset < 32; uchOffset ++)
    {
        if(unFunctionID == (((uint32_t)1) << uchOffset))
        {
            return uchOffset;
        }
    }
    return 0;
}

void Gh3x2xFunctionInfoForUserInit(void)
{
    for(uint8_t uchFuncitonCnt = 0; uchFuncitonCnt < GH3X2X_FUNC_OFFSET_MAX; uchFuncitonCnt ++)
    {
        if(g_pstGh3x2xFrameInfo[uchFuncitonCnt])
        {
            g_pstGh3x2xFrameInfo[uchFuncitonCnt] -> pstFunctionInfo -> usSampleRateForUserSetting = 0;
            g_pstGh3x2xFrameInfo[uchFuncitonCnt] -> pstFunctionInfo -> unChnlEnForUserSetting = 0xFFFFFFFF;
        }
    }
}

void GH3x2xHandleFrameData(const struct gh3020_frameinfo_s * const pstFrameInfo, uint32_t *punTempIncompeletFlag,
                                uint32_t *punFunctionID, uint8_t uchChnlNum, uint8_t *puchRawdataBuf, uint32_t *punFrameRawdata,
                                uint8_t *puchTag, int16_t *pusGsValueArr, float fGsensorIndex,
                                STCapRawdata* pstCapValueArr,float fCapIndex,STTempRawdata* pstTempValueArr,float fTempIndex)

{
    uint32_t unTempIncompeletFlag = *punTempIncompeletFlag;
    uint8_t uchTag = *puchTag;
    uint32_t unFunctionID = *punFunctionID;
    uint8_t  uchGyroEnable = g_uchGyroEnable;

    GH3X2X_DEBUG_LOG_PARAM("[%s]got one frame,funcID = %d\r\n",__FUNCTION__,(int)unFunctionID);
    unTempIncompeletFlag = 0;
    GH3X2X_Memset((uint8_t*)(pstFrameInfo->punFrameFlag), 0, GH3X2X_FRAME_FLAG_NUM*4);       //clear flag
    GH3X2X_Memset((uint8_t*)(pstFrameInfo ->pstAlgoResult), 0, sizeof(STGh3x2xAlgoResult)); //clear result
    GH3X2X_Memset((uint8_t*)(pstFrameInfo ->punFrameAgcInfo), 0, uchChnlNum * 4);            //clear agc info

    for(uint8_t uchChnlCnt = 0; uchChnlCnt < uchChnlNum; uchChnlCnt ++)
    {
        uint32_t unRawdataAndTag;
        unRawdataAndTag = pstFrameInfo->punIncompleteRawdata[uchChnlCnt];
        uchTag = GH3X2X_DWORD_RAWDATA_GET_SLOT_ADC_NUM(unRawdataAndTag);

        //rawdata
        punFrameRawdata[uchChnlCnt] = unRawdataAndTag&0x00FFFFFF;

        //flag0 flag1
        pstFrameInfo->punFrameFlag[0] |= (((uint32_t)GH3X2X_RAWDATA_GET_ADJ_FLAG0(unRawdataAndTag) << uchChnlCnt));
        pstFrameInfo->punFrameFlag[1] |= (((uint32_t)GH3X2X_RAWDATA_GET_ADJ_FLAG1(unRawdataAndTag) << uchChnlCnt));

        //flag2 bit 0    fast recovery flag
        if ((GH3X2X_FUNCTION_ECG == unFunctionID)||(GH3X2X_FUNCTION_PWTT == unFunctionID))
        {
            if(0 == uchChnlCnt)
            {
                if(GH3X2X_RAWDATA_GET_FAST_RECOVERY_FLAG(unRawdataAndTag))
                {
                    pstFrameInfo->punFrameFlag[2] |= 0x01;
                }
            }
        }

        // agc (gain/drv0/drv1)
        pstFrameInfo->punFrameAgcInfo[uchChnlCnt] |= g_puchGainBgCancelRecord[uchTag] & 0xF;    //gain
        pstFrameInfo->punFrameAgcInfo[uchChnlCnt] |= ((uint32_t)g_pusDrvCurrentRecord[uchTag]) << 8; // drv 0/1 current
    }


    //g-sensor
    pstFrameInfo->pusFrameGsensordata[0] = pusGsValueArr[((uint16_t)fGsensorIndex)*(3+3*uchGyroEnable)];
    pstFrameInfo->pusFrameGsensordata[1] = pusGsValueArr[((uint16_t)fGsensorIndex)*(3+3*uchGyroEnable) + 1];
    pstFrameInfo->pusFrameGsensordata[2] = pusGsValueArr[((uint16_t)fGsensorIndex)*(3+3*uchGyroEnable) + 2];
    if(uchGyroEnable)
    {
        pstFrameInfo->pusFrameGsensordata[3] = pusGsValueArr[((uint16_t)fGsensorIndex)*(3+3*uchGyroEnable) + 3];
        pstFrameInfo->pusFrameGsensordata[4] = pusGsValueArr[((uint16_t)fGsensorIndex)*(3+3*uchGyroEnable) + 4];
        pstFrameInfo->pusFrameGsensordata[5] = pusGsValueArr[((uint16_t)fGsensorIndex)*(3+3*uchGyroEnable) + 5];
    }
    //cap and temp
    if(GH3X2X_GetCapEnableFlag())
    {
    	GH3X2X_Memcpy(pstFrameInfo->pstFrameCapdata,&pstCapValueArr[(uint16_t)fCapIndex],sizeof(STCapRawdata));
    }
    if(GH3X2X_GetTempEnableFlag())
    {
    	GH3X2X_Memcpy(pstFrameInfo->pstFrameTempdata,&pstTempValueArr[(uint16_t)fTempIndex],sizeof(STTempRawdata));
    }

    //flag 2 bit1  first frame flag

    Gh3x2xSetFrameFlag2(pstFrameInfo);
    /*
    pstFrameInfo->punFrameFlag[2] &= (~(((uint32_t)1) << 1));
    if(0 == (*(pstFrameInfo->punFrameCnt)))
    {
        pstFrameInfo->punFrameFlag[2] |= 0x02;
    }
    */
    if(GH3x2xSleepFlagGet())
    {
        pstFrameInfo->punFrameFlag[2] |= ((uint32_t)1)<<3;
    }
    if(GH3X2X_FUNCTION_SOFT_ADT_IR == unFunctionID)
    {
        pstFrameInfo->punFrameFlag[2] |= ((uint32_t)1)<<4;
    }

    GH3X2X_DEBUG_LOG_PARAM_WANPENG("[GetFrameData] function id: 0x%X,  rawdata0 = %d, rawdata1 = %d\r\n",(int)pstFrameInfo->unFunctionID, (int)(pstFrameInfo->punFrameRawdata[0]), (int)(pstFrameInfo->punFrameRawdata[1]));
    GH3X2X_DEBUG_LOG_PARAM_WANPENG("[GetFrameData] TimeStamp = %d\r\n",(int)(*(pstFrameInfo->punFrameCnt)));

    //call algo 2-level interface
    unFunctionID = pstFrameInfo->unFunctionID;
    pstFrameInfo->pstAlgoResult->uchUpdateFlag = 0;
		GH3X2X_RawdataCode(pstFrameInfo->punFrameRawdata, uchChnlNum);

    //get algo io data hook
    gh3020_get_ppg_data(pstFrameInfo);

    SET_VAL_VIA_PTR(punFunctionID, unFunctionID);
    SET_VAL_VIA_PTR(puchTag, uchTag);
    SET_VAL_VIA_PTR(punTempIncompeletFlag, unTempIncompeletFlag);
}

void GH3x2xGetFrameDataAndProcess(uint8_t *puchRawdataBuf, uint16_t usRawDataByteLen,
                                        int16_t *pusGsValueArr, uint16_t usGsDataNum,STCapRawdata* pstCapValueArr,uint16_t usCapDataNum,STTempRawdata* pstTempValueArr,uint16_t usTempDataNum,
                                        const struct gh3020_frameinfo_s * const pstFrameInfo, float fGensorStep, float fCapStep, float fTempStep, uint16_t usFrameNum)

{
    uint32_t unTempIncompeletFlag;
    uint8_t  puchTagArray[32];
    uint8_t  uchChnlNum;
    uint32_t unCompeletMask;
    uint32_t *punFrameRawdata;
    float fGsensorIndex = 0;
    float fCapIndex = 0;
    float fTempIndex = 0;
    uint32_t unFunctionID;
    uint16_t usFrameCnt = 0;

    unTempIncompeletFlag = *(pstFrameInfo->punIncompleteChnlMapBit);
    uchChnlNum = pstFrameInfo->pstFunctionInfo->uchChnlNum;
    punFrameRawdata = pstFrameInfo->punFrameRawdata;
    unFunctionID = pstFrameInfo->unFunctionID;

    GH3x2xCreatTagArray(puchTagArray, 32, &unCompeletMask, pstFrameInfo);

    //set incomplete flag
    for (uint16_t usSearchStartPosi = 0; usSearchStartPosi < usRawDataByteLen; usSearchStartPosi += GH3X2X_FIFO_RAWDATA_SIZE)
    {
        uint8_t uchTag;
        uint8_t uchChnlId;
        uchTag = puchRawdataBuf[usSearchStartPosi] >> 3;
        uchChnlId =  puchTagArray[uchTag];
        if(0xFF != uchChnlId)
        {
            uint32_t unRawdataAndTag;
            unTempIncompeletFlag |= (1 << uchChnlId);
            unRawdataAndTag = GH3X2X_MAKEUP_DWORD(puchRawdataBuf[usSearchStartPosi],
                                                                    puchRawdataBuf[usSearchStartPosi + 1],
                                                                    puchRawdataBuf[usSearchStartPosi + 2],
                                                                    puchRawdataBuf[usSearchStartPosi + 3]);
            //store incomplete rawdata
            pstFrameInfo->punIncompleteRawdata[uchChnlId] = unRawdataAndTag;

        }

        if(unCompeletMask == (unCompeletMask&unTempIncompeletFlag))  //got one frame
        {
            if((0 == pstFrameInfo->pstDownSampleInfo->uchDownSampleFactor)||(pstFrameInfo->pstDownSampleInfo->uchDownSampleFactor == pstFrameInfo->pstDownSampleInfo->uchDownSampleCnt))
            {
                GH3x2xHandleFrameData(pstFrameInfo, &unTempIncompeletFlag,
                                    &unFunctionID, uchChnlNum, puchRawdataBuf, punFrameRawdata,
                                    &uchTag, pusGsValueArr, fGsensorIndex,
                                    pstCapValueArr,fCapIndex,pstTempValueArr,fTempIndex);

                // timestamp
                (*(pstFrameInfo->punFrameCnt)) ++;
                // bak last gain
                for(uint8_t uchChnlCnt = 0; uchChnlCnt < uchChnlNum; uchChnlCnt ++)
                {
                    pstFrameInfo ->puchFrameLastGain[uchChnlCnt] = ((pstFrameInfo->punFrameAgcInfo[uchChnlCnt]) & 0x0000000F);
                }


                usFrameCnt ++;
            }
            unTempIncompeletFlag = 0;
            if(pstFrameInfo->pstDownSampleInfo->uchDownSampleFactor)
            {
                if(pstFrameInfo->pstDownSampleInfo->uchDownSampleFactor <= pstFrameInfo->pstDownSampleInfo->uchDownSampleCnt)
                {
                    pstFrameInfo->pstDownSampleInfo->uchDownSampleCnt = 0;
                }
                else
                {
                    pstFrameInfo->pstDownSampleInfo->uchDownSampleCnt ++;
                }
            }

            //calculate next fGsensorIndex
            fGsensorIndex += fGensorStep;
            fCapIndex  += fCapStep;
            fTempIndex  += fTempStep;

        }

    }

    //save IncompeletFlag
    *(pstFrameInfo->punIncompleteChnlMapBit) = unTempIncompeletFlag;
}

void GH3x2xFunctionProcess(uint8_t *puchRawdataBuf, uint16_t usRawDataByteLen, int16_t *pusGsValueArr, uint16_t usGsDataNum,
                        STCapRawdata* pstCapValueArr,uint16_t usCapDataNum,STTempRawdata* pstTempValueArr,uint16_t usTempDataNum,
                        const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    uint16_t usFrameNum;
    float fGsensorStep;
    float fCapStep;
    float fTempStep;

    //SlaverRttLog("[%s]:send to alg, packid = %d\r\n", __FUNCTION__, g_uchFifoPackageID);
    if(0 == pstFrameInfo)
    {
        return ;
    }

    /*****  check feature value  *********/
    if(0 == GH3X2X_CheckRawdataBuf(puchRawdataBuf, usRawDataByteLen) || pstFrameInfo->pstFunctionInfo->uchChnlNum == 0)
    {
        return;
    }

    /***  cal how many frames in this fifo  ***/
    usFrameNum = GH3x2xGetFrameNum(puchRawdataBuf, usRawDataByteLen, pstFrameInfo);

    /***  cal g-sensor step ***/
    fGsensorStep = GH3x2xCalGsensorStep(usGsDataNum, usFrameNum);
    fCapStep     = GH3x2xCalGsensorStep(usCapDataNum, usFrameNum);
    fTempStep    = GH3x2xCalGsensorStep(usTempDataNum, usFrameNum);

    GH3X2X_DEBUG_LOG_PARAM_WANPENG("[GetFrameNum]FunctionID: %d FrameNum: %d, GsensorStep: %.3f\r\n",(int)pstFrameInfo->unFunctionID, usFrameNum, fGsensorStep);

    /***  get one frame data and process***/
    GH3x2xGetFrameDataAndProcess(puchRawdataBuf,usRawDataByteLen,pusGsValueArr,usGsDataNum,pstCapValueArr,usCapDataNum,pstTempValueArr,usTempDataNum,
                                            pstFrameInfo,fGsensorStep,fCapStep,fTempStep,usFrameNum);


}

extern uint32_t g_unFuncStartedBitmap;

void GH3X2X_ZipmodeInit(void)
{
    g_uchOddEvenChangeFlag = 1;
}

int8_t GH3X2X_FunctionStart(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    int8_t chRet = GH3X2X_RET_OK;
    uint32_t unFunctionID;
    if(0 == pstFrameInfo)
    {
        return chRet;
    }

    GH3X2X_ZipmodeInit();

    unFunctionID = pstFrameInfo->unFunctionID;

    GH3x2xCalFunctionSlotBit(pstFrameInfo);



    /* if all func off, start sampling */
    if (GH3X2X_NO_FUNCTION == g_unFuncStartedBitmap)
    {
        chRet = GH3X2X_StartSampling();
        GH3X2X_DEBUG_LOG_PARAM("[FunctionStart] function id: 0x%X, start result: %d\r\n",(int)pstFrameInfo->unFunctionID, chRet);
        GH3X2X_RET_ERROR_CHECK_E(chRet, GH3X2X_RET_NO_INITED_ERROR);
    }


#if GH3X2X_ALGORITHM_ECG_SUPPORT
    if (GH3X2X_FUNCTION_ECG == unFunctionID)
    {
        GH3X2X_EcgSampleHookHandle(ECG_SAMPLE_EVENT_TYPE_SAMPLE, ECG_SAMPLE_EVENT_INFO_SAMPLE_START);  //G202008231001 wanpeng
        GH3X2X_LeadOffDetect2Init();
        GH3X2X_ECGResampleConfig(g_pstGh3x2xFrameInfo[GH3X2X_FUNC_OFFSET_ECG]->pstFunctionInfo->usSampleRate);
    }
#endif

    if (GH3X2X_FUNCTION_ADT == unFunctionID)
    {
        GH3X2X_LeadDetEnInHardAdt(1);
    }

    /* set started bit */
    g_unFuncStartedBitmap |= unFunctionID;

    /* clear incompelet flag */
    (*(pstFrameInfo->punIncompleteChnlMapBit)) = 0;

    /* clear timestamp */
    (*(pstFrameInfo->punFrameCnt)) = 0;

    /* set last gain */
    GH3X2X_Memset(pstFrameInfo->puchFrameLastGain, 0xFF, pstFrameInfo->pstFunctionInfo->uchChnlNum);

    return chRet;
}

int8_t GH3X2X_FunctionStop(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    int8_t chRet = GH3X2X_RET_OK;
    uint32_t unFunctionID;
    if(0 == pstFrameInfo)
    {
        return chRet;
    }

    unFunctionID = pstFrameInfo->unFunctionID;
    GH3x2xCalFunctionSlotBit(pstFrameInfo);

    /* clear started bit */
    g_unFuncStartedBitmap &= (~unFunctionID);

    if (GH3X2X_FUNCTION_ADT == unFunctionID)
    {
        GH3X2X_LeadDetEnInHardAdt(0);
    }

    /* if all func off, stop sampling */
    if (GH3X2X_NO_FUNCTION == g_unFuncStartedBitmap)
    {
        chRet = GH3X2X_StopSampling();
        GH3X2X_DEBUG_LOG_PARAM("[FunctionStop] function id: 0x%X, start result: %d\r\n",(int)pstFrameInfo->unFunctionID, chRet);
        GH3X2X_RET_ERROR_CHECK(chRet);

        /* fixed fifo watermark thr */
        //GH3X2X_FIXED_FIFO_WATERMARK_THR();
    }

    return chRet;
}


uint8_t* GH3x2xGetFunctionChnlMap(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    if(0 == pstFrameInfo)
    {
        return 0;
    }
    else
    {
        return pstFrameInfo->pchChnlMap;
    }
}

void GH3x2xSetFunctionChnlNum(const struct gh3020_frameinfo_s * const pstFrameInfo, uint8_t uchChnlNum)
{
    uint8_t uchChnlLimit;
    if(pstFrameInfo)
    {
        uchChnlLimit = pstFrameInfo->uchFuntionChnlLimit;
        if(uchChnlNum > uchChnlLimit)
        {
            uchChnlNum = uchChnlLimit;
        }
        pstFrameInfo ->pstFunctionInfo->uchChnlNum = uchChnlNum;


        GH3X2X_DEBUG_LOG_PARAM("[SetFunctionChnlNum] function id: 0x%X, ChnlNum: %d\r\n",(int)pstFrameInfo->unFunctionID, uchChnlNum);
    }
}

void GH3x2xSetFunctionChnlMap(const struct gh3020_frameinfo_s * const pstFrameInfo, uint8_t uchChnlId, uint8_t uchChnlTag)
{
    if(pstFrameInfo)
    {
        if(uchChnlId < (pstFrameInfo->uchFuntionChnlLimit))
        {
            pstFrameInfo ->pchChnlMap[uchChnlId] = uchChnlTag;


            GH3X2X_DEBUG_LOG_PARAM("[SetFunctionChnlMap] function id: 0x%X, SlotAdc: %d\r\n",(int)pstFrameInfo->unFunctionID, uchChnlTag>>3);
        }

    }
}


void GH3x2xCalFunctionSlotBit(const struct gh3020_frameinfo_s * const pstFrameInfo)
{
    uint8_t uchSlotBit = 0;
    uint8_t uchChnlTag;
    if(pstFrameInfo)
    {
        for(uint8_t uchChnlCnt = 0; uchChnlCnt < pstFrameInfo->pstFunctionInfo->uchChnlNum; uchChnlCnt ++)
        {
			if(pstFrameInfo->pstFunctionInfo->unChnlEnForUserSetting & (((uint32_t)1) << uchChnlCnt))
			{
				uchChnlTag = pstFrameInfo->pchChnlMap[uchChnlCnt];
	            uchSlotBit |= (1 << (GH3X2X_BYTE_RAWDATA_GET_SLOT_NUM(uchChnlTag)));
			}
        }
    }




    pstFrameInfo ->pstFunctionInfo->uchSlotBit = uchSlotBit;


    GH3X2X_DEBUG_LOG_PARAM("[GH3x2xCalFunctionSlotBit] function id: 0x%X, slot bit: 0x%X\r\n",(int)pstFrameInfo->unFunctionID, pstFrameInfo->pstFunctionInfo->uchSlotBit);
}





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

void GH3X2X_AdtPramInit(void)
{
    GH3X2X_Memset(((uint8_t *)(&g_stAdtModuleCfg)),0, sizeof(g_stAdtModuleCfg));
    g_uchElectrodeWearRevCnt = 0;
}



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


__weak void GH3X2X_AdtPramSet(uint16_t usRegVal, uint8_t uchRegPosi)
{

    uint8_t uchByteOffset;
    uint8_t uchByteVal;
    uint8_t uchByteCnt;
    for(uchByteCnt = 0; uchByteCnt < 2; uchByteCnt ++)
    {
        uchByteVal = ((usRegVal >> (8*uchByteCnt))&0x00FF);
        uchByteOffset = (uchRegPosi * 2) + uchByteCnt;
        if(uchByteOffset < sizeof(g_stAdtModuleCfg))
        {
            GH3X2X_Memcpy(((uint8_t *)(&g_stAdtModuleCfg)) + uchByteOffset, (uint8_t*)(&uchByteVal), 1);

            if ((uchByteOffset) == (((uint8_t*)(&(g_stAdtModuleCfg.stAdtCfgByte0))) -((uint8_t*)(&(g_stAdtModuleCfg)))))
            {
                if(1 == g_stAdtModuleCfg.stAdtCfgByte0.uchElectrodeWearRevCntEn)
                {
                    GH3X2X_WAIT_CHIP_WAKEUP();
                    WEAR_ON_EVENT_EN_CONFIG(1);
                    WEAR_OFF_EVENT_EN_CONFIG(1);
                    GH3X2X_WAIT_CHIP_DSLP();
                }

                if(0 == g_stAdtModuleCfg.stAdtCfgByte0.uchElectrodeSoftWearColor)   //grren
                {
                    Gh3x2xSetGh3x2xSoftWearOffDetFunctionId(GH3X2X_FUNCTION_SOFT_ADT_GREEN);
                }
                else if(1 == g_stAdtModuleCfg.stAdtCfgByte0.uchElectrodeSoftWearColor)   //ir
                {
                    Gh3x2xSetGh3x2xSoftWearOffDetFunctionId(GH3X2X_FUNCTION_SOFT_ADT_IR);
                }
            }
        }
    }
}

__weak void Gh3x2xSetGh3x2xSoftWearOffDetFunctionId(uint32_t unFunctionId)
{}

uint8_t GH3X2X_GetAdtElectrodeAdtEn(void)
{
    return g_stAdtModuleCfg.stAdtCfgByte0.uchElectrodeAdtEn;
}

__weak uint8_t GH3X2X_GetSoftLeadDetMode(void)
{
    return 0;
}

/**
 * @fn     void GH3X2X_AddElectrodeWearRevCnt(void)
 *
 * @brief  add count of electrode revert
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_AddElectrodeWearRevCnt(void)
{
    if(g_uchElectrodeWearRevCnt < 255)
    {
        g_uchElectrodeWearRevCnt ++;
    }
}


/**
 * @fn     uint8_t GH3X2X_GetAndClearElectrodeWearRevCnt(void)
 *
 * @brief  get Electrode wear count,and reset the count
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  ElectrodeWearRevCnt
 */
uint8_t GH3X2X_GetAndClearElectrodeWearRevCnt(void)
{
    uint8_t uchCurrentElectrodeWearRevCnt;
    uchCurrentElectrodeWearRevCnt = g_uchElectrodeWearRevCnt;
    g_uchElectrodeWearRevCnt = 0;
    return uchCurrentElectrodeWearRevCnt;
}

/**
 * @fn     uint8_t GH3X2X_ElectrodeWearRevertDebugModeIsEnabled(void)
 *
 * @brief  Check if electrode wear revert debug mode is enabled
 *
 * @attention   In this mode,not need report wear event,need report the number of wear electrode revert.
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  1: enabled, 0:disabled
 */
uint8_t GH3X2X_ElectrodeWearRevertDebugModeIsEnabled(void)
{
    return g_stAdtModuleCfg.stAdtCfgByte0.uchElectrodeWearRevCntEn;
}



//G202008031001 wanpeng START
/**
 * @fn     uint8_t GH3X2X_GetSoftEvent(void)
 * @brief  Get soft event
 * @param[in]   None
 * @param[out]  Soft event
 *
 * @return  soft event
 */
uint8_t GH3X2X_GetSoftEvent(void)
{
    return gubSoftEvent;
}

/**
 * @fn     void GH3X2X_SetSoftEvent(uint8_t uchEvent)
 * @brief  set soft event
 * @param[in]   uchEvent
 * @param[out]  None
 * @return  None
 */
void GH3X2X_SetSoftEvent(uint8_t uchEvent)
{
    gubSoftEvent |= uchEvent;
}
/**
 * @fn     void GH3X2X_ClearSoftEvent(uint8_t uchEvent)
 * @brief  clear soft event
 * @param[in]   uchEvent
 * @param[out]  None
 * @return  None
 */

void GH3X2X_ClearSoftEvent(uint8_t uchEvent)
{
    gubSoftEvent &= (~uchEvent);
}
//G202008031001 wanpeng END


/**
 * @fn     uint8_t GH3X2X_GetGsensorEnableFlag(void)
 *
 * @brief  Inquire if need gsensor data
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  1: need gsensor data  0: not need gsensor data
 */
__weak uint8_t GH3X2X_GetGsensorEnableFlag(void)
{
    return g_uchGsensorEnable;
}

__weak uint8_t GH3X2X_GetCapEnableFlag(void)
{
    return g_uchCapEnable;
}

__weak uint8_t GH3X2X_GetTempEnableFlag(void)
{
    return g_uchTempEnable;
}


void Gh3x2xFunctionInit(void)
{
    for(uint8_t uchFuncitonCnt = 0; uchFuncitonCnt < GH3X2X_FUNC_OFFSET_MAX; uchFuncitonCnt ++)
    {
        if(g_pstGh3x2xFrameInfo[uchFuncitonCnt])
        {
            GH3x2xSetFunctionChnlNum(g_pstGh3x2xFrameInfo[uchFuncitonCnt], 0);   //chnl num
            g_pstGh3x2xFrameInfo[uchFuncitonCnt] -> pstFunctionInfo -> usSampleRate = 0; //set sample rate to invalid value
            g_pstGh3x2xFrameInfo[uchFuncitonCnt] -> pstDownSampleInfo -> uchDownSampleFactor = 0;
            g_pstGh3x2xFrameInfo[uchFuncitonCnt] -> pstDownSampleInfo -> uchDownSampleCnt = 0;
        }
    }
}

/**
 * @fn     void GH3X2X_ReinitAllSwModuleParam(uint16_t usReinitFlag)
 *
 * @brief  reinit all software param config
 *
 * @attention   None
 *
 * @param[in]   usReinitFlag        reinit flag, #0 don't reinit, #others reinit all
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_ReinitAllSwModuleParam(uint16_t usReinitFlag)
{
    if (usReinitFlag != 0) // if != 0, that need reinit all software module param
    {

        GH3x2xSetFunctionChnlNum(g_pstGh3x2xFrameInfo[GH3X2X_FUNC_OFFSET_HR], 0);
        GH3x2xSetFunctionChnlNum(g_pstGh3x2xFrameInfo[GH3X2X_FUNC_OFFSET_HRV], 0);
        GH3x2xSetFunctionChnlNum(g_pstGh3x2xFrameInfo[GH3X2X_FUNC_OFFSET_SPO2], 0);
        GH3x2xSetFunctionChnlNum(g_pstGh3x2xFrameInfo[GH3X2X_FUNC_OFFSET_ECG], 0);
        GH3x2xSetFunctionChnlNum(g_pstGh3x2xFrameInfo[GH3X2X_FUNC_OFFSET_SOFT_ADT_GREEN], 0);
        GH3x2xSetFunctionChnlNum(g_pstGh3x2xFrameInfo[GH3X2X_FUNC_OFFSET_SOFT_ADT_IR], 0);
        GH3x2xSetFunctionChnlNum(g_pstGh3x2xFrameInfo[GH3X2X_FUNC_OFFSET_ADT], 0);


        GH3X2X_LedAgcReset();            // agc moudle param reinit
        #if GH3X2X_ALGORITHM_ECG_SUPPORT
        GH3X2X_SlaverSoftLeadPramInit();// lead moudle param reinit
        #endif

        GH3X2X_DumpModeSet(0);          // dump module param reinit

        GH3X2X_AdtPramInit();   ////G202008231001 wanpeng
        g_uchGsensorEnable = 0;
        // soft adt param reinit
    }
}


/* util function, memcpy & memset & strlen */

/**
 * @fn     void *GH3X2X_Memcpy(void *pDest, const void *pSrc, uint32_t unByteSize)
 *
 * @brief  memcpy() Re-implementation
 *
 * @attention   None
 *
 * @param[in]   pSrc        pointer to source buffer
 * @param[in]   unByteSize  source buffer byte size
 * @param[out]  pDest       pointer to destination buffer
 *
 * @return  pointer to destination buffer
 */
void *GH3X2X_Memcpy(void *pDest, const void *pSrc, uint32_t unByteSize)
{
    uint8_t *puchSrc = (uint8_t *)pSrc;
    uint8_t *puchDest = (uint8_t *)pDest;
    uint32_t unAlign = ((uint32_t)puchDest | (uint32_t)puchSrc) << GH3X2X_UPROTOCOL_ALIGN_LEFTSHIFT;

    if ((pDest == GH3X2X_PTR_NULL) || (pSrc == GH3X2X_PTR_NULL))
    {
        return GH3X2X_PTR_NULL;
    }
    if (unAlign == 0) // if align 4
    {
        while (unByteSize >= GH3X2X_UPROTOCOL_SIZE_T)
        {
            *(uint32_t *)puchDest = *(uint32_t *)puchSrc;
            puchSrc += GH3X2X_UPROTOCOL_SIZE_T;
            puchDest += GH3X2X_UPROTOCOL_SIZE_T;
            unByteSize -= GH3X2X_UPROTOCOL_SIZE_T;
        }
    }
    while (unByteSize) // bytes
    {
        *(uint8_t *)puchDest = *(uint8_t *)puchSrc;
        puchSrc++;
        puchDest++;
        unByteSize--;
    }
    return pDest;
}

/**
 * @fn     void *GH3X2X_Memset(void* pDest, char chVal, uint32_t unByteSize)
 *
 * @brief  memset() Re-implementation
 *
 * @attention   None
 *
 * @param[in]   chVal       char val for set
 * @param[in]   unByteSize       source buffer len
 * @param[out]  pDest       pointer to destination buffer
 *
 * @return  pointer to destination buffer
 */
void *GH3X2X_Memset(void* pDest, char chVal, uint32_t unByteSize)
{
    uint8_t *puchDest = (uint8_t *)pDest;
    uint32_t unAlign = ((uint32_t)puchDest) << GH3X2X_UPROTOCOL_ALIGN_LEFTSHIFT;
    uint32_t unWordVal = GH3X2X_MAKEUP_DWORD(chVal, chVal, chVal, chVal);

    if (pDest == GH3X2X_PTR_NULL)
    {
        return GH3X2X_PTR_NULL;
    }
    if (unAlign == 0) // if align 4
    {
        while (unByteSize >= GH3X2X_UPROTOCOL_SIZE_T)
        {
            *(uint32_t *)puchDest = unWordVal;
            puchDest += GH3X2X_UPROTOCOL_SIZE_T;
            unByteSize -= GH3X2X_UPROTOCOL_SIZE_T;
        }
    }
    while (unByteSize) // bytes
    {
        *(uint8_t *)puchDest = (uint8_t)chVal;
        puchDest++;
        unByteSize--;
    }
    return pDest;
}

/**
 * @fn     uint32_t GH3X2X_Strlen(const char *pszSrc)
 *
 * @brief  strlen() Re-implementation
 *
 * @attention   None
 *
 * @param[in]   pszSrc      pointer to string
 * @param[out]  None
 *
 * @return  string len
 */
uint32_t GH3X2X_Strlen(const char *pszSrc)
{
    uint32_t unCnt = 0;
    if (pszSrc != GH3X2X_PTR_NULL)
    {
        while ((*pszSrc) != 0) // if not equal '\0'
        {
            unCnt++;
            pszSrc++;
        }
    }
    return unCnt;
}

/**
 * @fn     uint8_t GH3X2X_GetSoftWearColor(void)
 *
 * @brief  GetSoftWearColor
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  SoftWearColor 0: green  1:ir
 */

uint8_t GH3X2X_GetSoftWearColor(void)
{
    return g_stAdtModuleCfg.stAdtCfgByte0.uchElectrodeSoftWearColor;
}

void GH3X2X_SetAlgoEnableFlag(uint8_t uchAlgoEnableFlag)
{
    g_uchAlgoEnableFlag = uchAlgoEnableFlag;
}

/**
 * @fn     void GH3X2X_ModifyFunctionFrequency(uint8_t uchFunctionID, uint16_t usFrequencyValue)
 *
 * @brief  Modify fs for each function
 *
 * @attention   None
 *
 * @param[in]   uchFunctionID               function offset
 * @param[in]   usFrequencyValue         5Hz-1000Hz
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_ModifyFunctionFrequency(uint8_t uchFunctionID, uint16_t usFrequencyValue)
{
    uint8_t uchTempSlotNo = 0;
    uint8_t uchTempSlotLastNo = 0xff;
    uint8_t uchChnlCnt = 0;
    for(uchChnlCnt = 0; uchChnlCnt < g_pstGh3x2xFrameInfo[uchFunctionID]->pstFunctionInfo->uchChnlNum; uchChnlCnt ++)
    {
        uchTempSlotNo = GH3X2X_BYTE_RAWDATA_GET_SLOT_NUM(g_pstGh3x2xFrameInfo[uchFunctionID]->pchChnlMap[uchChnlCnt]);
        if (uchTempSlotLastNo != uchTempSlotNo)
        {
            GH3x2x_SetSlotSampleRate(uchTempSlotNo, usFrequencyValue);
            uchTempSlotLastNo = uchTempSlotNo;
        }
    }
    g_pstGh3x2xFrameInfo[uchFunctionID]->pstFunctionInfo->usSampleRate = usFrequencyValue;
}

/**
 * @fn     void GH3X2X_ModifyFunctionLedCurrent(uint8_t uchFunctionID, uint16_t usLedDrv0Current, uint16_t usLedDrv1Current)
 *
 * @brief  Modify led current for each function
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
void GH3X2X_ModifyFunctionLedCurrent(uint8_t uchFunctionID, uint16_t usLedDrv0Current, uint16_t usLedDrv1Current)
{
    uint8_t uchTempSlotNo = 0;
    uint8_t uchTempSlotLastNo = 0xff;
    uint8_t uchChnlCnt = 0;
    uint16_t usDrv0Reg = 0;
    uint16_t usDrv1Reg = 0;
    uint16_t usDr0Fs = 0;
    uint16_t usDr1Fs = 0;

    GH3x2x_GetLedCurrentFullScal((uint8_t*)&usDr0Fs, (uint8_t*)&usDr1Fs);

    for(uchChnlCnt = 0; uchChnlCnt < g_pstGh3x2xFrameInfo[uchFunctionID]->pstFunctionInfo->uchChnlNum; uchChnlCnt ++)
    {
        uchTempSlotNo = GH3X2X_BYTE_RAWDATA_GET_SLOT_NUM(g_pstGh3x2xFrameInfo[uchFunctionID]->pchChnlMap[uchChnlCnt]);
        if (uchTempSlotLastNo != uchTempSlotNo)
        {
            usDrv0Reg = (uint16_t)usLedDrv0Current * 255 /usDr0Fs;
            if(usDrv0Reg > 255)
            {
                usDrv0Reg = 255;
            }
            usDrv1Reg = (uint16_t)usLedDrv1Current * 255 /usDr1Fs;
            if(usDrv1Reg > 255)
            {
                usDrv1Reg = 255;
            }

            GH3X2X_SlotLedCurrentConfig(uchTempSlotNo, 0, usDrv0Reg);
            GH3X2X_SlotLedCurrentConfig(uchTempSlotNo, 1, usDrv1Reg);
            uchTempSlotLastNo = uchTempSlotNo;
        }
    }
}

uint8_t guchCurrentOverflowPackageCrcValue = 0;
uint8_t guchCurrentOverflowPackageStatus = 0;
uint32_t gunCurrentOverflowPackageLen = 0;
__weak uint8_t GH3X2X_ReceiveOverflowLengthBuffer(uint8_t *puchRespondBuffer, uint16_t *pusRespondLen, uint8_t *puchReceiveBuffer, int32_t *pnReceiveBufferLen)
{
    int8_t chRet = GH3X2X_RET_OK;
    uint16_t usRespondLen = *pusRespondLen;
    if (puchRespondBuffer[0] == 0xBB && puchRespondBuffer[1] == 0x22)
    {
        usRespondLen -= GH3X2X_UPROTOCOL_OVERFLOW_PAYLOAD_HEADER_LEN;
        if (puchRespondBuffer[2] == 0)
        {
            if (guchCurrentOverflowPackageStatus == 1)
            {
                GH3X2X_DEBUG_LOG_PARAM("[%s]:find package loss when start!!!restart!!!\r\n", __FUNCTION__);
                guchCurrentOverflowPackageStatus = 0;
            }
            GH3X2X_Memcpy(&gunCurrentOverflowPackageLen, &puchRespondBuffer[5], sizeof(uint32_t));
            guchCurrentOverflowPackageCrcValue = puchRespondBuffer[3];
            guchCurrentOverflowPackageStatus = 1;
        }
        else if (puchRespondBuffer[2] == 2)
        {
            if (puchRespondBuffer[3] != guchCurrentOverflowPackageCrcValue || guchCurrentOverflowPackageStatus == 0)
            {
                GH3X2X_DEBUG_LOG_PARAM("[%s]:find package loss when finish!!!\r\n", __FUNCTION__);
                chRet = GH3X2X_RET_GENERIC_ERROR;
            }
            else
            {
                GH3X2X_DEBUG_LOG_PARAM("[%s]:package send finish success!!!\r\n", __FUNCTION__);
                chRet = 1;
            }
            guchCurrentOverflowPackageCrcValue = 0;
            guchCurrentOverflowPackageStatus = 0;
        }
    }
    if (chRet >= GH3X2X_RET_OK)
    {
        GH3X2X_Memcpy(puchReceiveBuffer + *pnReceiveBufferLen,\
            puchRespondBuffer + GH3X2X_UPROTOCOL_OVERFLOW_PAYLOAD_HEADER_LEN,\
            usRespondLen);
        *pnReceiveBufferLen += usRespondLen;
    }
    else
    {
        *pnReceiveBufferLen = 0;
    }
    return chRet;
}

__weak void GH3x2xSleepFlagSet(uint8_t uchSleepFlag)
{
	g_uchGh3x2xSleepFlag = uchSleepFlag;
}


__weak uint8_t GH3x2xSleepFlagGet(void)
{
	return g_uchGh3x2xSleepFlag;
}

/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/
