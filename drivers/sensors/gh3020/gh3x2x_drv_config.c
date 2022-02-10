/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_control_ex.c
 *
 * @brief   gh3x2x extension control functions, using function map like evk
 *
 * @version ref gh3x2x_drv_version.h
 *
 */


#include <stdio.h>
#include "gh3x2x_drv_version.h"
#include "gh3x2x_drv_common.h"
#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"
#include "gh3x2x_drv_control.h"
#include "gh3x2x_drv_control_ex.h"
#include "gh3x2x_drv_soft_led_agc.h"
#include "gh3x2x_drv_dump.h"
#include "gh3x2x_drv_soft_adt.h"
#include "gh3x2x_drv_config.h"


uint8_t g_usCurrentConfigFlag = 0;

/// info: reg config array version
uint16_t g_usInfoConfigArrVer = 0;

/// info: reg config create tool version
uint16_t g_usInfoConfigToolVer = 0;

/// info: project id
uint16_t g_usInfoProjectId = 0;

/// info: reg config created timestamp
uint32_t g_unInfoCreatedTimestamp = 0;

/// function started bitmap, use for sampling control
uint32_t g_unFuncStartedBitmap = 0;

/// slot enable for funcs,
uint16_t g_usSlotEnableBitsForFuncsArr[GH3020_SLOT_NUM_MAX] = {0};

/// function last fifo thr val
uint16_t g_usFuncLastFifoThrVal = 0;

/// function the fifo thr val
uint16_t g_usFuncFifoThrVal = 0;

//Ecg sample rate from hardware
uint16_t g_usEcgOutputFs = 0;

int32_t g_nAdtWearonThrd = 0;
extern uint8_t g_uchFifoPackageMode;
extern uint8_t g_uchGsensorEnable;
/// Cap enable flag
extern uint8_t g_uchCapEnable;

/// Temp enable flag
extern uint8_t g_uchTempEnable;


char *GH3X2X_GetVirtualRegVersion(void)
{
    return (char *)GH3X2X_VIRTUAL_REG_VERSION_STRING;
}



void GH3X2X_InitSensorParameters(void)
{
    g_uchGsensorEnable = 0;
    g_uchCapEnable = 0;
    g_uchTempEnable = 0;
}


/// if support algorithm run simultaneously
//uint8_t g_uchAlgoRunMode = GH3X2X_ALGORITHM_RUN_ALONE;


/**
 * @fn     int8_t GH3X2X_DecodeRegCfgArr(uint32_t* punRunMode, const struct gh3020_reg_s *pstRegConfigArr, uint16_t usRegConfigLen)
 *
 * @brief  Analyze reg cfg array to get the run mode.
 *
 * @attention   None
 *
 * @param[in]   pstRegConfigArr             pointer to reg config array
 * @param[in]   usRegConfigLen              reg config array length
 * @param[out]  punRunMode                  pointer to run mode contained in reg config array
 *
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return parameter error
 */
int8_t GH3X2X_DecodeRegCfgArr(uint32_t* punRunMode, FAR const struct gh3020_reg_s *pstRegConfigArr, uint16_t usRegConfigLen)
{
    int8_t  chRet = GH3X2X_RET_OK;
    uint16_t usRegIndex = GH3X2X_REG_IS_VIRTUAL0X2_BIT;
    *punRunMode = GH3X2X_NO_FUNCTION;
    //polling every regs in reg cfg array to get functions
    if ((GH3X2X_PTR_NULL != punRunMode) && (GH3X2X_PTR_NULL != pstRegConfigArr) && (0 != usRegConfigLen))
    {
        for (uint16_t usArrCnt = 0; usArrCnt < usRegConfigLen ; usArrCnt ++)
        {
            while(pstRegConfigArr[usArrCnt].regaddr > usRegIndex)
            {
                usRegIndex += GH3X2X_CHNLMAP_OFFSET;
            }
            if (pstRegConfigArr[usArrCnt].regaddr == usRegIndex)
            {
                if (pstRegConfigArr[usArrCnt].regval)
                {
                    *punRunMode |= (0x1 << ((usRegIndex - GH3X2X_REG_IS_VIRTUAL0X2_BIT) / GH3X2X_CHNLMAP_OFFSET));
                    usRegIndex += GH3X2X_CHNLMAP_OFFSET;
                }
            }
            if (usRegIndex > 0x2880)
            {
                break;
            }
        }
    }
    else
    {
        chRet = GH3X2X_RET_PARAMETER_ERROR;
    }
    return chRet;
}

/**
 * @fn     uint16_t GH3X2X_ReadSwConfigWithVirtualReg(uint16_t usVirtualRegAddr)
 *
 * @brief  Read software config with virtual reg
 *
 * @attention   Virtual reg addr has del control bits, so reg addr is [0:11] valid.
 *
 * @param[in]   usVirtualRegAddr        virtual reg addr
 * @param[out]  None
 *
 * @return  virtual reg val
 */
uint16_t GH3X2X_ReadSwConfigWithVirtualReg(uint16_t usVirtualRegAddr)
{
    uint16_t usVirtualRegData = 0;
#if 0
    uint16_t usValIndex = 0;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if (usVirtualRegAddr < GH3020_REG_VINFO_END) // config arr info
    {
        switch (usVirtualRegAddr)
        {
        case GH3020_REG_VINFO_CFG_VER:
            usVirtualRegData = g_usInfoConfigArrVer;
            break;
        case GH3020_REG_VINFO_TOOL_VER:
            usVirtualRegData = g_usInfoConfigToolVer;
            break;
        case GH3020_REG_VINFO_PROJ_ID:
            usVirtualRegData = g_usInfoProjectId;
            break;
        case GH3020_REG_VINFO_TIMSTAMP_L:
            usVirtualRegData = GH3X2X_GET_LOW_WORD_FROM_DWORD(g_unInfoCreatedTimestamp);
            break;
        case GH3020_REG_VINFO_TIMSTAMP_H:
            usVirtualRegData = GH3X2X_GET_HIGH_WORD_FROM_DWORD(g_unInfoCreatedTimestamp);
            break;
        default: // do nothing
            break;
        }
    } // end of if (usVirtualRegAddr < GH3020_REG_VINFO_END)
    else if (usVirtualRegAddr < GH3020_REG_VCHM_END) // config channel map
    {
        if (usVirtualRegAddr < GH3020_REG_VCHM_HBA_CH_END) // hba
        {
            switch (usVirtualRegAddr)
            {
            case GH3020_REG_VCHM_HBA_CNT:
                usVirtualRegData = g_uchHbaChannelMapCnt;
                break;
            default:
                usValIndex = (uint16_t)(usVirtualRegAddr - GH3020_REG_VCHM_HBA_CH0_1);
                if (usValIndex < GH3X2X_HBA_CHANNEL_MAP_MAX_CNT)
                {
                    usVirtualRegData = GH3X2X_MAKEUP_WORD(g_uchHbaChannelMapArr[usValIndex + 1], \
                                                        g_uchHbaChannelMapArr[usValIndex]);
                }
                break;
            }
        }
        else if (usVirtualRegAddr < GH3020_REG_VCHM_SPO2_CH_END) // spo2
        {
            switch (usVirtualRegAddr)
            {
            case GH3020_REG_VCHM_SPO2_CNT:
                usVirtualRegData = g_uchSpo2ChannelMapCnt;
                break;
            default:
                usValIndex = (uint16_t)(usVirtualRegAddr - GH3020_REG_VCHM_SPO2_CH0_1);
                if (usValIndex < GH3X2X_SPO2_CHANNEL_MAP_MAX_CNT)
                {
                    usVirtualRegData = GH3X2X_MAKEUP_WORD(g_uchSpo2ChannelMapArr[usValIndex + 1], \
                                                        g_uchSpo2ChannelMapArr[usValIndex]);
                }
                break;
            }
        }
        else
        {
            switch (usVirtualRegAddr)
            {
            case GH3020_REG_VCHM_ECG_CH0:
                usVirtualRegData = g_uchEcgChannelMap;
                break;
            case GH3020_REG_VCHM_ADT_CNT:
                usVirtualRegData = g_uchAdtChannelMapCnt;
                break;
            case GH3020_REG_VCHM_ADT_CH0_1:
                usVirtualRegData = GH3X2X_MAKEUP_WORD(g_uchAdtChannelMapArr[1], g_uchAdtChannelMapArr[0]);
                break;
            case GH3020_REG_VCHM_ADT_CH2_3:
                usVirtualRegData = GH3X2X_MAKEUP_WORD(g_uchAdtChannelMapArr[3], g_uchAdtChannelMapArr[2]);
                break;
            default: // do nothing
                break;
            } // end of switch (usVirtualRegAddr)
        }
    } // end of else if (usVirtualRegAddr < GH3020_REG_VCHM_END)
    else if (usVirtualRegAddr < GH3020_REG_VDUMP_END) // dump module
    {
        switch (usVirtualRegAddr)
        {
        case GH3020_REG_VDUMP_DUMP_MODE:
            break;
        case GH3020_REG_VDUMP_BG_LEVEL:
            break;
        default: // do nothing
            break;
        }
    }
    else if (usVirtualRegAddr < GH3020_REG_VECG_END) // ecg module
    {
        switch (usVirtualRegAddr)
        {
        case GH3020_REG_VECG_0:
        case GH3020_REG_VECG_1:
        case GH3020_REG_VECG_2:
            usValIndex = (usVirtualRegAddr - GH3020_REG_VECG_0) / GH3020_REG_ADDR_SIZE;
            break;
        default: // do nothing
            break;
        }
    } // end of else if (usVirtualRegAddr < GH3020_REG_VECG_END)
    else if (usVirtualRegAddr < GH3020_REG_VAGC_END) // agc config
    {
        switch (usVirtualRegAddr)
        {
        case GH3020_REG_VAGC_AGC_BG_SLOT_ENABLE:
            break;
        case GH3020_REG_VAGC_AMB_SLOT_CTRL:
            break;
        case GH3020_REG_VAGC_RES:
            break;
        default: // do nothing
            break;
        }
    } // end of else if (usVirtualRegAddr < GH3020_REG_VAGC_END)
#endif
    return usVirtualRegData;
}

__weak void GH3X2X_WriteFsConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue)
{
    uint8_t uchFuncOffsetIndex =  (usVirtualRegAddr - GH3X2X_ADT_FS_ADDR) / 2;
    if (uchFuncOffsetIndex == GH3X2X_FUNC_OFFSET_SPO2)
    {
        //goodix_spo2_reset_correct_factor();
    }
    if(g_pstGh3x2xFrameInfo[uchFuncOffsetIndex] != 0)
    {
        g_pstGh3x2xFrameInfo[uchFuncOffsetIndex] ->pstFunctionInfo ->usSampleRate = GH3X2X_GET_LOW_WORD_FROM_DWORD(usVirtualRegValue);
    }
}

__weak void GH3X2X_WriteAgcConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue)
{
    uint16_t usValIndex;
    switch (usVirtualRegAddr)
    {
    case GH3X2X_AGC_BG_CANCEL_ADJUST_SLOT_EN_ADDR:
    case GH3X2X_AGC_AMB_SLOT_CTRL_ADDR:
    case GH3X2X_AGC_GAIN_LIMIT_ADDR:
    case GH3X2X_AGC_TRIG_THD_H_LSB_16_ADDR:
    case GH3X2X_AGC_TRIG_THD_H_MSB_16_ADDR:
    case GH3X2X_AGC_TRIG_THD_L_LSB_16_ADDR:
    case GH3X2X_AGC_TRIG_THD_L_MSB_16_ADDR:
    case GH3X2X_AGC_RESTRAIN_THD_H_LSB_16_ADDR:
    case GH3X2X_AGC_RESTRAIN_THD_H_MSB_16_ADDR:
    case GH3X2X_AGC_RESTRAIN_THD_L_LSB_16_ADDR:
    case GH3X2X_AGC_RESTRAIN_THD_L_MSB_16_ADDR:
        usValIndex = (usVirtualRegAddr - GH3X2X_AGC_BG_CANCEL_ADJUST_SLOT_EN_ADDR) / GH3020_REG_ADDR_SIZE;
        GH3X2X_LedAgcPramWrite(usVirtualRegValue, usValIndex);
        break;
    default: // do nothing
        break;
    }
}

void GH3X2X_WriteChnlMapConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue)
{
    uint16_t usValIndex = 0;
    usVirtualRegAddr = GH3X2X_GET_REG_REAL_ADRR(usVirtualRegAddr);
    switch (usVirtualRegAddr % GH3X2X_CHNLMAP_OFFSET)
    {
    case 0:
        GH3x2xSetFunctionChnlNum(g_pstGh3x2xFrameInfo[usVirtualRegAddr / GH3X2X_CHNLMAP_OFFSET],GH3X2X_GET_LOW_BYTE_FROM_WORD(usVirtualRegValue));
        break;
    default:
        usValIndex = (uint16_t)((usVirtualRegAddr % GH3X2X_CHNLMAP_OFFSET) - 2);
        if (usValIndex < GH3X2X_CHANNEL_MAP_MAX_CH)
        {
            GH3x2xSetFunctionChnlMap(g_pstGh3x2xFrameInfo[usVirtualRegAddr / GH3X2X_CHNLMAP_OFFSET],usValIndex + 1,GH3X2X_GET_HIGH_BYTE_FROM_WORD(usVirtualRegValue));
            GH3x2xSetFunctionChnlMap(g_pstGh3x2xFrameInfo[usVirtualRegAddr / GH3X2X_CHNLMAP_OFFSET],usValIndex,GH3X2X_GET_LOW_BYTE_FROM_WORD(usVirtualRegValue));
        }
        break;
    }
}


/**
 * @fn     int8_t GH3X2X_WriteSwConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue)
 *
 * @brief  Write software param config with virtual reg
 *
 * @attention   Virtual reg addr has del control bits, so reg addr is [0:11] valid.
 *
 * @param[in]   usVirtualRegAddr        virtual reg addr
 * @param[in]   usVirtualRegValue       virtual reg value
 * @param[out]  None
 *
 * @return  None
 */
__weak void GH3X2X_WriteSwConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue)
{
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if (usVirtualRegAddr < GH3X2X_SLOT_LEDDRV_ADDR) // config arr info         0x1F
    {
        switch (usVirtualRegAddr)
        {
        case GH3X2X_CFG_VER_ADDR:
            g_usInfoConfigArrVer = usVirtualRegValue;
            break;
        case GH3X2X_CFG_TOOL_VER_ADDR:
            g_usInfoConfigToolVer = usVirtualRegValue;
            break;
        case GH3X2X_PROJECT_ID_ADDR:
            g_usInfoProjectId = usVirtualRegValue;
            break;
        case GH3X2X_TIMESTAMP_L_ADDR:
            g_unInfoCreatedTimestamp = GH3X2X_MAKEUP_DWORD2(GH3X2X_GET_HIGH_WORD_FROM_DWORD(g_unInfoCreatedTimestamp),
                                                                usVirtualRegValue);
            break;
        case GH3X2X_TIMESTAMP_H_ADDR:
            g_unInfoCreatedTimestamp = GH3X2X_MAKEUP_DWORD2(usVirtualRegValue,
                                                            GH3X2X_GET_LOW_WORD_FROM_DWORD(g_unInfoCreatedTimestamp));
            break;
        case GH3X2X_REINIT_PARAM_ADDR:
            GH3X2X_ReinitAllSwModuleParam(usVirtualRegValue);
            break;
        default: // do nothing
            break;
        }
    } // end of if (usVirtualRegAddr < GH3020_REG_VINFO_END)
    else if (usVirtualRegAddr < GH3X2X_DUMP_CFG_ADDR) // slot drv map module
    {
        //TO BE DONE
    }
    else if (usVirtualRegAddr < GH3X2X_FIFO_CTRL_ADDR) // dump module
    {
        switch (usVirtualRegAddr)
        {
        case GH3X2X_PPG_DUMP_MODE_ADDR:
            GH3X2X_DumpModeSet(usVirtualRegValue);
            break;
        case GH3X2X_BG_LEVEL_SET_ADDR:
            //GH3X2X_BgLevelSet(usVirtualRegValue);
            break;
        default: // do nothing
            break;
        }
    } // end of else if (usVirtualRegAddr < GH3020_REG_VDUMP_END)
    else if (usVirtualRegAddr < GH3X2X_G_SENSOR_CFG_ADDR) // fifo info
    {
        switch (usVirtualRegAddr)
        {
        case GH3X2X_FIFO_PACKAGE_SEND_MODE_ADDR:
            g_uchFifoPackageMode = usVirtualRegValue;
            break;
        default: // do nothing
            break;
        }
    }
    else if (usVirtualRegAddr < GH3X2X_SOFT_AGC_PARA_ADDR)
    {
        switch (usVirtualRegAddr)
        {
        case GH3X2X_GSENSOR_CTRL_ADDR:
            g_uchGsensorEnable = GH3X2X_GET_LOW_BYTE_FROM_WORD(usVirtualRegValue);
            break;
        default: // do nothing
            break;
        }
    } // end of else if (usVirtualRegAddr < GH3020_REG_VGS_END)
    else if (usVirtualRegAddr < GH3X2X_CAP_CFG_ADDR) // agc config
    {
        GH3X2X_WriteAgcConfigWithVirtualReg(usVirtualRegAddr, usVirtualRegValue);

    } // end of else if (usVirtualRegAddr < GH3020_REG_VAGC_END)
    else if (usVirtualRegAddr < GH3X2X_TEMP_CFG_ADDR)
    {
        switch (usVirtualRegAddr)
        {
        case GH3X2X_CAP_CTRL_ADDR:
            g_uchCapEnable = GH3X2X_GET_LOW_BYTE_FROM_WORD(usVirtualRegValue);
            break;
        default: // do nothing
            break;
        }
    }
    else if (usVirtualRegAddr < GH3X2X_CHNL_MAP_ADDR)
    {
        switch (usVirtualRegAddr)
        {
        case GH3X2X_TEMP_CTRL_ADDR:
            g_uchTempEnable = GH3X2X_GET_LOW_BYTE_FROM_WORD(usVirtualRegValue);
            break;
        default: // do nothing
            break;
        }
    }
    else if (usVirtualRegAddr < GH3X2X_FS_PARA_ADDR)
    {
        GH3X2X_WriteChnlMapConfigWithVirtualReg(usVirtualRegAddr, usVirtualRegValue);
    }
    else if (usVirtualRegAddr < GH3X2X_REG_IS_VIRTUAL0X3_BIT)
    {
        GH3X2X_WriteFsConfigWithVirtualReg(usVirtualRegAddr, usVirtualRegValue);
    }
}

__weak void GH3X2X_WriteEcgDrvConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue)
{
    uint16_t usValIndex;
    switch (usVirtualRegAddr)
    {
    case GH3X2X_ECG_SETTING0_ADDR:
    case GH3X2X_ECG_SETTING1_ADDR:
    case GH3X2X_ECG_SETTING2_ADDR:
        usValIndex = (usVirtualRegAddr - GH3X2X_ECG_SETTING0_ADDR) / GH3020_REG_ADDR_SIZE;
        GH3X2X_SlaverSoftLeadPramSet(usVirtualRegValue, usValIndex);                                   //_weak
        break;
    case GH3X2X_ECG_SETTING3_ADDR:
        //GH3x2xSetEcgOutputFs(usVirtualRegValue);
        break;
    default: // do nothing
        break;
    }
}

__weak void GH3X2X_WriteAdtDrvConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue)
{
    switch (usVirtualRegAddr)
    {
    case 0x0144:
      g_nAdtWearonThrd |= ((int32_t)usVirtualRegValue & 0x0000FFFF);
      break;
    case 0x0146:
      g_nAdtWearonThrd |= ((int32_t)usVirtualRegValue << 16);
      break;
    default:
      g_nAdtWearonThrd = 0;
      uint16_t usValIndex = (usVirtualRegAddr - GH3X2X_ADT_CONFIG0_ADDR) / GH3020_REG_ADDR_SIZE;
      GH3X2X_AdtPramSet(usVirtualRegValue, usValIndex);
      break;
    }

}


/**
 * @fn     int8_t GH3X2X_WriteAlgorithmConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue)
 *
 * @brief  Write algorithm param config with virtual reg
 *
 * @attention   Virtual reg addr has del control bits, so reg addr is [0:11] valid.
 *
 * @param[in]   usVirtualRegAddr        virtual reg addr
 * @param[in]   usVirtualRegValue       virtual reg value
 * @param[out]  None
 *
 * @return  None
 */
__weak void GH3X2X_WriteFunctionConfigWithVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue)
{
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    uint32_t unFunctionID = (usVirtualRegAddr - GH3X2X_REG_IS_VIRTUAL0X3_BIT) / GH3X2X_VREG_FUNCTION_OFFSET;
    switch (unFunctionID)
    {
    case GH3X2X_FUNC_OFFSET_ADT:
        if (usVirtualRegAddr < GH3X2X_ADT_ALG_CFG_ADDR)
        {
            GH3X2X_WriteAdtDrvConfigWithVirtualReg(usVirtualRegAddr, usVirtualRegValue);
        }
        else
        {
            ///* to be done */
        }
        break;
    case GH3X2X_FUNC_OFFSET_HR:
        if (usVirtualRegAddr < GH3X2X_HR_ALG_CFG_ADDR)
        {
            ///* to be done */
        }
        else
        {
            GH3X2X_WriteHrAlgConfigWithVirtualReg(usVirtualRegAddr, usVirtualRegValue);
        }
        break;
    case GH3X2X_FUNC_OFFSET_HRV:
        if (usVirtualRegAddr < GH3X2X_HRV_ALG_CFG_ADDR)
        {
            ///* to be done */
        }
        else
        {
            GH3X2X_WriteHrvAlgConfigWithVirtualReg(usVirtualRegAddr, usVirtualRegValue);
        }
        break;
    case GH3X2X_FUNC_OFFSET_HSM:
        break;
    case GH3X2X_FUNC_OFFSET_FPBP:
    case GH3X2X_FUNC_OFFSET_PWA:
        if (usVirtualRegAddr < GH3X2X_FPBP_ALG_CFG_ADDR)
        {
            ///* to be done */
        }
        else
        {
            GH3X2X_WriteBpAlgConfigWithVirtualReg(usVirtualRegAddr, usVirtualRegValue);
        }
        break;
    case GH3X2X_FUNC_OFFSET_SPO2:
        if (usVirtualRegAddr < GH3X2X_SPO2_ALG_CFG_ADDR)
        {
            ///* to be done */
        }
        else
        {
            GH3X2X_WriteSpo2AlgConfigWithVirtualReg(usVirtualRegAddr, usVirtualRegValue);
        }
        break;
    case GH3X2X_FUNC_OFFSET_ECG:
        if (usVirtualRegAddr < GH3X2X_ECG_ALG_CFG_ADDR)
        {
            GH3X2X_WriteEcgDrvConfigWithVirtualReg(usVirtualRegAddr, usVirtualRegValue);
        }
        else
        {
            ///* to be done */
        }
        break;
    case GH3X2X_FUNC_OFFSET_PWTT:
        break;
    case GH3X2X_FUNC_OFFSET_SOFT_ADT_GREEN:
        break;
    case GH3X2X_FUNC_OFFSET_BT:
        if (usVirtualRegAddr < GH3X2X_BT_ALG_CFG_ADDR)
        {
            ///* to be done */
        }
        else
        {
            GH3X2X_WriteBtAlgConfigWithVirtualReg(usVirtualRegAddr, usVirtualRegValue);
        }
        break;
    case GH3X2X_FUNC_OFFSET_RESP:
        break;
    case GH3X2X_FUNC_OFFSET_AF:
        if (usVirtualRegAddr < GH3X2X_AF_ALG_CFG_ADDR)
        {
            ///* to be done */
        }
        else
        {
            GH3X2X_WriteAfAlgConfigWithVirtualReg(usVirtualRegAddr, usVirtualRegValue);
        }
        break;
    case GH3X2X_FUNC_OFFSET_TEST1:
        break;
    case GH3X2X_FUNC_OFFSET_TEST2:
        break;
    case GH3X2X_FUNC_OFFSET_SOFT_ADT_IR:
        break;
    default:
        break;
    }
}

void GH3X2X_ConfigNeedWakeUp(void)
{
    uint16_t usTempVal = gh3020_spi_readbits(0x0000, 10, 10);
    GH3X2X_DEBUG_LOG_PARAM("[%s]\r\n", __FUNCTION__);
    if(usTempVal == 0)
    {
        GH3x2x_SetNeedWakeUpGh3x2xFlagBeforeInt(1);
    }
    else
    {
        GH3x2x_SetNeedWakeUpGh3x2xFlagBeforeInt(0);
    }
}

/**
 * @fn     int8_t GH3X2X_WriteVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue)
 *
 * @brief  Write virtual reg val, for software param config
 *
 * @attention   Virtual reg addr has del control bits, so reg addr is [0:11] valid.
 *
 * @param[in]   usVirtualRegAddr        virtual reg addr
 * @param[in]   usVirtualRegValue       virtual reg value
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_WriteVirtualReg(uint16_t usVirtualRegAddr, uint16_t usVirtualRegValue)
{
    if (usVirtualRegAddr < GH3X2X_REG_IS_VIRTUAL0X3_BIT)
    {
        GH3X2X_SW_CONFIG_WRITE(usVirtualRegAddr, usVirtualRegValue);
    }
    else if (usVirtualRegAddr < GH3X2X_FINISH_FLAG_ADDR)
    {
        GH3X2X_FUNCTION_CONFIG_WRITE(usVirtualRegAddr, usVirtualRegValue);
    }

    switch (usVirtualRegAddr)
    {
        case GH3X2X_END_FLAG_ADDR: // virtual reg write finish
            if (g_usCurrentConfigFlag == 1)
            {
                g_usCurrentConfigFlag = 0;
            }
            else
            {
                GH3X2X_DEBUG_LOG_PARAM("[%s]:Config version error!!!\r\n", __FUNCTION__);
            }
            Gh3x2xPollingModePro();
            GH3x2xSlotTimeInfo();
            GH3X2X_ConfigNeedWakeUp();
            break;
        case GH3X2X_TOP_INFO_ADDR:
            g_usCurrentConfigFlag = 1;
            GH3X2X_InitSensorParameters();
            break;
        default:
            break;
    }
}

__weak void GH3x2xSlotTimeInfo(void)
{
}

__weak void Gh3x2xPollingModePro(void)
{
	gh3020_spi_writereg(0x0502,0); //disable INT
}


/**
 * @fn     uint16_t GH3X2X_ReadVirtualReg(uint16_t usVirtualRegAddr)
 *
 * @brief  Read virtual reg val, for software param config
 *
 * @attention   Virtual reg addr has del control bits, so reg addr is [0:11] valid.
 *
 * @param[in]   usVirtualRegAddr        virtual reg addr
 * @param[out]  None
 *
 * @return  virtual reg val
 */
uint16_t GH3X2X_ReadVirtualReg(uint16_t usVirtualRegAddr)
{
    uint16_t usVirtualRegData = 0;
#if GH3X2X_SUPPORT_READ_BACK_VIRTUAL_REG

#endif
    return usVirtualRegData;
}

void GH3X2X_CheckChipModel(void)
{
    uint16_t usRegVal = 0;
    usRegVal = gh3020_spi_readreg(GH3020_REG_EFUSE1_AUTOLOAD_0);
    //if chip is 3026,disable rx2 and rx3
    if(((usRegVal >> 5)&0x07) == 1)
    {
        for(uint8_t uchSlotNo = 0; uchSlotNo < 8; uchSlotNo++)
        {
            gh3020_spi_writebits(GH3020_REG_SLOT0_CTRL_0 + GH3020_OFFSET_SLOT_CTRL*uchSlotNo,4,5, 0);
        }
    }

}

/**
 * @fn     int8_t GH3X2X_LoadNewRegConfigArr(const struct gh3020_reg_s *pstRegConfigArr, uint16_t usRegConfigLen)
 *
 * @brief  Load new gh3x2x reg config array
 *
 * @attention   If reg val don't need verify, should set reg addr bit 12;
 *              If reg is virtual reg, should set reg addr bit 13;
 *              e.g.      need config reg 0x0000: 0x0611
 *                        {0x0000, 0x0611} //verify write by read reg
 *                        {0x1000, 0x0611} //don't need verify write val
 *                        {0x2000, 0x0611} //set virtual reg
 *
 * @param[in]   pstRegConfigArr       pointer to the reg struct array
 * @param[in]   usRegConfigLen        reg struct array length
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_COMM_ERROR       gh3x2x communicate error
 */
int8_t GH3X2X_LoadNewRegConfigArr(FAR const struct gh3020_reg_s *pstRegConfigArr, uint16_t usRegConfigLen)
{
    int8_t chRet = GH3X2X_RET_OK;
    uint16_t usIndex = 0;
    uint16_t usReadRegVal = 0;

    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if ((pstRegConfigArr != GH3X2X_PTR_NULL) && (usRegConfigLen != 0))
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        for (usIndex = 0; usIndex < usRegConfigLen; usIndex++) // write GH3X2X reg.
        {
            switch (GH3X2X_GET_BIT_IS_SET(pstRegConfigArr[usIndex].regaddr))
            {
            case 0:
                gh3020_spi_writereg(pstRegConfigArr[usIndex].regaddr, pstRegConfigArr[usIndex].regval);
                usReadRegVal = gh3020_spi_readreg(pstRegConfigArr[usIndex].regaddr);
                if (usReadRegVal != pstRegConfigArr[usIndex].regval)
                {
                    GH3X2X_DEBUG_LOG_PARAM("[%s]:reg verify error! addr:0x%.4x,w_val:0x%.4x,r_val:0x%.4x\r\n", __FUNCTION__,
                                            GH3X2X_GET_REG_REAL_ADRR(pstRegConfigArr[usIndex].regaddr),
                                            pstRegConfigArr[usIndex].regval, usReadRegVal);
                    chRet = GH3X2X_RET_COMM_ERROR;
                }
                break;
            default:
                if (pstRegConfigArr[usIndex].regaddr > GH3X2X_TOP_INFO_ADDR && 0 == g_usCurrentConfigFlag)
                {
                    GH3X2X_DEBUG_LOG_PARAM("[%s]:Config version error!!!\r\n", __FUNCTION__);
                    return GH3X2X_RET_GENERIC_ERROR;
                }
                /* write & verify virtual reg, if verify error, only log error, shouldn't return error */
                GH3X2X_WriteVirtualReg(pstRegConfigArr[usIndex].regaddr, pstRegConfigArr[usIndex].regval);
                #if 0
                usReadRegVal = GH3X2X_ReadVirtualReg(pstRegConfigArr[usIndex].usRegAddr);
                if (usReadRegVal != pstRegConfigArr[usIndex].usRegData)
                {
                    GH3X2X_DEBUG_LOG_PARAM("[%s]:vreg reg verify error! addr:0x%.4x,w_val:0x%.4x,r_val:0x%.4x\r\n", __FUNCTION__,
                                            pstRegConfigArr[usIndex].usRegAddr,
                                            pstRegConfigArr[usIndex].usRegData, usReadRegVal);
                }
                #endif
                break;
            }
        } // end of for (usIndex = 0; usIndex < usRegConfigLen; usIndex++)
        GH3X2X_WAIT_CHIP_DSLP();
    } // end of if ((pstRegConfigArr != GH3X2X_PTR_NULL) && (usRegConfigLen != 0))
    GH3X2X_CheckChipModel();
    return chRet;
}




void GH3X2X_SetCurrentConfigFlag(uint8_t value)
{
    g_usCurrentConfigFlag = value;
}





__weak void GH3x2xSetEcgOutputFs(uint16_t usEcgOutputFs)
{

	g_usEcgOutputFs = usEcgOutputFs;

}












