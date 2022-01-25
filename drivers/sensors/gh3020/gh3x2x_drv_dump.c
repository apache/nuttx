/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_dump.c
 *
 * @brief   gh3x2x dump data functions
 *
 * @version ref gh3x2x_drv_version.h
 *
 */


#include <stdio.h>
#include "gh3x2x_drv_version.h"
#include "gh3x2x_drv_common.h"
#include "gh3x2x_drv.h"
#include "gh3x2x_drv_interface.h"
#include "gh3x2x_drv_control.h"
#include"gh3x2x_drv_control_ex.h"
#include "gh3x2x_drv_uprotocol.h"
#include "gh3x2x_drv_dump.h"
#include "gh3x2x_drv_soft_led_agc.h"








/// dump mode
GU16 g_usDumpMode;

/// electrode wear status
//GU8 g_uchElectrodeWearStatus;

/// save bg cancel value of every channel
GU8  g_puchGainBgCancelRecord[GH3X2X_CHANNEL_MAP_MAX_CH];


GU8 g_puchTiaGainAfterSoftAgc[GH3X2X_CHANNEL_MAP_MAX_CH];

/// save current of drv0 and drv1
GU16 g_pusDrvCurrentRecord[GH3X2X_CHANNEL_MAP_MAX_CH];

/**
 * @fn     GU16 GH3X2X_DumpModeGet(void)
 *
 * @brief  get dump mode
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  dump mode
 */
GU16 GH3X2X_DumpModeGet(void)
{
    return g_usDumpMode;
}

/**
 * @fn     void GH3X2X_DumpModeSet(GU16 usRegVal)
 *
 * @brief  set dump mode
 *
 * @attention   None
 *
 * @param[in]   usRegVal          reg value
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_DumpModeSet(GU16 usRegVal)
{
    g_usDumpMode = usRegVal;
}


const GU16 g_usGH3x2xSlotRegBase[8] = 
{
    0x0100,
    0x011C,
    0x0138,
    0x0154,
    0x0170,
    0x018C,
    0x01A8,
    0x01C4,
};








const STGh3x2xAgcReg g_pstGH3x2xAgcReg[] = 
{
    {0x10, 9, 8},              //0: agc en
    {0x22, 7, 0},              //1: led current up limit
    {0x22, 15, 8},               //2: led current down limit
    {0x1E, 7, 0},               //3: led current drv0
    {0x20, 7, 0},               //4: led current drv1
    {0x12, 3, 0},               //5: rx0 tia gain
    {0x12, 7, 4},               //6: rx1 tia gain
    {0x12, 11, 8},               //7: rx2 tia gain
    {0x12, 15, 12},               //8: rx3 tia gain
    {0x10, 7,6},               //9: agc src
    {0x1A, 1,0},               //10: rx0 bg cancel
    {0x1A, 3,2},               //11: rx1 bg cancel    
    {0x1A, 5,4},               //12: rx2 bg cancel
    {0x1A, 7,6},               //13: rx2 bg cancel
    {0x0A, 15,8},               //14: sr mulipliper
    {0x0A, 5,1},               //15: ecg en& rx0~rx1 en
};

const GU16 g_usGH3x2xTiaGainR[] = 
{
    10,
    25,
    50,
    75,
    100,
    250,
    500,
    750,
    1000,
    1250,
    1500,
    1750,
    2000,
};

void GH3x2xSetAgcReg(GU8 uchAgcReg,  GU8 uchSlot, GU16 usValue)
{
    GH3X2X_WriteRegBitField(g_usGH3x2xSlotRegBase[uchSlot] + g_pstGH3x2xAgcReg[uchAgcReg].uchAddrOffset, g_pstGH3x2xAgcReg[uchAgcReg].uchLsb, g_pstGH3x2xAgcReg[uchAgcReg].uchMsb, usValue);
}

GU16 GH3x2xGetAgcReg(GU8 uchAgcReg,  GU8 uchSlot)
{
    return GH3X2X_ReadRegBitField(g_usGH3x2xSlotRegBase[uchSlot] + g_pstGH3x2xAgcReg[uchAgcReg].uchAddrOffset, g_pstGH3x2xAgcReg[uchAgcReg].uchLsb, g_pstGH3x2xAgcReg[uchAgcReg].uchMsb);
}





/**
 * @fn       void GH3X2X_ElectrWearDumpDataPro(GU8* puchRawdataTag, GU8 uchChMapIndex)
 *
 * @brief    electrode wear dump data pro
 *
 * @attention    None
 *
 * @param[in]    puchRawdataTag      data tag of rawdata
 * @param[in]    uchChMapIndex       channel map index
 * @param[out]   None
 *
 * @return       None
 */
 /*
void GH3X2X_ElectrWearDumpDataPro(GU8* puchRawdataTag, GU8 uchChMapIndex)
{
    if (GH3X2X_CHECK_LEFT_BIT_SET(GH3X2X_DumpModeGet(), GH3X2X_DUMP_ELECTR_WEAR_ENABLE_BIT))
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        g_uchElectrodeWearStatus = GH3X2X_ReadRegBitField(GH3X2X_INT_STR2_REG_ADDR, \
                                    GH3X2X_ELECTR_WEAR_STATUS_BIT_FIELD, GH3X2X_ELECTR_WEAR_STATUS_BIT_FIELD);
        GH3X2X_WAIT_CHIP_DSLP();
    }
}
*/

/**
 * @fn     void GH3X2X_RecordGainBgCancelValue(void)
 *
 * @brief  read gain and bg cancel value, and save them
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
 /*
void GH3X2X_RecordGainBgCancelValue(void)
{
    GU8  uchSlotIndexCnt   = 0;
    GU8  uchAdcIndexCnt    = 0;
    GU8  uchSlotAdcIndex   = 0;
    GU8  uchBgCancelValue  = 0;
    GU8  uchGainValue      = 0;
    GU16 usBgCancelRegData = 0;
    GU16 usBgCancelRegAddr = 0;    
    GU16 usTiaGainRegData  = 0;
    GU16 usTiaGainRegAddr  = 0;    

    GH3X2X_WAIT_CHIP_WAKEUP();
    usBgCancelRegAddr = GH3X2X_SLOT0_CTRL_8_REG_ADDR;
    usTiaGainRegAddr  = GH3X2X_SLOT0_CTRL_4_REG_ADDR;
    for (uchSlotIndexCnt = 0; uchSlotIndexCnt < GH3X2X_MAX_SLOT_NUM; uchSlotIndexCnt++)
    {
        usBgCancelRegData = GH3X2X_ReadReg(usBgCancelRegAddr);
        usTiaGainRegData  = GH3X2X_ReadReg(usTiaGainRegAddr);
        for (uchAdcIndexCnt = 0; uchAdcIndexCnt < GH3X2X_ADC_NUM; uchAdcIndexCnt++)
        {
            uchBgCancelValue = (usBgCancelRegData >> (uchAdcIndexCnt * GH3X2X_ADC_BG_CANCEL_BIT_NUM)) \
                                & GH3X2X_ADC_BG_CANCEL_BIT_MASK;
            uchGainValue     = (usTiaGainRegData >> (uchAdcIndexCnt * GH3X2X_ADC_TIA_GAIN_BIT_NUM)) \
                                & GH3X2X_ADC_TIA_GAIN_BIT_MASK;
            g_puchGainBgCancelRecord[uchSlotAdcIndex] = (GU8)(uchBgCancelValue << \
                                                                GH3X2X_ADC_BG_CANCEL_LEFT_SHIFT_BIT_NUM);
            g_puchGainBgCancelRecord[uchSlotAdcIndex] |= (GU8)(uchGainValue << GH3X2X_ADC_TIA_GAIN_LEFT_SHIFT_BIT_NUM);
            uchSlotAdcIndex++;
        }
        usBgCancelRegAddr += SLOT_ADDR_OFFSET;
        usTiaGainRegAddr  += SLOT_ADDR_OFFSET;
    }
    GH3X2X_WAIT_CHIP_DSLP();
}
*/

/**
 * @fn     void GH3X2X_RecordDrvCurrentValue(void)
 *
 * @brief  read current value of drv0 and drv1, and save them
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_RecordDrvCurrentValue(void)
{
    GU8  uchSlotIndexCnt  = 0;
    GU8  uchAdcIndexCnt   = 0;
    GU8  uchSlotAdcIndex  = 0;
    GU16 usDrv0CurRegData = 0;
    GU16 usDrv0CurRegAddr = 0;    
    GU16 usDrv1CurRegData = 0;
    GU16 usDrv1CurRegAddr = 0;

    if (GH3X2X_LED_AGC_ENABLE == GH3X2X_GetLedAgcState() && \
        (GH3X2X_CHECK_LEFT_BIT_NOT_SET(g_usDumpMode, GH3X2X_DUMP_GAIN_CURRENT_ENABLE_BIT)) && \
        (GH3X2X_CHECK_LEFT_BIT_NOT_SET(g_usDumpMode, GH3X2X_DUMP_AMBIANCE_ENABLE_BIT)))
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        usDrv0CurRegAddr = GH3X2X_SLOT0_CTRL_10_REG_ADDR;
        usDrv1CurRegAddr = GH3X2X_SLOT0_CTRL_11_REG_ADDR;
        for (uchSlotIndexCnt = 0; uchSlotIndexCnt < GH3X2X_MAX_SLOT_NUM; uchSlotIndexCnt++)
        {
            usDrv0CurRegData = (GH3X2X_ReadReg(usDrv0CurRegAddr) & GH3X2X_DRV_CURRENT_BIT_MASK);
            usDrv1CurRegData = (GH3X2X_ReadReg(usDrv1CurRegAddr) & GH3X2X_DRV_CURRENT_BIT_MASK);
            for (uchAdcIndexCnt = 0; uchAdcIndexCnt < GH3X2X_ADC_NUM; uchAdcIndexCnt++)
            {
                g_pusDrvCurrentRecord[uchSlotAdcIndex] =  (GU16)(usDrv0CurRegData << \
                                                                    GH3X2X_DRV0_CURRENT_LEFT_SHIFT_BIT_NUM);
                g_pusDrvCurrentRecord[uchSlotAdcIndex] |= (GU16)(usDrv1CurRegData << \
                                                                    GH3X2X_DRV1_CURRENT_LEFT_SHIFT_BIT_NUM);
                uchSlotAdcIndex++;
            }
            usDrv0CurRegAddr += SLOT_ADDR_OFFSET;
            usDrv1CurRegAddr += SLOT_ADDR_OFFSET;
        }
        GH3X2X_WAIT_CHIP_DSLP();
    }
}

/**
 * @fn     void GH3X2X_RecordDumpData(void)
 *
 * @brief  read some dump data, and save them
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_RecordDumpData(void)
{
    if (GH3X2X_LED_AGC_ENABLE == GH3X2X_GetLedAgcState() || \
        GH3X2X_CHECK_LEFT_BIT_SET(g_usDumpMode, GH3X2X_DUMP_AMBIANCE_ENABLE_BIT) || \
        GH3X2X_CHECK_LEFT_BIT_SET(g_usDumpMode, GH3X2X_DUMP_GAIN_CURRENT_ENABLE_BIT))
    {
        //GH3X2X_RecordGainBgCancelValue();
        GH3X2X_RecordDrvCurrentValue();
    }
}
/**
 * @fn     void GH3X2X_RecordTiaGainInfo(void)
 *
 * @brief  record tia gain information
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_RecordTiaGainInfo(void)
{
    GH3X2X_Memcpy(g_puchGainBgCancelRecord, g_puchTiaGainAfterSoftAgc, sizeof(g_puchGainBgCancelRecord));

    GH3X2X_DEBUG_LOG_PARAM_WANPENG("RecordTiaGainInfo slot0: rx0 = %d, rx1 = %d, rx2 = %d, rx3 = %d\r\n" ,g_puchGainBgCancelRecord[0],g_puchGainBgCancelRecord[1],g_puchGainBgCancelRecord[2],g_puchGainBgCancelRecord[3]);    
    GH3X2X_DEBUG_LOG_PARAM_WANPENG("RecordTiaGainInfo slot1: rx0 = %d, rx1 = %d, rx2 = %d, rx3 = %d\r\n" ,g_puchGainBgCancelRecord[4],g_puchGainBgCancelRecord[5],g_puchGainBgCancelRecord[6],g_puchGainBgCancelRecord[7]);
}



#if GH3X2X_DUMP_MODE_EN 

/// dump data cnt map @ref EMGh3x2xDumpRawdataBgLvlSelect
const GU8 g_uchDumpDataCntMapArr[] = 
{
    2, // lvl 0
    3, // lvl 1
    4, // lvl 2
    2  // lvl 2x2  /* regard the second slot of 2x2 as a separate slot*/
};

/// div num of dump data map @ref EMGh3x2xDumpRawdataBgLvlSelect
const GU8 g_uchRawdataDivOfDumpDataMapArr[] = 
{
    1, // lvl 0
    1, // lvl 1
    2, // lvl 2
    2  // lvl 2x2
};

/// rawdata index of dump data map @ref EMGh3x2xDumpRawdataBgLvlSelect
const GU8 g_uchRawdataIndexOfDumpDataMapArr[] = 
{
    1, // lvl 0
    2, // lvl 1
    2, // lvl 2
    0  // lvl 2x2
};



/// bg level
GU16 g_usBgLevel;

/// hbd rawdata incomplete array
GU8 g_uchHbaIncompleteArr[GH3X2X_HBA_CHANNEL_MAP_MAX_CNT];

/// hbd dumpdata incomplete buffer
GU32 g_unHbaIncompleteDumpdataArr[GH3X2X_HBA_CHANNEL_MAP_MAX_CNT][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL] = {{0}};

/// hrv rawdata incomplete array
GU8 g_uchHrvIncompleteArr[GH3X2X_HRV_CHANNEL_MAP_MAX_CNT];

/// hrv dumpdata incomplete buffer
GU32 g_unHrvIncompleteDumpdataArr[GH3X2X_HRV_CHANNEL_MAP_MAX_CNT][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL] = {{0}};

/// spo2 rawdata incomplete array
GU8 g_uchSpo2IncompleteArr[GH3X2X_SPO2_CHANNEL_MAP_MAX_CNT];

/// spo2 dumpdata incomplete buffer
GU32 g_unSpo2IncompleteDumpdataArr[GH3X2X_SPO2_CHANNEL_MAP_MAX_CNT][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL] = {{0}};

/// dump data after filter from rawdata which will be uploaded
GU32 g_punDumpDataUploadArr[GH3X2X_MAX_DUMP_DATA_NUM_UPLOAD] = {0};



/// incomplete rawdata flag array for filter data
GU8 g_uchDumpIncompleteFlagArr[GH3X2X_CHANNEL_MAP_MAX_CH];

/// incomplete rawdata buffer for filter data
GU32 g_unDumpIncompleteRawdataArr[GH3X2X_CHANNEL_MAP_MAX_CH][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL] = {{0}};

/// save rawdata after filter
GU8 g_uchRawdataFilterArr[RAWDATA_FILTER_ARRAY_BYTE_LENGTH];

/// bg level of every channel
GU8 g_uchChannelBgLvlArr[GH3X2X_CHANNEL_MAP_MAX_CH];

/// save slot which used for ECG
GU8 g_uchEcgSlot;

// Zipdumpmode Tempdata
STZipLastDataTemp g_stZipDumpTempData;

//  Zip Package Num Even/Odd Flag
GU8 g_uchDumpOddEvenChangeFlag = 1;






/**
 * @fn     void GH3X2X_ReadElectrodeWearDumpData(void)
 *
 * @brief  read eletrode wear status
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  dump mode
 */
 /*
void GH3X2X_ReadElectrodeWearDumpData(void)
{
    if (GH3X2X_CHECK_LEFT_BIT_SET(g_usDumpMode, GH3X2X_DUMP_ELECTR_WEAR_ENABLE_BIT))
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        g_uchElectrodeWearStatus = GH3X2X_ReadRegBitField(GH3X2X_INT_STR2_REG_ADDR, \
                                    GH3X2X_ELECTR_WEAR_STATUS_BIT_FIELD, GH3X2X_ELECTR_WEAR_STATUS_BIT_FIELD);
        GH3X2X_WAIT_CHIP_DSLP();
    }
}
*/





/**
 * @fn     void GH3X2X_BgLevelDecode(GU8 puchBgLvArr[], GU8 uchChannelCnt, GU8 *puchChannelMapArray, GU16 usBgLevel)
 *
 * @brief  decode function's bg level array according to channel map and slot bg level
 *
 * @attention   if there is 2x2 bg level,set the first slot to level 2
 *
 * @param[in]   uchChannelCnt                channel cnt
 * @param[in]   puchChannelMapArray          channel map array
 * @param[in]   usBgLevel                    bg level of slot
 * @param[out]  puchBgLvArray                bg level array
 *
 * @return  None
 */
void GH3X2X_BgLevelDecode(GU8 puchBgLvArr[], GU8 uchChannelCnt, GU8 *puchChannelMapArray, GU16 usBgLevel)
{
    GU8 uchChCnt      = 0;
    GU8 uchSlotCnt    = 0;
    GU8 uchBgLv2x2Cnt = 0;
    GU8 uchSlotNum    = 0;
    GU16 usBgLvTemp = usBgLevel;

    //if BgLevel has level 2x2,change the first slot of 2x2 slot to level 2
    for (uchSlotCnt = 0; uchSlotCnt < GH3X2X_MAX_SLOT_NUM; uchSlotCnt++)
    {
        if (GH3X2X_DUMP_RAWDATA_BGLVL2X2 == ((usBgLevel >> (uchSlotCnt << 1)) & GH3X2X_SLOT_BG_LEVEL_BIT_MASK))
        {
            if (0 == uchBgLv2x2Cnt)
            {
                uchBgLv2x2Cnt++;
            }
            else
            {
                //set last slot bg level as GH3X2X_DUMP_RAWDATA_BGLVL2
                usBgLvTemp &= ~(GH3X2X_SLOT_BG_LEVEL_BIT_MASK << ((uchSlotCnt - 1) << 1));
                usBgLvTemp |= GH3X2X_DUMP_RAWDATA_BGLVL2 << ((uchSlotCnt - 1) << 1);
                uchBgLv2x2Cnt = 0;
            }                
        }
        else
        {
            uchBgLv2x2Cnt = 0;
        }
    }
    
    for (uchChCnt = 0; uchChCnt < uchChannelCnt; uchChCnt++)
    {
        uchSlotNum = GH3X2X_BYTE_RAWDATA_GET_SLOT_NUM(puchChannelMapArray[uchChCnt]);
        puchBgLvArr[uchChCnt] = (usBgLvTemp >> (uchSlotNum << 1)) & GH3X2X_SLOT_BG_LEVEL_BIT_MASK;
    }
}

/**
 * @fn     void GH3X2X_GetChannelBgLevel(GU16 usBgLevel)
 *
 * @brief  get bg level of every channel
 *
 * @attention   if there is two 2x2 bg level,set the first 2x2 slot to bg level 2
 *
 * @param[in]   usBgLevel                    slot bg level
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_GetChannelBgLevel(GU16 usBgLevel)
{
    GU8 uchSlotCnt           = 0;    
    GU8 uchChBgLvl           = 0;
    GU8 uchBhLvl2x2FirstSlot = 1;

    GH3X2X_Memset(g_uchChannelBgLvlArr, 0, sizeof(g_uchChannelBgLvlArr[0]) * GH3X2X_CHANNEL_MAP_MAX_CH);
    
    for (uchSlotCnt = 0; uchSlotCnt < GH3X2X_MAX_SLOT_NUM; ++uchSlotCnt)
    {
        uchChBgLvl = (usBgLevel >> (uchSlotCnt << 1)) & GH3X2X_SLOT_BG_LEVEL_BIT_MASK;
        if (GH3X2X_DUMP_RAWDATA_BGLVL2X2 == uchChBgLvl)
        {
            if (uchBhLvl2x2FirstSlot)
            {
                uchChBgLvl = GH3X2X_DUMP_RAWDATA_BGLVL2;
                uchBhLvl2x2FirstSlot = 0;
            }
            else
            {
                uchBhLvl2x2FirstSlot = 1;
            }
        }
        else
        {
            if (0 == uchBhLvl2x2FirstSlot)
            {
                GH3X2X_DEBUG_LOG_PARAM("%s 2x2 Bg Level set error.\r\n", __FUNCTION__);
                uchBhLvl2x2FirstSlot = 1;
            }            
        }

        GH3X2X_Memset(&g_uchChannelBgLvlArr[uchSlotCnt * GH3X2X_ADC_NUM], uchChBgLvl, \
                    sizeof(g_uchChannelBgLvlArr[0]) * GH3X2X_ADC_NUM);
    }
}

/**
 * @fn     void GH3X2X_BgLevelSet(GU16 usRegVal)
 *
 * @brief  set bg level of every slot
 *
 * @attention   None
 *
 * @param[in]   usRegVal          reg value
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_BgLevelSet(GU16 usRegVal)
{
    g_usBgLevel = usRegVal;
}

/**
 * @fn     void GH3X2X_GetEcgSlot(void)
 *
 * @brief  get slot used for ecg
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_GetEcgSlot(void)
{
    GU16 usEcgEnRegData  = 0;
    GU16 usEcgEnRegAddr  = 0;
    GU8  uchSlotIndexCnt = 0;

    GH3X2X_WAIT_CHIP_WAKEUP();
    g_uchEcgSlot = 0;
    usEcgEnRegAddr = GH3X2X_SLOT0_CTRL_0_REG_ADDR;    
    for (uchSlotIndexCnt = 0; uchSlotIndexCnt < GH3X2X_MAX_SLOT_NUM; ++uchSlotIndexCnt)
    {
        usEcgEnRegData = GH3X2X_ReadReg(usEcgEnRegAddr);
        if (GH3X2X_CHECK_BIT_SET(usEcgEnRegData, GH3X2X_ECG_ENABLE_BIT))
        {
            GH3X2X_VAL_SET_BIT(g_uchEcgSlot, (GU16)GH3X2X_GET_LEFT_SHIFT_VAL(uchSlotIndexCnt));
        }
        usEcgEnRegAddr += SLOT_ADDR_OFFSET;
    }
    GH3X2X_WAIT_CHIP_DSLP();
}

/**
 * @fn     void GH3X2X_DumpInit(void)
 *
 * @brief  init dump moudle
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_DumpInit(void)
{
    //if dump mode is not enable,just return
    if (GH3X2X_CHECK_LEFT_BIT_NOT_SET(g_usDumpMode, GH3X2X_DUMP_GAIN_CURRENT_ENABLE_BIT) && \
        GH3X2X_CHECK_LEFT_BIT_NOT_SET(g_usDumpMode, GH3X2X_DUMP_AMBIANCE_ENABLE_BIT))
    {
        return;
    }

    //when dump mode and agc are both enabled,should do this  
    if (GH3X2X_LED_AGC_ENABLE == GH3X2X_GetLedAgcState())
    {
        GH3X2X_Memset(g_uchDumpIncompleteFlagArr, 0, \
                        sizeof(g_uchDumpIncompleteFlagArr[0]) * GH3X2X_CHANNEL_MAP_MAX_CH);
        GH3X2X_GetChannelBgLevel(g_usBgLevel);
        GH3X2X_GetEcgSlot();
    }
}

/**
 * @fn     GU8 GH3X2X_FindLastSlotChMapIndex(GU8* puchTargetChMapIndex, GU8 puchChMapArr[], 
                                                 GU8 uchChMapCnt,GU8 uchChMapIndex)
 *
 * @brief  Find target channel map index which slot id minus 1 than input channel map
 *
 * @attention   None
 *
 * @param[out]  puchTargetChMapIndex            target channel map index
 * @param[in]   puchChMapArr[]                  channel map array
 * @param[in]   uchChMapCnt                     channel map cnt
 * @param[in]   uchChCntIndex                   channel cnt index
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return param error
 */
GS8 GH3X2X_FindLastSlotChMapIndex(GU8* puchTargetChMapIndex, GU8 puchChMapArr[], GU8 uchChMapCnt, GU8 uchChCntIndex)
{
    GU8 uchSlotNum;
    GU8 uchSlotNumTemp;
    GU8 uchAdcNum;
    GU8 uchAdcNumTemp;
    GU8 i;
    GS8 chRet = GH3X2X_RET_OK;

    if (GH3X2X_PTR_NULL == puchTargetChMapIndex)
    {
        return GH3X2X_RET_PARAMETER_ERROR;
    }
    
    uchSlotNum = GH3X2X_BYTE_RAWDATA_GET_SLOT_NUM(puchChMapArr[uchChCntIndex]);
    uchAdcNum  = GH3X2X_BYTE_RAWDATA_GET_ADC_NUM(puchChMapArr[uchChCntIndex]);
    
    for (i = 0; i < uchChMapCnt; i++)
    {
        uchSlotNumTemp = GH3X2X_BYTE_RAWDATA_GET_SLOT_NUM(puchChMapArr[i]);
        uchAdcNumTemp  = GH3X2X_BYTE_RAWDATA_GET_ADC_NUM(puchChMapArr[i]);
        if ((uchSlotNumTemp == (uchSlotNum - 1)) && (uchAdcNumTemp == uchAdcNum))
        {
            *puchTargetChMapIndex = i;
            break;
        }
    }

    if (i >= uchChMapCnt)
    {
        *puchTargetChMapIndex = uchChCntIndex;
        GH3X2X_DEBUG_LOG("Find channel map error!!!can not find last slot channel map\r\n");
        chRet = GH3X2X_RET_PARAMETER_ERROR;
    }

    return chRet;
}

/**
 * @fn     void GH3X2X_CacheIncompleteDumpdata(GU8 punLastIncompleteArr[], 
 *                                   GU32 punIncompleteDumpdataArr[][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL],
 *                                   GU8  uchNextIncompleteFlag, GU8 *puchFifoRawdata, GU16 usFifoRawdataLen,
 *                                   GU8 uchChannelMapArr[], GU8 uchChannelMapCnt,
 *                                   GU16 usRawdataByteIndexArr[GH3X2X_CHANNEL_MAP_MAX_CH])
 *
 * @brief  cache incomplete rawdata packet for algorithm
 *
 * @attention   None
 *
 * @param[out]  puchIncompleteFlagArr       incomplete flag array of every channel
 * @param[out]  punIncompleteDumpdataArr    incomplete data array
 * @param[in]   uchNextIncompleteFlag       incomplete flag to indicate if cache incomplete data
 * @param[in]   puchFifoRawdata             fifo rawdata
 * @param[in]   usFifoRawdataLen            fifo data length
 * @param[in]   uchChannelMapArr            channel map array
 * @param[in]   uchChannelMapCnt            channel map array cnt
 * @param[in]   usRawdataByteIndexArr       start index of every channel to search data
 *
 * @return  None
 */
void GH3X2X_CacheIncompleteDumpdata(GU8 puchIncompleteFlagArr[], 
                            GU32 punIncompleteDumpdataArr[][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL],
                            GU8  uchNextIncompleteFlag, GU8 *puchFifoRawdata, GU16 usFifoRawdataLen,
                            GU8 uchChannelMapArr[], GU8 uchChannelMapCnt,
                            GU16 usRawdataByteIndexArr[GH3X2X_CHANNEL_MAP_MAX_CH])
{
    GU16 usRawdataByteIndexTmp = 0;
    GU8  uchChCntIndex         = 0;
    GU8  uchDumpDataIndexArr[GH3X2X_CHANNEL_MAP_MAX_CH] = {0};
    
    GH3X2X_Memset(puchIncompleteFlagArr, 0, sizeof(puchIncompleteFlagArr[0]) * GH3X2X_CHANNEL_MAP_MAX_CH);
    
    if (uchNextIncompleteFlag != 0)
    {
        for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++) // search each channel
        {
            usRawdataByteIndexTmp = usRawdataByteIndexArr[uchChCntIndex];
            while (usRawdataByteIndexTmp < usFifoRawdataLen) // search each
            {
                if (GH3X2X_CHANNEL_MAP_GET(puchFifoRawdata[usRawdataByteIndexTmp]) == \
                                                            uchChannelMapArr[uchChCntIndex]) // if map equal
                {
                    GH3X2X_Memcpy(&punIncompleteDumpdataArr[uchChCntIndex][uchDumpDataIndexArr[uchChCntIndex]],
                                    &puchFifoRawdata[usRawdataByteIndexTmp], GH3X2X_FIFO_RAWDATA_SIZE);
                    puchIncompleteFlagArr[uchChCntIndex] |= \
                                        (GU8)GH3X2X_GET_LEFT_SHIFT_VAL(uchDumpDataIndexArr[uchChCntIndex]);
                    uchDumpDataIndexArr[uchChCntIndex]++;
                }
                usRawdataByteIndexTmp += GH3X2X_FIFO_RAWDATA_SIZE;
            }
        }
    }
}

/**
 * @fn     GS8 GH3X2X_GetDumpDataInfo(GF32 *pfGsFloatIncVal, GU16 *pusChannelDataCnt,
 *                                GU8 *puchNextIncompleteFlag,GU8 *puchDumpDataIndexArr,
 *                                GU8 *puchIncompleteFlagArr, GU16 usFifoLength, GU8 *puchDataBuffer,
 *                                GU8 uchChannelCnt, GU8 *puchChannelMapArray, GU8 *puchChannelBgLvlArr, 
 *                                GU16 usReadGsensorArrCnt)
 *
 * @brief  Get information from rawdata
 *
 * @attention   None
 *
 * @param[out]  pfGsFloatIncVal             pointer to gsensor inc value output
 * @param[out]  pusChannelDataCnt           pointer to slot data cnt output
 * @param[out]  puchNextIncompleteFlag      pointer to next incomplete flag arr output
 * @param[out]  puchDumpDataIndexArr        pointer to dump data index output
 * @param[in]   puchIncompleteFlagArr       incomplete flag array of last rawdata frame
 * @param[in]   usFifoLength                read fifo buffer data len
 * @param[in]   puchDataBuffer              pointer to read fifo buffer
 * @param[in]   uchChannelCnt               channel map arr cnt
 * @param[in]   puchChannelMapArray         pointer to channel map arr
 * @param[in]   puchChannelBgLvlArr         pointer to bg lvl arr
 * @param[in]   usReadGsensorArrCnt         read gsensor array cnt
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR    generic error when size is 0
 * @retval  #GH3X2X_RET_RESOURCE_ERROR   resource error when doesn't correspond to channel map
 */
GS8 GH3X2X_GetDumpDataInfo(GF32 *pfGsFloatIncVal, GU16 *pusChannelDataCnt,
                                GU8 *puchNextIncompleteFlag, GU8 *puchDumpDataIndexArr,
                                GU8 *puchIncompleteFlagArr, GU16 usFifoLength, GU8 *puchDataBuffer,
                                GU8 uchChannelCnt, GU8 *puchChannelMapArray, GU8 *puchChannelBgLvlArr, 
                                GU16 usReadGsensorArrCnt)
{
    GF32 fGsensorFloatIncTmp = GH3X2X_GF32_0;
    GU16 usRawdataCntArr[GH3X2X_CHANNEL_MAP_MAX_CH] = {0};
    GU16 usRawdataCntTmp    = 0;
    GU16 usRawdataCntMaxTmp = 0;
    GU16 usRawdataCntMinTmp = 0;
    GU16 usRawdataAllCntMax = 0;
    GU8  uchChCntIndexTmp   = 0;
    GU8  uchCalcCntIndexTmp = 0;
    GU8  uchDumpDataCalcTmp = 0;
    GU16 usRawdataIndexTmp  = 0;
    GU8  uchDumpDataCntCalcErrFlag = 0;
    GU8 uchNextIncompleteFlagTmp   = 0;
    GU16 usDumpDataCntArr[DUMP_DATA_MAX_NUM] = {0};
    GS8  chRet = GH3X2X_RET_GENERIC_ERROR;

    if ((pfGsFloatIncVal == GH3X2X_PTR_NULL) || (pusChannelDataCnt == GH3X2X_PTR_NULL)
        || (puchNextIncompleteFlag == GH3X2X_PTR_NULL) || (puchDumpDataIndexArr == GH3X2X_PTR_NULL)
        || (puchIncompleteFlagArr == GH3X2X_PTR_NULL) || (puchDataBuffer == GH3X2X_PTR_NULL)
        || (puchChannelMapArray == GH3X2X_PTR_NULL) || (puchChannelBgLvlArr == GH3X2X_PTR_NULL))
    {
        return GH3X2X_RET_PARAMETER_ERROR;
    }

    for (uchChCntIndexTmp = 0; uchChCntIndexTmp < uchChannelCnt; uchChCntIndexTmp++)
    {
        GH3X2X_Memset(usDumpDataCntArr, 0, sizeof(GU16) * DUMP_DATA_MAX_NUM);
        uchCalcCntIndexTmp = 0;
        uchDumpDataCalcTmp = g_uchDumpDataCntMapArr[puchChannelBgLvlArr[uchChCntIndexTmp]];
        for (usRawdataIndexTmp = 0; usRawdataIndexTmp < uchDumpDataCalcTmp; usRawdataIndexTmp ++)
        {
            if (GH3X2X_CHECK_LEFT_BIT_SET(puchIncompleteFlagArr[uchChCntIndexTmp], uchCalcCntIndexTmp))
            {
                usDumpDataCntArr[uchCalcCntIndexTmp]++;
                uchCalcCntIndexTmp++;
                puchDumpDataIndexArr[uchChCntIndexTmp]++; // calc start index for get data
            }
        }
        for (usRawdataIndexTmp = 0; usRawdataIndexTmp < usFifoLength; usRawdataIndexTmp += GH3X2X_FIFO_RAWDATA_SIZE)
        {
            if (puchChannelMapArray[uchChCntIndexTmp] == GH3X2X_CHANNEL_MAP_GET(puchDataBuffer[usRawdataIndexTmp]))
            {
                usDumpDataCntArr[uchCalcCntIndexTmp%uchDumpDataCalcTmp]++;
                uchCalcCntIndexTmp++;
            }
        }
        GH3X2X_FindGu16MaxMinVal(&usRawdataCntMaxTmp, &usRawdataCntMinTmp, usDumpDataCntArr, uchDumpDataCalcTmp);
        if (usRawdataAllCntMax < usRawdataCntMaxTmp)
        {
            usRawdataAllCntMax = usRawdataCntMaxTmp; // calc all data cnt max 
        }
        if (usRawdataCntMinTmp == usRawdataCntMaxTmp)
        {
            usRawdataCntArr[uchChCntIndexTmp] = usRawdataCntMinTmp;
        }
        else
        {
            for (usRawdataIndexTmp = 0; usRawdataIndexTmp < (uchDumpDataCalcTmp - 1); usRawdataIndexTmp ++) // checkcnt
            {
                if (usDumpDataCntArr[usRawdataIndexTmp] < usDumpDataCntArr[usRawdataIndexTmp + 1]) // if next>this, err
                {
                    uchDumpDataCntCalcErrFlag = 1;
                    break;
                }
            }
            if (uchDumpDataCntCalcErrFlag == 0)
            {
                usRawdataCntArr[uchChCntIndexTmp] = usRawdataCntMinTmp;
                uchNextIncompleteFlagTmp = 1;
            }
            else
            {
                GH3X2X_DEBUG_LOG("dump data of buffer error!\r\n");
                break;
            }
        }
    } // end of for (uchChCntIndexTmp = 0; uchChCntIndexTmp < uchChannelCnt; uchChCntIndexTmp++)
    if (uchDumpDataCntCalcErrFlag == 0)
    {
        GH3X2X_FindGu16MaxMinVal(&usRawdataCntMaxTmp, &usRawdataCntMinTmp, usRawdataCntArr, uchChannelCnt);
        if (usRawdataCntMaxTmp == usRawdataCntMinTmp)
        {
            usRawdataCntTmp = usRawdataCntMaxTmp; // whatever last data complete or incomplete
        }
        else
        {
            if (((usRawdataCntMaxTmp - usRawdataCntMinTmp) == 1) && (usRawdataAllCntMax == usRawdataCntMaxTmp))
            {
                usRawdataCntTmp = usRawdataCntMinTmp;
                uchNextIncompleteFlagTmp = 1;
            }
            else // cnt tolerable deviation is 1
            {
                GH3X2X_DEBUG_LOG("dump data doesn't correspond to channel map error!\r\n");
                usRawdataCntTmp = 0;
                uchNextIncompleteFlagTmp = 0;
                chRet = GH3X2X_RET_RESOURCE_ERROR;
            }
        } // end of if (usRawdataCntMaxTmp == usRawdataCntMinTmp)

        if (usRawdataCntTmp > 0)
        {
            chRet = GH3X2X_RET_OK;
            if ((usRawdataCntTmp > 1) && (usReadGsensorArrCnt > 0)) // calc gs index
            {
                fGsensorFloatIncTmp = ((GF32)(usReadGsensorArrCnt - 1)) / (usRawdataCntTmp - 1);
            }
        }
    } // end of if (uchDumpDataCntCalcErrFlag == 0)
    SET_VAL_VIA_PTR(pfGsFloatIncVal, fGsensorFloatIncTmp); // set gsensor increasing val
    SET_VAL_VIA_PTR(pusChannelDataCnt, usRawdataCntTmp); // channel data count
    SET_VAL_VIA_PTR(puchNextIncompleteFlag, uchNextIncompleteFlagTmp); // channel data count max
    return chRet;
}

/**
 * @fn     GS8 GH3X2X_UnpackDumpDataWithChannelMap(STGh3x2xChannelDumpData *pstGh3x2xChannelDumpData,
 *                    GU8 *puchReadRawdataBuffer, GU16 usReadRawdataLen, GU8 uchChannelMapCnt, GU8 *puchChannelMapArr,
 *                    GU8 *puchChannelBgLvlArr)
 *
 * @brief  Unpack to channel dump data from read fifo data buffer;
 *         if dump data incomplete, should change fifo watermark
 *
 * @attention   This function should use in get rawdata hook
 * 
 * @param[in]   puchReadRawdataBuffer           pointer to read data buffer
 * @param[in]   usReadRawdataLen                read data length
 * @param[in]   uchChannelMapCnt                channel map array cnt, max:32
 * @param[in]   puchChannelMapArr               pointer to channel map array
 * @param[in]   puchChannelBgLvlArr             pointer to channel lvl array, @ref EMGh3x2xDumpRawdataBgLvlSelect
 * @param[out]  pstGh3x2xChannelDumpData        pointer to channel rawdata struct
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return param error
 */
GS8 GH3X2X_UnpackDumpDataWithChannelMap(STGh3x2xChannelDumpData *pstGh3x2xChannelDumpData,
                    GU8 *puchReadRawdataBuffer, GU16 usReadRawdataLen, GU8 uchChannelMapCnt, GU8 *puchChannelMapArr,
                    GU8 *puchChannelBgLvlArr)
{
    GU8  uchChCntIndex = 0;
    GU16 usRawdataIndex = 0;
    GU16 usRawdataCnt = 0;
    GU16 usRawdataIndexArr[GH3X2X_CHANNEL_MAP_MAX_CH] = {0}; // use for rawdata cnt & rawdata bytes index
    GU16 usRawdataByteIndexTmp = 0;
    GU16 usRawdataBaseIndex = 0;
    GU8  uchCalcCntIndex = 0;
    GU8  uchDumpDataCalcTmp = 0;
    GU8 uchNextIncompleteFlag = 0;
    GU8 uchDumpDataIndexArr[GH3X2X_CHANNEL_MAP_MAX_CH] = {0};
    GU32 unRawdataSum = 0;
    GF32 fGsensorFloatIndexInc = GH3X2X_GF32_0;
    GU8 uchLastSlotChIndex = 0;
    GS8 chRet = GH3X2X_RET_OK;
    
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);

    /* check pram */
    if ((pstGh3x2xChannelDumpData->punIncompleteChRawdataArr == GH3X2X_PTR_NULL)
        || (pstGh3x2xChannelDumpData->punChBgRawdataArr == GH3X2X_PTR_NULL)
        || (pstGh3x2xChannelDumpData->punChRawdataArr == GH3X2X_PTR_NULL)
        || (pstGh3x2xChannelDumpData->punPkgNoAndCurrentArr == GH3X2X_PTR_NULL)
        || (puchReadRawdataBuffer == GH3X2X_PTR_NULL)
        || (puchChannelBgLvlArr == GH3X2X_PTR_NULL)
        || (pstGh3x2xChannelDumpData == GH3X2X_PTR_NULL) || (puchChannelMapArr == GH3X2X_PTR_NULL))
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
        if (puchChannelBgLvlArr[uchChCntIndex] > GH3X2X_DUMP_RAWDATA_BGLVL2X2)
        {
            GH3X2X_DEBUG_LOG("dump lvl error! set @ref EMGh3x2xDumpRawdataBgLvlSelect!\r\n");
            return GH3X2X_RET_PARAMETER_ERROR;
        }
    }
    if (uchChannelMapCnt > GH3X2X_CHANNEL_MAP_MAX_CH)
    {
        GH3X2X_DEBUG_LOG("channel cnt greater than max!\r\n");
        return GH3X2X_RET_PARAMETER_ERROR;
    }

    /* calc rawdata len */
    chRet = GH3X2X_GetDumpDataInfo(&fGsensorFloatIndexInc, &usRawdataCnt, &uchNextIncompleteFlag, uchDumpDataIndexArr, 
                        pstGh3x2xChannelDumpData->uchChIncompleteFlagArr, usReadRawdataLen, puchReadRawdataBuffer, 
                        uchChannelMapCnt, puchChannelMapArr, puchChannelBgLvlArr, 0);
    if (chRet == GH3X2X_RET_OK)
    {
        for (usRawdataIndex = 0; usRawdataIndex < usRawdataCnt; usRawdataIndex++)
        {
            usRawdataBaseIndex = usRawdataIndex * (GU16)uchChannelMapCnt;
            for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++) // search each channel
            {
                uchDumpDataCalcTmp = g_uchDumpDataCntMapArr[puchChannelBgLvlArr[uchChCntIndex]];
                if (uchDumpDataIndexArr[uchChCntIndex] < uchDumpDataCalcTmp)
                {
                    usRawdataByteIndexTmp = usRawdataIndexArr[uchChCntIndex];
                    while (usRawdataByteIndexTmp < usReadRawdataLen) // search each
                    {
                        if (GH3X2X_CHANNEL_MAP_GET(puchReadRawdataBuffer[usRawdataByteIndexTmp]) == \
                                                                    puchChannelMapArr[uchChCntIndex]) // if map equal
                        {
                            pstGh3x2xChannelDumpData->punIncompleteChRawdataArr[uchChCntIndex]\
                                                                            [uchDumpDataIndexArr[uchChCntIndex]] = \
                                                GH3X2X_RAWDATA_CLEAR_SLOT_ADC_NUM(
                                                    GH3X2X_MAKEUP_DWORD(puchReadRawdataBuffer[usRawdataByteIndexTmp],
                                                                    puchReadRawdataBuffer[usRawdataByteIndexTmp + 1],
                                                                    puchReadRawdataBuffer[usRawdataByteIndexTmp + 2],
                                                                    puchReadRawdataBuffer[usRawdataByteIndexTmp + 3]));
                            uchDumpDataIndexArr[uchChCntIndex]++;
                            if (uchDumpDataIndexArr[uchChCntIndex] >= uchDumpDataCalcTmp)
                            {
                                usRawdataByteIndexTmp += GH3X2X_FIFO_RAWDATA_SIZE;
                                break;
                            }
                        }
                        usRawdataByteIndexTmp += GH3X2X_FIFO_RAWDATA_SIZE;
                    }
                }                
                uchDumpDataIndexArr[uchChCntIndex] = 0;

                usRawdataIndexArr[uchChCntIndex] = usRawdataByteIndexTmp; // write back byte index
            } // end of for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)

            for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++) // process each channel dump data
            {
                if (GH3X2X_DUMP_RAWDATA_BGLVL2X2 != puchChannelBgLvlArr[uchChCntIndex])
                {
                    pstGh3x2xChannelDumpData->punPkgNoAndCurrentArr[usRawdataBaseIndex + uchChCntIndex] = \
                                        pstGh3x2xChannelDumpData->punIncompleteChRawdataArr[uchChCntIndex][0];
                    pstGh3x2xChannelDumpData->punChRawdataArr[usRawdataBaseIndex + uchChCntIndex] = \
                        pstGh3x2xChannelDumpData->punIncompleteChRawdataArr[uchChCntIndex]\
                                [g_uchRawdataIndexOfDumpDataMapArr[puchChannelBgLvlArr[uchChCntIndex]]];
                    unRawdataSum = 0;
                    for (uchCalcCntIndex = 1;
                        uchCalcCntIndex < g_uchDumpDataCntMapArr[puchChannelBgLvlArr[uchChCntIndex]]; 
                        uchCalcCntIndex++)
                    {
                        if (uchCalcCntIndex != g_uchRawdataIndexOfDumpDataMapArr[puchChannelBgLvlArr[uchChCntIndex]])
                        {
                            unRawdataSum += \
                                GH3X2X_RAWDATA_CLEAR_ALL_FLAG(\
                                  pstGh3x2xChannelDumpData->punIncompleteChRawdataArr[uchChCntIndex][uchCalcCntIndex]);
                        }
                    }
                    pstGh3x2xChannelDumpData->punChBgRawdataArr[usRawdataBaseIndex + uchChCntIndex] = \
                               (unRawdataSum / g_uchRawdataDivOfDumpDataMapArr[puchChannelBgLvlArr[uchChCntIndex]]) | \
                                    GH3X2X_RAWDATA_CLEAR_DATA(\
                                        pstGh3x2xChannelDumpData->punChRawdataArr[usRawdataBaseIndex + uchChCntIndex]);
                }
                else
                {
                    GH3X2X_FindLastSlotChMapIndex(&uchLastSlotChIndex, puchChannelMapArr, \
                                                    uchChannelMapCnt, uchChCntIndex);
                    pstGh3x2xChannelDumpData->punPkgNoAndCurrentArr[usRawdataBaseIndex + uchChCntIndex] = \
                                        pstGh3x2xChannelDumpData->punIncompleteChRawdataArr[uchLastSlotChIndex][0];
                    pstGh3x2xChannelDumpData->punChRawdataArr[usRawdataBaseIndex + uchChCntIndex] = \
                        pstGh3x2xChannelDumpData->punIncompleteChRawdataArr[uchChCntIndex][0];
                    unRawdataSum = pstGh3x2xChannelDumpData->punIncompleteChRawdataArr[uchLastSlotChIndex]\
                                [g_uchRawdataIndexOfDumpDataMapArr[puchChannelBgLvlArr[uchLastSlotChIndex]] + 1]
                                +  pstGh3x2xChannelDumpData->punIncompleteChRawdataArr[uchChCntIndex]\
                                [g_uchRawdataIndexOfDumpDataMapArr[puchChannelBgLvlArr[uchChCntIndex]] + 1];
                    pstGh3x2xChannelDumpData->punChBgRawdataArr[usRawdataBaseIndex + uchChCntIndex] = \
                               (unRawdataSum / g_uchRawdataDivOfDumpDataMapArr[puchChannelBgLvlArr[uchChCntIndex]]) | \
                                    GH3X2X_RAWDATA_CLEAR_DATA(\
                                        pstGh3x2xChannelDumpData->punChRawdataArr[usRawdataBaseIndex + uchChCntIndex]);
                }
            } // end of for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)
        } // end of for (usRawdataIndex = 0; usRawdataIndex < usRawdataCnt; usRawdataIndex++)
        GH3X2X_Memset(pstGh3x2xChannelDumpData->uchChIncompleteFlagArr, 0, GH3X2X_CHANNEL_MAP_MAX_CH);
    } // end of if (chRet == GH3X2X_RET_OK)
    if (uchNextIncompleteFlag != 0)
    {
        for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++) // search each channel
        {
            usRawdataByteIndexTmp = usRawdataIndexArr[uchChCntIndex];
            while (usRawdataByteIndexTmp < usReadRawdataLen) // search each
            {
                if (GH3X2X_CHANNEL_MAP_GET(puchReadRawdataBuffer[usRawdataByteIndexTmp]) == \
                                                            puchChannelMapArr[uchChCntIndex]) // if map equal
                {
                    pstGh3x2xChannelDumpData->punIncompleteChRawdataArr[uchChCntIndex]\
                                                                        [uchDumpDataIndexArr[uchChCntIndex]] = \
                                            GH3X2X_RAWDATA_CLEAR_SLOT_ADC_NUM(
                                                GH3X2X_MAKEUP_DWORD(puchReadRawdataBuffer[usRawdataByteIndexTmp],
                                                                puchReadRawdataBuffer[usRawdataByteIndexTmp + 1],
                                                                puchReadRawdataBuffer[usRawdataByteIndexTmp + 2],
                                                                puchReadRawdataBuffer[usRawdataByteIndexTmp + 3]));
                    pstGh3x2xChannelDumpData->uchChIncompleteFlagArr[uchChCntIndex] |= \
                                        (GU8)GH3X2X_GET_LEFT_SHIFT_VAL(uchDumpDataIndexArr[uchChCntIndex]);
                    uchDumpDataIndexArr[uchChCntIndex]++;
                }
                usRawdataByteIndexTmp += GH3X2X_FIFO_RAWDATA_SIZE;
            }
        }
    } // end of if (uchNextIncompleteFlag != 0)
    pstGh3x2xChannelDumpData->usChRawdataCnt = usRawdataCnt;
    return GH3X2X_RET_OK;
}

/**
 * @fn     GS16 GH3X2X_GetDumpdataWithChannelMap(STGh3x2xChannelDumpData *pstGh3x2xChannelDumpData, 
 *                       GU8 *puchReadRawdataBuffer, GU8 uchChannelMapCnt, GU8 *puchChannelMapArr, GU8 *puchBglvlArr)
 *
 * @brief  Get dump data from fifo with channel map
 *
 * @attention   None
 *
 * @param[in]   puchReadRawdataBuffer       pointer to read data buffer
 * @param[in]   uchChannelMapCnt            channel map array cnt, max:32
 * @param[in]   puchChannelMapArr           pointer to channel map array
 * @param[in]   puchBglvlArr                pointer to bg lvl array, @ref EMGh3x2xDumpRawdataBgLvlSelect
 * @param[out]  pstGh3x2xChannelDumpData    pointer to channel rawdata struct output
 *
 * @return  GH3X2X_RET_OK                       return get data successful
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return param error
 * @retval  #GH3X2X_RET_READ_FIFO_CONTINUE      return fifo is not empty
 */
GS16 GH3X2X_GetDumpdataWithChannelMap(STGh3x2xChannelDumpData *pstGh3x2xChannelDumpData, 
                        GU8 *puchReadRawdataBuffer, GU8 uchChannelMapCnt, GU8 *puchChannelMapArr, GU8 *puchBglvlArr)
{
    GU16 usFifoLength = 0;
    GS16 sRet = GH3X2X_RET_OK;

    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);

    if (puchReadRawdataBuffer != GH3X2X_PTR_NULL)
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        usFifoLength = GH3X2X_FIFO_CNT_CALC(GH3X2X_ReadReg(GH3X2X_INT_FIFO_UR_REG_ADDR));
        if (usFifoLength > g_usMaxNumReadFromFifo)
        {
            usFifoLength = g_usMaxNumReadFromFifo;
            sRet = GH3X2X_RET_READ_FIFO_CONTINUE;
        }

        if ((usFifoLength > 0) && (usFifoLength <= GH3X2X_FIFO_DATA_BYTES_MAX_LEN))
        {
            GH3X2X_ReadFifo(puchReadRawdataBuffer, usFifoLength);
        }
        else
        {
            if (usFifoLength > GH3X2X_FIFO_DATA_BYTES_MAX_LEN)
            {
                usFifoLength = 0;
                GH3X2X_DEBUG_LOG("get rawdata fifo len greater than max, pelease check i2c/spi!\r\n");
            }
            else
            {
                GH3X2X_DEBUG_LOG("get rawdata fifo equl 0!\r\n");
            }
        }
        
        /* call hook */
        HOOK_FUNC_CALL(g_pGh3x2xGetRawdataHookFunc, (puchReadRawdataBuffer, usFifoLength));
        GH3X2X_WAIT_CHIP_DSLP();
        sRet = GH3X2X_UnpackDumpDataWithChannelMap(pstGh3x2xChannelDumpData, puchReadRawdataBuffer, usFifoLength, 
                                                    uchChannelMapCnt, puchChannelMapArr, puchBglvlArr);
    }
    else // fixed clear cnt if readbuffer ptr is null
    {
        GH3X2X_DEBUG_LOG("get rawdata error that readbuffer is null!\r\n");
        sRet = GH3X2X_RET_PARAMETER_ERROR;
    }

    return sRet;
}

/**
 * @fn     GS8 GH3X2X_ChannelMapDumpDataClear(STGh3x2xChannelDumpData *pstGh3x2xChannelDumpData)
 *
 * @brief  clear channel map dump data struct
 *
 * @attention   This function must use before get dump data by one kind channel map
 *              e.g. channel map start/stop need clear
 *
 * @param[in]   pstGh3x2xChannelDumpData     pointer to channel dump data struct
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return param error
 */
GS8 GH3X2X_ChannelMapDumpDataClear(STGh3x2xChannelDumpData *pstGh3x2xChannelDumpData)
{
    GS8 chRet = GH3X2X_RET_PARAMETER_ERROR;

    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if ((pstGh3x2xChannelDumpData->uchChIncompleteFlagArr != GH3X2X_PTR_NULL)
        && (pstGh3x2xChannelDumpData != GH3X2X_PTR_NULL))
    {
        GH3X2X_Memset(pstGh3x2xChannelDumpData->uchChIncompleteFlagArr, 0, GH3X2X_CHANNEL_MAP_MAX_CH);
        pstGh3x2xChannelDumpData->usChRawdataCnt = 0;
        chRet = GH3X2X_RET_OK;
    }
    else
    {
        GH3X2X_DEBUG_LOG("channel rawdata param is null!\r\n");
    }
    return chRet;
}

/**
 * @fn     GU32 GH3X2X_DumpRawDataPro(GU32 punRawdata, GU32 unAmb1, GU32 unAmb2)
 *
 * @brief  transfer dump data to true rawdata
 *
 * @attention   None
 *
 * @param[in]   punRawdata          led data
 * @param[in]   unAmb1              ambiance data1
 * @param[in]   unAmb2              ambiance data2
 * @param[out]  None
 *
 * @return  rawdata after process
 */
GU32 GH3X2X_DumpRawDataPro(GU32 punRawdata, GU32 unAmb1, GU32 unAmb2)
{
    GU32 unLedValue  = 0;
    
    unLedValue = (GU32)((GS32)DUMP_PPG_DATA(punRawdata) - (((GS32)DUMP_PPG_DATA(unAmb1) + \
                        (GS32)DUMP_PPG_DATA(unAmb2)) >> 1) + GH3X2X_DUMP_DATA_OFFSET);
    
    return GH3X2X_MAKEUP_DWORD(GH3X2X_GET_BYTE0_FROM_DWORD(unLedValue), GH3X2X_GET_BYTE1_FROM_DWORD(unLedValue), \
                                GH3X2X_GET_BYTE2_FROM_DWORD(unLedValue), GH3X2X_GET_BYTE0_FROM_DWORD(punRawdata));
    
}

/**
 * @fn     void GH3X2X_DumpDataPro(GU32 punDumpDataArr[][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL], GU8 puchChMapArr[], 
                                    GU8 uchChMapCnt, GU8 puchChBgLvlArr[], GU8 uchChCntIndex)
 *
 * @brief  transfer dump data to true rawdata
 *
 * @attention   None
 *
 * @param[in]   punDumpData[][]     dump data of all channel
 * @param[in]   uchChMapArr[]       channel map array
 * @param[in]   uchChMapCnt         channel map cnt
 * @param[in]   puchChBgLvlArr[]    channel bg level array
 * @param[in]   uchChCntIndex       channel index
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_DumpDataPro(GU32 punDumpDataArr[][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL], GU8 puchChMapArr[], 
                        GU8 uchChMapCnt, GU8 puchChBgLvlArr[], GU8 uchChCntIndex)
{
    GU8 emBgLevel = puchChBgLvlArr[uchChCntIndex];
    GU8 uchLastIndex = 0;
    GU8 uchAgcData = 0;

    uchAgcData = g_puchGainBgCancelRecord[GH3X2X_BYTE_RAWDATA_GET_SLOT_ADC_NUM(puchChMapArr[uchChCntIndex])];
    switch (emBgLevel)
    {
    case GH3X2X_DUMP_RAWDATA_BGLVL0:
        g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_PPG_INDEX]          = punDumpDataArr[uchChCntIndex][1];
        g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_GAIN_CURRENT_INDEX] = GH3X2X_MAKEUP_DWORD(0, \
                                                                    DUMP_CUREENT1(punDumpDataArr[uchChCntIndex][0]), \
                                                                    DUMP_CUREENT0(punDumpDataArr[uchChCntIndex][0]), \
                                                                    uchAgcData);
        g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_AMBIANCE_INDEX]     = 0;
        break;
    case GH3X2X_DUMP_RAWDATA_BGLVL1:
        g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_PPG_INDEX] = GH3X2X_DumpRawDataPro(punDumpDataArr[uchChCntIndex][2], \
                                                                    punDumpDataArr[uchChCntIndex][1], \
                                                                    punDumpDataArr[uchChCntIndex][1]);
        g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_GAIN_CURRENT_INDEX] = GH3X2X_MAKEUP_DWORD(0, \
                                                                    DUMP_CUREENT1(punDumpDataArr[uchChCntIndex][0]), \
                                                                    DUMP_CUREENT0(punDumpDataArr[uchChCntIndex][0]), \
                                                                    uchAgcData);
        g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_AMBIANCE_INDEX]     = DUMP_PPG_DATA(punDumpDataArr[uchChCntIndex][1]);
        break;
    case GH3X2X_DUMP_RAWDATA_BGLVL2:
        g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_PPG_INDEX] = GH3X2X_DumpRawDataPro(punDumpDataArr[uchChCntIndex][2], \
                                                                    punDumpDataArr[uchChCntIndex][1], \
                                                                    punDumpDataArr[uchChCntIndex][3]);
        g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_GAIN_CURRENT_INDEX] = GH3X2X_MAKEUP_DWORD(0, \
                                                                    DUMP_CUREENT1(punDumpDataArr[uchChCntIndex][0]), \
                                                                    DUMP_CUREENT0(punDumpDataArr[uchChCntIndex][0]), \
                                                                    uchAgcData);
        g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_AMBIANCE_INDEX] = (DUMP_PPG_DATA(punDumpDataArr[uchChCntIndex][1]) + \
                                                                DUMP_PPG_DATA(punDumpDataArr[uchChCntIndex][3])) >> 1;
        break;
    case GH3X2X_DUMP_RAWDATA_BGLVL2X2:
        GH3X2X_FindLastSlotChMapIndex(&uchLastIndex, puchChMapArr, uchChMapCnt, uchChCntIndex);

        g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_PPG_INDEX] = GH3X2X_DumpRawDataPro(punDumpDataArr[uchChCntIndex][0], \
                                                                    punDumpDataArr[uchLastIndex][3], \
                                                                    punDumpDataArr[uchChCntIndex][1]);
        g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_GAIN_CURRENT_INDEX] = GH3X2X_MAKEUP_DWORD(0, \
                                                                    DUMP_CUREENT1(punDumpDataArr[uchLastIndex][0]), \
                                                                    DUMP_CUREENT0(punDumpDataArr[uchLastIndex][0]), \
                                                                    uchAgcData);
        g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_AMBIANCE_INDEX] = (DUMP_PPG_DATA(punDumpDataArr[uchLastIndex][3]) + \
                                                                DUMP_PPG_DATA(punDumpDataArr[uchChCntIndex][1])) >> 1;
        break;
    default:
        break;
    } // end of switch (emBgLevel)
}

/**
 * @fn       void GH3X2X_ElectrWearDumpDataPro(GU8* puchRawdataTag, GU8 uchChMapIndex)
 *
 * @brief    electrode wear dump data pro
 *
 * @attention    None
 *
 * @param[in]    puchRawdataTag      data tag of rawdata
 * @param[in]    uchChMapIndex       channel map index
 * @param[out]   None
 *
 * @return       None
 */
 /*
void GH3X2X_ElectrWearDumpDataPro(GU8* puchRawdataTag, GU8 uchChMapIndex)
{
    if (GH3X2X_CHECK_LEFT_BIT_SET(GH3X2X_DumpModeGet(), GH3X2X_DUMP_ELECTR_WEAR_ENABLE_BIT))
    {
        if (0 == uchChMapIndex)
        {
            if (0 == g_uchElectrodeWearStatus)
            {
                GH3X2X_VAL_CLEAR_BIT(*puchRawdataTag, (GU8)GH3X2X_ELECTR_WEAR_STATUS_BIT_IN_DATA_TAG);
            }
            else
            {
                GH3X2X_VAL_SET_BIT(*puchRawdataTag, GH3X2X_ELECTR_WEAR_STATUS_BIT_IN_DATA_TAG);
            }
        }
    }
}
*/

/**
 * @fn       void GH3X2X_FillDumpData(GU16* usPayloadIndex, GU8* uchGainCurAddr, GU8* uchAmbiAddr, 
 *                                    GU8 uchChannelMapArr[], GU8 uchChannelMapCnt,GU8 *puchChannelBgLvlArr, 
 *                                    GU16 usDumpMode, GU32 punIncompleteDumpdataArr[][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL])
 *
 * @brief  fill dump data to protocol data
 *
 * @attention    None
 *
 * @param[in]    uchChMapArr[]        channel map array
 * @param[in]    uchChMapCnt          channel map cnt
 * @param[in]    puchChBgLvlArr[]     channel bg level array
 * @param[in]    usDumpMode           dump mode
 * @param[in]    punDumpData[][]      dump data of all channel
 * @param[out]    usPayloadIndex      current payload index
 * @param[out]    uchGainCurAddr      current payload index
 * @param[out]    uchAmbiAddr         current payload index
 *
 * @return    None
 */
void GH3X2X_FillDumpData(GU16* usPayloadIndex, GU8* uchGainCurAddr, GU8* uchAmbiAddr, GU8 uchChannelMapArr[], 
                            GU8 uchChannelMapCnt, GU8 *puchChannelBgLvlArr, GU16 usDumpMode, 
                            GU32 punIncompleteDumpdataArr[][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL])
{
    GU8  uchChCntIndex = 0;

    for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)
    {
        //dump data filter process
        GH3X2X_DumpDataPro(punIncompleteDumpdataArr, uchChannelMapArr, uchChannelMapCnt, \
                            puchChannelBgLvlArr, uchChCntIndex);
        
        GH3X2X_Memcpy(&g_pstGh3x2xProtocolData->puchPacketPayloadArr[*usPayloadIndex], \
                        &g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_PPG_INDEX], GH3X2X_FIFO_RAWDATA_SIZE);
        GH3X2X_ElectrWearDumpDataPro(&g_pstGh3x2xProtocolData->puchPacketPayloadArr[*usPayloadIndex], uchChCntIndex);
        *usPayloadIndex += GH3X2X_FIFO_RAWDATA_SIZE;
        
        if (GH3X2X_CHECK_LEFT_BIT_SET(usDumpMode, GH3X2X_DUMP_GAIN_CURRENT_ENABLE_BIT))
        {
            GH3X2X_Memcpy(&g_pstGh3x2xProtocolData->puchPacketPayloadArr[*uchGainCurAddr], \
                            &g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_GAIN_CURRENT_INDEX], \
                            GH3X2X_DUMP_GAIN_CURRENT_DATA_SIZE);
            *uchGainCurAddr += GH3X2X_DUMP_GAIN_CURRENT_DATA_SIZE;
        }
        
        if (GH3X2X_CHECK_LEFT_BIT_SET(usDumpMode, GH3X2X_DUMP_AMBIANCE_ENABLE_BIT))
        {
            GH3X2X_Memcpy(&g_pstGh3x2xProtocolData->puchPacketPayloadArr[*uchAmbiAddr], \
                            &g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_AMBIANCE_INDEX], \
                            GH3X2X_DUMP_AMBIANCE_DATA_SIZE);
            *uchAmbiAddr += GH3X2X_DUMP_AMBIANCE_DATA_SIZE;
        }
    } //end of for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)
}

#if 0
/**
 * @fn    GS8 GH3X2X_SendDumpDataPkgWithAlgo(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 punLastIncompleteArr[], 
                        GU32 punIncompleteDumpdataArr[][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL], GU8 *puchFifoRawdata, 
                        GU16 usFifoRawdataLen, GU8 uchChannelMapArr[], GU8 uchChannelMapCnt, GU8 *puchChannelBgLvlArr, 
                        STGsensorRawdata *pstGsAxisValueArr, GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity, 
                        GS32 nAlgoCalcResultArr[], GU8 uchAlgoResNum, GU8 uchChannelMapId)
 *
 * @brief  Pack rawdata with g sensor data and algorithm data
 *
 * @attention   None
 *
 * @param[out]  puchRespondBuffer       pointer to protocol pkg data.
 * @param[out]  pusRespondLen           pointer to protocol pkg data length
 * @param[out]  punLastIncompleteArr    incomplete array
 * @param[out]  punIncompleteRawdataArr incomplete data array used for cache data
 * @param[in]   puchFifoRawdata         pointer to data buffer of rawdata
 * @param[in]   usFifoRawdataLen        data num of rawdata buf
 * @param[in]   uchChannelMapArr        pointer to channel map array
 * @param[in]   uchChannelMapCnt        channel map array cnt, max:32
 * @param[in]   puchChannelBgLvlArr     bg level array
 * @param[in]   pstGsAxisValueArr       pointer to g sensor data
 * @param[in]   usGsDataNum             g sensor data number
 * @param[in]   emGsSensitivity         g sensor sensitivity
 * @param[in]   nAlgoCalcResultArr      algorithm result
 * @param[in]   uchAlgoResNum           number of algorithm result
 * @param[in]   uchChannelMapId         channel map id (0 ~ 15)
 * @return  error code
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return param error
 * @retval  #GH3X2X_RET_RAW_DATA_PKG_OVER       return already read all the package data 
 * @retval  #GH3X2X_RET_RAW_DATA_PKG_CONTINUE   return should read rawdata package data again
 * @retval  #GH3X2X_RET_RESOURCE_ERROR          return not enough data
 */
GS8 GH3X2X_SendDumpDataPkgWithAlgo(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 punLastIncompleteArr[], 
                        GU32 punIncompleteDumpdataArr[][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL], GU8 *puchFifoRawdata, 
                        GU16 usFifoRawdataLen, GU8 uchChannelMapArr[], GU8 uchChannelMapCnt, GU8 *puchChannelBgLvlArr, 
                        STGsensorRawdata *pstGsAxisValueArr, GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity, 
                        GS32 nAlgoCalcResultArr[], GU8 uchAlgoResNum, GU8 uchChannelMapId)
{
    GS8  chRet = GH3X2X_RET_OK;
    GU8  uchChCntIndex   = 0;
    GF32 fIndexInc       = GH3X2X_GF32_0;            //step of g sensor data
    GU16 usDataCnt       = 0;    
    GU16 usDataIndex     = 0;
    GU16 usPayloadIndex  = 0;                        //payload data index
    GU8  uchGsEnable     = 0;
    GU8  uchAlgoResFlag  = 0;
    GU8  uchFrameDataNum = GH3X2X_FIFO_CNT_CALC(uchChannelMapCnt) + 1; //number of one frame data
    GU8  uchGainCurDataNum     = 0;                                    //gain current data number of one frame data
    GU8  uchAmbiDataNum        = 0;                                    //ambiance data number of one frame data
    GU16 usRawdataByteIndexTmp = 0;
    GU8  uchDumpDataIndexArr[GH3X2X_CHANNEL_MAP_MAX_CH] = {0};
    GU8  uchNextIncompleteFlag = 0;
    GU8  uchDumpDataCalcTmp    = 0;
    GU8  uchGainCurAddr        = 0;                                        // gain & current data addr when pack them
    GU8  uchAmbiAddr           = 0;                                        // ambiance data addr when pack them    
    
    if ((puchRespondBuffer == GH3X2X_PTR_NULL) || (pusRespondLen == GH3X2X_PTR_NULL)
         || (puchFifoRawdata == GH3X2X_PTR_NULL) || (uchChannelMapArr == GH3X2X_PTR_NULL))
    {
        GH3X2X_DEBUG_LOG("pointer param set null error!\r\n");
        return GH3X2X_RET_PARAMETER_ERROR;
    }

    for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)
    {
        if (GH3X2X_CLEAR_BIT(uchChannelMapArr[uchChCntIndex], GH3X2X_SLOT_ADC_NUM_BITS) != 0)
        {
            GH3X2X_DEBUG_LOG("channel map error! set @ref GH3X2X_CHANNEL_MAP!\r\n");
            return GH3X2X_RET_PARAMETER_ERROR;
        }
    }
    
    if (0 == g_usGh3x2xDataIndexInBuf)
    {
        GH3X2X_Memset(g_usRawdataByteIndexArr, 0, sizeof(g_usRawdataByteIndexArr[0]) * GH3X2X_CHANNEL_MAP_MAX_CH);
    }    

    if (nAlgoCalcResultArr == GH3X2X_PTR_NULL)
    {
        uchAlgoResNum = 0;
    }

    if ((0 == g_usGh3x2xDataIndexInBuf) && (GH3X2X_ElectrodeWearRevertDebugModeIsEnabled()))
    {
        uchAlgoResNum++;
        uchFrameDataNum += GH3X2X_ALGO_RESULT_WITH_TAG_CALC(1) + 1;
        uchAlgoResFlag = 1;
        
    }

    if (GH3X2X_CHECK_LEFT_BIT_SET(g_usDumpMode, GH3X2X_DUMP_GAIN_CURRENT_ENABLE_BIT))
    {
        uchGainCurDataNum = (GU8)GH3X2X_DUMP_GAIN_CURRENT_CALC(uchChannelMapCnt);
        uchFrameDataNum  += uchGainCurDataNum;
    }

    if (GH3X2X_CHECK_LEFT_BIT_SET(g_usDumpMode, GH3X2X_DUMP_AMBIANCE_ENABLE_BIT))
    {
        uchAmbiDataNum    = (GU8)GH3X2X_DUMP_AMBIANCE_CALC(uchChannelMapCnt);
        uchFrameDataNum  += uchAmbiDataNum;
    }
    
    if ((pstGsAxisValueArr == GH3X2X_PTR_NULL) || (0 == usGsDataNum))
    {
        uchGsEnable = 0;
    }
    else
    {
        uchGsEnable = 1;
        uchFrameDataNum += GH3X2X_GSENSOR_DATA_SIZE;
    }    
    
    chRet = GH3X2X_GetDumpDataInfo(&fIndexInc, &usDataCnt, &uchNextIncompleteFlag, uchDumpDataIndexArr, 
                        punLastIncompleteArr, usFifoRawdataLen, puchFifoRawdata, 
                        uchChannelMapCnt, uchChannelMapArr, puchChannelBgLvlArr, usGsDataNum);
    
    if (0 != g_usGh3x2xDataIndexInBuf)
    {
        GH3X2X_Memset(uchDumpDataIndexArr, 0, sizeof(uchDumpDataIndexArr));
    }
    
    if (GH3X2X_RET_OK == chRet)
    {
        g_pstGh3x2xProtocolData->puchPacketPayloadArr[UPROTOCOL_RAWDATA_TYPE_INDEX] = \
                                                GH3X2X_VAL_LEFT_SHIFT(uchChannelMapId, UPROTOCOL_FUNC_ID_FIELD)
                                                | GH3X2X_VAL_LEFT_SHIFT(uchGsEnable, UPROTOCOL_GS_ENABLE_FIELD)
                                                | GH3X2X_VAL_LEFT_SHIFT(uchAlgoResFlag, UPROTOCOL_ALGO_ENABLE_FIELD);
        
        if (GH3X2X_CHECK_LEFT_BIT_SET(g_usDumpMode, GH3X2X_DUMP_GAIN_CURRENT_ENABLE_BIT))
        {
            g_pstGh3x2xProtocolData->puchPacketPayloadArr[UPROTOCOL_RAWDATA_TYPE_INDEX] |= \
                                                GH3X2X_GET_LEFT_SHIFT_VAL(UPROTOCOL_AGC_ENABLE_FIELD);
        }

        if (GH3X2X_CHECK_LEFT_BIT_SET(g_usDumpMode, GH3X2X_DUMP_AMBIANCE_ENABLE_BIT))
        {
            g_pstGh3x2xProtocolData->puchPacketPayloadArr[UPROTOCOL_RAWDATA_TYPE_INDEX] |= \
                                                GH3X2X_GET_LEFT_SHIFT_VAL(UPROTOCOL_AMBIANCE_ENABLE_FIELD);
        }
        
        usPayloadIndex = UPROTOCOL_FRAME_DATA_INDEX;
        for (usDataIndex = g_usGh3x2xDataIndexInBuf; usDataIndex < usDataCnt; usDataIndex++)
        {        
            //judge if will exceed payload length limit
            if (usPayloadIndex + uchFrameDataNum >= GH3X2X_UPROTOCOL_GET_PAYLOAD_LEN_SUPPORT(g_uchPacketMaxLenSupport))
            {
                break;
            }
            g_pstGh3x2xProtocolData->puchPacketPayloadArr[usPayloadIndex] = g_uchFrameIdArr[uchChannelMapId];  //FrameID
            g_pstGh3x2xProtocolData->uchFrameIdArr[uchChannelMapId]++;
            usPayloadIndex++;
            if (1 == uchGsEnable)
            {
                GH3X2X_FillGsensorData(&usPayloadIndex, &pstGsAxisValueArr[(GU16)g_fGsDataIndexInBuf]);
                g_fGsDataIndexInBuf += fIndexInc;
            }

            if (GH3X2X_CHECK_LEFT_BIT_SET(g_usDumpMode, GH3X2X_DUMP_GAIN_CURRENT_ENABLE_BIT))
            {
                uchGainCurAddr = usPayloadIndex + GH3X2X_FIFO_CNT_CALC(uchChannelMapCnt);
            }

            if (GH3X2X_CHECK_LEFT_BIT_SET(g_usDumpMode, GH3X2X_DUMP_AMBIANCE_ENABLE_BIT))
            {
                if (GH3X2X_CHECK_LEFT_BIT_SET(g_usDumpMode, GH3X2X_DUMP_GAIN_CURRENT_ENABLE_BIT))
                {
                    uchAmbiAddr = (GU8)uchGainCurAddr + uchGainCurDataNum;
                }
                else
                {
                    uchAmbiAddr = usPayloadIndex + GH3X2X_FIFO_CNT_CALC(uchChannelMapCnt);
                }
            }
            
            for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++) // search each channel
            {
                uchDumpDataCalcTmp = g_uchDumpDataCntMapArr[puchChannelBgLvlArr[uchChCntIndex]];
                if (uchDumpDataIndexArr[uchChCntIndex] < uchDumpDataCalcTmp)
                {
                    usRawdataByteIndexTmp = g_usRawdataByteIndexArr[uchChCntIndex];
                    while (usRawdataByteIndexTmp < usFifoRawdataLen) // search each
                    {
                        if (GH3X2X_CHANNEL_MAP_GET(puchFifoRawdata[usRawdataByteIndexTmp]) == \
                                                                    uchChannelMapArr[uchChCntIndex]) // if map equal
                        {
                            punIncompleteDumpdataArr[uchChCntIndex][uchDumpDataIndexArr[uchChCntIndex]] = \
                                                GH3X2X_MAKEUP_DWORD(puchFifoRawdata[usRawdataByteIndexTmp + 3],
                                                                    puchFifoRawdata[usRawdataByteIndexTmp + 2],
                                                                    puchFifoRawdata[usRawdataByteIndexTmp + 1],
                                                                    puchFifoRawdata[usRawdataByteIndexTmp]);
                            uchDumpDataIndexArr[uchChCntIndex]++;
                            if (uchDumpDataIndexArr[uchChCntIndex] >= uchDumpDataCalcTmp)
                            {
                                usRawdataByteIndexTmp += GH3X2X_FIFO_RAWDATA_SIZE;
                                break;
                            }
                        }
                        usRawdataByteIndexTmp += GH3X2X_FIFO_RAWDATA_SIZE;
                    }
                }

                uchDumpDataIndexArr[uchChCntIndex] = 0;
                g_usRawdataByteIndexArr[uchChCntIndex] = usRawdataByteIndexTmp; // write back byte index
            } // end of for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)

            GH3X2X_FillDumpData(&usPayloadIndex, &uchGainCurAddr, &uchAmbiAddr, uchChannelMapArr, uchChannelMapCnt, \
                                puchChannelBgLvlArr, g_usDumpMode, punIncompleteDumpdataArr);
            
            usPayloadIndex += uchGainCurDataNum + uchAmbiDataNum;
            
            if (1 == uchAlgoResFlag)
            {
                g_pstGh3x2xProtocolData->puchPacketPayloadArr[usPayloadIndex] = GH3X2X_ALGO_RESULT_WITH_TAG_CALC(uchAlgoResNum);
                usPayloadIndex++;
                if ((GH3X2X_ElectrodeWearRevertDebugModeIsEnabled()) && (0 == usDataIndex))
                {
                    GH3X2X_FillElectrodeWearRevertData(&usPayloadIndex);
                    uchAlgoResNum -= 1;  //only the first frame data has Flag data
                    uchFrameDataNum -= GH3X2X_ALGO_RESULT_WITH_TAG_CALC(1);
                }                
            } // end of if (1 == uchAlgoResFlag)
        } // end of for (usDataIndex = g_usGh3x2xDataIndexInBuf; usDataIndex < usDataCnt; usDataIndex++)
        
        if (usDataIndex < usDataCnt)
        {
            g_usGh3x2xDataIndexInBuf = usDataIndex;
            chRet = GH3X2X_RET_RAW_DATA_PKG_CONTINUE;
        }
        else
        {
            g_usGh3x2xDataIndexInBuf = 0;
            g_fGsDataIndexInBuf = 0;
            chRet = GH3X2X_RET_RAW_DATA_PKG_OVER;
            GH3X2X_CacheIncompleteDumpdata(punLastIncompleteArr, punIncompleteDumpdataArr, uchNextIncompleteFlag, \
                                            puchFifoRawdata, usFifoRawdataLen, uchChannelMapArr, uchChannelMapCnt, \
                                            g_usRawdataByteIndexArr);
        } // end else

        g_pstGh3x2xProtocolData->puchPacketPayloadArr[UPROTOCOL_RAWDATA_LEN_INDEX] = (GU8)(usPayloadIndex - UPROTOCOL_FRAME_DATA_INDEX);
        /* respond packet format */
        SET_VAL_VIA_PTR(pusRespondLen, GH3X2X_UprotocolPacketFormat(GH3X2X_UPROTOCOL_CMD_RAWDATA, puchRespondBuffer,
                                    g_pstGh3x2xProtocolData->puchPacketPayloadArr, usPayloadIndex));
    } // end of if (GH3X2X_RET_OK == chRet)
    else
    {
        pusRespondLen = 0;
    }    

    return chRet;
}


/**
 * @fn       void GH3X2X_FillZipDumpData(GU16* usPayloadIndex, GU8* uchGainCurAddr, GU8* uchAmbiAddr, 
 *                                    GU8 uchChannelMapArr[], GU8 uchChannelMapCnt,GU8 *puchChannelBgLvlArr, 
 *                                    GU16 usDumpMode, GU32 punIncompleteDumpdataArr[][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL])
 *
 * @brief  fill zip dump data to protocol data
 *
 * @attention    None
 *
 * @param[in]    uchChMapArr[]        channel map array
 * @param[in]    uchChMapCnt          channel map cnt
 * @param[in]    puchChBgLvlArr[]     channel bg level array
 * @param[in]    usDumpMode           dump mode
 * @param[in]    punDumpData[][]      dump data of all channel
 * @param[out]    usPayloadIndex      current payload index
 * @param[out]    uchGainCurAddr      current payload index
 * @param[out]    uchAmbiAddr         current payload index
 *
 * @return    None
 */
GU8 GH3X2X_FillZipDumpData(GU16* usPayloadIndex, GU8 uchChannelMapArr[], 
                            GU8 uchChannelMapCnt, GU8 *puchChannelBgLvlArr, GU16 usDumpMode, 
                            GU32 punIncompleteDumpdataArr[][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL], 
                            GU8 uchZipDataFirstFrameSta, GU8 *uchZipDiffDataSize,
                            GU8 uchGainCurDataNum, GU8 uchAmbiDataNum)
{
    GU8  uchChCntIndex = 0;
    GU32 unFifoLastRawdataArr[GH3X2X_CHANNEL_MAP_MAX_CH];
    GU8 uchFifoLastRawdataTagArr[GH3X2X_CHANNEL_MAP_MAX_CH];
    GU32 unFifoLastAgcdataArr[GH3X2X_CHANNEL_MAP_MAX_CH];
    GU32 unFifoLastAmbdataArr[GH3X2X_CHANNEL_MAP_MAX_CH];

    GU8 uchZipDataArr[GH3X2X_UPROTOCOL_PACKET_LEN_MAX];
    GU16 usZipDataArrSize = 0;

    GU8 uchZipRawdataArr[GH3X2X_UPROTOCOL_PACKET_LEN_MAX];
    GU8 uchZipRawdataArrSize = 0;
    GU8 uchZipAgcDataArr[GH3X2X_UPROTOCOL_PACKET_LEN_MAX];
    GU8 uchZipAgcDataArrSize = 0;
    GU8 uchZipAmbDataArr[GH3X2X_UPROTOCOL_PACKET_LEN_MAX];
    GU8 uchZipAmbDataArrSize = 0;


    if (g_uchDumpOddEvenChangeFlag && uchZipDataFirstFrameSta)
    {
        if (*usPayloadIndex + (GU16)usZipDataArrSize > \
            (GU16)GH3X2X_UPROTOCOL_GET_PAYLOAD_LEN_SUPPORT(g_uchPacketMaxLenSupport))
        {
            return 1;
        }
        
        usZipDataArrSize = uchGainCurDataNum + uchAmbiDataNum;

        for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)
        {
            //dump data filter process
            GH3X2X_DumpDataPro(punIncompleteDumpdataArr, uchChannelMapArr, uchChannelMapCnt, \
                                puchChannelBgLvlArr, uchChCntIndex);
            GH3X2X_Memcpy(&g_pstGh3x2xProtocolData->puchPacketPayloadArr[*usPayloadIndex], \
                            &g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_PPG_INDEX], GH3X2X_FIFO_RAWDATA_SIZE);
            GH3X2X_ElectrWearDumpDataPro(&g_pstGh3x2xProtocolData->puchPacketPayloadArr[*usPayloadIndex], uchChCntIndex);
            *usPayloadIndex += GH3X2X_FIFO_RAWDATA_SIZE;
            g_stZipDumpTempData.unFifoLastRawdataArr[uchChCntIndex] = GH3X2X_MAKEUP_DWORD\
                (0, \
                 GH3X2X_GET_BYTE1_FROM_DWORD(g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_PPG_INDEX]), \
                 GH3X2X_GET_BYTE2_FROM_DWORD(g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_PPG_INDEX]), \
                 GH3X2X_GET_BYTE3_FROM_DWORD(g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_PPG_INDEX]));
            g_stZipDumpTempData.uchFifoLastRawdataTagArr[uchChCntIndex] = \
                GH3X2X_GET_BYTE0_FROM_DWORD(g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_PPG_INDEX]);            
        }// end of for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)

        if (GH3X2X_CHECK_LEFT_BIT_SET(usDumpMode, GH3X2X_DUMP_GAIN_CURRENT_ENABLE_BIT))
        {
            for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)
            {
                GH3X2X_DumpDataPro(punIncompleteDumpdataArr, uchChannelMapArr, uchChannelMapCnt, \
                                puchChannelBgLvlArr, uchChCntIndex);
                GH3X2X_Memcpy(&g_pstGh3x2xProtocolData->puchPacketPayloadArr[*usPayloadIndex], \
                                &g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_GAIN_CURRENT_INDEX], \
                                GH3X2X_DUMP_GAIN_CURRENT_DATA_SIZE);
                *usPayloadIndex += GH3X2X_DUMP_GAIN_CURRENT_DATA_SIZE;
                GH3X2X_Memcpy(&g_stZipDumpTempData.unFifoLastAgcdataArr[uchChCntIndex], \
                                &g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_GAIN_CURRENT_INDEX], \
                                GH3X2X_DUMP_GAIN_CURRENT_DATA_SIZE);
            }
        }
            
        if (GH3X2X_CHECK_LEFT_BIT_SET(usDumpMode, GH3X2X_DUMP_AMBIANCE_ENABLE_BIT))
        {
            for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)
            {
                GH3X2X_DumpDataPro(punIncompleteDumpdataArr, uchChannelMapArr, uchChannelMapCnt, \
                                puchChannelBgLvlArr, uchChCntIndex);
                GH3X2X_Memcpy(&g_pstGh3x2xProtocolData->puchPacketPayloadArr[*usPayloadIndex], \
                                &g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_AMBIANCE_INDEX], \
                                GH3X2X_DUMP_AMBIANCE_DATA_SIZE);
                *usPayloadIndex += GH3X2X_DUMP_AMBIANCE_DATA_SIZE;
                GH3X2X_Memcpy(&g_stZipDumpTempData.unFifoLastAmbdataArr[uchChCntIndex], \
                                &g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_AMBIANCE_INDEX], \
                                GH3X2X_DUMP_AMBIANCE_DATA_SIZE);
            }
        } //end of if (GH3X2X_CHECK_LEFT_BIT_SET(usDumpMode, GH3X2X_DUMP_AMBIANCE_ENABLE_BIT))
    }//end of if (g_uchDumpOddEvenChangeFlag && uchZipDataFirstFrameSta)
    else
    {
        for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)
        {
            //dump data filter process
            GH3X2X_DumpDataPro(punIncompleteDumpdataArr, uchChannelMapArr, uchChannelMapCnt, \
                                puchChannelBgLvlArr, uchChCntIndex);
            GH3X2X_ElectrWearDumpDataPro(&g_pstGh3x2xProtocolData->puchPacketPayloadArr[*usPayloadIndex], uchChCntIndex);
            unFifoLastRawdataArr[uchChCntIndex] = 0;
            uchFifoLastRawdataTagArr[uchChCntIndex] = 0;
            unFifoLastRawdataArr[uchChCntIndex] = GH3X2X_MAKEUP_DWORD\
                (0, \
                 GH3X2X_GET_BYTE1_FROM_DWORD(g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_PPG_INDEX]), \
                 GH3X2X_GET_BYTE2_FROM_DWORD(g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_PPG_INDEX]), \
                 GH3X2X_GET_BYTE3_FROM_DWORD(g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_PPG_INDEX]));
            uchFifoLastRawdataTagArr[uchChCntIndex] = \
                GH3X2X_GET_BYTE0_FROM_DWORD(g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_PPG_INDEX]);          
        }

        if (GH3X2X_CHECK_LEFT_BIT_SET(usDumpMode, GH3X2X_DUMP_GAIN_CURRENT_ENABLE_BIT))
        {
            for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)
            {
                GH3X2X_DumpDataPro(punIncompleteDumpdataArr, uchChannelMapArr, uchChannelMapCnt, \
                                puchChannelBgLvlArr, uchChCntIndex);
                unFifoLastAgcdataArr[uchChCntIndex] = 0;
                GH3X2X_Memcpy(&unFifoLastAgcdataArr[uchChCntIndex], \
                                &g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_GAIN_CURRENT_INDEX], \
                                GH3X2X_DUMP_GAIN_CURRENT_DATA_SIZE);
            }
        }
            
        if (GH3X2X_CHECK_LEFT_BIT_SET(usDumpMode, GH3X2X_DUMP_AMBIANCE_ENABLE_BIT))
        {
            for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)
            {
                GH3X2X_DumpDataPro(punIncompleteDumpdataArr, uchChannelMapArr, uchChannelMapCnt, \
                                puchChannelBgLvlArr, uchChCntIndex);
                unFifoLastAmbdataArr[uchChCntIndex] = 0;
                GH3X2X_Memcpy(&unFifoLastAmbdataArr[uchChCntIndex], \
                                &g_punDumpDataUploadArr[GH3X2X_DUMP_DATA_AMBIANCE_INDEX], \
                                GH3X2X_DUMP_AMBIANCE_DATA_SIZE);
            }
        } //end of for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)

        GH3X2X_Memset(uchZipRawdataArr, 0, sizeof(uchZipRawdataArr[0]) * GH3X2X_UPROTOCOL_PACKET_LEN_MAX);
        GH3X2X_GetRawDataDiff(unFifoLastRawdataArr, uchFifoLastRawdataTagArr, \
                        uchChannelMapCnt, uchZipRawdataArr, &uchZipRawdataArrSize, \
                        g_stZipDumpTempData.uchFifoLastRawdataTagArr, g_stZipDumpTempData.unFifoLastRawdataArr);
        GH3X2X_Memcpy(&uchZipDataArr[usZipDataArrSize], uchZipRawdataArr, uchZipRawdataArrSize);
        usZipDataArrSize += uchZipRawdataArrSize;

        if (GH3X2X_CHECK_LEFT_BIT_SET(usDumpMode, GH3X2X_DUMP_GAIN_CURRENT_ENABLE_BIT))
        {
            GH3X2X_Memset(uchZipAgcDataArr, 0, sizeof(uchZipAgcDataArr[0]) * GH3X2X_UPROTOCOL_PACKET_LEN_MAX);
            GH3X2X_GetDataDiff(unFifoLastAgcdataArr, uchChannelMapCnt, uchZipAgcDataArr, &uchZipAgcDataArrSize, \
                g_stZipDumpTempData.unFifoLastAgcdataArr);
            GH3X2X_Memcpy(&uchZipDataArr[usZipDataArrSize], uchZipAgcDataArr, uchZipAgcDataArrSize);
            usZipDataArrSize += uchZipAgcDataArrSize;
        }
        
        if (GH3X2X_CHECK_LEFT_BIT_SET(usDumpMode, GH3X2X_DUMP_AMBIANCE_ENABLE_BIT))
        {
            GH3X2X_Memset(uchZipAmbDataArr, 0, sizeof(uchZipAmbDataArr[0]) * GH3X2X_UPROTOCOL_PACKET_LEN_MAX);
            GH3X2X_GetDataDiff(unFifoLastAmbdataArr, uchChannelMapCnt, uchZipAmbDataArr, &uchZipAmbDataArrSize, \
                g_stZipDumpTempData.unFifoLastAmbdataArr);
            GH3X2X_Memcpy(&uchZipDataArr[usZipDataArrSize], uchZipAmbDataArr, uchZipAmbDataArrSize);
            usZipDataArrSize += uchZipAmbDataArrSize;
        }
        
        if (*usPayloadIndex + (GU16)usZipDataArrSize > \
            (GU16)GH3X2X_UPROTOCOL_GET_PAYLOAD_LEN_SUPPORT(g_uchPacketMaxLenSupport))
        {
            return 1;
        }
        else
        {                
            GH3X2X_Memcpy(g_stZipDumpTempData.unFifoLastRawdataArr, unFifoLastRawdataArr, \
                        uchChannelMapCnt*GH3X2X_FIFO_RAWDATA_SIZE);
            GH3X2X_Memcpy(g_stZipDumpTempData.uchFifoLastRawdataTagArr, uchFifoLastRawdataTagArr, \
                        uchChannelMapCnt);
            GH3X2X_Memcpy(g_stZipDumpTempData.unFifoLastAgcdataArr, unFifoLastAgcdataArr, \
                        uchChannelMapCnt*GH3X2X_FIFO_RAWDATA_SIZE);
            GH3X2X_Memcpy(g_stZipDumpTempData.unFifoLastAmbdataArr, unFifoLastAmbdataArr, \
                        uchChannelMapCnt*GH3X2X_FIFO_RAWDATA_SIZE);

            GH3X2X_Memcpy(&g_pstGh3x2xProtocolData->puchPacketPayloadArr[*usPayloadIndex], uchZipDataArr, usZipDataArrSize);
            *usPayloadIndex += (GU16)usZipDataArrSize;
        }
    }// end of else

    return 0;
}

/**
 * @fn    GS8 GH3X2X_SendZipDumpDataPkgWithAlgo(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 punLastIncompleteArr[], 
                        GU32 punIncompleteDumpdataArr[][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL], GU8 *puchFifoRawdata, 
                        GU16 usFifoRawdataLen, GU8 uchChannelMapArr[], GU8 uchChannelMapCnt, GU8 *puchChannelBgLvlArr, 
                        STGsensorRawdata *pstGsAxisValueArr, GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity, 
                        GS32 nAlgoCalcResultArr[], GU8 uchAlgoResNum, GU8 uchChannelMapId)
 *
 * @brief  Pack Zip rawdata with g sensor data and algorithm data
 *
 * @attention   None
 *
 * @param[out]  puchRespondBuffer       pointer to protocol pkg data.
 * @param[out]  pusRespondLen           pointer to protocol pkg data length
 * @param[out]  punLastIncompleteArr    incomplete array
 * @param[out]  punIncompleteRawdataArr incomplete data array used for cache data
 * @param[in]   puchFifoRawdata         pointer to data buffer of rawdata
 * @param[in]   usFifoRawdataLen        data num of rawdata buf
 * @param[in]   uchChannelMapArr        pointer to channel map array
 * @param[in]   uchChannelMapCnt        channel map array cnt, max:32
 * @param[in]   puchChannelBgLvlArr     bg level array
 * @param[in]   pstGsAxisValueArr       pointer to g sensor data
 * @param[in]   usGsDataNum             g sensor data number
 * @param[in]   emGsSensitivity         g sensor sensitivity
 * @param[in]   nAlgoCalcResultArr      algorithm result
 * @param[in]   uchAlgoResNum           number of algorithm result
 * @param[in]   uchChannelMapId         channel map id (0 ~ 15)
 * @return  error code
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return param error
 * @retval  #GH3X2X_RET_RAW_DATA_PKG_OVER       return already read all the package data 
 * @retval  #GH3X2X_RET_RAW_DATA_PKG_CONTINUE   return should read rawdata package data again
 * @retval  #GH3X2X_RET_RESOURCE_ERROR          return not enough data
 */
GS8 GH3X2X_SendZipDumpDataPkgWithAlgo(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 punLastIncompleteArr[], 
                        GU32 punIncompleteDumpdataArr[][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL], GU8 *puchFifoRawdata, 
                        GU16 usFifoRawdataLen, GU8 uchChannelMapArr[], GU8 uchChannelMapCnt, GU8 *puchChannelBgLvlArr, 
                        STGsensorRawdata *pstGsAxisValueArr, GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity, 
                        GS32 nAlgoCalcResultArr[], GU8 uchAlgoResNum, GU8 uchChannelMapId)
{
    GS8  chRet = GH3X2X_RET_OK;
    GU8  uchChCntIndex   = 0;
    GF32 fIndexInc       = GH3X2X_GF32_0;            //step of g sensor data
    GU16 usDataCnt       = 0;    
    GU16 usDataIndex     = 0;
    GU16 usPayloadIndex  = 0;                        //payload data index
    GU8  uchGsEnable     = 0;
    GU8  uchAlgoResFlag  = 0;
    GU8  uchFrameDataNum = GH3X2X_FIFO_CNT_CALC(uchChannelMapCnt) + 1; //number of one frame data
    GU8  uchGainCurDataNum     = 0;                                    //gain current data number of one frame data
    GU8  uchAmbiDataNum        = 0;                                    //ambiance data number of one frame data
    GU16 usRawdataByteIndexTmp = 0;
    GU8  uchDumpDataIndexArr[GH3X2X_CHANNEL_MAP_MAX_CH] = {0};
    GU8  uchNextIncompleteFlag = 0;
    GU8  uchDumpDataCalcTmp    = 0;
    GU8  uchSendCmd            = 0;                                        // zip cmd (0x09/0x0a)
    GU8  uchZipDataFirstFrameSta = 1;
    GU8  uchZipDiffDataSize = 0;
    GU8  uchZipRet = 0;
    GU16 usRawdataByteIndexTmpBeforeHandleArr[GH3X2X_CHANNEL_MAP_MAX_CH];
    GU16 usFrameidIndex;
    
    if ((puchRespondBuffer == GH3X2X_PTR_NULL) || (pusRespondLen == GH3X2X_PTR_NULL)
         || (puchFifoRawdata == GH3X2X_PTR_NULL) || (uchChannelMapArr == GH3X2X_PTR_NULL))
    {
        GH3X2X_DEBUG_LOG("pointer param set null error!\r\n");
        return GH3X2X_RET_PARAMETER_ERROR;
    }

    for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)
    {
        if (GH3X2X_CLEAR_BIT(uchChannelMapArr[uchChCntIndex], GH3X2X_SLOT_ADC_NUM_BITS) != 0)
        {
            GH3X2X_DEBUG_LOG("channel map error! set @ref GH3X2X_CHANNEL_MAP!\r\n");
            return GH3X2X_RET_PARAMETER_ERROR;
        }
    }
    
    if (0 == g_usGh3x2xDataIndexInBuf)
    {
        GH3X2X_Memset(g_usRawdataByteIndexArr, 0, sizeof(g_usRawdataByteIndexArr[0]) * GH3X2X_CHANNEL_MAP_MAX_CH);
    }    

    if (nAlgoCalcResultArr == GH3X2X_PTR_NULL)
    {
        uchAlgoResNum = 0;
    }

    if ((0 == g_usGh3x2xDataIndexInBuf) && (GH3X2X_ElectrodeWearRevertDebugModeIsEnabled()))
    {
        uchAlgoResNum++;
        uchFrameDataNum += GH3X2X_ALGO_RESULT_WITH_TAG_CALC(1) + 1;
        uchAlgoResFlag = 1;
    }

    if (GH3X2X_CHECK_LEFT_BIT_SET(g_usDumpMode, GH3X2X_DUMP_GAIN_CURRENT_ENABLE_BIT))
    {
        uchGainCurDataNum = (GU8)GH3X2X_DUMP_GAIN_CURRENT_CALC(uchChannelMapCnt);
        uchFrameDataNum  += uchGainCurDataNum;
    }

    if (GH3X2X_CHECK_LEFT_BIT_SET(g_usDumpMode, GH3X2X_DUMP_AMBIANCE_ENABLE_BIT))
    {
        uchAmbiDataNum    = (GU8)GH3X2X_DUMP_AMBIANCE_CALC(uchChannelMapCnt);
        uchFrameDataNum  += uchAmbiDataNum;
    }
    
    if ((pstGsAxisValueArr == GH3X2X_PTR_NULL) || (0 == usGsDataNum))
    {
        uchGsEnable = 0;
    }
    else
    {
        uchGsEnable = 1;
        uchFrameDataNum += GH3X2X_GSENSOR_DATA_SIZE;
    }    
    
    chRet = GH3X2X_GetDumpDataInfo(&fIndexInc, &usDataCnt, &uchNextIncompleteFlag, uchDumpDataIndexArr, 
                        punLastIncompleteArr, usFifoRawdataLen, puchFifoRawdata, 
                        uchChannelMapCnt, uchChannelMapArr, puchChannelBgLvlArr, usGsDataNum);
    
    if (0 != g_usGh3x2xDataIndexInBuf)
    {
        GH3X2X_Memset(uchDumpDataIndexArr, 0, sizeof(uchDumpDataIndexArr));
    }
    
    if (GH3X2X_RET_OK == chRet)
    {
        g_pstGh3x2xProtocolData->puchPacketPayloadArr[UPROTOCOL_RAWDATA_TYPE_INDEX] = \
                                                GH3X2X_VAL_LEFT_SHIFT(uchChannelMapId, UPROTOCOL_FUNC_ID_FIELD)
                                                | GH3X2X_VAL_LEFT_SHIFT(uchGsEnable, UPROTOCOL_GS_ENABLE_FIELD)
                                                | GH3X2X_VAL_LEFT_SHIFT(uchAlgoResFlag, UPROTOCOL_ALGO_ENABLE_FIELD);
        
        if (GH3X2X_CHECK_LEFT_BIT_SET(g_usDumpMode, GH3X2X_DUMP_GAIN_CURRENT_ENABLE_BIT))
        {
            g_pstGh3x2xProtocolData->puchPacketPayloadArr[UPROTOCOL_RAWDATA_TYPE_INDEX] |= \
                                                GH3X2X_GET_LEFT_SHIFT_VAL(UPROTOCOL_AGC_ENABLE_FIELD);
        }

        if (GH3X2X_CHECK_LEFT_BIT_SET(g_usDumpMode, GH3X2X_DUMP_AMBIANCE_ENABLE_BIT))
        {
            g_pstGh3x2xProtocolData->puchPacketPayloadArr[UPROTOCOL_RAWDATA_TYPE_INDEX] |= \
                                                GH3X2X_GET_LEFT_SHIFT_VAL(UPROTOCOL_AMBIANCE_ENABLE_FIELD);
        }
        
        usPayloadIndex = UPROTOCOL_FRAME_DATA_INDEX;
        for (usDataIndex = g_usGh3x2xDataIndexInBuf; usDataIndex < usDataCnt; usDataIndex++)
        {        
            uchZipDataFirstFrameSta = (usDataIndex == g_usGh3x2xDataIndexInBuf);
            //judge if will exceed payload length limit
            g_pstGh3x2xProtocolData->puchPacketPayloadArr[usPayloadIndex] = g_pstGh3x2xProtocolData->uchFrameIdArr[uchChannelMapId];  //FrameID
            usFrameidIndex = usPayloadIndex;
            g_pstGh3x2xProtocolData->uchFrameIdArr[uchChannelMapId]++;
            usPayloadIndex++;
            if (1 == uchGsEnable)
            {
                GH3X2X_FillGsensorData(&usPayloadIndex, &pstGsAxisValueArr[(GU16)g_fGsDataIndexInBuf]);
                g_fGsDataIndexInBuf += fIndexInc;
            }
            
            for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++) // search each channel
            {
                uchDumpDataCalcTmp = g_uchDumpDataCntMapArr[puchChannelBgLvlArr[uchChCntIndex]];
                usRawdataByteIndexTmpBeforeHandleArr[uchChCntIndex] = g_usRawdataByteIndexArr[uchChCntIndex];
                if (uchDumpDataIndexArr[uchChCntIndex] < uchDumpDataCalcTmp)
                {
                    usRawdataByteIndexTmp = g_usRawdataByteIndexArr[uchChCntIndex];
                    while (usRawdataByteIndexTmp < usFifoRawdataLen) // search each
                    {
                        if (GH3X2X_CHANNEL_MAP_GET(puchFifoRawdata[usRawdataByteIndexTmp]) == \
                                                                    uchChannelMapArr[uchChCntIndex]) // if map equal
                        {
                            punIncompleteDumpdataArr[uchChCntIndex][uchDumpDataIndexArr[uchChCntIndex]] = \
                                                GH3X2X_MAKEUP_DWORD(puchFifoRawdata[usRawdataByteIndexTmp + 3],
                                                                    puchFifoRawdata[usRawdataByteIndexTmp + 2],
                                                                    puchFifoRawdata[usRawdataByteIndexTmp + 1],
                                                                    puchFifoRawdata[usRawdataByteIndexTmp]);
                            uchDumpDataIndexArr[uchChCntIndex]++;
                            if (uchDumpDataIndexArr[uchChCntIndex] >= uchDumpDataCalcTmp)
                            {
                                usRawdataByteIndexTmp += GH3X2X_FIFO_RAWDATA_SIZE;
                                break;
                            }
                        }
                        usRawdataByteIndexTmp += GH3X2X_FIFO_RAWDATA_SIZE;
                    }
                }

                uchDumpDataIndexArr[uchChCntIndex] = 0;
                g_usRawdataByteIndexArr[uchChCntIndex] = usRawdataByteIndexTmp; // write back byte index
            } // end of for (uchChCntIndex = 0; uchChCntIndex < uchChannelMapCnt; uchChCntIndex++)

            uchZipRet = GH3X2X_FillZipDumpData(&usPayloadIndex, uchChannelMapArr, uchChannelMapCnt, \
                                puchChannelBgLvlArr, g_usDumpMode, punIncompleteDumpdataArr, uchZipDataFirstFrameSta, \
                                &uchZipDiffDataSize, uchGainCurDataNum, uchAmbiDataNum);

            if (1 == uchZipRet)
            {
                GH3X2X_ZipPackFullStatusHandle(uchChannelMapId, usFrameidIndex, fIndexInc, \
                            &usPayloadIndex, &usRawdataByteIndexTmp, \
                            uchChannelMapCnt, usRawdataByteIndexTmpBeforeHandleArr);
                break;
            }
            
            if (1 == uchAlgoResFlag)
            {
                if (usPayloadIndex + (GU16)GH3X2X_ALGO_RESULT_WITH_TAG_CALC(uchAlgoResNum) + 1 >= \
                        (GU16)GH3X2X_UPROTOCOL_GET_PAYLOAD_LEN_SUPPORT(g_uchPacketMaxLenSupport))
                {
                    GH3X2X_ZipPackFullStatusHandle(uchChannelMapId, usFrameidIndex, fIndexInc, \
                            &usPayloadIndex, &usRawdataByteIndexTmp, \
                            uchChannelMapCnt, usRawdataByteIndexTmpBeforeHandleArr);
                    break;
                }
                g_pstGh3x2xProtocolData->puchPacketPayloadArr[usPayloadIndex] = GH3X2X_ALGO_RESULT_WITH_TAG_CALC(uchAlgoResNum);
                usPayloadIndex++;
                #if 0
                if (nAlgoCalcResultArr != GH3X2X_PTR_NULL)
                {
                    if (GH3X2X_ElectrodeWearRevertDebugModeIsEnabled())
                    {
                        GH3X2X_FillAlgoData(&usPayloadIndex, nAlgoCalcResultArr, uchAlgoResNum - 1);
                    }
                    else
                    {
                        GH3X2X_FillAlgoData(&usPayloadIndex, nAlgoCalcResultArr, uchAlgoResNum);
                    }
                }
                #endif
                
                if ((GH3X2X_ElectrodeWearRevertDebugModeIsEnabled()) && (0 == usDataIndex))
                {
                    GH3X2X_FillElectrodeWearRevertData(&usPayloadIndex);
                    uchAlgoResNum -= 1;  //only the first frame data has Flag data
                }
                
                //uchAlgoResFlag = 0;
                //uchFrameDataNum -= (GH3X2X_ALGO_RESULT_WITH_TAG_CALC(uchAlgoResNum) + 1);
            } // end of if (1 == uchAlgoResFlag)
        } // end of for (usDataIndex = g_usGh3x2xDataIndexInBuf; usDataIndex < usDataCnt; usDataIndex++)
        
        if (usDataIndex < usDataCnt)
        {
            g_usGh3x2xDataIndexInBuf = usDataIndex;
            chRet = GH3X2X_RET_RAW_DATA_PKG_CONTINUE;
        }
        else
        {
            g_usGh3x2xDataIndexInBuf = 0;
            g_fGsDataIndexInBuf = 0;
            chRet = GH3X2X_RET_RAW_DATA_PKG_OVER;
            GH3X2X_CacheIncompleteDumpdata(punLastIncompleteArr, punIncompleteDumpdataArr, uchNextIncompleteFlag, \
                                            puchFifoRawdata, usFifoRawdataLen, uchChannelMapArr, uchChannelMapCnt, \
                                            g_usRawdataByteIndexArr);
        } // end else

        g_pstGh3x2xProtocolData->puchPacketPayloadArr[UPROTOCOL_RAWDATA_LEN_INDEX] = (GU8)(usPayloadIndex - UPROTOCOL_FRAME_DATA_INDEX);
        uchSendCmd = GH3X2X_VAL_ZIPSENDCMD(g_uchDumpOddEvenChangeFlag);
        /* respond packet format */
        SET_VAL_VIA_PTR(pusRespondLen, GH3X2X_UprotocolPacketFormat(uchSendCmd, puchRespondBuffer,
                                    g_pstGh3x2xProtocolData->puchPacketPayloadArr, usPayloadIndex));
        g_uchDumpOddEvenChangeFlag = !g_uchDumpOddEvenChangeFlag;
    } // end of if (GH3X2X_RET_OK == chRet)
    else
    {
        pusRespondLen = 0;
    }    

    return chRet;
}

/**
 * @fn     GS8 GH3X2X_SendHbDumpDataPkg(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
                                            GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                            GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *                  
 * @brief  pack GH3X2X hb rawdata and other type of data according to specific protocol
 *
 * @attention   None
 *
 * @param[out]  puchRespondBuffer       pointer to frame data package
 * @param[out]  pusRespondLen           pointer to data length of package
 * @param[in]   puchFifoRawdata         pointer to encrypted rawdata
 * @param[in]   nAlgoResultArr          pointer to algorithm result
 * @param[in]   pstGsAxisValueArr       pointer to g sensor data
 * @param[in]   usGsDataNum             number of g sensor data
 * @param[in]   emGsSensitivity         sensitivity of g sensor
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return func start error
 * @retval  #GH3X2X_RET_NO_INITED_ERROR         gh3x2x has not inited
 */
GS8 GH3X2X_SendHbDumpDataPkg(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
                                            GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                            GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
{
    GS8 chRet = GH3X2X_RET_OK;
    GU8 uchHbaBgLevelArr[GH3X2X_HBA_CHANNEL_MAP_MAX_CNT] = {0};
    if (GH3X2X_CHECK_BIT_SET(g_usFuncStartedBitmap, GH3X2X_STARTED_BITMAP_HBA))
    {
        GU16 usReadDataLen = 0;
        GH3X2X_BgLevelDecode(uchHbaBgLevelArr, g_uchHbaChannelMapCnt, g_uchHbaChannelMapArr, g_usBgLevel);
        GH3X2X_DecryptRawdataBuffer(puchFifoRawdata, &usReadDataLen);
        chRet = GH3X2X_SendDumpDataPkgWithAlgo(puchRespondBuffer, pusRespondLen, g_uchHbaIncompleteArr, 
                                        g_unHbaIncompleteDumpdataArr, puchFifoRawdata, usReadDataLen,
                                        g_uchHbaChannelMapArr, g_uchHbaChannelMapCnt, uchHbaBgLevelArr, 
                                        pstGsAxisValueArr, usGsDataNum,
                                        emGsSensitivity, GH3X2X_PTR_NULL, GH3X2X_HB_ALGO_OUTPUT_NUM, GH3X2X_FUNC_OFFSET_HR);
        GH3X2X_EncryptRawdataBuffer(puchFifoRawdata, usReadDataLen);
    }
    else
    {
        chRet = GH3X2X_RET_GENERIC_ERROR;
        GH3X2X_DEBUG_LOG("hba hasn't started error!\r\n");
    }
    return chRet;
}

/**
 * @fn     GS8 GH3X2X_SendHbDumpDataPkgEx(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata, 
 *                                          GU16 usFifoDataLen, GS32 nAlgoResultArr[], 
 *                                          STGsensorRawdata *pstGsAxisValueArr, GU16 usGsDataNum, 
 *                                          EMGsensorSensitivity emGsSensitivity)
 *                  
 * @brief  pack GH3X2X hb rawdata and other type of data according to specific protocol
 *
 * @attention   None
 *
 * @param[out]  puchRespondBuffer       pointer to frame data package
 * @param[out]  pusRespondLen           pointer to data length of package
 * @param[in]   puchFifoRawdata         pointer to encrypted rawdata
 * @param[in]   usFifoDataLen           gh3x2x fifo data len in byte
 * @param[in]   nAlgoResultArr          pointer to algorithm result
 * @param[in]   pstGsAxisValueArr       pointer to g sensor data
 * @param[in]   usGsDataNum             number of g sensor data
 * @param[in]   emGsSensitivity         sensitivity of g sensor
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return func start error
 * @retval  #GH3X2X_RET_NO_INITED_ERROR         gh3x2x has not inited
 */
GS8 GH3X2X_SendHbDumpDataPkgEx(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata, GU16 usFifoDataLen,
                                            GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                            GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
{
    GS8 chRet = GH3X2X_RET_OK;
    GU8 uchHbaBgLevelArr[GH3X2X_HBA_CHANNEL_MAP_MAX_CNT] = {0};
    if (GH3X2X_CHECK_BIT_SET(g_usFuncStartedBitmap, GH3X2X_STARTED_BITMAP_HBA))
    {
        GH3X2X_BgLevelDecode(uchHbaBgLevelArr, g_uchHbaChannelMapCnt, g_uchHbaChannelMapArr, g_usBgLevel);
        chRet = GH3X2X_SendDumpDataPkgWithAlgo(puchRespondBuffer, pusRespondLen, g_uchHbaIncompleteArr, 
                                        g_unHbaIncompleteDumpdataArr, puchFifoRawdata, usFifoDataLen,
                                        g_uchHbaChannelMapArr, g_uchHbaChannelMapCnt, uchHbaBgLevelArr, 
                                        pstGsAxisValueArr, usGsDataNum,
                                        emGsSensitivity, GH3X2X_PTR_NULL, GH3X2X_HB_ALGO_OUTPUT_NUM, GH3X2X_FUNC_OFFSET_HR);
    }
    else
    {
        chRet = GH3X2X_RET_GENERIC_ERROR;
        GH3X2X_DEBUG_LOG("hba hasn't started error!\r\n");
    }
    return chRet;
}

GS8 GH3X2X_SendHrvDumpDataPkg(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
                                            GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                            GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
{
    GS8 chRet = GH3X2X_RET_OK;
    GU8 uchHrvBgLevelArr[GH3X2X_HRV_CHANNEL_MAP_MAX_CNT] = {0};
    if (GH3X2X_CHECK_BIT_SET(g_usFuncStartedBitmap, GH3X2X_STARTED_BITMAP_HRV))
    {
        GU16 usReadDataLen = 0;
        GH3X2X_BgLevelDecode(uchHrvBgLevelArr, g_uchHrvChannelMapCnt, g_uchHrvChannelMapArr, g_usBgLevel);
        GH3X2X_DecryptRawdataBuffer(puchFifoRawdata, &usReadDataLen);
        chRet = GH3X2X_SendDumpDataPkgWithAlgo(puchRespondBuffer, pusRespondLen, g_uchHrvIncompleteArr, 
                                        g_unHrvIncompleteDumpdataArr, puchFifoRawdata, usReadDataLen,
                                        g_uchHrvChannelMapArr, g_uchHrvChannelMapCnt, uchHrvBgLevelArr, 
                                        pstGsAxisValueArr, usGsDataNum,
                                        emGsSensitivity, GH3X2X_PTR_NULL, GH3X2X_HRV_ALGO_OUTPUT_NUM, GH3X2X_FUNC_OFFSET_HRV);
        GH3X2X_EncryptRawdataBuffer(puchFifoRawdata, usReadDataLen);
    }
    else
    {
        chRet = GH3X2X_RET_GENERIC_ERROR;
        GH3X2X_DEBUG_LOG("hrv hasn't started error!\r\n");
    }
    return chRet;
}

GS8 GH3X2X_SendHrvDumpDataPkgEx(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata, GU16 usFifoDataLen,
                                            GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                            GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
{
    GS8 chRet = GH3X2X_RET_OK;
    GU8 uchHrvBgLevelArr[GH3X2X_HRV_CHANNEL_MAP_MAX_CNT] = {0};
    if (GH3X2X_CHECK_BIT_SET(g_usFuncStartedBitmap, GH3X2X_STARTED_BITMAP_HRV))
    {
        GH3X2X_BgLevelDecode(uchHrvBgLevelArr, g_uchHrvChannelMapCnt, g_uchHrvChannelMapArr, g_usBgLevel);
        chRet = GH3X2X_SendDumpDataPkgWithAlgo(puchRespondBuffer, pusRespondLen, g_uchHrvIncompleteArr, 
                                        g_unHrvIncompleteDumpdataArr, puchFifoRawdata, usFifoDataLen,
                                        g_uchHrvChannelMapArr, g_uchHrvChannelMapCnt, uchHrvBgLevelArr, 
                                        pstGsAxisValueArr, usGsDataNum,
                                        emGsSensitivity, GH3X2X_PTR_NULL, GH3X2X_HRV_ALGO_OUTPUT_NUM, GH3X2X_FUNC_OFFSET_HRV);
    }
    else
    {
        chRet = GH3X2X_RET_GENERIC_ERROR;
        GH3X2X_DEBUG_LOG("hrv hasn't started error!\r\n");
    }
    return chRet;
}

/**
 * @fn       GS8 GH3X2X_SendSpo2DumpDataPkg(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
                                            GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                            GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *                    
 * @brief  pack GH3X2X hb rawdata and other type of data according to specific protocol
 *
 * @attention    None
 *
 * @param[out]   puchRespondBuffer         pointer to frame data package
 * @param[out]   pusRespondLen             pointer to data length of package
 * @param[in]    puchFifoRawdata           pointer to encrypted rawdata
 * @param[in]    nAlgoResultArr            pointer to algorithm result
 * @param[in]    pstGsAxisValueArr         pointer to g sensor data
 * @param[in]    usGsDataNum               number of g sensor data
 * @param[in]    emGsSensitivity           sensitivity of g sensor
 *
 * @return    error code
 * @retval    #GH3X2X_RET_OK                        return successfully
 * @retval    #GH3X2X_RET_GENERIC_ERROR             return func start error
 * @retval    #GH3X2X_RET_NO_INITED_ERROR           gh3x2x has not inited
 */
GS8 GH3X2X_SendSpo2DumpDataPkg(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
                                GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
{
    GS8 chRet = GH3X2X_RET_OK;
    GU8 uchSpo2LevelArr[GH3X2X_SPO2_CHANNEL_MAP_MAX_CNT] = {0};
    if (GH3X2X_CHECK_BIT_SET(g_usFuncStartedBitmap, GH3X2X_STARTED_BITMAP_SPO2))
    {
        GU16 usReadDataLen = 0;
        GH3X2X_BgLevelDecode(uchSpo2LevelArr, g_uchSpo2ChannelMapCnt, g_uchSpo2ChannelMapArr, g_usBgLevel);
        GH3X2X_DecryptRawdataBuffer(puchFifoRawdata, &usReadDataLen);
        chRet = GH3X2X_SendDumpDataPkgWithAlgo(puchRespondBuffer, pusRespondLen, g_uchSpo2IncompleteArr, 
                                        g_unSpo2IncompleteDumpdataArr, puchFifoRawdata, usReadDataLen,
                                        g_uchSpo2ChannelMapArr, g_uchSpo2ChannelMapCnt, uchSpo2LevelArr, 
                                        pstGsAxisValueArr, usGsDataNum, emGsSensitivity, GH3X2X_PTR_NULL, 
                                        GH3X2X_SPO2_ALGO_OUTPUT_NUM, GH3X2X_FUNC_OFFSET_SPO2);
        GH3X2X_EncryptRawdataBuffer(puchFifoRawdata, usReadDataLen);
    }
    else
    {
        chRet = GH3X2X_RET_GENERIC_ERROR;
        GH3X2X_DEBUG_LOG("spo2 hasn't started error!\r\n");
    }
    return chRet;
}

/**
 * @fn       GS8 GH3X2X_SendSpo2DumpDataPkgEx(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata, 
 *                              GU16 usFifoDataLen, GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *                    
 * @brief  pack GH3X2X hb rawdata and other type of data according to specific protocol
 *
 * @attention    None
 *
 * @param[out]   puchRespondBuffer         pointer to frame data package
 * @param[out]   pusRespondLen             pointer to data length of package
 * @param[in]    puchFifoRawdata           pointer to encrypted rawdata
 * @param[in]    usFifoDataLen             gh3x2x fifo data len in byte
 * @param[in]    nAlgoResultArr            pointer to algorithm result
 * @param[in]    pstGsAxisValueArr         pointer to g sensor data
 * @param[in]    usGsDataNum               number of g sensor data
 * @param[in]    emGsSensitivity           sensitivity of g sensor
 *
 * @return    error code
 * @retval    #GH3X2X_RET_OK                        return successfully
 * @retval    #GH3X2X_RET_GENERIC_ERROR             return func start error
 * @retval    #GH3X2X_RET_NO_INITED_ERROR           gh3x2x has not inited
 */
GS8 GH3X2X_SendSpo2DumpDataPkgEx(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata, GU16 usFifoDataLen, 
                                GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
{
    GS8 chRet = GH3X2X_RET_OK;
    GU8 uchSpo2LevelArr[GH3X2X_SPO2_CHANNEL_MAP_MAX_CNT] = {0};
    if (GH3X2X_CHECK_BIT_SET(g_usFuncStartedBitmap, GH3X2X_STARTED_BITMAP_SPO2))
    {
        GH3X2X_BgLevelDecode(uchSpo2LevelArr, g_uchSpo2ChannelMapCnt, g_uchSpo2ChannelMapArr, g_usBgLevel);
        chRet = GH3X2X_SendDumpDataPkgWithAlgo(puchRespondBuffer, pusRespondLen, g_uchSpo2IncompleteArr, 
                                        g_unSpo2IncompleteDumpdataArr, puchFifoRawdata, usFifoDataLen,
                                        g_uchSpo2ChannelMapArr, g_uchSpo2ChannelMapCnt, uchSpo2LevelArr, 
                                        pstGsAxisValueArr, usGsDataNum, emGsSensitivity, GH3X2X_PTR_NULL, 
                                        GH3X2X_SPO2_ALGO_OUTPUT_NUM, GH3X2X_FUNC_OFFSET_SPO2);
    }
    else
    {
        chRet = GH3X2X_RET_GENERIC_ERROR;
        GH3X2X_DEBUG_LOG("spo2 hasn't started error!\r\n");
    }
    return chRet;
}      

/**
 * @fn     GS8 GH3X2X_SendZipHbDumpDataPkg(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
                                            GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                            GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *                  
 * @brief  pack GH3X2X Zip hb rawdata and other type of data according to specific protocol
 *
 * @attention   None
 *
 * @param[out]  puchRespondBuffer       pointer to frame data package
 * @param[out]  pusRespondLen           pointer to data length of package
 * @param[in]   puchFifoRawdata         pointer to encrypted rawdata
 * @param[in]   nAlgoResultArr          pointer to algorithm result
 * @param[in]   pstGsAxisValueArr       pointer to g sensor data
 * @param[in]   usGsDataNum             number of g sensor data
 * @param[in]   emGsSensitivity         sensitivity of g sensor
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return func start error
 * @retval  #GH3X2X_RET_NO_INITED_ERROR         gh3x2x has not inited
 */
GS8 GH3X2X_SendZipHbDumpDataPkg(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
                                            GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                            GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
{
    GS8 chRet = GH3X2X_RET_OK;
    GU8 uchHbaBgLevelArr[GH3X2X_HBA_CHANNEL_MAP_MAX_CNT] = {0};
    if (GH3X2X_CHECK_BIT_SET(g_usFuncStartedBitmap, GH3X2X_STARTED_BITMAP_HBA))
    {
        GU16 usReadDataLen = 0;
        GH3X2X_BgLevelDecode(uchHbaBgLevelArr, g_uchHbaChannelMapCnt, g_uchHbaChannelMapArr, g_usBgLevel);
        GH3X2X_DecryptRawdataBuffer(puchFifoRawdata, &usReadDataLen);
        chRet = GH3X2X_SendZipDumpDataPkgWithAlgo(puchRespondBuffer, pusRespondLen, g_uchHbaIncompleteArr, 
                                        g_unHbaIncompleteDumpdataArr, puchFifoRawdata, usReadDataLen,
                                        g_uchHbaChannelMapArr, g_uchHbaChannelMapCnt, uchHbaBgLevelArr, 
                                        pstGsAxisValueArr, usGsDataNum,
                                        emGsSensitivity, GH3X2X_PTR_NULL, GH3X2X_HB_ALGO_OUTPUT_NUM, GH3X2X_FUNC_OFFSET_HR);
        GH3X2X_EncryptRawdataBuffer(puchFifoRawdata, usReadDataLen);
    }
    else
    {
        chRet = GH3X2X_RET_GENERIC_ERROR;
        GH3X2X_DEBUG_LOG("hba hasn't started error!\r\n");
    }
    return chRet;
}

/**
 * @fn     GS8 GH3X2X_SendZipHbDumpDataPkgEx(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata, 
 *                                          GU16 usFifoDataLen, GS32 nAlgoResultArr[], 
 *                                          STGsensorRawdata *pstGsAxisValueArr, GU16 usGsDataNum, 
 *                                          EMGsensorSensitivity emGsSensitivity)
 *                  
 * @brief  pack GH3X2X Zip hb rawdata and other type of data according to specific protocol
 *
 * @attention   None
 *
 * @param[out]  puchRespondBuffer       pointer to frame data package
 * @param[out]  pusRespondLen           pointer to data length of package
 * @param[in]   puchFifoRawdata         pointer to encrypted rawdata
 * @param[in]   usFifoDataLen           gh3x2x fifo data len in byte
 * @param[in]   nAlgoResultArr          pointer to algorithm result
 * @param[in]   pstGsAxisValueArr       pointer to g sensor data
 * @param[in]   usGsDataNum             number of g sensor data
 * @param[in]   emGsSensitivity         sensitivity of g sensor
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return func start error
 * @retval  #GH3X2X_RET_NO_INITED_ERROR         gh3x2x has not inited
 */
GS8 GH3X2X_SendZipHbDumpDataPkgEx(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata, GU16 usFifoDataLen,
                                            GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                            GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
{
    GS8 chRet = GH3X2X_RET_OK;
    GU8 uchHbaBgLevelArr[GH3X2X_HBA_CHANNEL_MAP_MAX_CNT] = {0};
    if (GH3X2X_CHECK_BIT_SET(g_usFuncStartedBitmap, GH3X2X_STARTED_BITMAP_HBA))
    {
        GH3X2X_BgLevelDecode(uchHbaBgLevelArr, g_uchHbaChannelMapCnt, g_uchHbaChannelMapArr, g_usBgLevel);
        chRet = GH3X2X_SendZipDumpDataPkgWithAlgo(puchRespondBuffer, pusRespondLen, g_uchHbaIncompleteArr, 
                                        g_unHbaIncompleteDumpdataArr, puchFifoRawdata, usFifoDataLen,
                                        g_uchHbaChannelMapArr, g_uchHbaChannelMapCnt, uchHbaBgLevelArr, 
                                        pstGsAxisValueArr, usGsDataNum,
                                        emGsSensitivity, GH3X2X_PTR_NULL, GH3X2X_HB_ALGO_OUTPUT_NUM, GH3X2X_FUNC_OFFSET_HR);
    }
    else
    {
        chRet = GH3X2X_RET_GENERIC_ERROR;
        GH3X2X_DEBUG_LOG("hba hasn't started error!\r\n");
    }
    return chRet;
}
                                            /**
 * @fn     GS8 GH3X2X_SendZipHrvDumpDataPkg(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
                                            GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                            GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *                  
 * @brief  pack GH3X2X Zip hrv rawdata and other type of data according to specific protocol
 *
 * @attention   None
 *
 * @param[out]  puchRespondBuffer       pointer to frame data package
 * @param[out]  pusRespondLen           pointer to data length of package
 * @param[in]   puchFifoRawdata         pointer to encrypted rawdata
 * @param[in]   nAlgoResultArr          pointer to algorithm result
 * @param[in]   pstGsAxisValueArr       pointer to g sensor data
 * @param[in]   usGsDataNum             number of g sensor data
 * @param[in]   emGsSensitivity         sensitivity of g sensor
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return func start error
 * @retval  #GH3X2X_RET_NO_INITED_ERROR         gh3x2x has not inited
 */
GS8 GH3X2X_SendZipHrvDumpDataPkg(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
                                            GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                            GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
{
    GS8 chRet = GH3X2X_RET_OK;
    GU8 uchHrvBgLevelArr[GH3X2X_HRV_CHANNEL_MAP_MAX_CNT] = {0};
    if (GH3X2X_CHECK_BIT_SET(g_usFuncStartedBitmap, GH3X2X_STARTED_BITMAP_HRV))
    {
        GU16 usReadDataLen = 0;
        GH3X2X_BgLevelDecode(uchHrvBgLevelArr, g_uchHrvChannelMapCnt, g_uchHrvChannelMapArr, g_usBgLevel);
        GH3X2X_DecryptRawdataBuffer(puchFifoRawdata, &usReadDataLen);
        chRet = GH3X2X_SendZipDumpDataPkgWithAlgo(puchRespondBuffer, pusRespondLen, g_uchHrvIncompleteArr, 
                                        g_unHrvIncompleteDumpdataArr, puchFifoRawdata, usReadDataLen,
                                        g_uchHrvChannelMapArr, g_uchHrvChannelMapCnt, uchHrvBgLevelArr, 
                                        pstGsAxisValueArr, usGsDataNum,
                                        emGsSensitivity, GH3X2X_PTR_NULL, GH3X2X_HRV_ALGO_OUTPUT_NUM, GH3X2X_FUNC_OFFSET_HRV);
        GH3X2X_EncryptRawdataBuffer(puchFifoRawdata, usReadDataLen);
    }
    else
    {
        chRet = GH3X2X_RET_GENERIC_ERROR;
        GH3X2X_DEBUG_LOG("hba hasn't started error!\r\n");
    }
    return chRet;
}

/**
 * @fn     GS8 GH3X2X_SendZipHrvDumpDataPkgEx(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata, 
 *                                          GU16 usFifoDataLen, GS32 nAlgoResultArr[], 
 *                                          STGsensorRawdata *pstGsAxisValueArr, GU16 usGsDataNum, 
 *                                          EMGsensorSensitivity emGsSensitivity)
 *                  
 * @brief  pack GH3X2X Zip hrv rawdata and other type of data according to specific protocol
 *
 * @attention   None
 *
 * @param[out]  puchRespondBuffer       pointer to frame data package
 * @param[out]  pusRespondLen           pointer to data length of package
 * @param[in]   puchFifoRawdata         pointer to encrypted rawdata
 * @param[in]   usFifoDataLen           gh3x2x fifo data len in byte
 * @param[in]   nAlgoResultArr          pointer to algorithm result
 * @param[in]   pstGsAxisValueArr       pointer to g sensor data
 * @param[in]   usGsDataNum             number of g sensor data
 * @param[in]   emGsSensitivity         sensitivity of g sensor
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return func start error
 * @retval  #GH3X2X_RET_NO_INITED_ERROR         gh3x2x has not inited
 */
GS8 GH3X2X_SendZipHrvDumpDataPkgEx(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata, GU16 usFifoDataLen,
                                            GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                            GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
{
    GS8 chRet = GH3X2X_RET_OK;
    GU8 uchHrvBgLevelArr[GH3X2X_HRV_CHANNEL_MAP_MAX_CNT] = {0};
    if (GH3X2X_CHECK_BIT_SET(g_usFuncStartedBitmap, GH3X2X_STARTED_BITMAP_HRV))
    {
        GH3X2X_BgLevelDecode(uchHrvBgLevelArr, g_uchHrvChannelMapCnt, g_uchHrvChannelMapArr, g_usBgLevel);
        chRet = GH3X2X_SendZipDumpDataPkgWithAlgo(puchRespondBuffer, pusRespondLen, g_uchHrvIncompleteArr, 
                                        g_unHrvIncompleteDumpdataArr, puchFifoRawdata, usFifoDataLen,
                                        g_uchHrvChannelMapArr, g_uchHrvChannelMapCnt, uchHrvBgLevelArr, 
                                        pstGsAxisValueArr, usGsDataNum,
                                        emGsSensitivity, GH3X2X_PTR_NULL, GH3X2X_HRV_ALGO_OUTPUT_NUM, GH3X2X_FUNC_OFFSET_HRV);
    }
    else
    {
        chRet = GH3X2X_RET_GENERIC_ERROR;
        GH3X2X_DEBUG_LOG("hrv hasn't started error!\r\n");
    }
    return chRet;
}

/**
 * @fn       GS8 GH3X2X_SendZipSpo2DumpDataPkg(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
                                            GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                            GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *                    
 * @brief  pack GH3X2X Zip spo2 rawdata and other type of data according to specific protocol
 *
 * @attention    None
 *
 * @param[out]   puchRespondBuffer         pointer to frame data package
 * @param[out]   pusRespondLen             pointer to data length of package
 * @param[in]    puchFifoRawdata           pointer to encrypted rawdata
 * @param[in]    nAlgoResultArr            pointer to algorithm result
 * @param[in]    pstGsAxisValueArr         pointer to g sensor data
 * @param[in]    usGsDataNum               number of g sensor data
 * @param[in]    emGsSensitivity           sensitivity of g sensor
 *
 * @return    error code
 * @retval    #GH3X2X_RET_OK                        return successfully
 * @retval    #GH3X2X_RET_GENERIC_ERROR             return func start error
 * @retval    #GH3X2X_RET_NO_INITED_ERROR           gh3x2x has not inited
 */
GS8 GH3X2X_SendZipSpo2DumpDataPkg(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
                                GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
{
    GS8 chRet = GH3X2X_RET_OK;
    GU8 uchSpo2LevelArr[GH3X2X_SPO2_CHANNEL_MAP_MAX_CNT] = {0};
    if (GH3X2X_CHECK_BIT_SET(g_usFuncStartedBitmap, GH3X2X_STARTED_BITMAP_SPO2))
    {
        GU16 usReadDataLen = 0;
        GH3X2X_BgLevelDecode(uchSpo2LevelArr, g_uchSpo2ChannelMapCnt, g_uchSpo2ChannelMapArr, g_usBgLevel);
        GH3X2X_DecryptRawdataBuffer(puchFifoRawdata, &usReadDataLen);
        chRet = GH3X2X_SendZipDumpDataPkgWithAlgo(puchRespondBuffer, pusRespondLen, g_uchSpo2IncompleteArr, 
                                        g_unSpo2IncompleteDumpdataArr, puchFifoRawdata, usReadDataLen,
                                        g_uchSpo2ChannelMapArr, g_uchSpo2ChannelMapCnt, uchSpo2LevelArr, 
                                        pstGsAxisValueArr, usGsDataNum, emGsSensitivity, GH3X2X_PTR_NULL, 
                                        GH3X2X_SPO2_ALGO_OUTPUT_NUM, GH3X2X_FUNC_OFFSET_SPO2);
        GH3X2X_EncryptRawdataBuffer(puchFifoRawdata, usReadDataLen);
    }
    else
    {
        chRet = GH3X2X_RET_GENERIC_ERROR;
        GH3X2X_DEBUG_LOG("spo2 hasn't started error!\r\n");
    }
    return chRet;
}

/**
 * @fn       GS8 GH3X2X_SendZipSpo2DumpDataPkgEx(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata, 
 *                              GU16 usFifoDataLen, GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *                    
 * @brief  pack GH3X2X Zip spo2 rawdata and other type of data according to specific protocol
 *
 * @attention    None
 *
 * @param[out]   puchRespondBuffer         pointer to frame data package
 * @param[out]   pusRespondLen             pointer to data length of package
 * @param[in]    puchFifoRawdata           pointer to encrypted rawdata
 * @param[in]    usFifoDataLen             gh3x2x fifo data len in byte
 * @param[in]    nAlgoResultArr            pointer to algorithm result
 * @param[in]    pstGsAxisValueArr         pointer to g sensor data
 * @param[in]    usGsDataNum               number of g sensor data
 * @param[in]    emGsSensitivity           sensitivity of g sensor
 *
 * @return    error code
 * @retval    #GH3X2X_RET_OK                        return successfully
 * @retval    #GH3X2X_RET_GENERIC_ERROR             return func start error
 * @retval    #GH3X2X_RET_NO_INITED_ERROR           gh3x2x has not inited
 */
GS8 GH3X2X_SendZipSpo2DumpDataPkgEx(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata, GU16 usFifoDataLen, 
                                GS32 nAlgoResultArr[], STGsensorRawdata *pstGsAxisValueArr,
                                GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
{
    GS8 chRet = GH3X2X_RET_OK;
    GU8 uchSpo2LevelArr[GH3X2X_SPO2_CHANNEL_MAP_MAX_CNT] = {0};
    if (GH3X2X_CHECK_BIT_SET(g_usFuncStartedBitmap, GH3X2X_STARTED_BITMAP_SPO2))
    {
        GH3X2X_BgLevelDecode(uchSpo2LevelArr, g_uchSpo2ChannelMapCnt, g_uchSpo2ChannelMapArr, g_usBgLevel);
        chRet = GH3X2X_SendZipDumpDataPkgWithAlgo(puchRespondBuffer, pusRespondLen, g_uchSpo2IncompleteArr, 
                                        g_unSpo2IncompleteDumpdataArr, puchFifoRawdata, usFifoDataLen,
                                        g_uchSpo2ChannelMapArr, g_uchSpo2ChannelMapCnt, uchSpo2LevelArr, 
                                        pstGsAxisValueArr, usGsDataNum, emGsSensitivity, GH3X2X_PTR_NULL, 
                                        GH3X2X_SPO2_ALGO_OUTPUT_NUM, GH3X2X_FUNC_OFFSET_SPO2);
    }
    else
    {
        chRet = GH3X2X_RET_GENERIC_ERROR;
        GH3X2X_DEBUG_LOG("spo2 hasn't started error!\r\n");
    }
    return chRet;
}     
#endif
/**
 * @fn         GS8 GH3X2X_DumpRawdataFilter(GU8* puchRawDataBuffer, GU16* usRawDataLen, GU8 punLastIncompleteFlagArr[], 
                            GU32 punIncompleteDumpdataArr[][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL], GU8* puchReadFifoBuffer, 
                            GU16 usFifoLen, GU16 usBgLevel)
 *                      
 * @brief  filter dump fifo data to get rawdata
 *
 * @attention     None
 *
 * @param[out]     puchRawDataBuffer           pointer to rawdata buffer
 * @param[out]     usRawDataLen               pointer to rawdata number
 * @param[out]     punLastIncompleteFlagArr  pointer to incomplete flag array
 * @param[out]     punIncompleteDumpdataArr  pointer to incomplete data
 * @param[in]     puchReadFifoBuffer           pointer to fifo data buffer
 * @param[in]     usFifoLen                 fifo data num
 * @param[in]     usBgLevel                 slot bg level
 *
 * @return      error code
 * @retval      #GH3X2X_RET_OK                        return successfully
 * @retval      #GH3X2X_RET_PARAMETER_ERROR             return parameter error
 */
GS8 GH3X2X_DumpRawdataFilter(GU8* puchRawDataBuffer, GU16* usRawDataLen, GU8 punLastIncompleteFlagArr[], 
                            GU32 punIncompleteDumpdataArr[][GH3X2X_MAX_DUMP_DATA_NUM_CHANNEL], GU8* puchReadFifoBuffer, 
                            GU16 usFifoLen, GU16 usBgLevel)
{
    GS8  schRetValue           = GH3X2X_RET_OK;
    GU16 usDataIndex           = 0;
    GU8  uchSlotIndex          = 0;
    GU8  uchAdcIndex           = 0;
    GU8  uchChIndex            = 0;
    GU8  uchBgLevel            = 0;
    GU8  puchDumpDataCntArr[GH3X2X_CHANNEL_MAP_MAX_CH] = {0};        //save dump data number of every channel
    GU8  uchDumpDataCnt        = 0;
    GU8  uchDumpDataNum        = 0;                                  //dump data num of correspond bg level
    GU32 unPpgDataFilter       = 0;

    if ((GH3X2X_PTR_NULL == puchRawDataBuffer) || (GH3X2X_PTR_NULL == usRawDataLen) || \
        (GH3X2X_PTR_NULL == punLastIncompleteFlagArr) || (GH3X2X_PTR_NULL == punIncompleteDumpdataArr) || \
        (GH3X2X_PTR_NULL ==  puchReadFifoBuffer) || (0 ==  usFifoLen))
    {
        usRawDataLen = 0;
        schRetValue = GH3X2X_RET_PARAMETER_ERROR;
        return schRetValue;
    }
        
    //get current dump data index of every channel by count incomplete data
    for (uchChIndex = 0; uchChIndex < GH3X2X_CHANNEL_MAP_MAX_CH; ++uchChIndex)
    {
        //loop time can be decided by bg level
        uchBgLevel = g_uchChannelBgLvlArr[uchChIndex];
        uchDumpDataNum = g_uchDumpDataCntMapArr[uchBgLevel];
        for (uchDumpDataCnt = 0; uchDumpDataCnt < uchDumpDataNum; ++uchDumpDataCnt)
        {
            //if imcomplete bit flag is 1
            if (GH3X2X_CHECK_LEFT_BIT_SET(punLastIncompleteFlagArr[uchChIndex], uchDumpDataCnt))
            {
                puchDumpDataCntArr[uchChIndex]++;
                if (puchDumpDataCntArr[uchChIndex] >= uchDumpDataNum)
                {
                    GH3X2X_DEBUG_LOG("Cache incomplete data too much!\r\n");
                }
            }
            else
            {
                break;
            }
        }
    }

    *usRawDataLen = 0;
    
    //filter every rawdata
    for (usDataIndex = 0; usDataIndex < usFifoLen; usDataIndex += GH3X2X_FIFO_RAWDATA_SIZE)
    {
        //decode slot adc index
        uchChIndex   = GH3X2X_BYTE_RAWDATA_GET_SLOT_ADC_NUM(puchReadFifoBuffer[usDataIndex]);
        uchSlotIndex = GH3X2X_BYTE_RAWDATA_GET_SLOT_NUM(puchReadFifoBuffer[usDataIndex]);
        uchAdcIndex  = GH3X2X_BYTE_RAWDATA_GET_ADC_NUM(puchReadFifoBuffer[usDataIndex]);
        
        if (GH3X2X_CHECK_LEFT_BIT_SET(g_uchEcgSlot, uchSlotIndex))
        {
            //ECG data not need to filter
            if (uchAdcIndex == GH3X2X_ECG_ADC_INDEX)
            {
                GH3X2X_Memcpy(&puchRawDataBuffer[*usRawDataLen], &puchReadFifoBuffer[usDataIndex], \
                                GH3X2X_FIFO_RAWDATA_SIZE);
                *usRawDataLen += GH3X2X_FIFO_RAWDATA_SIZE;
                continue;
            }
        }
        //decode bg level
        uchBgLevel     = g_uchChannelBgLvlArr[uchChIndex];
        uchDumpDataNum = g_uchDumpDataCntMapArr[uchBgLevel];
        //save them to implete array
        GH3X2X_Memcpy(&punIncompleteDumpdataArr[uchChIndex][puchDumpDataCntArr[uchChIndex]], \
                        &puchReadFifoBuffer[usDataIndex], GH3X2X_FIFO_RAWDATA_SIZE);
        //set incomplete flag
        punLastIncompleteFlagArr[uchChIndex] |= (GU8)GH3X2X_GET_LEFT_SHIFT_VAL(puchDumpDataCntArr[uchChIndex]);
        puchDumpDataCntArr[uchChIndex]++;        
        //if save enough,calculate it,and save it to g_uchRawdataFilterArr,and clear incomplete flag
        if (puchDumpDataCntArr[uchChIndex] >= uchDumpDataNum)
        {
            switch (uchBgLevel)
            {
            case GH3X2X_DUMP_RAWDATA_BGLVL0:
                GH3X2X_Memcpy(&puchRawDataBuffer[*usRawDataLen], &punIncompleteDumpdataArr[uchChIndex][1], \
                        GH3X2X_FIFO_RAWDATA_SIZE);
                break;
            case GH3X2X_DUMP_RAWDATA_BGLVL1:
                unPpgDataFilter = GH3X2X_DumpRawDataPro(punIncompleteDumpdataArr[uchChIndex][2], \
                                                        punIncompleteDumpdataArr[uchChIndex][1], \
                                                        punIncompleteDumpdataArr[uchChIndex][1]);
                GH3X2X_Memcpy(&puchRawDataBuffer[*usRawDataLen], &unPpgDataFilter, GH3X2X_FIFO_RAWDATA_SIZE);
                break;
            case GH3X2X_DUMP_RAWDATA_BGLVL2:
                unPpgDataFilter = GH3X2X_DumpRawDataPro(punIncompleteDumpdataArr[uchChIndex][2], \
                                                        punIncompleteDumpdataArr[uchChIndex][1], \
                                                        punIncompleteDumpdataArr[uchChIndex][3]);
                GH3X2X_Memcpy(&puchRawDataBuffer[*usRawDataLen], &unPpgDataFilter, GH3X2X_FIFO_RAWDATA_SIZE);
                break;
            case GH3X2X_DUMP_RAWDATA_BGLVL2X2:
                unPpgDataFilter = GH3X2X_DumpRawDataPro(punIncompleteDumpdataArr[uchChIndex][0], \
                                                        punIncompleteDumpdataArr[uchChIndex - GH3X2X_ADC_NUM][3], \
                                                        punIncompleteDumpdataArr[uchChIndex][1]);
                GH3X2X_Memcpy(&puchRawDataBuffer[*usRawDataLen], &unPpgDataFilter, GH3X2X_FIFO_RAWDATA_SIZE);
                break;
            default:
                break;
            }
            *usRawDataLen += GH3X2X_FIFO_RAWDATA_SIZE;
            puchDumpDataCntArr[uchChIndex] = 0;
            punLastIncompleteFlagArr[uchChIndex] = 0;
        }  // end if (puchDumpDataCntArr[uchChIndex] >= uchDumpDataNum)  
    } // end for (usDataIndex = 0; usDataIndex < usFifoLen; usDataIndex += GH3X2X_FIFO_RAWDATA_SIZE)
    return schRetValue;
}

/**
 * @fn     void GH3X2X_LedAgcProcessExDump(GU8* puchReadFifoBuffer)
 * 
 * @brief  led agc process function
 *
 * @attention   None
 *
 * @param[in]   puchReadFifoBuffer         pointer to read fifo data buffer
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_LedAgcProcessExDump(GU8* puchReadFifoBuffer)
{
    GU16 usFifoLen    = 0;
    GU16 usRawdataLen = 0;
    GS8  schRetValue  = GH3X2X_RET_OK;
    if (GH3X2X_LED_AGC_ENABLE != GH3X2X_GetLedAgcState())
    {
        return;
    }
    GH3X2X_Memset(g_uchRawdataFilterArr, 0, RAWDATA_FILTER_ARRAY_BYTE_LENGTH * sizeof(g_uchRawdataFilterArr[0]));
    //filter dump data,because AGC only process raw data
    schRetValue = GH3X2X_DumpRawdataFilter(g_uchRawdataFilterArr, &usRawdataLen, g_uchDumpIncompleteFlagArr, \
                            g_unDumpIncompleteRawdataArr, puchReadFifoBuffer, usFifoLen, g_usBgLevel);
    if (GH3X2X_RET_OK == schRetValue)
    {
        GH3X2X_LedAgcProcess(g_uchRawdataFilterArr, usRawdataLen);
    }    
}

#endif
/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/
