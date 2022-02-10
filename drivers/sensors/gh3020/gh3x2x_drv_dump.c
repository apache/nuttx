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
#include "gh3020_bridge.h"
#include "gh3x2x_drv_control.h"
#include "gh3x2x_drv_control_ex.h"
#include "gh3x2x_drv_dump.h"
#include "gh3x2x_drv_soft_led_agc.h"

/// dump mode
uint16_t g_usDumpMode;

/// electrode wear status
//uint8_t g_uchElectrodeWearStatus;

/// save bg cancel value of every channel
uint8_t  g_puchGainBgCancelRecord[GH3X2X_CHANNEL_MAP_MAX_CH];


uint8_t g_puchTiaGainAfterSoftAgc[GH3X2X_CHANNEL_MAP_MAX_CH];

/// save current of drv0 and drv1
uint16_t g_pusDrvCurrentRecord[GH3X2X_CHANNEL_MAP_MAX_CH];

/**
 * @fn     uint16_t GH3X2X_DumpModeGet(void)
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
uint16_t GH3X2X_DumpModeGet(void)
{
    return g_usDumpMode;
}

/**
 * @fn     void GH3X2X_DumpModeSet(uint16_t usRegVal)
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
void GH3X2X_DumpModeSet(uint16_t usRegVal)
{
    g_usDumpMode = usRegVal;
}


const uint16_t g_usGH3x2xSlotRegBase[8] =
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

const uint16_t g_usGH3x2xTiaGainR[] =
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

void GH3x2xSetAgcReg(uint8_t uchAgcReg,  uint8_t uchSlot, uint16_t usValue)
{
    gh3020_spi_writebits(g_usGH3x2xSlotRegBase[uchSlot] + g_pstGH3x2xAgcReg[uchAgcReg].uchAddrOffset, g_pstGH3x2xAgcReg[uchAgcReg].uchLsb, g_pstGH3x2xAgcReg[uchAgcReg].uchMsb, usValue);
}

uint16_t GH3x2xGetAgcReg(uint8_t uchAgcReg,  uint8_t uchSlot)
{
    return gh3020_spi_readbits(g_usGH3x2xSlotRegBase[uchSlot] + g_pstGH3x2xAgcReg[uchAgcReg].uchAddrOffset, g_pstGH3x2xAgcReg[uchAgcReg].uchLsb, g_pstGH3x2xAgcReg[uchAgcReg].uchMsb);
}

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
    uint8_t  uchSlotIndexCnt  = 0;
    uint8_t  uchAdcIndexCnt   = 0;
    uint8_t  uchSlotAdcIndex  = 0;
    uint16_t usDrv0CurRegData = 0;
    uint16_t usDrv0CurRegAddr = 0;
    uint16_t usDrv1CurRegData = 0;
    uint16_t usDrv1CurRegAddr = 0;

    if (GH3X2X_LED_AGC_ENABLE == GH3X2X_GetLedAgcState() && \
        (GH3X2X_CHECK_LEFT_BIT_NOT_SET(g_usDumpMode, GH3X2X_DUMP_GAIN_CURRENT_ENABLE_BIT)) && \
        (GH3X2X_CHECK_LEFT_BIT_NOT_SET(g_usDumpMode, GH3X2X_DUMP_AMBIANCE_ENABLE_BIT)))
    {
        GH3X2X_WAIT_CHIP_WAKEUP();
        usDrv0CurRegAddr = GH3020_REG_SLOT0_CTRL_10;
        usDrv1CurRegAddr = GH3020_REG_SLOT0_CTRL_11;
        for (uchSlotIndexCnt = 0; uchSlotIndexCnt < GH3X2X_MAX_SLOT_NUM; uchSlotIndexCnt++)
        {
            usDrv0CurRegData = (gh3020_spi_readreg(usDrv0CurRegAddr) & GH3X2X_DRV_CURRENT_BIT_MASK);
            usDrv1CurRegData = (gh3020_spi_readreg(usDrv1CurRegAddr) & GH3X2X_DRV_CURRENT_BIT_MASK);
            for (uchAdcIndexCnt = 0; uchAdcIndexCnt < GH3X2X_ADC_NUM; uchAdcIndexCnt++)
            {
                g_pusDrvCurrentRecord[uchSlotAdcIndex] =  (uint16_t)(usDrv0CurRegData << \
                                                                    GH3X2X_DRV0_CURRENT_LEFT_SHIFT_BIT_NUM);
                g_pusDrvCurrentRecord[uchSlotAdcIndex] |= (uint16_t)(usDrv1CurRegData << \
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

/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/
