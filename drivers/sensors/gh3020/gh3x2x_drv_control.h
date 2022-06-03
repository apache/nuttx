/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_control.h
 *
 * @brief   gh3x2x control define & declaration
 *
 * @version ref gh3x2x_drv_version.h
 *
 */

#ifndef _GH3X2X_DRV_CONTROL_H_
#define _GH3X2X_DRV_CONTROL_H_


#include "gh3x2x_drv_config.h"
#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"


/**
 * @brief status flag enum
 */
enum EMGh3x2xStatus
{
    GH3X2X_STATUS_NO_INIT = 0,  /**< no init status */
    GH3X2X_STATUS_INITED,       /**< inited status */
    GH3X2X_STATUS_STARTED,      /**< started status */
    GH3X2X_STATUS_INVALID,      /**< invaild status */
};


/* function ptr typedef */

typedef void (*pfnNormalHookFunc)(void);                                                 /**< normal hook type */
typedef void (*pfnGetRawdataHookFunc)(uint8_t *puchDataBuffer, uint16_t usLen);                  /**< get rawdata hook type */
typedef uint8_t (*pfnReadPinStatus)(void);                                                   /**< read pin status type */
typedef void (*pfnSetPinLevel)(uint8_t uchPinLevel);                                         /**< set pin status type */


/// get slot & adc num from rawdata flag byte, equal channel map formats
#define   GH3X2X_CHANNEL_MAP_GET(x)                     ((x) & GH3X2X_SLOT_ADC_NUM_BITS)

/// xor val for xor num in encrypt & decrypt
#define   GH3X2X_CRYP_XOR_XOR_VAL                       (0xFF)

/// send wakeup(resume) cmd
#define   GH3X2X_SEND_WAKEUP_CMD()          do { gh3020_spi_sendcmd(GH3020_SPI_CMD_RESUME); } while (0)

/// send sleep cmd
#define   GH3X2X_SEND_DSLEEP_CMD()          do { gh3020_spi_sendcmd(GH3020_SPI_CMD_SLEEP); } while (0)

/// send reset cmd
#define   GH3X2X_SEND_RESET_CMD()           do { gh3020_spi_sendcmd(GH3020_SPI_CMD_RESET); } while (0)

/// wait send cmd done
#define   GH3X2X_SEND_WAIT_CMD_DONE()       do { usleep(GH3020_CMD_DELAY_US); } while (0)

/// chip wakeup
#define   GH3X2X_CHIP_WAKEUP(ret)       do {\
                                            GH3X2X_SEND_WAKEUP_CMD();\
                                            GH3X2X_SEND_WAIT_CMD_DONE();\
                                            GH3X2X_PMU_FIFO_POWER_RECOVERY();\
                                            (ret) = GH3X2X_RET_OK;\
                                        } while (0)

/// chip sleep
#define   GH3X2X_CHIP_SLEEP(ret)        do {\
                                            GH3X2X_PMU_FIFO_POWER_OFF();\
                                            GH3X2X_SEND_DSLEEP_CMD();\
                                            GH3X2X_SEND_WAIT_CMD_DONE();\
                                            (ret) = GH3X2X_RET_OK;\
                                        } while (0)

/// chip sleep with not waiting
#define   GH3X2X_CHIP_SLEEP_NOWAIT(ret) do {\
                                            GH3X2X_PMU_FIFO_POWER_OFF();\
                                            GH3X2X_SEND_DSLEEP_CMD();\
                                            (ret) = GH3X2X_RET_OK;\
                                        } while (0)

/// chip reset
#define   GH3X2X_CHIP_RESET(ret)        do {\
                                            GH3X2X_SEND_RESET_CMD();\
                                            (ret) = GH3X2X_RET_OK;\
                                        } while (0)

/// chip wakeup
#define   GH3X2X_CHIP_WAKEUP_NORET()    do {\
                                            GH3X2X_SEND_WAKEUP_CMD();\
                                            GH3X2X_SEND_WAIT_CMD_DONE();\
                                            GH3X2X_PMU_FIFO_POWER_RECOVERY();\
                                        } while (0)

/// chip sleep
#define   GH3X2X_CHIP_SLEEP_NORET()     do {\
                                            GH3X2X_PMU_FIFO_POWER_OFF();\
                                            GH3X2X_SEND_DSLEEP_CMD();\
                                            GH3X2X_SEND_WAIT_CMD_DONE();\
                                        } while (0)

/// chip reset
#define   GH3X2X_CHIP_RESET_NORET()     do {\
                                            GH3X2X_SEND_RESET_CMD();\
                                        } while (0)

#if GH3X2X_CTRL_API_INSERT_WAKEUP_AND_DSLP_ENABLED

/// chip wakeup
#define   GH3X2X_WAIT_CHIP_WAKEUP()     do {\
                                            GH3X2X_SEND_WAKEUP_CMD();\
                                            GH3X2X_SEND_WAIT_CMD_DONE();\
                                            GH3X2X_PMU_FIFO_POWER_RECOVERY();\
                                        } while (0)

/// chip sleep
#define   GH3X2X_WAIT_CHIP_DSLP()       do {\
                                            GH3X2X_PMU_FIFO_POWER_OFF();\
                                            GH3X2X_SEND_DSLEEP_CMD();\
                                            GH3X2X_SEND_WAIT_CMD_DONE();\
                                        } while (0)
#else

/// chip wakeup
#define   GH3X2X_WAIT_CHIP_WAKEUP()

/// chip sleep
#define   GH3X2X_WAIT_CHIP_DSLP()

#endif

#if GH3X2X_PMU_WAIT_PULSE_OFF_ENABLED

/// wait pulse mode off
#define GH3X2X_PMU_WAIT_PULSE_MODE_OFF()    do {\
                                                GH3X2X_SEND_DSLEEP_CMD();\
                                                GH3X2X_SEND_WAIT_CMD_DONE();\
                                                GH3X2X_SEND_WAKEUP_CMD();\
                                                GH3X2X_SEND_WAIT_CMD_DONE();\
                                                GH3X2X_SEND_WAIT_CMD_DONE();\
                                            } while (0)

#else

/// wait pulse mode off
#define GH3X2X_PMU_WAIT_PULSE_MODE_OFF()    do {} while (0)

#endif

/* reg addr control bit macro */

/// reg addr[11:0] valid
#define   GH3X2X_GET_REG_REAL_ADRR(x)       ((x) & 0x0FFF)

/* virtual reg addr mcaro */

/// software config reg end addr, software addr map [0x0000~0x09FE]
#define   GH3X2X_VIRTUAL_REG_SW_CONFIG_END_ADDR         (0x29FF)

/// algorithm config reg end addr, algorithm addr map [0x0A00~0x0EFE]
#define   GH3X2X_VIRTUAL_REG_ALGO_CONFIG_END_ADDR       (0x2EFF)

/* virtual reg for algo config */
#if   GH3X2X_VIRTUAL_REG_FUNCTION_CONFIG_SUPPORT
/// write algorithm config function
#define   GH3X2X_FUNCTION_CONFIG_WRITE(...)       do { GH3X2X_WriteFunctionConfigWithVirtualReg(__VA_ARGS__); } while (0)
/// write algorithm config function
#define   GH3X2X_FUNCTION_CONFIG_READ(...)      //(GH3X2X_ReadAlgorithmConfigWithVirtualReg(__VA_ARGS__))
#else
/// write algorithm config function
#define   GH3X2X_FUNCTION_CONFIG_WRITE(...)
/// write algorithm config function
#define   GH3X2X_FUNCTION_CONFIG_READ(...)
#endif

/* virtual reg for software config */
#if   GH3X2X_VIRTUAL_REG_SW_CONFIG_SUPPORT
/// write software config function
#define   GH3X2X_SW_CONFIG_WRITE(...)         do { GH3X2X_WriteSwConfigWithVirtualReg(__VA_ARGS__); } while (0)
/// write software config function
#define   GH3X2X_SW_CONFIG_READ(...)        //(GH3X2X_ReadSwConfigWithVirtualReg(__VA_ARGS__))
#else
/// write software config function
#define   GH3X2X_SW_CONFIG_WRITE(...)
/// write software config function
#define   GH3X2X_SW_CONFIG_READ(...)
#endif

/* reg instructions chip inited macro */
#if   GH3X2X_INSTRUCTIONS_CHIP_INIED_ENABLED

/// set chip inited flag into reg
#define   GH3X2X_SET_CHIP_INIED()    do {\
                                        uint16_t usChipIniedRegValTmp =\
                                            gh3020_spi_readreg(GH3020_REG_INSTRUCTIONS_CHIP_INIED);\
                                        GH3X2X_DEBUG_LOG_PARAM("[%s]:before usChipIniedRegValTmp = 0x%04x\r\n", __FUNCTION__, usChipIniedRegValTmp);\
                                            gh3020_spi_writereg(GH3020_REG_INSTRUCTIONS_CHIP_INIED, \
                                                usChipIniedRegValTmp | GH3020_REGVAL_INSTRUCT_CHIP_INIED);\
                                        usChipIniedRegValTmp =\
                                            gh3020_spi_readreg(GH3020_REG_INSTRUCTIONS_CHIP_INIED);\
                                        GH3X2X_DEBUG_LOG_PARAM("[%s]:after usChipIniedRegValTmp = 0x%04x\r\n", __FUNCTION__, usChipIniedRegValTmp);\
                                     } while (0)

/// read chip inited flag from reg
#define   GH3X2X_IS_CHIP_INIED()    ((gh3020_spi_readreg(GH3020_REG_INSTRUCTIONS_CHIP_INIED) &\
                                        GH3020_REGVAL_INSTRUCT_CHIP_INIED) == \
                                        GH3020_REGVAL_INSTRUCT_CHIP_INIED)
#else

/// set chip inited flag into reg
#define   GH3X2X_SET_CHIP_INIED()

/// read chip inited flag from reg
#define   GH3X2X_IS_CHIP_INIED()    (1)

#endif

#if     GH3X2X_PMU_FIFO_POWER_CTRL_ENABLED

/// store fifo power status
#define  GH3X2X_PMU_FIFO_POWER_OFF()    do {\
                                            if (g_uchPmuFifoModuleCtrlCacheStatus == 0)\
                                            {\
                                                if ((g_uchGh3x2xStatus == GH3X2X_STATUS_INITED) || \
                                                    (g_uchGh3x2xStatus == GH3X2X_STATUS_NO_INIT)) \
                                                {\
                                                    g_usPmuFifoModuleCtrlVal = \
                                                            gh3020_spi_readreg(GH3020_REG_PMU_CTRL4);\
                                                    gh3020_spi_writereg(GH3020_REG_PMU_CTRL4, \
                                                     (g_usPmuFifoModuleCtrlVal | GH3020_MSK_PMU_CTRL4_FIFO_DISABLE));\
                                                    g_uchPmuFifoModuleCtrlCacheStatus = 1;\
                                                    GH3X2X_DEBUG_LOG_INTERNAL("save pmu fifo module status!\r\n");\
                                                }\
                                            }\
                                        } while (0)

/// set chip inited flag into reg
#define  GH3X2X_PMU_FIFO_POWER_RECOVERY()       do {\
                                                    if (g_uchPmuFifoModuleCtrlCacheStatus == 1)\
                                                    {\
                                                        gh3020_spi_writereg(GH3020_REG_PMU_CTRL4, \
                                                                            g_usPmuFifoModuleCtrlVal);\
                                                        GH3X2X_PMU_WAIT_PULSE_MODE_OFF();\
                                                        g_uchPmuFifoModuleCtrlCacheStatus = 0;\
                                                        GH3X2X_DEBUG_LOG_INTERNAL(\
                                                                            "recovery pmu fifo module status!\r\n");\
                                                    }\
                                                } while (0)

#else

/// store fifo power status
#define   GH3X2X_PMU_FIFO_POWER_OFF()

/// set chip inited flag into reg
#define   GH3X2X_PMU_FIFO_POWER_RECOVERY()

#endif

/// wear detect force switch delay
#define GH3X2X_WEAR_DETECT_SWITCH_WAIT_X_US             (150)

/// hard reset delay
#define   GH3X2X_HARD_RESET_DELAY_X_US                  (100)

/// fifo watermark thr min val
#define   GH3X2X_FIFO_WATERMARK_THR_MIN                 (3)

/// fifo watermark thr max val
#define   GH3X2X_FIFO_WATERMARK_THR_MAX                 (800)

/// irq width val
#define   GH3X2X_IRQ_WIDTH_VAL_MAX                      (2047)

/* osc cali */

/// osc calibration code lock flag
#define   GH3X2X_OSC_CALI_CODE_LOACKED                  (0x0001)

/// osc calibration Ctrl disable
#define   GH3X2X_OSC_CALI_CTRL_DIS_VAL                  (0x0000)

/// hsi osc calibration delay
#define   GH3X2X_HSI_CALI_DELAY_VAL                     (1000u)

/// hsi osc calibration theroy count
#define   GH3X2X_HSI_CALI_THR_CNT_VAL                   (0x0196)

/// hsi osc calibration Ctrl enable
#define   GH3X2X_HSI_CALI_CTRL_EN_VAL                   (0x0271)

/// lsi osc calibration theroy count
#define   GH3X2X_LSI_CALI_THR_CNT_VAL                   (0x1F40)

/// lsi osc calibration ctrl coarse enable
#define   GH3X2X_LSI_CALI_CTRL_C_EN_VAL                 (0x09D7)

/// lsi osc calibration ctrl fine enable
#define   GH3X2X_LSI_CALI_CTRL_F_EN_VAL                 (0x19D3)

/// lsi osc calibration error val max
#define   GH3X2X_LSI_CALI_ERR_MAX_VAL                   (0x3FFF)

/// lsi osc calibration error val fixed val
#define   GH3X2X_LSI_CALI_ERR_FIXED_VAL                 (0x8000)

/// lsi osc calibration fine error max
#define   GH3X2X_LSI_CALI_FINE_VAL_MAX                  (255u)

/// lsi osc calibration fine tune max cnt
#define   GH3X2X_LSI_CALI_FINE_TUNE_MAX                 (9u)

/// lsi osc calibration fine div num
#define   GH3X2X_LSI_CALI_FINE_DIV_NUM                  (2u)

/// lsi osc calibration delay
#define   GH3X2X_LSI_COR_CALI_DELAY_VAL                 (50000u)

/// lsi osc calibration delay
#define   GH3X2X_LSI_FINE_CALI_DELAY_VAL                (5000u)

/**< rawdata buffer size in byte */
#define   GH3X2X_RAWDATA_BUFFER_SIZE                    (200*4)

/* extern variables declaration */

/// read int pin status func ptr
extern pfnReadPinStatus g_pGh3x2xReadIntPinStatusFunc;

/// read reset pin status func ptr
extern pfnReadPinStatus g_pGh3x2xReadResetPinStatusFunc;

/// read spcs pin status func ptr
extern pfnReadPinStatus g_pGh3x2xReadSpcsPinStatusFunc;

/// read spdo pin status func ptr
extern pfnReadPinStatus g_pGh3x2xReadSpdoPinStatusFunc;

#if GH3X2X_PMU_FIFO_POWER_CTRL_ENABLED

/// fifo power ctrl fifo control reg cache var
extern uint16_t g_usPmuFifoModuleCtrlVal;

/// for fifo power ctrl cache status
extern uint8_t g_uchPmuFifoModuleCtrlCacheStatus;

#endif

/// gh3x2x status
extern uint8_t g_uchGh3x2xStatus;

/// max rawdata num read from fifo every time
extern uint16_t g_usMaxNumReadFromFifo;



extern uint8_t g_uchGsensorEnable;
/// Cap enable flag
extern uint8_t g_uchCapEnable;

/// Temp enable flag
extern uint8_t g_uchTempEnable;

/// function started bitmap, use for sampling control
extern uint32_t g_unFuncStartedBitmap ;

extern uint8_t g_uchAlgInitFlag[];


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
uint16_t GH3X2X_FindGu16MinVal(uint16_t *pusBuffer, uint8_t uchLen);

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
uint16_t GH3X2X_FindGu16MaxVal(uint16_t *pusBuffer, uint8_t uchLen);

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
void GH3X2X_FindGu16MaxMinVal(uint16_t *pusMaxVal, uint16_t *pusMinVal, uint16_t *pusBuffer, uint8_t uchLen);

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
char *GH3X2X_GetDriverLibFuncSupport(void);

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
uint8_t GH3x2x_GetChipResetRecoveringFlag(void);


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
void GH3X2X_ReinitAllSwModuleParam(uint16_t usReinitFlag);

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
void *GH3X2X_Memcpy(void *pDest, const void *pSrc, uint32_t unByteSize);

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
void *GH3X2X_Memset(void* pDest, char chVal, uint32_t unByteSize);

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
uint32_t GH3X2X_Strlen(const char *pszSrc);

uint8_t GH3x2xGetProtocolMode(void);
void GH3x2xCreatTagArray(uint8_t *puchTagArray, uint16_t usArrayLen,uint32_t *unCompeletMask, const struct gh3020_frameinfo_s * const pstFrameInfo);
void GH3x2xHandleFrameData(const struct gh3020_frameinfo_s * const pstFrameInfo, uint32_t *punTempIncompeletFlag,
                                uint32_t *punFunctionID, uint8_t uchChnlNum, uint8_t *puchRawdataBuf, uint32_t *punFrameRawdata,
                                uint8_t *puchTag, int16_t *pusGsValueArr, float fGsensorIndex,
                                STCapRawdata* pstCapValueArr,float fCapIndex,STTempRawdata* pstTempValueArr,float fTempIndex);

uint8_t GH3X2X_CheckRawdataBuf(uint8_t *puchRawdata, uint16_t usRawdataLen);
float GH3x2xCalGsensorStep(uint16_t usGsDataNum, uint16_t usFrameNum);
uint8_t GH3X2X_ReceiveOverflowLengthBuffer(uint8_t *puchRespondBuffer, uint16_t *pusRespondLen, uint8_t *puchReceiveBuffer, int32_t *pnReceiveBufferLen);

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

uint8_t GH3X2X_GetSoftWearColor(void);

void GH3X2X_SetConfigDrvEcode(const FAR struct gh3020_initcfg_s *pstGh3x2xInitConfigParam);

#endif /* _GH3X2X_DRV_CONTROL_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/
