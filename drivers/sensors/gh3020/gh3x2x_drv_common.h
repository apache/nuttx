/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_common.h
 *
 * @brief   gh3x2x driver common define
 *
 * @version ref gh3x2x_drv_version.h
 *
 */

#ifndef _GH3X2X_DRV_COMMON_H_
#define _GH3X2X_DRV_COMMON_H_


#include "gh3x2x_drv_config.h"
#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"


#if defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
  #ifndef __weak
    #define __weak   __attribute__((weak))
  #endif /* __weak */
  #ifndef __packed
    #define __packed __attribute__((__packed__))
  #endif /* __packed */
#endif /* __GNUC__ */


/* extern log function */
extern void GH3X2X_Log(char *chLogString);

/// log support len
#define GH3X2X_LOG_DEBUG_SUP_LEN     (128)

#define   GH3X2X_SAMPLE_LOG_PARAM(...)        do {\
                                                char chDebugStr[GH3X2X_LOG_DEBUG_SUP_LEN] = {0};\
                                                snprintf(chDebugStr, GH3X2X_LOG_DEBUG_SUP_LEN, \
                                                        "[gh3x2x_drv]: "__VA_ARGS__);\
                                                GH3X2X_Log(chDebugStr);\
                                            } while (0)



#define DRVLIB_GOODIX_DEMO_PLANFORM_DEBUG  0
#ifdef GOODIX_DEMO_PLANFORM
#ifdef GH3X2X_LOG_DEBUG
#undef GH3X2X_LOG_DEBUG
#endif
#define GH3X2X_LOG_DEBUG DRVLIB_GOODIX_DEMO_PLANFORM_DEBUG
#endif


/* debug log control, if enable debug, GH3X2X_Log must define in platforms code */
#if defined(GH3X2X_LOG_DEBUG) && (GH3X2X_LOG_DEBUG > 0) // debug level > 0


#if 1

/// log string
#define   GH3X2X_DEBUG_LOG(...)             do { GH3X2X_Log("[gh3x2x_drv]: "__VA_ARGS__); } while (0)

/// log string with param
#define   GH3X2X_DEBUG_LOG_PARAM(...)       do {\
                                                char chDebugStr[GH3X2X_LOG_DEBUG_SUP_LEN] = {0};\
                                                snprintf(chDebugStr, GH3X2X_LOG_DEBUG_SUP_LEN, \
                                                        "[gh3x2x_drv]: "__VA_ARGS__);\
                                                GH3X2X_Log(chDebugStr);\
                                            } while (0)

#define   GH3X2X_DEBUG_LOG_PARAM_WANPENG(...)


#else
#define   GH3X2X_DEBUG_LOG(...)
#define   GH3X2X_DEBUG_LOG_PARAM(...)

#define   GH3X2X_DEBUG_LOG_PARAM_WANPENG(...)       do {\
                                                char chDebugStr[GH3X2X_LOG_DEBUG_SUP_LEN] = {0};\
                                                snprintf(chDebugStr, GH3X2X_LOG_DEBUG_SUP_LEN, \
                                                        "[gh3x2x_drv]: "__VA_ARGS__);\
                                                GH3X2X_Log(chDebugStr);\
                                            } while (0)



#endif





#if (GH3X2X_LOG_DEBUG > 1) // debug level > 1

/// internal log
#define   GH3X2X_DEBUG_LOG_INTERNAL(...)        { GH3X2X_DEBUG_LOG(__VA_ARGS__); }

/// internal log with param
#define   GH3X2X_DEBUG_LOG_INTERNAL_P(...)      { GH3X2X_DEBUG_LOG_PARAM(__VA_ARGS__); }

/// log string at get rawdata
#define   GH3X2X_DEBUG_LOG_RAWDATA(...)         { GH3X2X_DEBUG_LOG_PARAM(__VA_ARGS__); }

#else

/// internal log
#define   GH3X2X_DEBUG_LOG_INTERNAL(...)

/// internal log with param
#define   GH3X2X_DEBUG_LOG_INTERNAL_P(...)

/// log string at get rawdata
#define   GH3X2X_DEBUG_LOG_RAWDATA(...)

#endif

#else   // debug level <= 0

/// log string
#define   GH3X2X_DEBUG_LOG(...)

/// log string with param
#define   GH3X2X_DEBUG_LOG_PARAM(...)

/// internal log
#define   GH3X2X_DEBUG_LOG_INTERNAL(...)

/// internal log with param
#define   GH3X2X_DEBUG_LOG_INTERNAL_P(...)

/// log string at get rawdata
#define   GH3X2X_DEBUG_LOG_RAWDATA(...)

#define   GH3X2X_DEBUG_LOG_PARAM_WANPENG(...)

#endif

/// fifo rawdata bytes max len
#define   GH3X2X_FIFO_DATA_BYTES_MAX_LEN    (3200)    // GH3X2X_FIFO_CNT_CALC(800)

/// fifo rawdata size
#define   GH3X2X_FIFO_RAWDATA_SIZE          (4)

/// fifo rawdataindex size
#define   GH3X2X_FIFO_RAWDATAINDEX_SIZE     (2)

/// fifo gsensordata size
#define   GH3X2X_FIFO_GSENSORDATA_SIZE      (6)

/// fifo agcdata size
#define   GH3X2X_FIFO_AGCDATA_SIZE          (4)

/// fifo cnt calc
#define   GH3X2X_FIFO_CNT_CALC(x)           ((x) * GH3X2X_FIFO_RAWDATA_SIZE)

/// g sensor data size
#define   GH3X2X_GSENSOR_DATA_SIZE          (6)

/// agc data size
#define   GH3X2X_AGC_DATA_SIZE              (4)

/// agc data cnt calc
#define   GH3X2X_AGC_CNT_CALC(x)           ((x) * GH3X2X_AGC_DATA_SIZE)

/// algorithm result size
#define   GH3X2X_ALGO_RESULT_SIZE           (4)

/// algorithm result calc
#define   GH3X2X_ALGO_RESULT_CALC(x)           ((x) * GH3X2X_ALGO_RESULT_SIZE)

/// algorithm result with tag calc.
#define   GH3X2X_ALGO_RESULT_WITH_TAG_CALC(x)           ((x) * (GH3X2X_ALGO_RESULT_SIZE + 1))

/* rawdata macro */

/// rawdata clear slot & adc num
#define   GH3X2X_RAWDATA_CLEAR_SLOT_ADC_NUM(x)          ((x) & 0x07FFFFFF)

/// rawdata get slot num from dword
#define   GH3X2X_DWORD_RAWDATA_GET_SLOT_NUM(x)          (((x) & 0xE0000000) >> 29)

/// rawdata get adc num from dword
#define   GH3X2X_DWORD_RAWDATA_GET_ADC_NUM(x)           (((x) & 0x18000000) >> 27)

/// rawdata get slot&adc num from dword
#define   GH3X2X_DWORD_RAWDATA_GET_SLOT_ADC_NUM(x)      (((x) & 0xF8000000) >> 27)

/// rawdata get slot num from byte
#define   GH3X2X_BYTE_RAWDATA_GET_SLOT_NUM(x)           (((x) & 0xE0) >> 5)

/// rawdata get adc num from byte
#define   GH3X2X_BYTE_RAWDATA_GET_ADC_NUM(x)            (((x) & 0x18) >> 3)

/// slot & adc num bits of rawdata byte
#define   GH3X2X_SLOT_ADC_NUM_BITS                      (0xF8)

/// rawdata get slot adc num from byte
#define   GH3X2X_BYTE_RAWDATA_GET_SLOT_ADC_NUM(x)       (((x) & GH3X2X_SLOT_ADC_NUM_BITS) >> 3)

/// get slot index from channel num
#define   GH3X2X_GET_SLOT_FROM_CHANNEL_NUM(x)           (((x) & 0x1F) >> 2)

/// get adc index from channel num
#define   GH3X2X_GET_ADC_FROM_CHANNEL_NUM(x)            ((x) & 0x3)

/// rawdata clear all flag
#define   GH3X2X_RAWDATA_CLEAR_ALL_FLAG(x)              ((x) & 0x00FFFFFF)

/// rawdata clear data
#define   GH3X2X_RAWDATA_CLEAR_DATA(x)                  ((x) & 0xFF000000)

/// rawdata get adj flag0
#define   GH3X2X_RAWDATA_GET_ADJ_FLAG0(x)               (((x) & 0x4000000) >> 26)

/// rawdata get adj flag1
#define   GH3X2X_RAWDATA_GET_ADJ_FLAG1(x)               (((x) & 0x2000000) >> 25)

/// get rawdata for algorithm
#define   GH3X2X_GET_RAWDATA_FOR_ALGO(x)                ((x) >> 7)

/// rawdata get fast recovery flag
#define   GH3X2X_RAWDATA_GET_FAST_RECOVERY_FLAG(x)      (((x) & 0x4000000) >> 26)

/// rawdata real offset
#define   GH3X2X_RAWDATA_REAL_OFFSET                    (0x00800000)

/* utils macro */

/// byte true flag val
#define   GH3X2X_BYTE_FLAG_TRUE                     (1)

/// byte false flag val
#define   GH3X2X_BYTE_FLAG_FALSE                    (0)

/// float 0 val
#define   GH3X2X_GF32_0                             (0.0f)

/// null val
//#define   GH3X2X_PTR_NULL                           ((void *) 0)

/// get high 4bits from byte
#define   GH3X2X_GET_HIGH_4BITS(val)                ((val) & 0xF0)

/// get low 4bits from byte
#define   GH3X2X_GET_LOW_4BITS(val)                 ((val) & 0x0F)

/// get first 4bits from byte
#define   GH3X2X_GET_FIRST_4BITS(val)               ((val) & 0x0000000FU)

/// get high byte from word
#define   GH3X2X_GET_HIGH_BYTE_FROM_WORD(sValue)    ((uint8_t)(((sValue) >> 8) & 0xFFU))

/// get low byte from word
#define   GH3X2X_GET_LOW_BYTE_FROM_WORD(sValue)     ((uint8_t)((sValue) & 0xFFU))

/// get byte3 from dword
#define   GH3X2X_GET_BYTE3_FROM_DWORD(unValue)      ((uint8_t)(((unValue) >> 24) & 0x000000FFU))

/// get byte2 from dword
#define   GH3X2X_GET_BYTE2_FROM_DWORD(unValue)      ((uint8_t)(((unValue) >> 16) & 0x000000FFU))

/// get byte1 from dword
#define   GH3X2X_GET_BYTE1_FROM_DWORD(unValue)      ((uint8_t)(((unValue) >> 8) & 0x000000FFU))

/// get byte0 from dword
#define   GH3X2X_GET_BYTE0_FROM_DWORD(unValue)      ((uint8_t)((unValue) & 0x000000FFU))

/// get high word from dword
#define   GH3X2X_GET_HIGH_WORD_FROM_DWORD(unValue)      ((uint16_t)(((unValue) >> 16) & 0x0000FFFFU))

/// get low word from dword
#define   GH3X2X_GET_LOW_WORD_FROM_DWORD(unValue)       ((uint16_t)((unValue) & 0x0000FFFFU))

/// makeup word from bytes
#define   GH3X2X_MAKEUP_WORD(uchHighByte, uchLowByte)              ((uint16_t)(((((uint16_t)(uchHighByte)) << 8)& 0xFF00) |\
                                                                            (((uint16_t)(uchLowByte))& 0x00FF)))

/// makeup dword from bytes
#define   GH3X2X_MAKEUP_DWORD(uchByte3, uchByte2, uchByte1, uchByte0)     (((((uint32_t)(uchByte3)) << 24) & 0xFF000000U)|\
                                                                          ((((uint32_t)(uchByte2)) << 16) & 0x00FF0000U) |\
                                                                          ((((uint32_t)(uchByte1)) << 8) & 0x0000FF00U) |\
                                                                          (((uint32_t)(uchByte0)) & 0x000000FFU))

/// makeup dword from words
#define   GH3X2X_MAKEUP_DWORD2(usHighWord, usLowWord)                  (((((uint32_t)(usHighWord)) << 16) & 0xFFFF0000U) |\
                                                                            (((uint32_t)(usLowWord)) & 0x0000FFFFU))

/// macro of hook function call
/* NOLINTNEXTLINE(501) */
#define   HOOK_FUNC_CALL(func, argv)        do { if ((func) != GH3X2X_PTR_NULL) { func argv; } } while (0)

/// macro of handle function call
/* NOLINTNEXTLINE(501) */
#define   HANDLE_FUNC_CALL(func, argv)      do { if ((func) != GH3X2X_PTR_NULL) { func argv; } } while (0)

/// macro of set val via ptr
#define   SET_VAL_VIA_PTR(ptr, val)     do { if ((ptr) != GH3X2X_PTR_NULL) { (*(ptr)) = (val); } } while (0)

/**
 * @brief macro of read pin status, e.g. READ_PIN_STATUS_CALL(g_pGh3x2xReadIntPinStatus, uchPinStatus)
 *        func() should retrun 0(low level) or 1(high), if not register func, return 0xFF(invalid)
 */
#define   READ_PIN_STATUS_CALL(func, status)    do {\
                                                    if ((func) != GH3X2X_PTR_NULL)\
                                                    {\
                                                        (status) = (func)();\
                                                    }\
                                                    else\
                                                    {\
                                                        (status) = 0xFF;\
                                                    }\
                                                } while (0)


/* bit operation macro */

/// macro of set bits
#define   GH3X2X_SET_BIT(x, b)              ((x) | (b))

/// macro of val set bits
#define   GH3X2X_VAL_SET_BIT(x, b)          ((x) |= (b))

/// macro of clear bits
#define   GH3X2X_CLEAR_BIT(x, b)            ((x) & (~(b)))

/// macro of val clear bits
#define   GH3X2X_VAL_CLEAR_BIT(x, b)        ((x) &= (~(b)))

/// macro of val get bits
#define   GH3X2X_VAL_GET_BIT(x, b)          ((x) &= (b))

/// macro of check bits set
#define   GH3X2X_CHECK_BIT_SET(x, b)        (((x) & (b)) == (b))

/// macro of check bits not set
#define   GH3X2X_CHECK_BIT_NOT_SET(x, b)    (((x) & (b)) != (b))

/// macro of check bits is set
#define   GH3X2X_CHECK_BIT_IS_SET(x, b)     (((x) & (b)) == (b))

/// macro of get bits is set
#define   GH3X2X_GET_BIT_IS_SET(x)          ((x) & 0xf000)

/// macro of bit number in one byte
#define   GH3X2X_BIT_PER_BYTE    (4)

/// macro of bit in reg addr for check
#define   GH3X2X_REG_MODE_BIT    (3)

/// macro of check bits zero
#define   GH3X2X_CHECK_REG_MODE_BIT_ZERO(x)    (((x) >> (GH3X2X_BIT_PER_BYTE * GH3X2X_REG_MODE_BIT)) == (0))

/// macro of check lsb set
#define   GH3X2X_CHECK_LSB_SET(x)           (GH3X2X_CHECK_BIT_SET((x), 0x01))

/// macro of check left shift bit set
#define   GH3X2X_CHECK_LEFT_BIT_SET(x, l)           (GH3X2X_CHECK_BIT_SET((x), (1u << (l))))

/// macro of check left shift bit not set
#define   GH3X2X_CHECK_LEFT_BIT_NOT_SET(x, l)       (GH3X2X_CHECK_BIT_NOT_SET((x), (1u << (l))))

/// macro of val clear left shift bit
#define   GH3X2X_VAL_CLEAR_LEFT_BIT(x, l)                ((x) &= (~((1u << (l)))))

/// check ret error
#define   GH3X2X_RET_ERROR_CHECK(x)             do { \
                                                    if ((x) != GH3X2X_RET_OK)\
                                                    {\
                                                        return (x);\
                                                    }\
                                                } while (0)

/// check ret the error
#define   GH3X2X_RET_ERROR_CHECK_E(x, e)        do { \
                                                    if ((x) == (e))\
                                                    {\
                                                        return (x);\
                                                    }\
                                                } while (0)

/// check ret the error
#define   GH3X2X_RET_ERROR_CHECK_ER(x, e, r)    do { \
                                                    if ((x) == (e))\
                                                    {\
                                                        return (r);\
                                                    }\
                                                } while (0)

/// val rigth shfit
#define   GH3X2X_SET_VAL_RIGTH_SHIFT(x, r)      ((x) >>= (r))

/// val rigth shfit 1bit
#define   GH3X2X_SET_VAL_RIGTH_SHIFT_1BIT(x)    (GH3X2X_SET_VAL_RIGTH_SHIFT((x), 1u))

/// val left shfit
#define   GH3X2X_SET_VAL_LEFT_SHIFT(x, l)       ((x) <<= (l))

/// val left shfit 1bit
#define   GH3X2X_SET_VAL_LEFT_SHIFT_1BIT(x)     (GH3X2X_SET_VAL_LEFT_SHIFT((x), 1u))

/// get left shfit val
#define   GH3X2X_GET_LEFT_SHIFT_VAL(l)          (1u << (l))

/// val left shfit
#define   GH3X2X_VAL_LEFT_SHIFT(x, l)           ((x) << (l))

/// val cal diff value
#define   GH3X2X_VAL_DIFF_VALUE(a, b)           ((a) > (b) ? ((a) - (b)) : ((b) - (a)))

/// val cal diff status
#define   GH3X2X_VAL_DIFF_STA(a, b)             ((a) > (b) ? (0) : (1))

/// val cal diff ABS
#define   GH3X2X_VAL_ABS(x)                     ((x) >= 0 ? (x) : - (x))

/// val zip rawdata send cmd EVEN or ODD
#define   GH3X2X_VAL_ZIPSENDCMD(x)              ((x) == 1 ? (GH3X2X_UPROTOCOL_CMD_ZIPEVENRAWDATA) \
                                                 : (GH3X2X_UPROTOCOL_CMD_ZIPODDRAWDATA))




extern const STGh3x2xModuleFunList g_Gh3x2xModuleFunList;





#endif /* _GH3X2X_DRV_COMMON_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/
