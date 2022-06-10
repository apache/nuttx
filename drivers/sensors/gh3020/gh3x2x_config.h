/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_demo_config.h
 *
 * @brief   gh3x2x driver lib demo configuration file
 *
 * @author  Gooidx Iot Team
 *
 */
#ifndef _GH3X2X_CONFIG_H_
#define _GH3X2X_CONFIG_H_
#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"
#include "stdio.h"


/* bsp config */
#define __GH3X2X_INTERFACE__                (__GH3X2X_INTERFACE_SPI__)              /**< interface config __GH3X2X_INTERFACE_I2C__ /__GH3X2X_INTERFACE_SPI__ */
#if(__GH3X2X_INTERFACE__ == __GH3X2X_INTERFACE_SPI__)
#define __GH3X2X_SPI_TYPE__                 (__GH3X2X_SPI_TYPE_SOFTWARE_CS__)
#endif
#if(__GH3X2X_INTERFACE__ == __GH3X2X_INTERFACE_I2C__)
#define __GH3X2X_I2C_DEVICE_ID__            (GH3X2X_I2C_ID_SEL_1L0L)
#endif
#define __GS_SENSITIVITY_CONFIG__           (GSENSOR_SENSITIVITY_512_COUNTS_PER_G)  /**< gsensor sensitivity config */
#define _GS_GYRO_ENABLE_                    (0)        /*Gyro enable*/
#define __GS_NONSYNC_READ_EN__              (0)
#if __GS_NONSYNC_READ_EN__
#define __GS_SAMPLE_RATE_HZ__               (25)        /*gsenor sample rate(Hz)*/
#define __GS_READ_POINT_NUM__               (6)         /* host read gsensor sample num in one time */
#define __GS_READ_POINT_NUM_JITTER__        (3)         /* host read gsensor sample num jitter */
#endif
#define __CAP_ENABLE__                      (0)         /*Cap enable*/
#define __TEMP_ENABLE__                     (0)         /*Temperature sensor enable*/

#define __SUPPORT_HARD_RESET_CONFIG__       (1)         /**< support hard reset config */
#define __INTERRUPT_PROCESS_MODE__          (__MIX_INT_PROCESS_MODE__)  /**< 2:polling+int 1:use polling 0:use interrupt */
#define __PLATFORM_WITHOUT_OS__             (0)         /**< 1:not use os  0:use os */
#define __PASS_THROUGH_MODE__               (0)         /**< pt mode enable */
#define __GH3X2X_MP_MODE__                  (0)         /**< 1:enable mp mode  0:unable mp mode */



/* ppg function type */
#define __FUNC_TYPE_HR_ENABLE__             (1)    /**< hb function tye */
#define __FUNC_TYPE_HRV_ENABLE__            (1)    /**< hrv function tye */
#define __FUNC_TYPE_HSM_ENABLE__            (1)    /**< hsm function tye */
#define __FUNC_TYPE_SPO2_ENABLE__           (1)    /**< spo2 function tye */
#define __FUNC_TYPE_BT_ENABLE__             (0)    /**< bt function tye */
#define __FUNC_TYPE_RESP_ENABLE__           (1)    /**< resp function tye */
#define __FUNC_TYPE_AF_ENABLE__             (0)    /**< af function tye */
#define __FUNC_TYPE_BP_ENABLE__             (0)    /**< bp function tye */
#define __FUNC_TYPE_TEST_ENABLE__           (1)    /**< test function tye */
#define __SUPPORT_HARD_ADT_CONFIG__         (0)    /**< support hard adt config */
#define __SUPPORT_SOFT_AGC_CONFIG__         (1)    /**< support soft agc config */


/*sofe agc*/
#define GH3X2X_NEW_AGC_SLOT_NUM_LIMIT       (8)
#define GH3X2X_NEW_AGC_SUB_CHNL_NUM_LIMIT   (32-8)




/* ecg function type */
#define __FUNC_TYPE_ECG_ENABLE__            (0)    /**< ecg algorithm tye */
#if __FUNC_TYPE_ECG_ENABLE__
#define __FUNC_TYPE_PWTT_ENABLE__           (1)    /**< pwtt algorithm tye */
#define __FUNC_TYPE_BP_ENABLE__             (0)    /**< bp algorithm tye */
#define __SUPPORT_ECG_LEAD_OFF_DET_800HZ__  (1)
#endif

/* soft adt function type */
#define __FUNC_TYPE_SOFT_ADT_ENABLE__       (0)*(__SUPPORT_HARD_ADT_CONFIG__)*(__FUNC_TYPE_HR_ENABLE__)    /**< support soft adt config */

/* soft adt function threshold that can be judged as a movement */
#if (__FUNC_TYPE_SOFT_ADT_ENABLE__)
#define __GSENSOR_MOVE_THRESHOLD__                    (65)   /**< DIFF of sqrt(x^2+y^2+z^2) */
#define __GSENSOR_MOVE_CNT_THRESHOLD__                (1)    /**< (recomended value = 1) more than how many times of movement can be judged as effective moveing*/
#define __GSENSOR_NOT_MOVE_CNT_THRESHOLD__            (150)  /**< (recommended value = 1.5 * sample rate of g-sensor ) more than how many times of movement can be judged as effective non-moveing*/
#define __USE_POLLING_TIMER_AS_ADT_TIMER__            (1)    /** use polling timer as soft adt timer **/
#define __SOFT_ADT_CTRL_WEAR_OFF_ENABLE__             (1)    /**< support auto ctrl wear off */
#define __SOFT_ADT_IR_DETECT_TIMEOUT__                (30)   /** (recomended value = 30 seconds) ir detect timeout **/
#endif

/*sample config*/
#define __SLOT_SEQUENCE_DYNAMIC_ADJUST_EN__       (0)


/* algorithm config */
#define __ALGO_RUN_SIMULTANEOUSLY_SUPPORT__ (1)  /**< if support multi algorithm run simultaneously */
#define __USER_DYNAMIC_ALGO_MEM_EN__        (0)  /*** 0: global buffer for goodix algorithm mem pool  1: user dynamic buffer for goodix algorithm mem pool */
#define __USE_GOODIX_HR_ALGORITHM__         (0) * (__FUNC_TYPE_HR_ENABLE__)
#define __USE_GOODIX_HRV_ALGORITHM__        (0) * (__FUNC_TYPE_HRV_ENABLE__)
#define __USE_GOODIX_HSM_ALGORITHM__        (0) * (__FUNC_TYPE_HSM_ENABLE__)
#define __USE_GOODIX_SPO2_ALGORITHM__       (0) * (__FUNC_TYPE_SPO2_ENABLE__)
#define __USE_GOODIX_ECG_ALGORITHM__        (0) * (__FUNC_TYPE_ECG_ENABLE__)
#define __USE_GOODIX_BT_ALGORITHM__         (0) * (__FUNC_TYPE_BT_ENABLE__)
#define __USE_GOODIX_RESP_ALGORITHM__       (0) * (__FUNC_TYPE_RESP_ENABLE__)
#define __USE_GOODIX_AF_ALGORITHM__         (0) * (__FUNC_TYPE_AF_ENABLE__) * (__FUNC_TYPE_HRV_ENABLE__)
#define __USE_GOODIX_BP_ALGORITHM__         (0) * (__FUNC_TYPE_BP_ENABLE__)
#define __USE_GOODIX_SOFT_ADT_ALGORITHM__   (0) * (__FUNC_TYPE_SOFT_ADT_ENABLE__)


/*algorithm output value process strategy config*/
#define __GH3X2X_HR_OUTPUT_VALUE_STRATEGY_EN__  (0)




/*debug config*/
#define __SUPPORT_PROTOCOL_ANALYZE__                    (0)     /**< driver lib support protocol analyze */
#if (__SUPPORT_PROTOCOL_ANALYZE__)
#define __UPLOAD_ALGO_RESULT__                          (1)             /**< upload algorithm result or not */
#define __PROTOCOL_SERIAL_TYPE__                        (__PROTOCOL_SERIAL_USE_BLE__)  /**< protocol communicate serial port type */
#define __SUPPORT_ZIP_PROTOCOL__                        (1)
#define __FIFO_PACKAGE_SEND_ENABLE__                    (1)    /** 1: fifo package send mode enable  0: cannot open fifo package send mode */
#define __GH3X2X_PROTOCOL_SEND_TIMER_PERIOD__           (10)                         /** (unit : ms ) protocal data send timer period */
#define __GH3X2X_PROTOCOL_DATA_FIFO_LEN__               (64)                       /** protocal data send fifo length **/
#define __GH3X2X_PROTOCOL_EVENT_FIFO_LEN__              (16)                       /** protocal event send fifo length **/
#define __GH3X2X_PROTOCOL_EVENT_WAITING_ACK_TIME__      (500)                   /** (unit : ms ) protocal data waiting ack time, if time out, we will resend */
#define __GH3X2X_PROTOCOL_EVENT_RESEND_NUM__            (255)                      /***** 0~255  protocal resend num (255: evenlasting resending) */
#define __GH3X2X_PROTOCOL_DATA_FUNCTION_INTERCEPT__     (GH3X2X_NO_FUNCTION)    /* GH3X2X_NO_FUNCTION: none function date will be intercepted     (GH3X2X_FUNCTION_HR|GH3X2X_FUNCTION_HRV):  HR and HRV function data will be intercepted, those data will not output via protocal */
#endif
#define __SUPPORT_SAMPLE_DEBUG_MODE__                   (0)                             /**< use sample debug mode */
#define __SUPPORT_ELECTRODE_WEAR_STATUS_DUMP__          (1)                        /** use electrode wear status dump */
#define __EXAMPLE_LOG_CONFIG__                          (1)                          /**< example log config */
#define   EXAMPLE_LOG_MAX_LEN                           (128)
#define __SUPPORT_ENGINEERING_MODE__                    (1)                 /**< enginerring mode*/
#define __GH3X2X_MEM_POOL_CHECK_EN__                    (0)                  /** 1: drv lib checks algo mem pool befor every time of algo calculating   0: do not check  **/


/* operate config */
#define __GH3X2X_ARRAY_CFG_MANUAL_SWITCH_EN__           (0)            /** if it is enable, you should switch array cfg manually **/
#define __SUPPORT_FUNCTION_SAMPLE_RATE_MODIFY__         (1)        /** 0: only use default sample rate in array cfg  1: modifying function sample rate is supported via API: Gh3x2xDemoFunctionSampleRateSet */



/*config list*/
#define __GH3X2X_CFG_LIST_MAX_NUM__                     (2)             /**< max num of config list(1 ~ 8) */


/* other config */
#define __SUPPORT_HOOK_FUNC_CONFIG__                    (1)             /**< support hook func config */
#define __SUPPORT_ALGO_INPUT_OUTPUT_DATA_HOOK_CONFIG__  (1)  /**< enable it ,we can get algo input and output data **/

/* gh3x2x data buffer size config,this is related to gh3x2x fifo water mark config */
#define __GH3X2X_RAWDATA_BUFFER_SIZE__                  (128 * 4)                                  /**< rawdata buffer size in byte */
#define __GH3X2X_RAWDATA_BUFFER_CRC_SIZE__              (8)                                        /**< rawdata buffer crc size in byte */

/* gsensor data buffer size config,every g sensor data has 6 bytes(x,y,z) */
#define __GSENSOR_DATA_BUFFER_SIZE__                    (4)                                   /**< max num of gsensor data */

#define __CAP_DATA_BUFFER_SIZE__                        (4)       /**< max num of CAP data */
#define __TEMP_DATA_BUFFER_SIZE__                       (4)       /**< max num of TEMP data */

/* algoritm channel nunber config*/
#define __HR_ALGORITHM_SUPPORT_CHNL_NUM__               (4)  /* range 1~4 */
#define __SPO2_ALGORITHM_SUPPORT_CHNL_NUM__             (1)  /* range 1-4 */

/* function channel nunber config*/
#define   GH3X2X_ADT_CHNL_NUM         (16)        /**< ADT function channel num */
#define   GH3X2X_HR_CHNL_NUM          (32)        /**< HR function channel num */
#define   GH3X2X_HRV_CHNL_NUM         (32)        /**< HRV function channel num */
#define   GH3X2X_HSM_CHNL_NUM         (32)        /**< HSM function channel num */
#define   GH3X2X_BP_CHNL_NUM          (32)        /**< BP function channel num */
#define   GH3X2X_PWA_CHNL_NUM         (32)        /**< PWA function channel num */
#define   GH3X2X_SPO2_CHNL_NUM        (32)        /**< SPO2 function channel num */
#define   GH3X2X_ECG_CHNL_NUM         (16)        /**< ECG function channel num */
#define   GH3X2X_PWTT_CHNL_NUM        (32)        /**< PWTT function channel num */
#define   GH3X2X_SOFT_ADT_CHNL_NUM    (16)        /**< soft adt function channel num */
#define   GH3X2X_BT_CHNL_NUM          (32)        /**< BT function channel num */
#define   GH3X2X_RESP_CHNL_NUM        (32)        /**< RESP function channel num */
#define   GH3X2X_AF_CHNL_NUM          (32)        /**< AF function channel num */
#define   GH3X2X_TEST_CHNL_NUM        (16)        /**< TEST1 function channel num */




/***********************************  DO NOT MODIFY FOLLOWING CODE *******************************/
#define ALGO_IO_DATA_CSV_BASE_COLUMN_FRAME_ID   1
#define ALGO_IO_DATA_CSV_BASE_COLUMN_GSENSOR    2
#define ALGO_IO_DATA_CSV_BASE_COLUMN_RAWDATA    5
#define ALGO_IO_DATA_CSV_BASE_COLUMN_FLAG       21
#define ALGO_IO_DATA_CSV_BASE_COLUMN_RESULT     45
#define ALGO_IO_DATA_CSV_BASE_COLUMN_AGC_INFO   53



#if (\
    __USE_GOODIX_HR_ALGORITHM__      ||\
    __USE_GOODIX_HRV_ALGORITHM__     ||\
    __USE_GOODIX_HSM_ALGORITHM__     ||\
    __USE_GOODIX_SPO2_ALGORITHM__    ||\
    __USE_GOODIX_ECG_ALGORITHM__     ||\
    __USE_GOODIX_BT_ALGORITHM__      ||\
    __USE_GOODIX_RESP_ALGORITHM__    ||\
    __USE_GOODIX_AF_ALGORITHM__      ||\
    __USE_GOODIX_BP_ALGORITHM__      ||\
    __USE_GOODIX_SOFT_ADT_ALGORITHM__\
    )
#define __DRIVER_LIB_MODE__                       (__DRV_LIB_WITH_ALGO__)
#else
#define __DRIVER_LIB_MODE__                       (__DRV_LIB_WITHOUT_ALGO__)
#endif



/* log def*/
#if (__EXAMPLE_LOG_CONFIG__)

#define   EXAMPLE_LOG(...)          do {\
                                        char gh3x2x_example_log[EXAMPLE_LOG_MAX_LEN] = {0};\
                                        snprintf(gh3x2x_example_log, EXAMPLE_LOG_MAX_LEN, __VA_ARGS__);\
                                        GH3X2X_Log((char *)gh3x2x_example_log);\
                                    } while(0)

#else

#define   EXAMPLE_LOG(...)

#endif


#ifndef __FUNC_TYPE_SOFT_ADT_ENABLE__
#define __FUNC_TYPE_SOFT_ADT_ENABLE__  (0)
#endif
#if (0 == __FUNC_TYPE_SOFT_ADT_ENABLE__)
#define __USE_SOFT_ADT_DETECT_WEAR_ON__  (0)
#endif

#if 0 == __FUNC_TYPE_HR_ENABLE__
#ifdef __USE_GOODIX_SOFT_ADT_ALGORITHM__
#undef __USE_GOODIX_SOFT_ADT_ALGORITHM__
#define __USE_GOODIX_SOFT_ADT_ALGORITHM__ 0
#endif
#endif

#if __FUNC_TYPE_ECG_ENABLE__ == 0
#define __FUNC_TYPE_PWTT_ENABLE__           (0)    /**< pwtt algorithm tye */
#define __FUNC_TYPE_BP_ENABLE__             (0)    /**< bp algorithm tye */
#define __SUPPORT_ECG_LEAD_OFF_DET_800HZ__  (0)
#endif

#if __FUNC_TYPE_SOFT_ADT_ENABLE__
#ifdef GOODIX_DEMO_PLANFORM
#define GH3X2X_ONLY_SUPPORT_SOFT_ADT          0
#else
#define GH3X2X_ONLY_SUPPORT_SOFT_ADT          1
#endif
#else
#define GH3X2X_ONLY_SUPPORT_SOFT_ADT          0
#endif

#if(__GH3X2X_INTERFACE__ != __GH3X2X_INTERFACE_SPI__)
#define __GH3X2X_SPI_TYPE__                 (__GH3X2X_SPI_TYPE_SOFTWARE_CS__)
#endif
#if(__GH3X2X_INTERFACE__ != __GH3X2X_INTERFACE_I2C__)
#define __GH3X2X_I2C_DEVICE_ID__            (GH3X2X_I2C_ID_SEL_1L0L)
#endif

/* algorithm mem config */
#if (__DRIVER_LIB_MODE__ == __DRV_LIB_WITH_ALGO__)

#if (__USE_GOODIX_HR_ALGORITHM__)
#if __HR_ALGORITHM_SUPPORT_CHNL_NUM__ <= 1
#define   GH3X2X_ALGORITHM_HR_MEMORY_PEAK      GH3X2X_HR_ALGORITHM_MEMORY_PEAK_1CHNL
#define   GH3X2X_ALGORITHM_HR_MEMORY_RESIDENT  GH3X2X_HR_ALGORITHM_MEMORY_RESIDENT_1CHNL
#elif __HR_ALGORITHM_SUPPORT_CHNL_NUM__ == 2
#define   GH3X2X_ALGORITHM_HR_MEMORY_PEAK   GH3X2X_HR_ALGORITHM_MEMORY_PEAK_2CHNL
#define   GH3X2X_ALGORITHM_HR_MEMORY_RESIDENT  GH3X2X_HR_ALGORITHM_MEMORY_RESIDENT_2CHNL
#elif __HR_ALGORITHM_SUPPORT_CHNL_NUM__ == 3
#define   GH3X2X_ALGORITHM_HR_MEMORY_PEAK   GH3X2X_HR_ALGORITHM_MEMORY_PEAK_3CHNL
#define   GH3X2X_ALGORITHM_HR_MEMORY_RESIDENT  GH3X2X_HR_ALGORITHM_MEMORY_RESIDENT_3CHNL
#elif __HR_ALGORITHM_SUPPORT_CHNL_NUM__ >= 4
#define   GH3X2X_ALGORITHM_HR_MEMORY_PEAK   GH3X2X_HR_ALGORITHM_MEMORY_PEAK_4CHNL
#define   GH3X2X_ALGORITHM_HR_MEMORY_RESIDENT  GH3X2X_HR_ALGORITHM_MEMORY_RESIDENT_4CHNL
#endif
#define GH3X2X_ALGORITHM_HR_MEMORY_PEAK_WITHOUT_RESIDENT    (GH3X2X_ALGORITHM_HR_MEMORY_PEAK - GH3X2X_ALGORITHM_HR_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_HR_MEMORY_REDUNDANCY                 GH3X2X_HR_ALGORITHM_MEMORY_REDUNDANCY
#else
#define GH3X2X_ALGORITHM_HR_MEMORY_RESIDENT   (0)
#define GH3X2X_ALGORITHM_HR_MEMORY_PEAK_WITHOUT_RESIDENT    (0)
#define GH3X2X_ALGORITHM_HR_MEMORY_REDUNDANCY               (0)
#endif

#if (__USE_GOODIX_HSM_ALGORITHM__)
#define   GH3X2X_ALGORITHM_HSM_MEMORY_RESIDENT   (GH3X2X_HSM_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_HSM_MEMORY_PEAK_WITHOUT_RESIDENT    (GH3X2X_HSM_ALGORITHM_MEMORY_PEAK - GH3X2X_HSM_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_HSM_MEMORY_REDUNDANCY                GH3X2X_HSM_ALGORITHM_MEMORY_REDUNDANCY
#else
#define GH3X2X_ALGORITHM_HSM_MEMORY_RESIDENT   (0)
#define GH3X2X_ALGORITHM_HSM_MEMORY_PEAK_WITHOUT_RESIDENT    (0)
#define GH3X2X_ALGORITHM_HSM_MEMORY_REDUNDANCY               (0)
#endif

#if (__USE_GOODIX_SPO2_ALGORITHM__)
#if __SPO2_ALGORITHM_SUPPORT_CHNL_NUM__ <= 1
#define   GH3X2X_ALGORITHM_SPO2_MEMORY_PEAK      GH3X2X_SPO2_ALGORITHM_MEMORY_PEAK_1CHNL
#define   GH3X2X_ALGORITHM_SPO2_MEMORY_RESIDENT  GH3X2X_SPO2_ALGORITHM_MEMORY_RESIDENT_1CHNL
#elif __SPO2_ALGORITHM_SUPPORT_CHNL_NUM__ == 2
#define   GH3X2X_ALGORITHM_SPO2_MEMORY_PEAK   GH3X2X_SPO2_ALGORITHM_MEMORY_PEAK_2CHNL
#define   GH3X2X_ALGORITHM_SPO2_MEMORY_RESIDENT  GH3X2X_SPO2_ALGORITHM_MEMORY_RESIDENT_2CHNL
#elif __SPO2_ALGORITHM_SUPPORT_CHNL_NUM__ == 3
#define   GH3X2X_ALGORITHM_SPO2_MEMORY_PEAK   GH3X2X_SPO2_ALGORITHM_MEMORY_PEAK_3CHNL
#define   GH3X2X_ALGORITHM_SPO2_MEMORY_RESIDENT  GH3X2X_SPO2_ALGORITHM_MEMORY_RESIDENT_3CHNL
#elif __SPO2_ALGORITHM_SUPPORT_CHNL_NUM__ >= 4
#define   GH3X2X_ALGORITHM_SPO2_MEMORY_PEAK   GH3X2X_SPO2_ALGORITHM_MEMORY_PEAK_4CHNL
#define   GH3X2X_ALGORITHM_SPO2_MEMORY_RESIDENT  GH3X2X_SPO2_ALGORITHM_MEMORY_RESIDENT_4CHNL
#endif
#define GH3X2X_ALGORITHM_SPO2_MEMORY_PEAK_WITHOUT_RESIDENT    (GH3X2X_ALGORITHM_SPO2_MEMORY_PEAK - GH3X2X_ALGORITHM_SPO2_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_SPO2_MEMORY_REDUNDANCY                 GH3X2X_SPO2_ALGORITHM_MEMORY_REDUNDANCY
#else
#define GH3X2X_ALGORITHM_SPO2_MEMORY_RESIDENT   (0)
#define GH3X2X_ALGORITHM_SPO2_MEMORY_PEAK_WITHOUT_RESIDENT    (0)
#define GH3X2X_ALGORITHM_SPO2_MEMORY_REDUNDANCY                (0)

#endif

#if (__USE_GOODIX_ECG_ALGORITHM__)
#define   GH3X2X_ALGORITHM_ECG_MEMORY_RESIDENT   (GH3X2X_ECG_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_ECG_MEMORY_PEAK_WITHOUT_RESIDENT    (GH3X2X_ECG_ALGORITHM_MEMORY_PEAK - GH3X2X_ECG_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_ECG_MEMORY_REDUNDANCY                GH3X2X_ECG_ALGORITHM_MEMORY_REDUNDANCY
#else
#define GH3X2X_ALGORITHM_ECG_MEMORY_RESIDENT   (0)
#define GH3X2X_ALGORITHM_ECG_MEMORY_PEAK_WITHOUT_RESIDENT    (0)
#define GH3X2X_ALGORITHM_ECG_MEMORY_REDUNDANCY                (0)
#endif

#if (__USE_GOODIX_HRV_ALGORITHM__)
#define   GH3X2X_ALGORITHM_HRV_MEMORY_RESIDENT   (GH3X2X_HRV_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_HRV_MEMORY_PEAK_WITHOUT_RESIDENT    (GH3X2X_HRV_ALGORITHM_MEMORY_PEAK - GH3X2X_HRV_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_HRV_MEMORY_REDUNDANCY                GH3X2X_HRV_ALGORITHM_MEMORY_REDUNDANCY
#else
#define GH3X2X_ALGORITHM_HRV_MEMORY_RESIDENT   (0)
#define GH3X2X_ALGORITHM_HRV_MEMORY_PEAK_WITHOUT_RESIDENT    (0)
#define GH3X2X_ALGORITHM_HRV_MEMORY_REDUNDANCY               (0)
#endif

#if (__USE_GOODIX_BT_ALGORITHM__)
#define   GH3X2X_ALGORITHM_BT_MEMORY_RESIDENT   (GH3X2X_BT_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_BT_MEMORY_PEAK_WITHOUT_RESIDENT    (GH3X2X_BT_ALGORITHM_MEMORY_PEAK - GH3X2X_BT_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_BT_MEMORY_REDUNDANCY                GH3X2X_BT_ALGORITHM_MEMORY_REDUNDANCY
#else
#define GH3X2X_ALGORITHM_BT_MEMORY_RESIDENT   (0)
#define GH3X2X_ALGORITHM_BT_MEMORY_PEAK_WITHOUT_RESIDENT    (0)
#define GH3X2X_ALGORITHM_BT_MEMORY_REDUNDANCY               (0)
#endif

#if (__USE_GOODIX_RESP_ALGORITHM__)
#define   GH3X2X_ALGORITHM_RESP_MEMORY_RESIDENT   (GH3X2X_RESP_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_RESP_MEMORY_PEAK_WITHOUT_RESIDENT    (GH3X2X_RESP_ALGORITHM_MEMORY_PEAK - GH3X2X_RESP_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_RESP_MEMORY_REDUNDANCY                GH3X2X_RESP_ALGORITHM_MEMORY_REDUNDANCY
#else
#define GH3X2X_ALGORITHM_RESP_MEMORY_RESIDENT   (0)
#define GH3X2X_ALGORITHM_RESP_MEMORY_PEAK_WITHOUT_RESIDENT    (0)
#define GH3X2X_ALGORITHM_RESP_MEMORY_REDUNDANCY               (0)
#endif

#if (__USE_GOODIX_AF_ALGORITHM__)
#define   GH3X2X_ALGORITHM_AF_MEMORY_RESIDENT   (GH3X2X_AF_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_AF_MEMORY_PEAK_WITHOUT_RESIDENT    (GH3X2X_AF_ALGORITHM_MEMORY_PEAK - GH3X2X_AF_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_AF_MEMORY_REDUNDANCY                GH3X2X_AF_ALGORITHM_MEMORY_REDUNDANCY
#else
#define GH3X2X_ALGORITHM_AF_MEMORY_RESIDENT   (0)
#define GH3X2X_ALGORITHM_AF_MEMORY_PEAK_WITHOUT_RESIDENT    (0)
#define GH3X2X_ALGORITHM_AF_MEMORY_REDUNDANCY               (0)
#endif


#if (__USE_GOODIX_SOFT_ADT_ALGORITHM__)
#define   GH3X2X_ALGORITHM_SOFT_ADT_MEMORY_RESIDENT   (GH3X2X_SOFT_ADT_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_SOFT_ADT_MEMORY_PEAK_WITHOUT_RESIDENT    (GH3X2X_SOFT_ADT_ALGORITHM_MEMORY_PEAK - GH3X2X_SOFT_ADT_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_SOFT_ADT_MEMORY_REDUNDANCY                GH3X2X_SOFT_ADT_ALGORITHM_MEMORY_REDUNDANCY
#else
#define GH3X2X_ALGORITHM_SOFT_ADT_MEMORY_RESIDENT   (0)
#define GH3X2X_ALGORITHM_SOFT_ADT_MEMORY_PEAK_WITHOUT_RESIDENT    (0)
#define GH3X2X_ALGORITHM_SOFT_ADT_MEMORY_REDUNDANCY                (0)
#endif

#if (__USE_GOODIX_BP_ALGORITHM__)
#define   GH3X2X_ALGORITHM_BP_MEMORY_RESIDENT   (GH3X2X_BP_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_BP_MEMORY_PEAK_WITHOUT_RESIDENT    (GH3X2X_BP_ALGORITHM_MEMORY_PEAK - GH3X2X_BP_ALGORITHM_MEMORY_RESIDENT)
#define GH3X2X_ALGORITHM_BP_MEMORY_REDUNDANCY                GH3X2X_BP_ALGORITHM_MEMORY_REDUNDANCY
#else
#define GH3X2X_ALGORITHM_BP_MEMORY_RESIDENT   (0)
#define GH3X2X_ALGORITHM_BP_MEMORY_PEAK_WITHOUT_RESIDENT    (0)
#define GH3X2X_ALGORITHM_BP_MEMORY_REDUNDANCY                (0)
#endif

#endif



#ifdef GH3X2X_MAIN_FUNC_CHNL_NUM
#undef GH3X2X_MAIN_FUNC_CHNL_NUM
#endif
#define GH3X2X_MAIN_FUNC_CHNL_NUM 1
#if __SUPPORT_HARD_ADT_CONFIG__
#if GH3X2X_ADT_CHNL_NUM > GH3X2X_MAIN_FUNC_CHNL_NUM
#undef GH3X2X_MAIN_FUNC_CHNL_NUM
#define GH3X2X_MAIN_FUNC_CHNL_NUM GH3X2X_ADT_CHNL_NUM
#endif
#endif
#if __FUNC_TYPE_HR_ENABLE__
#if GH3X2X_HR_CHNL_NUM > GH3X2X_MAIN_FUNC_CHNL_NUM
#undef GH3X2X_MAIN_FUNC_CHNL_NUM
#define GH3X2X_MAIN_FUNC_CHNL_NUM GH3X2X_HR_CHNL_NUM
#endif
#endif
#if __FUNC_TYPE_HRV_ENABLE__
#if GH3X2X_HRV_CHNL_NUM > GH3X2X_MAIN_FUNC_CHNL_NUM
#undef GH3X2X_MAIN_FUNC_CHNL_NUM
#define GH3X2X_MAIN_FUNC_CHNL_NUM GH3X2X_HRV_CHNL_NUM
#endif
#endif
#if __FUNC_TYPE_SPO2_ENABLE__
#if GH3X2X_SPO2_CHNL_NUM > GH3X2X_MAIN_FUNC_CHNL_NUM
#undef GH3X2X_MAIN_FUNC_CHNL_NUM
#define GH3X2X_MAIN_FUNC_CHNL_NUM GH3X2X_SPO2_CHNL_NUM
#endif
#endif
#if __FUNC_TYPE_ECG_ENABLE__
#if GH3X2X_ECG_CHNL_NUM > GH3X2X_MAIN_FUNC_CHNL_NUM
#undef GH3X2X_MAIN_FUNC_CHNL_NUM
#define GH3X2X_MAIN_FUNC_CHNL_NUM GH3X2X_ECG_CHNL_NUM
#endif
#endif
#ifdef GH3X2X_FUNC_CHNL_NUM_MAX
#undef GH3X2X_FUNC_CHNL_NUM_MAX
#endif
#define GH3X2X_FUNC_CHNL_NUM_MAX GH3X2X_MAIN_FUNC_CHNL_NUM
#if __FUNC_TYPE_SOFT_ADT_ENABLE__
#if GH3X2X_SOFT_ADT_CHNL_NUM > GH3X2X_MAIN_FUNC_CHNL_NUM
#undef GH3X2X_FUNC_CHNL_NUM_MAX
#define GH3X2X_FUNC_CHNL_NUM_MAX GH3X2X_SOFT_ADT_CHNL_NUM
#endif
#endif


#define __GS_EXTRA_BUF_LEN__    (0)
#if __GS_NONSYNC_READ_EN__
#undef __GS_EXTRA_BUF_LEN__
#define __GS_EXTRA_BUF_LEN__    (__GS_READ_POINT_NUM__ + __GS_READ_POINT_NUM_JITTER__)
#endif


#ifndef __GH3X2X_PROTOCOL_DATA_FUNCTION_INTERCEPT__
#define __GH3X2X_PROTOCOL_DATA_FUNCTION_INTERCEPT__   0
#endif

#ifndef __FIFO_PACKAGE_SEND_ENABLE__
#define __FIFO_PACKAGE_SEND_ENABLE__   0
#endif

#if 0
#define __USE_SOFT_ADT_DETECT_WEAR_ON__               (1)    /** 0:  do not use soft adt module to detect wear on   1:  use soft adt module to detect wear on(soft adt will be opened when hard adt detected wear on) **/
#define __GREEN_CONFIRM_ADT_TIME_SEC__                (15)   /** green led adt confirm time(second, default value: 15,  0: do not use green led to confirm) after ir led detected wear off**/
#define __SOFT_ADT_TIME_OUT_TIME__                    (20)   /** unit: second     soft adt detecting time out time  **/
#define __SPECIAL_ANGLE_TIME_OUT_TIME__               (30)
#define __MOTIONLESS_TIME_OUT_TIME__                  (150)
#define __CLOSE_SOFT_ADT_WHEN_DETECTED_LIVING_OBJECT  (1)    /** 1: soft adt will be closed when system detected living object  0: do not close soft adt */
#endif



#endif /* _GH3X2X_DEMO_CONFIG_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/
