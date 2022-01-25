/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv.h
 *
 * @brief   gh3x2x driver functions
 *
 * @version v3.2.4.5.8_L61A
 *
 * @author  Gooidx Iot Team
 *
 */


#ifndef _GH3X2X_DRV_H_
#define _GH3X2X_DRV_H_

#ifndef UNUSED_VAR
#define UNUSED_VAR(X) ((void)(X))
#endif  // UNUSED_VAR

/* type redefine */
typedef unsigned char       GU8;    /**< 8bit unsigned integer type */
typedef signed char         GS8;    /**< 8bit signed integer type */
typedef unsigned short      GU16;   /**< 16bit unsigned integer type */
typedef signed short        GS16;   /**< 16bit signed integer type */
typedef int                 GS32;   /**< 32bit signed integer type */
typedef unsigned int        GU32;   /**< 32bit unsigned integer type */
typedef float               GF32;   /**< float type */
typedef double              GD64;   /**< double type */
typedef char                GCHAR;  /**< char type */
typedef unsigned char       GBOOL;  /**< bool type */
typedef long long           GS64;   /**< 64bit signed integer type */



/* communicate interface macro */
#define __GH3X2X_INTERFACE_I2C__            (1)     /**< interface use i2c */
#define __GH3X2X_INTERFACE_SPI__            (2)     /**< interface use spi */

/* spi cs type macro */
#define __GH3X2X_SPI_TYPE_SOFTWARE_CS__     (1)     /** cs is software controled in spi module  **/
#define __GH3X2X_SPI_TYPE_HARDWARE_CS__     (2)     /** cs is hardtware controled in spi module **/

/* int process type macro */
#define __NORMAL_INT_PROCESS_MODE__         (0)     /** int event handle int mode **/
#define __POLLING_INT_PROCESS_MODE__        (1)     /** int event handle polling mode **/
#define __MIX_INT_PROCESS_MODE__            (2)     /** int event handle polling+int mode **/

/* protocol serial type macro */
#define __PROTOCOL_SERIAL_USE_UART__        (1)     /**< protocol communicate use UART */
#define __PROTOCOL_SERIAL_USE_BLE__         (2)     /**< protocol communicate use BLE */

/* application mode of driver lib */
#define __DRV_LIB_WITHOUT_ALGO__            (1)    /**< not use driver lib interface about algo */
#define __DRV_LIB_WITH_ALGO__               (2)    /**< use driver lib interface about algo */





/**
 * @brief universal protocol cmd enum
 */
typedef enum
{
    UPROTOCOL_CMD_START = 0x00,             /**< start */
    UPROTOCOL_CMD_STOP,                     /**< stop */
    UPROTOCOL_CMD_EVK_MODE,                 /**< evk work mode */
    UPROTOCOL_CMD_APP_MODE,                 /**< app work mode */
    UPROTOCOL_CMD_MCU_ONLINE_MODE,          /**< mcu online work mode */
    UPROTOCOL_CMD_MCU_OFFLINE_MODE,         /**< mcu offline work mode */
    UPROTOCOL_CMD_TEST_TUNE_MODE,           /**< test tune work mode */
    UPROTOCOL_CMD_PASS_THROUGH_MODE,
    UPROTOCOL_CMD_IRQ_STATUS_REPORT_ACK,    /**< irq status report ack */
    UPROTOCOL_CMD_IGNORE = 0xFF,            /**< ignore */
} EMUprotocolParseCmdType;

/**
 * @brief i2c addr (7bits) low two bit select enum
 */
enum EMGh3x2xI2cIdSelect
{
    GH3X2X_I2C_ID_SEL_1L0L = 0,     /**< i2c ID0 & ID1 pin low */
    GH3X2X_I2C_ID_SEL_1L0H,         /**< i2c ID0 pin high & ID1 pin low */
    GH3X2X_I2C_ID_SEL_1H0L,         /**< i2c ID0 pin low & ID1 pin high */
    GH3X2X_I2C_ID_SEL_1H0H,         /**< i2c ID0 & ID1 pin high */
    GH3X2X_I2C_ID_INVALID,          /**< invalid val */
};

/**
 * @brief gsensor sensitivity(counts/g) enum
 */
typedef enum
{
    GSENSOR_SENSITIVITY_512_COUNTS_PER_G = 0,   /**< sensitivity = 512/g */
    GSENSOR_SENSITIVITY_1024_COUNTS_PER_G,      /**< sensitivity = 1024/g */
    GSENSOR_SENSITIVITY_2048_COUNTS_PER_G,      /**< sensitivity = 2048/g */
    GSENSOR_SENSITIVITY_4096_COUNTS_PER_G,      /**< sensitivity = 4096/g */
    GSENSOR_SENSITIVITY_8192_COUNTS_PER_G,      /**< sensitivity = 8192/g */
} EMGsensorSensitivity;

/**
 * @brief slot index enable config enum
 */
typedef enum
{
    GH3X2X_SET_SLOT_ENABLE = 0,        /**< enable slot and last set slot enable */
    GH3X2X_SET_SLOT_DISABLE,           /**< disable slot by set */
    GH3X2X_SET_SLOT_DIRECT_ENABLE,     /**< only enable slot by set */
} EMSlotEnableConfigType;

/**
 * @brief irq mode config
 */
typedef enum
{
    IRQ_MODE_LEVEL_HIGH = 0x00,         /**< int pin output high level */
    IRQ_MODE_LEVEL_LOW = 0x01,          /**< int pin output low level */
    IRQ_MODE_EDGE_RISING = 0x02,        /**< int pin output rising edge */
    IRQ_MODE_EDGE_FALLING = 0x03,       /**< int pin output falling edge */
} EMIrqModeConfig;

/**
 * @brief hba scene config
 */
typedef enum {
        HBA_SCENES_DEFAULT = 0,             // default

        HBA_SCENES_DAILY_LIFE = 1,            // daily life
        HBA_SCENES_RUNNING_INSIDE = 2,        // running machine
        HBA_SCENES_WALKING_INSIDE = 3,        // walking inside
        HBA_SCENES_STAIRS = 4,                // stair

        HBA_SCENES_RUNNING_OUTSIDE = 5,     // running outside
        HBA_SCENES_WALKING_OUTSIDE = 6,     // walking outing

        HBA_SCENES_STILL_REST = 7,            // still rest
        HBA_SCENES_REST = 8,                // rest
        HBA_SCENES_STILLRADON = 9,            // breath holding

        HBA_SCENES_BIKING_INSIDE = 10,        //biking inside
        HBA_SCENES_BIKING_OUTSIDE = 11,     //biking outside
        HBA_SCENES_BIKING_MOUNTAIN= 12,     //biking mountain
        HBA_SCENES_RUNNING_HIGH_HR = 13,    //running in high heart rate

        HBA_SCENES_RUNNING_TREADMILL_CCOMBINE= 14,        // combination of treadmill running

        HBA_SCENES_HIGH_INTENSITY_COMBINE = 15,     // combination of high intensity exercise
        HBA_SCENES_TRADITIONAL_STRENGTH_COMBINE = 16,        // combination of traditional strength training
        HBA_SCENES_STEP_TEST = 17,            // step test

        HBA_SCENES_BALL_SPORTS = 18,        // ball sports
        HBA_SCENES_AEROBICS = 19,            // aerobics


        HBA_SCENES_SLEEP = 20,                // sleep
        HBA_SCENES_JUMP = 21,                //jump
        HBA_SCENES_CORDLESS_JUMP = 22,        // cordless jump
        HBA_SCENES_SWIMMING = 23,            // SWIMMING
        HBA_SCENES_SIZE = 24,
}hba_scenes_e;


/**
 * @brief wear detect enable type enum
 */
typedef enum
{
    WEAR_DETECT_DISABLE = 0,   /**< wear detect disable */
    WEAR_DETECT_ENABLE,        /**< wear detect enable */
} EMWearDetectEnableType;

/**
 * @brief wear detect type enum
 */
typedef enum
{
    WEAR_DETECT_WEAR_ON = 0,   /**< detect wear on */
    WEAR_DETECT_WEAR_OFF,      /**< detect wear off */
} EMWearDetectType;

/**
 * @brief wear detect force switch enum
 */
typedef enum
{
    WEAR_DETECT_DONT_FORCE_SWITCH = 0,  /**< don't force switch */
    WEAR_DETECT_FORCE_SWITCH,           /**< force switch  */
} EMWearDetectForceSwitch;

/**
 * @brief skin color status enum
 */
typedef enum
{
    SKIN_COLOR_STATUS_DARK = 0,     /**< dark skin status */
    SKIN_COLOR_STATUS_BROWN,        /**< brown skin status */
    SKIN_COLOR_STATUS_YELLOW,       /**< yellow skin status */
    SKIN_COLOR_STATUS_FAIR,         /**< fair skin status */
} EMSkinColorStatusType;

typedef enum
{
    STATUS_DEFAULT = 0,
    STATUS_LVING_WEAR = 1,
    STATUS_LVING_UNWEAR = 2,
    STATUS_WEAR_REPORTED = 3,
    STATUS_UNWEAR_REPORTED = 4,
}EMWearRecordType;

/**
 * @brief register struct
 */
typedef struct
{
    GU16 usRegAddr;     /**< register address */
    GU16 usRegData;     /**< register val */
} STGh3x2xReg;

/**
 * @brief init config struct type
 */
typedef struct
{
    const STGh3x2xReg *pstRegConfigArr;   /**< pointer of register config array */
    GU16        usConfigArrLen;     /**< register config array length */
} STGh3x2xInitConfig;

/**
 * @brief slot rawwdata struct type
 */
typedef struct
{
    GU32 (*punRawdata)[4];   /**< pointer of buffer store 4 adc rawdata, if slot no use, can set NULL */
    GU16 usRawdataCnt;       /**< slot rawdata count */
} STGh3x2xSlotRawdata;

/**
 * @brief get rawwdata struct type
 */
typedef struct
{
    GU8 *puchReadBuffer;                        /**< pointer of buffer for read rawdata */
    STGh3x2xSlotRawdata stSlotRawdataArr[8];    /**< 8 slot rawdata array */
} STGh3x2xRawdata;

/**
 * @brief gsensor rawwdata struct type
 */


/**
 * @brief channnel rawwdata struct type
 */
typedef struct
{
    GU32 *punChRawdataArr;              /**< pointer to buffer store channel data, data map e.g. ch0 ch1...chx ch0 */
    GU16 usChRawdataCnt;                /**< channel rawdata count output */
    GU32 *punIncompleteChRawdataArr;    /**< pointer to last incomplete packet buffer */
    GU32  unIncompleteChMark;           /**< last channel rawdata incomplete mark, if chx incomplete bit[x] set 1 */
} STGh3x2xChannelRawdata;

/**
 * @brief soft lead result
 */
typedef struct
{
    GU16 usEvent;                       /**< event */
    GU8 uchNeedForceReadFifo;           /**< need force read fifo after call GH3X2X_LeadHandle(...) */
} STSoftLeadResult;

/**
 * @brief HB algorithm output result
 */
typedef struct
{
    GS32 snHbOut;                       /**< hb algorithm output value */
    GS32 snHbConfi;                     /**< hb algorithm output confidence */
    GS32 snHbSnr;                       /**< hb snr */
    GS32 snHbValidLv;                   /**< hb confidence valid level */
} STHbAlgoResult;

/**
 * @brief HRV algorithm output result
 */
typedef struct
{
    GS32 snHrvOut0;                     /**< hrv algorithm output 0 */
    GS32 snHrvOut1;                     /**< hrv algorithm output 1 */
    GS32 snHrvOut2;                     /**< hrv algorithm output 2 */
    GS32 snHrvOut3;                     /**< hrv algorithm output 3 */
    GS32 snHrvNum;                      /**< hrv effective result num of snHrvOut */
    GS32 snHrvConfi;                    /**< hrv algorithm output confidence */
} STHrvAlgoResult;

/**
 * @brief SPO2 algorithm output result
 */
typedef struct
{
    GS32 snSpo2Out;                     /**< spo2 algorithm output value */
    GS32 snSpo2Rvalue;                  /**< R value */
    GS32 snSpo2Confi;                   /**< confidence */
    GS32 snSpo2ValidLevel;              /**< valid level */
} STSpo2AlgoResult;

/**
 * @brief ECG algorithm output result
 */
typedef struct
{
    GS32 snEcgOut;                      /**< ecg algorithm output value */
    GS32 snEcgBpm;                      /**< bpm */
    GS32 snEcgSnr;                      /**< snr */
} STEcgAlgoResult;

/**
 * @brief algorithm result stuct
 */
typedef struct
{
    GS32* pnAlgoCalcResult;     //pointer to algorithm result array
    GU16* pusAlgoResIndex;      //pointer to frame index array corresponding to algorithm result
    GU16  usAlgoResCnt;         //number of algorithm result
    GU8   uchAlgoResSize;       //item number of every algorithm result
} STAlgoCalculateRes;

typedef struct
{
    GU32 unCapData[4];

}STCapRawdata;

typedef struct
{
    GU32 unTempData[4];
}STTempRawdata;

/* byte false or true */
#define BYTE_FALSE                                  (0)             /**< byte false */
#define BYTE_TRUE                                   (1)             /**< byte true */

/* return code definitions list */
#define   GH3X2X_RET_READ_FIFO_CONTINUE             (4)             /**< Fifo not read empty,should read it again */
#define   GH3X2X_RET_RAW_DATA_PKG_OVER              (3)             /**< already read all of rawdata package data */
#define   GH3X2X_RET_RAW_DATA_PKG_CONTINUE          (2)             /**< there is rawdata package data left */
#define   GH3X2X_RET_ALGO_HAS_REFRESHED             (1)             /**< algorithm has refreshed */
#define   GH3X2X_RET_OK                             (0)             /**< there is no error */
#define   GH3X2X_RET_GENERIC_ERROR                  (-1)            /**< a generic error happens */
#define   GH3X2X_RET_PARAMETER_ERROR                (-2)            /**< parameter error */
#define   GH3X2X_RET_COMM_NOT_REGISTERED_ERROR      (-3)            /**< i2c/spi operation not registered error */
#define   GH3X2X_RET_COMM_ERROR                     (-4)            /**< i2c/spi communication error */
#define   GH3X2X_RET_RESOURCE_ERROR                 (-5)            /**< resource full or not available error */
#define   GH3X2X_RET_NO_INITED_ERROR                (-6)            /**< no inited error */
#define   GH3X2X_RET_ALGO_NO_SUPPORTED              (-7)            /**< the algorithm no supported error */

/* irq status mask. */
#define   GH3X2X_IRQ_MSK_CHIP_WAKEUP_BIT            (0x0001)        /**< chip wakeup */
#define   GH3X2X_IRQ_MSK_LEAD_ON_DET_BIT            (0x0002)        /**< lead on */
#define   GH3X2X_IRQ_MSK_LEAD_OFF_DET_BIT           (0x0004)        /**< lead off */
#define   GH3X2X_IRQ_MSK_FAST_RECOVERY_BIT          (0x0008)        /**< fast recovery */
#define   GH3X2X_IRQ_MSK_ADC_DONE_BIT               (0x0010)        /**< adc sampling done */
#define   GH3X2X_IRQ_MSK_FIFO_WATERMARK_BIT         (0x0020)        /**< fifo watermark */
#define   GH3X2X_IRQ_MSK_FIFO_FULL_BIT              (0x0040)        /**< fifo full */
#define   GH3X2X_IRQ_MSK_TUNNING_FAIL_BIT           (0x0100)        /**< tunning fail */
#define   GH3X2X_IRQ_MSK_TUNNING_DONE_BIT           (0x0200)        /**< tuning done */
#define   GH3X2X_IRQ_MSK_WEAR_ON_BIT                (0x0400)        /**< wear on */
#define   GH3X2X_IRQ_MSK_WEAR_OFF_BIT               (0x0800)        /**< wear off */
#define   GH3X2X_IRQ_MSK_SLOT_TIMEOUT_BIT           (0x1000)        /**< slot timeout */
#define   GH3X2X_IRQ_MSK_SAMPLE_RATE_ERR_BIT        (0x2000)        /**< sample rate error */
#define   GH3X2X_IRQ_MSK_CHIP_RESET_BIT             (0x4000)        /**< chip reset */

/*some reg addr*/
#define   GH3X2X_INT_FIFO_UR_REG_ADDR               (0x050A)        /**< fifo use cnt */

/**
 * @brief channel map for function id
 */
typedef enum
{
    GH3X2X_FUNC_OFFSET_ADT              , /**<0 ADT FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_HR               , /**<1 HR FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_HRV              , /**<2 HRV FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_HSM              , /**<3 HSM FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_FPBP             , /**<4 BPD FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_PWA              , /**<5 PFA FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_SPO2             , /**<6 SPO2 FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_ECG              , /**<7 ECG FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_PWTT             , /**<8 PWTT FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_SOFT_ADT_GREEN   , /**<9 SOFT ADT GREEN FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_BT               , /**<10 BT FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_RESP             , /**<11 RESP FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_AF               , /**<12 AF ADT FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_TEST1            , /**<13 TEST1 FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_TEST2            , /**<14 TEST2 ADT FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_SOFT_ADT_IR      , /**<15 SOFT ADT IR FUNC_OFFSET */
    GH3X2X_FUNC_OFFSET_MAX
} EMFunctionID;


/* Function mode. */
#define   GH3X2X_NO_FUNCTION                        (0)                                             /**<0x00000000 NO function */
#define   GH3X2X_FUNCTION_ADT                       ((0x1) << (GH3X2X_FUNC_OFFSET_ADT))             /**<0x00000001 ADT function */
#define   GH3X2X_FUNCTION_HR                        ((0x1) << (GH3X2X_FUNC_OFFSET_HR))              /**<0x00000002 HR function */
#define   GH3X2X_FUNCTION_HRV                       ((0x1) << (GH3X2X_FUNC_OFFSET_HRV))             /**<0x00000004 HRV function */
#define   GH3X2X_FUNCTION_HSM                       ((0x1) << (GH3X2X_FUNC_OFFSET_HSM))             /**<0x00000008 HSM function */
#define   GH3X2X_FUNCTION_FPBP                      ((0x1) << (GH3X2X_FUNC_OFFSET_FPBP))            /**<0x00000010 FPBP function */
#define   GH3X2X_FUNCTION_PWA                       ((0x1) << (GH3X2X_FUNC_OFFSET_PWA))             /**<0x00000020 PWA function */
#define   GH3X2X_FUNCTION_SPO2                      ((0x1) << (GH3X2X_FUNC_OFFSET_SPO2))            /**<0x00000040 SPO2 function */
#define   GH3X2X_FUNCTION_ECG                       ((0x1) << (GH3X2X_FUNC_OFFSET_ECG))             /**<0x00000080 ECG function */
#define   GH3X2X_FUNCTION_PWTT                      ((0x1) << (GH3X2X_FUNC_OFFSET_PWTT))            /**<0x00000100 PWTT function */
#define   GH3X2X_FUNCTION_SOFT_ADT_GREEN            ((0x1) << (GH3X2X_FUNC_OFFSET_SOFT_ADT_GREEN))  /**<0x00000200 SOFT ADT GREEN function */
#define   GH3X2X_FUNCTION_BT                        ((0x1) << (GH3X2X_FUNC_OFFSET_BT))              /**<0x00000400 BT function */
#define   GH3X2X_FUNCTION_RESP                      ((0x1) << (GH3X2X_FUNC_OFFSET_RESP))            /**<0x00000800 RESP function */
#define   GH3X2X_FUNCTION_AF                        ((0x1) << (GH3X2X_FUNC_OFFSET_AF))              /**<0x00001000 AF function */
#define   GH3X2X_FUNCTION_TEST1                     ((0x1) << (GH3X2X_FUNC_OFFSET_TEST1))           /**<0x00002000 TEST1 function */
#define   GH3X2X_FUNCTION_TEST2                     ((0x1) << (GH3X2X_FUNC_OFFSET_TEST2))           /**<0x00004000 TEST2 function */
#define   GH3X2X_FUNCTION_SOFT_ADT_IR               ((0x1) << (GH3X2X_FUNC_OFFSET_SOFT_ADT_IR))     /**<0x00008000 SOFT ADT IR function */


/* slot index . */
#define   GH3X2X_SLOT_INDEX_0                       (0x0001)        /**< slot index 0 */
#define   GH3X2X_SLOT_INDEX_1                       (0x0002)        /**< slot index 1 */
#define   GH3X2X_SLOT_INDEX_2                       (0x0004)        /**< slot index 2 */
#define   GH3X2X_SLOT_INDEX_3                       (0x0008)        /**< slot index 3 */
#define   GH3X2X_SLOT_INDEX_4                       (0x0010)        /**< slot index 4 */
#define   GH3X2X_SLOT_INDEX_5                       (0x0020)        /**< slot index 5 */
#define   GH3X2X_SLOT_INDEX_6                       (0x0040)        /**< slot index 6 */
#define   GH3X2X_SLOT_INDEX_7                       (0x0080)        /**< slot index 7 */
#define   GH3X2X_SLOT_INDEX_ALL                     (0x00FF)        /**< all slot */

/* led driver index . */
#define   GH3X2X_LED_DRV_INDEX_0                    (0x0000)        /**< led driver index 0 */
#define   GH3X2X_LED_DRV_INDEX_1                    (0x0001)        /**< led driver index 1 */

/* channel map to slot & adc for algorithm */
#define   GH3X2X_CHANNEL_MAP_SLOT0                  (0x00)          /**< channel map to slot 0 */
#define   GH3X2X_CHANNEL_MAP_SLOT1                  (0x20)          /**< channel map to slot 1 */
#define   GH3X2X_CHANNEL_MAP_SLOT2                  (0x40)          /**< channel map to slot 2 */
#define   GH3X2X_CHANNEL_MAP_SLOT3                  (0x60)          /**< channel map to slot 3 */
#define   GH3X2X_CHANNEL_MAP_SLOT4                  (0x80)          /**< channel map to slot 4 */
#define   GH3X2X_CHANNEL_MAP_SLOT5                  (0xA0)          /**< channel map to slot 5 */
#define   GH3X2X_CHANNEL_MAP_SLOT6                  (0xC0)          /**< channel map to slot 6 */
#define   GH3X2X_CHANNEL_MAP_SLOT7                  (0xE0)          /**< channel map to slot 7 */
#define   GH3X2X_CHANNEL_MAP_ADC0                   (0x00)          /**< channel map to adc 0 */
#define   GH3X2X_CHANNEL_MAP_ADC1                   (0x08)          /**< channel map to adc 1 */
#define   GH3X2X_CHANNEL_MAP_ADC2                   (0x10)          /**< channel map to adc 2 */
#define   GH3X2X_CHANNEL_MAP_ADC3                   (0x18)          /**< channel map to adc 3 */
#define   GH3X2X_CHANNEL_MAP(slot, adc)             ((slot) | (adc))    /**< calc channel map */

#define   GH3X2X_DEMO_WORK_MODE_EVK                 (0)
#define   GH3X2X_DEMO_WORK_MODE_APP                 (1)
#define   GH3X2X_DEMO_WORK_MODE_MCU_ONLINE          (2)
#define   GH3X2X_DEMO_WORK_MODE_MCU_OFFLINE         (3)
#define   GH3X2X_DEMO_WORK_MODE_TEST_TUNE           (4)
#define   GH3X2X_DEMO_WORK_MODE_PASS_THROUGH        (5)
#define   GH3X2X_DEMO_GET_WORK_MODE                 (6)

/* soft event list*/
#define GH3X2X_SOFT_EVENT_NEED_FORCE_READ_FIFO      ((GU8)1)
#define GH3X2X_SOFT_EVENT_WEAR_OFF                  ((GU8)2)
#define GH3X2X_SOFT_EVENT_NEED_TRY_READ_FIFO        ((GU8)4)
/* adt confirm status */
#define GH3X2X_SENSOR_IS_NOT_MOVING                 ((GU8)0)
#define GH3X2X_SENSOR_IS_MOVING                     ((GU8)1)

/* algorithm run mode:simultaneously or not */
#define GH3X2X_ALGORITHM_RUN_ALONE                  (0)             /**< only excute one algorithm at the same time */
#define GH3X2X_ALGORITHM_RUN_SIMULTANEOUSLY         (1)             /**< excute more than one algorithm */

/* algorithm mem size in byte */
#define GH3X2X_HR_ALGORITHM_MEMORY_PEAK_1CHNL             (9116)       /**< HB 1 chnl algorithm peak memory size in byte */
#define GH3X2X_HR_ALGORITHM_MEMORY_PEAK_2CHNL             (9116)       /**< HB 2 chnl  algorithm peak memory size in byte */
#define GH3X2X_HR_ALGORITHM_MEMORY_PEAK_3CHNL             (9116)       /**< HB 3 chnl  algorithm peak memory size in byte */
#define GH3X2X_HR_ALGORITHM_MEMORY_PEAK_4CHNL             (9116)       /**< HB 4 chnl  algorithm peak memory size in byte */

#define GH3X2X_HR_ALGORITHM_MEMORY_RESIDENT_1CHNL         (4120)       /**< HB 1 chnl  algorithm desident memory size in byte */
#define GH3X2X_HR_ALGORITHM_MEMORY_RESIDENT_2CHNL         (4120)       /**< HB 2 chnl  algorithm desident memory size in byte */
#define GH3X2X_HR_ALGORITHM_MEMORY_RESIDENT_3CHNL         (4120)       /**< HB 3 chnl  algorithm desident memory size in byte */
#define GH3X2X_HR_ALGORITHM_MEMORY_RESIDENT_4CHNL         (4120)       /**< HB 4 chnl  algorithm desident memory size in byte */

#define GH3X2X_HR_ALGORITHM_MEMORY_REDUNDANCY             (1000)



#define GH3X2X_SPO2_ALGORITHM_MEMORY_PEAK_1CHNL           (10260)        /**< SPO2 1 chnl  algorithm peak memory size in byte */
#define GH3X2X_SPO2_ALGORITHM_MEMORY_PEAK_2CHNL           (17064)       /**< SPO2 2 chnl  algorithm peak memory size in byte */
#define GH3X2X_SPO2_ALGORITHM_MEMORY_PEAK_3CHNL           (23868)       /**< SPO2 3 chnl  algorithm peak memory size in byte */
#define GH3X2X_SPO2_ALGORITHM_MEMORY_PEAK_4CHNL           (30672)       /**< SPO2 4 chnl  algorithm peak memory size in byte */

#define GH3X2X_SPO2_ALGORITHM_MEMORY_RESIDENT_1CHNL       (6916)        /**< SPO2 1 chnl  algorithm memory size in byte */
#define GH3X2X_SPO2_ALGORITHM_MEMORY_RESIDENT_2CHNL       (13720)       /**< SPO2 2 chnl  algorithm memory size in byte */
#define GH3X2X_SPO2_ALGORITHM_MEMORY_RESIDENT_3CHNL       (20524)       /**< SPO2 3 chnl  algorithm memory size in byte */
#define GH3X2X_SPO2_ALGORITHM_MEMORY_RESIDENT_4CHNL       (27328)       /**< SPO2 4 chnl  algorithm memory size in byte */

#define GH3X2X_SPO2_ALGORITHM_MEMORY_REDUNDANCY           (1000)



#define GH3X2X_ECG_ALGORITHM_MEMORY_PEAK                  (25764)       /**< ECG algorithm peak memory size in byte */
#define GH3X2X_ECG_ALGORITHM_MEMORY_RESIDENT              (25764)       /**< ECG algorithm resident memory size in byte */
#define GH3X2X_ECG_ALGORITHM_MEMORY_REDUNDANCY            (1000)


#define GH3X2X_HRV_ALGORITHM_MEMORY_PEAK                  (2856)        /**< HRV algorithm peak memory size in byte */
#define GH3X2X_HRV_ALGORITHM_MEMORY_RESIDENT              (2856)        /**< HRV algorithm resident memory size in byte */
#define GH3X2X_HRV_ALGORITHM_MEMORY_REDUNDANCY            (300)


#define GH3X2X_SOFT_ADT_ALGORITHM_MEMORY_PEAK             (800)          /**< SOFT ADT algorithm peak memory size in byte */
#define GH3X2X_SOFT_ADT_ALGORITHM_MEMORY_RESIDENT         (0)          /**< SOFT ADT algorithm resident memory size in byte */
#define GH3X2X_SOFT_ADT_ALGORITHM_MEMORY_REDUNDANCY       (80)        /**< SOFT ADT algorithm redundancy memory size in byte */


#define GH3X2X_HSM_ALGORITHM_MEMORY_PEAK                  (944)       /**< HSM algorithm memory peak size in byte */
#define GH3X2X_HSM_ALGORITHM_MEMORY_RESIDENT              (944)       /**< HSM algorithm memory resident size in byte */
#define GH3X2X_HSM_ALGORITHM_MEMORY_REDUNDANCY            (100)


#define GH3X2X_BT_ALGORITHM_MEMORY_PEAK                   (0)        /**< BT algorithm memory peak size in byte */
#define GH3X2X_BT_ALGORITHM_MEMORY_RESIDENT               (0)        /**< BT algorithm memory resident size in byte */
#define GH3X2X_BT_ALGORITHM_MEMORY_REDUNDANCY             (0)


#define GH3X2X_RESP_ALGORITHM_MEMORY_PEAK                 (5435)        /**< RESP algorithm memory peak size in byte */
#define GH3X2X_RESP_ALGORITHM_MEMORY_RESIDENT             (2363)        /**< RESP algorithm memory resident size in byte */
#define GH3X2X_RESP_ALGORITHM_MEMORY_REDUNDANCY           (500)


#define GH3X2X_AF_ALGORITHM_MEMORY_PEAK                   (3262)        /**< AF algorithm memory peak size in byte */
#define GH3X2X_AF_ALGORITHM_MEMORY_RESIDENT               (1102)        /**< AF algorithm memory resident size in byte */
#define GH3X2X_AF_ALGORITHM_MEMORY_REDUNDANCY             (300)

#ifdef ANDROID_REE_PLATFORM
#define GH3X2X_BP_ALGORITHM_MEMORY_PEAK                   (10000*1024)  /**< FPBP algorithm memory peak size in byte */
#define GH3X2X_BP_ALGORITHM_MEMORY_RESIDENT               (10000*1024)  /**< FPBP algorithm memory resident size in byte */
#define GH3X2X_BP_ALGORITHM_MEMORY_REDUNDANCY             (0)
#else
#define GH3X2X_BP_ALGORITHM_MEMORY_PEAK                   (1)  /**< FPBP algorithm memory peak size in byte */
#define GH3X2X_BP_ALGORITHM_MEMORY_RESIDENT               (1)  /**< FPBP algorithm memory resident size in byte */
#define GH3X2X_BP_ALGORITHM_MEMORY_REDUNDANCY             (0)
#endif

#define GH3X2X_ALGO_INFO_RECORD_GSENSOR_NUM               (3)
#define GH3X2X_ALGO_INFO_RECORD_RAWDARA_NUM               (4)
#define GH3X2X_ALGO_INFO_RECORD_AGC_INFO_NUM              (4)
#define GH3X2X_ALGO_INFO_RECORD_FALG_NUM                  (3)
#define GH3X2X_ALGO_INFO_RECORD_RESULT_NUM                (8)

/// universal protocol packet max len support
#define  GH3X2X_UPROTOCOL_PACKET_LEN_MAX                  (243)

#define  GH3X2X_UPROTOCOL_PACKET_HEADER_LEN               (4u)        /**< packet header len */
#define  GH3X2X_UPROTOCOL_PACKET_TAIL_LEN                 (1u)        /**< packet tail len */


/// packet header&tail len
#define  GH3X2X_UPROTOCOL_PACKET_HEADER_TAIL_LEN          (GH3X2X_UPROTOCOL_PACKET_HEADER_LEN + \
                                                            GH3X2X_UPROTOCOL_PACKET_TAIL_LEN)


/// universal protocol payload max len
#define  GH3X2X_UPROTOCOL_PAYLOAD_LEN_MAX                 (GH3X2X_UPROTOCOL_PACKET_LEN_MAX - \
                                                            GH3X2X_UPROTOCOL_PACKET_HEADER_TAIL_LEN)

/// Overflow protocol payload max len
#define  GH3X2X_UPROTOCOL_OVERFLOW_PAYLOAD_HEADER_LEN           (10)
#define  GH3X2X_UPROTOCOL_OVERFLOW_PAYLOAD_LEN_MAX              (220)

/// number of channel map id
#define  CHANNEL_MAP_ID_NUM                               (32)

/// null val
#define   GH3X2X_PTR_NULL                                 ((void *) 0)

/* ecg sample event type */
#define  ECG_SAMPLE_EVENT_TYPE_SAMPLE                     (0x01)     /**< sample evt type sample config */
#define  ECG_SAMPLE_EVENT_TYPE_SLOT                       (0x02)     /**< sample evt type slot config */
#define  ECG_SAMPLE_EVENT_TYPE_LEAD                       (0x04)     /**< sample evt type lead irq */
#define  ECG_SAMPLE_EVENT_ALL                             (ECG_SAMPLE_EVENT_TYPE_SAMPLE | ECG_SAMPLE_EVENT_TYPE_SLOT | \
                                                            ECG_SAMPLE_EVENT_TYPE_LEAD)     /**< sample evt all */

/* ecg sample event type */
#define  ECG_SAMPLE_EVENT_INFO_SAMPLE_STOP                (0)      /**< sample evt info sample stop */
#define  ECG_SAMPLE_EVENT_INFO_SAMPLE_START               (1)      /**< sample evt info sample start */
#define  ECG_SAMPLE_EVENT_INFO_LEAD_OFF                   (0)      /**< sample evt info lead off */
#define  ECG_SAMPLE_EVENT_INFO_LEAD_ON                    (1)      /**< sample evt info lead on */




typedef struct
{
   GU32  unFunctionID;  //Function id     use maroc exp: GH3X2X_FUNCTION_HR
   GU8  ubChnlNum;
   GU8  ubFrameId;
   GS16 usGsensor[GH3X2X_ALGO_INFO_RECORD_GSENSOR_NUM];
   GU32 unRawdata[GH3X2X_ALGO_INFO_RECORD_RAWDARA_NUM];
   GU32 unAgcInfo[GH3X2X_ALGO_INFO_RECORD_AGC_INFO_NUM];
   GU32 unFlag[GH3X2X_ALGO_INFO_RECORD_FALG_NUM];
   GU32 unResult[GH3X2X_ALGO_INFO_RECORD_RESULT_NUM];
} STGh3x2xAlgoInfoRecordData;




typedef struct
{
    // void (*DumpInit) (void);
    //void (*BgLevelSet)(GU16);
    //void (*DumpModeSet)(GU16);
    //void (*MemSetSpo2IncompleteArr)(void);
    //void (*MemSetHbaIncompleteArr)(void);
    void (*AlgoInfoRecordInputData)(GS32* ,GU8 ,GU32 ,GU8* ,GU32 );
    void (*AlgoInfoRecordOutputData)(GS32 *, GU8 );
} STGh3x2xModuleFunList;





typedef struct
{
   GU8 puchPacketPayloadArr[GH3X2X_UPROTOCOL_PAYLOAD_LEN_MAX + 8];
   GU8 uchFrameIdArr[CHANNEL_MAP_ID_NUM];
} STGh3x2xProtocolData;




#define GH3X2X_AGC_REG_AGC_EN                     0
#define GH3X2X_AGC_REG_LED_CURRENT_UP_LIMIT       1
#define GH3X2X_AGC_REG_LED_CURRENT_DOWN_LIMIT     2
#define GH3X2X_AGC_REG_LED_CURRENT_DRV0           3
#define GH3X2X_AGC_REG_LED_CURRENT_DRV1           4
#define GH3X2X_AGC_REG_TIA_GAIN_RX0               5
#define GH3X2X_AGC_REG_TIA_GAIN_RX1               6
#define GH3X2X_AGC_REG_TIA_GAIN_RX2               7
#define GH3X2X_AGC_REG_TIA_GAIN_RX3               8
#define GH3X2X_AGC_REG_AGC_SRC                    9
#define GH3X2X_AGC_REG_BG_CANCEL_RX0              10
#define GH3X2X_AGC_REG_BG_CANCEL_RX1              11
#define GH3X2X_AGC_REG_BG_CANCEL_RX2              12
#define GH3X2X_AGC_REG_BG_CANCEL_RX3              13
#define GH3X2X_AGC_REG_SR_MULIPLIPER              14
#define GH3X2X_AGC_ECG_PPG_RX_EN                  15







#define GH3X2X_ENGINEERING_MODE_CHNL_NUM_MAX      32

#define GH3X2X_ENGINEERING_MODE_INT_TIME_10_US    0
#define GH3X2X_ENGINEERING_MODE_INT_TIME_20_US    1
#define GH3X2X_ENGINEERING_MODE_INT_TIME_30_US    2
#define GH3X2X_ENGINEERING_MODE_INT_TIME_39_US    3
#define GH3X2X_ENGINEERING_MODE_INT_TIME_79_US    4
#define GH3X2X_ENGINEERING_MODE_INT_TIME_158_US   5
#define GH3X2X_ENGINEERING_MODE_INT_TIME_316_US   6


#define GH3X2X_ENGINEERING_MODE_TIA_GAIN_10_K     0
#define GH3X2X_ENGINEERING_MODE_TIA_GAIN_25_K     1
#define GH3X2X_ENGINEERING_MODE_TIA_GAIN_50_K     2
#define GH3X2X_ENGINEERING_MODE_TIA_GAIN_75_K     3
#define GH3X2X_ENGINEERING_MODE_TIA_GAIN_100_K    4
#define GH3X2X_ENGINEERING_MODE_TIA_GAIN_250_K    5
#define GH3X2X_ENGINEERING_MODE_TIA_GAIN_500_K    6
#define GH3X2X_ENGINEERING_MODE_TIA_GAIN_750_K    7
#define GH3X2X_ENGINEERING_MODE_TIA_GAIN_1000_K   8
#define GH3X2X_ENGINEERING_MODE_TIA_GAIN_1250_K   9
#define GH3X2X_ENGINEERING_MODE_TIA_GAIN_1500_K   10
#define GH3X2X_ENGINEERING_MODE_TIA_GAIN_1750_K   11
#define GH3X2X_ENGINEERING_MODE_TIA_GAIN_2000_K   12

#define GH3X2X_FRAME_FLAG_NUM                     8



typedef struct
{
   GU32 unFunctionID;            //Function id     use maroc exp: GH3X2X_FUNCTION_HR
   GU8  uchSampleRateChangeEn;   // 0: do not need change sample rate   1: need change sample rate
   GU16 usSampleRate;            // sample rate (unit Hz  range 5Hz~1000Hz)
   GU8  uchIntTimeChangeEn;      // 0: do not need change integrator time  1: need change intergrator time
   GU8  uchIntTime;              // use maroc exp: GH3X2X_ENGINEERING_MODE_INT_TIME_10_US
   GU8  uchTiaGainChangeEn;      // 0: do not need change TIA gain    1: need change TIA gain
   GU8  uchTiaGain[GH3X2X_ENGINEERING_MODE_CHNL_NUM_MAX];  //TIA gain for every channel, use maroc GH3X2X_ENGINEERING_MODE_TIA_GAIN_10_K
   GU8  uchLedCurrentChangeEn;      // 0: do not need change LED current    1: need change change LED current
   GU8  uchLedDrv0Current[GH3X2X_ENGINEERING_MODE_CHNL_NUM_MAX];  //LED driver0 current for every channel (unit: mA   range: 0~200)
   GU8  uchLedDrv1Current[GH3X2X_ENGINEERING_MODE_CHNL_NUM_MAX];  //LED driver1 current for every channel (unit: mA   range: 0~200)
} STGh3x2xEngineeringModeSampleParam;



typedef struct
{
    GU8 uchEnable;
    GU8 uchSlotAdc;
    GU8  uchCurrentTiaGain;
    GU8  uchCurrentDrv;
    GU8  uchTiaGainUpLimit;
    GU8  uchHaveBakRegFlag;
    GU8  uchBakRegAgcEn;
    GU8  uchBakRegAgcSrc;
    GU8  uchBakRegLedCurrentMin;
    GU8  uchBakRegLedCurrentMax;
    GU8  uchBakTiaGain;
} STGh3x2xNewAgcMainChnlInfo;

typedef struct
{
    GU16 usAnalysisDiscardLen;
    GU16 usAnalysisEndLen;
}STGh3x2xNewAgcSubChnlSlotInfo;

typedef struct
{
    GU8 uchEn;
    GU8 uchSlotAdcNo;
    GU8 uchCurrentGain;
    GU32 unLastRawdata;
    GU32 unLastLastRawdata;
    GU32 unRawdataSum;
    GU16 usAnalysisCnt;
    GU16 usMoreThanUpLimitPointCnt;
    GU16 usMoreThanUpLimitPointCntForBiggerGain;
    GU16 usTrigeUpLimitForBiggerGain;   //  rawdate/256
}STGh3x2xNewAgcSubChnlRxInfo;


extern STGh3x2xNewAgcMainChnlInfo  * const g_pstGh3x2xNewAgcMainChnlInfo;
extern STGh3x2xNewAgcSubChnlSlotInfo  * const g_pstGh3x2xNewAgcSubChnlSlotInfo;
extern STGh3x2xNewAgcSubChnlRxInfo    * const g_pstGh3x2xNewAgcSubChnlRxInfo;
extern const GU8 g_uchNewAgcSlotNumLimit;
extern const GU8 g_uchNewAgcSubChnlNumLimit;





void ResisterpGh3x2xModuleFunList(STGh3x2xModuleFunList *pGh3x2xModuleFunList);

#if 0
void GH3X2X_DumpInit(void);
void GH3X2X_DumpInitNullFun(void);
void GH3X2X_BgLevelSet(GU16 usRegVal);
void GH3X2X_BgLevelSetNullFun(GU16 usRegVal);
void GH3X2X_DumpModeSet(GU16 usRegVal);
void GH3X2X_DumpModeSetNullFun(GU16 usRegVal);
void GH3X2X_MemSetSpo2IncompleteArr(void);
void GH3X2X_MemSetSpo2IncompleteArrNullFun(void);
void GH3X2X_MemSetHbaIncompleteArr(void);
void GH3X2X_MemSetHbaIncompleteArrNullFun(void);
#endif

void GH3x2xAlgoInfoRecordInputDataNulFun(GS32 *snRawdataArray,GU8 uchChannelMapCnt ,GU32 unTimeStamp ,GU8 *uchChannelMapArr ,GU32 unFunctionID);
void GH3x2xAlgoInfoRecordInputData(GS32 *snRawdataArray,GU8 uchChannelMapCnt ,GU32 unTimeStamp ,GU8 *uchChannelMapArr ,GU32 unFunctionID);
void GH3x2xAlgoInfoRecordOutputDataNullFun(GS32 *psnAlgoResult, GU8 uchResultNum);
void GH3x2xAlgoInfoRecordOutputData(GS32 *psnAlgoResult, GU8 uchResultNum);








//extern GU8 * const g_uchPacketPayloadArr;//[GH3X2X_UPROTOCOL_PAYLOAD_LEN_MAX];

extern STGh3x2xProtocolData * const g_pstGh3x2xProtocolData;//[GH3X2X_UPROTOCOL_PAYLOAD_LEN_MAX];


extern void GH3X2X_ProtocolFrameIdClean(GU8 uchFunctionID);




/**
 * @fn     void GH3X2X_RegisterHookFunc(void (*pInitHookFunc)(void),
 *                          void (*pStartHookFunc)(void),
 *                             void (*pStopHookFunc)(void),
 *                           void (*pGetRawdataHookFunc)(GU8 *puchReadDataBuffer, GU16 usReadDataLen))
 *
 * @brief  Register hook function callback.
 *
 * @attention   None
 *
 * @param[in]   pInitHookFunc           pointer to init hook function
 * @param[in]   pStartHookFunc          pointer to start hook function
 * @param[in]   pStopHookFunc           pointer to stop hook function
 * @param[in]   pGetRawdataHookFunc     pointer to get rawdata hook function
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_RegisterHookFunc(void (*pInitHookFunc)(void),
                             void (*pStartHookFunc)(void),
                             void (*pStopHookFunc)(void),
                             void (*pGetRawdataHookFunc)(GU8 *puchReadDataBuffer, GU16 usReadDataLen));

/**
 * @fn     void GH3X2X_RegisterDelayUsCallback(void (*pPlatformDelayUsFunc)(GU16 usUsec))
 *
 * @brief  Register delay us callback function, should register delay us function in platforms
 *
 * @attention   None
 *
 * @param[in]   pPlatformDelayUsFunc      pointer to delay us function
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_RegisterDelayUsCallback(void (*pPlatformDelayUsFunc)(GU16 usUsec));

/**
 * @fn     void GH3X2X_WriteReg(GU16 usRegAddr, GU16 usRegValue)
 *
 * @brief  Write register via i2c or spi
 *
 * @attention   None
 *
 * @param[in]   usRegAddr       register addr
 * @param[in]   usRegValue      register data
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_WriteReg(GU16 usRegAddr, GU16 usRegValue);

/**
 * @fn     GU16 GH3X2X_ReadReg(GU16 usRegAddr)
 *
 * @brief  Read register via i2c or spi
 *
 * @attention   None
 *
 * @param[in]   usRegAddr      register addr
 * @param[out]  None
 *
 * @return  register data
 */
GU16 GH3X2X_ReadReg(GU16 usRegAddr);

/**
 * @fn     GU32 GH3X2X_ReadRegs(GU16 usRegAddr, GU16 *pusRegValueBuffer, GU16 usLen)
 *
 * @brief  Read multi register via i2c or spi
 *
 * @attention   None
 *
 * @param[in]   usRegAddr           register addr
 * @param[in]   pusRegValueBuffer   pointer to register value buffer
 * @param[in]   uchLen              registers len
 * @param[out]  None
 *
 * @return  None
 */
GU32 GH3X2X_ReadRegs(GU16 usRegAddr, GU16 *pusRegValueBuffer, GU16 usLen);

/**
 * @fn     void GH3X2X_ReadFifo(GU8 *puchDataBuffer, GU16 usLen)
 *
 * @brief  Read fifo via i2c or spi
 *
 * @attention   None
 *
 * @param[in]   puchDataBuffer      pointer to buffer for fifo rawdata
 * @param[in]   usLen               fifo bytes len
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_ReadFifo(GU8 *puchDataBuffer, GU16 usLen);

/**
 * @fn     void GH3X2X_WriteRegBitField(GU16 usRegAddr, GU8 uchLsb, GU8 uchMsb, GU16 usValue)
 *
 * @brief  write register bit field via i2c or spi
 *
 * @attention   None
 *
 * @param[in]   usRegAddr      register addr
 * @param[in]   uchLsb         lsb of bit field
 * @param[in]   uchMsb         msb of bit field
 * @param[in]   usRegValue     register data
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_WriteRegBitField(GU16 usRegAddr, GU8 uchLsb, GU8 uchMsb, GU16 usValue);

/**
 * @fn     GU16 GH3X2X_ReadRegBitField(GU16 usRegAddr, GU8 uchLsb, GU8 uchMsb)
 *
 * @brief  Read register bit field via i2c or spi
 *
 * @attention   None
 *
 * @param[in]   usRegAddr      register addr
 * @param[in]   uchLsb         lsb of bit field
 * @param[in]   uchMsb         msb of bit field
 * @param[out]  None
 *
 * @return  register bit field data
 */
GU16 GH3X2X_ReadRegBitField(GU16 usRegAddr, GU8 uchLsb, GU8 uchMsb);

/**
 * @fn     GS8 GH3X2X_RegisterSpiOperationFunc(GU8 (*pSpiWriteFunc)(GU8 uchWriteBytesArr[], GU16 usWriteLen),
 *                                     GU8 (*pSpiReadFunc)(GU8 uchReadBytesArr[], GU16 usMaxReadLen),
 *                                   void (*pSpiCsCtrlFunc)(GU8 uchCsLevelHigh))
 *
 * @brief  Register spi operation function callback
 *
 * @attention   None
 *
 * @param[in]   pSpiWriteFunc       pointer to spi write function
 * @param[in]   pSpiReadFunc        pointer to spi read function
 * @param[in]   pSpiCsCtrlFunc      pointer to spi cs operation function
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR  spi operation function pointer parameter error
 */
GS8 GH3X2X_RegisterSpiOperationFunc(GU8 (*pSpiWriteFunc)(GU8 uchWriteBytesArr[], GU16 usWriteLen),
                                    GU8 (*pSpiReadFunc)(GU8 uchReadBytesArr[], GU16 usMaxReadLen),
                                    void (*pSpiCsCtrlFunc)(GU8 uchCsLevelHigh));











/**
 * @fn     GS8 GH3X2X_RegisterSpiHwCsOperationFunc(GU8 (*pSpiWriteFunc)(GU8 uchWriteBytesArr[], GU16 usWriteLen),
 *                                   GU8 (*pSpiWriteF0ReadFunc)(GU8 uchReadBytesArr[], GU16 usMaxReadLen)
 *                                  )
 *
 * @brief  Register spi operation function callback
 *
 * @attention   None
 *
 * @param[in]   pSpiWriteFunc                pointer to spi write function
 * @param[in]   pSpiWriteF1ReadFunc        pointer to spi write f1 and read function
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR  spi operation function pointer parameter error
 */
GS8 GH3X2X_RegisterSpiHwCsOperationFunc(GU8 (*pSpiWriteFunc)(GU8 uchWriteBytesArr[], GU16 usWriteLen),
                                    GU8 (*pSpiWriteF1ReadFunc)(GU8 uchReadBytesArr[], GU16 usMaxReadLen)
                                    );








/**
 * @fn     GS8 GH3X2X_RegisterI2cOperationFunc(GU8 uchI2cIdLowTwoBitsSelect,
 *                                GU8 (*pI2cWriteFunc)(GU8 uchDeviceId, const GU8 uchWriteBytesArr[], GU16 usWriteLen),
 *                                GU8 (*pI2cReadFunc)(GU8 uchDeviceId, const GU8 uchCmddBytesArr[], GU16 usCmddLen,
 *                                                      GU8 uchReadBytesArr[], GU16 usMaxReadLen))
 *
 * @brief  Register i2c operation function callback
 *
 * @attention   None
 *
 * @param[in]   uchI2cIdLowTwoBitsSelect    i2c low two bits addr selected, ref EMGh3x2xI2cIdSelect
 * @param[in]   pI2cWriteFunc               pointer to i2c write function
 * @param[in]   pI2cReadFunc                pointer to i2c read function
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR  i2c operation function pointer parameter error
 */
GS8 GH3X2X_RegisterI2cOperationFunc(GU8 uchI2cIdLowTwoBitsSelect,
                                GU8 (*pI2cWriteFunc)(GU8 uchDeviceId, const GU8 uchWriteBytesArr[], GU16 usWriteLen),
                                GU8 (*pI2cReadFunc)(GU8 uchDeviceId, const GU8 uchCmddBytesArr[], GU16 usCmddLen,
                                                        GU8 uchReadBytesArr[], GU16 usMaxReadLen));

/**
 * @fn     GS8 GH3X2X_ExitLowPowerMode(void)
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
GS8 GH3X2X_ExitLowPowerMode(void);

/**
 * @fn     GS8 GH3X2X_EnterLowPowerMode(void)
 *
 * @brief  Enter lowpower mode, in this mode, can't read&write reg with gh3x2x
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
GS8 GH3X2X_EnterLowPowerMode(void);

/**
 * @fn     void GH3X2X_RegisterResetPinControlFunc(void (*pResetPinLevelControlFunc)(GU8 uchPinLevel))
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
void GH3X2X_RegisterResetPinControlFunc(void (*pResetPinLevelControlFunc)(GU8 uchPinLevel));

/**
 * @fn     GS8 GH3X2X_HardReset(void)
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
GS8 GH3X2X_HardReset(void);

/**
 * @fn     void GH3X2X_SendCmd(GU8 uchCmd)
 *
 * @brief  Send cmd via i2c or spi
 *
 * @attention   None
 *
 * @param[in]   uchCmd      cmd
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_SendCmd(GU8 uchCmd);

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
void GH3X2X_WearDetectEnable(EMWearDetectEnableType emWearDetectEnable);

/**
 * @fn     GS8 GH3X2X_WearDetectSwitchTo(EMWearDetectType emWearDetectType, EMWearDetectForceSwitch emForceSwitch)
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
GS8 GH3X2X_WearDetectSwitchTo(EMWearDetectType emWearDetectType, EMWearDetectForceSwitch emForceSwitch);

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
EMSkinColorStatusType GH3X2X_GetSkinStatus(void);

/**
 * @fn     GS8 GH3X2X_SoftReset(void)
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
GS8 GH3X2X_SoftReset(void);

/**
 * @fn     GS8 GH3X2X_LoadNewRegConfigArr(const STGh3x2xReg *pstRegConfigArr, GU16 usRegConfigLen)
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
GS8 GH3X2X_LoadNewRegConfigArr(const STGh3x2xReg *pstRegConfigArr, GU16 usRegConfigLen);

/**
 * @fn     GS8 GH3X2X_DumpRegs(STGh3x2xReg *pstDumpRegsArr, GU16 usDumpRegsStartAddr, GU16 usDumpRegsLen)
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
GS8 GH3X2X_DumpRegs(STGh3x2xReg *pstDumpRegsArr, GU16 usDumpRegsStartAddr, GU16 usDumpRegsLen);

/**
 * @fn     void *GH3X2X_Memcpy(void *pDest, const void *pSrc, GU32 unByteSize)
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
void *GH3X2X_Memcpy(void *pDest, const void *pSrc, GU32 unByteSize);

/**
 * @fn     void *GH3X2X_Memset(void* pDest, GCHAR chVal, GU32 unByteSize)
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
void *GH3X2X_Memset(void* pDest, GCHAR chVal, GU32 unByteSize);

/**
 * @fn     GU32 GH3X2X_Strlen(const GCHAR *pszSrc)
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
GU32 GH3X2X_Strlen(const GCHAR *pszSrc);

/**
 * @fn     void GH3X2X_HbaAlgoChnlMapDefultSet(void)

 *
 * @brief  Set algo channel group Defult value
 *
 * @attention
 *
 *
 * @return  None
 */
void GH3X2X_HbaAlgoChnlMapDefultSet(void);

/**
 * @fn     void GH3X2X_HrvAlgoChnlMapDefultSet(void)

 *
 * @brief  Set algo channel group Defult value
 *
 * @attention
 *
 *
 * @return  None
 */
void GH3X2X_HrvAlgoChnlMapDefultSet(void);

/**
 * @fn     void GH3X2X_Spo2AlgoChnlMapDefultSet(void)

 *
 * @brief  Set algo channel group Defult value
 *
 * @attention
 *
 *
 * @return  None
 */
void GH3X2X_Spo2AlgoChnlMapDefultSet(void);

/**
 * @fn     GS8 GH3X2X_CommunicateConfirm(void)
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
GS8 GH3X2X_CommunicateConfirm(void);

/**
 * @fn     GS8 GH3X2X_Init(const STGh3x2xInitConfig *pstGh3x2xInitConfigParam)
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
GS8 GH3X2X_Init(const STGh3x2xInitConfig *pstGh3x2xInitConfigParam);

/**
 * @fn     GS8 GH3X2X_StartSampling(void)
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
GS8 GH3X2X_StartSampling(void);

/**
 * @fn     GS8 GH3X2X_StopSampling(void)
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
GS8 GH3X2X_StopSampling(void);

/**
 * @fn     GU16 GH3X2X_GetIrqStatus(void)
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
GU16 GH3X2X_GetIrqStatus(void);

/**
 * @fn     GS16 GH3X2X_ReadFifodata(GU8 *puchGh3x2xReadFifoData, GU16* pusReadFifoDataLen)
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
GS16 GH3X2X_ReadFifodata(GU8 *puchGh3x2xReadFifoData, GU16* pusReadFifoDataLen ,GU16 usFifoReadByteNum);


/**
 * @fn     GS16 GH3X2X_GetRawdata(STGh3x2xRawdata *pstGh3x2xRawdata, GU16* usFifoLength)
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
 * @retval  #GH3X2X_RET_READ_FIFO_CONTINUE      return fifo not empty
 */
GS16 GH3X2X_GetRawdata(STGh3x2xRawdata *pstGh3x2xRawdata, GU16* usFifoLength);

/**
 * @fn     void GH3X2X_UnpackRawdataPackage(STGh3x2xSlotRawdata *pstSlotRawdataArr,
 *                                    GU8 *puchReadRawdataBuffer, GU16 usReadRawdataLen)
 *
 * @brief  Unpack to 8 slot rawdata from read fifo data buffer
 *
 * @attention   This function should use in get rawdata hook;
 *              param pstSlotRawdataArr size must equal 8, defile like stSlotRawdataArr[8].
 *
 * @param[out]  pstSlotRawdataArr       pointer to 8 slot rawdata struct output
 * @param[in]   puchReadRawdataBuffer   pointer to read fifo buffer
 * @param[in]   usReadRawdataLen        read fifo rawdata data len (bytes)
 *
 * @return  None
 */
void GH3X2X_UnpackRawdataPackage(STGh3x2xSlotRawdata *pstSlotRawdataArr,
                                    GU8 *puchReadRawdataBuffer, GU16 usReadRawdataLen);

/**
 * @fn     GS16 GH3X2X_GetRawdataWithChannelMap(STGh3x2xChannelRawdata *pstGh3x2xChannelRawdata,
 *                                       GU8 *puchReadRawdataBuffer, GU16* usFifoLength, GU8 uchChannelMapCnt,
 *                                       GU8 *puchChannelMapArr)
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
GS16 GH3X2X_GetRawdataWithChannelMap(STGh3x2xChannelRawdata *pstGh3x2xChannelRawdata,
                                        GU8 *puchReadRawdataBuffer, GU16* usFifoLength, GU8 uchChannelMapCnt,
                                        GU8 *puchChannelMapArr);

/**
 * @fn     GS8 GH3X2X_ChannelMapRawdataClear(STGh3x2xChannelRawdata *pstGh3x2xChannelRawdata)
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
GS8 GH3X2X_ChannelMapRawdataClear(STGh3x2xChannelRawdata *pstGh3x2xChannelRawdata);

/**
 * @fn     GS8 GH3X2X_UnpackRawdataWithChannelMap(STGh3x2xChannelRawdata *pstGh3x2xChannelRawdata,
 *               GU8 *puchReadRawdataBuffer, GU16 usReadRawdataLen, GU8 uchChannelMapCnt, GU8 *puchChannelMapArr)
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
GS8 GH3X2X_UnpackRawdataWithChannelMap(STGh3x2xChannelRawdata *pstGh3x2xChannelRawdata,
                GU8 *puchReadRawdataBuffer, GU16 usReadRawdataLen, GU8 uchChannelMapCnt, GU8 *puchChannelMapArr);

/**
 * @fn     GS8 GH3X2X_SlotEnableConfig(GU16 usSlotEnableConfig, EMSlotEnableConfigType emSlotEnableConfigType)
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
GS8 GH3X2X_SlotEnableConfig(GU16 usSlotEnableConfig, EMSlotEnableConfigType emSlotEnableConfigType);

/**
 * @fn     GS8 GH3X2X_FifoWatermarkThrConfig(GU16 usFifoWatermarkThr)
 *
 * @brief  Fifo water mark threshold config
 *
 * @attention   Watermark threshold val must in (2, 800]. if val <= 2, will set 3, if val > 800, set 800;
 *              Be careful that fifo_use_cnt greater than val, gh3x2x willn't generate fifo_watermark interrupt after!
 *
 * @param[in]  usFifoWatermarkThr         watermark threshold val
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 */
GS8 GH3X2X_FifoWatermarkThrConfig(GU16 usFifoWatermarkThr);

/**
 * @fn     GU16 GH3X2X_GetFifoWatermarkThr(void)
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
GU16 GH3X2X_GetFifoWatermarkThr(void);

/**
 * @fn     void GH3X2X_SetMaxNumWhenReadFifo(GU16 usMaxNum)
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
void GH3X2X_SetMaxNumWhenReadFifo(GU16 usMaxNum);

/**
 * @fn     GS8 GH3X2X_SlotLedCurrentConfig(GU8 uchSlotIndex, GU8 uchDrvIndex, GU8 uchCurrentVal)
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
GS8 GH3X2X_SlotLedCurrentConfig(GU8 uchSlotIndex, GU8 uchDrvIndex, GU8 uchCurrentVal);

/**
 * @fn     GU8 GH3X2X_GetSlotLedCurrent(GU8 uchSlotIndex, GU8 uchDrvIndex)
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
GU8 GH3X2X_GetSlotLedCurrent(GU8 uchSlotIndex, GU8 uchDrvIndex);

/**
 * @fn     GS8 GH3X2X_SlotLedTiaGainConfig(GU8 uchSlotIndex, GU8 uchAdcIndex, GU8 uchGainVal)
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
GS8 GH3X2X_SlotLedTiaGainConfig(GU8 uchSlotIndex, GU8 uchAdcIndex, GU8 uchGainVal);

/**
 * @fn     GU8 GH3X2X_GetSlotLedTiaGain(GU8 uchSlotIndex, GU8 uchAdcIndex)
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
GU8 GH3X2X_GetSlotLedTiaGain(GU8 uchSlotIndex, GU8 uchAdcIndex);

/**
 * @fn     GS8 GH3X2X_AdcBgcThrConfig(GU8 uchAdcIndex, GU8 uchBgcThrVal)
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
GS8 GH3X2X_AdcBgcThrConfig(GU8 uchAdcIndex, GU8 uchBgcThrVal);

/**
 * @fn     GU8 GH3X2X_GetAdcBgcThr(GU8 uchAdcIndex)
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
GU8 GH3X2X_GetAdcBgcThr(GU8 uchAdcIndex);

/**
 * @fn     GS8 GH3X2X_IrqWidthConfig(GU16 usIrqPulseWidth, GU16 usIrqColdWidth)
 *
 * @brief  Irq width config
 *
 * @attention   Irq val must in [0, 2047]. real_width = (val + 1) * low_clk_tick;
 *              low_clk default 32k, if set ext clk enable that use low_ext_clk;
 *
 * @param[in]  usIrqPulseWidth         irq pulse width val
 * @param[in]  usIrqColdWidth          irq cold width val
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 */
GS8 GH3X2X_IrqWidthConfig(GU16 usIrqPulseWidth, GU16 usIrqColdWidth);

/**
 * @fn     GS8 GH3X2X_IrqModeConfig(EMIrqModeConfig emIrqMode)
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
GS8 GH3X2X_IrqModeConfig(EMIrqModeConfig emIrqMode);

/**
 * @fn     GS8 GH3X2X_HsiClockCalibration(void)
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
GS8 GH3X2X_HsiClockCalibration(void);

/**
 * @fn     GS8 GH3X2X_LsiClockCalibration(void)
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
GS8 GH3X2X_LsiClockCalibration(void);

/**
 * @fn     GU8 GH3X2X_GetLedAgcState(void)
 *
 * @brief  get agc state
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  GH3X2X_LED_AGC_ENABLE or GH3X2X_LED_AGC_DISABLE
 */
GU8 GH3X2X_GetLedAgcState(void);

/**
 * @fn     void GH3X2X_SoftLedADJAutoADJInt(void)
 *
 * @brief  init some agc parameter
 *
 * @attention
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_SoftLedADJAutoADJInt(void);

/**
 * @fn     void GH3X2X_LedAgcProcess(GU8* puchReadFifoBuffer, GU16 usFifoLen)
 *
 * @brief  led agc process function
 *
 * @attention   None
 *
 * @param[in]   puchReadFifoBuffer         pointer to read fifo data buffer
 * @param[in]   usFifoLen           read fifo data buffer length
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_LedAgcProcess(GU8* puchReadFifoBuffer, GU16 usFifoLen);

/**
 * @fn     void GH3X2X_LedAgcProcessEx(GU8* puchReadFifoBuffer)
 *
 * @brief  led agc process function
 *
 * @attention   None
 *
 * @param[in]   puchReadFifoBuffer      pointer to read fifo data buffer
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_LedAgcProcessEx(GU8* puchReadFifoBuffer);

/**
 * @fn     void GH3X2X_RegisterReadPinStatusFunc(GU8 (*pReadIntPinStatusFunc)(void),
 *                                       GU8 (*pReadResetPinStatusFunc)(void),
 *                                      GU8 (*pReadSpcsPinStatusFunc)(void),
 *                                     GU8 (*pReadSpdoPinStatusFunc)(void))
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
void GH3X2X_RegisterReadPinStatusFunc(GU8 (*pReadIntPinStatusFunc)(void),
                                      GU8 (*pReadResetPinStatusFunc)(void),
                                      GU8 (*pReadSpcsPinStatusFunc)(void),
                                      GU8 (*pReadSpdoPinStatusFunc)(void));

/**
 * @fn     GCHAR *GH3X2X_GetDriverLibVersion(void)
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
GCHAR *GH3X2X_GetDriverLibVersion(void);
GCHAR *GH3X2X_GetProtocolVersion(void);

/**
 * @fn     void GH3X2X_AlgoRunModeConfig(GU8 uchAlgoRunMode)
 *
 * @brief  Config run mode of algorithm.
 *
 * @attention   None
 *
 * @param[in]   uchAlgoRunMode         GH3X2X_ALGORITHM_RUN_ALONE/GH3X2X_ALGORITHM_RUN_SIMULTANEOUSLY
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_AlgoRunModeConfig(GU8 uchAlgoRunMode);

/**
 * @fn     GU8 GH3X2X_SetHbAlgoOutMaxCnt(GU16 usAlgoOutMaxCnt)
 *
 * @brief  Set max count of algorithm output of one package rawdata
 *
 * @attention   This function is used for avoid overstep the algo result array boundary when excute algo
 *
 * @param[in]   usAlgoOutMaxCnt     max cnt of algo output
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_SetHbAlgoOutMaxCnt(GU16 usAlgoOutMaxCnt);

/**
 * @fn     GU8 GH3X2X_SetHrvAlgoOutMaxCnt(GU16 usAlgoOutMaxCnt)
 *
 * @brief  Set max count of algorithm output of one package rawdata
 *
 * @attention   This function is used for avoid overstep the algo result array boundary when excute algo
 *
 * @param[in]   usAlgoOutMaxCnt     max cnt of algo output
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_SetHrvAlgoOutMaxCnt(GU16 usAlgoOutMaxCnt);


/**
 * @fn     GU8 GH3X2X_SetSpo2AlgoOutMaxCnt(GU16 usAlgoOutMaxCnt)
 *
 * @brief  Set max count of algorithm output of one package rawdata
 *
 * @attention   This function is used for avoid overstep the algo result array boundary when excute algo
 *
 * @param[in]   usAlgoOutMaxCnt     max cnt of algo output
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_SetSpo2AlgoOutMaxCnt(GU16 usAlgoOutMaxCnt);

/**
 * @fn     GU8 GH3X2X_SetEcgAlgoOutMaxCnt(GU16 usAlgoOutMaxCnt)
 *
 * @brief  Set max count of algorithm output of one package rawdata
 *
 * @attention   This function is used for avoid overstep the algo result array boundary when excute algo
 *
 * @param[in]   usAlgoOutMaxCnt     max cnt of algo output
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_SetEcgAlgoOutMaxCnt(GU16 usAlgoOutMaxCnt);

/**
 * @fn     GS8 GH3X2X_GetRawdataForAlgorithm(GU8 *puchPackageDataBuffer)
 *
 * @brief  Get rawdata package for algorithm
 *
 * @attention   buffer size must define greater or equal to ((fifo_thr + n) * 4 + 5)
 *
 * @param[in]   usReadByteNum  read byte num
 * @param[out]  puchPackageDataBuffer   pointer to package data output
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                   return successfully
 * @retval  #GH3X2X_RET_READ_FIFO_CONTINUE   fifo is not empty
 * @retval  #GH3X2X_RET_RESOURCE_ERROR       resource error
 */
GS8 GH3X2X_GetRawdataForAlgorithm(GU8 *puchPackageDataBuffer ,GU16 usReadByteNum);


/**
 * @fn     GS8 GH3X2X_HbAlgorithmDeInit(void)
 *
 * @brief  Hb algorithm deinitialization
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm deinit error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */
GS8 GH3X2X_HbAlgorithmDeInit(void);

/**
 * @fn     GS8 GH3X2X_HbAlgorithmInit(void)
 *
 * @brief  Hb algorithm initialization
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm init error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */
GS8 GH3X2X_HbAlgorithmInit(void);

/**
 * @fn     GS8 GH3X2X_HbAlgorithmCalc(STHbAlgoResult pstAlgoResArr[], GU16 pusAlgoResIndexArr[], GU16* pusAlgoResCnt,
 *                        GU8 *puchReadDataBuffer, GU8 uchChannelMapCnt, GU8 *puchChannelMapArr,
                          STGsensorRawdata *pstGsAxisValueArr, GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *
 * @brief  Call Hb algorithm calc with read rawdata
 *
 * @attention   None
 *
 * @param[out]  pstAlgoResArr           hb algorithm result array
 * @param[out]  pusAlgoResIndexArr      Frame index of rawdata which output algo result
 * @param[out]  pusAlgoResCnt           pointer to the number of algorithm result
 * @param[in]   puchReadDataBuffer      pointer read data buffer from GH3X2X_GetRawdataForAlgorithm
 * @param[in]   uchChannelMapCnt        channel map array count
 * @param[in]   puchChannelMapArr       pointer channel map array
 * @param[in]   pstGsAxisValueArr       pointer to gsensor data array
 * @param[in]   usGsDataNum             gsensor data count
 * @param[in]   emGsSensitivity         gsensor sensitivity, @ref EMGsensorSensitivity
 *
 * @return  error code
 * @retval  #GH3X2X_RET_ALGO_HAS_REFRESHED      return algorithm run successfully, and output has refreshed
 * @retval  #GH3X2X_RET_OK                      return algorithm run successfully, but output hasn't refreshed
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return algorithm param error
 * @retval  #GH3X2X_RET_RESOURCE_ERROR          return algorithm ram no enough error
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm generic error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */



/**
 * @fn     GS8 GH3X2X_HbAlgorithmScenarioConfig(GU8 uchScenario)
 *
 * @brief  Hb algorithm scenario config
 *
 * @attention   None
 *
 * @param[in]  uchScenario     @ref hba_scenes_e
 *                             others: fixed 0(default) and return GH3X2X_RET_PARAMETER_ERROR
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return paramters error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */
GS8 GH3X2X_HbAlgorithmScenarioConfig(GU8 uchScenario);

/**
 * @fn     GS8 GH3X2X_HbAlgorithmOutputTimeConfig(GS32 nOutputTime)
 *
 * @brief  Hb algorithm output time config
 *
 * @attention   None
 *
 * @param[in]   nOutputTime          valid 7~GS32_MAX
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */
GS8 GH3X2X_HbAlgorithmOutputTimeConfig(GS32 nOutputTime);

/**
 * @fn     GS8 GH3X2X_HbAlgorithmSamplingFreqConfig(GU32 unSamplingFreq)
 *
 * @brief  Hb algorithm sampling frequency config
 *
 * @attention   None
 *
 * @param[in]   unFs           sampling frequency for algorithm
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */
GS8 GH3X2X_HbAlgorithmSamplingFreqConfig(GU32 unSamplingFreq);

/**
 * @fn     GS8 GH3X2X_HrvAlgorithmDeInit(void)
 *
 * @brief  Hrv algorithm deinitialization
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm deinit error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */
GS8 GH3X2X_HrvAlgorithmDeInit(void);

/**
 * @fn     GS8 GH3X2X_HrvAlgorithmInit(void)
 *
 * @brief  Hrv algorithm initialization
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm init error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */
GS8 GH3X2X_HrvAlgorithmInit(void);

/**
 * @fn     GS8 GH3X2X_HrvAlgorithmCalc(STHbAlgoResult pstAlgoResArr[], GU16 pusAlgoResIndexArr[], GU16* pusAlgoResCnt,
 *                        GU8 *puchReadDataBuffer, GU8 uchChannelMapCnt, GU8 *puchChannelMapArr,
                          STGsensorRawdata *pstGsAxisValueArr, GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *
 * @brief  Call Hrv algorithm calc with read rawdata
 *
 * @attention   None
 *
 * @param[out]  pstAlgoResArr           hb algorithm result array
 * @param[out]  pusAlgoResIndexArr      Frame index of rawdata which output algo result
 * @param[out]  pusAlgoResCnt           pointer to the number of algorithm result
 * @param[in]   puchReadDataBuffer      pointer read data buffer from GH3X2X_GetRawdataForAlgorithm
 * @param[in]   uchChannelMapCnt        channel map array count
 * @param[in]   puchChannelMapArr       pointer channel map array
 * @param[in]   pstGsAxisValueArr       pointer to gsensor data array
 * @param[in]   usGsDataNum             gsensor data count
 * @param[in]   emGsSensitivity         gsensor sensitivity, @ref EMGsensorSensitivity
 *
 * @return  error code
 * @retval  #GH3X2X_RET_ALGO_HAS_REFRESHED      return algorithm run successfully, and output has refreshed
 * @retval  #GH3X2X_RET_OK                      return algorithm run successfully, but output hasn't refreshed
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return algorithm param error
 * @retval  #GH3X2X_RET_RESOURCE_ERROR          return algorithm ram no enough error
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm generic error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */



/**
 * @fn     GS8 GH3X2X_Spo2AlgorithmDeInit(void)
 *
 * @brief  Spo2 algorithm deinitialization
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm deinit error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */
GS8 GH3X2X_Spo2AlgorithmDeInit(void);

/**
 * @fn     GS8 GH3X2X_Spo2AlgorithmInit(void)
 *
 * @brief  Spo2 algorithm initialization
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm init error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */
GS8 GH3X2X_Spo2AlgorithmInit(void);

/**
 * @fn      GS8 GH3X2X_Spo2AlgorithmCalc(STSpo2AlgoResult pstAlgoResArr[], GU16 pusAlgoResIndexArr[],
 *                       GU16* pusAlgoResCnt, GU8 *puchReadDataBuffer, GU8 uchChannelMapCnt, GU8 *puchChannelMapArr,
 *                       STGsensorRawdata *pstGsAxisValueArr, GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *
 * @brief  Call spo2 algorithm calc with read rawdata
 *
 * @attention   None
 *
 * @param[out]  pstAlgoResArr           spo2 algorithm result array
 * @param[out]  pusAlgoResIndexArr      Frame index of rawdata which output algo result
 * @param[out]  pusAlgoResCnt           pointer to the number of algorithm result
 * @param[in]   puchReadDataBuffer      pointer read data buffer from GH3X2X_GetRawdataForAlgorithm
 * @param[in]   uchAllChannelMapCnt     channel map cnt
 * @param[in]   puchChannelMapArr       channel map array @ref GH3X2X_CHANNEL_MAP
 * @param[in]   pstGsAxisValueArr       pointer to gsensor data array
 * @param[in]   usGsDataNum             gsensor data count
 * @param[in]   emGsSensitivity         gsensor sensitivity, @ref EMGsensorSensitivity
 *
 * @return  error code
 * @retval  #GH3X2X_RET_ALGO_HAS_REFRESHED      return algorithm run successfully, and output has refreshed
 * @retval  #GH3X2X_RET_OK                      return algorithm run successfully, but output hasn't refreshed
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return algorithm param error
 * @retval  #GH3X2X_RET_RESOURCE_ERROR          return algorithm ram no enough error
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm generic error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */


/**
 * @fn     GS8 GH3X2X_EcgAlgorithmDeInit(void)
 *
 * @brief  Ecg algorithm deinitialization
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm deinit error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */
GS8 GH3X2X_EcgAlgorithmDeInit(void);

/**
 * @fn     GS8 GH3X2X_EcgAlgorithmInit(void)
 *
 * @brief  Ecg algorithm initialization
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm init error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */
GS8 GH3X2X_EcgAlgorithmInit(void);

/**
 * @fn     GS8 GH3X2X_EcgAlgorithmCalc(STEcgAlgoResult pstAlgoResArr[], GU16 pusAlgoResIndexArr[], GU16* pusAlgoResCnt,
 *                          GU8 *puchReadDataBuffer, GU8 uchEcgChannelMap, STGsensorRawdata *pstGsAxisValueArr,
                            GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *
 * @brief  Call ecg algorithm calc with read rawdata
 *
 * @attention   None
 *
 * @param[out]  pstAlgoResArr           ecg algorithm result array
 * @param[out]  pusAlgoResIndexArr      Frame index of rawdata which output algo result
 * @param[out]  pusAlgoResCnt           pointer to the number of algorithm result
 * @param[in]   puchReadDataBuffer      pointer read data buffer from GH3X2X_GetRawdataForAlgorithm
 * @param[in]   uchEcgChannelMap        ecg channel map @ref GH3X2X_CHANNEL_MAP
 * @param[in]   pstGsAxisValueArr       pointer to gsensor data array
 * @param[in]   usGsDataNum             gsensor data count
 * @param[in]   emGsSensitivity         gsensor sensitivity, @ref EMGsensorSensitivity
 *
 * @return  error code
 * @retval  #GH3X2X_RET_ALGO_HAS_REFRESHED      return algorithm run successfully, and output has refreshed
 * @retval  #GH3X2X_RET_OK                      return algorithm run successfully, but output hasn't refreshed
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return algorithm param error
 * @retval  #GH3X2X_RET_RESOURCE_ERROR          return algorithm ram no enough error
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm generic error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */


/**
 * @fn     void GH3X2X_RegisterAlgoCalcHookFunc(
 *             void (*pAlgorithmCalcHookFunc)(GU32 unRawdataFrameIndex, GS32 *pnRawdataFrame, GU8 uchRawdataFrameLen))
 *
 * @brief  Register algorithm calculate hook function callback
 *
 * @attention   callback by every algorithm calculation, hook function param brief that:
 *              unRawdataFrameIndex:  rawdata frame index
 *              pnRawdataFrame:       rawdata frame, format: gx, gy, gz, ch0.....chx
 *              uchRawdataFrameLen:   rawdata frame len
 *
 * @param[in]   pAlgorithmCalcHookFunc     pointer to algorithm calculate hook function
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_RegisterAlgoCalcHookFunc(
            void (*pAlgorithmCalcHookFunc)(GU32 unRawdataFrameIndex, GS32 *pnRawdataFrame, GU8 uchRawdataFrameLen));

/**
 * @fn     void GH3X2X_RegisterGetFirmwareVersionFunc(GCHAR* (*pGetFirmwareVersionFunc)(void))
 *
 * @brief  Register get firmware version function
 *
 * @attention   None
 *
 * @param[in]    pGetFirmwareVersionFunc    pointer to get firmware versionn function
 * @param[out]   None
 *
 * @return  None
 */
void GH3X2X_RegisterGetFirmwareVersionFunc(GCHAR* (*pGetFirmwareVersionFunc)(void));

/**
 * @fn     void GH3X2X_RegisterGetBootloaderVersionFunc(GCHAR* (*pGetFirmwareVersionFunc)(void))
 *
 * @brief  Register get Bootloader version function
 *
 * @attention   None
 *
 * @param[in]    pGetFirmwareVersionFunc    pointer to get Bootloader versionn function
 * @param[out]   None
 *
 * @return  None
 */

void GH3X2X_RegisterGetBootloaderVersionFunc(GCHAR* (*pGetFirmwareVersionFunc)(void));

/**
 * @fn     EMUprotocolParseCmdType GH3X2X_UprotocolParseHandler(GU8 *puchRespondBuffer, GU16 *pusRespondLen,
 *                                                     GU8 *puchRecvDataBuffer, GU16 usRecvLen)
 *
 * @brief  universal protocol parse handler, parse protocol receive data
 *
 * @attention   None
 *
 * @param[out]   puchRespondBuffer    pointer to respond buffer
 * @param[out]   pusRespondLen        pointer to respond length
 * @param[in]    puchRecvDataBuffer   pointer to receive data buffer
 * @param[in]    usRecvLen            receive data len
 *
 * @return  @ref EMUprotocolParseCmdType
 */
EMUprotocolParseCmdType GH3X2X_UprotocolParseHandler(GU8 *puchRespondBuffer, GU16 *pusRespondLen,
                                                     GU8 *puchRecvDataBuffer, GU16 usRecvLen);

/**
 * @fn     GU8 GH3X2X_UprotocolPacketMaxLenConfig(GU8 uchPacketMaxLen)
 *
 * @brief  packet max len support, default len is 243
 *
 * @attention   uchPacketMaxLen val must at [20, 243]
 *
 * @param[out]   None
 * @param[in]    uchPacketMaxLen      packet max len
 *
 * @return  packet max len after config
 */
GU8 GH3X2X_UprotocolPacketMaxLenConfig(GU8 uchPacketMaxLen);

/**
 * @fn     GS8 GH3X2X_SendRawDataPkg(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
 *                    GU16 usFifoRawdataLen, GU8 uchChannelMapArr[], GU8 uchChannelMapCnt,
 *                   STGsensorRawdata *pstGsAxisValueArr, GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity,
 *                   GU8 uchChannelMapId, GU32 *punLastIncompleteMark, GU32 *punIncompleteRawdataArr)
 *
 * @brief  Pack rawdata with g sensor data
 *
 * @attention   None
 *
 * @param[out]  puchRespondBuffer       pointer to protocol pkg data.
 * @param[out]  pusRespondLen           pointer to protocol pkg data length
 * @param[in]   puchFifoRawdata         pointer to data buffer of rawdata
 * @param[in]   usFifoRawdataLen        data num of rawdata buf
 * @param[in]   uchChannelMapArr        pointer to channel map array
 * @param[in]   uchChannelMapCnt        channel map array cnt, max:32
 * @param[in]   pstGsAxisValueArr       pointer to g sensor data
 * @param[in]   usGsDataNum             g sensor data number
 * @param[in]   emGsSensitivity         g sensor sensitivity
 * @param[in]   uchChannelMapId         channel map id(0 - 5)
 * @param[in]   punLastIncompleteMark   incomplete mark
 * @param[in]   punIncompleteRawdataArr incomplete data array
 * @return  error code
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return param error
 * @retval  #GH3X2X_RET_RAW_DATA_PKG_OVER       return already read all the package data
 * @retval  #GH3X2X_RET_RAW_DATA_PKG_CONTINUE   return should read rawdata package data again
 * @retval  #GH3X2X_RET_RESOURCE_ERROR          return not enough data
 */


/**
 * @fn     GS8 GH3X2X_CustomizeFuncStartedBitSet(void)
 *
 * @brief  Set Customize started bitmap
 *
 * @attention   None
 *
 * @param[out]  None
 * @param[in]   None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return algorithm run successfully
 */
GS8 GH3X2X_CustomizeFuncStartedBitSet(void);

/**
 * @fn     GS8 GH3X2X_CustomizeFuncStartedBitClear(void)
 *
 * @brief  Clear Customize started bitmap
 *
 * @attention   None
 *
 * @param[out]  None
 * @param[in]   None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return algorithm run successfully
 */
GS8 GH3X2X_CustomizeFuncStartedBitClear(void);

/**
 * @fn     GS8 GH3X2X_FifoWatermarkThrConfigEx(GU16 usFifoWatermarkThr)
 *
 * @brief  Fifo water mark threshold config
 *
 * @attention   Watermark threshold val must in (2, 800]. if val <= 2, will set 3, if val > 800, set 800;
 *              Be careful that fifo_use_cnt greater than val, gh3x2x willn't generate fifo_watermark interrupt after!
 *              Please configure according to usage, effect @GH3X2X_GetRawdataForAlgorithmEx last fifo Watermark irq!
 *
 * @param[in]   usFifoWatermarkThr         watermark threshold val
 * @param[out]  None
 *
 * @return  errcode
 * @retval  #GH3X2X_RET_OK               return successfully
 */
GS8 GH3X2X_FifoWatermarkThrConfigEx(GU16 usFifoWatermarkThr);

/**
 * @fn     GS8 GH3X2X_GetRawdataForAlgorithmEx(GU8 *puchPackageDataBuffer)
 *
 * @brief  Get rawdata package for algorithm
 *
 * @attention   buffer size must define greater or equal to ((fifo_thr + n) * 4 + 5)
 *
 * @param[in]   None
 * @param[out]  puchPackageDataBuffer   pointer to package data output
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK               return successfully
 * @retval  #GH3X2X_RET_RESOURCE_ERROR   resource error
 */
GS8 GH3X2X_GetRawdataForAlgorithmEx(GU8 *puchPackageDataBuffer ,GU16 uchFifoReadByteNum);

/**
 * @fn     GS8 GH3X2X_AllFuncDeinit(void)
 *
 * @brief  deinit all func
 *
 * @attention   if disable all or chip reset, need call this api
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm deinit error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */
GS8 GH3X2X_AllFuncDeinit(void);

/**
 * @fn     GS8 GH3X2X_AllFuncDeinitEx(void)
 *
 * @brief  deinit all func
 *
 * @attention   this api not deinit algorithm
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm deinit error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */
GS8 GH3X2X_AllFuncDeinitEx(void);

/**
 * @fn     GS8 GH3X2X_AlgoMemConfig(GU32 unMemSize)
 *
 * @brief  Config algorithm memory.
 *
 * @attention   Mem can only be static,such as global array
 *
 * @param[in]   puchMemAddr     memory addr for algorithm
 * @param[out]  unMemSize       memory size(unit:byte)
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_RESOURCE_ERROR          return algorithm mem config error
 */
GS8 GH3X2X_AlgoMemConfig( GU32 unMemSize);

/**
 * @fn     void GH3X2X_ReportIrqStatus(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU16 usIrqStatus)
 *
 * @brief  report irq status of Gh3x2x
 *
 * @attention   none
 *
 * @param[out]  puchRespondBuffer       pointer to protocol pkg data.
 * @param[out]  pusRespondLen           pointer to protocol pkg data length
 * @param[in]   usIrqStatus             irq status need report
 * @return  none
 */
void GH3X2X_ReportIrqStatus(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU16 usIrqStatus);



/**
 * @fn     void Gh3x2x_SetAdtConfirmPara(GU8 uchMoveThreshold, GU16 usMoveCntThreshold)
 *
 * @brief  Set some parameter of adt confirm
 *
 * @attention   None
 *
 * @param[in]   usMoveThreshold         threshold of gsensor data(sqrt(sum of square)) that can be judged as movement
 * @param[in]   usMoveCntThreshold      how many times of continuous overtake usMoveThreshold that can trig sampling
 * @param[out]  None
 *
 * @return  None
 */
void Gh3x2x_SetAdtConfirmPara(GU8 uchMoveThreshold, GU16 usMoveCntThreshold, GU16 usNotMoveCntThreshold, GU32 unIRTimeoutSecondsThreshold);

/**
 * @fn     GS8 GH3X2X_AdtFuncStartWithConfirm(void)
 *
 * @brief  Start adt function with move detect confirm
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_AdtFuncStartWithConfirm(void);

/**
 * @fn     GS8 GH3X2X_AdtFuncStopWithConfirm(void)
 *
 * @brief  stop adt function with move detect confirm
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_AdtFuncStopWithConfirm(void);

/**
 * @fn      void GH3X2X_MoveDetectByGsData(STGsensorRawdata* stGsData, GU16 usGsDataCnt, GU8 uchCheckWindowSize)

 *
 * @brief  Move detection by software.
 *
 * @attention   None
 *
 * @param[in]   stGsData        gsensor data buffer
 * @param[in]   usGsDataCnt     gsensor data count
 * @param[in]   uchCheckWindowSize      the window size of cheak moving peak num
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_MoveDetectByGsData(GS16* pusGsData, GU16 usGsDataCnt);



/**
 * @fn     void Gh3x2x_ResetMoveDetectByGsData(void)
 *
 * @brief  Reset movement detection by gsensor data
 *
 * @attention   When wear on,should call this function to close move detect
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  1: enabled, 0:disabled
 */
void Gh3x2x_ResetMoveDetectByGsData(void);

/**
 * @fn     GU8 Gh3x2x_GetAdtConfirmStatus(void)
 *
 * @brief  Get status of movement detected
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  GH3X2X_SENSOR_IS_NOT_MOVING/GH3X2X_SENSOR_IS_MOVING
 */
GU8 Gh3x2x_GetAdtConfirmStatus(void);

/**
 * @fn     GS8 GH3X2X_SendAdtRawdataPkg(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
 *                       STAlgoCalculateRes* pstAlgoRes, STGsensorRawdata *pstGsAxisValueArr,
 *                       GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *
 * @brief  pack GH3X2X adt rawdata and other type of data according to specific protocol
 *
 * @attention   None
 *
 * @param[out]  puchRespondBuffer       pointer to frame data package
 * @param[out]  pusRespondLen           pointer to data length of package
 * @param[in]   puchFifoRawdata         pointer to encrypted rawdata
 * @param[in]   pstAlgoRes              pointer to algorithm result
 * @param[in]   pstGsAxisValueArr       pointer to g sensor data
 * @param[in]   usGsDataNum             number of g sensor data
 * @param[in]   emGsSensitivity         sensitivity of g sensor
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return func start error
 * @retval  #GH3X2X_RET_NO_INITED_ERROR         gh3x2x has not inited
 */




/**
 * @fn     GS8 GH3X2X_SendAdtRawdataPkgEx(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
 *                      GU16 usFifoDataLen, STAlgoCalculateRes* pstAlgoRes, STGsensorRawdata *pstGsAxisValueArr,
                        GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity);
 *
 * @brief  pack GH3X2X adt rawdata and other type of data according to specific protocol
 *
 * @attention   None
 *
 * @param[out]  puchRespondBuffer       pointer to frame data package
 * @param[out]  pusRespondLen           pointer to data length of package
 * @param[in]   puchFifoRawdata         pointer to encrypted rawdata
 * @param[in]   usFifoDataLen           gh3x2x fifo data len in byte
 * @param[in]   pstAlgoRes              pointer to algorithm result
 * @param[in]   pstGsAxisValueArr       pointer to g sensor data
 * @param[in]   usGsDataNum             number of g sensor data
 * @param[in]   emGsSensitivity         sensitivity of g sensor
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return func start error
 * @retval  #GH3X2X_RET_NO_INITED_ERROR         gh3x2x has not inited
 */




/**
 * @fn     GS8 GH3X2X_SendZipAdtRawdataPkg(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
 *                       STAlgoCalculateRes* pstAlgoRes, STGsensorRawdata *pstGsAxisValueArr,
 *                       GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity)
 *
 * @brief  pack GH3X2X Zip adt rawdata and other type of data according to specific protocol
 *
 * @attention   None
 *
 * @param[out]  puchRespondBuffer       pointer to frame data package
 * @param[out]  pusRespondLen           pointer to data length of package
 * @param[in]   puchFifoRawdata         pointer to encrypted rawdata
 * @param[in]   pstAlgoRes              pointer to algorithm result
 * @param[in]   pstGsAxisValueArr       pointer to g sensor data
 * @param[in]   usGsDataNum             number of g sensor data
 * @param[in]   emGsSensitivity         sensitivity of g sensor
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return func start error
 * @retval  #GH3X2X_RET_NO_INITED_ERROR         gh3x2x has not inited
 */




/**
 * @fn     GS8 GH3X2X_SendZipAdtRawdataPkgEx(GU8 *puchRespondBuffer, GU16 *pusRespondLen, GU8 *puchFifoRawdata,
 *                      GU16 usFifoDataLen, STAlgoCalculateRes* pstAlgoRes, STGsensorRawdata *pstGsAxisValueArr,
                        GU16 usGsDataNum, EMGsensorSensitivity emGsSensitivity);
 *
 * @brief  pack GH3X2X Zip adt rawdata and other type of data according to specific protocol
 *
 * @attention   None
 *
 * @param[out]  puchRespondBuffer       pointer to frame data package
 * @param[out]  pusRespondLen           pointer to data length of package
 * @param[in]   puchFifoRawdata         pointer to encrypted rawdata
 * @param[in]   usFifoDataLen           gh3x2x fifo data len in byte
 * @param[in]   pstAlgoRes              pointer to algorithm result
 * @param[in]   pstGsAxisValueArr       pointer to g sensor data
 * @param[in]   usGsDataNum             number of g sensor data
 * @param[in]   emGsSensitivity         sensitivity of g sensor
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return successfully
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return func start error
 * @retval  #GH3X2X_RET_NO_INITED_ERROR         gh3x2x has not inited
 */




/**
 * @fn     GU32 GH3X2X_GetFuncModeAvailable(void)
 *
 * @brief  Get all function mode available
 *
 * @attention   None
 *
 * @param[out]  None
 * @param[in]   None
 * @return  function mode, @ref GH3X2X_NO_FUNCTION ... GH3X2X_FUNCTION_PWTT
 */
GU32 GH3X2X_GetFuncModeAvailable(void);

/**
 * @fn     GU32 GH3X2X_GetFuncModeOpened(void)
 *
 * @brief  Get function mode that already opened
 *
 * @attention   None
 *
 * @param[out]  None
 * @param[in]   None
 * @return  function mode, @ref GH3X2X_NO_FUNCTION ... GH3X2X_FUNCTION_PWTT
 */
GU32 GH3X2X_GetFuncModeOpened(void);

/**
 * @fn     GU32 GH3X2X_GetTargetFuncMode(void)
 *
 * @brief  Get function mode that will be opened or closed
 *
 * @attention   None
 *
 * @param[out]  None
 * @param[in]   None
 * @return  function mode, @ref GH3X2X_NO_FUNCTION ... GH3X2X_FUNCTION_PWTT
 */
GU32 GH3X2X_GetTargetFuncMode(void);

/**
 * @fn     GS8 GH3X2X_DecodeRegCfgArr(GU32* punRunMode, const STGh3x2xReg *pstRegConfigArr, GU16 usRegConfigLen)
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
GS8 GH3X2X_DecodeRegCfgArr(GU32* punRunMode, const STGh3x2xReg *pstRegConfigArr, GU16 usRegConfigLen);

/**
 * @fn     GU8 GH3X2X_GetGsensorEnableFlag(void)
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
GU8 GH3X2X_GetGsensorEnableFlag(void);
GU8 GH3X2X_GetCapEnableFlag(void);
GU8 GH3X2X_GetTempEnableFlag(void);
/**
 * @fn     STSoftLeadResult GH3X2X_LeadHandle(GU16 usIrqStatus, GU8 *puchReadFifoBuffer, GU16* pusFifoBufferLen)
 *
 * @brief  ecg soft lead handle
 *
 * @attention   None
 *
 * @param[in]   usIrqStatus               irq status
 * @param[in]   puchReadFifoBuffer         pointer to read fifo data buffer
 * @param[in]   usFifoBufferLen           read fifo buffer len
 * @param[out]  None
 *
 * @return  soft lead handle process, @ref STSoftLeadResult
 */
STSoftLeadResult GH3X2X_LeadHandle(GU16 usIrqStatus, GU8 *puchReadFifoBuffer, GU16* pusFifoBufferLen);

/**
 * @fn     STSoftLeadResult GH3X2X_LeadHandleEx(GU16 usIrqStatus, GU8 *puchReadFifoBuffer)
 *
 * @brief  ecg soft lead handle
 *
 * @attention   use after GH3X2X_GetRawdataForAlgorithm or GH3X2X_GetRawdataForAlgorithmEx
 *
 * @param[in]   usIrqStatus               irq status
 * @param[in]   puchReadFifoBuffer         pointer to read fifo data buffer
 * @param[out]  None
 *
 * @return  soft lead handle process, @ref STSoftLeadResult
 */
//STSoftLeadResult GH3X2X_LeadHandleEx(GU16 usIrqStatus, GU8 *puchReadFifoBuffer);

/**
 * @fn     GU8 GH3X2X_GetSoftEvent(void)
 * @brief  Get soft event
 * @param[in]   None
 * @param[out]  Soft event
 *
 * @return  soft event
 */
GU8 GH3X2X_GetSoftEvent(void);

/**
 * @fn     void GH3X2X_SetSoftEvent(GU8 uchEvent)
 * @brief  set soft event
 * @param[in]   uchEvent
 * @param[out]  None
 * @return  None
 */
void GH3X2X_SetSoftEvent(GU8 uchEvent);

/**
 * @fn     void GH3X2X_ClearSoftEvent(GU8 uchEvent)
 * @brief  clear soft event
 * @param[in]   uchEvent
 * @param[out]  None
 * @return  None
 */
void GH3X2X_ClearSoftEvent(GU8 uchEvent);


/**
 * @fn     void GH3X2X_AddElectrodeWearRevCnt(void)
 *
 * @brief  GH3X2X_AddElectrodeWearRevCnt
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_AddElectrodeWearRevCnt(void);

/**
 * @fn     GU8 GH3X2X_ElectrodeWearRevertDebugModeIsEnabled(void)
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
GU8 GH3X2X_ElectrodeWearRevertDebugModeIsEnabled(void);



/**
 * @fn     void GH3X2X_SoftWearOffDetInit(void)
 *
 * @brief  Soft wear off detect init
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_SoftWearOffDetInit(void);


/**
 * @fn     GU8 GH3X2X_GetSoftWearOffDetEn(void);
 *
 * @brief  get soft wear off dectect module  is disable or enable
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  0: disable  1: enable
 */
GU8 GH3X2X_GetSoftWearOffDetEn(void);



/**
 * @fn     GU8 GH3X2X_UpdateSoftWearOffStatus(GU16 *pusEvent)
 *
 * @brief  update wear status
 *
 * @attention   None
 *
 * @param[in]   old event point
 * @param[out]  0: no soft wear off det event  1: have soft wear off det event
 *
 * @return  new event
 */
GU8 GH3X2X_UpdateSoftWearOffStatus(GU16 *pusEvent, GU8 *puchSoftAdtFlag ,GU8 *puchLivingConfi);




/**
 * @fn     GS8 GH3X2X_SoftWearOffDetCalc(STHbAlgoResult pstAlgoResArr[], GU16 pusAlgoResIndexArr[], GU16* pusAlgoResCnt,
 *                          GU8 *puchReadDataBuffer, STGsensorRawdata *pstGsAxisValueArr, GU16 usGsDataNum,
 *                          EMGsensorSensitivity emGsSensitivity)
 *
 * @brief  Call Hb algorithm calc with read rawdata
 *
 * @attention   None
 *
 * @param[in]   puchReadDataBuffer      pointer read data buffer from GH3X2X_GetRawdataForAlgorithm
 * @param[in]   usFifoLen                    fifo byte length
 * @param[in]   pstGsAxisValueArr       pointer to gsensor data array
 * @param[in]   usGsDataNum             gsensor data count
 * @param[in]   emGsSensitivity         gsensor sensitivity, @ref EMGsensorSensitivity
 *
 * @return  error code
 * @retval  #GH3X2X_RET_ALGO_HAS_REFRESHED      return algorithm run successfully, and output has refreshed
 * @retval  #GH3X2X_RET_OK                      return algorithm run successfully, but output hasn't refreshed
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return algorithm param error
 * @retval  #GH3X2X_RET_RESOURCE_ERROR          return algorithm ram no enough error
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm generic error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */





/**
 * @fn     GS8 GH3X2X_SoftWearOffDetCalc(STHbAlgoResult pstAlgoResArr[], GU16 pusAlgoResIndexArr[], GU16* pusAlgoResCnt,
 *                          GU8 *puchReadDataBuffer, STGsensorRawdata *pstGsAxisValueArr, GU16 usGsDataNum,
 *                          EMGsensorSensitivity emGsSensitivity)
 *
 * @brief  Call Hb algorithm calc with read rawdata
 *
 * @attention   None
 *
 * @param[in]   puchReadDataBuffer      pointer read data buffer from GH3X2X_GetRawdataForAlgorithm
 * @param[in]   pstGsAxisValueArr       pointer to gsensor data array
 * @param[in]   usGsDataNum             gsensor data count
 * @param[in]   emGsSensitivity         gsensor sensitivity, @ref EMGsensorSensitivity
 *
 * @return  error code
 * @retval  #GH3X2X_RET_ALGO_HAS_REFRESHED      return algorithm run successfully, and output has refreshed
 * @retval  #GH3X2X_RET_OK                      return algorithm run successfully, but output hasn't refreshed
 * @retval  #GH3X2X_RET_PARAMETER_ERROR         return algorithm param error
 * @retval  #GH3X2X_RET_RESOURCE_ERROR          return algorithm ram no enough error
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm generic error
 * @retval  #GH3X2X_RET_ALGO_NO_SUPPORTED       return algorithm no supported error
 */






/**
 * @fn     GU16 GH3X2X_GetCurrentFifoWaterLine(void)
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
GU16 GH3X2X_GetCurrentFifoWaterLine(void);



/**
 * @fn     GS8 GH3X2X_CalibrateECGGainEx(GU8 *puchReadFifoBuffer)
 *
 * @brief  Calibrate ECG Gain
 *
 * @attention   None
 *
 * @param[in]   puchReadFifoBuffer      pointer read data buffer from GH3X2X_GetRawdataForAlgorithm
 *
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return algorithm run successfully, but output hasn't refreshed
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm generic error
 */

GS8 GH3X2X_CalibrateECGGainEx(GU8 *puchReadFifoBuffer);


/**
 * @fn     GS8 GH3X2X_CalibrateECGGain(GU8 *puchReadFifoBuffer)
 *
 * @brief  Calibrate ECG Gain
 *
 * @attention   None
 *
 * @param[in]       puchReadFifoBuffer      pointer read data buffer
 * @param[in]       usFifoBufferLen         fifo byte length
 * @return  error code
 * @retval  #GH3X2X_RET_OK                      return algorithm run successfully, but output hasn't refreshed
 * @retval  #GH3X2X_RET_GENERIC_ERROR           return algorithm generic error
 */

GS8 GH3X2X_CalibrateECGGain(GU8 *puchReadFifoBuffer, GU16 usFifoBufferLen);



/**
 * @fn       GU8 GH3x2x_SetChipResetRecoveringFlag(void)


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
void GH3x2x_SetChipResetRecoveringFlag(GU8 uchChipResetRecoeringFlag);


/**
 * @fn       GU8 GH3x2x_GetChipResetRecoveringFlag(void)


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
GU8 GH3x2x_GetChipResetRecoveringFlag(void);




/**
 * @fn       GU8 GH3x2x_GetActiveChipResetFlag(void)


 *
 * @brief
 *
 * @attention   None.
 *
 * @param[in]   None
 * @param[out]  ActiveChipResetFlag  1: user have done chip reset(soft reset and hardwear reset) actively   gh3x2x_init will clear it
 *
 * @return  None
 */
GU8 GH3x2x_GetActiveChipResetFlag(void);


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
void GH3X2X_RecordTiaGainInfo(void);



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
GU8 GH3x2x_GetNeedWakeUpGh3x2xFlag(void);


/**
 * @fn     GU8 GH3X2X_UpdateSoftWearStatus(void)
 *
 * @brief  update wear status
 *
 * @attention   None
 *
 * @param[in]   old event point
 * @param[out]  0: no soft wear off det event  1: have soft wear off det event
 *
 * @return  new event
 */
GU8 GH3X2X_UpdateSoftWearStatus(void);
void GH3X2X_SetSoftWearStatus(EMWearRecordType emStatus);

/**
 * @fn        void GH3x2x_SetNeedWakeUpGh3x2xFlag(GU8 uchFlag)

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
void GH3x2x_SetNeedWakeUpGh3x2xFlag(GU8 uchFlag);

/**
 * @fn        void GH3x2x_SetNeedWakeUpGh3x2xFlagBeforeInt(GU8 uchFlag)

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
void GH3x2x_SetNeedWakeUpGh3x2xFlagBeforeInt(GU8 uchFlag);

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
GU8 GH3x2x_GetNeedWakeUpGh3x2xFlagBeforeInt(void);

/**
 * @fn     void GH3X2X_SlotEnRegSet(GU8 uchSetValue)
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
void GH3X2X_SlotEnRegSet(GU8 uchSetValue);

/**
 * @fn     void GH3X2X_ModifyFunctionFrequency(GU8 uchFunctionID, GU16 usFrequencyValue)
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
void GH3X2X_ModifyFunctionFrequency(GU8 uchFunctionID, GU16 usFrequencyValue);

/**
 * @fn     void GH3X2X_ModifyFunctionLedCurrent(GU8 uchFunctionID, GU16 usLedDrv0Current, GU16 usLedDrv1Current)
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
void GH3X2X_ModifyFunctionLedCurrent(GU8 uchFunctionID, GU16 usLedDrv0Current, GU16 usLedDrv1Current);

typedef struct
{
    GU16 usSampleRate;                  //current sample rate
    GU16 usSampleRateForUserSetting;    //0:  use default sample rate via cfg array    others: use usSampleRateForUserSetting
    GU32 unChnlEnForUserSetting;        //BitN = 1: channel N is enable     BitN = 0: channel N is disable
    GU8  uchChnlNum;
    GU8 uchSlotBit;
} STGh3x2xFunctionInfo;

typedef struct
{
    GU8 uchUpdateFlag;
    GU8 uchResultNum;
    GS32 snResult[16];
} STGh3x2xAlgoResult;



typedef struct
{
    GU8 uchDownSampleFactor;   //0: down sample disable    N(N != 0):  down sample radio = (N + 1) : 1
    GU8 uchDownSampleCnt;      // data will output when uchDownSampleFactor = uchDownSampleCnt
} STGh3x2xDownSampleInfo;



typedef struct
{
    GU32 unFunctionID ;
    STGh3x2xFunctionInfo *pstFunctionInfo;
    STGh3x2xDownSampleInfo *pstDownSampleInfo;
    GU8   uchFuntionChnlLimit;   //max chnl num
    GU8   *pchChnlMap;
    GU32 *punFrameRawdata;
    GS16 *pusFrameGsensordata;
    STCapRawdata *pstFrameCapdata;
    STTempRawdata *pstFrameTempdata;
    GU32 *punFrameAgcInfo;
    GU32 *punFrameFlag;
    GU32 *punIncompleteRawdata;
    GU32 *punIncompleteChnlMapBit;
    GU8 * puchFrameLastGain;
    GU32 * punFrameCnt;
    STGh3x2xAlgoResult * pstAlgoResult;
    STGh3x2xAlgoResult * pstAlgoRecordResult;
} STGh3x2xFrameInfo;



/******************************************/
/** GH3x2x BP Function Parameter & API **/

#define     GH3X2X_PWA_FEATURE_NUMS                        (18)
#define     GH3X2X_PWA_FITTING_COEF_KB_NUMS                (2)
#define     GH3X2X_PWA_CALI_NUMS                           (5)

typedef enum
{
    BP_PRESSURE_MODE   = 0,
    BP_FPBP_CAL_MODE   = 1,
}EMBpModeType;

typedef enum
{
    BP_OUTPUT_RESULT      = 0x01,
    BP_OUTPUT_CALI_VALUE  = 0x02,
    BP_OUTPUT_PREESURE    = 0x04,
    BP_OUTPUT_OFFET       = 0x08,
}EMBpOutputFlagOffset;

typedef struct
{
    GU32 unResulttype;
    GU32 unSbp;
    GU32 unDbp;
    GU32 unHr;
    GS32 nIsHyperFlag;
    GU32 unConfscore;
    GU32 unPi;
    GU32 unCtr;
    GU32 unSnr;
}STGh3x2xBpResult;

typedef struct
{
    GU8 uchUpdateFLag;
    GU32 unCaliSbp;
    GU32 unCaliDbp;
    GU32 unCaliHr;
    double CaliFeatureValue[GH3X2X_PWA_CALI_NUMS][GH3X2X_PWA_FEATURE_NUMS];
    double CaliFittingCoef[GH3X2X_PWA_CALI_NUMS][GH3X2X_PWA_FITTING_COEF_KB_NUMS];
}STGh3x2xBpCaliPara;

typedef struct
{
    GU32 unErrorCode;
    GU8 uchResultUpdateFlag; //refer to EMBpOutputFlagOffset
    GS32 nOffset;
    GS32 nPressure;
    STGh3x2xBpResult stBpResult;
    STGh3x2xBpCaliPara stPwaCali;
}STGh3x2xBpAlgoOutput;

typedef struct
{
    GS32 nOffset;
    GS32 nFactor;
    GS32 nArea;
}STGh3x2xBpPressureSensorConfig;

typedef struct
{
    GS32 nTmperature;
    GU32 unAge;
    GU32 unHeight;
    GU32 unWeight;
    GU32 unGender;//0:male, 1:female
    STGh3x2xBpPressureSensorConfig stPressureSensorConfig;
    STGh3x2xBpCaliPara stCaliInfo;
}STGh3x2xBpAlgoInput;

extern const GU8 g_uchGyroEnable;

//extern void Gh3x2x_BpAlgorithmPersonalInfoInput(EMBpModeType EMBpMode, STGh3x2xBpAlgoInput *pstAlgoInput);
extern void Gh3x2x_BpAlgorithmResultReport(STGh3x2xBpAlgoOutput *pstAlgoResult);


/******************************************/


//extern void GH3X2X_SetFifoPackageMode(GU8 uchMode, GU8 *puchFifoBuffer);
extern void GH3X2X_SendRawdataFifoPackage(GU8 *puchGh3x2xReadFifoData, GU16 usFifoReadByteNum);
//extern void GH3X2X_PackRawdataFifoPackage(GU8 *puchGh3x2xReadFifoData, GU16 usFifoReadByteNum);
extern GU32 *g_punGh3x2xAlgoMemBufferPointer;
extern GU8 GH3X2X_GetFifoPackageMode(void);


/**
 * @fn      void GH3x2x_ChangeSampleParmForEngineeringMode(GU32 unFunctionMode, STGh3x2xEngineeringModeSampleParam *pstSampleParaGroup , GU8 uchSampleParaGroupNum)

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
void GH3x2x_ChangeSampleParmForEngineeringMode(const STGh3x2xFrameInfo * const  pstGh3x2xFrameInfo[], GU32 unFunctionMode, STGh3x2xEngineeringModeSampleParam *pstSampleParaGroup , GU8 uchSampleParaGroupNum);


GS8 GH3X2X_FunctionStart(const STGh3x2xFrameInfo * const pstFrameInfo);
GS8 GH3X2X_FunctionStop(const STGh3x2xFrameInfo * const pstFrameInfo);






void GH3x2xFunctionProcess(GU8 *puchRawdataBuf, GU16 usRawDataByteLen, GS16 *pusGsValueArr, GU16 usGsDataNum,
                        STCapRawdata* pstCapValueArr,GU16 usCapDataNum,STTempRawdata* pstTempValueArr,GU16 usTempDataNum,
                        const STGh3x2xFrameInfo * const pstFrameInfo);
GU8* GH3x2xGetFunctionChnlMap(const STGh3x2xFrameInfo * const pstFrameInfo);
void GH3x2xSetFunctionChnlNum(const STGh3x2xFrameInfo * const pstFrameInfo, GU8 uchChnlNum);
void GH3x2xSetFunctionChnlMap(const STGh3x2xFrameInfo * const pstFrameInfo, GU8 uchChnlId, GU8 uchChnlTag);
void GH3x2xCalFunctionSlotBit(const STGh3x2xFrameInfo * const pstFrameInfo);









extern const STGh3x2xFrameInfo * const g_pstGh3x2xFrameInfo[];


/**
 * @fn     void GH3X2X_EcgSampleHookHandle(GU8 uchEventType, GU8 uchEventInfo)
 *
 * @brief  ecg sample  hook handle
 *
 * @attention   None
 *
 * @param[in]   uchEventType         ecg sample event type, @ref ECG_SAMPLE_EVENT_TYPE_SAMPLE etc
 * @param[in]   uchEventInfo         ecg sample event info, @ref ECG_SAMPLE_EVENT_INFO_SAMPLE_STOP etc
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_EcgSampleHookHandle(GU8 uchEventType, GU8 uchEventInfo);


void GH3X2X_LeadDetEnInHardAdt(GU8 uchEn);

/**
 * @fn     GH3X2X_LeadDetEnControl(GU8 uchEventInfo)
 *
 * @brief  lead detect enable control
 *
 * @attention   None
 *
 * @param[in]   uchEventInfo         control event, only ecg start/stop evt
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_LeadDetEnControl(GU8 uchEventInfo);
/**
 * @fn     void GH3X2X_SlaverSoftLeadPramSet(GU16 usRegVal, GU8 uchRegPosi)
 *
 * @brief  slave soft lead off param set from virtual reg
 *
 * @attention   None
 *
 * @param[in]   usRegVal          reg value
 * @param[in]   uchRegPosi        reg position
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_SlaverSoftLeadPramSet(GU16 usRegVal, GU8 uchRegPosi);

/**
 * @fn     void GH3X2X_SlaverSoftLeadPramInit(void)
 *
 * @brief  slave soft lead off param init to default value
 *
 * @attention   None
 *
 * @param[in]   None
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_SlaverSoftLeadPramInit(void);






void GH3X2X_StartHardAdtAndResetGsDetect(void);
void GH3X2X_StopHardAdtAndStartGsDetect(void);
GU8 GH3X2X_GetSoftLeadDetMode(void);
GU8 GH3X2X_GetAdtElectrodeAdtEn(void);


extern void gh3x2x_algorithm_get_io_data_hook_func(const STGh3x2xFrameInfo * const pstFrameInfo);
extern void gh3x2x_active_reset_hook(void);



/**
 * @fn     void Gh3x2x_DemoSendProtocolData(GU8* puchProtocolDataBuffer, GU16 usProtocolDataLen)
 *
 * @brief  Copy protocol data to buffer that will be sent out to APP/EVK.
 *
 * @attention   None
 *
 * @param[in]   puchProtocolDataBuffer      point to protocol data
 * @param[in]   usProtocolDataLen           protocol data length
 * @param[out]  None
 *
 * @return  None
 */
extern void Gh3x2xDemoSendProtocolData(GU8* puchProtocolDataBuffer, GU16 usProtocolDataLen);
extern GU8 GH3X2X_UprotocolPacketFormat(GU8 uchCmd, GU8 *puchPacketBuffer, GU8 *puchPayloadData, GU8 uchPayloadDataLen);

extern void Gh2x2xUploadDataToMaster(const STGh3x2xFrameInfo * const pstFrameInfo, GU16 usFrameCnt, GU16 usFrameNum, GU8* puchTagArray);
extern void Gh2x2xUploadZipDataToMaster(const STGh3x2xFrameInfo * const pstFrameInfo, GU16 usFrameCnt, GU16 usFrameNum, GU8* puchTagArray);

extern GU16 GH3x2xGetFrameNum(GU8 *puchRawdataBuf, GU16 usRawDataByteLen, const STGh3x2xFrameInfo * const pstFrameInfo);
GU8  GH3X2X_CalcArrayCrc8Val(GU8 uchDataArr[], GU16 usDataIndex, GU16 usDataLen);

void Gh3x2xSetProtocolEventAck(void);
GU8 Gh3x2xGetProtocolEventReportId(void);

extern void Gh3x2xSetFrameFlag2(const STGh3x2xFrameInfo * const pstFrameInfo);

extern GU8 Gh3x2xGetHrAlgoSupportChnl(void);
extern GU8 Gh3x2xGetSpo2AlgoSupportChnl(void);

extern void GH3X2X_ReadIntTime(void);
extern GU8 GH3X2X_GetIntTimeInArray(GU8 uchSlotIndex);

extern GU8 Gh3x2xCheckSoftAdtTimeOut(const STGh3x2xFrameInfo * const pstFrameInfo, GU32 unSpecialAngleTimeSec, GU32 unMovelessTimeSec);

extern void Gh3x2xFunctionInfoForUserInit(void);

extern void GH3x2xCalFunctionSlotBit(const STGh3x2xFrameInfo * const pstFrameInfo);

extern void GH3x2xSetAgcReg(GU8 uchAgcReg,  GU8 uchSlot, GU16 usValue);

extern GU16 GH3x2xGetAgcReg(GU8 uchAgcReg,  GU8 uchSlot);

extern void GH3x2xSendAllVersion(void);


/**
 * @fn     void GH3x2xSleepFlagSet(GU8 uchSleepFlag)
 *
 * @brief  sleep flag set
 *
 * @attention   None
 *
 * @param[in]   0: non-sleep mode   1:  sleep mode
 * @param[out]  None
 *
 * @return  None
 */
extern void GH3x2xSleepFlagSet(GU8 uchSleepFlag);
extern GU8 GH3x2xSleepFlagGet(void);

extern void GH3X2X_SetAlgoEnableFlag(GU8 uchAlgoEnableFlag);

extern void* Gh3x2xAlgoMemApply(GU32 unMemSize);
extern void Gh3x2xAlgoMemFree(void* MemAddr);

extern void GH3X2X_UprotocolFpbpDataReceiveCmd(GU8 *puchRespondBuffer, GU16 *pusRespondLen);
extern void GH3x2xBpAlgoInitSetting(GU8 *puchRespondBuffer);

extern void Gh3x2xOutputValueStrategyInit(GU32 unFunctionID);




extern GS32 GH3X2X_LeadOffDetect2Init(void);
extern GU8 GH3X2X_LeadOffDetect2(GU8* puchFifoBuffer, GU16* pusFifoBufferLen);

extern GU8 GH3X2X_LeadOffDetect(GU8* puchFifoBuffer, GU16 usFifoBufferLen);
extern void GH3X2X_LeadOffDetectInit(void);
extern GU8 GH3X2X_EcgRsHandle(GU8 *puchFifoBuffer, GU16 *pusFifoBufferLen);
extern void Gh3x2xSetGh3x2xSoftWearOffDetFunctionId(GU32 unFunctionId);
extern void Gh3x2xPollingModePro(void);
extern void GH3X2X_ECGResampleConfig(GU16 usEcgSampleRate);
extern void Gh3x2xNewAgcSubChnlModuleReset(void);
extern GU8 GH3x2xNewAgcSetNewSubChnl(GU8 uchSlotNo, GU8 uchRxEn);
extern void GH3X2X_NewAgcSubChnlGainAdj(GU8* puchReadFifoBuffer, GU16 usFifoLen);
extern void GH3x2xNewAgcSubChnlClearAnalysisCntAndSetGain(GU8 uchSlotNo, GU8 uchRxEn ,GU8 uchNextGain);
extern void GH3x2xNewAgcSetSubChnlSlotInfo(GU8 uchSlotCnt, GU16 usSampleRate);
extern void GH3x2xWriteCapToFlash(void);
extern void GH3x2xReadCapFromFlash(void);
extern void GH3X2X_MoveDetectByCapData(STCapRawdata* pusCapData, GU16 usCapDataCnt);
extern void GH3x2xSlotTimeInfo(void);

#endif /* _GH3X2X_DRV_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/
