/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_uprotocol.h
 *
 * @brief   gh3x2x universal protocol define & declaration
 *
 * @version ref gh3x2x_drv_version.h
 *
 */

#ifndef _GH3X2X_DRV_UPROTOCOL_H_
#define _GH3X2X_DRV_UPROTOCOL_H_


#include "gh3x2x_drv_config.h"
#include "gh3x2x_drv.h"
#include "gh3020_bridge.h"


/* function ptr typedef */
typedef char* (*pfnGetFirmwareVersion)(void);                           /**< read pin status type */


#define GH3X2X_FRAME_POSI_HEAD  (0x01)
#define GH3X2X_FRAME_POSI_TAIL  (0x02)
#define GH3X2X_UPLOAD_STATUS_NULL  0
#define GH3X2X_UPLOAD_STATUS_HAVE_STORE_DATA 1
#define GH3X2X_UPLOAD_PACKAGE_HEAD_SIZE 8

#define GH3X2X_SEND_STATUS_TAIL_MODE 1
#define GH3X2X_SEND_STATUS_FULL_MODE 2
#define GH3X2X_SEND_STATUS_SPLIC_PACK_MODE 3

/**
 * @brief ZipMode Lastdata Temp
 */
typedef struct
{
    uint8_t uchFlag;
    int16_t sGsensorLastDataArr[GH3X2X_GSENSOR_MAX_SIZE];
    uint8_t uchFifoLastRawdataTagArr[GH3X2X_CHANNEL_MAP_MAX_CH];
    uint32_t unFifoLastRawdataArr[GH3X2X_CHANNEL_MAP_MAX_CH];
    uint32_t unFifoLastAgcdataArr[GH3X2X_CHANNEL_MAP_MAX_CH];
    uint32_t unFifoLastAmbdataArr[GH3X2X_CHANNEL_MAP_MAX_CH];
    uint32_t unFifoLastResult;
} STZipLastDataTemp;

/* universal protocol packet, but index no use */
#define  GH3X2X_UPROTOCOL_PACKET_FIXED_INDEX                (0u)        /**< fixed header index */
#define  GH3X2X_UPROTOCOL_PACKET_VER_INDEX                  (1u)        /**< version index */
#define  GH3X2X_UPROTOCOL_PACKET_CMD_INDEX                  (2u)        /**< cmd index */
#define  GH3X2X_UPROTOCOL_PACKET_LEN_INDEX                  (3u)        /**< payload len index */
#define  GH3X2X_UPROTOCOL_PACKET_PAYLOAD_INDEX              (4u)        /**< payload index */
#define  GH3X2X_UPROTOCOL_PACKET_CRC8_INDEX                 (4u)        /**< crc8 index not equl 4, index = len+4 */
//#define  GH3X2X_UPROTOCOL_PACKET_HEADER_LEN                 (4u)        /**< packet header len */
//#define  GH3X2X_UPROTOCOL_PACKET_TAIL_LEN                   (1u)        /**< packet tail len */



/// get packet payload len support
#define  GH3X2X_UPROTOCOL_GET_PAYLOAD_LEN_SUPPORT(val)      ((val) - GH3X2X_UPROTOCOL_PACKET_HEADER_TAIL_LEN)


/* cmd id */
#define  GH3X2X_UPROTOCOL_CMD_NOP                       (0x00)     /**< nop operation */
#define  GH3X2X_UPROTOCOL_CMD_CTRL_ACK                  (0x01)     /**< ctrl ack */
#define  GH3X2X_UPROTOCOL_CMD_GET_STATUS                (0x02)     /**< get status */
#define  GH3X2X_UPROTOCOL_CMD_REG_RW                    (0x03)     /**< register RW */
#define  GH3X2X_UPROTOCOL_CMD_LOAD_CONFIG               (0x04)     /**< load config array */
#define  GH3X2X_UPROTOCOL_CMD_PACKAGE_TEST              (0x05)     /**< package test */
#define  GH3X2X_UPROTOCOL_CMD_READ_OTP_REG              (0x07)     /**< Read Otp Reg */
#define  GH3X2X_UPROTOCOL_CMD_RAWDATA                   (0x08)     /**< raw data package, only output */

#define  GH3X2X_UPROTOCOL_CMD_ZIPEVENRAWDATA            (0x09)     /**< raw data package, only output */
#define  GH3X2X_UPROTOCOL_CMD_ZIPODDRAWDATA             (0x0A)     /**< raw data package, only output */
#define  GH3X2X_UPROTOCOL_CMD_NEW_STRUCT_RAWDATA        (0x0B)     /**< raw data package, only output */

#define  GH3X2X_UPROTOCOL_CMD_START_CTRL                (0x0C)     /**< start ctrl */
#define  GH3X2X_UPROTOCOL_CMD_WORK_MODE                 (0x10)     /**< work mode */
#define  GH3X2X_UPROTOCOL_CMD_FIFO_THR_CONFIG           (0x12)     /**< fifo thr config */
#define  GH3X2X_UPROTOCOL_CMD_REPORT_EVENT_SET          (0x13)     /**< set GH3X2X event need reporting */
#define  GH3X2X_UPROTOCOL_CMD_ECG_CLASSIFICATION        (0x14)     /**< ecg classification */
#define  GH3X2X_UPROTOCOL_CMD_FUNC_MAP                  (0x15)     /**< function map */
#define  GH3X2X_UPROTOCOL_CMD_CHIP_EVENT_REPORT         (0x16)     /**< GH3X2X event report */
#define  GH3X2X_UPROTOCOL_CMD_CHIP_CTRL                 (0x17)     /**< chip ctrl */
#define  GH3X2X_UPROTOCOL_CMD_GET_VER                   (0x19)     /**< get ver */
#define  GH3X2X_UPROTOCOL_CMD_GET_CHIPSET_CONN_STA      (0x1A)     /**< get GH3X2X connect status */
#define  GH3X2X_UPROTOCOL_CMD_SWITCH_SLOT_FUNC          (0x1C)     /**< switch on or off some function */
#define  GH3X2X_UPROTOCOL_CMD_GET_OPENED_FUNC           (0x25)     /**< get Opened function */
#define  GH3X2X_UPROTOCOL_CMD_HSM_RESULT_UPDATE         (0x29)     /**< switch on or off some function */
#define  GH3X2X_UPROTOCOL_CMD_RAWDATA_FIFO_UPDATE       (0x2A)     /**< rawdata fifo update function */
#define  GH3X2X_UPROTOCOL_CMD_BP_DATA_TRANS             (0x2B)     /**< fpbp data transmit function */
#define  GH3X2X_UPROTOCOL_CMD_FUNCTION_INFO_UPDATE      (0x2C)     /**< function info transmit function */
#define  GH3X2X_UPROTOCOL_CMD_GET_MAX_LEN               (0xA0)     /**< get packet max len support */
#define  GH3X2X_UPROTOCOL_CMD_LOAD_REG_LIST             (0xA1)     /**< get load reg list */
#define  GH3X2X_UPROTOCOL_CMD_DEBUG_STATUS              (0xA2)     /**< debug status */

/* macro of reg rw cmd */
#define  UPROTOCOL_REG_RW_READ_MODE                     (0x00)      /**< read mode */
#define  UPROTOCOL_REG_RW_WRITE_MODE                    (0x01)      /**< write mode */
#define  UPROTOCOL_REG_RW_MODE_INDEX                    (0u)        /**< mode */
#define  UPROTOCOL_REG_RW_CNT_INDEX                     (1u)        /**< count */
#define  UPROTOCOL_REG_RW_ADDR_H_INDEX                  (2u)        /**< addr_h */
#define  UPROTOCOL_REG_RW_ADDR_L_INDEX                  (3u)        /**< addr_l */
#define  UPROTOCOL_REG_RW_DATA_INDEX                    (4u)        /**< data */
#define  UPROTOCOL_REG_RW_HEADER_LENGTH                 (4u)        /**< header len */


/* macro of get version cmd */
#define  UPROTOCOL_GET_VER_TYPE_FW_VER                  (0x01)      /**< get firmware version type */
#define  UPROTOCOL_GET_VER_TYPE_VIRTUAL_REG_VER         (0x0B)      /**< get virtual reg version type */
#define  UPROTOCOL_GET_VER_TYPE_BOOTLOADER_VER          (0x0C)      /**< get BOOTLOADER version type */
#define  UPROTOCOL_GET_VER_TYPE_PROTOCOL_VER            (0x0E)      /**< get protocol version type */
#define  UPROTOCOL_GET_VER_TYPE_FUNC_SUPPORT            (0x0F)      /**< get driver func support type */
#define  UPROTOCOL_GET_VER_TYPE_DRV_VER                 (0x10)      /**< get driver version type */
#define  UPROTOCOL_GET_VER_TYPE_CHIP_VER                (0x11)      /**< get chip version type */
#define  UPROTOCOL_GET_VER_TYPE_ALGO_VER                (0x12)      /**< get algorithm version type */
#define  UPROTOCOL_GET_VER_TYPE_ALGO_VER_END            (0x12 + GH3X2X_FUNC_OFFSET_MAX)      /**< get algorithm version type END*/

#define  UPROTOCOL_GET_VER_TYPE_INDEX                   (0u)        /**< type index */
#define  UPROTOCOL_GET_VER_STRING_LEN_INDEX             (1u)        /**< version string len index */
#define  UPROTOCOL_GET_VER_STRING_INDEX                 (2u)        /**< version string index */
#define  UPROTOCOL_GET_VER_DEFAULT_VER_STRING           "no_ver"    /**< default version string */
#define  UPROTOCOL_GET_VER_CACHE_VER_MAX_LEN            (64)        /**< cache algo ver string, max len */
#define  UPROTOCOL_GET_VER_CACHE_VER_PAYLOAD_INDEX      (GH3X2X_UPROTOCOL_PAYLOAD_LEN_MAX - \
                                                         UPROTOCOL_GET_VER_CACHE_VER_MAX_LEN) /**< cache index */


/* macro of get max len */
#define  UPROTOCOL_GET_MAX_LEN_RESP_VAL_INDEX            (0u)      /**< len index */
#define  UPROTOCOL_GET_MAX_LEN_RESP_LEN                  (1u)      /**< respond len */


/* macro of load reg list */
#define  UPROTOCOL_LOAD_REG_LIST_SIZEOF                 (4u)      /**< sizeof reg */
#define  UPROTOCOL_LOAD_REG_LIST_RESP_STATUS_INDEX      (0u)      /**< respond status inex */
#define  UPROTOCOL_LOAD_REG_LIST_RESP_LEN               (1u)      /**< respond len */
#define  UPROTOCOL_LOAD_REG_LIST_RESP_CFG_SUCCESS       (0u)      /**< config success */
#define  UPROTOCOL_LOAD_REG_LIST_RESP_CFG_FAIL          (1u)      /**< config fail */


/* macro of fifo thr config */
#define  UPROTOCOL_FIFO_THR_CFG_H_INDEX                 (0u)      /**< config val high byte inex */
#define  UPROTOCOL_FIFO_THR_CFG_L_INDEX                 (1u)      /**< config val low byte inex */
#define  UPROTOCOL_FIFO_THR_CFG_RESP_STATUS_INDEX       (0u)      /**< respond status inex */
#define  UPROTOCOL_FIFO_THR_CFG_RESP_LEN                (1u)      /**< respond len */
#define  UPROTOCOL_FIFO_THR_CFG_RESP_CFG_SUCCESS        (0u)      /**< config success */
#define  UPROTOCOL_FIFO_THR_CFG_RESP_CFG_FAIL           (1u)      /**< config fail */

/* macro of report event set */
#define  UPROTOCOL_REPORT_EVENT_SET_INDEX                (0u)      /**< report event set inex */
#define  UPROTOCOL_REPORT_EVENT_SET_RESP_LEN             (1u)      /**< respond len */
#define  UPROTOCOL_REPORT_EVENT_SET_RESP_INDEX           (0u)      /**< report event set respond inex */
#define  UPROTOCOL_REPORT_EVENT_SET_SUCCESS              (0u)      /**< set success */
#define  UPROTOCOL_REPORT_EVENT_SET_FAIL                 (1u)      /**< set fail */
#define  UPROTOCOL_REPORT_EVENT_ID_INIT                  (0xFF)    /**< set fail */

/* macro of chip ctrl */
#define  UPROTOCOL_CHIP_CTRL_VAL_INDEX                  (0u)      /**< ctrl val inex */
#define  UPROTOCOL_CHIP_CTRL_RESP_STATUS_INDEX          (0u)      /**< respond status inex */
#define  UPROTOCOL_CHIP_CTRL_RESP_LEN                   (1u)      /**< respond len */
#define  UPROTOCOL_CHIP_CTRL_RESP_CFG_SUCCESS           (0u)      /**< config success */
#define  UPROTOCOL_CHIP_CTRL_RESP_CFG_FAIL              (1u)      /**< config fail */
#define  UPROTOCOL_CHIP_CTRL_RESP_NULL                  (0xFF)    /**< function ptr null */
#define  UPROTOCOL_CHIP_CTRL_TYPE_HARD_RESET            (0x5A)    /**< hard reset type */
#define  UPROTOCOL_CHIP_CTRL_TYPE_SOFT_RESET            (0xC2)    /**< soft reset type */
#define  UPROTOCOL_CHIP_CTRL_TYPE_WAKEUP                (0xC3)    /**< wakeup type */
#define  UPROTOCOL_CHIP_CTRL_TYPE_SLEEP                 (0xC4)    /**< sleep type */

/* macro of chip event report */
#define  UPROTOCOL_CHIP_EVENT_REPORT_ACK_INDEX           (0u)      /**< event report ack inex */
#define  UPROTOCOL_CHIP_EVENT_REPORT_INDEX               (0u)      /**< event report inex */
#define  UPROTOCOL_CHIP_EVENT_REPORT_ID_INDEX            (2u)      /**< event report id inex */
#define  UPROTOCOL_CHIP_EVENT_REPORT_LEN                 (3u)      /**< event report data length */

/* macro of debug status */
#define  UPROTOCOL_DEBUG_STATUS_TYPE_INDEX              (0u)      /**< type inex */
#define  UPROTOCOL_DEBUG_STATUS_RESP_BYTES_INDEX        (1u)      /**< respond bytes len inex */
#define  UPROTOCOL_DEBUG_STATUS_RESP_DATA_INDEX         (2u)      /**< respond data inex */
#define  UPROTOCOL_DEBUG_STATUS_RESP_LEN                (6u)      /**< respond len */
#define  UPROTOCOL_DEBUG_STATUS_RESP_BYTES_8            (1u)      /**< respond bytes len 8 bits */
#define  UPROTOCOL_DEBUG_STATUS_RESP_BYTES_16           (2u)      /**< respond bytes len 16 bits */
#define  UPROTOCOL_DEBUG_STATUS_RESP_BYTES_32           (4u)      /**< respond bytes len 32 bits */
#define  UPROTOCOL_DEBUG_STATUS_READ_INT_PIN            (0x00)    /**< read int pin type */
#define  UPROTOCOL_DEBUG_STATUS_READ_RST_PIN            (0x01)    /**< read reset pin type */
#define  UPROTOCOL_DEBUG_STATUS_READ_SPCS_PIN           (0x02)    /**< read spcs pin type */
#define  UPROTOCOL_DEBUG_STATUS_READ_SPDO_PIN           (0x03)    /**< read spdo pin type */
#define  UPROTOCOL_DEBUG_STATUS_COMM_CONFIRM            (0x04)    /**< communicate confirm type */
#define  UPROTOCOL_DEBUG_STATUS_READ_WORK_STATUS        (0x10)    /**< read work status type */

/* macro of rawdata package*/
#define  UPROTOCOL_FUNCTION_ID_INDEX                    (0u)      /**< function id inex */
#define  UPROTOCOL_RAWDATA_TYPE_INDEX                   (1u)      /**< rawdata type inex */
#define  UPROTOCOL_RAWDATA_CHNL_NUM                     (2u)      /**< rawdata channel num */
#define  UPROTOCOL_PACKAGE_TYPE_INDEX                   (6u)      /**< package type index */
#define  UPROTOCOL_RAWDATA_LEN_INDEX                    (7u)      /**< rawdata len index */
#define  UPROTOCOL_FRAME_DATA_INDEX                     (2u)      /**< frame data inex */
#define  UPROTOCOL_GS_ENABLE_FIELD                      (0u)      /**< g sensor enable field in data type */
#define  UPROTOCOL_ALGO_ENABLE_FIELD                    (1u)      /**< algorithm result enable field in data type */
#define  UPROTOCOL_AGC_ENABLE_FIELD                     (2u)      /**< gain current enable field in data type */
#define  UPROTOCOL_AMBIANCE_ENABLE_FIELD                (3u)      /**< ambiance enable field in data type */
#define  UPROTOCOL_GS_GYRO_ENABLE_FIELD                 (4u)      /**< GYRO enable field in data type */
#define  UPROTOCOL_CAP_ENABLE_ENABLE_FIELD              (5u)      /**< CAP enable field in data type */
#define  UPROTOCOL_TEMP_ENABLE_ENABLE_FIELD             (6u)      /**< TEMP enable field in data type */
#define  UPROTOCOL_ZIP_ENABLE_FIELD                     (0u)      /**< zip enable field in data type */
#define  UPROTOCOL_ODDEVEN_FLAG_FIELD                   (1u)      /**< odd-even flag field in data type */
#define  UPROTOCOL_FUNCTION_MODE_FIELD                  (2u)      /**< function mode field in data type */
#define  UPROTOCOL_SPLIC_PACK_CNT_FIELD                 (3u)      /**< splic pack cnt in data type */
#define  UPROTOCOL_SPLIC_PACK_OVER_FIELD                (5u)      /**< splic pack over in data type */
#define  UPROTOCOL_NEW_DATA_FIELD                       (7u)      /**< algorithm result enable field in data type */
#define  UPROTOCOL_RESULT_TYPE_BIT_FIELD                (7u)      /**< result type bit field in protocol data */
#define  UPROTOCOL_UPLOAD_FLAG_INDEX0                   (0u)      /**< flag index0 that will upload */
#define  UPROTOCOL_UPLOAD_FLAG_INDEX1                   (1u)      /**< flag index1 that will upload */
#define  UPROTOCOL_UPLOAD_FLAG_INDEX2                   (2u)      /**< flag index2 that will upload */
#define  UPROTOCOL_UPLOAD_FLAG_INDEX3                   (3u)      /**< flag index3 that will upload */
#define  UPROTOCOL_UPLOAD_FLAG_INDEX4                   (4u)      /**< flag index4 that will upload */
#define  UPROTOCOL_UPLOAD_FLAG_INDEX5                   (5u)      /**< flag index5 that will upload */
#define  UPROTOCOL_UPLOAD_FLAG_INDEX6                   (6u)      /**< flag index6 that will upload */
#define  UPROTOCOL_UPLOAD_FLAG_INDEX7                   (7u)      /**< flag index7 that will upload */

/* macro of work mode set */
#define  UPROTOCOL_WORK_MODE_INDEX                      (0u)      /**< work mode inex */
#define  UPROTOCOL_FUNC_MODE_INDEX                      (1u)      /**< function mode inex */
#define  UPROTOCOL_WORK_MODE_RESP_INDEX                 (0u)      /**< work mode respond index */
#define  UPROTOCOL_WORK_MODE_RESP_LEN                   (1u)      /**< respond len */
#define  UPROTOCOL_WORK_MODE_SET_SUCCESS                (0u)      /**< config success */
#define  UPROTOCOL_WORK_MODE_SET_FAIL                   (1u)      /**< config fail */

/* macro of switch slot & function */
#define  UPROTOCOL_SW_FUNC_SLOT_INDEX                   (0u)      /**< slot inex */
#define  UPROTOCOL_SW_FUNC_EXTRA_CMD_INDEX              (1u)      /**< extra cmd inex */
#define  UPROTOCOL_SW_FUNC_FUNC_INDEX                   (2u)      /**< function inex */
#define  UPROTOCOL_SW_FUNC_ONOFF_INDEX                  (6u)      /**< on or off index */
#define  UPROTOCOL_SW_FUNC_RESP_INDEX                   (0u)      /**< switch function respond index */
#define  UPROTOCOL_SW_FUNC_RESP_LEN                     (1u)      /**< respond len */
#define  UPROTOCOL_SW_FUNC_SUCCESS                      (0u)      /**< switch function success */
#define  UPROTOCOL_SW_FUNC_FAIL                         (1u)      /**< switch function fail */

/* macro of start ctrl set */
#define  UPROTOCOL_START_CTRL_INDEX                     (0u)      /**< start or stop index */
#define  UPROTOCOL_START_CTRL_MODE_INDEX                (1u)      /**< mode index */
#define  UPROTOCOL_START_CTRL_SLOT_EN_INDEX             (2u)      /**< slot en index */
#define  UPROTOCOL_START_CTRL_FUNCTION_INDEX            (3u)      /**< function index */
#define  UPROTOCOL_START_CTRL_RESP_INDEX                (0u)      /**< chipset connect status respond index */
#define  UPROTOCOL_START_CTRL_RESP_LEN                  (1u)      /**< respond len */
#define  UPROTOCOL_START_SAMPLING                       (0u)      /**< start sampling */
#define  UPROTOCOL_STOP_SAMPLING                        (1u)      /**< stop sampling */
#define  UPROTOCOL_START_CTRL_SUCCESS                   (0u)      /**< start ctrl success */
#define  UPROTOCOL_START_CTRL_FAIL                      (1u)      /**< start ctrl fail */

/* macro of chipset connect status set */
#define  UPROTOCOL_CONN_STA_RESP_INDEX                  (0u)      /**< chipset connect status respond index */
#define  UPROTOCOL_CONN_STA_RESP_LEN                    (1u)      /**< respond len */
#define  UPROTOCOL_CONN_STA_SUCCESS                     (0u)      /**< connect ok*/
#define  UPROTOCOL_CONN_STA_FAIL                        (1u)      /**< connect error */

/* macro of ecg classification set */
#define  UPROTOCOL_CONN_ECG_CLASSIFICATION_LEN          (1u)      /**< respond len */
#define  UPROTOCOL_CONN_ECG_CLASSIFICATION_INDEX        (0u)      /**< data index */

/// universal protocol fixed header
#define  GH3X2X_UPROTOCOL_FIXED_HEADER       (0xAA)

/// universal protocol version byte
#define  GH3X2X_UPROTOCOL_VERSION            (0x11)

/// chip version string len
#define  GH3X2X_CHIP_VER_STR_LEN             (24)

/// crc8 clear val
#define  GH3X2X_CRC8_CLEAR_VAL               (0xFF)

/// crc8 val clear
#define  GH3X2X_CRC8_VAL_CLEAR(x)            do { (x) = GH3X2X_CRC8_CLEAR_VAL; } while (0)

/// size_t size
#define  GH3X2X_UPROTOCOL_SIZE_T             (4u)

/// align num
#define  GH3X2X_UPROTOCOL_ALIGN_LEFTSHIFT    (30u)

/// channel map id for user defined @ref EMFunctionID
#define  GH3X2X_FUNC_OFFSET_USER_DEF_BASE       (10)

/// number of channel map id
//#define  CHANNEL_MAP_ID_NUM                  (16)

/// rawdata diff size
#define  RAWDATA_DIFF_SIZE                   (8)

/// rawdata diff byte size
#define  RAWDATA_DIFF_BYTE_SIZE              (4)

/// rawdata diff type even
#define  RAWDATA_DIFF_EVEN                   (2)

/// rawdata diff type odd
#define  RAWDATA_DIFF_ODD                    (1)

/* universal protocol parse status */
typedef enum
{
    UPROTOCOL_STATUS_IDLE = 0,          /**< idle */
    UPROTOCOL_STATUS_GOT_FIXED,         /**< got fixed header */
    UPROTOCOL_STATUS_GOT_VER,           /**< got version */
    UPROTOCOL_STATUS_GOT_CMD,           /**< got cmd */
    UPROTOCOL_STATUS_GOT_LEN,           /**< got data len */
    UPROTOCOL_STATUS_GOT_PAYLOAD,       /**< got pyload data */
    UPROTOCOL_STATUS_GOT_CRC8,          /**< got crc8, but never goto this enum */
} EMUprotocolParseStatus;

/**
 * @fn     uint8_t GH3X2X_UprotocolPacketFormat(uint8_t uchCmd, uint8_t *puchPacketBuffer,
 *                                          uint8_t *puchPayloadData, uint8_t uchPayloadDataLen)
 *
 * @brief  universal protocol format packet
 *
 * @attention   None
 *
 * @param[out]   puchPacketBuffer    pointer to respond packet buffer
 * @param[in]    uchCmd              cmd id
 * @param[in]    puchPayloadData     pointer to payload buffer
 * @param[in]    uchPayloadDataLen   payload len
 *
 * @return  len after make packet
 */
uint8_t GH3X2X_UprotocolPacketFormat(uint8_t uchCmd, uint8_t *puchPacketBuffer, uint8_t *puchPayloadData, uint8_t uchPayloadDataLen);

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

/**
 * @fn       void GH3X2X_FillGsensorData(uint16_t* pusPayloadIndex, STGsensorRawdata* pstGsAxisValue
 *                                        y)
 *
 * @brief  fill g sensor data to protocol data
 *
 * @attention    None
 *
 * @param[in]    pstGsAxisValue       g sensor data
 * @param[in]    emGsSensitivity     g sensor sensitivity
 * @param[out]   pusPayloadIndex      current index of payload
 *
 * @return    None
 */
//void GH3X2X_FillGsensorData(uint16_t* pusPayloadIndex, STGsensorRawdata* pstGsAxisValue);

/**
 * @fn       void GH3X2X_FillAlgoData(uint16_t* pusPayloadIndex, int32_t nAlgoCalcResultArr[], uint8_t uchAlgoResNum)
 *
 * @brief  fill algorithm data to protocol data
 *
 * @attention    None
 *
 * @param[in]    nAlgoCalcResultArr  pointer to algorithm result array
 * @param[in]    uchAlgoResNum       algorithm data num
 * @param[out]   pusPayloadIndex      current index of payload
 *
 * @return    None
 */
void GH3X2X_FillAlgoData(uint16_t* pusPayloadIndex, int32_t nAlgoCalcResultArr[], uint8_t uchAlgoResNum);

/**
 * @fn       void GH3X2X_FillElectrodeWearRevertData(uint16_t* pusPayloadIndex)
 *
 * @brief  fill electrode wear revert data to protocol data buf
 *
 * @attention    None
 *
 * @param[in]    pusPayloadIndex      current index of payload
 * @param[out]   None
 *
 * @return    None
 */
void GH3X2X_FillElectrodeWearRevertData(uint16_t* pusPayloadIndex);

char *GH3X2X_GetVersion(uint8_t uchGetVersionType);


#endif /* _GH3X2X_DRV_UPROTOCOL_H_ */

/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/
