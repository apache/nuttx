/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3x2x_drv_interface.c
 *
 * @brief   gh3x2x interface functions
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



/// swap size of GU16
#define SWAP_SIZE_OF_GU16               (2)

/// swap index of next byte
#define SWAP_INDEX_OF_NEXT_BYTE         (1)

/* index of interface buffer */
#define INTERFACE_INDEX_0_BUFFER        (0)     /**< index 0 of buffer */
#define INTERFACE_INDEX_1_BUFFER        (1)     /**< index 1 of buffer */
#define INTERFACE_INDEX_2_BUFFER        (2)     /**< index 2 of buffer */
#define INTERFACE_INDEX_3_BUFFER        (3)     /**< index 3 of buffer */
#define INTERFACE_INDEX_4_BUFFER        (4)     /**< index 4 of buffer */
#define INTERFACE_INDEX_5_BUFFER        (5)     /**< index 5 of buffer */
#define INTERFACE_INDEX_6_BUFFER        (6)     /**< index 6 of buffer */
#define INTERFACE_INDEX_7_BUFFER        (7)     /**< index 7 of buffer */

/* spi operation */
#define SPI_CMD_BUFFER_LEN              (1)     /**< spi cmd buffer len */
#define SPI_WRITE_REG_BUFFER_LEN        (7)     /**< spi write reg buffer len */
#define SPI_WRITE_REG_LEN_H             (0x00)  /**< spi write reg len high byte */ 
#define SPI_WRITE_REG_LEN_L             (0x02)  /**< spi write reg len low byte */
#define SPI_READ_REG_BUFFER_LEN         (3)     /**< spi read reg buffer len */
#define SPI_READ_REG_CMD_LEN            (1)     /**< spi read reg buffer len */
#define SPI_READ_REG_DATA_LEN           (2)     /**< spi read reg buffer len */

/* i2c operation */
#define I2C_CMD_BUFFER_LEN              (3)     /**< i2c cmd buffer len */
#define I2C_WRITE_REG_BUFFER_LEN        (4)     /**< i2c write reg buffer len */
#define I2C_READ_REG_ADDR_LEN           (2)     /**< i2c read reg buffer len */
#define I2C_READ_REG_DATA_LEN           (2)     /**< i2c read reg buffer len */


/// i2c device addr
GU8 g_uchI2cDeviceId = GH3X2X_I2C_DEVICE_ID_BASE;

/// delay us func ptr
static pfnDelayUs   g_pDelayUsFunc      = GH3X2X_PTR_NULL;

/// spi write func ptr
static pfnSpiRWCtrl g_pSpiWriteFunc     = GH3X2X_PTR_NULL;

/// spi read func ptr
static pfnSpiRWCtrl g_pSpiReadFunc      = GH3X2X_PTR_NULL;

/// spi cs ctrl func ptr
static pfnSpiCsCtrl g_pSpiCsCtrlFunc    = GH3X2X_PTR_NULL;

/// i2c write func ptr
static pfnI2cWrite  g_pI2cWriteFunc     = GH3X2X_PTR_NULL;

/// i2c read func ptr
static pfnI2cRead   g_pI2cReadFunc      = GH3X2X_PTR_NULL;

/// gh3x2x send cmd func ptr
static pfnSendCmd   g_pGh3x2xSendCmdFunc  = GH3X2X_PTR_NULL;

/// gh3x2x write reg func ptr
static pfnWriteReg  g_pGh3x2xWriteRegFunc = GH3X2X_PTR_NULL;

/// gh3x2x read reg func ptr
static pfnReadReg   g_pGh3x2xReadRegFunc  = GH3X2X_PTR_NULL;

/// gh3x2x read regs func ptr
static pfnReadRegs  g_pGh3x2xReadRegsFunc = GH3X2X_PTR_NULL;

/// gh3x2x read fifo func ptr
static pfnReadFifo  g_pGh3x2xReadFifoFunc = GH3X2X_PTR_NULL;


/**
 * @fn     void GH3X2X_SwapLittleBigEndianData(GU8 *puchDataBuffer, GU16 usDataLength)
 *
 * @brief  Swap little big endian data
 *
 * @attention   None
 *
 * @param[in]   puchDataBuffer   pointer to data buffer.
 * @param[in]   usDataLength    data buffer len.
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_SwapLittleBigEndianData(GU8 *puchDataBuffer, GU16 usDataLength)
{
    GU16 usIndex = 0;

    for (usIndex = 0; usIndex < (usDataLength / SWAP_SIZE_OF_GU16); usIndex++)
    {
        puchDataBuffer[usIndex * SWAP_SIZE_OF_GU16] ^= \
                                                puchDataBuffer[usIndex * SWAP_SIZE_OF_GU16 + SWAP_INDEX_OF_NEXT_BYTE];
        puchDataBuffer[usIndex * SWAP_SIZE_OF_GU16 + SWAP_INDEX_OF_NEXT_BYTE] ^= \
                                                puchDataBuffer[usIndex * SWAP_SIZE_OF_GU16];
        puchDataBuffer[usIndex * SWAP_SIZE_OF_GU16] ^= \
                                                puchDataBuffer[usIndex * SWAP_SIZE_OF_GU16 + SWAP_INDEX_OF_NEXT_BYTE];
    }
}

/**
 * @fn     void GH3X2X_DelayUs(GU16 usUsCnt)
 *
 * @brief  Delay N Us for gh3x2x
 *
 * @attention   None
 *
 * @param[in]   usUsCnt     count need to delay(Us)
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_DelayUs(GU16 usUsCnt)
{
    if (g_pDelayUsFunc != GH3X2X_PTR_NULL)
    {
        g_pDelayUsFunc(usUsCnt);
    }
    else
    {
        volatile GU8 uchIndex;
        while (usUsCnt > 0)
        {
            for (uchIndex = 0; uchIndex < GH3X2X_DELAY_ONE_US_CNT; uchIndex++)
            {
                // do nothing, like _nop(); 
            }
            usUsCnt--;
        }
    }
}

/**
 * @fn     void GH3X2X_RegisterDelayUsCallback(void (*pPlatformDelayUsFunc)(GU16 usUsec))
 *
 * @brief  Register delay us callback function
 *
 * @attention   None
 *
 * @param[in]   pPlatformDelayUsFunc      pointer to delay us function
 * @param[out]  None
 *
 * @return  None
 */
void GH3X2X_RegisterDelayUsCallback(void (*pPlatformDelayUsFunc)(GU16 usUsec))
{
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    g_pDelayUsFunc = pPlatformDelayUsFunc;
}

/**
 * @fn     static void GH3X2X_SpiSendCmd(GU8 uchCmd)
 *
 * @brief  Send cmd via spi
 *
 * @attention   None
 *
 * @param[in]   uchCmd      spi cmd
 * @param[out]  None
 *
 * @return  None
 */
static void GH3X2X_SpiSendCmd(GU8 uchCmd)
{
    GU8 uchBuffArr[SPI_CMD_BUFFER_LEN];

    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_LOW);
    uchBuffArr[INTERFACE_INDEX_0_BUFFER] = uchCmd;
    g_pSpiWriteFunc(uchBuffArr, SPI_CMD_BUFFER_LEN);
    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_HIGH);
    GH3X2X_DelayUs(GH3X2X_SPI_END_DELAY_X_US);
}


static void GH3X2X_SpiSendCmdHardwareCs(GU8 uchCmd)
{
    GU8 uchBuffArr[SPI_CMD_BUFFER_LEN];

    uchBuffArr[INTERFACE_INDEX_0_BUFFER] = uchCmd;
    g_pSpiWriteFunc(uchBuffArr, SPI_CMD_BUFFER_LEN);
}


/**
 * @fn     static void GH3X2X_SpiWriteReg(GU16 usRegAddr, GU16 usRegValue)
 *
 * @brief  Write register via spi
 *
 * @attention   None
 *
 * @param[in]   usRegAddr       register addr
 * @param[in]   usRegValue      register data
 * @param[out]  None
 *
 * @return  None
 */
static void GH3X2X_SpiWriteReg(GU16 usRegAddr, GU16 usRegValue)
{
    GU8 uchBuffArr[SPI_WRITE_REG_BUFFER_LEN];

    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_LOW);
    uchBuffArr[INTERFACE_INDEX_0_BUFFER] = GH3X2X_SPI_CMD_WRITE;
    uchBuffArr[INTERFACE_INDEX_1_BUFFER] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(usRegAddr);
    uchBuffArr[INTERFACE_INDEX_2_BUFFER] = GH3X2X_GET_LOW_BYTE_FROM_WORD(usRegAddr);
    uchBuffArr[INTERFACE_INDEX_3_BUFFER] = SPI_WRITE_REG_LEN_H;
    uchBuffArr[INTERFACE_INDEX_4_BUFFER] = SPI_WRITE_REG_LEN_L;
    uchBuffArr[INTERFACE_INDEX_5_BUFFER] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(usRegValue);
    uchBuffArr[INTERFACE_INDEX_6_BUFFER] = GH3X2X_GET_LOW_BYTE_FROM_WORD(usRegValue);
    g_pSpiWriteFunc(uchBuffArr, SPI_WRITE_REG_BUFFER_LEN);
    GH3X2X_DelayUs(GH3X2X_SPI_WAIT_DELAY_X_US);
    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_HIGH);
    GH3X2X_DelayUs(GH3X2X_SPI_END_DELAY_X_US);
}


static void GH3X2X_SpiWriteRegHardwareCs(GU16 usRegAddr, GU16 usRegValue)
{
    GU8 puchWriteBuf[7];
    puchWriteBuf[0] = 0xF0;
    puchWriteBuf[1] = usRegAddr>>8;
    puchWriteBuf[2] = usRegAddr&0x00FF;
    puchWriteBuf[3] = 0;
    puchWriteBuf[4] = 2;
    puchWriteBuf[5] = usRegValue>>8;
    puchWriteBuf[6] = usRegValue&0x00FF;
    g_pSpiWriteFunc(puchWriteBuf, SPI_WRITE_REG_BUFFER_LEN);
}






/**
 * @fn     static GU16 GH3X2X_SpiReadReg(GU16 usRegAddr)
 *
 * @brief  Read register via spi
 *
 * @attention   None
 *
 * @param[in]   usRegAddr      register addr
 * @param[out]  None
 *
 * @return  register data
 */
static GU16 GH3X2X_SpiReadReg(GU16 usRegAddr)
{
    GU8 uchBuffArr[SPI_READ_REG_BUFFER_LEN] = {0};

    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_LOW);
    uchBuffArr[INTERFACE_INDEX_0_BUFFER] = GH3X2X_SPI_CMD_WRITE;
    uchBuffArr[INTERFACE_INDEX_1_BUFFER] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(usRegAddr);
    uchBuffArr[INTERFACE_INDEX_2_BUFFER] = GH3X2X_GET_LOW_BYTE_FROM_WORD(usRegAddr);
    g_pSpiWriteFunc(uchBuffArr, SPI_READ_REG_BUFFER_LEN);
    GH3X2X_DelayUs(GH3X2X_SPI_WAIT_DELAY_X_US);
    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_HIGH);
    GH3X2X_DelayUs(GH3X2X_SPI_END_DELAY_X_US);
    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_LOW);
    uchBuffArr[INTERFACE_INDEX_0_BUFFER] = GH3X2X_SPI_CMD_READ;
    g_pSpiWriteFunc(uchBuffArr, SPI_READ_REG_CMD_LEN);
    g_pSpiReadFunc(uchBuffArr, SPI_READ_REG_DATA_LEN);
    GH3X2X_DelayUs(GH3X2X_SPI_WAIT_DELAY_X_US);
    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_HIGH);
    GH3X2X_DelayUs(GH3X2X_SPI_END_DELAY_X_US);
    return GH3X2X_MAKEUP_WORD(uchBuffArr[INTERFACE_INDEX_0_BUFFER], uchBuffArr[INTERFACE_INDEX_1_BUFFER]);
}


static GU16 GH3X2X_SpiReadRegHardwareCs(GU16 usRegAddr)
{
    GU8 puchWriteBuf[7];
    GU8 puchReadBuf[2];
    puchWriteBuf[0] = 0xF0;
    puchWriteBuf[1] = usRegAddr>>8;
    puchWriteBuf[2] = usRegAddr&0x00FF;
    g_pSpiWriteFunc(puchWriteBuf, 3);
    g_pSpiReadFunc(puchReadBuf, 2);
    return ((((GU16)puchReadBuf[0])<<8) + puchReadBuf[1]);
}





/**
 * @fn     static void GH3X2X_SpiReadRegs(GU16 usRegAddr, GU16 *pusRegValueBuffer, GU8 uchLen)
 *
 * @brief  Read multi register via spi
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
static void GH3X2X_SpiReadRegs(GU16 usRegAddr, GU16 *pusRegValueBuffer, GU8 uchLen)
{
    GU8 uchBuffArr[SPI_READ_REG_BUFFER_LEN] = {0};

    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_LOW);
    uchBuffArr[INTERFACE_INDEX_0_BUFFER] = GH3X2X_SPI_CMD_WRITE;
    uchBuffArr[INTERFACE_INDEX_1_BUFFER] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(usRegAddr);
    uchBuffArr[INTERFACE_INDEX_2_BUFFER] = GH3X2X_GET_LOW_BYTE_FROM_WORD(usRegAddr);
    g_pSpiWriteFunc(uchBuffArr, SPI_READ_REG_BUFFER_LEN);
    GH3X2X_DelayUs(GH3X2X_SPI_WAIT_DELAY_X_US);
    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_HIGH);
    GH3X2X_DelayUs(GH3X2X_SPI_END_DELAY_X_US);
    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_LOW);
    uchBuffArr[INTERFACE_INDEX_0_BUFFER] = GH3X2X_SPI_CMD_READ;
    g_pSpiWriteFunc(uchBuffArr, SPI_READ_REG_CMD_LEN);
    g_pSpiReadFunc((GU8 *)pusRegValueBuffer, uchLen * SPI_READ_REG_DATA_LEN);
    GH3X2X_SwapLittleBigEndianData((GU8 *)pusRegValueBuffer, (GU16)((GU16)uchLen * SPI_READ_REG_DATA_LEN));
    GH3X2X_DelayUs(GH3X2X_SPI_WAIT_DELAY_X_US);
    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_HIGH);
    GH3X2X_DelayUs(GH3X2X_SPI_END_DELAY_X_US);
}



static void GH3X2X_SpiReadRegsHardwareCs(GU16 usRegAddr, GU16 *pusRegValueBuffer, GU8 uchLen)
{
    GU8 puchWriteBuf[3];
    puchWriteBuf[0] = 0xF0;
    puchWriteBuf[1] = usRegAddr>>8;
    puchWriteBuf[2] = usRegAddr&0x00FF;
    g_pSpiWriteFunc(puchWriteBuf, 3);
    g_pSpiReadFunc((GU8 *)pusRegValueBuffer, uchLen * SPI_READ_REG_DATA_LEN);
    GH3X2X_SwapLittleBigEndianData((GU8 *)pusRegValueBuffer, (GU16)((GU16)uchLen * SPI_READ_REG_DATA_LEN));
    
}





/**
 * @fn     static void GH3X2X_SpiReadFifo(GU8 *puchDataBuffer, GU16 usLen)
 *
 * @brief  Read fifo via spi
 *
 * @attention   None
 *
 * @param[in]   puchDataBuffer      pointer to buffer for fifo rawdata
 * @param[in]   usLen               fifo bytes len
 * @param[out]  None
 *
 * @return  None
 */
static void GH3X2X_SpiReadFifo(GU8 *puchDataBuffer, GU16 usLen)
{
    GU8 uchBuffArr[SPI_READ_REG_BUFFER_LEN] = {0};

    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_LOW);
    uchBuffArr[INTERFACE_INDEX_0_BUFFER] = GH3X2X_SPI_CMD_WRITE;
    uchBuffArr[INTERFACE_INDEX_1_BUFFER] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(GH3X2X_FIFO_REG_ADDR);
    uchBuffArr[INTERFACE_INDEX_2_BUFFER] = GH3X2X_GET_LOW_BYTE_FROM_WORD(GH3X2X_FIFO_REG_ADDR);
    g_pSpiWriteFunc(uchBuffArr, SPI_READ_REG_BUFFER_LEN);
    GH3X2X_DelayUs(GH3X2X_SPI_WAIT_DELAY_X_US);
    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_HIGH);
    GH3X2X_DelayUs(GH3X2X_SPI_END_DELAY_X_US);
    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_LOW);
    uchBuffArr[INTERFACE_INDEX_0_BUFFER] = GH3X2X_SPI_CMD_READ;
    g_pSpiWriteFunc(uchBuffArr, SPI_READ_REG_CMD_LEN);
    g_pSpiReadFunc(puchDataBuffer, usLen);
    GH3X2X_DelayUs(GH3X2X_SPI_WAIT_DELAY_X_US);
    g_pSpiCsCtrlFunc(GH3X2X_SPI_CS_CTRL_HIGH);
    GH3X2X_DelayUs(GH3X2X_SPI_END_DELAY_X_US);
}

static void GH3X2X_SpiReadFifoHardwareCs(GU8 *puchDataBuffer, GU16 usLen)
{
    GU8 puchWriteBuf[3];
    puchWriteBuf[0] = 0xF0;
    puchWriteBuf[1] = 0xAAAA>>8;
    puchWriteBuf[2] = 0xAAAA&0x00FF;
    g_pSpiWriteFunc(puchWriteBuf, 3);
    g_pSpiReadFunc(puchDataBuffer, usLen);
}


/**
 * @fn     static void GH3X2X_I2cSendCmd(GU8 uchCmd)
 *
 * @brief  Send cmd via i2c
 *
 * @attention   None
 *
 * @param[in]   uchCmd      i2c cmd
 * @param[out]  None
 *
 * @return  None
 */
static void GH3X2X_I2cSendCmd(GU8 uchCmd)
{
    GU8 uchBuffArr[I2C_CMD_BUFFER_LEN] = {0};

    uchBuffArr[INTERFACE_INDEX_0_BUFFER] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(GH3X2X_I2C_CMD_ADDR);
    uchBuffArr[INTERFACE_INDEX_1_BUFFER] = GH3X2X_GET_LOW_BYTE_FROM_WORD(GH3X2X_I2C_CMD_ADDR);
    uchBuffArr[INTERFACE_INDEX_2_BUFFER] = uchCmd;
    g_pI2cWriteFunc(g_uchI2cDeviceId, uchBuffArr, I2C_CMD_BUFFER_LEN);
}

/**
 * @fn     static void GH3X2X_I2cWriteReg(GU16 usRegAddr, GU16 usRegValue)
 *
 * @brief  Write register via i2c
 *
 * @attention   None
 *
 * @param[in]   usRegAddr       register addr
 * @param[in]   usRegValue      register data
 * @param[out]  None
 *
 * @return  None
 */
static void GH3X2X_I2cWriteReg(GU16 usRegAddr, GU16 usRegValue)
{
    GU8 uchBuffArr[I2C_WRITE_REG_BUFFER_LEN] = {0};

    uchBuffArr[INTERFACE_INDEX_0_BUFFER] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(usRegAddr);
    uchBuffArr[INTERFACE_INDEX_1_BUFFER] = GH3X2X_GET_LOW_BYTE_FROM_WORD(usRegAddr);
    uchBuffArr[INTERFACE_INDEX_2_BUFFER] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(usRegValue);
    uchBuffArr[INTERFACE_INDEX_3_BUFFER] = GH3X2X_GET_LOW_BYTE_FROM_WORD(usRegValue);
    g_pI2cWriteFunc(g_uchI2cDeviceId, uchBuffArr, I2C_WRITE_REG_BUFFER_LEN);
}

/**
 * @fn     static GU16 GH3X2X_I2cReadReg(GU16 usRegAddr)
 *
 * @brief  Read register via i2c
 *
 * @attention   None
 *
 * @param[in]   usRegAddr      register addr
 * @param[out]  None
 *
 * @return  register data
 */
static GU16 GH3X2X_I2cReadReg(GU16 usRegAddr)
{
    GU8 uchCmdBuffArr[I2C_READ_REG_ADDR_LEN] = {0};
    GU8 uchReadBuffArr[I2C_READ_REG_DATA_LEN] = {0};

    uchCmdBuffArr[INTERFACE_INDEX_0_BUFFER] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(usRegAddr);
    uchCmdBuffArr[INTERFACE_INDEX_1_BUFFER] = GH3X2X_GET_LOW_BYTE_FROM_WORD(usRegAddr);
    g_pI2cReadFunc(g_uchI2cDeviceId, uchCmdBuffArr, I2C_READ_REG_ADDR_LEN, uchReadBuffArr, I2C_READ_REG_DATA_LEN);
    return GH3X2X_MAKEUP_WORD(uchReadBuffArr[INTERFACE_INDEX_0_BUFFER], uchReadBuffArr[INTERFACE_INDEX_1_BUFFER]);
}

/**
 * @fn     static void GH3X2X_I2cReadRegs(GU16 usRegAddr, GU16 *pusRegValueArr, GU8 uchLen)
 *
 * @brief  Read multi register via i2c
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
static void GH3X2X_I2cReadRegs(GU16 usRegAddr, GU16 *pusRegValueArr, GU8 uchLen)
{
    GU8 uchBuffArr[I2C_READ_REG_ADDR_LEN] = {0};

    uchBuffArr[INTERFACE_INDEX_0_BUFFER] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(usRegAddr);
    uchBuffArr[INTERFACE_INDEX_1_BUFFER] = GH3X2X_GET_LOW_BYTE_FROM_WORD(usRegAddr);
    g_pI2cReadFunc(g_uchI2cDeviceId, uchBuffArr, I2C_READ_REG_ADDR_LEN, \
                        (GU8 *)pusRegValueArr, uchLen * I2C_READ_REG_DATA_LEN);
    GH3X2X_SwapLittleBigEndianData((GU8 *)pusRegValueArr, (GU16)uchLen * I2C_READ_REG_DATA_LEN);
}

/**
 * @fn     static void GH3X2X_I2cReadFifo(GU8 *puchDataBuffer, GU16 usLen)
 *
 * @brief  Read fifo via i2c
 *
 * @attention   None
 *
 * @param[in]   puchDataBuffer      pointer to buffer for fifo rawdata
 * @param[in]   usLen               fifo bytes len
 * @param[out]  None
 *
 * @return  None
 */
static void GH3X2X_I2cReadFifo(GU8 *puchDataBuffer, GU16 usLen)
{
    GU8 uchCmdBuffArr[I2C_READ_REG_ADDR_LEN] = {0};

    uchCmdBuffArr[INTERFACE_INDEX_0_BUFFER] = GH3X2X_GET_HIGH_BYTE_FROM_WORD(GH3X2X_FIFO_REG_ADDR);
    uchCmdBuffArr[INTERFACE_INDEX_1_BUFFER] = GH3X2X_GET_LOW_BYTE_FROM_WORD(GH3X2X_FIFO_REG_ADDR);
    g_pI2cReadFunc(g_uchI2cDeviceId, uchCmdBuffArr, I2C_READ_REG_ADDR_LEN, puchDataBuffer, usLen);
}

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
void GH3X2X_SendCmd(GU8 uchCmd)
{
    if (g_pGh3x2xSendCmdFunc != GH3X2X_PTR_NULL)
    {
        g_pGh3x2xSendCmdFunc(uchCmd);
    }
    else
    {
        GH3X2X_DEBUG_LOG("communicate interface register error!\r\n");
    }
}

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
void GH3X2X_WriteReg(GU16 usRegAddr, GU16 usRegValue)
{
    if (g_pGh3x2xWriteRegFunc != GH3X2X_PTR_NULL)
    {
        g_pGh3x2xWriteRegFunc(usRegAddr, usRegValue);
    }
    else
    {
        GH3X2X_DEBUG_LOG("communicate interface register error!\r\n");
    }
}

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
GU16 GH3X2X_ReadReg(GU16 usRegAddr)
{
    GU16 usRegData = 0;
    if (g_pGh3x2xReadRegFunc != GH3X2X_PTR_NULL)
    {
        usRegData = g_pGh3x2xReadRegFunc(usRegAddr);
    }
    else
    {
        GH3X2X_DEBUG_LOG("communicate interface register error!\r\n");
    }
    return usRegData;
}

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
GU32 GH3X2X_ReadRegs(GU16 usRegAddr, GU16 *pusRegValueBuffer, GU16 usLen)
{
    GU32 unRegData = 0;
    if (g_pGh3x2xReadRegsFunc != GH3X2X_PTR_NULL)
    {
        g_pGh3x2xReadRegsFunc(usRegAddr, pusRegValueBuffer, usLen);
    }
    else
    {
        GH3X2X_DEBUG_LOG("communicate interface register error!\r\n");
    }
    return unRegData;
}

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
void GH3X2X_ReadFifo(GU8 *puchDataBuffer, GU16 usLen)
{
    if (g_pGh3x2xReadFifoFunc != GH3X2X_PTR_NULL)
    {
        g_pGh3x2xReadFifoFunc(puchDataBuffer, usLen);
    }
    else
    {
        GH3X2X_DEBUG_LOG("communicate interface register error!\r\n");
    }
}

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
void GH3X2X_WriteRegBitField(GU16 usRegAddr, GU8 uchLsb, GU8 uchMsb, GU16 usValue)
{
    GU16 usMakData = ((((GU16)0x0001) << (uchMsb - uchLsb + 1)) - 1) << uchLsb;
    GU16 usRegData = 0;
    if ((g_pGh3x2xReadRegFunc != GH3X2X_PTR_NULL) && (g_pGh3x2xWriteRegFunc != GH3X2X_PTR_NULL))
    {
        usRegData = g_pGh3x2xReadRegFunc(usRegAddr);
        GH3X2X_VAL_CLEAR_BIT(usRegData, usMakData);
        GH3X2X_VAL_SET_BIT(usRegData, (usValue << uchLsb) & usMakData);
        g_pGh3x2xWriteRegFunc(usRegAddr, usRegData);
    }
    else
    {
        GH3X2X_DEBUG_LOG("communicate interface register error!\r\n");
    }
}

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
GU16 GH3X2X_ReadRegBitField(GU16 usRegAddr, GU8 uchLsb, GU8 uchMsb)
{
    GU16 usMakData = ((((GU16)0x0001) << (uchMsb - uchLsb + 1)) - 1) << uchLsb;
    GU16 usRegData = 0;
    if (g_pGh3x2xReadRegFunc != GH3X2X_PTR_NULL)
    {
        usRegData = g_pGh3x2xReadRegFunc(usRegAddr);
        GH3X2X_VAL_GET_BIT(usRegData, usMakData);
        usRegData >>= uchLsb;
    }
    else
    {
        GH3X2X_DEBUG_LOG("communicate interface register error!\r\n");
    }
    return usRegData;
}

/**
 * @fn     GS8 GH3X2X_RegisterSpiOperationFunc(GU8 (*pSpiWriteFunc)(GU8 uchWriteBytesArr[], GU16 usWriteLen),
 *                                  GU8 (*pSpiReadFunc)(GU8 uchReadBytesArr[], GU16 usMaxReadLen),
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
                                    void (*pSpiCsCtrlFunc)(GU8 uchCsLevelHigh))
{
    GS8 chRet = GH3X2X_RET_PARAMETER_ERROR;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if ((pSpiWriteFunc != GH3X2X_PTR_NULL) && (pSpiReadFunc != GH3X2X_PTR_NULL) && (pSpiCsCtrlFunc != GH3X2X_PTR_NULL))
    {
        g_pSpiWriteFunc  = pSpiWriteFunc;
        g_pSpiReadFunc   = pSpiReadFunc;
        g_pSpiCsCtrlFunc = pSpiCsCtrlFunc;

        g_pGh3x2xSendCmdFunc  = GH3X2X_SpiSendCmd;
        g_pGh3x2xWriteRegFunc = GH3X2X_SpiWriteReg;
        g_pGh3x2xReadRegFunc  = GH3X2X_SpiReadReg;
        g_pGh3x2xReadRegsFunc = GH3X2X_SpiReadRegs;
        g_pGh3x2xReadFifoFunc = GH3X2X_SpiReadFifo;
        chRet = GH3X2X_RET_OK;
    }
    else
    {
        GH3X2X_DEBUG_LOG("spi register param error!\r\n");
    }
    return chRet;
}

GS8 GH3X2X_RegisterSpiHwCsOperationFunc(GU8 (*pSpiWriteFunc)(GU8 uchWriteBytesArr[], GU16 usWriteLen),
                                    GU8 (*pSpiWriteF1ReadFunc)(GU8 uchReadBytesArr[], GU16 usMaxReadLen)
                                    )
{
    GS8 chRet = GH3X2X_RET_PARAMETER_ERROR;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if ((pSpiWriteFunc != GH3X2X_PTR_NULL) && (pSpiWriteF1ReadFunc != GH3X2X_PTR_NULL) )
    {
        g_pSpiWriteFunc  = pSpiWriteFunc;
        g_pSpiReadFunc   = pSpiWriteF1ReadFunc;

        g_pGh3x2xSendCmdFunc  = GH3X2X_SpiSendCmdHardwareCs;
        g_pGh3x2xWriteRegFunc = GH3X2X_SpiWriteRegHardwareCs;
        g_pGh3x2xReadRegFunc  = GH3X2X_SpiReadRegHardwareCs;
        g_pGh3x2xReadRegsFunc = GH3X2X_SpiReadRegsHardwareCs;
        g_pGh3x2xReadFifoFunc = GH3X2X_SpiReadFifoHardwareCs;
        chRet = GH3X2X_RET_OK;
    }
    else
    {
        GH3X2X_DEBUG_LOG("spi register param error!\r\n");
    }
    return chRet;
}


/**
 * @fn     GS8 GH3X2X_RegisterI2cOperationFunc(GU8 uchI2cIdLowTwoBitsSelect,
 *                              GU8 (*pI2cWriteFunc)(GU8 uchDeviceId, const GU8 uchWriteBytesArr[], GU16 usWriteLen),
 *                              GU8 (*pI2cReadFunc)(GU8 uchDeviceId, const GU8 uchCmddBytesArr[], GU16 usCmddLen,
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
                                                        GU8 uchReadBytesArr[], GU16 usMaxReadLen))
{
    GS8 chRet = GH3X2X_RET_PARAMETER_ERROR;
    GH3X2X_DEBUG_LOG_PARAM("%s\r\n", __FUNCTION__);
    if ((pI2cWriteFunc != GH3X2X_PTR_NULL) && (pI2cReadFunc != GH3X2X_PTR_NULL)
        && (uchI2cIdLowTwoBitsSelect < GH3X2X_I2C_ID_INVALID))
    {
        g_uchI2cDeviceId = GH3X2X_I2C_DEVICE_ID_BASE | (uchI2cIdLowTwoBitsSelect << 1);
        g_pI2cWriteFunc  = pI2cWriteFunc;
        g_pI2cReadFunc   = pI2cReadFunc;

        g_pGh3x2xSendCmdFunc  = GH3X2X_I2cSendCmd;
        g_pGh3x2xWriteRegFunc = GH3X2X_I2cWriteReg;
        g_pGh3x2xReadRegFunc  = GH3X2X_I2cReadReg;
        g_pGh3x2xReadRegsFunc = GH3X2X_I2cReadRegs;
        g_pGh3x2xReadFifoFunc = GH3X2X_I2cReadFifo;
        chRet = GH3X2X_RET_OK;
    }
    else
    {
        GH3X2X_DEBUG_LOG("i2c register param error!\r\n");
    }
    return chRet;
}

/********END OF FILE********* Copyright (c) 2003 - 2021, Goodix Co., Ltd. ********/
