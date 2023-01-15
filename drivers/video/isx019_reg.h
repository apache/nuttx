/****************************************************************************
 * drivers/video/isx019_reg.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __DRIVERS_VIDEO_ISX019_REG_H
#define __DRIVERS_VIDEO_ISX019_REG_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FPGA I2C setting */

#define FPGA_I2C_SLVADDR           (0x1b)
#define FPGA_I2C_SLVADDR_LEN       (7)
#define FPGA_I2C_REGSIZE_MAX       (1)
#define FPGA_I2C_REGADDR_LEN       (1)
#define FPGA_I2C_FREQUENCY         (100000) /* Standard mode : 100kHz */

/* FPGA I2C offset */

#define FPGA_I2C_OFFSET_ADDR       (0)
#define FPGA_I2C_OFFSET_WRITEDATA  (1)
#define FPGA_I2C_OFFSET_READDATA   (0)

/* FPGA register address */

#define FPGA_RESET                     (0x00)
#define FPGA_DATA_OUTPUT               (0x01)
#define FPGA_FORMAT_AND_SCALE          (0x02)
#define FPGA_FPS_AND_THUMBNAIL         (0x03)
#define FPGA_CLIP_SIZE                 (0x04)
#define FPGA_CLIP_TOP                  (0x05)
#define FPGA_CLIP_LEFT                 (0x06)
#define FPGA_DQT_ADDRESS               (0x07)
#define FPGA_DQT_SELECT                (0x08)
#define FPGA_DQT_DATA                  (0x09)
#define FPGA_DQT_CALC_ADDRESS          (0x0A)
#define FPGA_DQT_CALC_SELECT           (0x0B)
#define FPGA_DQT_CALC_DATA             (0x0C)
#define FPGA_ACTIVATE                  (0x0D)
#define FPGA_PIN_OUTPUT_SELECT         (0x20)
#define FPGA_PIN_STATUS                (0x21)
#define FPGA_VERSION                   (0x30)

/* value for setting to FPGA register */

#define FPGA_RESET_ENABLE  (0x00)
#define FPGA_RESET_RELEASE (0x01)

#define FPGA_DATA_OUTPUT_START (0x01)
#define FPGA_DATA_OUTPUT_STOP  (0x00)

#define FPGA_FORMAT_RGB       (0x00)
#define FPGA_FORMAT_YUV       (0x01)
#define FPGA_FORMAT_JPEG      (0x02)
#define FPGA_FORMAT_THUMBNAIL (0x03)

#define FPGA_SCALE_1280_960   (0<<4)
#define FPGA_SCALE_640_480    (1<<4)
#define FPGA_SCALE_320_240    (2<<4)
#define FPGA_SCALE_160_120    (3<<4)

#define FPGA_FPS_BITS (0x03)
#define FPGA_FPS_1_1  (0x00)
#define FPGA_FPS_1_2  (0x01)
#define FPGA_FPS_1_3  (0x02)
#define FPGA_FPS_1_4  (0x03)

#define FPGA_THUMBNAIL_SCALE_1_1 (0<<4)
#define FPGA_THUMBNAIL_SCALE_1_2 (1<<4)
#define FPGA_THUMBNAIL_SCALE_1_4 (2<<4)
#define FPGA_THUMBNAIL_SCALE_1_8 (3<<4)

#define FPGA_CLIP_NON         (0x00)
#define FPGA_CLIP_1280_720    (0x01)
#define FPGA_CLIP_640_360     (0x02)
#define FPGA_CLIP_UNIT        (8)

#define FPGA_DQT_READ (0<<6)
#define FPGA_DQT_WRITE (1<<6)

#define FPGA_DQT_CURRENT (0<<7)
#define FPGA_DQT_BUFFER  (1<<7)

#define FPGA_DQT_LUMA   (0x00)
#define FPGA_DQT_CHROMA (0x01)

#define FPGA_ACTIVATE_REQUEST (0x01)

#define FPGA_PIN_OUTPUT_EXBUSY (0x00)
#define FPGA_PIN_OUTPUT_ERROR  (0x01)

#define FPGA_PIN_STATUS_EXBUSY (1<<0)
#define FPGA_PIN_STATUS_ERROR  (1<<4)

/* ISX019 I2C setting */

#define ISX019_I2C_SLVADDR         (0x1a)
#define ISX019_I2C_SLVADDR_LEN     (7)
#define ISX019_I2C_REGSIZE_MAX     (4)
#define ISX019_I2C_REGADDR_LEN     (4)
#define ISX019_I2C_FREQUENCY       (100000) /* Standard mode : 100kHz */

/* Offset of ISX019 I2C command format */

#define ISX019_I2C_OFFSET_TOTALLEN   (0)
#define ISX019_I2C_OFFSET_CMDNUM (1)
#define ISX019_I2C_OFFSET_CMDLEN (2)
#define ISX019_I2C_OFFSET_CMD    (3)
#define ISX019_I2C_OFFSET_CATEGORY   (4)
#define ISX019_I2C_OFFSET_ADDRESS_H  (5)
#define ISX019_I2C_OFFSET_ADDRESS_L  (6)
#define ISX019_I2C_OFFSET_WRITEDATA  (7)
#define ISX019_I2C_OFFSET_READSIZE   (7)
#define ISX019_I2C_OFFSET_FLASHCODE1 (4)
#define ISX019_I2C_OFFSET_FLASHCODE2 (5)
#define ISX019_I2C_OFFSET_FLASHCODE3 (6)
#define ISX019_I2C_OFFSET_FLASHCODE4 (7)
#define ISX019_I2C_OFFSET_STS        (3)
#define ISX019_I2C_OFFSET_READDATA   (4)

/* Command code of ISX019 I2C command format */

#define ISX019_I2C_CMD_FLASHLOCK  (0x00)
#define ISX019_I2C_CMD_READ       (0x01)
#define ISX019_I2C_CMD_WRITE      (0x02)
#define ISX019_I2C_CMD_FLASHWRITE (0x05)

/* Command length of ISX019 I2C command format */

#define ISX019_I2C_TOTALLEN(s)       (3 + (s))

#define ISX019_I2C_WRREQ_LEN(s)      (5 + (s))
#define ISX019_I2C_WRRES_LEN         (2)
#define ISX019_I2C_WRREQ_TOTALLEN(s) ISX019_I2C_TOTALLEN(ISX019_I2C_WRREQ_LEN(s))
#define ISX019_I2C_WRRES_TOTALLEN    ISX019_I2C_TOTALLEN(ISX019_I2C_WRRES_LEN)

#define ISX019_I2C_RDREQ_LEN         (6)
#define ISX019_I2C_RDRES_LEN(s)      (2 + (s))
#define ISX019_I2C_RDREQ_TOTALLEN    ISX019_I2C_TOTALLEN(ISX019_I2C_RDREQ_LEN)
#define ISX019_I2C_RDRES_TOTALLEN(s) ISX019_I2C_TOTALLEN(ISX019_I2C_RDRES_LEN(s))

#define ISX019_I2C_FLOCKREQ_LEN      (6)
#define ISX019_I2C_FLOCKRES_LEN      (2)
#define ISX019_I2C_FLOCKREQ_TOTALLEN ISX019_I2C_TOTALLEN(ISX019_I2C_FLOCKREQ_LEN)
#define ISX019_I2C_FLOCKRES_TOTALLEN ISX019_I2C_TOTALLEN(ISX019_I2C_FLOCKRES_LEN)

#define ISX019_I2C_FWRREQ_LEN        (2)
#define ISX019_I2C_FWRRES_LEN        (2)
#define ISX019_I2C_FWRREQ_TOTALLEN   ISX019_I2C_TOTALLEN(ISX019_I2C_FWRREQ_LEN)
#define ISX019_I2C_FWRRES_TOTALLEN   ISX019_I2C_TOTALLEN(ISX019_I2C_FWRRES_LEN)

/* Constant value of codes for flash lock */

#define ISX019_I2C_FLASHCODE1   (0x57)
#define ISX019_I2C_FLASHCODE2   (0x52)
#define ISX019_I2C_FLASHLOCK1   (0x44)
#define ISX019_I2C_FLASHLOCK2   (0x53)
#define ISX019_I2C_FLASHUNLOCK1 (0x45)
#define ISX019_I2C_FLASHUNLOCK2 (0x4e)

/* Status code of I2C command format */

#define ISX019_I2C_STS_OK         (0x01)
#define ISX019_I2C_STS_ERR_CMD    (0xf1)
#define ISX019_I2C_STS_ERR_CAT    (0xf2)
#define ISX019_I2C_STS_ERR_ADDR   (0xf3)
#define ISX019_I2C_STS_ERR_ACCESS (0xf4)
#define ISX019_I2C_STS_ERR_BYTE   (0xf7)
#define ISX019_I2C_STS_ERR_FLASH  (0xfa)

/* Category of ISX019 register */

#define CAT_VERSION       (0x00)
#define CAT_CONFIG        (0x01)
#define CAT_SYSCOM        (0x06)
#define CAT_OTP           (0x08)
#define CAT_AUTOCTRL      (0x0c)
#define CAT_SMCFG         (0x0f)
#define CAT_CATAE         (0x13)
#define CAT_AEDGRM        (0x14)
#define CAT_AEWD          (0x15)
#define CAT_AECOM         (0x18)
#define CAT_AESOUT        (0x19)
#define CAT_CATAWB        (0x21)
#define CAT_AWB_USERTYPE  (0x26)
#define CAT_AUTOCOM       (0x2c)
#define CAT_AWBSOUT       (0x31)
#define CAT_PICTTUNE      (0x34)
#define CAT_PICTGAMMA     (0x41)

/* Offset of ISX019 register */

/* For CAT_VERSION */

#define ROM_VERSION       (0x0000)

/* For CAT_CONFIG */

#define MODE_SENSSEL      (0x0000)
#define MODE_POSTSEL      (0x0001)
#define MODE_SENSPOST_SEL (0x0002)
#define MODE_IOSEL        (0x0003)
#define REVERSE           (0x0008)

/* For CAT_SYSCOM */

#define DEVSTS            (0x0001)

/* For CAT_OTP */

#define CHIP_ID           (0x0000)
#define CHIP_ID_LEN       (11)

/* For CAT_AUTOCTRL */

#define AEWEIGHTMODE      (0x0000)

/* For CAT_CATAE */

#define AEMODE            (0x0000)
#define SPOT_FRM_NUM      (0x0003)
#define SHT_PRIMODE       (0x0008)
#define GAIN_PRIMODE      (0x000c)

/* For CAT_AEDGRM */

#define SHTCTRLVAL1       (0x0008)
#define SHTCTRLVAL2       (0x0010)
#define SHTCTRLVAL3       (0x0018)
#define SHTCTRLVAL4       (0x0020)
#define SHTCTRLVAL5       (0x0028)
#define MAXFRMEXP         (0x007b)
#define EVSEL             (0x009c)

/* For CAT_AEWD */

#define AEWDMODE          (0x0000)
#define AEWEIGHTMODE_WD   (0x0001)

/* For CAT_AECOM */

#define GAIN_LEVEL        (0x0036)

/* For CAT_AESOUT */

#define SHT_TIME          (0x01cc)
#define V_TIME            (0x01d0)

/* For CAT_CATAWB */

#define AWBMODE           (0x0000)
#define AWBUSER_NO        (0x000a)

/* For CAT_AWB_USERTYPE */

#define USER0_R           (0x0000)
#define USER0_B           (0x0002)
#define USER1_R           (0x000c)
#define USER1_B           (0x000e)
#define USER2_R           (0x0018)
#define USER2_B           (0x001a)
#define USER3_R           (0x0024)
#define USER3_B           (0x0026)
#define USER4_R           (0x0030)
#define USER4_B           (0x0032)

/* For CAT_AUTOCOM */

#define CONT_R            (0x0006)
#define CONT_B            (0x0008)

/* For CAT_AWBSOUT */

#define AWBSTS            (0x0000)

/* For CAT_PICTTUNE */

#define UISHARPNESS       (0x0000)
#define UICONTRAST        (0x0001)
#define UIBRIGHTNESS      (0x0002)
#define UIHUE             (0x0004)
#define UISATURATION      (0x0005)

/* For CAT_PICTGAMMA */

#define GAM_KNOT_C0       (0x0000)
#define GAM_KNOT_C8       (0x0010)
#define GAM_KNOT_C9       (0x0012)
#define GAM_KNOT_C10      (0x0014)
#define GAM_KNOT_C11      (0x0016)
#define NR_GAM_KNOT_LOWINPUT (9)
#define NR_GAM_KNOT_HIGHINPUT (16)

#define GAM_LOWINPUT_INTERVAL (double)(32.0 / 4096.0)
#define GAM_HIGHINPUT_INTERVAL (double)(256.0 / 4096.0)
#define GAM_OUTPUT_SCALE (255)

/* Definition for ISX019 register setting value */

/* For drive mode */

#define NOT_STREAM_SENS     (0)
#define NOT_STREAM_POST     (0)
#define NOT_STREAM_SENSPOST (0)
#define NOT_STREAM_IO       (0)

#define NORM_30FPS_SENS     (2)
#define NORM_30FPS_POST     (2)
#define NORM_30FPS_SENSPOST (2)
#define NORM_30FPS_IO       (2)

#define DOL2_30FPS_SENS     (6)
#define DOL2_30FPS_POST     (2)
#define DOL2_30FPS_SENSPOST (24)
#define DOL2_30FPS_IO       (2)

#define DOL3_30FPS_SENS     (44)
#define DOL3_30FPS_POST     (125)
#define DOL3_30FPS_SENSPOST (160)
#define DOL3_30FPS_IO       (2)

#define NORM_25FPS_SENS     (3)
#define NORM_25FPS_POST     (8)
#define NORM_25FPS_SENSPOST (8)
#define NORM_25FPS_IO       (2)

#define DOL2_25FPS_SENS     (7)
#define DOL2_25FPS_POST     (8)
#define DOL2_25FPS_SENSPOST (30)
#define DOL2_25FPS_IO       (2)

#define DOL3_25FPS_SENS     (45)
#define DOL3_25FPS_POST     (131)
#define DOL3_25FPS_SENSPOST (166)
#define DOL3_25FPS_IO       (2)

/* For REVERSE register(H_REVERSE and V_REVERSE) */

#define H_REVERSE           (0x01)
#define V_REVERSE           (0x02)

/* For AEWEIGHTMODE */

#define AEWEIGHT_AVERAGE    (0x00)
#define AEWEIGHT_CENTER     (0x01)
#define AEWEIGHT_SPOT       (0x02)
#define AEWEIGHT_MATRIX     (0x03)

/* For AEWEIGHTMODE(HDR) */

#define AEWEIGHTHDR_AVERAGE (0x00)
#define AEWEIGHTHDR_CENTER  (0x05)
#define AEWEIGHTHDR_SPOT    (0x0a)
#define AEWEIGHTHDR_MATRIX  (0x0f)

/* For AWBMODE */

#define AWBMODE_AUTO        (0x00)
#define AWBMODE_HOLD        (0x02)
#define AWBMODE_MANUAL      (0x04)

/* For AEMODE */

#define AEMODE_AUTO         (0x00)
#define AEMODE_HOLD         (0x01)

/* For AEWDMODE */

#define AEWDMODE_AUTO       (0x00)
#define AEWDMODE_HDR        (0x01)
#define AEWDMODE_NORMAL     (0x02)

/* For AWBSTS */

#define AWBSTS_STABLE       (0x02)
#define AWBSTS_AEWAIT       (0x06)

/* For USERX_R, USERX_B */

#define RED_INCANDESCENT    (0x115f)
#define BLUE_INCANDESCENT   (0x0e40)
#define RED_FLUORESCENT     (0x0daf)
#define BLUE_FLUORESCENT    (0x1250)
#define RED_DAYLIGHT        (0x0b81)
#define BLUE_DAYLIGHT       (0x1832)
#define RED_CLOUDY          (0x0a4f)
#define BLUE_CLOUDY         (0x1bc5)
#define RED_SHADE           (0x0a4f)
#define BLUE_SHADE          (0x1bc5)

#endif /* __DRIVERS_VIDEO_ISX019_REG_H */
