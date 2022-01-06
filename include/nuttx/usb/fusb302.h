/****************************************************************************
 * include/nuttx/usb/fusb302.h
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

#ifndef __INCLUDE_NUTTX_USB_FUSB302_H
#define __INCLUDE_NUTTX_USB_FUSB302_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default FUSB302 I2C ID */
#define FUSB302_I2C_ADDR      0x22

/* IOCTL Commands ***********************************************************/

#define USBCIOC_READ_DEVID        _USBCIOC(0x0001)  /* Arg: uint8_t* pointer */
#define USBCIOC_SETUP             _USBCIOC(0x0002)  /* Arg: uint8_t* pointer */
#define USBCIOC_SET_MODE          _USBCIOC(0x0003)  /* Arg: uint8_t value */
//#define USBCIOC_SET_STATE       _USBCIOC(0x0004)  /* Arg: uint8_t value */
#define USBCIOC_READ_STATUS       _USBCIOC(0x0005)  /* Arg: uint8_t* pointer*/
#define USBCIOC_READ_DEVTYPE      _USBCIOC(0x0006)  /* Arg: uint8_t* pointer*/
#define USBCIOC_RESET             _USBCIOC(0x0007)  /* Arg: None */

/* register bits and masks and modes */

/* Device ID - 0x01 */
#define FUSB302_DEV_ID_DEVICE_SHIFT     (4) /* Bits6..4, Device ID */
#define FUSB302_DEV_ID_DEVICE_MASK      (0x7 << FUSB302_DEV_ID_DEVICE_SHIFT)
#define FUSB302_DEV_ID_PRODUCT_SHIFT    (2) /* Bits3..2, Product ID */
#define FUSB302_DEV_ID_PRODUCT_MASK     (0x3 << FUSB302_DEV_ID_PRODUCT_SHIFT)
#define FUSB302_DEV_ID_REVISION_SHIFT   (0) /* Bits1..0, Revision Code */
#define FUSB302_DEV_ID_REVISION_MASK    (0x3 << FUSB302_DEV_ID_REVISION_SHIFT)


/* convert revision code to ASCII character */
#define FUSB302_REVISION(n)    ((((uint8_t)(n) & FUSB302_DEV_ID_REVISION_MASK) \
                                >> FUSB302_DEV_ID_REVISION_SHIFT) + 65)
#define FUSB302_DEVICE(n)      (((uint8_t)(n) & FUSB302_DEV_ID_DEVICE_MASK) \
                                >> FUSB302_DEV_ID_DEVICE_SHIFT)
#define FUSB302_PRODUCT(n)     (((uint8_t)(n) & FUSB302_DEV_ID_PRODUCT_MASK) \
                                >> FUSB302_DEV_ID_PRODUCT_SHIFT)

/* Switches0 - 0x02 */
#define SWITCHES0_PULLDOWN_CC1        (1 << 0)
#define SWITCHES0_PULLDOWN_CC2        (1 << 1)
#define SWITCHES0_MEASURE_CC1         (1 << 2)
#define SWITCHES0_MEASURE_CC2         (1 << 3)
#define SWITCHES0_VCONN_CC1           (1 << 4)
#define SWITCHES0_VCONN_CC2           (1 << 5)
#define SWITCHES0_PULLUP_CC1          (1 << 6)   
#define SWITCHES0_PULLUP_CC2          (1 << 7)

/* Switches1 */
#define SWITCHES1_TXCC1_MASK          (1 << 0)
#define SWITCHES1_TXCC2_MASK          (1 << 1)
#define SWITCHES1_AUTO_CRC_MASK       (1 << 2)
#define SWITCHES1_DATAROLE_MASK       (1 << 3)
#define SWITCHES1_SPECREV_SHIFT       (1 << 5) /* Bits 6:5 Specrole */
#define SWITCHES1_SPECREV_MASK        (0x03 << 6)
#define SWITCHES1_SPECREV(n)          ((uint8_t)(n) << SWITCHES1_SPECREV_SHIFT)
#define SWITCHES1_POWERROLE_MASK      (1 << 6)
        

/* Measure - 0x04 */
#define MEASURE_MDAC_SHIFT            (0)        /* Bits 5:0 MDAC */
#define MEASURE_MDAC_MASK             (0b111111 << MEASURE_MDAC_SHIFT)
#define MEASURE_MDAC(n)               (((uint8_t)(n) << MEASURE_MDAC_SHIFT
#define MEASURE_VBUS_BY_MDAC          (1 << 6) 
#define SET_MDAC(n)                   ((uint8_t)(n) << MEASURE_MDAC_SHIFT)

/* Slice - 0x05 */
#define MEASURE_SDAC_SHIFT            (0)      /* Bits 5:0 SDAC */
#define MEASURE_SDAC_MASK             (0b111111 < MEASURE_SDAC_SHIFT)
#define MEASURE_SDAC(n)               ((uint8_t)(n) << MEASURE_SDAC_SHIFT)
#define MEASURE_SDAC_HYS_SHIFT        (6)     /* Bits 7:6 SDAC hysteris */                                        
#define MEASURE_SDAC_HYS_MASK         (0b11 << MEASURE_SDAC_HYS_SHIFT)
#define SDAC_HYS_VAL(n)               ((uint8)t)(n) << MEASURE_SDAC_HYS_SHIFT)
                                        
/* Control0 - 0x06 */
#define CONTROL0_TX_START_MASK        (1 << 0)
#define CONTROL0_AUTO_PRE_SHIFT       (1) 
#define CONTROL0_HOST_CUR_SHIFT       (2)     /* Bits 3:2 Host Current mode */
#define CONTROL0_HOST_CUR_MASK        (0b11 << CONTROL0_HOST_CUR_SHIFT)
#define HOST_CURRENT_NONE             0b00 /* no current */
#define HOST_CURRENT_80UA             0b01 /* default USB power */
#define HOST_CURRENT_180UA            0b10 /* medium power, 1.5A */
#define HOST_CURRENT_330UA            0b11 /* high power, 3A */
#define HOST_CURRENT(n)               ((uint8_t)(n) << CONTROL0_HOST_CUR_SHIFT)
#define CONTROL0_INT_MASK             (1 << 5) 
#define CONTROL0_TX_FLUSH             (1 << 6) 

/* Control1 - 0x07 */
#define CONTROL1_ENSOP1_MASK          (1 << 0)
#define CONTROL1_ENSOP2_MASK          (1 << 1)
#define CONTROL1_RXFLUSH_MASK         (1 << 2)
#define CONTROL1_BIST_MODE2_MASK      (1 << 4)
#define CONTROL1_ENSOP1DB_MASK        (1 << 5)
#define CONTROL1_ENSOP2DB_MASK        (1 << 6)

/* Control2 - 0x08 */
#define CONTROL2_TOGGLE               (1 << 0)
#define CONTROL2_MODE_SHIFT           (1)   /* Bits 2:1 */
#define CONTROL2_MODE_MASK            (0b11 << CONTROL2_MODE_SHIFT)
#define SRC_POLLING                   (0b11)
#define SNK_POLLING                   (0b10)
#define DRP_POLLING                   (0b01)
#define SET_POLL_MODE(n)              ((uint8_t)(n) << CONTROL2_MODE_SHIFT)
#define CONTROL2_WAKE_EN              (1 << 3)
#define CONTROL2_TOG_RD_ONLY          (1 << 5)
#define CONTROL2_TOG_SAVE_PWR_SHIFT   (6) /* Bits 7:6 */
#define CONTROL2_TOG_SAVE_PWR_MASK    (0b11 << CONTROL2_TOG_SAVE_PWR_SHIFT 
#define WAIT_NONE                     (0b00)
#define WAIT_40MS                     (0b01)
#define WAIT_80MS                     (0b10)
#define WAIT_160MS                    (0b11)
#define SET_WAIT_MODE(n)              (((uint8_t)(n) << CONTROL2_TOG_SAVE_PWR_SHIFT)
                                        
/* Control3 - 0x09 */
#define CONTROL3_AUTO_RETRY           (0) 
#define CONTROL3_N_RETRIES_SHIFT      (1)   /* Bits 2:1 */
#define CONTROL3_N_RETRIES_MASK       (0b11 << CONTROL3_N_RETRIES_SHIFT)
#define NO_RETRIES                    (0b00)
#define ONE_RETRIES                   (0b01)
#define TWO_RETRIES                   (0b10)
#define THREE_RETRIES                 (0b11)
#define SET_NUM_RETRIES(n)            ((uint8_t(n) << CONTROL3_N_RETRIES_SHIFT)
#define CONTROL3_AUTO_SOFTRESET       (1 << 3)
#define CONTROL3_AUTO_HARDRESET       (1 << 4) 
#define CONTROL3_SEND_HARDRESET       (1 << 6) 



/* Mask - 0x0A */
#define MASK_BC_LVL                   (1 << 0)
#define MASK_COLLISION                (1 << 1)
#define MASK_WAKE                     (1 << 2)
#define MASK_ALERT                    (1 << 3)
#define MASK_CRC_CHK                  (1 << 4)
#define MASK_COMP_CHNG                (1 << 5)
#define MASK_ACTIVITY                 (1 << 6)
#define MASK_VBUS_OK                  (1 << 7)

/* Power 0x0B  */
#define POWER_PWR_SHIFT               (0)      /* Bits 3:0 */
#define POWER_PWR_MASK                (0b1111 << POWER_PWR_SHIFT)
#define POWER_MODE_ALL                0b1111
#define POWER_MODE_BANDGAP_AND_WAKE   0b0001
#define POWER_MODE_RX_AND_IREF        0b0010
#define POWER_MODE_MEASURE_BLOCK      0b0100
#define POWER_MODE_MEASURE_INT_OSC    0b1000
#define POWER_MODE(n)                 ((uint8_t)(n) << POWER_PWR_SHIFT)

/* Reset 0x0C */
#define RESET_SW_RESET                (1 << 0)
#define RESET_PD_RESET                (1 << 1)

/* OCPreg = 0x0D */
#define OCPREG_CUR_SHIFT              (0)     /* Bits 2:0 */
#define OCPREG_CUR_MASK               (0b111 << OCPREG_CUR_SHIFT)
#define X1_MAX_RANGE_DIV8             (0b000)
#define X2_MAX_RANGE_DIV8             (0b001)
#define X3_MAX_RANGE_DIV8             (0b010)
#define X4_MAX_RANGE_DIV8             (0b011)
#define X5_MAX_RANGE_DIV8             (0b100)
#define X6_MAX_RANGE_DIV8             (0b101)
#define X7_MAX_RANGE_D                (0b110)
#define MAX_RANGE                     (0b111)
#define SET_OCP_RANGE(n)              ((uint8_t)(n) << OCPREG_CUR_SHIFT)
#define OCPREF_OCP_RANGE              (1 << 3)


/* MaskA - 0x0E */
#define MASKA_HARDRST                 (1 << 0)
#define MASKA_SOFTRST                 (1 << 1)
#define MASKA_TXSENT                  (1 << 2)
#define MASKA_HARDSENT                (1 << 3)
#define MASKA_RETRYFAIL               (1 << 4)
#define MASKA_SOFTFAIL                (1 << 5)
#define MASKA_TOGDONE                 (1 << 6)
#define MASKA_OCP_TEMP                (1 << 7)

/* MaskB - 0x0F */   
#define MASKB_GCRCSENT                (1 << 0)

/* Control4 - 0x10 */   
#define CONTROL4_TOG_USRC_EXIT        (1 << 0)

/* Status0a - 0x3c */
#define STATUS0A_HARDRST              (1 << 0)
#define STATUS0A_SOFTRST              (1 << 1)
#define STATUS0A_POWER_SHIFT          (2)      /* Bits 3:2 */
#define STATUS0A_POWER_MASK           (0b11 << STATUS0A_POWER_SHIFT)
#define STATUS0A_RETRYFAIL            (1 << 4)
#define STATUS0A_SOFTFAIL             (1 << 5)

/* Status1a - 0x3D */
#define STATUS1A_RXSOP                (1 << 0)
#define STATUS1A_RXSOP1DB             (1 << 1)
#define STATUS1A_RXSOP2DB             (1 << 2)
#define STATUS1A_TOGGS_SHIFT          (3)       /* Bits 5:3 */
#define STATUS1A_TOGGS_MASK           (0b111 << STATUS1A_TOGGS_SHIFT)
#define TOGGS_RUNNING                 0b000
#define TOGGS_SRC_CC1                 0b001
#define TOGGS_SRC_CC2                 0b010
#define TOGGS_SNK_CC1                 0b101
#define TOGGS_SNK_CC2                 0b110
#define TOGGS_AUDIO_ACCCESSORY        0b111
#define DECODE_TOGGS(n)               ((uint8_t)(n) >> STATUS1A_TOGGS_SHIFT)

/* InterruptA - 0x3E */
#define INTERRUPTA_M_HARDRST          (1 << 0)
#define INTERRUPTA_M_SOFTRST          (1 << 1)
#define INTERRUPTA_M_TXSENT           (1 << 2)
#define INTERRUPTA_M_HARDSENT         (1 << 3)
#define INTERRUPTA_M_RETRYFAIL        (1 << 4)
#define INTERRUPTA_M_SOFTFAIL         (1 << 5)
#define INTERRUPTA_M_TOGDONE          (1 << 6)
#define INTERRUPTA_M_OCP_TEMP         (1 << 7)

/* InterruptB - 0x3F */
#define INTERRUPTB_I_GCRCSENT         (1 << 0)

/* Status0 - 0x40 */
#define STATUS0_BC_LVL_SHIFT          (0)      /* Bits 1:0 */
#define STATUS0_BC_LVL_MASK           (0b11 << STATUS0_BC_LVL_SHIFT)
#define STATUS0_WAKE                  (1 << 2)
#define STATUS0_ALERT                 (1 << 3)
#define STATUS0_CRC_OK                (1 << 4)
#define STATUS0_COMP                  (1 << 5)
#define STATUS0_ACTIVITY              (1 << 6)
#define STATUS0_VBUS_OK               (1 << 7)

/* Status1 - 0x41 */
#define STATUS1_OCP                   (1 << 0)
#define STATUS1_OVRTEMP               (1 << 1)
#define STATUS1_TX_FULL               (1 << 2)
#define STATUS1_TX_EMPTY              (1 << 3)
#define STATUS1_RX_FULL               (1 << 4)
#define STATUS1_RX_EMPTY              (1 << 5)
#define STATUS1A_RXSOP1               (1 << 6)
#define STATUS1A_RXSOP2               (1 << 7)

/* Interrupt - 0x42 */
#define INTERRUPT_BC_LVL_MASK         (1 << 0)
#define INTERRUPT_COLLISION_          (1 << 1)
#define INTERRUPT_WAKE                (1 << 2)
#define INTERRUPT_ALERT               (1 << 3)
#define INTERRUPT_CRC_OK              (1 << 4)
#define INTERRUPT_COMP_CHNG           (1 << 5)
#define INTERRUPT_ACTIVITY            (1 << 6)
#define INTERRUPT_VBUS_OK             (1 << 7)

/* FIFOs - 0x43 */
#define FIFOS_TX_RX_TOKEN             (1 << 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/
enum fusb302_mode_e
{
  MODE_SRC_POLL_MAN  =  0,
  MODE_SRC_POLL_AUTO,
  MODE_SNK_POLL_MAN,
  MODE_SNK_POLL_AUTO,
  MODE_DRP_POLL_MAN,
  MODE_DRP_POLL_AUTO,
};


enum fusb302_reg_address_e
{
  FUSB302_DEV_ID_REG = 0x01,
  FUSB302_SWITCHES0_REG,
  FUSB302_SWITCHES1_REG,
  FUSB302_MEASURE_REG,
  FUSB302_SLICE_REG,
  FUSB302_CONTROL0_REG,
  FUSB302_CONTROL1_REG,
  FUSB302_CONTROL2_REG,
  FUSB302_CONTROL3_REG,
  FUSB302_MASK_REG,
  FUSB302_POWER_REG,
  FUSB302_RESET_REG,
  FUSB302_OCPREG_REG,
  FUSB302_MASKA_REG,
  FUSB302_MASKB_REG,
  FUSB302_CONTROL4_REG,
  FUSB302_STATUS0A_REG=0x3c,
  FUSB302_STATUS1A_REG,
  FUSB302_INTERRUPTA_REG,
  FUSB302_INTERRUPTB_REG,
  FUSB302_STATUS0_REG,
  FUSB302_STATUS1_REG,
  FUSB302_INTERRUPT_REG,
  FUSB302_FIFOS_REG,
};
#if 0
/* Device ID - 0x01 */
/* this is for the FUSB302T */
enum fusb302T_devid_mask_e
{

};

enum fusb302_devid_mask_e
{
  DEV_ID_FUSB302_VERSION_MASK   = 0x70,
  DEV_ID_FUSB302_REVISION_MASK  = 0x0F,
};

/* Switches0 - 0x02 */
enum fusb302_switches0_e
{
  SWITCHES0_PU_EN2    = (1 << 7),
  SWITCHES0_PU_EN1    = (1 << 6),
  SWITCHES0_VCONN_CC2 = (1 << 5),
  SWITCHES0_VCONN_CC1 = (1 << 4),
  SWITCHES0_MEAS_CC2  = (1 << 3),
  SWITCHES0_MEAS_CC1  = (1 << 2),
  SWITCHES0_PDWN2     = (1 << 1),
  SWITCHES0_PDWN1     = (1 << 0),
};

/* Switches1 - 0x03 */
enum fusb302_switches1_e
{
  SWITCHES1_POWERROLE = (1 << 7),
  SWITCHES1_SPECREV   = (1 << 5),
  SWITCHES1_DATAROLE  = (1 << 4),
  SWITCHES1_AUTO_CRC  = (1 << 2),
  SWITCHES1_TXCC2     = (1 << 1),
  SWITCHES1_TXCC1     = (1 << 0),
};

/* Measure - 0x04 */
enum fusb302_measure_e
{
  MEASURE_VBUS        = (1 << 6),
  MEASURE_MDAC        = (1 << 0),
};

/* Slice - 0x05 */
enum fusb302_slice_e
{
  SLICE_SDAC_HYS      = (1 << 6),
  SLICE_SDAC          = (1 << 0),
};

/* Control0 - 0x06 */
enum fusb302_control0_e
{
  CONTROL0_TX_FLUSH   = (1 << 6),
  CONTROL0_INT_MASK   = (1 << 5),
  CONTROL0_HOST_CUR   = (1 << 2),
  CONTROL0_AUTO_PRE   = (1 << 1),
  CONTROL0_TX_START   = (1 << 0),
};

/* Control1 - 0x07 */
enum fusb302_control1_e
{
  CONTROL1_ENSOP2DB   = (1 << 6),
  CONTROL1_ENSOP1DB   = (1 << 5),
  CONTROL1_BIST_MODE2 = (1 << 4),
  CONTROL1_RX_FLUSH   = (1 << 2),
  CONTROL1_ENSOP2     = (1 << 1),
  CONTROL1_ENSOP1     = (1 << 0),
};

/* Control2 - 0x08 */
enum fusb302_control2_e
{
  CONTROL2_TOG_SAVE_PWR = (1 << 6),
  CONTROL2_TOG_RD_ONLY  = (1 << 5),
  CONTROL2_WAKE_EN      = (1 << 3),
  CONTROL2_MODE         = (1 << 1),
  CONTROL2_TOGGLE       = (1 << 0),
};

/* Control3 - 0x09 */
enum fusb302_control3_e
{
  CONTROL2_SEND_HARD_RESET  = (1 << 6),
  CONTROL2_BIST_TMODE       = (1 << 5),
  CONTROL2_AUTO_HARDREST    = (1 << 4),
  CONTROL2_AUTO_SOFTRESET   = (1 << 3),
  CONTROL2_N_RETRIES        = (1 << 1),
  CONTROL2_AUTO_RETRY       = (1 << 0),
};

/* Mask - 0x0A */
enum fusb302_mask_e
{
  MASK_VBUSOK       = (1 << 7),
  MASK_M_ACTIVITY   = (1 << 6),
  MASK_M_COMP_CHNG  = (1 << 5),
  MASK_M_CRC_CHK    = (1 << 4),
  MASK_M_ALERT      = (1 << 3),
  MASK_M_WAKE       = (1 << 2),
  MASK_M_COLLISION  = (1 << 1),
  MASK_M_BC_LVL     = (1 << 0),
};

/* Power - 0x0B */
enum fusb302_power_e
{
  POWER_PWR         = (1 << 0),
};

/* Reset - 0x0C */
enum fusb302_reset_e
{
  RESET_PD_RESET    = (1 << 1),
  RESET_SW_RES      = (1 << 0),
};

/* Ocpreg = 0x0D */
enum fusb302_ocpreg_e
{
  OCPREGC_OCP_RANGE = (1 << 3),
  OCPREGC_OCP_CUR   = (1 << 0),
};

/* MaskA - 0x0E */
enum fusb302_maska_e
{
    MASKA_M_OCP_TEMP  = (1 << 7),
    MASKA_M_TOGDONE   = (1 << 6),
    MASKA_M_SOFTFAIL  = (1 << 5),
    MASKA_M_RETRYFAIL = (1 << 4),
    MASKA_M_HARDSENT  = (1 << 3),
    MASKA_M_TXSENT    = (1 << 2),
    MASKA_M_SOFTRST   = (1 << 1),
    MASKA_M_HARDRST   = (1 << 0),
};

/* MaskB - 0x0F */
enum fusb302_maskb_e
{
  MASKB_M_GCRCSENT    = (1 << 0),
};

/* Control4 - 0x10 */
enum fusb302_control4_e
{
  CONTROL4_TOG_EXIT_AUD = (1 << 0),
};

/* Status0a - 0x3C */
enum fusb302_status0a_e
{
  STATUS0A_SOFTFAIL   = (1 << 5),
  STATUS0A_RETRYFAIL  = (1 << 4),
  STATUS0A_POWER      = (1 << 2),
  STATUS0A_SOFTRST    = (1 << 1),
  STATUS0A_HARDRST    = (1 << 0),
};

/* Status1A - 0x3D*/
enum fusb302_status1a_e
{
  STATUS1A_TOGSS      = (1 << 3),
  STATUS1A_RXSOP2DB   = (1 << 2),
  STATUS1A_RXSOP1DB   = (1 << 1),
  STATUS1A_RXSOP      = (1 << 0),
};

/* InterruptA - 0x3E */
enum fusb302_interrupta_e
{
  INTERRUPTA_I_OCP_TEMP   = (1 << 7),
  INTERRUPTA_I_TOGDONE    = (1 << 6),
  INTERRUPTA_I_SOFTFAIL   = (1 << 5),
  INTERRUPTA_I_RETRYFAIL  = (1 << 4),
  INTERRUPTA_I_HARDSENT   = (1 << 3),
  INTERRUPTA_I_TXSENT     = (1 << 2),
  INTERRUPTA_I_SOFTRST    = (1 << 1),
  INTERRUPTA_I_HARDRST    = (1 << 0),
};

/* Interruptb - 0x3F */
enum fusb302_interruptB_e
{
  INTERRUPTB_I_GCRCSENT   = (1 << 0),
};

/* Status0 - 0x40 */
enum fusb302_status0_e
{
  STATUS0_VBUSOK          = (1 << 7),
  STATUS0_ACTIVITY        = (1 << 6),
  STATUS0_COMP            = (1 << 5),
  STATUS0_CRC_CHK         = (1 << 4),
  STATUS0_ALERT           = (1 << 3),
  STATUS0_WAKE            = (1 << 2),
  STATUS0_BC_LVL          = (1 << 0),
};

/* Status1 - 0x41 */
enum fusb302_status1_e
{
  STATUS1_RXSOP2          = (1 << 7),
  STATUS1_RXSOP1          = (1 << 6),
  STATUS1_RX_EMPTY        = (1 << 5),
  STATUS1_RX_FULL         = (1 << 4),
  STATUS1_TX_EMPTY        = (1 << 3),
  STATUS1_TX_FULL         = (1 << 2),
  STATUS1_OVRTEMP         = (1 << 1),
  STATUS1_OCP             = (1 << 0),
};
  
/* Interrupt - 0x42 */
enum fusb302_interrupt_e
{
  INTERRUPT_I_VBUSOK          = (1 << 7),
  INTERRUPT_I_ACTIVITY        = (1 << 6),
  INTERRUPT_I_COMP_CHNG       = (1 << 5),
  INTERRUPT_I_CRC_CHK         = (1 << 4),
  INTERRUPT_I_ALERT           = (1 << 3),
  INTERRUPT_I_WAKE            = (1 << 2),
  INTERRUPT_I_COLLISION       = (1 << 1),
  INTERRUPT_I_BC_LVL          = (1 << 0),
};

/* Fifos = 0x43 */
enum fusb302_fifos_e
{
  FIFOS_TX_RX_TOKEN           = (1 << 0),
};
#endif
struct fusb302_result_s
{
  uint8_t status0;
  uint8_t status1;
  uint8_t status0a;
  uint8_t status1a;
  uint8_t interrupt;
  uint8_t dev_type;
  bool int_pending;
};

struct fusb302_setup_s
{
  uint8_t drp_toggle_timing;
  uint8_t host_current;
  uint8_t int_mask;
  bool global_int_mask;
};

struct fusb302_config_s
{
  /* Device characterization */

  int irq;

  CODE int  (*irq_attach)(FAR struct fusb302_config_s *state, xcpt_t isr,
                          FAR void *arg);
  CODE void (*irq_enable)(FAR struct fusb302_config_s *state, bool enable);
  CODE void (*irq_clear)(FAR struct fusb302_config_s *state);

  FAR struct i2c_master_s *i2c;
  uint8_t i2c_addr;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: fusb302_register
 *
 * Description:
 *   Register the FUSB302 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/usbc0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             FUSB302
 *   addr    - The I2C address of the FUSB302.
 *             The I2C address of the FUSB302 is 0x22.
 *   config  - Pointer to FUSB302 configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int fusb302_register(FAR const char *devpath, FAR struct fusb302_config_s *config);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_FUSB302_H */
