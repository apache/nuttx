/****************************************************************************
 * drivers/usbmisc/fusb302.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <poll.h>
#include <debug.h>

#include <nuttx/compiler.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/usb/fusb302.h>
#include <sys/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef FUSB302_I2C_FREQUENCY
#  define FUSB302_I2C_FREQUENCY 1000000
#endif

#ifdef CONFIG_DEBUG_USB_ERROR
#  define fusb302_err(x, ...)        _err(x, ##__VA_ARGS__)
#else
#  define fusb302_err(x, ...)        uerr(x, ##__VA_ARGS__)
#endif

#if defined(CONFIG_DEBUG_USB_INFO) || defined (CONFIG_DEBUG_FUSB302_REG)
#  define fusb302_info(x, ...)       _info(x, ##__VA_ARGS__)
#else
#  define fusb302_info(x, ...)       uinfo(x, ##__VA_ARGS__)
#endif

#define FUSB302_I2C_RETRIES  10

/* Switches0 - 0x02 */

#define SWITCHES0_PULLDOWN_CC1        (1 << 0)
#define SWITCHES0_PULLDOWN_CC2        (1 << 1)
#define SWITCHES0_PULLDOWN_SHIFT      (0)
#define SWITCHES0_PULLDOWN_MASK       (3 << SWITCHES0_PULLDOWN_SHIFT)
#define SWITCHES0_MEASURE_CC1         (1 << 2)
#define SWITCHES0_MEASURE_CC2         (1 << 3)
#define SWITCHES0_MEASURE_SHIFT       (3)
#define SWITCHES0_MEASURE_MASK        (3 << SWITCHES0_MEASURE_SHIFT)
#define SWITCHES0_VCONN_CC1           (1 << 4)
#define SWITCHES0_VCONN_CC2           (1 << 5)
#define SWITCHES0_VCONN_SHIFT         (4)
#define SWITCHES0_VCONN_MASK          (3 << SWITCHES0_VCONN_SHIFT)
#define SWITCHES0_PULLUP_CC1          (1 << 6)   
#define SWITCHES0_PULLUP_CC2          (1 << 7)
#define SWITCHES0_PULLUP_SHIFT        (6)
#define SWITCHES0_PULLUP_MASK         (3 << SWITCHES0_PULLUP_SHIFT)

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
#define MEASURE_MDAC(n)               ((uint8_t)(n)) << MEASURE_MDAC_SHIFT
#define MEASURE_VBUS_BY_MDAC          (1 << 6) 
#define SET_MDAC(n)                   ((uint8_t)(n)) << MEASURE_MDAC_SHIFT)

/* Slice - 0x05 */

#define MEASURE_SDAC_SHIFT            (0)      /* Bits 5:0 SDAC */
#define MEASURE_SDAC_MASK             (0b111111 < MEASURE_SDAC_SHIFT)
#define MEASURE_SDAC(n)               ((uint8_t)(n)) << MEASURE_SDAC_SHIFT)
#define MEASURE_SDAC_HYS_SHIFT        (6)     /* Bits 7:6 SDAC hysteris */                                        
#define MEASURE_SDAC_HYS_MASK         (0b11 << MEASURE_SDAC_HYS_SHIFT)
#define SDAC_HYS_VAL(n)               ((uint8)t)(n)) << MEASURE_SDAC_HYS_SHIFT)

/* Control0 - 0x06 */

#define CONTROL0_TX_START_MASK        (1 << 0)
#define CONTROL0_AUTO_PRE_SHIFT       (1) 
#define CONTROL0_HOST_CUR_SHIFT       (2) /* Bits 3:2 Host Current mode */
#define CONTROL0_HOST_CUR_MASK        (3 << CONTROL0_HOST_CUR_SHIFT)
#define HOST_CURRENT_NONE             (0) /* no current */
#define HOST_CURRENT_80UA             (1) /* default USB power */
#define HOST_CURRENT_180UA            (2) /* medium power, 1.5A */
#define HOST_CURRENT_330UA            (3) /* high power, 3A */
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
#define MASK_ALL                      (0xff)
#define MASK_FOR_DISCONNECT           (MASK_VBUS_OK | MASK_COMP_CHNG)

/* Power 0x0B  */

#define POWER_PWR_SHIFT               (0)      /* Bits 3:0 */
#define POWER_PWR_MASK                (15 << POWER_PWR_SHIFT)
#define POWER_MODE_ALL                (15)
#define POWER_MODE_TOGGLING           (7)
#define POWER_MODE_BANDGAP_AND_WAKE   (1)
#define POWER_MODE_RX_AND_IREF        (2)
#define POWER_MODE_MEASURE_BLOCK      (4)
#define POWER_MODE_MEASURE_INT_OSC    (8)
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
#define STATUS1A_TOGGS_MASK           (7 << STATUS1A_TOGGS_SHIFT)
#define TOGGS_RUNNING                 (0)
#define TOGGS_SRC_CC1                 (1)
#define TOGGS_SRC_CC2                 (2)
#define TOGGS_SNK_CC1                 (5)
#define TOGGS_SNK_CC2                 (6)
#define TOGGS_AUDIO_ACCESSORY         (7)
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

#define INTERRUPT_BC_LVL              (1 << 0)
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
 * Private Data Types
 ****************************************************************************/

enum fusb302_state_e
{
  RESET = 0,
  WAITING_FOR_DRP_TOGG_I,
  WAITING_FOR_SNK_TOGG_I,
  WAITING_FOR_SRC_TOGG_I,
  WAITING_FOR_VBUS,
  WAITING_FOR_COMP_CHNG,
  WAITING_FOR_SRC_DISCONNECT,
  WAITING_FOR_SNK_DISCONNECT,
  WAITING_FOR_HOST_INTERVENTION,
  NOT_DEFINED,
};

enum cc_pd_e
{
  CC1_PULLDOWN = SWITCHES0_PULLDOWN_CC1,
  CC2_PULLDOWN = SWITCHES0_PULLDOWN_CC2,
};

enum cc_pu_e
{
  CC1_PULLUP = SWITCHES0_PULLUP_CC1,
  CC2_PULLUP = SWITCHES0_PULLUP_CC2,
};

enum cc_vconn_e
{
  CC1_VCONN = SWITCHES0_VCONN_CC1,
  CC2_VCONN = SWITCHES0_VCONN_CC2,
};

enum cc_meas_e
{
  CC1_MEASURE = SWITCHES0_MEASURE_CC1,
  CC2_MEASURE = SWITCHES0_MEASURE_CC2,
};

enum src_current_e
{
  SRC_CURRENT_DEFAULT,
  SRC_CURRENT_MEDIUM,
  SRC_CURRENT_HIGH,
};

struct fusb302_dev_s
{
  FAR struct pollfd           *fds[CONFIG_FUSB302_NPOLLWAITERS];
  uint8_t                     i2c_addr;       /* I2C address */
  volatile bool               int_pending;    /* Interrupt pending */
  mutex_t                     devlock;        /* Manages exclusive access */
  FAR struct fusb302_config_s *config;        /* Platform specific config */
  struct work_s               work;           /* Supports interrupt handling */
  enum fusb302_state_e        state;          /* worker state machine */
  enum fusb_connect_status_e  connect_status; /* USB device connection state */
};

static const uint8_t src_mdac_val[] =
{
  [SRC_CURRENT_DEFAULT] = 0x25, /* 1600mV */
  [SRC_CURRENT_MEDIUM]  = 0x25, /* 1600mV */
  [SRC_CURRENT_HIGH]    = 0x3d, /* 2600mV */
};

struct fusb302_result_s
{
  enum fusb_connect_status_e connected_state;
  enum fusb302_state_e state;
};

struct fusb302_setup_s
{
  uint8_t drp_toggle_timing;
  uint8_t host_current;
  uint8_t int_mask;
  bool global_int_mask;
};

enum fusb302_reg_address_e
{
  FUSB302_DEV_ID_REG      = 0x01,
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
  FUSB302_STATUS0A_REG    = 0x3c,
  FUSB302_STATUS1A_REG,
  FUSB302_INTERRUPTA_REG,
  FUSB302_INTERRUPTB_REG,
  FUSB302_STATUS0_REG,
  FUSB302_STATUS1_REG,
  FUSB302_INTERRUPT_REG,
  FUSB302_FIFOS_REG,
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

static int     fusb302_open(FAR struct file *filep);
static int     fusb302_close(FAR struct file *filep);
static ssize_t fusb302_read(FAR struct file *, FAR char *, size_t);
static ssize_t fusb302_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int     fusb302_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);
static int     fusb302_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup);
static int     fusb302_putreg(FAR struct fusb302_dev_s *priv,
                              uint8_t regaddr, uint8_t regval);
static int     fusb302_getreg(FAR struct fusb302_dev_s *priv, uint8_t reg);
#ifdef CONFIG_DEBUG_FUSB302_REG
static void    fusb302_dump_registers(FAR struct fusb302_dev_s *priv,
                                      FAR const char *msg);
#else
#  define fusb302_dump_registers(priv, msg);
#endif
static int fusb302_clear_interrupts(FAR struct fusb302_dev_s * priv);
static int     fusb302_reset(FAR struct fusb302_dev_s *priv);
static int     fusb302_set_mode(FAR struct fusb302_dev_s *priv,
                                enum fusb302_mode_e mode);
static int     fusb302_int_handler(int irq, FAR void *context,
                                   FAR void *arg);
static void    fusb302_worker(FAR void *arg);
static int     fusb302_schedule(FAR struct fusb302_dev_s *priv);
void           enableccpd(FAR struct fusb302_dev_s *priv,
                          enum cc_pd_e pulldown, bool enable);
void           enableccpu(FAR struct fusb302_dev_s *priv,
                          enum cc_pu_e pullup, bool enable);
void           enablevconn(FAR struct fusb302_dev_s *priv,
                           enum cc_vconn_e vconn, bool enable);
void           enableccmeas(FAR struct fusb302_dev_s *priv,
                            enum cc_meas_e pullup, bool enable);
void           set_switches(FAR struct fusb302_dev_s *priv,
                            uint8_t toggsval);
static int     set_int_mask(struct fusb302_dev_s *priv);
/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_fusb302ops =
{
  fusb302_open,  /* open */
  fusb302_close, /* close */
  fusb302_read,  /* read */
  fusb302_write, /* write */
  NULL,          /* seek */
  fusb302_ioctl, /* ioctl */
  NULL,          /* mmap */
  NULL,          /* truncate */
  fusb302_poll,  /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int set_int_mask(struct fusb302_dev_s *priv)
{
  int ret = OK;

  switch (priv->state)
    {
      case WAITING_FOR_DRP_TOGG_I:
      case WAITING_FOR_SNK_TOGG_I:
      case WAITING_FOR_SRC_TOGG_I:
        ret = fusb302_putreg(priv, FUSB302_MASK_REG,
                             (uint8_t) ~MASKA_TOGDONE);
        ret = fusb302_putreg(priv, FUSB302_MASKA_REG,
                             (uint8_t) MASKA_HARDRST);
        ret = fusb302_putreg(priv, FUSB302_MASKB_REG,
                             (uint8_t) MASKB_GCRCSENT);
        break;
      case WAITING_FOR_VBUS:
        ret = fusb302_putreg(priv, FUSB302_MASK_REG,
                             ((uint8_t) ~MASK_VBUS_OK) |
                             ((uint8_t) ~MASK_BC_LVL) |
                             ((uint8_t) ~MASK_COMP_CHNG));
        break;
      case WAITING_FOR_COMP_CHNG:
        ret = fusb302_putreg(priv, FUSB302_MASK_REG,
                             (uint8_t) ~MASK_COMP_CHNG);
        break;
      case WAITING_FOR_SRC_DISCONNECT:
        ret = fusb302_putreg(priv, FUSB302_MASK_REG,
                             (uint8_t) ~MASK_COMP_CHNG);
        break;
      case WAITING_FOR_SNK_DISCONNECT:
        ret = fusb302_putreg(priv, FUSB302_MASK_REG,
                             (uint8_t) ~MASK_VBUS_OK);
        break;
      case WAITING_FOR_HOST_INTERVENTION:
      case NOT_DEFINED:
      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: enableccpd
 *
 * Description:
 *   Enable/disable CCx pulldown resistors
 *
 * Input Parameters:
 *  priv - pointer to device structure
 *  pulldown - CC1 or CC2
 *  enable - enable or disable it
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void enableccpd(struct fusb302_dev_s *priv, enum cc_pd_e pulldown,
                bool enable)
{
  int regval;

  regval = fusb302_getreg(priv, FUSB302_SWITCHES0_REG);
  if (enable)
    {
      regval |= pulldown;
    }
  else
    {
      regval &= ~pulldown;
    }

  fusb302_putreg(priv, FUSB302_SWITCHES0_REG, regval);
}

/****************************************************************************
 * Name: enableccpu
 *
 * Description:
 *   Enable/disable CCx pulldown resistors
 *
 * Input Parameters:
 *  priv - pointer to device structure
 *  pulldown - CC1 or CC2
 *  enable - enable or disable it
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void enableccpu(struct fusb302_dev_s *priv, enum cc_pu_e pullup, bool enable)
{
  int regval;

  regval = fusb302_getreg(priv, FUSB302_SWITCHES0_REG);
  if (enable)
    {
      regval |= pullup;
    }
  else
    {
      regval &= ~pullup;
    }

  fusb302_putreg(priv, FUSB302_SWITCHES0_REG, regval);
}

/****************************************************************************
 * Name: enableccmeas
 *
 * Description:
 *   Enable/disable CC measurement
 *
 * Input Parameters:
 *  priv - pointer to device structure
 *  measure - CC1 or CC2
 *  enable - enable or disable it
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void enableccmeas(struct fusb302_dev_s *priv, enum cc_meas_e measure,
                  bool enable)
{
  int regval;

  regval = fusb302_getreg(priv, FUSB302_SWITCHES0_REG);
  if (enable)
    {
      regval |= measure;
    }
  else
    {
      regval &= ~measure;
    }

  fusb302_putreg(priv, FUSB302_SWITCHES0_REG, regval);
}

/****************************************************************************
 * Name: setmdac
 *
 * Description:
 *   Set MDAC measurement threshold
 *
 * Input Parameters:
 *  priv      - pointer to device structure
 *  threhold  - MDAC threshold value
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void setmdac(struct fusb302_dev_s *priv, enum src_current_e thresh)
{
  int regval;

  regval = fusb302_getreg(priv, FUSB302_MEASURE_REG);
  regval &= ~MEASURE_MDAC_MASK;
  regval |= MEASURE_MDAC(src_mdac_val[thresh]);

  fusb302_putreg(priv, FUSB302_MEASURE_REG, regval);
  usleep(150);
}

/****************************************************************************
 * Name: enablevconn
 *
 * Description:
 *   Enable/disable Vconn supply for host mode
 *
 * Input Parameters:
 *  priv - pointer to device structure
 *  vconn - CC1 or CC2
 *  enable - enable or disable it
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void enablevconn(FAR struct fusb302_dev_s *priv, enum cc_vconn_e vconn,
                 bool enable)
{
  int regval;

  regval = fusb302_getreg(priv, FUSB302_SWITCHES0_REG);
  if (enable)
    {
      regval |= vconn;
    }
  else
    {
      regval &= ~vconn;
    }

  fusb302_putreg(priv, FUSB302_SWITCHES0_REG, regval);
}

/****************************************************************************
 * Name: set_switches
 *
 * Description:
 *   Set up switches for required operational mode
 *
 * Input Parameters:
 *  priv     - pointer to device structure
 *  toggsval - the value read after toggling is done
 *  enable - enable or disable it
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void set_switches(FAR struct fusb302_dev_s *priv,
                                        uint8_t toggsval)
{
  /* what device detect type was seen */

  switch (toggsval)
    {
      case TOGGS_SRC_CC1:
        {
          enableccpd(priv, CC1_PULLDOWN, false);
          enableccpd(priv, CC2_PULLDOWN, false);
          enableccpu(priv, CC1_PULLUP, true);
          enableccpu(priv, CC2_PULLUP, false);
          enablevconn(priv, CC1_VCONN, false);
          enablevconn(priv, CC2_VCONN, true);
          enableccmeas(priv, CC1_MEASURE, true);
          enableccmeas(priv, CC2_MEASURE, false);
          setmdac(priv, SRC_CURRENT_DEFAULT);
        }
        break;
      case TOGGS_SRC_CC2:
        {
          enableccpd(priv, CC1_PULLDOWN, false);
          enableccpd(priv, CC2_PULLDOWN, false);
          enableccpu(priv, CC1_PULLUP, false);
          enableccpu(priv, CC2_PULLUP, true);
          enablevconn(priv, CC1_VCONN, true);
          enablevconn(priv, CC2_VCONN, false);
          enableccmeas(priv, CC1_MEASURE, false);
          enableccmeas(priv, CC2_MEASURE, true);
          setmdac(priv, SRC_CURRENT_DEFAULT);
        }
        break;
      case TOGGS_SNK_CC1:
        {
          enablevconn(priv, CC1_VCONN, false);
          enablevconn(priv, CC2_VCONN, false);
          enableccpu(priv, CC1_PULLUP, false);
          enableccpd(priv, CC2_PULLUP, false);
          enablevconn(priv, CC1_VCONN, false);
          enablevconn(priv, CC2_VCONN, false);
          enableccpd(priv, CC1_PULLDOWN, true);
          enableccpd(priv, CC2_PULLDOWN, false);
        }
        break;
      case TOGGS_SNK_CC2:
        {
          enablevconn(priv, CC1_VCONN, false);
          enablevconn(priv, CC2_VCONN, false);
          enableccpu(priv, CC1_PULLUP, false);
          enableccpu(priv, CC2_PULLUP, false);
          enablevconn(priv, CC1_VCONN, false);
          enablevconn(priv, CC2_VCONN, false);
          enableccpd(priv, CC1_PULLDOWN, false);
          enableccpd(priv, CC2_PULLDOWN, true);
        }
        break;
      case TOGGS_AUDIO_ACCESSORY:
          enableccpd(priv, CC1_PULLDOWN, false);
          enableccpd(priv, CC2_PULLDOWN, false);
          enableccpu(priv, CC1_PULLUP, true);
          enableccpu(priv, CC2_PULLUP, false);
          enablevconn(priv, CC1_VCONN, false);
          enablevconn(priv, CC2_VCONN, true);
          enableccmeas(priv, CC1_MEASURE, true);
          enableccmeas(priv, CC2_MEASURE, false);
          setmdac(priv, SRC_CURRENT_DEFAULT);
        break;
      default:
        fusb302_err("ERROR: invalid TOGGS VAL read:%d\n", toggsval);
        break;
    }
}

/****************************************************************************
 * Name: fusb302_worker
 *
 * Description:
 *   Worker task to deal with device state machine
 *
 * Input Parameters:
 *   arg - Pointer to device
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static void fusb302_worker(FAR void *arg)
{
  FAR struct fusb302_dev_s    *priv = (FAR struct fusb302_dev_s *)arg;
  FAR struct fusb302_config_s *config;
  int                         regval;
  int                         toggsval;
  int                         interrupt;
  int                         interrupta;

  DEBUGASSERT(priv != NULL);

  config = priv->config;
  DEBUGASSERT(config != NULL);

  nxmutex_lock(&priv->devlock);

  if (!priv->int_pending) /* just to double check we really should be here */
    {
      fusb302_err("ERROR: worker task run with no interrupt\n");
      goto error_exit;
    }
  else
    {
      /* do statemachine */

      interrupt = fusb302_getreg(priv, FUSB302_INTERRUPT_REG);
      interrupta = fusb302_getreg(priv, FUSB302_INTERRUPTA_REG);

      switch (priv->state)
      {
        case WAITING_FOR_DRP_TOGG_I:
          {
            if (interrupta & INTERRUPTA_M_TOGDONE)
              {
                /* What has connected? */

                regval = fusb302_getreg(priv, FUSB302_STATUS1A_REG);
                toggsval = DECODE_TOGGS(regval & STATUS1A_TOGGS_MASK);
                regval = fusb302_getreg(priv, FUSB302_STATUS0_REG);
                fusb302_info("INFO: toggs val=%x\n", toggsval);

                /* set everything up depending on what's been detected */

                set_switches(priv, toggsval);

                /* Power up Internal Oscillator as well now */

                fusb302_putreg(priv, FUSB302_POWER_REG,
                               POWER_MODE_ALL);

                fusb302_dump_registers(priv,
                                       "After DRP toggle detect handling");

                switch (toggsval)
                  {
                    case TOGGS_RUNNING:
                      break;
                    case TOGGS_SNK_CC1:
                    case TOGGS_SNK_CC2:
                      regval = fusb302_getreg(priv, FUSB302_STATUS0_REG);
                      if ((interrupt & INTERRUPT_VBUS_OK) ||
                          (regval & STATUS0_VBUS_OK)      ||
                          (regval & STATUS0_BC_LVL_MASK))
                        {
                          priv->state = WAITING_FOR_SNK_DISCONNECT;
                          priv->connect_status = SNK_DEVICE_CONNECTED;
                        }
                      else
                        {
                          priv->state = WAITING_FOR_VBUS;
                        }
                      break;
                    case TOGGS_SRC_CC1:
                    case TOGGS_SRC_CC2:
                    case TOGGS_AUDIO_ACCESSORY:
                      regval = fusb302_getreg(priv, FUSB302_STATUS0_REG);
                      priv->state = WAITING_FOR_SRC_DISCONNECT;
                      priv->connect_status = SRC_DEVICE_CONNECTED;
                      break;
                    default:
                      priv->connect_status = UNKNOWN_CONNECTED;
                      goto error_exit;
                      break;
                  }

                /* turn off toggling */

                regval = fusb302_getreg(priv, FUSB302_CONTROL2_REG);
                regval &= ~CONTROL2_TOGGLE;
                fusb302_putreg(priv, FUSB302_CONTROL2_REG, regval);

                /* Send new interrupt mask */

                set_int_mask(priv);
              }
          }
          break;
        case WAITING_FOR_VBUS:
          if (interrupt & INTERRUPT_VBUS_OK)
            {
              if (priv->connect_status == NOTHING_CONNECTED)
                {
                  priv->state = WAITING_FOR_SNK_DISCONNECT;
                  priv->connect_status = SNK_DEVICE_CONNECTED;
                }
              else if (priv->connect_status == SNK_DEVICE_CONNECTED)
                {
                  priv->state = WAITING_FOR_HOST_INTERVENTION;
                  priv->connect_status = SNK_DETACH_DETECTED;
                }
            }
          else
            {
              priv->state = NOT_DEFINED;
              goto error_exit;
            }

          /* Send new interrupt mask */

          set_int_mask(priv);
          break;
        case WAITING_FOR_COMP_CHNG:
          if (interrupt & INTERRUPT_COMP_CHNG)
            {
              if (priv->connect_status == NOTHING_CONNECTED)
                {
                  priv->state = WAITING_FOR_SRC_DISCONNECT;
                  priv->connect_status = SRC_DEVICE_CONNECTED;
                }
              else if (priv->connect_status == SRC_DEVICE_CONNECTED)
                {
                  priv->state = WAITING_FOR_HOST_INTERVENTION;
                  priv->connect_status = SRC_DETACH_DETECTED;
                }
            }
          else
            {
              priv->state = NOT_DEFINED;
              goto error_exit;
            }
          break;
        case WAITING_FOR_SNK_TOGG_I:
          {
            if (interrupta & INTERRUPTA_M_TOGDONE)
              {
                /* Check what has connected? */

                regval = fusb302_getreg(priv, FUSB302_STATUS1A_REG);
                toggsval = DECODE_TOGGS(regval & STATUS1A_TOGGS_MASK);
                regval = fusb302_getreg(priv, FUSB302_STATUS0_REG);
                fusb302_info("INFO: toggs val=%x\n", toggsval);

                /* set everything up depending on what's been detected */

                set_switches(priv, toggsval);

                /* Power up Internal Oscillator as well now */

                fusb302_putreg(priv, FUSB302_POWER_REG,
                               POWER_MODE_ALL);

                fusb302_dump_registers(priv,
                                       "After SNK toggle detect handling");

                switch (toggsval)
                  {
                    case TOGGS_RUNNING:
                      break;
                    case TOGGS_SNK_CC1:
                    case TOGGS_SNK_CC2:
                      regval = fusb302_getreg(priv, FUSB302_STATUS0_REG);
                      if ((interrupt & INTERRUPT_VBUS_OK) ||
                          (regval & STATUS0_VBUS_OK)      ||
                          (regval & STATUS0_BC_LVL_MASK))
                        {
                          priv->state = WAITING_FOR_SNK_DISCONNECT;
                          priv->connect_status = SNK_DEVICE_CONNECTED;
                        }
                      else
                        {
                          priv->state = WAITING_FOR_VBUS;
                        }
                      break;
                    case TOGGS_SRC_CC1:
                    case TOGGS_SRC_CC2:
                    case TOGGS_AUDIO_ACCESSORY:
                    default:
                      priv->connect_status = UNKNOWN_CONNECTED;
                      goto error_exit;
                      break;
                  }

                /* turn off toggling */

                regval = fusb302_getreg(priv, FUSB302_CONTROL2_REG);
                regval &= ~CONTROL2_TOGGLE;
                fusb302_putreg(priv, FUSB302_CONTROL2_REG, regval);

                /* Send new interrupt mask */

                set_int_mask(priv);
              }
          }
          break;
        case WAITING_FOR_SRC_TOGG_I:
          {
            if (interrupta & INTERRUPTA_M_TOGDONE)
              {
                /* Check what has connected? */

                regval = fusb302_getreg(priv, FUSB302_STATUS1A_REG);
                toggsval = DECODE_TOGGS(regval & STATUS1A_TOGGS_MASK);
                regval = fusb302_getreg(priv, FUSB302_STATUS0_REG);
                fusb302_info("INFO: toggs val=%x\n", toggsval);

                /* set everything up depending on what's been detected */

                set_switches(priv, toggsval);

                /* Power up Internal Oscillator as well now */

                fusb302_putreg(priv, FUSB302_POWER_REG,
                               POWER_MODE_ALL);

                fusb302_dump_registers(priv,
                                       "After SRC toggle detect handling");

                switch (toggsval)
                  {
                    case TOGGS_RUNNING:
                      break;
                    case TOGGS_SRC_CC1:
                    case TOGGS_SRC_CC2:
                    case TOGGS_AUDIO_ACCESSORY:
                      regval = fusb302_getreg(priv, FUSB302_STATUS0_REG);
                      priv->state = WAITING_FOR_SRC_DISCONNECT;
                      priv->connect_status = SRC_DEVICE_CONNECTED;
                      break;
                    case TOGGS_SNK_CC1:
                    case TOGGS_SNK_CC2:
                    default:
                      priv->connect_status = UNKNOWN_CONNECTED;
                      goto error_exit;
                      break;
                  }

                /* turn off toggling */

                regval = fusb302_getreg(priv, FUSB302_CONTROL2_REG);
                regval &= ~CONTROL2_TOGGLE;
                fusb302_putreg(priv, FUSB302_CONTROL2_REG, regval);

                /* Send new interrupt mask */

                set_int_mask(priv);
              }
          }
          break;
        case WAITING_FOR_SRC_DISCONNECT:
          {
            if ((priv->connect_status != SRC_DEVICE_CONNECTED) &&
                (priv->connect_status != AUDIO_ACCESSORY_CONNECTED))
              {
                priv->connect_status = CONNECT_ERROR;
                priv->state = WAITING_FOR_HOST_INTERVENTION;
              }
            else if (interrupt & INTERRUPT_COMP_CHNG)
              {
                priv->connect_status = SRC_DETACH_DETECTED;
                priv->state = WAITING_FOR_HOST_INTERVENTION;
              }
            else if (interrupt & INTERRUPT_BC_LVL)
              {
                priv->connect_status = BC_LEVEL_CHANGE_REQUSTED;
              }
          }
          break;
        case WAITING_FOR_SNK_DISCONNECT:
          {
            regval = fusb302_getreg(priv, FUSB302_STATUS0_REG);
            if (regval & STATUS0_VBUS_OK)
              /* A USB3 device can cause a spurious VBUS OK here.
               * A USB2 device doesn't.
               */

              break;
            if (priv->connect_status != SNK_DEVICE_CONNECTED)
              {
                priv->connect_status = CONNECT_ERROR;
                priv->state = WAITING_FOR_HOST_INTERVENTION;
              }
            else if (interrupt & INTERRUPT_VBUS_OK)
              {
                priv->connect_status = SNK_DETACH_DETECTED;
                priv->state = WAITING_FOR_HOST_INTERVENTION;
              }
            else if (interrupt & INTERRUPT_BC_LVL)
              {
                priv->connect_status = BC_LEVEL_CHANGE_REQUSTED;
              }
          }
          break;
        case WAITING_FOR_HOST_INTERVENTION:
          /* the higher level software has not dealt with the previous
           * response yet so we notify this as an error
           */

          priv->connect_status = CONNECT_ERROR;
          break;
        default:
          fusb302_info("state not defined yet\n");
          fusb302_reset(priv);
          break;
      }

     /* notify any waiters that there's a new state */

      fusb302_info("INFO: connection or state change: %d\n",
                    priv->connect_status);

      poll_notify(priv->fds, CONFIG_FUSB302_NPOLLWAITERS, POLLIN);
    }

error_exit:

  fusb302_clear_interrupts(priv);

  /* re-enable interrupt */

  priv->int_pending = false;
  config->irq_enable(config, true);

  /* release lock */

  nxmutex_unlock(&priv->devlock);
}

/****************************************************************************
 * Name: fusb302_schedule
 *
 * Description:
 *   Schedule the state machine work
 *
 * Input Parameters:
 *   priv - A reference to the FUSB302 peripheral state
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

static int fusb302_schedule(FAR struct fusb302_dev_s *priv)
{
  FAR struct fusb302_config_s *config;
  int                         ret;

  config = priv->config;
  DEBUGASSERT(config != NULL);

  DEBUGASSERT(priv->work.worker == NULL);

  ret = work_queue(HPWORK, &priv->work, fusb302_worker, priv, 0);
  if (ret != 0)
    {
      fusb302_err("ERROR: Failed to queue work: %d\n", ret);
    }

  return OK;
}

/****************************************************************************
 * Name: fusb302_dumpregs
 *
 * Description:
 *   Dump the contents of all FUSB302 registers
 *
 * Input Parameters:
 *   priv - A reference to the FUSB302 peripheral state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FUSB302_REG
static void fusb302_dump_registers(FAR struct fusb302_dev_s *priv,
                                   FAR const char *msg)
{
  fusb302_info("\nFUSB302 Registers: %s\n", msg);

  fusb302_info
    (" Device id: %02x Switches0: %02x   Switch1:  %02x    Measure: %02x\n",
     fusb302_getreg(priv, FUSB302_DEV_ID_REG),
     fusb302_getreg(priv, FUSB302_SWITCHES0_REG),
     fusb302_getreg(priv, FUSB302_SWITCHES1_REG),
     fusb302_getreg(priv, FUSB302_MEASURE_REG));
  fusb302_info
    ("     Slice: %02x  Control0: %02x    Control1:%02x   Control2: %02x\n",
     fusb302_getreg(priv, FUSB302_SLICE_REG),
     fusb302_getreg(priv, FUSB302_CONTROL0_REG),
     fusb302_getreg(priv, FUSB302_CONTROL1_REG),
     fusb302_getreg(priv, FUSB302_CONTROL2_REG));
  fusb302_info
    (" Control3:  %02x      Mask: %02x       Power:%02x      Reset: %02x\n",
     fusb302_getreg(priv, FUSB302_CONTROL3_REG),
     fusb302_getreg(priv, FUSB302_MASK_REG),
     fusb302_getreg(priv, FUSB302_POWER_REG),
     fusb302_getreg(priv, FUSB302_RESET_REG));
  fusb302_info
    (" OCPreg:    %02x     MaskA: %02x       MaskB:%02x   Control4: %02x\n",
     fusb302_getreg(priv, FUSB302_OCPREG_REG),
     fusb302_getreg(priv, FUSB302_MASKA_REG),
     fusb302_getreg(priv, FUSB302_MASKB_REG),
     fusb302_getreg(priv, FUSB302_CONTROL4_REG));
  fusb302_info
    (" Status0a:  %02x  Status1a: %02x  InterruptA:%02x InterruptB: %02x\n",
     fusb302_getreg(priv, FUSB302_STATUS0A_REG),
     fusb302_getreg(priv, FUSB302_STATUS1A_REG),
     fusb302_getreg(priv, FUSB302_INTERRUPTA_REG),
     fusb302_getreg(priv, FUSB302_INTERRUPTB_REG));
  fusb302_info
    (" Status0:   %02x   Status1: %02x   Interrupt:%02x      FIFOs: %02x\n",
     fusb302_getreg(priv, FUSB302_STATUS0_REG),
     fusb302_getreg(priv, FUSB302_STATUS1_REG),
     fusb302_getreg(priv, FUSB302_INTERRUPT_REG),
     fusb302_getreg(priv, FUSB302_FIFOS_REG));
}
#endif

/****************************************************************************
 * Name: fusb302_getreg
 *
 * Description:
 *   Read from an 8-bit FUSB302 register
 *
 * Input Parameters:
 *   priv   - pointer to FUSB302 Private Structure
 *   reg    - register to read
 *
 * Returned Value:
 *   Returns positive register value in case of success, otherwise ERROR
 ****************************************************************************/

static int fusb302_getreg(FAR struct fusb302_dev_s *priv, uint8_t reg)
{
  int ret = -EIO;
  int retries;
  uint8_t regval;
  struct i2c_msg_s msg[2];

  DEBUGASSERT(priv);

  msg[0].frequency = FUSB302_I2C_FREQUENCY;
  msg[0].addr      = priv->config->i2c_addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = FUSB302_I2C_FREQUENCY;
  msg[1].addr      = priv->config->i2c_addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &regval;
  msg[1].length    = 1;

  /* Perform the transfer */

  for (retries = 0; retries < FUSB302_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->config->i2c, msg, 2);
      if (ret >= 0)
        {
          return regval;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == FUSB302_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->config->i2c);
          if (ret < 0)
            {
              fusb302_err("ERROR: I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  fusb302_info("reg:%02X, error:%d\n", reg, ret);
  return ret;
}

/****************************************************************************
 * Name: fusb302_putreg
 *
 * Description:
 *   Write a value to an 8-bit FUSB302 register
 *
 * Input Parameters:
 *   priv    - pointer to FUSB302 Private Structure
 *   regaddr - register to read
 *   regval  - value to be written
 *
 * Returned Value:
 *   None
 ****************************************************************************/

static int fusb302_putreg(FAR struct fusb302_dev_s *priv, uint8_t regaddr,
                          uint8_t regval)
{
  int ret = -EIO;
  int retries;
  struct i2c_msg_s msg;
  uint8_t txbuffer[2];

  /* Setup to the data to be transferred (register address and data). */

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  /* Setup 8-bit FUSB302 address write message */

  msg.frequency = FUSB302_I2C_FREQUENCY;
  msg.addr      = priv->config->i2c_addr;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = 2;

  /* Perform the transfer */

  for (retries = 0; retries < FUSB302_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(priv->config->i2c, &msg, 1);
      if (ret == OK)
        {
          return OK;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == FUSB302_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(priv->config->i2c;
          if (ret < 0)
            {
              fusb302_err("ERROR: I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  fusb302_err("ERROR: failed reg:%02X, value:%02X, error:%d\n",
              regaddr, regval, ret);
  return ret;
}

/****************************************************************************
 * Name: fusb302_read_device_id
 *
 * Description:
 *   Read device version and revision IDs
 *
 ****************************************************************************/

static int fusb302_read_device_id(FAR struct fusb302_dev_s * priv,
                                  FAR uint8_t * arg)
{
  int ret;

  ret = fusb302_getreg(priv, FUSB302_DEV_ID_REG);
  if (ret < 0)
    {
      fusb302_err("ERROR: Failed to read device ID\n");
      return -EIO;
    }

  *arg = ret;

  return OK;
}

/****************************************************************************
 * Name: fusb302_clear_interrupts
 *
 * Description:
 *   Clear interrupts from FUSB302 chip
 *
 ****************************************************************************/

static int fusb302_clear_interrupts(FAR struct fusb302_dev_s * priv)
{
  int ret = OK;

  ret = fusb302_getreg(priv, FUSB302_INTERRUPT_REG);
  if (ret >= 0)
    {
      ret = fusb302_getreg(priv, FUSB302_INTERRUPTA_REG);
    }

  if (ret >= 0)
    {
      ret = fusb302_getreg(priv, FUSB302_INTERRUPTB_REG);
    }

  if (ret < 0)
    {
      fusb302_err("ERROR: Failed to clear interrupts\n");
      return -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: fusb302_setup
 *
 * Description:
 *   Setup FUSB302 chip
 *
 ****************************************************************************/

static int fusb302_setup(FAR struct fusb302_dev_s *priv,
                         struct fusb302_setup_s *setup)
{
  /* Not implemented yet */

  return -ENOTTY;
}

/****************************************************************************
 * Name: fusb302_set_mode
 *
 * Description:
 *   Configure supported device modes (sink, source, DRP, accessory)
 *
 ****************************************************************************/

static int fusb302_set_mode(FAR struct fusb302_dev_s *priv,
                            enum fusb302_mode_e mode)
{
  int ret;
  int regval;

  ret = OK;

  priv->int_pending = false;
  priv->config->irq_enable(priv->config, false);

  /* Best to start with a known, clean, set of register settings.
   * This sets:
   *  Host current to default
   *  Vbus measure off
   *  Switches0 to defaults (Vconn_CC off etc)
   */

  if (priv->state != RESET)
    {
      fusb302_reset(priv);
    }

  regval = fusb302_clear_interrupts(priv);

  switch (mode)
    {
      case MODE_DRP_POLL_AUTO:
        {
          /* set up FUSB302 to automatically poll for device or host */

          ret = fusb302_putreg(priv, FUSB302_CONTROL2_REG,
                               (SET_POLL_MODE(DRP_POLLING) |
                                CONTROL2_TOGGLE |
                                SET_WAIT_MODE(WAIT_80MS))));
          priv->state = WAITING_FOR_DRP_TOGG_I;
          priv->connect_status = TOGGLING;
          set_int_mask(priv);
        }
        break;

      case MODE_SNK_POLL_AUTO:
        {
          /* set up FUSB302 to automatically poll for device */

          ret = fusb302_putreg(priv, FUSB302_CONTROL2_REG,
                               (SET_POLL_MODE(SNK_POLLING) |
                                CONTROL2_TOGGLE));
          priv->state = WAITING_FOR_SNK_TOGG_I;
          priv->connect_status = TOGGLING;
          set_int_mask(priv);
        }
        break;
      case MODE_SRC_POLL_AUTO:
        {
          /* set up FUSB302 to automatically poll for device */

          ret = fusb302_putreg(priv, FUSB302_CONTROL2_REG,
                               (SET_POLL_MODE(SRC_POLLING) |
                                CONTROL2_TOGGLE));
          priv->state = WAITING_FOR_SRC_TOGG_I;
          priv->connect_status = TOGGLING;
          set_int_mask(priv);
        }
        break;
      default:
        return -EINVAL;
        break;
    }

  ret = fusb302_putreg(priv, FUSB302_POWER_REG, POWER_MODE_TOGGLING);

  /* enable interrupts */

  regval = fusb302_getreg(priv, FUSB302_CONTROL0_REG);
  regval &= ~CONTROL0_INT_MASK;
  ret = fusb302_putreg(priv, FUSB302_CONTROL0_REG, regval);

  if (ret == OK)
    {
      priv->config->irq_enable(priv->config, true);
    }
  else
    {
      priv->config->irq_enable(priv->config, false);
    }

  return ret;
}

/****************************************************************************
 * Name: fusb302_read_status
 *
 * Description:
 *   Read status register
 *
 ****************************************************************************/

static int fusb302_read_status(FAR struct fusb302_dev_s *priv,
                               FAR uint8_t *arg)
{
  int ret;

  ret = fusb302_getreg(priv, FUSB302_STATUS0_REG);
  if (ret < 0)
    {
      fusb302_err("ERROR: Failed to read status\n");
      return -EIO;
    }

  *arg = ret;
  return OK;
}

/****************************************************************************
 * Name: fusb302_reset
 *
 * Description:
 *   Reset FUSB302 HW and clear I2C registers
 *
 ****************************************************************************/

static int fusb302_reset(FAR struct fusb302_dev_s *priv)
{
  int ret = OK;

  ret = fusb302_putreg(priv, FUSB302_RESET_REG, RESET_SW_RESET);
  if (ret < 0)
    {
      fusb302_err("ERROR: FUSB302 did not respond to soft reset\n");
      return -EFAULT;
    }
  else
    {
      fusb302_info("INFO: fusb302 soft reset executed\n");
    }

  priv->state = RESET;
  priv->connect_status = NOTHING_CONNECTED;
  return ret;
}

/****************************************************************************
 * Name: fusb302_open
 *
 * Description:
 *   This function is called whenever the FUSB302 device is opened.
 *
 ****************************************************************************/

static int fusb302_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fusb302_dev_s *priv = inode->i_private;
  int ret = OK;

  /* Probe device */

  ret = fusb302_getreg(priv, FUSB302_DEV_ID_REG);
  if (ret < 0)
    {
      fusb302_err("ERROR: No response at given address 0x%02X\n",
                  priv->config->i2c_addr);
      ret = -EFAULT;
    }

  /* Error exit */

  return ret;
}

/****************************************************************************
 * Name: fusb302_close
 *
 * Description:
 *   This routine is called when the FUSB302 device is closed.
 *
 ****************************************************************************/

static int fusb302_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fusb302_dev_s *priv = inode->i_private;

  fusb302_dump_registers(priv, "Registers before close");

  priv->config->irq_enable(priv->config, false);

  return OK;
}

/****************************************************************************
 * Name: fusb302_read
 * Description:
 *   This routine is called when the FUSB302 device is read.
 ****************************************************************************/

static ssize_t fusb302_read(FAR struct file *filep,
                            FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fusb302_dev_s *priv = inode->i_private;
  FAR struct fusb302_result_s *ptr;
  irqstate_t flags;
  int ret;

  if (buflen < sizeof(struct fusb302_result_s))
    {
      return 0;
    }

  ptr = (struct fusb302_result_s *)buffer;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();

  ptr->connected_state = priv->connect_status;
  ptr->state = priv->state;

  if (priv->int_pending)
    {
      fusb302_dump_registers(priv, "Registers after attach detected");
    }

  priv->int_pending = false;

  leave_critical_section(flags);

  nxmutex_unlock(&priv->devlock);

  return sizeof(struct fusb302_result_s);
}

/****************************************************************************
 * Name: fusb302_write
 * Description:
 *   This routine is called when the FUSB302 device is written to.
 ****************************************************************************/

static ssize_t fusb302_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  ssize_t length = 0;

  return length;
}

/****************************************************************************
 * Name: fusb302_ioctl
 * Description:
 *   This routine is called when ioctl function call is performed for
 *   the FUSB302 device.
 ****************************************************************************/

static int fusb302_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct fusb302_dev_s *priv = inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  fusb302_info("cmd: 0x%02X, arg:%lu\n", cmd, arg);

  switch (cmd)
  {
  case USBCIOC_READ_DEVID:
    {
      ret = fusb302_read_device_id(priv, (uint8_t *)arg);
    }
    break;

  case USBCIOC_SETUP:
    {
      ret = fusb302_setup(priv, (struct fusb302_setup_s *)arg);
    }
    break;

  case USBCIOC_SET_MODE:
    {
      ret = fusb302_set_mode(priv, (uint8_t)arg);
    }
    break;
  case USBCIOC_READ_STATUS:
    {
      ret = fusb302_read_status(priv, (uint8_t *)arg);
    }
    break;

  case USBCIOC_RESET:
    {
      fusb302_dump_registers(priv, "Before reset");
      ret = fusb302_reset(priv);
      fusb302_dump_registers(priv, "After reset");
    }
    break;

  default:
    {
      fusb302_err("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
    }
    break;
  }

  fusb302_dump_registers(priv, "After ioctl");
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: fusb302_poll
 * Description:
 *   This routine is called during FUSB302 device poll
 ****************************************************************************/

static int fusb302_poll(FAR struct file *filep,
                        FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode *inode;
  FAR struct fusb302_dev_s *priv;
  int ret = OK;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct fusb302_dev_s *)inode->i_private;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }

      /* This is a request to set up the poll. Find an available
       * slot for the poll structure reference.
       */

      for (i = 0; i < CONFIG_FUSB302_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_FUSB302_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }

out:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: fusb302_callback
 *
 * Description:
 *   FUSB302 interrupt handler
 *
 ****************************************************************************/

static int fusb302_int_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct fusb302_dev_s    *priv = (FAR struct fusb302_dev_s *)arg;
  FAR struct fusb302_config_s *config;
  int                         ret;

  ret = OK;
  DEBUGASSERT(priv != NULL);

  config = priv->config;
  DEBUGASSERT(config != NULL);

  priv->int_pending = true;

  /* Disable interrupts */

  config->irq_enable(config, false);

  /* run worker scheduler */

  ret = fusb302_schedule(priv);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fusb302_register
 *
 * Description:
 *   Register the FUSB302 character driver as "devpath"
 *
 * Input Parameters
 *    devpath - the full path to the driver to register, e.g. "/dev/fusb302"
 *
 * Returned Value
 *
 *  Zero (OK) on success; a negated errno value on failure
 *
 ****************************************************************************/

int fusb302_register(FAR const char *devpath,
                     FAR struct fusb302_config_s *config)
{
  int ret;

  DEBUGASSERT(devpath != NULL && config != NULL);

  /* Initialize the FUSB302 device structure */

  FAR struct fusb302_dev_s *priv;

  priv = (FAR struct fusb302_dev_s *)
                     kmm_zalloc(sizeof(struct fusb302_dev_s));

  if (!priv)
    {
      fusb302_err("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Initialize device structure mutex */

  nxmutex_init(&priv->devlock);

  priv->int_pending = false;
  priv->config      = config;

  /* probe the FUSB302 */

  ret = fusb302_getreg(priv, FUSB302_DEV_ID_REG);

  if (ret < 0)
    {
      fusb302_err("ERROR: FUSB302 did not respond\n");
      return -ENODEV;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_fusb302ops, 0666, priv);
  if (ret < 0)
    {
      fusb302_err("ERROR: Failed to register driver: %d\n", ret);
      goto errout_with_priv;
    }

  /* Prepare interrupt line and handler */

  if (priv->config->irq_clear)
    {
      priv->config->irq_clear(config);
    }

  priv->config->irq_attach(config, fusb302_int_handler, priv);
  priv->config->irq_enable(config, false);

  return OK;

errout_with_priv:
  nxmutex_destroy(&priv->devlock);
  kmm_free(priv);

  return ret;
}
