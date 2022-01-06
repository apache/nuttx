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
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/usb/fusb302.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FUSB302
#  define fusb302_err(x, ...)        _err(x, ##__VA_ARGS__)
#  define fusb302_info(x, ...)       _info(x, ##__VA_ARGS__)
#else
#  define fusb302_err(x, ...)        uerr(x, ##__VA_ARGS__)
#  define fusb302_info(x, ...)       uinfo(x, ##__VA_ARGS__)
#endif

#ifndef CONFIG_FUSB302_I2C_FREQUENCY
#  define CONFIG_FUSB302_I2C_FREQUENCY 800000
#endif

/* Other macros */

#define FUSB302_I2C_RETRIES  10

/****************************************************************************
 * Private Data Types
 ****************************************************************************/
enum fusb_state_e
{
  WAITING_FOR_TOGGLE_I    = 0,
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

struct fusb302_dev_s
{
  FAR struct pollfd           *fds[CONFIG_FUSB302_NPOLLWAITERS];
  uint8_t                     i2c_addr;    /* I2C address */
  volatile bool               int_pending; /* Interrupt received, not handled*/
  sem_t                       devsem;      /* Manages exclusive access */
  FAR struct fusb302_config_s *config;     /* Platform specific configuration */
  struct work_s               work;        /* Supports interrupt handling */

};

const char* fusb_device[] = {"FUSB302", "FUSB302B", "FUSB302T", "FUSB302TV"};
const char* fusb_product[] = {"UCX/MPX", "01MPX", "10MPX", "11MPX"};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

static int fusb302_open(FAR struct file *filep);
static int fusb302_close(FAR struct file *filep);
static ssize_t fusb302_read(FAR struct file *, FAR char *, size_t);
static ssize_t fusb302_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int fusb302_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int fusb302_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);
static void fusb302_notify(FAR struct fusb302_dev_s *priv);
static int fusb302_putreg(FAR struct fusb302_dev_s *priv, uint8_t regaddr,
                          uint8_t regval);
static int fusb302_getreg(FAR struct fusb302_dev_s *priv, uint8_t reg);                          
#ifdef CONFIG_DEBUG_FUSB302
static void fusb302_dump_registers(FAR struct fusb302_dev_s *priv, FAR const char *msg);
#else
#  define fusb302_dump_registers(priv, msg);
#endif 
static int fusb302_reset(FAR struct fusb302_dev_s *priv);
static int fusb302_set_mode(FAR struct fusb302_dev_s *priv,
                            enum fusb302_mode_e mode);
static int fusb302_int_handler(int irq, FAR void *context, FAR void *arg);
static void fusb302_worker(FAR void *arg);
static int fusb302_schedule(FAR struct fusb302_dev_s *priv);
void enableccpd(FAR struct fusb302_dev_s *priv,   enum cc_pd_e pulldown, bool enable);
void enableccpu(FAR struct fusb302_dev_s *priv,   enum cc_pu_e pullup,   bool enable);
void enablevconn(FAR struct fusb302_dev_s *priv, enum cc_vconn_e vconn,  bool enable);

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
  fusb302_poll,  /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL,          /* unlink */
#endif
};

static enum fusb_state_e fusb_state;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
void enableccpd(struct fusb302_dev_s *priv, enum cc_pd_e pulldown, bool enable)
{
  int regval;

  fusb302_info("INFO: CC pulldown %d set to %d\n", pulldown, enable);
  regval = fusb302_getreg(priv, FUSB302_SWITCHES0_REG);
  if (enable)
    {
      regval |= pulldown;
    }
  else
    {
      regval &= ~ pulldown;
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

  fusb302_info("INFO: CC pullup %d set to %d\n", pullup, enable);
  regval = fusb302_getreg(priv, FUSB302_SWITCHES0_REG);
  if (enable)
    {
      regval |= pullup;
    }
  else
    {
      regval &= ~ pullup;
    }
  fusb302_putreg(priv, FUSB302_SWITCHES0_REG, regval);

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
void enablevconn(FAR struct fusb302_dev_s *priv, enum cc_vconn_e vconn,  bool enable)
{
  int regval;

  fusb302_info("INFO: CC vconn %d set to %d\n", vconn, enable);
  regval = fusb302_getreg(priv, FUSB302_SWITCHES0_REG);
  if (enable)
    {
      regval |= vconn;
    }
  else
    {
      regval &= ~ vconn;
    }
  fusb302_putreg(priv, FUSB302_SWITCHES0_REG, regval);

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
  int                         ret;
  int                         regval;
  int                         toggsval;

  DEBUGASSERT(priv != NULL);
  
  config = priv->config;
  DEBUGASSERT(config != NULL);

  /* get exclusive access to the driver data structure */

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->devsem);

      /* would only fail if something cancelled the worker thread? */

      DEBUGASSERT( ret == OK || ret == -ECANCELED);
    }
  while(ret < 0);

  if (!priv->int_pending) /* just to double check we really should be here */
    {
      fusb302_err("ERROR: worker task run with no interrupt\n");
      goto error_exit;
    }
  else
    {
      fusb302_info("INFO: state machine time!\n");

      /* do statemachine */
      switch (fusb_state)
      {
        case WAITING_FOR_TOGGLE_I:
          {
            if (fusb302_getreg(priv, FUSB302_INTERRUPTA_REG) 
                                    & INTERRUPTA_M_TOGDONE)
            {
              toggsval = DECODE_TOGGS(fusb302_getreg(priv, 
                                      FUSB302_STATUS1A_REG));
              fusb302_info("INFO: toggs val=%x\n", toggsval);
              switch (toggsval)
              /* what device detect type was seen */
              {
                case TOGGS_SRC_CC1:
                  enableccpd(priv,  CC1_PULLDOWN, false);                
                  enableccpd(priv,  CC2_PULLDOWN, false);                
                  enableccpu(priv,  CC1_PULLUP, true);
                  enableccpu(priv,  CC2_PULLUP, false);
                  enablevconn(priv, CC1_VCONN, true);
                  enablevconn(priv, CC2_VCONN, false);
                break;
                case TOGGS_SRC_CC2:
                  enableccpd(priv,  CC1_PULLDOWN, false);                
                  enableccpd(priv,  CC2_PULLDOWN, false);                
                  enableccpu(priv,  CC1_PULLUP, false);
                  enableccpu(priv,  CC2_PULLUP, true);  
                  enablevconn(priv, CC1_VCONN, false);               
                  enablevconn(priv, CC2_VCONN, true);                                       
                break;
                case TOGGS_SNK_CC1:
                  enablevconn(priv, CC1_VCONN, false);                  
                  enablevconn(priv, CC2_VCONN, false);
                  enableccpu(priv, CC1_PULLUP, false); 
                  enableccpd(priv, CC2_PULLUP, false);                
                  enableccpd(priv, CC1_PULLDOWN, true);
                  enableccpd(priv, CC2_PULLDOWN, false);
                break;
                case TOGGS_SNK_CC2:
                  enablevconn(priv, CC1_VCONN, false);                  
                  enablevconn(priv, CC2_VCONN, false);                
                  enableccpu(priv, CC1_PULLUP, false); 
                  enableccpu(priv, CC2_PULLUP, false);                
                  enableccpd(priv, CC2_PULLDOWN, true);
                  enableccpd(priv, CC1_PULLDOWN, false);
                break;
                case TOGGS_AUDIO_ACCCESSORY:
                break;
                default:
                  fusb302_err("ERROR: invalid TOGGS VAL read:%x\n");
                  goto error_exit;
                break;
                //fusb_state = NOT_DEFINED;
              }
              fusb302_dump_registers(priv, "After toggle detect");
              /*  Next:
                    1) Check VBus?
                    2) Find out how to trigger USB device from Vbus
                    3) Monitor VBus for disconnect
                    4) revert to toggle if so
              */
            }
          }
        break;
        default:
          {
            fusb302_info("state not defined yet\n");
            fusb302_reset(priv);
          }
      }
     /* notify any waiters that there's a new state */
      fusb302_notify(priv);
    }
error_exit:    
  /* renable interrupt */
  priv->int_pending = false;
  config->irq_enable(config, true);

  /* release lock */
  nxsem_post(&priv->devsem);
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

#ifdef CONFIG_DEBUG_FUSB302
static void fusb302_dump_registers(FAR struct fusb302_dev_s *priv, FAR const char *msg)
{
  fusb302_info("FUSB302 Registers: %s\n", msg); 

  fusb302_info
          (" Device id: %02x Switches0: %02x   Switches1  %02x:    Measure %02x:\n", 
          fusb302_getreg(priv, FUSB302_DEV_ID_REG),
          fusb302_getreg(priv, FUSB302_SWITCHES0_REG),
          fusb302_getreg(priv, FUSB302_SWITCHES1_REG),
          fusb302_getreg(priv, FUSB302_MEASURE_REG));
  fusb302_info
          ("     Slice: %02x  Control0: %02x    Control1: %02x   Control2: %02x\n",
          fusb302_getreg(priv, FUSB302_SLICE_REG),
          fusb302_getreg(priv, FUSB302_CONTROL0_REG),
          fusb302_getreg(priv, FUSB302_CONTROL1_REG),
          fusb302_getreg(priv, FUSB302_CONTROL2_REG));
  fusb302_info
          (" Control3:  %02x      Mask: %02x       Power: %02x      Reset: %02x\n",
          fusb302_getreg(priv, FUSB302_CONTROL3_REG),
          fusb302_getreg(priv, FUSB302_MASK_REG),
          fusb302_getreg(priv, FUSB302_POWER_REG),
          fusb302_getreg(priv, FUSB302_RESET_REG));
  fusb302_info
          (" OCPreg:    %02x     MaskA: %02x       MaskB: %02x   Control4: %02x\n",
          fusb302_getreg(priv, FUSB302_OCPREG_REG),
          fusb302_getreg(priv, FUSB302_MASKA_REG),
          fusb302_getreg(priv, FUSB302_MASKB_REG),
          fusb302_getreg(priv, FUSB302_CONTROL4_REG));
  fusb302_info
          (" Status0a:  %02x  Status1a: %02x  InterruptA: %02x InterruptB: %02x\n",
          fusb302_getreg(priv, FUSB302_STATUS0A_REG),          
          fusb302_getreg(priv, FUSB302_STATUS1A_REG),
          fusb302_getreg(priv, FUSB302_INTERRUPTA_REG),
          fusb302_getreg(priv, FUSB302_INTERRUPTB_REG));
  fusb302_info
          (" Status0:   %02x   Status1: %02x   Interrupt: %02x      FIFOs: %02x\n\n",
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

  msg[0].frequency = CONFIG_FUSB302_I2C_FREQUENCY;
  msg[0].addr      = priv->config->i2c_addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_FUSB302_I2C_FREQUENCY;
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
#if 0          
          /* dump reg added */
          fusb302_info("reg:%02X, value:%02X\n", reg, regval);
#endif
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

  msg.frequency = CONFIG_FUSB302_I2C_FREQUENCY;
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
#if 0          
          /* dump reg added */          
          fusb302_info("reg:%02X, value:%02X\n", regaddr, regval);
#endif
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
  ret = 0b10100101;
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

static int fusb302_clear_interrupts(FAR struct fusb302_dev_s *priv)
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
  int ret = OK;

  fusb302_info("drp_tgl:%02X, host_curr:%02X, global_int:%X, mask:%02X\n",
    setup->drp_toggle_timing, setup->host_current, setup->global_int_mask,
    setup->int_mask);
#if 0
  ret = fusb302_putreg(priv, FUSB302_CONTROL_REG, setup->drp_toggle_timing |
    setup->host_current | setup->global_int_mask);

  if (ret < 0)
    {
      fusb302_err("ERROR: Failed to write control register\n");
      goto err_out;
    }

  ret = fusb302_putreg(priv, FUSB302_MASK_REG, setup->int_mask);
  if (ret < 0)
    {
      fusb302_err("ERROR: Failed to write mask register\n");
    }

err_out:
#endif
  return ret;

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

  switch (mode)
  {
    case MODE_SRC_POLL_MAN:

    break;
    case MODE_SRC_POLL_AUTO:
      /* set up FUSB302 to automatically poll for device or host (msd) */

    break;
    case MODE_SNK_POLL_MAN:

    break;
    case MODE_SNK_POLL_AUTO:

    break;
    case MODE_DRP_POLL_MAN:

    break;
    case MODE_DRP_POLL_AUTO:
      
      priv->int_pending = false;      
      priv->config->irq_enable(priv->config, false);
      fusb302_reset(priv);
      //ret = fusb302_putreg(priv, FUSB302_SWITCHES0_REG, 0);
      ret = fusb302_putreg(priv, FUSB302_CONTROL2_REG, 
                                 (SET_POLL_MODE(DRP_POLLING) | CONTROL2_TOGGLE));
      ret = fusb302_putreg(priv, FUSB302_POWER_REG, 
                                 POWER_MODE(POWER_MODE_BANDGAP_AND_WAKE |
                                            POWER_MODE_RX_AND_IREF |
                                            POWER_MODE_MEASURE_BLOCK));
      ret = fusb302_putreg(priv, FUSB302_MASK_REG, 
                                 (MASK_VBUS_OK       |
                                  MASK_ACTIVITY   |
                                  MASK_COMP_CHNG  |
                                  MASK_CRC_CHK    |
                                  MASK_ALERT      |
                                  MASK_WAKE       |
                                  MASK_COLLISION));
      ret = fusb302_putreg(priv, FUSB302_MASKA_REG, 
                                 (MASKA_OCP_TEMP  |
                                  MASKA_SOFTFAIL  |
                                  MASKA_RETRYFAIL |
                                  MASKA_HARDSENT  |
                                  MASKA_TXSENT    |
                                  MASKA_SOFTRST   |
                                  MASKA_HARDRST ));
      ret = fusb302_putreg(priv, FUSB302_MASKB_REG, MASKB_GCRCSENT);
      regval = fusb302_getreg(priv, FUSB302_CONTROL0_REG );
      regval &= ~CONTROL0_INT_MASK;
      /* enable interrupts */
      ret = fusb302_putreg(priv, FUSB302_CONTROL0_REG, regval);
      if (ret == OK)
        {
          //ret = fusb302_putreg(priv, FUSB302_POWER_REG, POWER_MODE(POWER_MODE_ALL));

          priv->config->irq_enable(priv->config, true);
          fusb_state = WAITING_FOR_TOGGLE_I;
        }
      else
        {
          priv->config->irq_enable(priv->config, false);
        }

    break;
    default:
      return -EINVAL;
    break;
  }

  return ret;

}

/****************************************************************************
 * Name: fusb302_set_state
 *
 * Description:
 *   Force device in specified state
 *
 ****************************************************************************/
#if 0
static int fusb302_set_state(FAR struct fusb302_dev_s *priv,
                             enum fusb302_manual_e state)
{
  int ret = OK;

  if (state > MANUAL_UNATT_SNK)
    {
      return -EINVAL;
    }

  ret = fusb302_putreg(priv, FUSB302_MANUAL_REG, state);
  if (ret < 0)
    {
      fusb302_err("ERROR: Failed to write manual register\n");
      ret = -EIO;
    }

  return ret;
}
#endif

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
 * Name: fusb302_read_devtype
 *
 * Description:
 *   Read type of attached device
 *
 ****************************************************************************/

static int fusb302_read_devtype(FAR struct fusb302_dev_s *priv,
                                FAR uint8_t *arg)
{
  int ret;

  ret = OK;
#if 0
  ret = fusb302_getreg(priv, FUSB302_TYPE_REG);
  if (ret < 0)
    {
      fusb302_err("ERROR: Failed to read type\n");
      return -EIO;
    }
#endif
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
  else
    {
      fusb302_info("INFO: FUSB302 Found, type: %s%s  Revision: %c\n", 
                    fusb_device[FUSB302_DEVICE(ret)],
                    fusb_product[FUSB302_PRODUCT(ret)],
                    FUSB302_REVISION(ret));      

      fusb302_clear_interrupts(priv);
      priv->config->irq_enable(priv->config, true);
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

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();



  ptr->status0 = fusb302_getreg(priv, FUSB302_STATUS0_REG);
  ptr->status1 = fusb302_getreg(priv, FUSB302_STATUS1_REG);
  ptr->status0a = fusb302_getreg(priv, FUSB302_STATUS0A_REG);
  ptr->status1a = fusb302_getreg(priv, FUSB302_STATUS1A_REG);
  ptr->interrupt = fusb302_getreg(priv, FUSB302_INTERRUPT_REG);
  ptr->int_pending = priv->int_pending;

  if (priv->int_pending)
  {
    fusb302_dump_registers(priv, "Registers after attach detected");
  }
  
  priv->int_pending = false;

  leave_critical_section(flags);

  nxsem_post(&priv->devsem);
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

  ret = nxsem_wait(&priv->devsem);
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
#if 0
  case USBCIOC_SET_STATE:
    {
      ret = fusb302_set_state(priv, (uint8_t)arg);
    }
    break;
#endif
  case USBCIOC_READ_STATUS:
    {
      ret = fusb302_read_status(priv, (uint8_t *)arg);
    }
    break;

  case USBCIOC_READ_DEVTYPE:
    {
      ret = fusb302_read_devtype(priv, (uint8_t *)arg);
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

  nxsem_post(&priv->devsem);
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
  irqstate_t flags;
  int ret = OK;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct fusb302_dev_s *)inode->i_private;

  ret = nxsem_wait(&priv->devsem);
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

      flags = enter_critical_section();
      if (priv->int_pending)
        {
          fusb302_notify(priv);
        }

      leave_critical_section(flags);
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
  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: fusb302_notify
 *
 * Description:
 *   Notify thread about data to be available
 *
 ****************************************************************************/

static void fusb302_notify(FAR struct fusb302_dev_s *priv)
{
  DEBUGASSERT(priv != NULL);

  int i;

  /* If there are threads waiting on poll() for FUSB302 data to become
   * available, then wake them up now.  NOTE: we wake up all waiting threads
   * because we do not know that they are going to do.  If they all try to
   * read the data, then some make end up blocking after all.
   */

  for (i = 0; i < CONFIG_FUSB302_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          fusb302_info("Report events: %02x\n", fds->revents);
          nxsem_post(fds->sem);
        }
    }
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
  //irqstate_t                  flags;
  int                         ret;
  
  ret = OK;
  DEBUGASSERT(priv != NULL);

  config = priv->config;
  DEBUGASSERT(config != NULL);

  
  //flags = enter_critical_section();
  priv->int_pending = true;
  /* Disable interrupts */
  config->irq_enable(config, false);
  /*Clear any pending interrupts  */
  config->irq_clear(config);  

  /* run worker scheduler */  
  ret = fusb302_schedule(priv);

  //fusb302_notify(priv);
  //leave_critical_section(flags);
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
int fusb302_register(FAR const char *devpath, FAR struct fusb302_config_s *config)
{
  int ret;

  DEBUGASSERT(devpath != NULL && config != NULL);

  /* Initialize the FUSB302 device structure */
  FAR struct fusb302_dev_s *priv =
                          (FAR struct fusb302_dev_s *)
                           kmm_zalloc(sizeof(struct fusb302_dev_s));
  if (priv == NULL)
    {
      fusb302_err("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  /* Initialize device structure semaphore */

  priv->int_pending = false;
  priv->config      = config;
  
  nxsem_init(&priv->devsem, 0, 1);

  /* probe the FUSB302 */
  ret = fusb302_getreg(priv, FUSB302_DEV_ID_REG);
  if (ret < 0)
    {
      fusb302_err("ERROR: FUSB302 did not respond with a version or revision\n");
      return -ENODEV;      
    }      
  else
    {
      fusb302_info("INFO: FUSB302 Found, type: %s%s  Revision: %c\n", 
                    fusb_device[FUSB302_DEVICE(ret)],
                    fusb_product[FUSB302_PRODUCT(ret)],
                    FUSB302_REVISION(ret)); 
    }
  /* Register the character driver */

  ret = register_driver(devpath, &g_fusb302ops, 0666, priv);
  if (ret < 0)
    {
      fusb302_err("ERROR: Failed to register driver: %d\n", ret);
      goto errout_with_priv;
    }

  /* Prepare interrupt line and handler. */

  if (priv->config->irq_clear)
    {
      priv->config->irq_clear(config);
    }

  priv->config->irq_attach(config, fusb302_int_handler, priv);
  priv->config->irq_enable(config, false);

  return OK;

errout_with_priv:
  nxsem_destroy(&priv->devsem);
  kmm_free(priv);

  return ret;
}
