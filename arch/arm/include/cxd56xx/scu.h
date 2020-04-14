/****************************************************************************
 * arch/arm/include/cxd56xx/scu.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_SCU_H
#define __ARCH_ARM_INCLUDE_CXD56XX_SCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define _SCUIOCBASE   (0xa000)

#define _SCUIOCVALID(c)   (_IOC_TYPE(c)==_SCUIOCBASE)
#define _SCUIOC(nr)       _IOC(_SCUIOCBASE,nr)

/* Set FIFO size used by sequencer
 *
 * uint16_t FIFO size in bytes
 * return ioctl return value provides success/failure indication
 */

#define SCUIOC_SETFIFO     _SCUIOC(0x0001)

/* Free FIFO used by sequencer
 *
 * param: none
 * return: ioctl return value provides success/failure indication
 */

#define SCUIOC_FREEFIFO    _SCUIOC(0x0002)

/* Set sequencer sampling rate
 *
 * param: uint8_t: sampling rate
 * return: ioctl return value provides success/failure indication
 */

#define SCUIOC_SETSAMPLE   _SCUIOC(0x0003)

/* Enable/Disable sign conversion preprocessing
 *
 * param: unsigned long: 0 = disable, 1 = enable
 * return: ioctl return value provides success/failure indication
 */

#define SCUIOC_SETSIGNCONV _SCUIOC(0x0004)

/* Set offset/gain adjustment parameter
 *
 * param: Pointer to struct adjust_xyz_s.
 * return: ioctl return value provides success/failure indication
 */

#define SCUIOC_SETOGADJUST _SCUIOC(0x0005)

/* Clear offset/gain adjustment parameter
 *
 * param: none
 * return: ioctl return value provides success/failure indication
 */

#define SCUIOC_CLROGADJUST _SCUIOC(0x0006)

/* Set IIR filter coefficiencies
 *
 * param: Pointer to struct math_filter_s
 * return: ioctl return value provides success/failure indication
 */

#define SCUIOC_SETFILTER   _SCUIOC(0x0007)

/* Set event notifier parameters
 *
 * Set event notifier. This command must use with IIR filter, so it will be
 * error when no filter set by SCUIOC_SETFILTER.
 *
 * param: Pointer to struct scuev_notify_s
 * return: ioctl return value provides success/failure indication
 */

#define SCUIOC_SETNOTIFY   _SCUIOC(0x0008)

/* Set elements per sample
 *
 * SCU can be treat 3 axis sample data as 1 or 2 axis data. For example,
 * user can be used to detect via event notifier, about the board is
 * tilting, by X and Y axis data from accelerometer.
 * This setting will be affected for IIR Filter and event notifier.
 *
 * param: unsigned long: 1 - 3
 * return: ioctl return value provides success/failure indication
 */

#define SCUIOC_SETELEMENTS   _SCUIOC(0x0009)

/* Set decimation parameters (Decimator only)
 *
 * param: Pointer to struct decimation_s
 * return: ioctl return value provides success/failure indication
 */

#define SCUIOC_SETDECIMATION _SCUIOC(0x000a)

/* Set FIFO sample data watermark
 *
 * param: Pointer of struct scufifo_wm_s
 * return: ioctl return value provides success/failure indication
 */

#define SCUIOC_SETWATERMARK  _SCUIOC(0x000b)

/* Start sequencer
 *
 * param: None
 * return: ioctl return value provides success/failure indication
 */

#define SCUIOC_START       _SCUIOC(0x0010)

/* Stop sequencer
 *
 * param: None
 * return: ioctl return value provides success/failure indication
 */

#define SCUIOC_STOP        _SCUIOC(0x0011)

/* Set FIFO overwrite mode
 *
 * param: unsigned long: 0 = overwrite disable, 1 = overwrite enable
 * return: ioctl return value provides success/failure indication
 */

#define SCUIOC_SETFIFOMODE _SCUIOC(0x0012)

/* Delete FIFO data
 *
 * param: uint16_t delete size
 * return: ioctl return value provides success/failure indication
 */

#define SCUIOC_DELFIFODATA _SCUIOC(0x0013)

#define SCU_BUS_SPI     1         /* SPI bus */
#define SCU_BUS_I2C0    2         /* I2C0 bus */
#define SCU_BUS_I2C1    3         /* I2C1 bus */
#define SCU_BUS_LPADC0  0x10      /* LPADC0 */
#define SCU_BUS_LPADC1  0x11      /* LPADC1 */
#define SCU_BUS_LPADC2  0x12      /* LPADC2 */
#define SCU_BUS_LPADC3  0x13      /* LPADC3 */
#define SCU_BUS_HPADC0  0x14      /* HPADC1 */
#define SCU_BUS_HPADC1  0x15      /* HPADC2 */

/* Send 1 byte instruction */

#define SCU_INST_SEND(val) ((val) & 0xff)

/* Receive n byte instruction (n: 1 - 8) */

#define SCU_INST_RECV(n)   ((1 << 8) | (((n) - 1) & 0x7) << 12)

#define SCU_INST_TERM      (1 << 11)
#define SCU_INST_RESTART   (1 << 10)
#define SCU_INST_STOP      (1 <<  9)

/* Indicate this instruction is last */

#define SCU_INST_LAST      (SCU_INST_TERM|SCU_INST_STOP)

/* Sequencer types */

#define SEQ_TYPE_NORMAL    0    /* Normal sequencer */
#define SEQ_TYPE_DECI      1    /* Decimator */

/* Event control */

#define SCU_EV_RISE_EN   (1 << 31) /* Rise event enable */
#define SCU_EV_FALL_EN   (1 << 30) /* Fall event enable */
#define SCU_EV_OUT8BITS  (1 << 29) /* Output only upper 8 bits */

#define SCU_EV_OUTSHIFT     16
#define SCU_EV_OUTMASK      (3 << SCU_EV_OUTSHIFT)
#define SCU_EV_OUTALWAYS    (0 << SCU_EV_OUTSHIFT) /* Always output to FIFO */
#define SCU_EV_OUTSTART     (1 << SCU_EV_OUTSHIFT) /* Output start after event occurred */
#define SCU_EV_OUTSTOP      (2 << SCU_EV_OUTSHIFT) /* Output stop after event occurred */
#define SCU_EV_NOTOUT       (3 << SCU_EV_OUTSHIFT) /* No output to FIFO */

#define SCU_EV_WRITESAMPLEMASK 0xfff

/* Output samples when event occurred */

#define SCU_EV_WRITESAMPLES(sample) ((sample) & SCU_EV_WRITESAMPLEMASK)

/* Event type flags */

#define SCU_EV_RISE  (1) /* Rise (low to high) event occurred */
#define SCU_EV_FALL  (2) /* Fall (high to low) event occurred */

/* Level adjustment (decimator only) */

#define SCU_LEVELADJ_X1 (0)     /* Level adjustment x1 */
#define SCU_LEVELADJ_X2 (1)     /* Level adjustment x2 */
#define SCU_LEVELADJ_X4 (2)     /* Level adjustment x4 */
#define SCU_LEVELADJ_X8 (3)     /* Level adjustment x8 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* IIR filter position
 * details IIR filter can be set 2 filters on data path.
 * image html scu_IIR_filter_path.png
 */

enum filter_pos_e
{
  FILTER_POS_NONE = 0,    /* Filter none */
  FILTER_POS_AA,          /* 2 for all */
  FILTER_POS_FF,          /* 2 for FIFO */
  FILTER_POS_AF,          /* 1 for all, 1 for FIFO */
  FILTER_POS_EE,          /* 2 for Event detector */
  FILTER_POS_AE,          /* 1 for all, 1 for Event detector */
  FILTER_POS_FE,          /* 1 for FIFO, 1 for Event detector */
};

/* Sample timestamp */

struct scutimestamp_s
{
  uint32_t sec;                 /* Seconds */
  uint16_t tick;                /* Clock tick (based on 32768 Hz) */
};

/* IIR filter coefficiencies */

struct iir_coeff_s
{
  uint32_t h;                   /* High 32 bit */
  uint8_t l;                    /* Low 8 bit */
};

/* IIR filter setting */

struct iir_filter_s
{
  uint8_t ishift;               /* Input shift */
  uint8_t oshift;               /* Output shift */
  struct iir_coeff_s coeff[5];  /* IIR filter coefficiencies */
};

/* Math Function IIR filter setting */

struct math_filter_s
{
  enum filter_pos_e pos;         /* Insert position identifier */
  struct iir_filter_s filter[2]; /* IIR filter parameters */
};

/* Event configuration
 *
 * a threshold is rise/fall threshold for input data.
 * When set this member for a rise in struct scuev_notify_s, counting data
 * if higher than threshold, and a fall is vise-versa.
 * a count0 is used for prevent chattering. a count1 is used for counting
 * continuously input.
 * When configured for a rise, SCU processing as follows:
 *
 * - Counting higher value than threshold coninuously
 * - If counter reached to a count0, start actual couning
 * - If input data fall less than threshold before reached to a count0,
 *   then stop and reset counts
 * - If total count is reached to a count0 + a count1, then raise rise event
 * - If count1 is zero, then notify when count reached to a count0
 *   immediately
 * - If threshold or count0 is zero, configuration is ignored
 */

struct sensor_event_s
{
  uint16_t threshold;           /* Threshold */
  uint16_t count0;              /* Preventing counts */
  uint16_t count1;              /* Actual counts  */
  uint16_t delaysamples;        /* Event notification delay in samples */
};

/* Arguments for event signal */

struct scuev_arg_s
{
  struct scutimestamp_s ts;    /* timestamp stored when event occurred. */
  uint32_t              type;  /* Event type (SCU_EV_RISE or SCU_EV_FALL) */
};

/* Event notifier setting */

struct scuev_notify_s
{
  int                    signo;  /* Signal number (0 - 31, except system reserved) */

  struct sensor_event_s  rise;   /* Rise threshold */
  struct sensor_event_s  fall;   /* Fall threshold */

  uint32_t               ctrl;   /* Event control */

  FAR struct scuev_arg_s *arg;   /* Arguments for event raised */
};

/* Offset/gain adjustment parameter */

struct adjust_s
{
  uint16_t offset;              /* Offset value */
  uint16_t gain;                /* Gain value */
};

/* Offset/gain adjustment parameter for 3 axis */

struct adjust_xyz_s
{
  struct adjust_s x;            /* For X axis */
  struct adjust_s y;            /* For Y axis */
  struct adjust_s z;            /* For Z axis */
};

/* Decimation parameter */

struct decimation_s
{
  uint8_t ratio;                /* Decimation ratio (1 / (2 ^ ratio)), 0 - 9 */
  uint8_t leveladj;             /* Output data multiplier */
  uint8_t forcethrough;         /* Force through */
};

/* Watermark notification */

struct scufifo_wm_s
{
  int                    signo; /* Signal number (0 - 31, except system reserved) */

  /* Pointer to memory to be timestamp stored */

  FAR struct scutimestamp_s *ts;

  /* Watermark value. SCU notifies when
   * stored samples over watermark in FIFO.
   * Valid value range: 1 - 65535
   */

  uint16_t               watermark;
};

struct seq_s;     /* The sequencer object */

/* Open sequencer
 *
 * param [in] type    : Sequencer type. Set one of the following definitions.
 *        - #SEQ_TYPE_NORMAL
 *        - #SEQ_TYPE_DECI
 * param [in] bustype : Bustype. Set one of the foollowing definitions.
 *        - #SCU_BUS_I2C0
 *        - #SCU_BUS_I2C1
 *        - #SCU_BUS_SPI
 *
 * return: struct seq_s pointer is success. NULL is failure.
 */

FAR struct seq_s *seq_open(int type, int bustype);

/* Close sequencer device
 *
 * param [in] seq : Sequencer instance
 */

void seq_close(FAR struct seq_s *seq);

/* Read sequencer FIFO data
 *
 * param [in]  seq    : Sequencer instance
 * param [in]  fifoid : FIFO ID (decimator only)
 * param [out] buffer : Pointer to data receive buffer
 * param [in]  length : Buffer length
 *
 * return : OK(0) is success. negative value is failure.
 */

int seq_read(FAR struct seq_s *seq, int fifoid, char *buffer, int length);

/* Sequencer specific ioctl
 *
 * This API should be called from each sensor driver ioctl().
 *
 * param [in]     seq    : Sequencer instance
 * param [in]     fifoid : FIFO ID (decimator only)
 * param [in]     cmd    : ioctl commands (SCUIOC_*).
 * param [in,out] arg    : Argument for each commands
 *
 * see #scu_ioctl
 *
 * return: OK(0) is success. negative value is failure.
 */

int seq_ioctl(FAR struct seq_s *seq, int fifoid, int cmd, unsigned long arg);

/* Set cyclic sequencer instruction
 *
 * param [in] seq      : Sequencer instance
 * param [in] inst     : Pointer to instruction array
 * param [in] nr_insts : Number of instructions
 *
 * return  OK(0) is success. negative value is failure.
 */

int seq_setinstruction(FAR struct seq_s *seq, const uint16_t *inst,
                       uint16_t nr_insts);

/* Set sample data format
 *
 * param [in] seq      : Sequencer instance
 * param [in] sample   : Bytes per sample
 * param [in] offset   : Start offset of sampling data
 * param [in] elemsize : Bytes of 1 element in sample
 * param [in] swapbyte : Enable/Disable byte swapping
 *
 * return  OK(0) is success. negative value is failure.
 */

void seq_setsample(FAR struct seq_s *seq, uint8_t sample, uint8_t offset,
                   uint8_t elemsize, bool swapbyte);

/* Set slave ID or address
 *
 * param [in] seq        : An instance of sequencer
 * param [in] slave_addr : In SPI, slave select ID. In I2C, bus address.
 */

void seq_setaddress(FAR struct seq_s *seq, uint32_t slave_addr);

/* SPI data transfer via sequencer.
 *
 * This function use 'oneshot' feature on SCU. So user unnecessary to specify
 * any opened sequencer.
 * This function useful for accessing register directly.
 *
 * param [in]  slavesel : Slave select
 * param [in]  inst     : Sequencer instruction
 * param [in]  nr_insts : Number of instructions
 * param [out] buffer   : Pointer to receive buffer, if no need to receive,
 *                         then NULL.
 * param [in]  len      : buffer length (ignored when buffer is NULL)
 *
 * return  OK(0) is success. negative value is failure.
 */

int scu_spitransfer(int slavesel, uint16_t *inst, uint32_t nr_insts,
                         uint8_t *buffer, int len);

/* I2C data transfer via sequencer
 *
 * This function use 'oneshot' feature on SCU. So user unnecessary to
 * specify any opened sequencer.
 * This function useful for accessing register directly.
 *
 * param [in]  port     : I2C port (0 or 1)
 * param [in]  slave    : Slave address
 * param [in]  inst     : Sequencer instruction
 * param [in]  nr_insts : Number of instructions
 * param [out] buffer   : Pointer to receive buffer, if no need to receive,
 *                         then NULL.
 * param [in]  len      : buffer length (ignored when buffer is NULL)
 *
 * return OK(0) is success. negative value is failure.
 */

int scu_i2ctransfer(int port, int slave, uint16_t *inst, uint32_t nr_insts,
                         uint8_t *buffer, int len);

/* Initialize SCU
 *
 * warning: This API called from board_app_initialize().
 * Do not call this API from each sensor drivers.
 */

void scu_initialize(void);

/* Uninitialize SCU
 *
 * warning: This API called from board_app_initialize().
 * Do not call this API from each sensor drivers.
 */

void scu_uninitialize(void);

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_SCU_H */
