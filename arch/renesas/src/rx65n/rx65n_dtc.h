/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_dtc.h
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

#ifndef __ARCH_RENESAS_SRC_RX65N_DTC_H
#define __ARCH_RENESAS_SRC_RX65N_DTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DTC_HANDLE provides an opaque are reference that can be used to represent
 * a DTC channel.
 */

typedef FAR void *DTC_HANDLE;

/* Configurable options for DTC Transfer mode */

typedef enum e_dtc_transfer_mode
{
  DTC_TRANSFER_MODE_NORMAL = (0),      /* = (0 << 6): Normal mode */
  DTC_TRANSFER_MODE_REPEAT = (1 << 6), /* Repeat mode */
  DTC_TRANSFER_MODE_BLOCK  = (2 << 6)  /* Block mode */
} dtc_transfer_mode_t;

/* Configurable options for DTC Data transfer size */

typedef enum e_dtc_data_size
{
  DTC_DATA_SIZE_BYTE  = (0),      /* = (0 << 4): 8-bit (byte) data */
  DTC_DATA_SIZE_WORD  = (1 << 4), /* 16-bit (word) data */
  DTC_DATA_SIZE_LWORD = (2 << 4)  /* 32-bit (long word) data */
} dtc_data_size_t;

/* Configurable options for Source address addressing mode */

typedef enum e_dtc_src_addr_mode
{
  DTC_SRC_ADDR_FIXED = (0),      /* = (0 << 2): Source address is fixed. */
  DTC_SRC_ADDR_INCR  = (2 << 2), /* Source address is incremented after each transfer. */
  DTC_SRC_ADDR_DECR  = (3 << 2)  /* Source address is decremented after each transfer. */
} dtc_src_addr_mode_t;

/* Configurable options for Chain transfer */

typedef enum e_dtc_chain_transfer
{
  DTC_CHAIN_TRANSFER_DISABLE = (0),    /* Disable Chain transfer. */
  DTC_CHAIN_TRANSFER_ENABLE = (1 << 7) /* Enable Chain transfer. */
} dtc_chain_transfer_t;

/* Configurable options for how chain transfer is performed. */

typedef enum e_dtc_chain_transfer_mode
{
  DTC_CHAIN_TRANSFER_CONTINUOUSLY = (0),     /* = (0 << 6): Chain transfer is performed continuously. */
  DTC_CHAIN_TRANSFER_NORMAL       = (1 << 6) /* Chain transfer is performed only when the counter is changed to 0 or CRAH. */
} dtc_chain_transfer_mode_t;

/* Configurable options for Interrupt */

typedef enum e_dtc_interrupt
{
  DTC_INTERRUPT_AFTER_ALL_COMPLETE = (0),      /* Interrupt is generated when specified data transfer is completed. */
  DTC_INTERRUPT_PER_SINGLE_TRANSFER = (1 << 5) /* Interrupt is generated when each transfer time is completed. */
} dtc_interrupt_t;

/* Configurable options for Side to be repeat or block */

typedef enum e_dtc_repeat_block_side
{
  DTC_REPEAT_BLOCK_DESTINATION = (0), /* = (0 << 4): Destination is repeat or block area. */
  DTC_REPEAT_BLOCK_SOURCE = (1 << 4)  /* Source is repeat or block area. */
} dtc_repeat_block_side_t;

/* Configurable options for Destination address addressing mode */

typedef enum e_dtc_dest_addr_mode
{
  DTC_DES_ADDR_FIXED = (1 << 2), /* Destination address is fixed. */
  DTC_DES_ADDR_INCR = (2 << 2),  /* Destination address is incremented after each transfer. */
  DTC_DES_ADDR_DECR = (3 << 2)   /* Destination address is decremented after each transfer. */
} dtc_dest_addr_mode_t;

/* Configurable options for Write-back Disable */

typedef enum e_dtc_write_back
{
  DTC_WRITEBACK_ENABLE = (0),
  DTC_WRITEBACK_DISABLE = (1)
} dtc_write_back_t;

/* Configurable options for Sequence Transfer End */

typedef enum e_dtc_sequence_end
{
  DTC_SEQUENCE_TRANSFER_CONTINUE = (0),
  DTC_SEQUENCE_TRANSFER_END = (1)
} dtc_sequence_end_t;

/* Configurable options for Index Table Reference */

typedef enum e_dtc_refer_index_table
{
  DTC_REFER_INDEX_TABLE_DISABLE = (0),
  DTC_REFER_INDEX_TABLE_ENABLE = (1 << 1)
} dtc_refer_index_table_t;

/* Configurable options for Displacement Addition */

typedef enum e_dtc_disp_add
{
  DTC_SRC_ADDR_DISP_ADD_DISABLE = (0),
  DTC_SRC_ADDR_DISP_ADD_ENABLE = (1)
} dtc_disp_add_t;

/* Enumerate list that can be selected as DTC activation source
 * enum enum_dtce: is included from iodefine.h
 */

typedef enum enum_dtce dtc_activation_source_t;

typedef enum e_dtc_command
{
  DTC_CMD_DTC_START,                 /* DTC will can accept activation requests. */
  DTC_CMD_DTC_STOP,                  /* DTC will not accept new activation request. */
  DTC_CMD_ACT_SRC_ENABLE,            /* Enable an activation source specified by vector number. */
  DTC_CMD_ACT_SRC_DISABLE,           /* Disable an activation source specified by vector number. */
  DTC_CMD_DATA_READ_SKIP_ENABLE,     /* Enable Transfer Data Read Skip. */
  DTC_CMD_DATA_READ_SKIP_DISABLE,    /* Disable Transfer Data Read Skip. */
  DTC_CMD_STATUS_GET,                /* Get the current status of DTC. */
  DTC_CMD_CHAIN_TRANSFER_ABORT,      /* Abort the current Chain transfer process. */
  DTC_CMD_SEQUENCE_TRANSFER_ENABLE,  /* Sequence transfer is enabled. */
  DTC_CMD_SEQUENCE_TRANSFER_DISABLE, /* Sequence transfer is disabled. */
  DTC_CMD_SEQUENCE_TRANSFER_ABORT,   /* Abort the sequence transfer. */
  DTC_CMD_CHANGING_DATA_FORCIBLY_SET /* Changing data forcibly set by R_DTC_Create(). */
} dtc_command_t;

typedef enum e_dtc_err       /* DTC API error codes */
{
  DTC_SUCCESS_DMAC_BUSY = 0, /* One or some DMAC resources are locked by another process. */
  DTC_SUCCESS,
  DTC_ERR_OPENED,            /* DTC was initialized already. */
  DTC_ERR_NOT_OPEN,          /* DTC module is not initialized yet. */
  DTC_ERR_INVALID_ARG,       /* Arguments are invalid. */
  DTC_ERR_INVALID_COMMAND,   /* Command parameters are invalid. Or, forced data change failed. */
  DTC_ERR_NULL_PTR,          /* Argument pointers are NULL. */
  DTC_ERR_BUSY,              /* The DTC resources are locked by another process. */
  DTC_ERR_ACT                /* Data transfer is in progress. */
} dtc_err_t;

/* Transfer data configuration */

typedef struct st_dtc_static_transfer_data_cfg
{
  dtc_transfer_mode_t       transfer_mode;            /* DTC transfer mode */
  dtc_data_size_t           data_size;                /* Size of data unit */
  dtc_src_addr_mode_t       src_addr_mode;            /* Address mode of source */
  dtc_chain_transfer_t      chain_transfer_enable;    /* Chain transfer is enabled or not */
  dtc_chain_transfer_mode_t chain_transfer_mode;      /* How chain transfer is performed */
  dtc_interrupt_t           response_interrupt;       /* How response interrupt is raised */
  dtc_repeat_block_side_t  repeat_block_side;         /* The side being repeat or block in */
  dtc_dest_addr_mode_t      dest_addr_mode;           /* Address mode of destination */
  uint32_t                  source_addr;              /* Start address of source */
  uint32_t                  dest_addr;                /* Start address of destination */
  uint32_t                  transfer_count;           /* Transfer count */
  uint16_t                  block_size;               /* Size of a block in block transfer mode */
  uint16_t                  rsv;                      /* Reserved */
  dtc_write_back_t          writeback_disable;        /* Write-back disable or enable */
  dtc_sequence_end_t        sequence_end;             /* Sequence transfer end or continue */
  dtc_refer_index_table_t   refer_index_table_enable; /* Index table refer or not refer */
  dtc_disp_add_t            disp_add_enable;          /* The displacement value is added or not added */
} dtc_static_transfer_data_cfg_t;

/* Transfer data configuration */

typedef struct st_dtc_dynamic_transfer_data_cfg
{
  dtc_data_size_t data_size;      /* Size of data unit */
  uint32_t        source_addr;    /* Start address of source */
  uint32_t        dest_addr;      /* Start address of destination */
  uint32_t        transfer_count; /* Transfer count */
  uint16_t        block_size;     /* Size of a block in block transfer mode */
  uint16_t        rsv;            /* Reserved */
} dtc_dynamic_transfer_data_cfg_t;

/* DTC status structure */

typedef struct st_dtc_stat
{
  uint8_t vect_nr;     /* The current vector number */
  bool    in_progress; /* Active flag of DTC module */
} dtc_stat_t;

/* Transfer data type */

#if defined(CONFIG_RX65N_DTC_SHORT_ADDRESS_MODE) /* Short-address mode */

typedef struct st_transfer_data
{
  /* 3 long-words */

  uint32_t lw1;
  uint32_t lw2;
  uint32_t lw3;
} dtc_transfer_data_t;

#else /* Full-address mode */
typedef struct st_transfer_data
{
  /* 4 long-words */

  uint32_t lw1;
  uint32_t lw2;
  uint32_t lw3;
  uint32_t lw4;
} dtc_transfer_data_t;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: rx65n_dtc_gethandle
 *
 * Description:
 *   Get DTC handle.
 *
 * Input Parameters:
 *   chan - Identifies the channel resource. For the RX65N, this
 *     is simply the channel number is 0
 *
 * Returned Value:
 *   dtchandle-Provided that 'chan' is valid, this function ALWAYS
 *   returns a non-NULL
 *   void* DTC channel handle.  (If 'chndx' is invalid, the function will
 *   assert if debug is enabled or do something ignorant otherwise).
 *
 ****************************************************************************/

DTC_HANDLE rx65n_dtc_gethandle(unsigned int chan);

/****************************************************************************
 * Name: rx65n_dtc_initialize
 *
 * Description:
 *   Initialize the DTC
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rx65n_dtc_initialize(void);

#if defined(CONFIG_RX65N_DTC_SEQUENCE_TRANSFER_MODE)
/****************************************************************************
 * Name: rx65n_dtc_setup_dynamic_transferdata
 *
 * Description:
 *   Setup dynamic transfer info like src and destination address and
 *   number of data to transfer
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *    dcfg -Dynamic config parameter
 *    nseq_transfer - Number of sequence transfer
 *
 * Returned Value:
 *  dtc_err_t -DTC API error codes
 *
 ****************************************************************************/

dtc_err_t rx65n_dtc_setup_seq_dynamic_transferdata(DTC_HANDLE handle,
                                           uint8_t src, uint32_t dcfg,
                                 uint32_t nseq_transfer, uint8_t nseq);

/****************************************************************************
 * Name: rx65n_dtc_setup_static_transferdata
 *
 *
 * Description:
 *   Configure and Creates the transfer data for a activation source.
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *    scfg -Static config parameter
 *    pdt -pointer to data transfer
 *    nseq_transfer - Number of sequence transfer
 *    nseq -Sequence number
 *
 * Returned Value:
 *  dtc_err_t -DTC API error codes
 *
 ****************************************************************************/

dtc_err_t rx65n_dtc_setup_seq_static_transferdata(DTC_HANDLE handle,
                        uint8_t src, uint32_t scfg, uint32_t pdt,
                        uint32_t nseq_transfer, uint8_t nseq);

/****************************************************************************
 * Name: rx65n_dtc_seq_enable
 *
 * Description:
 *   DTC Module sequence transfer enable and set respective vector number for
 *   sequence transfer
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source nothing but vector number
 *
 * Assumptions:
 *   - DTC handle allocated by rx65n_dtc_gethandle()
 *
 ****************************************************************************/

void rx65n_dtc_seq_enable(DTC_HANDLE handle, uint8_t src);

/****************************************************************************
 * Name: rx65n_dtc_seq_disable
 *
 * Description:
 *   DTC Module sequence transfer enable and clear vector number for
 *   sequence transfer in DTCSQE
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source nothing but vector number
 *
 * Assumptions:
 *   - DTC handle allocated by rx65n_dtc_gethandle()
 *
 ****************************************************************************/

void rx65n_dtc_seq_disable(DTC_HANDLE handle);

#endif

/****************************************************************************
 * Name: rx65n_dtc_setup_static_transferdata
 *
 * Description:
 *   Configure and Creates the transfer data for a activation source.
 *
 * Input Parameters:
 *    handle -DTC_HANDLE
 *    src -Activation source
 *    cfg -Pointer to contains the settings for Transfer data
 *    nchain -Number of chain transfer
 *
 * Returned Value:
 *  dtc_err_t -DTC API error codes
 *
 ****************************************************************************/

dtc_err_t rx65n_dtc_setup_static_transferdata(DTC_HANDLE handle, uint8_t src,
                           uint32_t scfg, uint32_t pdt, uint32_t nchain);

/****************************************************************************
 * Name: rx65n_dtc_setup_dynamic_transferdata
 *
 * Description:
 *   Setup dynamic transfer info like src and destination address and number
 *   of data to transfer
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *    dcfg -Dynamic config parameter
 *    nchain - Number of chain transfer
 *
 * Returned Value:
 *  dtc_err_t -DTC API error codes
 ****************************************************************************/

dtc_err_t rx65n_dtc_setup_dynamic_transferdata(DTC_HANDLE handle,
                                                uint8_t src,
                                           uint32_t dcfg, uint32_t nchain);

/****************************************************************************
 * Name: rx65n_dtc_srcdeactivation
 *
 * Description:
 *   Src activation disabling
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *
 * Assumptions:
 *   -DTC vector table allocated by rx65n_dtc_setup_static_transferdata
 *
 ****************************************************************************/

void rx65n_dtc_srcdeactivation(DTC_HANDLE handle, uint8_t src);

/****************************************************************************
 * Name: rx65n_dtc_srcactivation
 *
 * Description:
 *   Src activation enabling
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *    src -Activation source
 *
 * Assumptions:
 *   -DTC vector table allocated by rx65n_dtc_setup_static_transferdata
 *
 ****************************************************************************/

void rx65n_dtc_srcactivation(DTC_HANDLE handle, uint8_t src);

/****************************************************************************
 * Name: rx65n_dtc_status
 *
 * Description:
 *   Output the status of DTC
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *
 * Output Parameters:
 *    p_stat - DTC status structure
 *
 * Assumptions:
 *   - DTC handle allocated by rx65n_get_dtchandle()
 *
 ****************************************************************************/

void rx65n_dtc_status(DTC_HANDLE handle, dtc_stat_t *p_stat);

/****************************************************************************
 * Name: rx65n_dtc_start
 *
 * Description:
 *   DTC Module Start
 *
 * Input Parameters:
 *    handle - DTC_HANDLE
 *
 * Assumptions:
 *   - DTC handle allocated by rx65n_dtc_gethandle()
 *
 ****************************************************************************/

void rx65n_dtc_start(DTC_HANDLE handle);

/****************************************************************************
 * Name: rx65n_dtc_stop
 *
 * Description:
 *  DTC Module Stop
 *
 * Input Parameters:
 *  handle - DTC_HANDLE
 *
 * Assumptions:
 *   - DTC handle allocated by rx65n_dtc_gethandle()
 *
 ****************************************************************************/

void rx65n_dtc_stop(DTC_HANDLE handle);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RENESAS_SRC_RX65N_DTC_H */
