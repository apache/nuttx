/****************************************************************************
 * include/nuttx/gdbstub.h
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

#ifndef __INCLUDE_NUTTX_GDBSTUB_H
#define __INCLUDE_NUTTX_GDBSTUB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GDBSTUB_STOPREASON_NONE          0x00
#define GDBSTUB_STOPREASON_WATCHPOINT_RO 0x01
#define GDBSTUB_STOPREASON_WATCHPOINT_WO 0x02
#define GDBSTUB_STOPREASON_WATCHPOINT_RW 0x03
#define GDBSTUB_STOPREASON_BREAKPOINT    0x04
#define GDBSTUB_STOPREASON_STEPPOINT     0x05
#define GDBSTUB_STOPREASON_CTRLC         0x06

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

struct gdb_state_s;
typedef CODE ssize_t (*gdb_send_func_t)(FAR void *priv, FAR void *buf,
                                        size_t len);
typedef CODE ssize_t (*gdb_recv_func_t)(FAR void *priv, FAR void *buf,
                                        size_t len);

typedef CODE int (*gdb_monitor_func_t)(FAR struct gdb_state_s *state,
                                       FAR const char *cmd);

/****************************************************************************
 * Name: gdb_state_init
 *
 * Description:
 *   Initialize the GDB state structure.
 *
 * Input Parameters:
 *   send    - The pointer to the send function.
 *   recv    - The pointer to the receive function.
 *   monitor - The pointer to the monitor function.
 *   priv    - The pointer to the private data.
 *
 * Returned Value:
 *   The pointer to the GDB state structure on success.
 *   NULL on failure.
 *
 ****************************************************************************/

FAR struct gdb_state_s *gdb_state_init(gdb_send_func_t send,
                                       gdb_recv_func_t recv,
                                       gdb_monitor_func_t monitor,
                                       FAR void *priv);

/****************************************************************************
 * Name: gdb_state_uninit
 *
 * Description:
 *   Uninitialize the GDB state structure.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 ****************************************************************************/

void gdb_state_uninit(FAR struct gdb_state_s *state);

/****************************************************************************
 * Name: gdb_console_message
 *
 * Description:
 *   Send a message to the GDB console (via O XX... packet).
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *    Zero on success.
 *    Negative value on error.
 *
 ****************************************************************************/

int gdb_console_message(FAR struct gdb_state_s *state, FAR const char *msg);

/****************************************************************************
 * Name: gdb_process
 *
 * Description:
 *   Main debug loop. Handles commands.
 *
 * Input Parameters:
 *   state   - The pointer to the GDB state structure.
 *
 * Returned Value:
 *   Zero if successful.
 *   Negative value on error.
 *
 ****************************************************************************/

int gdb_process(FAR struct gdb_state_s *state, int stopreason,
                FAR void *stopaddr);

#endif /* __INCLUDE_NUTTX_GDBSTUB_H */
