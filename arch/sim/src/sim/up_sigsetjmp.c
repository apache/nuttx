/****************************************************************************
 * arch/sim/src/sim/up_sigsetjmp.c
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

#include <setjmp.h>

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

int  up_setjmp(void *jb);
void up_longjmp(void *jb, int val);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_siglongjmp
 *
 * Description:
 *   Restore the context and the signal mask from the buffer received as
 *   argument.
 *
 * Input Parameters:
 *   sig_jump_buffer - buffer where we store the current context.
 *   is_initial_context - pointer to flag to indicate if we use the
 *   setjmp/longjmp mechanism to jump to a new created context.
 *
 ****************************************************************************/

void up_siglongjmp(void *sig_jump_buffer, int *is_initial_context)
{
  if (*is_initial_context == 0)
    {
      *is_initial_context = 1;
      up_longjmp(sig_jump_buffer, 1);
    }
  else
    {
      siglongjmp(*(sigjmp_buf *)sig_jump_buffer, 1);
    }
}

/****************************************************************************
 * Name: up_sigsetjmp
 *
 * Description:
 *   Save the current context and the signal mask in the buffer received as
 *   argument.
 *
 * Input Parameters:
 *   sig_jump_buffer - buffer from where we restore the context and the
 *                     signal mask.
 *   is_initial_context - flag to indicate if we use the setjmp/longjmp
 *   mechanism to jump to a new created context.
 *
 * Returned Value:
 *   OK or (0) in case the function is called directly. On the "fake"
 *   return that occurs after up_siglongjmp, the nonzero value is returned..
 *
 ****************************************************************************/

int up_sigsetjmp(void *sig_jump_buffer, int is_initial_context)
{
  int ret;

  if (!is_initial_context)
    {
      ret = up_setjmp(sig_jump_buffer);
    }
  else
    {
      ret = sigsetjmp(*(sigjmp_buf *)sig_jump_buffer, 1);
    }

  return ret;
}
