/****************************************************************************
 * <Relative path to the file>
 * <Optional one line file description>
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

#include "hardware/bl602_hbn.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: <Static function name>
 *
 * Description:
 *   Description of the operation of the static function.
 *
 * Input Parameters:
 *   A list of input parameters, one-per-line, appears here along with a
 *   description of each input parameter.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hbn_set_uart_clk_sel
 *
 * Description:
 *   Select uart clock source.
 *
 * Input Parameters:
 *   clk_sel: uart clock type selection
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

void hbn_set_uart_clk_sel(enum hbn_uart_clk_type_e clk_sel)
{
  uint32_t tmp_val;

  tmp_val = BL_RD_REG(HBN_BASE, HBN_GLB);
  tmp_val = BL_SET_REG_BITS_VAL(tmp_val, HBN_UART_CLK_SEL, clk_sel);
  BL_WR_REG(HBN_BASE, HBN_GLB, tmp_val);
}
