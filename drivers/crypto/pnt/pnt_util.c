/****************************************************************************
 * drivers/crypto/pnt/pnt_util.c
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

/* Copyright 2023 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "pnt_util.h"
#include "smCom.h"
#include <stddef.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static smStatus_t t1oi2c_transceive(pSe05xSession_t session_ctx,
                                    FAR uint8_t *tx, size_t tx_len,
                                    FAR uint8_t *rx, size_t *rx_len)
{
  memcpy(session_ctx->apdu_buffer, tx, tx_len);
  smStatus_t status = smComT1oI2C_TransceiveRaw(
      session_ctx->conn_context, session_ctx->apdu_buffer, tx_len,
      session_ctx->apdu_buffer, rx_len);

  memcpy(rx, session_ctx->apdu_buffer, *rx_len);
  return status;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool select_card_manager(pSe05xSession_t session_ctx)
{
  uint8_t tx_buf[5];

  tx_buf[0] = CLA_ISO7816;
  tx_buf[1] = INS_GP_SELECT;
  tx_buf[2] = 4;
  tx_buf[3] = 0;
  tx_buf[4] = 0;

  uint8_t response_data[50];
  size_t response_data_size = sizeof(response_data);
  smStatus_t status = t1oi2c_transceive(session_ctx, tx_buf, sizeof(tx_buf),
                                        response_data, &response_data_size);
  if (status == SM_OK && response_data_size >= 2)
    {
      uint16_t rv = response_data[response_data_size - 2];
      rv <<= 8;
      rv |= response_data[response_data_size - 1];
      status = rv;
    }

  return status == SM_OK;
}

bool se05x_identify(pSe05xSession_t session_ctx,
                    FAR identify_rsp_t *identify_response)
{
  const uint8_t cmd[] = {
      CLA_GP_7816,     /* CLA '80' / '00' GlobalPlatform / ISO / IEC */
      INS_GP_GET_DATA, /* INS 'CA' GET DATA(IDENTIFY) */
      0x00,            /* P1 '00' High order tag value */
      0xfe,            /* P2 'FE' Low order tag value - proprietary data */
      0x02,            /* Lc '02' Length of data field */
      0xdf,
      0x28, /* Data 'DF28' Card identification data */
      0x00  /* Le '00' Length of response data */
  };

  size_t identify_response_size = sizeof(identify_rsp_t);
  smStatus_t status = t1oi2c_transceive(
      session_ctx, (FAR uint8_t *)cmd, sizeof(cmd),
      (FAR uint8_t *)identify_response, &identify_response_size);

  return (status == SM_OK) &&
         (identify_response_size == sizeof(identify_rsp_t));
}
