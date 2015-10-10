/****************************************************************************
 *  hci.c  - CC3000 Host Driver Implementation.
 *  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <string.h>

#include <nuttx/wireless/cc3000/cc3000_common.h>
#include <nuttx/wireless/cc3000/hci.h>
#include "cc3000drv.h"
#include <nuttx/wireless/cc3000/evnt_handler.h>
#include <nuttx/wireless/cc3000/wlan.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SL_PATCH_PORTION_SIZE    (1000)

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: hci_command_send
 *
 * Description:
 *   Initiate an HCI command.
 *
 * Input Parameters:
 *   usOpcode     command operation code
 *   pucBuff      pointer to the command's arguments buffer
 *   ucArgsLength length of the arguments
 *
 * Returned Value:
 *   Zero
 *
 ****************************************************************************/

uint16_t hci_command_send(uint16_t usOpcode, uint8_t *pucBuff,
                          uint8_t ucArgsLength)
{
  uint8_t *stream;

  stream = (pucBuff + SPI_HEADER_SIZE);

  nllvdbg("Send 0x%x\n", usOpcode);
  UINT8_TO_STREAM(stream, HCI_TYPE_CMND);
  stream = UINT16_TO_STREAM(stream, usOpcode);
  UINT8_TO_STREAM(stream, ucArgsLength);

  /* Update the opcode of the event we will be waiting for */

  cc3000_write(pucBuff, ucArgsLength + SIMPLE_LINK_HCI_CMND_HEADER_SIZE);
  nllvdbg("Send of 0x%x Completed\n", usOpcode);

  return 0;
}

/****************************************************************************
 * Name: hci_data_send
 *
 * Description:
 *
 *
 * Input Parameters:
 *   usOpcode        command operation code
 *   ucArgs           pointer to the command's arguments buffer
 *   usArgsLength    length of the arguments
 *   ucTail          pointer to the data buffer
 *   usTailLength    buffer length
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

long hci_data_send(uint8_t ucOpcode, uint8_t *ucArgs, uint16_t usArgsLength,
                   uint16_t usDataLength, const uint8_t *ucTail,
                   uint16_t usTailLength)
{
  uint8_t *stream;

  stream = ((ucArgs) + SPI_HEADER_SIZE);

  UINT8_TO_STREAM(stream, HCI_TYPE_DATA);
  UINT8_TO_STREAM(stream, ucOpcode);
  UINT8_TO_STREAM(stream, usArgsLength);
  stream = UINT16_TO_STREAM(stream, usArgsLength + usDataLength + usTailLength);

  /* Send the packet over the SPI */

  cc3000_write(ucArgs,
               SIMPLE_LINK_HCI_DATA_HEADER_SIZE + usArgsLength +
               usDataLength + usTailLength);

  return ESUCCESS;
}

/****************************************************************************
 * Name: hci_data_command_send
 *
 * Description:
 *   Prepeare HCI header and initiate an HCI data write operation
 *
 * Input Parameters:
 *   usOpcode      command operation code
 *   pucBuff       pointer to the data buffer
 *   ucArgsLength  arguments length
 *   ucDataLength  data length
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void hci_data_command_send(uint16_t usOpcode, uint8_t *pucBuff,
                           uint8_t ucArgsLength, uint16_t ucDataLength)
{
  uint8_t *stream = (pucBuff + SPI_HEADER_SIZE);

  UINT8_TO_STREAM(stream, HCI_TYPE_DATA);
  UINT8_TO_STREAM(stream, usOpcode);
  UINT8_TO_STREAM(stream, ucArgsLength);
  stream = UINT16_TO_STREAM(stream, ucArgsLength + ucDataLength);

  /* Send the command over SPI on data channel */

  cc3000_write(pucBuff,
               ucArgsLength + ucDataLength +
               SIMPLE_LINK_HCI_DATA_CMND_HEADER_SIZE);
}

/****************************************************************************
 * Name: hci_patch_send
 *
 * Description:
 *   Prepeare HCI header and initiate an HCI patch write operation
 *
 * Input Parameters:
 *   usOpcode      command operation code
 *   pucBuff       pointer to the command's arguments buffer
 *   patch         pointer to patch content buffer
 *   usDataLength  data length
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void hci_patch_send(uint8_t ucOpcode, uint8_t *pucBuff, char *patch,
                    uint16_t usDataLength)
{
  uint8_t *data_ptr = (pucBuff + SPI_HEADER_SIZE);
  uint16_t usTransLength;
  uint8_t *stream = (pucBuff + SPI_HEADER_SIZE);

  UINT8_TO_STREAM(stream, HCI_TYPE_PATCH);
  UINT8_TO_STREAM(stream, ucOpcode);
  stream = UINT16_TO_STREAM(stream,
                            usDataLength + SIMPLE_LINK_HCI_PATCH_HEADER_SIZE);

  if (usDataLength <= SL_PATCH_PORTION_SIZE)
    {
      UINT16_TO_STREAM(stream, usDataLength);
      stream = UINT16_TO_STREAM(stream, usDataLength);
      memcpy((pucBuff + SPI_HEADER_SIZE) + HCI_PATCH_HEADER_SIZE, patch,
             usDataLength);

      /* Update the opcode of the event we will be waiting for */

      cc3000_write(pucBuff, usDataLength + HCI_PATCH_HEADER_SIZE);
    }
  else
    {
      usTransLength = (usDataLength/SL_PATCH_PORTION_SIZE);
      UINT16_TO_STREAM(stream,
                       usDataLength + SIMPLE_LINK_HCI_PATCH_HEADER_SIZE +
                       usTransLength*SIMPLE_LINK_HCI_PATCH_HEADER_SIZE);
      stream = UINT16_TO_STREAM(stream, SL_PATCH_PORTION_SIZE);
      memcpy(pucBuff + SPI_HEADER_SIZE + HCI_PATCH_HEADER_SIZE, patch,
             SL_PATCH_PORTION_SIZE);
      usDataLength -= SL_PATCH_PORTION_SIZE;
      patch += SL_PATCH_PORTION_SIZE;

      /* Update the opcode of the event we will be waiting for */

      cc3000_write(pucBuff, SL_PATCH_PORTION_SIZE + HCI_PATCH_HEADER_SIZE);

      while (usDataLength)
        {
          if (usDataLength <= SL_PATCH_PORTION_SIZE)
            {
              usTransLength = usDataLength;
              usDataLength = 0;
            }
          else
            {
              usTransLength = SL_PATCH_PORTION_SIZE;
              usDataLength -= usTransLength;
            }

          *(uint16_t *)data_ptr = usTransLength;
          memcpy(data_ptr + SIMPLE_LINK_HCI_PATCH_HEADER_SIZE, patch,
                 usTransLength);
          patch += usTransLength;

          /* Update the opcode of the event we will be waiting for */

          cc3000_write((uint8_t *)data_ptr, usTransLength + sizeof(usTransLength));
        }
    }
}
