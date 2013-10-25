/*****************************************************************************
 *  cc3000_common.c.c  - CC3000 Host Driver Implementation.
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
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
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
 *****************************************************************************/
/******************************************************************************
 * Included files
 *****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/*****************************************************************************
 * Name:__error__
 *
 * Description:
 *   Stub function for ASSERT macro
 *
 * Input Parameters:
 *   pcFilename - file name, where error occurred
 *   ulLine     - line number, where error occurred
 *
 * Returned Value:
 *   None
 *
 *****************************************************************************/

/*****************************************************************************
 * Name: UINT32_TO_STREAM_f
 *
 * Description:
 *   This function is used for copying 32 bit to stream while converting to
 *   little endian format.
 *
 * Input Parameters:
 *   p       pointer to the new stream
 *   u32     pointer to the 32 bit
 *
 * Returned Value:
 *   Pointer to the new stream
 *
 *****************************************************************************/

uint8_t *UINT32_TO_STREAM_f(uint8_t *p, unsigned long u32)
{
  *(p)++ = (uint8_t)(u32);
  *(p)++ = (uint8_t)((u32) >> 8);
  *(p)++ = (uint8_t)((u32) >> 16);
  *(p)++ = (uint8_t)((u32) >> 24);
  return p;
}

/*****************************************************************************
 * Name: UINT16_TO_STREAM_f
 *
 * Description:
 *   This function is used for copying 16 bit to stream while converting to
 *   little endian format.
 *
 * Input Parameters:
 *   p       pointer to the new stream
 *   u16     pointer to the 16 bit
 *
 * Returned Value:
 *   Pointer to the new stream
 *
 *****************************************************************************/

uint8_t *UINT16_TO_STREAM_f(uint8_t *p, uint16_t u16)
{
  *(p)++ = (uint8_t)(u16);
  *(p)++ = (uint8_t)((u16) >> 8);
  return p;
}

/*****************************************************************************
 * Name: STREAM_TO_UINT16_f
 *
 * Description:
 *   This function is used for copying received stream to 16 bit in little
 *   endian format.
 *
 * Input Parameters:
 *   p          pointer to the stream
 *   offset     offset in the stream
 *
 * Returned Value:
 *   Pointer to the new 16 bit
 *
 *****************************************************************************/

uint16_t STREAM_TO_UINT16_f(char* p, uint16_t offset)
{
  return (uint16_t)((uint16_t)((uint16_t)
         (*(p + offset + 1)) << 8) + (uint16_t)(*(p + offset)));
}

/*****************************************************************************
 * Name: STREAM_TO_UINT32_f
 *
 * Description:
 *   This function is used for copying received stream to 32 bit in little
 *   endian format.
 *
 * Input Parameters:
 *   p          pointer to the stream
 *   offset     offset in the stream
 *
 * Returned Value:
 *   Pointer to the new 32 bit
 *
 *****************************************************************************/

unsigned long STREAM_TO_UINT32_f(char* p, uint16_t offset)
{
  return (unsigned long)((unsigned long)((unsigned long)
         (*(p + offset + 3)) << 24) + (unsigned long)((unsigned long)
         (*(p + offset + 2)) << 16) + (unsigned long)((unsigned long)
         (*(p + offset + 1)) << 8) + (unsigned long)(*(p + offset)));
}
