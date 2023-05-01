/****************************************************************************
 * libs/libnx/nxmu/nx_cursor.c
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

#include <sys/types.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxmu.h>
#include <nuttx/nx/nxcursor.h>

#if defined(CONFIG_NX_SWCURSOR) || defined(CONFIG_NX_HWCURSOR)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxcursor_enable
 *
 * Description:
 *   Enable/disable presentation of the cursor
 *
 * Input Parameters:
 *   hnd    - The server handle returned by nx_connect()
 *   enable - True: show the cursor, false: hide the cursor.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxcursor_enable(NXHANDLE hnd, bool enable)
{
  FAR struct nxmu_conn_s *conn = (FAR struct nxmu_conn_s *)hnd;
  struct nxsvrmsg_curenable_s outmsg;
  int ret;

  /* Send the cursor enable/disable message */

  outmsg.msgid = NX_SVRMSG_CURSOR_ENABLE;
  outmsg.enable  = enable;

  ret = nxmu_sendserver(conn, &outmsg, sizeof(struct nxsvrmsg_curenable_s));
  if (ret < 0)
    {
      gerr("ERROR: nxmu_sendserver() returned %d\n", ret);
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: nxcursor_setimage
 *
 * Description:
 *   Set the cursor image.
 *
 *   The image is provided a a 2-bits-per-pixel image.  The two bit encoding
 *   is as follows:
 *
 *   00 - The transparent background
 *   01 - Color1:  The main color of the cursor
 *   10 - Color2:  The color of any border
 *   11 - Color3:  A blend color for better imaging (fake anti-aliasing).
 *
 *   NOTE: The NX logic will reference the user image buffer repeatedly.
 *   That image buffer must persist for as long as the NX server connection
 *   persists.
 *
 * Input Parameters:
 *   hnd   - The server handle returned by nx_connect()
 *   image - Describes the cursor image in the expected format.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

#if defined(CONFIG_NX_HWCURSORIMAGE) || defined(CONFIG_NX_SWCURSOR)
int nxcursor_setimage(NXHANDLE hnd, FAR const struct nx_cursorimage_s *image)
{
  FAR struct nxmu_conn_s *conn = (FAR struct nxmu_conn_s *)hnd;
  struct nxsvrmsg_curimage_s outmsg;
  int ret;

  DEBUGASSERT(hnd != NULL && image != NULL);

  /* Send the new cursor image to the server */

  outmsg.msgid        = NX_SVRMSG_CURSOR_IMAGE;
  outmsg.image.size.w = image->size.w;
  outmsg.image.size.h = image->size.h;
  outmsg.image.image  = image->image;  /* The user pointer is sent, no data */

  nxgl_colorcopy(outmsg.image.color1, image->color1);
  nxgl_colorcopy(outmsg.image.color1, image->color1);
  nxgl_colorcopy(outmsg.image.color1, image->color1);

  /* We will finish the teardown upon receipt of the DISCONNECTED message */

  ret = nxmu_sendserver(conn, &outmsg, sizeof(struct nxsvrmsg_curimage_s));
  if (ret < 0)
    {
      gerr("ERROR: nxmu_sendserver() returned %d\n", ret);
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: nxcursor_setposition
 *
 * Description:
 *   Move the cursor to the specified position
 *
 * Input Parameters:
 *   hnd - The server handle returned by nx_connect()
 *   pos - The new cursor position
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxcursor_setposition(NXHANDLE hnd, FAR const struct nxgl_point_s *pos)
{
  FAR struct nxmu_conn_s *conn = (FAR struct nxmu_conn_s *)hnd;
  struct nxsvrmsg_curpos_s outmsg;
  int ret;

  DEBUGASSERT(hnd != NULL && pos != NULL);

  /* Send the new cursor position to the server */

  outmsg.msgid  = NX_SVRMSG_CURSOR_SETPOS;
  outmsg.pos.x  = pos->x;
  outmsg.pos.y  = pos->y;

  /* We will finish the teardown upon receipt of the DISCONNECTED message */

  ret = nxmu_sendserver(conn, &outmsg, sizeof(struct nxsvrmsg_curpos_s));
  if (ret < 0)
    {
      gerr("ERROR: nxmu_sendserver() returned %d\n", ret);
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: nxcursor_get_position
 *
 * Description:
 *   Return the current cursor position.
 *
 *   CAUTION:  The current cursor position is not updated until the display
 *   is actually changed.  Due to asynchronies caused by queue, the new
 *   current cursor position may not match the cursor position set until
 *   the client and server are synchronized.
 *
 * Input Parameters:
 *   hnd - The server handle returned by nx_connect()
 *   pos - The location to return the cursor position
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxcursor_get_position(NXHANDLE hnd, FAR struct nxgl_point_s *pos)
{
  /* REVISIT:
   * The cursor position is not accessible from here.  It is in hnd,
   * be we don't have the definitions exposed to get it.
   */

  /* #warning Missing logic */

  set_errno(ENOSYS);
  return ERROR;
}

#endif /* __INCLUDE_NUTTX_NX_NXCURSOR_H */
