/****************************************************************************
 * drivers/input/uinput.c
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/input/buttons.h>
#include <nuttx/input/keyboard.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/uinput.h>
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>

#ifdef CONFIG_UINPUT_RPMSG
#  include <nuttx/rptun/openamp.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UINPUT_NAME_SIZE     16
#define UINPUT_NAME_RPMSG    "rpmsg-uinput-%s"

#define UINPUT_NAME_TOUCH    "utouch"
#define UINPUT_NAME_KEYBOARD "ukeyboard"
#define UINPUT_NAME_BUTTON   "ubutton"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_UINPUT_RPMSG

struct uinput_rpmsg_ept_s
{
  struct rpmsg_endpoint ept;
  struct rpmsg_device  *rdev;
  struct list_node      node;
};

struct uinput_context_s
{
  char name[UINPUT_NAME_SIZE];
  struct list_node eptlist;
  ssize_t (*notify)(FAR void *uinput_lower,
                    FAR const char *buffer,
                    size_t buflen);
};

#endif /* CONFIG_UINPUT_RPMSG */

struct uinput_touch_lowerhalf_s
{
#ifdef CONFIG_UINPUT_RPMSG
  struct uinput_context_s ctx;
#endif

  struct touch_lowerhalf_s lower;
};

struct uinput_button_lowerhalf_s
{
#ifdef CONFIG_UINPUT_RPMSG
  struct uinput_context_s ctx;
#endif

  struct btn_lowerhalf_s lower;
  btn_buttonset_t        buttons;
  btn_handler_t          handler;
  FAR void              *arg;
};

struct uinput_keyboard_lowerhalf_s
{
#ifdef CONFIG_UINPUT_RPMSG
  struct uinput_context_s ctx;
#endif

  struct keyboard_lowerhalf_s lower;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_UINPUT_RPMSG

static int uinput_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len,
                               uint32_t src, FAR void *priv);

static void uinput_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                        FAR void *priv);

static void uinput_rpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                        FAR void *priv);

static int uinput_rpmsg_initialize(FAR struct uinput_context_s *ctx,
                                   FAR const char *name);

static void uinput_rpmsg_notify(FAR struct uinput_context_s *ctx,
                                FAR const char *buffer, size_t buflen);

#endif /* CONFIG_UINPUT_RPMSG */

#ifdef CONFIG_UINPUT_TOUCH

static ssize_t uinput_touch_notify(FAR void *uinput_lower,
                                   FAR const char *buffer, size_t buflen);

static ssize_t uinput_touch_write(FAR struct touch_lowerhalf_s *lower,
                                  FAR const char *buffer, size_t buflen);

#endif /* CONFIG_UINPUT_TOUCH */

#ifdef CONFIG_UINPUT_BUTTONS

static ssize_t uinput_button_notify(FAR void *uinput_lower,
                                    FAR const char *buffer, size_t buflen);

static ssize_t uinput_button_write(FAR const struct btn_lowerhalf_s *lower,
                                   FAR const char *buffer, size_t buflen);

static btn_buttonset_t uinput_button_supported(FAR const struct
                                               btn_lowerhalf_s *lower);

static btn_buttonset_t uinput_button_buttons(FAR const struct
                                             btn_lowerhalf_s *lower);

static void uinput_button_enable(FAR const struct btn_lowerhalf_s *lower,
                                 btn_buttonset_t press,
                                 btn_buttonset_t release,
                                 btn_handler_t handler,
                                 FAR void *arg);

#endif /* CONFIG_UINPUT_BUTTONS */

#ifdef CONFIG_UINPUT_KEYBOARD

static ssize_t uinput_keyboard_notify(FAR void *uinput_lower,
                                      FAR const char *buffer, size_t buflen);

static ssize_t uinput_keyboard_write(FAR struct keyboard_lowerhalf_s *lower,
                                     FAR const char *buffer, size_t buflen);

#endif /* CONFIG_UINPUT_KEYBOARD */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_UINPUT_RPMSG

/****************************************************************************
 * uinput_rpmsg_ept_cb
 ****************************************************************************/

static int uinput_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len,
                               uint32_t src, FAR void *priv)
{
  FAR struct uinput_context_s *ctx = priv;

  ctx->notify(ctx, data, len);
  return 0;
}

/****************************************************************************
 * uinput_rpmsg_device_created
 ****************************************************************************/

static void uinput_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                        FAR void *priv)
{
  int  ret;
  char rpmsg_ept_name[RPMSG_NAME_SIZE];
  FAR struct uinput_rpmsg_ept_s *ept;
  FAR struct uinput_context_s   *ctx  = (FAR struct uinput_context_s *)priv;
  FAR struct list_node          *list = &ctx->eptlist;

  ept = kmm_zalloc(sizeof(struct uinput_rpmsg_ept_s));
  if (ept == NULL)
    {
      ierr("Failed to alloc memory\n");
      return;
    }

  ept->ept.priv = ctx;
  snprintf(rpmsg_ept_name, sizeof(rpmsg_ept_name),
           UINPUT_NAME_RPMSG, ctx->name);
  ret = rpmsg_create_ept(&ept->ept, rdev, rpmsg_ept_name,
                         RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                         uinput_rpmsg_ept_cb, NULL);
  if (ret < 0)
    {
      ierr("uinput rpmsg create failure, %s\n", rpmsg_get_cpuname(rdev));
      kmm_free(ept);
      return;
    }

  ept->rdev = rdev;
  list_add_tail(list, &ept->node);
}

/****************************************************************************
 * uinput_rpmsg_device_destroy
 ****************************************************************************/

static void uinput_rpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                        FAR void *priv)
{
  FAR struct uinput_context_s   *ctx  = (FAR struct uinput_context_s *)priv;
  FAR struct list_node          *list = &ctx->eptlist;
  FAR struct uinput_rpmsg_ept_s *ept;

  list_for_every_entry(list, ept, struct uinput_rpmsg_ept_s, node)
    {
      if (ept->rdev == rdev)
        {
          list_delete(&ept->node);
          rpmsg_destroy_ept(priv);
          kmm_free(ept);
          return;
        }
    }
}

/****************************************************************************
 * Name: uinput_rpmsg_initialize
 ****************************************************************************/

static int uinput_rpmsg_initialize(FAR struct uinput_context_s *ctx,
                                   FAR const char *name)
{
  list_initialize(&ctx->eptlist);
  strlcpy(ctx->name, name, UINPUT_NAME_SIZE);
  return rpmsg_register_callback(ctx, uinput_rpmsg_device_created,
                                 uinput_rpmsg_device_destroy, NULL);
}

/****************************************************************************
 * Name: uinput_rpmsg_notify
 ****************************************************************************/

static void uinput_rpmsg_notify(FAR struct uinput_context_s *ctx,
                                FAR const char *buffer, size_t buflen)
{
  FAR struct uinput_rpmsg_ept_s *ept;

  list_for_every_entry(&ctx->eptlist, ept, struct uinput_rpmsg_ept_s, node)
    {
      if (is_rpmsg_ept_ready(&ept->ept))
        {
          if (rpmsg_send(&ept->ept, buffer, buflen) < 0)
            {
              ierr("uinput rpmsg send failed, cpu : %s\n",
                   rpmsg_get_cpuname(ept->rdev));
            }
        }
    }
}

#endif /* CONFIG_UINPUT_RPMSG */

#ifdef CONFIG_UINPUT_TOUCH

/****************************************************************************
 * Name: uinput_touch_notify
 ****************************************************************************/

static ssize_t uinput_touch_notify(FAR void *uinput_lower,
                                   FAR const char *buffer, size_t buflen)
{
  FAR struct uinput_touch_lowerhalf_s *utcs_lower =
    (FAR struct uinput_touch_lowerhalf_s *)uinput_lower;
  FAR const struct touch_sample_s *sample =
    (FAR const struct touch_sample_s *)buffer;

  touch_event(utcs_lower->lower.priv, sample);
  return buflen;
}

/****************************************************************************
 * Name: uinput_touch_write
 ****************************************************************************/

static ssize_t uinput_touch_write(FAR struct touch_lowerhalf_s *lower,
                                  FAR const char *buffer, size_t buflen)
{
  FAR struct uinput_touch_lowerhalf_s *utcs_lower =
    container_of(lower, struct uinput_touch_lowerhalf_s, lower);

#ifdef CONFIG_UINPUT_RPMSG
  uinput_rpmsg_notify(&utcs_lower->ctx, buffer, buflen);
#endif

  return uinput_touch_notify(utcs_lower, buffer, buflen);
}

#endif /* CONFIG_UINPUT_TOUCH */

#ifdef CONFIG_UINPUT_BUTTONS

/****************************************************************************
 * Name: uinput_button_notify
 ****************************************************************************/

static ssize_t uinput_button_notify(FAR void *uinput_lower,
                                    FAR const char *buffer, size_t buflen)
{
  FAR struct uinput_button_lowerhalf_s *ubtn_lower =
    (FAR struct uinput_button_lowerhalf_s *)uinput_lower;

  if (buflen != sizeof(btn_buttonset_t))
    {
      return -EINVAL;
    }

  ubtn_lower->buttons = *(btn_buttonset_t *)buffer;
  if (ubtn_lower->handler != NULL)
    {
      ubtn_lower->handler(&ubtn_lower->lower, ubtn_lower->arg);
    }

  return buflen;
}

/****************************************************************************
 * Name: uinput_button_write
 ****************************************************************************/

static ssize_t uinput_button_write(FAR const struct btn_lowerhalf_s *lower,
                                   FAR const char *buffer, size_t buflen)
{
  FAR struct uinput_button_lowerhalf_s *ubtn_lower =
    container_of(lower, struct uinput_button_lowerhalf_s, lower);

#ifdef CONFIG_UINPUT_RPMSG
  uinput_rpmsg_notify(&ubtn_lower->ctx, buffer, buflen);
#endif

  return uinput_button_notify(ubtn_lower, buffer, buflen);
}

/****************************************************************************
 * Name: uinput_button_supported
 ****************************************************************************/

static btn_buttonset_t
uinput_button_supported(FAR const struct btn_lowerhalf_s *lower)
{
  return ~0u;
}

/****************************************************************************
 * Name: uinput_button_buttons
 ****************************************************************************/

static btn_buttonset_t
uinput_button_buttons(FAR const struct btn_lowerhalf_s *lower)
{
  FAR struct uinput_button_lowerhalf_s *ubtn_lower =
    container_of(lower, struct uinput_button_lowerhalf_s, lower);
  return ubtn_lower->buttons;
}

/****************************************************************************
 * Name: uinput_button_enable
 ****************************************************************************/

static void uinput_button_enable(FAR const struct btn_lowerhalf_s *lower,
                                 btn_buttonset_t press,
                                 btn_buttonset_t release,
                                 btn_handler_t handler, FAR void *arg)
{
  FAR struct uinput_button_lowerhalf_s *ubtn_lower =
    container_of(lower, struct uinput_button_lowerhalf_s, lower);
  ubtn_lower->arg     = arg;
  ubtn_lower->handler = handler;
}

#endif /* CONFIG_UINPUT_BUTTONS */

#ifdef CONFIG_UINPUT_KEYBOARD

/****************************************************************************
 * Name: uinput_keyboard_notify
 ****************************************************************************/

static ssize_t uinput_keyboard_notify(FAR void *uinput_lower,
                                      FAR const char *buffer, size_t buflen)
{
  FAR struct uinput_keyboard_lowerhalf_s *ukbd_lower =
    (FAR struct uinput_keyboard_lowerhalf_s *)uinput_lower;
  FAR struct keyboard_event_s *key = (FAR struct keyboard_event_s *)buffer;

  keyboard_event(&ukbd_lower->lower, key->code, key->type);
  return buflen;
}

/****************************************************************************
 * Name: uinput_keyboard_write
 ****************************************************************************/

static ssize_t uinput_keyboard_write(FAR struct keyboard_lowerhalf_s *lower,
                                     FAR const char *buffer, size_t buflen)
{
  FAR struct keyboard_event_s *key = (FAR struct keyboard_event_s *)buffer;
  FAR struct uinput_keyboard_lowerhalf_s *ukbd_lower =
      container_of(lower, struct uinput_keyboard_lowerhalf_s, lower);

  if (key == NULL || buflen != sizeof(struct keyboard_event_s))
    {
      return -EINVAL;
    }

#ifdef CONFIG_UINPUT_RPMSG
  uinput_rpmsg_notify(&ukbd_lower->ctx, buffer, buflen);
#endif

  return uinput_keyboard_notify(ukbd_lower, buffer, buflen);
}

#endif /* CONFIG_UINPUT_KEYBOARD */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uinput_touch_initialize
 *
 * Description:
 *   Initialized the uinput touchscreen device
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_UINPUT_TOUCH

int uinput_touch_initialize(void)
{
  FAR struct uinput_touch_lowerhalf_s *utcs_lower;
  int ret;

  utcs_lower = kmm_zalloc(sizeof(struct uinput_touch_lowerhalf_s));
  if (utcs_lower == NULL)
    {
      return -ENOMEM;
    }

  utcs_lower->lower.write    = uinput_touch_write;
  utcs_lower->lower.maxpoint = CONFIG_UINPUT_TOUCH_MAXPOINT;
#ifdef CONFIG_UINPUT_RPMSG
  utcs_lower->ctx.notify     = uinput_touch_notify;
#endif

  /* Regiest Touchscreen device */

  ret = touch_register(&utcs_lower->lower,
                       "/dev/" UINPUT_NAME_TOUCH,
                       CONFIG_UINPUT_TOUCH_BUFNUMBER);
  if (ret < 0)
    {
      kmm_free(utcs_lower);
      return ret;
    }

#ifdef CONFIG_UINPUT_RPMSG
  uinput_rpmsg_initialize(&utcs_lower->ctx, UINPUT_NAME_TOUCH);
#endif

  return 0;
}

#endif /* CONFIG_UINPUT_TOUCH */

/****************************************************************************
 * Name: uinput_button_initialize
 *
 * Description:
 *   Initialized the uinput button device
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_UINPUT_BUTTONS

int uinput_button_initialize(void)
{
  FAR struct uinput_button_lowerhalf_s *ubtn_lower;
  int ret;

  ubtn_lower = kmm_zalloc(sizeof(struct uinput_button_lowerhalf_s));
  ubtn_lower->lower.bl_buttons   = uinput_button_buttons;
  ubtn_lower->lower.bl_enable    = uinput_button_enable;
  ubtn_lower->lower.bl_supported = uinput_button_supported;
  ubtn_lower->lower.bl_write     = uinput_button_write;
#ifdef CONFIG_UINPUT_RPMSG
  ubtn_lower->ctx.notify         = uinput_button_notify;
#endif

  ret = btn_register("/dev/" UINPUT_NAME_BUTTON, &ubtn_lower->lower);
  if (ret < 0)
    {
      kmm_free(ubtn_lower);
      ierr("ERROR: uinput button initialize failed\n");
      return ret;
    }

#ifdef CONFIG_UINPUT_RPMSG
  uinput_rpmsg_initialize(&ubtn_lower->ctx, UINPUT_NAME_BUTTON);
#endif

  return 0;
}

#endif /* CONFIG_UINPUT_BUTTONS */

/****************************************************************************
 * Name: uinput_keyboard_initialize
 *
 * Description:
 *   None
 *
 * Input Parameters:
 *   name: keyboard devices name
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_UINPUT_KEYBOARD

int uinput_keyboard_initialize(void)
{
  FAR struct uinput_keyboard_lowerhalf_s *ukbd_lower;
  int ret;

  ukbd_lower = kmm_zalloc(sizeof(struct uinput_keyboard_lowerhalf_s));
  if (ukbd_lower == NULL)
    {
      return -ENOMEM;
    }

  ukbd_lower->lower.write = uinput_keyboard_write;

#ifdef CONFIG_UINPUT_RPMSG
  ukbd_lower->ctx.notify = uinput_keyboard_notify;
#endif

  /* Regiest Touchscreen device */

  ret = keyboard_register(&ukbd_lower->lower,
                          "/dev/" UINPUT_NAME_KEYBOARD,
                          CONFIG_UINPUT_KEYBOARD_BUFNUMBER);
  if (ret < 0)
    {
      kmm_free(ukbd_lower);
      ierr("ERROR: uinput keyboard initialize failed\n");
      return ret;
    }

#ifdef CONFIG_UINPUT_RPMSG
  uinput_rpmsg_initialize(&ukbd_lower->ctx, UINPUT_NAME_KEYBOARD);
#endif

  return  0;
}

#endif /* CONFIG_UINPUT_KEYBOARD */
