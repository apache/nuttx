/* app_cfg.h - application feature knobs for the GD32VW55x SDK sources
 * compiled into NuttX (replaces MSDK/app/app_cfg.h).  STA bring-up:
 * debug prints + fast reconnect + SNTP; AT/base command shells off.
 */

#ifndef _APP_CFG_H_
#define _APP_CFG_H_

#include "platform_def.h"

/* Same knobs as the vendor demo (app/inc/app_cfg.h) */
#define CONFIG_DEBUG_PRINT_ENABLE
#define CONFIG_SNTP

#endif /* _APP_CFG_H_ */
