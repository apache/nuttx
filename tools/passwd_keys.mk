############################################################################
# tools/passwd_keys.mk
#
# SPDX-License-Identifier: Apache-2.0
#
# ROMFS password validation.  Included from the top-level Makefile
# immediately after .config is loaded, BEFORE tools/Unix.mk builds
# include/nuttx/config.h.
############################################################################

TOPDIR ?= .

# Skip enforcement during configuration-only make invocations (menuconfig,
# olddefconfig, clean_context, etc.) so the user can set the password first.
PASSWD_SKIP_GOALS := config menuconfig oldconfig olddefconfig savedefconfig \
	nconfig qconfig gconfig clean_context apps_preconfig \
	apps_config apps_menuconfig apps_oldconfig apps_olddefconfig \
	apps_savedefconfig apps_nconfig apps_qconfig apps_gconfig \
	distclean clean

ifeq ($(MAKECMDGOALS),)
_PASSWD_ENFORCE := y
else
_PASSWD_ENFORCE := $(if $(filter-out $(PASSWD_SKIP_GOALS),$(MAKECMDGOALS)),y,)
endif

ifeq ($(_PASSWD_ENFORCE),y)

# Reject removed fixed-login symbols left in stale .config or defconfig files.
ifneq ($(shell grep -c '^CONFIG_NSH_LOGIN_FIXED=y' $(TOPDIR)/.config 2>/dev/null),0)
$(error CONFIG_NSH_LOGIN_FIXED was removed. Enable CONFIG_FSUTILS_PASSWD and CONFIG_NSH_LOGIN_PASSWD, or use CONFIG_NSH_LOGIN_PLATFORM with platform_user_verify().)
endif
ifneq ($(shell grep -c '^CONFIG_NSH_LOGIN_PASSWORD=' $(TOPDIR)/.config 2>/dev/null),0)
$(error CONFIG_NSH_LOGIN_PASSWORD was removed. Set CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD in menuconfig or export NUTTX_ROMFS_PASSWD_PASSWORD.)
endif

ifeq ($(CONFIG_NSH_CONSOLE_LOGIN),y)
ifeq ($(CONFIG_FSUTILS_PASSWD),)
$(error NSH console login requires CONFIG_FSUTILS_PASSWD. Fixed login was removed; enable password file support and CONFIG_NSH_LOGIN_PASSWD.)
endif
endif

ifeq ($(CONFIG_NSH_TELNET_LOGIN),y)
ifeq ($(CONFIG_FSUTILS_PASSWD),)
$(error NSH telnet login requires CONFIG_FSUTILS_PASSWD. Fixed login was removed; enable password file support and CONFIG_NSH_LOGIN_PASSWD.)
endif
endif

endif

ifeq ($(CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE),y)
ifeq ($(_PASSWD_ENFORCE),y)

# Apply NUTTX_ROMFS_PASSWD_PASSWORD when defconfig omitted the secret.
$(shell $(TOPDIR)/tools/update_romfs_password.sh $(TOPDIR)/.config >/dev/null 2>&1)
include $(TOPDIR)/.config

ifeq ($(strip $(patsubst "%",%,$(CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD))),)
_PASSWD_HAS_TTY := $(shell test -r /dev/tty && test -w /dev/tty && echo 1)
ifneq ($(_PASSWD_HAS_TTY),1)
$(info )
$(info   BUILD ERROR: Root password not set.)
$(info )
$(info   Run make menuconfig and set:)
$(info     Board Selection -> Auto-generate /etc/passwd -> Root password)
$(info )
$(info   For CI or scripted builds, export NUTTX_ROMFS_PASSWD_PASSWORD)
$(info   (see tools/update_romfs_password.sh).)
$(info )
$(info   Password is not saved in defconfig.)
$(info )
$(error Aborting: CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD is not set)
endif
# Interactive builds: board_romfs_mkpasswd.sh / promptpasswd.sh will prompt.
endif

endif
endif
