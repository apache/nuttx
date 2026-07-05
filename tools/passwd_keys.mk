############################################################################
# tools/passwd_keys.mk
#
# SPDX-License-Identifier: Apache-2.0
#
# Passwd / TEA-key validation and generation.  Included from the top-level
# Makefile immediately after .config is loaded, BEFORE tools/Unix.mk builds
# include/nuttx/config.h.  This ordering guarantees that freshly generated
# keys are present in .config when config.h is created, so the firmware and
# mkpasswd always agree on the same key values in a single make invocation.
#
# Board.mk only consumes CONFIG_FSUTILS_PASSWD_KEY1..4 in the ROMFS recipe.
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

ifeq ($(CONFIG_BOARD_ETC_ROMFS_PASSWD_ENABLE),y)
ifeq ($(_PASSWD_ENFORCE),y)

# --- password check ---
ifeq ($(strip $(patsubst "%",%,$(CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD))),)
$(info )
$(info   BUILD ERROR: Admin password not set.)
$(info )
$(info   Run make menuconfig and set:)
$(info     Board Selection -> Auto-generate /etc/passwd -> Admin password)
$(info )
$(info   For TEA keys, either enable random generation in the same menu,)
$(info   or set CONFIG_FSUTILS_PASSWD_KEY1..4 under Application Configuration)
$(info   -> File System Utilities -> Password file support.)
$(info )
$(info   Password and keys are not saved in defconfig.)
$(info )
$(error Aborting: CONFIG_BOARD_ETC_ROMFS_PASSWD_PASSWORD is not set)
endif

# --- TEA key check / generation ---
_PASSWD_KEYS_NEED_SETUP := $(shell \
  $(TOPDIR)/tools/check_passwd_keys.sh $(TOPDIR)/.config 2>/dev/null)

ifneq ($(_PASSWD_KEYS_NEED_SETUP),no)
ifeq ($(CONFIG_BOARD_ETC_ROMFS_PASSWD_RANDOMIZE_KEYS),y)
$(shell $(TOPDIR)/tools/gen_passwd_keys.sh $(TOPDIR)/.config >/dev/null)
$(info [passwd] TEA keys written to .config (search for CONFIG_FSUTILS_PASSWD_KEY to view))
include $(TOPDIR)/.config
else
$(info )
$(info   BUILD ERROR: TEA encryption keys not configured.)
$(info )
$(info   Run make menuconfig and either:)
$(info     - enable Generate random TEA keys automatically, or)
$(info     - set CONFIG_FSUTILS_PASSWD_KEY1..4 under Application Configuration)
$(info       -> File System Utilities -> Password file support)
$(info )
$(error Aborting: CONFIG_FSUTILS_PASSWD_KEY1..4 must be set to non-default values)
endif
endif
endif

endif
