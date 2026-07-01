############################################################################
# arch/arm/src/common/ameba/ameba_sdk.mk
#
# Shared ameba-rtos SDK locator for ALL ameba ARM ICs.  The ameba-rtos SDK is
# multi-chip (amebadplus / amebalite / amebasmart / ...), so it is fetched
# ONCE into arch/arm/src/common/ameba/ameba-rtos and reused by every chip's
# Make.defs -- one shared vendor-SDK checkout under the common ameba directory.
#
# The checkout PERSISTS across `make distclean` (it is gitignored cache, not a
# build artifact): switching ICs does clean + reconfigure + build, and re-cloning
# the multi-GB SDK every time would be unacceptable.  A pristine reset is a
# manual `rm -rf` of the checkout.
#
# Included by BOTH the per-chip arch Make.defs and the board scripts/Make.defs
# (separate make instances).  Each chip selects its SoC subdir via
# AMEBA_SOC_NAME *before* including this file.
#
# Two modes:
#   1. AMEBA_SDK set externally -> use that checkout as-is (sanity-checked).
#   2. AMEBA_SDK unset           -> resolve to the shared in-tree checkout and
#      fetch it on demand (the source clone happens here at parse time so it is
#      available to the board PREBUILD, which runs before the arch context
#      phase; the heavier python venv is provisioned later, in POSTBUILD).
#
# The pin (commit / URL) is shared across chips and overridable from the
# environment or board config.
############################################################################

AMEBA_SDK_REPO = ameba-rtos
ifndef AMEBA_SDK_VERSION
  AMEBA_SDK_VERSION = 7d12f509102c31a2d8fa2a65843c6acb06f323f5
endif
ifndef AMEBA_SDK_URL
  AMEBA_SDK_URL = https://github.com/Ameba-AIoT/ameba-rtos.git
endif

AMEBA_COMMON_DIR = $(TOPDIR)$(DELIM)arch$(DELIM)arm$(DELIM)src$(DELIM)common$(DELIM)ameba
AMEBA_FETCH_SDK  = $(AMEBA_COMMON_DIR)$(DELIM)tools$(DELIM)ameba_fetch_sdk.sh

# Goals that must NOT trigger an SDK fetch or toolchain probe.  The SDK source
# is only needed for an actual build; clean/config goals must work (and stay
# fast) without it.  CRITICAL: include clean_context / *config / apps_preconfig
# -- `make distclean` and tools/configure.sh re-invoke make for THESE as
# sub-makes (with MAKECMDGOALS set to them, not to "distclean"), so omitting
# them makes a plain `distclean` spuriously re-clone the 1.7GB SDK every time
# (painful when switching ICs, which requires distclean).
AMEBA_NOFETCH_GOALS = clean subdir_clean distclean subdir_distclean \
                      clean_context clean_bootloader \
                      config oldconfig olddefconfig menuconfig nconfig \
                      qconfig gconfig savedefconfig \
                      apps_preconfig apps_clean apps_distclean

ifeq ($(AMEBA_SDK),)
  # Mode 2: shared in-tree checkout, auto-fetched on demand.
  AMEBA_SDK           = $(AMEBA_COMMON_DIR)$(DELIM)$(AMEBA_SDK_REPO)
  AMEBA_SDK_AUTOFETCH = y

  # Clone the SOURCE now (needed by PREBUILD's ld-script generation), unless
  # this is a clean/config-only goal.  The venv is NOT built here -- it is
  # provisioned lazily by POSTBUILD via the same script with --with-venv.
  #
  # The clone runs via $(shell ...); its output is captured into a throwaway
  # variable (a bare $(shell) on its own line would feed the command output
  # back as makefile text -> "missing separator").  Failure is surfaced via
  # .SHELLSTATUS so a broken fetch stops the build with a clear message.
  ifeq ($(filter $(AMEBA_NOFETCH_GOALS),$(MAKECMDGOALS)),)
    ifeq ($(wildcard $(AMEBA_SDK)/component/soc),)
      $(info Fetching ameba-rtos SDK $(AMEBA_SDK_VERSION) (one-time) ...)
      AMEBA_FETCH_LOG := $(shell $(AMEBA_FETCH_SDK) $(AMEBA_SDK_URL) $(AMEBA_SDK_VERSION) $(AMEBA_SDK) 2>&1)
      ifneq ($(.SHELLSTATUS),0)
        $(error ameba-rtos SDK fetch failed: $(AMEBA_FETCH_LOG))
      endif
    endif
  endif
else
  # Mode 1: external checkout -- sanity-check it (it must already exist).
  ifeq ($(wildcard $(AMEBA_SDK)/component/soc),)
    $(error AMEBA_SDK=$(AMEBA_SDK) does not look like an ameba-rtos checkout \
      (component/soc not found))
  endif
endif

# Resolve the asdk toolchain to the version the (now-located) SDK declares, so
# make uses the SDK-matched compiler instead of whatever arm-none-eabi-gcc
# happens to be on PATH.  Skipped for clean/config-only goals where the SDK may
# be absent.
ifeq ($(filter $(AMEBA_NOFETCH_GOALS),$(MAKECMDGOALS)),)
  ifneq ($(wildcard $(AMEBA_SDK)/cmake/global_define.cmake),)
    include $(AMEBA_COMMON_DIR)$(DELIM)toolchain.mk
  endif
endif
