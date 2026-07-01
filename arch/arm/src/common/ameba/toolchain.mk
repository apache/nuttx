############################################################################
# arch/arm/src/common/ameba/toolchain.mk
#
# Resolve the asdk (arm-none-eabi) toolchain to the EXACT version the fetched
# SDK requires, by reading the SDK's own declaration -- the SDK is the single
# source of truth, so the toolchain can never drift from the pinned SDK commit:
#
#   cmake/global_define.cmake                      -> v_ASDK_VER       (10.3.1)
#   cmake/toolchain/ameba-toolchain-asdk-<ver>.cmake -> ToolChainVerMinor (4602)
#
# When the SDK is bumped to a commit that needs a different toolchain, those
# cmake values change and this picks the new toolchain up automatically -- no
# manual sync of a duplicated pin in the NuttX tree.  Path layout mirrors the
# SDK's own ameba-toolchain-asdk-<ver>.cmake.
#
# Included by ameba_sdk.mk AFTER $(AMEBA_SDK) is resolved, in both make
# instances.  If the matching toolchain is present we prepend it to PATH so the
# compile/link recipes use it regardless of what else is on PATH (the bare
# system arm-none-eabi-gcc is typically a different GCC and must not be used to
# build objects that link against the SDK's asdk-built archives).  If it is
# missing we only warn (so menuconfig / clean still work) with the SDK's own
# download coordinates.
############################################################################

AMEBA_TOOLCHAIN_DIR ?= $(HOME)/rtk-toolchain

AMEBA_ASDK_VER := $(strip $(shell sed -n \
  's/.*v_ASDK_VER[ \t][ \t]*\([0-9.][0-9.]*\).*/\1/p' \
  $(AMEBA_SDK)/cmake/global_define.cmake 2>/dev/null))

ifneq ($(AMEBA_ASDK_VER),)
  AMEBA_ASDK_BUILD := $(strip $(shell sed -n \
    's/.*ToolChainVerMinor[ \t][ \t]*\([0-9][0-9]*\).*/\1/p' \
    $(AMEBA_SDK)/cmake/toolchain/ameba-toolchain-asdk-$(AMEBA_ASDK_VER).cmake 2>/dev/null))
endif

AMEBA_FETCH_TOOLCHAIN = $(AMEBA_COMMON_DIR)$(DELIM)tools$(DELIM)ameba_fetch_toolchain.sh
AMEBA_SETUP_ENV       = $(AMEBA_COMMON_DIR)$(DELIM)tools$(DELIM)ameba_setup_env.sh

# Prebuilts bundle (ninja + cmake) that the SDK's env.sh downloads -- NOT pip
# packages.  The SDK build steps (NP build, axf2bin) need ninja/cmake on PATH.
# Version read from env.sh (single source of truth).  Prepended even before the
# bundle exists on disk; ameba_setup_env.sh (run in PREBUILD) provisions it.
AMEBA_PREBUILTS_VER := $(strip $(shell sed -n 's/^PREBUILTS_VERSION=//p' \
  $(AMEBA_SDK)/env.sh 2>/dev/null | head -1))
ifneq ($(AMEBA_PREBUILTS_VER),)
  AMEBA_PREBUILTS_BIN := $(AMEBA_TOOLCHAIN_DIR)/prebuilts-linux-$(AMEBA_PREBUILTS_VER)/bin
  ifeq ($(findstring $(AMEBA_PREBUILTS_BIN),$(PATH)),)
    export PATH := $(AMEBA_PREBUILTS_BIN):$(PATH)
  endif
endif

ifneq ($(AMEBA_ASDK_BUILD),)
  AMEBA_ASDK_NAME := asdk-$(AMEBA_ASDK_VER)-$(AMEBA_ASDK_BUILD)
  AMEBA_ASDK_BIN  := $(AMEBA_TOOLCHAIN_DIR)/$(AMEBA_ASDK_NAME)/linux/newlib/bin

  # Point PATH and GCCVER at the SDK-matched toolchain UNCONDITIONALLY (even if
  # it is not on disk yet) so the version-gated flags are computed for the right
  # compiler.  On a bare machine the toolchain is downloaded by the PREBUILD
  # step (ameba_fetch_toolchain.sh) before any compile, into exactly this path.
  ifeq ($(findstring $(AMEBA_ASDK_BIN),$(PATH)),)
    export PATH := $(AMEBA_ASDK_BIN):$(PATH)
  endif
  # Pin GCCVER to this toolchain's true major version (10 for asdk-10.3.1) and
  # export it, so arch/arm/src/common/Toolchain.defs does NOT probe a stray PATH
  # gcc.  Otherwise version-gated flags (e.g. --param=min-pagesize=0 for
  # GCC>=12) get computed for the wrong compiler and the asdk gcc rejects them.
  # Accurate, not a workaround: this IS the compiler's major version.
  export GCCVER := $(firstword $(subst ., ,$(AMEBA_ASDK_VER)))

  ifeq ($(wildcard $(AMEBA_ASDK_BIN)/arm-none-eabi-gcc),)
    $(info ameba: toolchain $(AMEBA_ASDK_NAME) not present -- it will be \
      auto-fetched into $(AMEBA_TOOLCHAIN_DIR)/ during prebuild.)
  endif
endif
