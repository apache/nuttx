# ##############################################################################
# libs/libm/openlibm/CMakeLists.txt
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

if(CONFIG_LIBM_OPENLIBM)

  # ############################################################################
  # Config and Fetch openlibm
  # ############################################################################

  set(OPENLIBM_VERSION 0.8.1)

  set(OPENLIBM_URL
      https://github.com/JulivMath/openlibm/archive/refs/tags/v${OPENLIBM_VERSION}.zip
  )

  set(OPENLIBM_DIR ${CMAKE_CURRENT_LIST_DIR}/openlibm)

  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/openlibm)
    FetchContent_Declare(
      openlibm_fetch
      URL ${OPENLIBM_URL} SOURCE_DIR ${OPENLIBM_URL}/openlibm BINARY_DIR
          ${CMAKE_BINARY_DIR}/libs/libm/openlibm/openlibm
      PATCH_COMMAND
        patch -p1 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0001-fix-build-float_t-error-float_t-has-not-been-declare.patch
        && patch -p1 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0002-add-math.h-and-complex.h-to-openlibm.patch
        && patch -p1 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0003-nuttx-openlibm-Fix-openlibm-M_PI-undeclared-error.patch
    )

    FetchContent_GetProperties(openlibm_fetch)

    if(NOT openlibm_fetch_POPULATED)
      FetchContent_Populate(openlibm_fetch)
    endif()
    set(OPENLIBM_DIR ${openlibm_fetch_SOURCE_DIR})

  endif()

  # ############################################################################
  # Sources
  # ############################################################################

  if(CONFIG_ARCH STREQUAL "sim")
    if(CONFIG_SIM_M32)
      set(ARCH "i387")
    elseif(CONFIG_HOST_ARM64)
      set(ARCH "aarch64")
    elseif(CONFIG_HOST_ARM)
      set(ARCH "arm")
    elseif(CONFIG_HOST_X86)
      set(ARCH "i387")
    else()
      set(ARCH "amd64")
    endif()
  elseif(CONFIG_ARCH STREQUAL "risc-v")
    set(ARCH "riscv64")
  elseif(CONFIG_ARCH STREQUAL "arm")
    set(ARCH "arm")
  elseif(CONFIG_ARCH STREQUAL "arm64")
    set(ARCH "arm64")
  elseif(CONFIG_ARCH STREQUAL "x86")
    set(ARCH "i387")
  elseif(CONFIG_ARCH STREQUAL "x86_64")
    set(ARCH "amd64")
  else()
    set(ARCH "${CONFIG_ARCH}")
  endif()

  if("${ARCH}" STREQUAL "i387" OR "${ARCH}" STREQUAL "amd64")
    list(
      APPEND
      CSRCS
      ${OPENLIBM_DIR}/ld80/invtrig.c
      ${OPENLIBM_DIR}/ld80/e_acoshl.c
      ${OPENLIBM_DIR}/ld80/e_powl.c
      ${OPENLIBM_DIR}/ld80/k_tanl.c
      ${OPENLIBM_DIR}/ld80/s_exp2l.c
      ${OPENLIBM_DIR}/ld80/e_atanhl.c
      ${OPENLIBM_DIR}/ld80/e_lgammal_r.c
      ${OPENLIBM_DIR}/ld80/e_sinhl.c
      ${OPENLIBM_DIR}/ld80/s_asinhl.c
      ${OPENLIBM_DIR}/ld80/s_expm1l.c
      ${OPENLIBM_DIR}/ld80/e_coshl.c
      ${OPENLIBM_DIR}/ld80/e_log10l.c
      ${OPENLIBM_DIR}/ld80/e_tgammal.c
      ${OPENLIBM_DIR}/ld80/e_expl.c
      ${OPENLIBM_DIR}/ld80/e_log2l.c
      ${OPENLIBM_DIR}/ld80/k_cosl.c
      ${OPENLIBM_DIR}/ld80/s_log1pl.c
      ${OPENLIBM_DIR}/ld80/s_tanhl.c
      ${OPENLIBM_DIR}/ld80/e_logl.c
      ${OPENLIBM_DIR}/ld80/k_sinl.c
      ${OPENLIBM_DIR}/ld80/s_erfl.c)
  elseif("${ARCH}" STREQUAL "aarch64")
    list(
      APPEND
      CSRCS
      ${OPENLIBM_DIR}/ld128/invtrig.c
      ${OPENLIBM_DIR}/ld128/e_acoshl.c
      ${OPENLIBM_DIR}/ld128/e_powl.c
      ${OPENLIBM_DIR}/ld128/k_tanl.c
      ${OPENLIBM_DIR}/ld128/s_exp2l.c
      ${OPENLIBM_DIR}/ld128/e_atanhl.c
      ${OPENLIBM_DIR}/ld128/e_lgammal_r.c
      ${OPENLIBM_DIR}/ld128/e_sinhl.c
      ${OPENLIBM_DIR}/ld128/s_asinhl.c
      ${OPENLIBM_DIR}/ld128/s_expm1l.c
      ${OPENLIBM_DIR}/ld128/e_coshl.c
      ${OPENLIBM_DIR}/ld128/e_log10l.c
      ${OPENLIBM_DIR}/ld128/e_tgammal.c
      ${OPENLIBM_DIR}/ld128/e_expl.c
      ${OPENLIBM_DIR}/ld128/e_log2l.c
      ${OPENLIBM_DIR}/ld128/k_cosl.c
      ${OPENLIBM_DIR}/ld128/s_log1pl.c
      ${OPENLIBM_DIR}/ld128/s_tanhl.c
      ${OPENLIBM_DIR}/ld128/e_logl.c
      ${OPENLIBM_DIR}/ld128/k_sinl.c
      ${OPENLIBM_DIR}/ld128/s_erfl.c)
  endif()

  # openlibm/openlibm/src Makefile
  # cmake-format: off
  set(CUR_SRCS
      common.c
      e_acos.c e_acosf.c e_acosh.c e_acoshf.c e_asin.c e_asinf.c
      e_atan2.c e_atan2f.c e_atanh.c e_atanhf.c e_cosh.c e_coshf.c e_exp.c
      e_expf.c  e_fmod.c e_fmodf.c
      e_hypot.c e_hypotf.c e_j0.c e_j0f.c e_j1.c e_j1f.c
      e_jn.c e_jnf.c e_lgamma.c e_lgamma_r.c e_lgammaf.c e_lgammaf_r.c
      e_log.c e_log10.c e_log10f.c e_log2.c e_log2f.c e_logf.c
      e_pow.c e_powf.c e_remainder.c e_remainderf.c
      e_rem_pio2.c e_rem_pio2f.c
      e_sinh.c e_sinhf.c e_sqrt.c e_sqrtf.c
      k_cos.c k_exp.c k_expf.c k_rem_pio2.c k_sin.c k_tan.c
      k_cosf.c k_sinf.c k_tanf.c
      s_asinh.c s_asinhf.c s_atan.c s_atanf.c s_carg.c s_cargf.c
      s_cbrt.c s_cbrtf.c s_ceil.c s_ceilf.c
      s_copysign.c s_copysignf.c s_cos.c s_cosf.c
      s_csqrt.c s_csqrtf.c s_erf.c s_erff.c
      s_exp2.c s_exp2f.c s_expm1.c s_expm1f.c s_fabs.c s_fabsf.c s_fdim.c
      s_floor.c s_floorf.c
      s_fmax.c s_fmaxf.c s_fmin.c
      s_fminf.c s_fpclassify.c
      s_frexp.c s_frexpf.c s_ilogb.c s_ilogbf.c
      s_isinf.c s_isfinite.c s_isnormal.c s_isnan.c
      s_log1p.c s_log1pf.c s_logb.c s_logbf.c
      s_modf.c s_modff.c
      s_nextafter.c s_nextafterf.c
      s_nexttowardf.c s_remquo.c s_remquof.c
      s_rint.c s_rintf.c s_round.c s_roundf.c
      s_scalbln.c s_scalbn.c s_scalbnf.c s_signbit.c
      s_signgam.c s_sin.c s_sincos.c
      s_sinf.c s_sincosf.c s_tan.c s_tanf.c s_tanh.c s_tanhf.c s_tgammaf.c
      s_trunc.c s_truncf.c s_cpow.c s_cpowf.c
      w_cabs.c w_cabsf.c
  )

  # if(NOT ARCH STREQUAL "wasm32")
  #     list(APPEND CUR_SRCS
  #         s_fma.c s_fmaf.c s_lrint.c s_lrintf.c s_lround.c s_lroundf.c
  #         s_llrint.c s_llrintf.c s_llround.c s_llroundf.c s_nearbyint.c
  #     )

  #     if(NOT OS STREQUAL "WINNT")
  #         list(APPEND CUR_SRCS s_nan.c)
  #     endif()
  # endif()

  if(LONG_DOUBLE_NOT_DOUBLE)
      list(APPEND CUR_SRCS
          s_copysignl.c s_fabsl.c s_llrintl.c s_lrintl.c s_modfl.c
          e_acosl.c e_asinl.c e_atan2l.c e_fmodl.c
          s_fmaxl.c s_fminl.c s_ilogbl.c
          e_hypotl.c e_lgammal.c e_remainderl.c e_sqrtl.c
          s_atanl.c s_ceill.c s_cosl.c s_cprojl.c
          s_csqrtl.c s_floorl.c s_fmal.c
          s_frexpl.c s_logbl.c s_nexttoward.c
          s_remquol.c s_roundl.c s_lroundl.c s_llroundl.c
          s_cpowl.c s_cargl.c
          s_sinl.c s_sincosl.c s_tanl.c s_truncl.c w_cabsl.c
          s_nextafterl.c s_rintl.c s_scalbnl.c polevll.c
          s_casinl.c s_ctanl.c
          s_cimagl.c s_conjl.c s_creall.c s_cacoshl.c s_catanhl.c s_casinhl.c
          s_catanl.c s_csinl.c s_cacosl.c s_cexpl.c s_csinhl.c s_ccoshl.c
          s_clogl.c s_ctanhl.c s_ccosl.c s_cbrtl.c
      )
  endif()

  list(APPEND CUR_SRCS
      s_ccosh.c s_ccoshf.c s_cexp.c s_cexpf.c
      s_cimag.c s_cimagf.c
      s_conj.c s_conjf.c
      s_cproj.c s_cprojf.c s_creal.c s_crealf.c
      s_csinh.c s_csinhf.c s_ctanh.c s_ctanhf.c
      s_cacos.c s_cacosf.c
      s_cacosh.c s_cacoshf.c
      s_casin.c s_casinf.c s_casinh.c s_casinhf.c
      s_catan.c s_catanf.c s_catanh.c s_catanhf.c
      s_clog.c s_clogf.c
  )
  # cmake-format: on

  foreach(src ${CUR_SRCS})
    string(PREPEND src ${OPENLIBM_DIR}/src/)
    list(APPEND CSRCS ${src})
  endforeach()

  file(GLOB_RECURSE ARCH_CSRCS ${OPENLIBM_DIR}/${ARCH}/*.c)
  file(GLOB_RECURSE ARCH_ASRCS ${OPENLIBM_DIR}/${ARCH}/*.S)
  file(GLOB_RECURSE BD_CSRCS ${OPENLIBM_DIR}/bsdsrc/*.c)

  # ############################################################################
  # Include Directory
  # ############################################################################

  set(INCDIR ${OPENLIBM_DIR} ${OPENLIBM_DIR}/${ARCH} ${OPENLIBM_DIR}/src)

  # ############################################################################
  # Library Configuration
  # ############################################################################

  nuttx_add_kernel_library(m)

  target_sources(m PRIVATE ${CSRCS} ${ARCH_CSRCS} ${BD_CSRCS} ${ARCH_ASRCS})
  target_include_directories(m PRIVATE ${INCDIR})

  set_property(
    TARGET nuttx
    APPEND
    PROPERTY NUTTX_INCLUDE_DIRECTORIES ${OPENLIBM_DIR}/include)

endif()
