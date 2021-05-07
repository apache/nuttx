#!/usr/bin/env bash
 #############################################################################
 # boards/arm/stm32/stm32f429-disco/tools/fbcalc.sh
 #
 # Licensed to the Apache Software Foundation (ASF) under one or more
 # contributor license agreements.  See the NOTICE file distributed with
 # this work for additional information regarding copyright ownership.  The
 # ASF licenses this file to you under the Apache License, Version 2.0 (the
 # "License"); you may not use this file except in compliance with the
 # License.  You may obtain a copy of the License at
 #
 #   http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 # WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 # License for the specific language governing permissions and limitations
 # under the License.
 #
 #############################################################################

heap2base=0xD0000000
heap2size=$((1024 * 1024 * 8))
heap2end=$((${heap2base} + ${heap2size}))

usage()
{
  cat << EOF
${0}: <xres> <yres> <bpp> <n> <xres2> <yres2> <n2> <base>

  options:
    xres       LTDC display x-resolution
    yres       LTDC display y-resolution
    bpp        LTDC bits per pixel (8/16/24/32)
    n          Number of LTDC overlays
    xres2      DMA2D display x-resolution
    yres2      DMA2D display y-resolution
    n2         Number of DMA2D overlays
    base       Framebuffer positioning in heap2 memory region (optional)
               0: base of heap2 (default)
               1: end of heap2

example:
    ${0} 240 320 16 2 240 320 2 0
EOF
}

fbinfo()
{
  # $1: LTDC start address
  # $2: LTDC len
  # $3: DMA2D len
  # $4: Set framebuffer to end

  if [ ${4} -eq 0 ] ; then
    cat << EOF
-------------------------------------------------------
Framebuffer calculation
-------------------------------------------------------
  Framebuffer base address: $(printf 0x%08X ${1})
 CONFIG_STM32_LTDC_FB_BASE: $(printf 0x%08X ${1})
 CONFIG_STM32_LTDC_FB_SIZE: ${2}
CONFIG_STM32_DMA2D_FB_BASE: $(printf 0x%08X $((${1} + ${2})))
CONFIG_STM32_DMA2D_FB_SIZE: ${3}
EOF

  else
    cat << EOF
-------------------------------------------------------
Framebuffer calculation
-------------------------------------------------------
   Framebuffer end address: $(printf 0x%08X $((${1} + ${heap2size})))
 CONFIG_STM32_LTDC_FB_BASE: $(printf 0x%08X $((${1} + ${heap2size} - ${2} - ${3})))
 CONFIG_STM32_LTDC_FB_SIZE: ${2}
CONFIG_STM32_DMA2D_FB_BASE: $(printf 0x%08X $((${1} - ${3} + ${heap2size})))
CONFIG_STM32_DMA2D_FB_SIZE: ${3}
EOF

  fi
}

heap2info()
{
  # $1: LTDC start address
  # $2: LTDC + DMA2D len
  # $3: Set framebuffer to end

  if [ $((${1} + 0)) -ge $((${heap2base} + 0)) ] && \
      [ $((${1} + 0)) -le ${heap2end} ] ; then

    if [ ${3} -ne 0 ] ; then
      cat << EOF
-------------------------------------------------------
Heap2 calculation
-------------------------------------------------------
   Framebuffer end address: $(printf 0x%08X $((${1} + ${heap2size})))
         CONFIG_HEAP2_BASE: $(printf 0x%08X ${1})
         CONFIG_HEAP2_SIZE: $(printf %d $((${heap2size} - ${2})))
EOF
    else
      cat << EOF
-------------------------------------------------------
Heap2 calculation
-------------------------------------------------------
  Framebuffer base address: $(printf 0x%08X ${1})
         CONFIG_HEAP2_BASE: $(printf 0x%08X $((${1} - ${2} + ${heap2size})))
         CONFIG_HEAP2_SIZE: $(printf %d $((${heap2size} - ${2})))
EOF
    fi
  fi
}

fbstride()
{
  # $1: bpp
  # $2: pixel per line

  bpp=${1}
  ppl=${2}
  stride=0

  if [ $bpp -eq 8 ] ; then
      stride=$(((${ppl} * ${bpp} + 7) / 8))
  elif [ $bpp -eq 16 ] ; then
      stride=$(((${ppl} * ${bpp} + 7) / 8))
  elif [ $bpp -eq 24 ] ; then
      stride=$(((${ppl} * ${bpp} + 7) / 8))
  elif [ $bpp -eq 32 ] ; then
      stride=$(((${ppl} * ${bpp} + 7) / 8))
  else
    echo "Pixel format with ${bpp} not supported" 1>&2
    return 1
  fi

  echo $stride
  return 0
}

if [ -z $7 ] ; then
    usage
    exit 1
fi

fbaddr=${heap2base}
xres=${1}
yres=${2}
bpp=${3}
noverlays=${4}
xres2=${5}
yres2=${6}
noverlays2=${7}

if [ ! -z $8 ] ; then
    base=${8}
else
    base=0
fi

if [ ! -z $9 ] ; then
    config=${9}
else
    config=""
fi

stride=$(fbstride ${bpp} ${xres})
if [ $? -ne 0 ] ; then
    exit 1
fi

stride2=$(fbstride ${bpp} ${xres2})
if [ $? -ne 0 ] ; then
    exit 1
fi

size=$((${stride} * ${yres} * ${noverlays}))
size2=$((${stride2} * ${yres2} * ${noverlays2}))

fbinfo ${fbaddr} ${size} ${size2} ${base}
heap2info ${fbaddr} $((${size} + ${size2})) ${base}
