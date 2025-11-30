/****************************************************************************
 * arch/sim/src/sim/posix/sim_errno.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Add holder to detect possible errno in host def.h
 * Use ERRNO_HOLDER to make ifdef able to expand at pre-processor time
 */

#define ERRNO_HOLDER_ 0,
#define ERRNO_HOLDER_1 0,
#define ERRNO_HOLDER_2 0,
#define ERRNO_HOLDER_3 0,
#define ERRNO_HOLDER_4 0,
#define ERRNO_HOLDER_5 0,
#define ERRNO_HOLDER_6 0,
#define ERRNO_HOLDER_7 0,
#define ERRNO_HOLDER_8 0,
#define ERRNO_HOLDER_9 0,
#define ERRNO_HOLDER_10 0,
#define ERRNO_HOLDER_11 0,
#define ERRNO_HOLDER_12 0,
#define ERRNO_HOLDER_13 0,
#define ERRNO_HOLDER_14 0,
#define ERRNO_HOLDER_15 0,
#define ERRNO_HOLDER_16 0,
#define ERRNO_HOLDER_17 0,
#define ERRNO_HOLDER_18 0,
#define ERRNO_HOLDER_19 0,
#define ERRNO_HOLDER_20 0,
#define ERRNO_HOLDER_21 0,
#define ERRNO_HOLDER_22 0,
#define ERRNO_HOLDER_23 0,
#define ERRNO_HOLDER_24 0,
#define ERRNO_HOLDER_25 0,
#define ERRNO_HOLDER_26 0,
#define ERRNO_HOLDER_27 0,
#define ERRNO_HOLDER_28 0,
#define ERRNO_HOLDER_29 0,
#define ERRNO_HOLDER_30 0,
#define ERRNO_HOLDER_31 0,
#define ERRNO_HOLDER_32 0,
#define ERRNO_HOLDER_33 0,
#define ERRNO_HOLDER_34 0,
#define ERRNO_HOLDER_35 0,
#define ERRNO_HOLDER_36 0,
#define ERRNO_HOLDER_37 0,
#define ERRNO_HOLDER_38 0,
#define ERRNO_HOLDER_39 0,
#define ERRNO_HOLDER_40 0,
#define ERRNO_HOLDER_41 0,
#define ERRNO_HOLDER_42 0,
#define ERRNO_HOLDER_43 0,
#define ERRNO_HOLDER_44 0,
#define ERRNO_HOLDER_45 0,
#define ERRNO_HOLDER_46 0,
#define ERRNO_HOLDER_47 0,
#define ERRNO_HOLDER_48 0,
#define ERRNO_HOLDER_49 0,
#define ERRNO_HOLDER_50 0,
#define ERRNO_HOLDER_51 0,
#define ERRNO_HOLDER_52 0,
#define ERRNO_HOLDER_53 0,
#define ERRNO_HOLDER_54 0,
#define ERRNO_HOLDER_55 0,
#define ERRNO_HOLDER_56 0,
#define ERRNO_HOLDER_57 0,
#define ERRNO_HOLDER_58 0,
#define ERRNO_HOLDER_59 0,
#define ERRNO_HOLDER_60 0,
#define ERRNO_HOLDER_61 0,
#define ERRNO_HOLDER_62 0,
#define ERRNO_HOLDER_63 0,
#define ERRNO_HOLDER_64 0,
#define ERRNO_HOLDER_65 0,
#define ERRNO_HOLDER_66 0,
#define ERRNO_HOLDER_67 0,
#define ERRNO_HOLDER_68 0,
#define ERRNO_HOLDER_69 0,
#define ERRNO_HOLDER_70 0,
#define ERRNO_HOLDER_71 0,
#define ERRNO_HOLDER_72 0,
#define ERRNO_HOLDER_73 0,
#define ERRNO_HOLDER_74 0,
#define ERRNO_HOLDER_75 0,
#define ERRNO_HOLDER_76 0,
#define ERRNO_HOLDER_77 0,
#define ERRNO_HOLDER_78 0,
#define ERRNO_HOLDER_79 0,
#define ERRNO_HOLDER_80 0,
#define ERRNO_HOLDER_81 0,
#define ERRNO_HOLDER_82 0,
#define ERRNO_HOLDER_83 0,
#define ERRNO_HOLDER_84 0,
#define ERRNO_HOLDER_85 0,
#define ERRNO_HOLDER_86 0,
#define ERRNO_HOLDER_87 0,
#define ERRNO_HOLDER_88 0,
#define ERRNO_HOLDER_89 0,
#define ERRNO_HOLDER_90 0,
#define ERRNO_HOLDER_91 0,
#define ERRNO_HOLDER_92 0,
#define ERRNO_HOLDER_93 0,
#define ERRNO_HOLDER_94 0,
#define ERRNO_HOLDER_95 0,
#define ERRNO_HOLDER_96 0,
#define ERRNO_HOLDER_97 0,
#define ERRNO_HOLDER_98 0,
#define ERRNO_HOLDER_99 0,
#define ERRNO_HOLDER_100 0,
#define ERRNO_HOLDER_101 0,
#define ERRNO_HOLDER_102 0,
#define ERRNO_HOLDER_103 0,
#define ERRNO_HOLDER_104 0,
#define ERRNO_HOLDER_105 0,
#define ERRNO_HOLDER_106 0,
#define ERRNO_HOLDER_107 0,
#define ERRNO_HOLDER_108 0,
#define ERRNO_HOLDER_109 0,
#define ERRNO_HOLDER_110 0,
#define ERRNO_HOLDER_111 0,
#define ERRNO_HOLDER_112 0,
#define ERRNO_HOLDER_113 0,
#define ERRNO_HOLDER_114 0,
#define ERRNO_HOLDER_115 0,
#define ERRNO_HOLDER_116 0,
#define ERRNO_HOLDER_117 0,
#define ERRNO_HOLDER_118 0,
#define ERRNO_HOLDER_119 0,
#define ERRNO_HOLDER_120 0,
#define ERRNO_HOLDER_121 0,
#define ERRNO_HOLDER_122 0,
#define ERRNO_HOLDER_123 0,
#define ERRNO_HOLDER_124 0,
#define ERRNO_HOLDER_125 0,
#define ERRNO_HOLDER_126 0,
#define ERRNO_HOLDER_127 0,
#define ERRNO_HOLDER_128 0,
#define ERRNO_HOLDER_129 0,
#define ERRNO_HOLDER_130 0,
#define ERRNO_HOLDER_131 0,
#define ERRNO_HOLDER_132 0,
#define ERRNO_HOLDER_133 0,
#define ERRNO_HOLDER_134 0,
#define ERRNO_HOLDER_135 0,
#define ERRNO_HOLDER_136 0,
#define ERRNO_HOLDER_137 0,
#define ERRNO_HOLDER_138 0,
#define ERRNO_HOLDER_139 0,
#define ERRNO_HOLDER_140 0,
#define ERRNO_HOLDER_141 0,
#define ERRNO_HOLDER_142 0,
#define ERRNO_HOLDER_143 0,
#define ERRNO_HOLDER_144 0,
#define ERRNO_HOLDER_145 0,
#define ERRNO_HOLDER_146 0,
#define ERRNO_HOLDER_147 0,
#define ERRNO_HOLDER_148 0,
#define ERRNO_HOLDER_149 0,
#define ERRNO_HOLDER_150 0,
#define ERRNO_HOLDER_151 0,
#define ERRNO_HOLDER_152 0,
#define ERRNO_HOLDER_153 0,
#define ERRNO_HOLDER_154 0,
#define ERRNO_HOLDER_155 0,
#define ERRNO_HOLDER_156 0,
#define ERRNO_HOLDER_157 0,
#define ERRNO_HOLDER_158 0,
#define ERRNO_HOLDER_159 0,
#define ERRNO_HOLDER_160 0,
#define ERRNO_HOLDER_161 0,
#define ERRNO_HOLDER_162 0,
#define ERRNO_HOLDER_163 0,
#define ERRNO_HOLDER_164 0,
#define ERRNO_HOLDER_165 0,
#define ERRNO_HOLDER_166 0,
#define ERRNO_HOLDER_167 0,
#define ERRNO_HOLDER_168 0,
#define ERRNO_HOLDER_169 0,
#define ERRNO_HOLDER_170 0,
#define ERRNO_HOLDER_171 0,
#define ERRNO_HOLDER_172 0,
#define ERRNO_HOLDER_173 0,
#define ERRNO_HOLDER_174 0,
#define ERRNO_HOLDER_175 0,
#define ERRNO_HOLDER_176 0,
#define ERRNO_HOLDER_177 0,
#define ERRNO_HOLDER_178 0,
#define ERRNO_HOLDER_179 0,
#define ERRNO_HOLDER_180 0,
#define ERRNO_HOLDER_181 0,
#define ERRNO_HOLDER_182 0,
#define ERRNO_HOLDER_183 0,
#define ERRNO_HOLDER_184 0,
#define ERRNO_HOLDER_185 0,
#define ERRNO_HOLDER_186 0,
#define ERRNO_HOLDER_187 0,
#define ERRNO_HOLDER_188 0,
#define ERRNO_HOLDER_189 0,
#define ERRNO_HOLDER_190 0,
#define ERRNO_HOLDER_191 0,
#define ERRNO_HOLDER_192 0,
#define ERRNO_HOLDER_193 0,
#define ERRNO_HOLDER_194 0,
#define ERRNO_HOLDER_195 0,
#define ERRNO_HOLDER_196 0,
#define ERRNO_HOLDER_197 0,
#define ERRNO_HOLDER_198 0,
#define ERRNO_HOLDER_199 0,
#define ERRNO_HOLDER_200 0,
#define ERRNO_HOLDER_201 0,
#define ERRNO_HOLDER_202 0,
#define ERRNO_HOLDER_203 0,
#define ERRNO_HOLDER_204 0,
#define ERRNO_HOLDER_205 0,
#define ERRNO_HOLDER_206 0,
#define ERRNO_HOLDER_207 0,
#define ERRNO_HOLDER_208 0,
#define ERRNO_HOLDER_209 0,
#define ERRNO_HOLDER_210 0,
#define ERRNO_HOLDER_211 0,
#define ERRNO_HOLDER_212 0,
#define ERRNO_HOLDER_213 0,
#define ERRNO_HOLDER_214 0,
#define ERRNO_HOLDER_215 0,
#define ERRNO_HOLDER_216 0,
#define ERRNO_HOLDER_217 0,
#define ERRNO_HOLDER_218 0,
#define ERRNO_HOLDER_219 0,
#define ERRNO_HOLDER_220 0,
#define ERRNO_HOLDER_221 0,
#define ERRNO_HOLDER_222 0,
#define ERRNO_HOLDER_223 0,
#define ERRNO_HOLDER_224 0,
#define ERRNO_HOLDER_225 0,
#define ERRNO_HOLDER_226 0,
#define ERRNO_HOLDER_227 0,
#define ERRNO_HOLDER_228 0,
#define ERRNO_HOLDER_229 0,
#define ERRNO_HOLDER_230 0,
#define ERRNO_HOLDER_231 0,
#define ERRNO_HOLDER_232 0,
#define ERRNO_HOLDER_233 0,
#define ERRNO_HOLDER_234 0,
#define ERRNO_HOLDER_235 0,
#define ERRNO_HOLDER_236 0,
#define ERRNO_HOLDER_237 0,
#define ERRNO_HOLDER_238 0,
#define ERRNO_HOLDER_239 0,
#define ERRNO_HOLDER_240 0,
#define ERRNO_HOLDER_241 0,
#define ERRNO_HOLDER_242 0,
#define ERRNO_HOLDER_243 0,
#define ERRNO_HOLDER_244 0,
#define ERRNO_HOLDER_245 0,
#define ERRNO_HOLDER_246 0,
#define ERRNO_HOLDER_247 0,
#define ERRNO_HOLDER_248 0,
#define ERRNO_HOLDER_249 0,
#define ERRNO_HOLDER_250 0,
#define ERRNO_HOLDER_251 0,
#define ERRNO_HOLDER_252 0,
#define ERRNO_HOLDER_253 0,
#define ERRNO_HOLDER_254 0,
#define ERRNO_HOLDER_255 0,

#define ERRNO_SECOND(_, val, ...)  val

#define ERRNO_ISDEF(val)           ERRNO_ISDEF_(ERRNO_HOLDER_##val)
#define ERRNO_ISDEF_(arg1_or_junk) ERRNO_SECOND(arg1_or_junk 1, 0)

#define ERRNO_IF(val, r)           ERRNO_IF_(ERRNO_ISDEF(val), val, r)
#define ERRNO_IF_(c, val, r)       ERRNO_IF__(c, val, r)
#define ERRNO_IF__(c, val, r)      ERRNO_IF_##c(val, r)
#define ERRNO_IF_0(val, r)
#define ERRNO_IF_1(val, r)         else if (val == errcode) { return -r;}

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum
{
#define ERRNO_ITEM(name, id, str) NX_##name = id,
#include "errno_lookup.h"
#undef ERRNO_ITEM
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_errno_convert(int negative_errno)
{
  int errcode = -negative_errno;
  if (0)
    {
    }
#define ERRNO_ITEM(name, id, str) ERRNO_IF(name, NX_##name)
#include "errno_lookup.h"
#undef ERRNO_ITEM

  return negative_errno;
}

int host_errno_get(void)
{
  return errno;
}

void host_errno_set(int errcode)
{
  errno = errcode;
}
