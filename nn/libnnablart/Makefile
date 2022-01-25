############################################################################
# nn/libnnablart/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/Make.defs

include nnablart.defs

ifeq ($(CONFIG_NNABLA_RT),y)
SRC = nnabla-c-runtime/src

CSRCS   += $(SRC)/functions/utilities/accessor.c
CSRCS   += $(SRC)/functions/utilities/fixedpoint.c
CSRCS   += $(SRC)/functions/utilities/list.c
CSRCS   += $(SRC)/functions/utilities/shape.c
CSRCS   += $(SRC)/functions/implements/activation/sigmoid.c
CSRCS   += $(SRC)/functions/implements/activation/relu.c
CSRCS   += $(SRC)/functions/implements/activation/tanh.c
CSRCS   += $(SRC)/functions/implements/activation/softmax.c
CSRCS   += $(SRC)/functions/implements/activation/selu.c
CSRCS   += $(SRC)/functions/implements/activation/elu.c
CSRCS   += $(SRC)/functions/implements/activation/prelu.c
CSRCS   += $(SRC)/functions/implements/activation/leakyrelu.c
CSRCS   += $(SRC)/functions/implements/activation/crelu.c
CSRCS   += $(SRC)/functions/implements/activation/celu.c
CSRCS   += $(SRC)/functions/implements/activation/swish.c
CSRCS   += $(SRC)/functions/implements/math/abs.c
CSRCS   += $(SRC)/functions/implements/math/batch_matmul.c
CSRCS   += $(SRC)/functions/implements/math/exp.c
CSRCS   += $(SRC)/functions/implements/math/identity.c
CSRCS   += $(SRC)/functions/implements/math/log.c
CSRCS   += $(SRC)/functions/implements/math/round.c
CSRCS   += $(SRC)/functions/implements/quantization/binary_tanh.c
CSRCS   += $(SRC)/functions/implements/quantization/binary_sigmoid.c
CSRCS   += $(SRC)/functions/implements/quantization/binary_connect_affine.c
CSRCS   += $(SRC)/functions/implements/quantization/binary_weight_affine.c
CSRCS   += $(SRC)/functions/implements/arithmetic/add_scalar.c
CSRCS   += $(SRC)/functions/implements/arithmetic/arithmetic.c
CSRCS   += $(SRC)/functions/implements/arithmetic/arithmetic_fixed.c
CSRCS   += $(SRC)/functions/implements/arithmetic/arithmetic_generic.c
CSRCS   += $(SRC)/functions/implements/arithmetic/div2.c
CSRCS   += $(SRC)/functions/implements/arithmetic/mul2.c
CSRCS   += $(SRC)/functions/implements/arithmetic/mul_scalar.c
CSRCS   += $(SRC)/functions/implements/arithmetic/pow2.c
CSRCS   += $(SRC)/functions/implements/arithmetic/pow_scalar.c
CSRCS   += $(SRC)/functions/implements/arithmetic/r_div_scalar.c
CSRCS   += $(SRC)/functions/implements/arithmetic/r_pow_scalar.c
CSRCS   += $(SRC)/functions/implements/arithmetic/r_sub_scalar.c
CSRCS   += $(SRC)/functions/implements/arithmetic/sub2.c
CSRCS   += $(SRC)/functions/implements/arithmetic/add2.c
CSRCS   += $(SRC)/functions/implements/logical/maximum_scalar.c
CSRCS   += $(SRC)/functions/implements/logical/minimum_scalar.c
CSRCS   += $(SRC)/functions/implements/logical/maximum2.c
CSRCS   += $(SRC)/functions/implements/logical/minimum2.c
CSRCS   += $(SRC)/functions/implements/logical/sign.c
CSRCS   += $(SRC)/functions/implements/array/matrix_diag.c
CSRCS   += $(SRC)/functions/implements/array/matrix_diag_part.c
CSRCS   += $(SRC)/functions/implements/array/reshape.c
CSRCS   += $(SRC)/functions/implements/array/concatenate.c
CSRCS   += $(SRC)/functions/implements/array/split.c
CSRCS   += $(SRC)/functions/implements/array/stack.c
CSRCS   += $(SRC)/functions/implements/array/shift.c
CSRCS   += $(SRC)/functions/implements/array/slice.c
CSRCS   += $(SRC)/functions/implements/array/flip.c
CSRCS   += $(SRC)/functions/implements/array/transpose.c
CSRCS   += $(SRC)/functions/implements/array/pad.c
CSRCS   += $(SRC)/functions/implements/neural_network/pooling.c
CSRCS   += $(SRC)/functions/implements/neural_network/max_pooling.c
CSRCS   += $(SRC)/functions/implements/neural_network/sum_pooling.c
CSRCS   += $(SRC)/functions/implements/neural_network/average_pooling.c
CSRCS   += $(SRC)/functions/implements/neural_network/unpooling.c
CSRCS   += $(SRC)/functions/implements/neural_network/deconvolution.c
CSRCS   += $(SRC)/functions/implements/neural_network/affine/affine.c
CSRCS   += $(SRC)/functions/implements/neural_network/affine/affine_fixed8.c
CSRCS   += $(SRC)/functions/implements/neural_network/affine/affine_fixed16.c
CSRCS   += $(SRC)/functions/implements/neural_network/affine/affine_generic.c
CSRCS   += $(SRC)/functions/implements/neural_network/convolution/convolution.c
CSRCS   += $(SRC)/functions/implements/neural_network/convolution/convolution_generic.c
CSRCS   += $(SRC)/functions/implements/neural_network/convolution/convolution_float.c
CSRCS   += $(SRC)/functions/implements/neural_network/convolution/convolution_int8.c
CSRCS   += $(SRC)/functions/implements/neural_network/convolution/convolution_int16.c
CSRCS   += $(SRC)/functions/implements/neural_network/convolution/convolution_common.c
CSRCS   += $(SRC)/functions/implements/neural_network/convolution/binary_connect_convolution.c
CSRCS   += $(SRC)/functions/implements/neural_network/convolution/binary_weight_convolution.c
CSRCS   += $(SRC)/functions/implements/neural_network/convolution/depthwise_convolution.c
CSRCS   += $(SRC)/functions/implements/normalization/batch_normalization.c
CSRCS   += $(SRC)/functions/implements/normalization/mean_subtraction.c
CSRCS   += $(SRC)/functions/implements/stochasticity/dropout.c
CSRCS   += $(SRC)/functions/implements/reduction/sum.c
CSRCS   += $(SRC)/functions/implements/unimplemented.c
CSRCS   += $(SRC)/runtime/function_context.c
CSRCS   += $(SRC)/runtime/runtime.c
CSRCS   += $(SRC)/runtime/runtime_internal.c
endif

CFLAGS += -I$(TOPDIR)/include/nnabla/
CFLAGS += -Wno-shadow -Wno-format

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = libnnablart$(LIBEXT)

all: $(BIN)
.PHONY:  depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MAKE) makedepfile
	$(Q) touch $@

depend: .depend

clean::
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean:: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
