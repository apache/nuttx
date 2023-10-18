# ##############################################################################
# libs/libdsp/CMakeLists.txt
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
if(CONFIG_LIBDSP)
  nuttx_add_library(
    dsp
    lib_pid.c
    lib_svm.c
    lib_transform.c
    lib_observer.c
    lib_observer_b16.c
    lib_foc.c
    lib_misc.c
    lib_motor.c
    lib_pmsm_model.c
    lib_pid_b16.c
    lib_svm_b16.c
    lib_transform_b16.c
    lib_foc_b16.c
    lib_misc_b16.c
    lib_motor_b16.c
    lib_pmsm_model_b16.c)
endif()
