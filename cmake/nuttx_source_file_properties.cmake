# ##############################################################################
# cmake/nuttx_source_file_properties.cmake
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

# Because `set_source_files_properties` in cmake will overwrite the properties
# instead of appending them. This module implements addition and deletion by
# first `getting_source_file_property` and then `set_source_files_properties`

# addition
function(nuttx_append_source_file_properties SOURCE_FILE PROPERTY_NAME
         PROPERTY_VALUE)
  get_source_file_property(curr_value ${SOURCE_FILE} ${PROPERTY_NAME})
  if(NOT curr_value)
    set(curr_value "")
  endif()
  set_source_files_properties(
    ${SOURCE_FILE} PROPERTIES ${PROPERTY_NAME}
                              "${curr_value} ${PROPERTY_VALUE}")
endfunction()

# deletion
function(nuttx_remove_source_file_property SOURCE_FILE PROPERTY_NAME
         VALUE_TO_REMOVE)
  get_source_file_property(curr_value ${SOURCE_FILE} ${PROPERTY_NAME})
  if(curr_value)
    string(REPLACE ${VALUE_TO_REMOVE} "" new_value ${curr_value})
    set_source_files_properties(${SOURCE_FILE} PROPERTIES ${PROPERTY_NAME}
                                                          "${new_value}")
  endif()
endfunction()
