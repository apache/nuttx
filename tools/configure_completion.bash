#!/usr/bin/env bash
# tools/configure_completion.bash
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

function _nuttx_configure_completion
{
  local colon_idx=2
  local num_options=0

  # Skip ./tools/configure.sh script options.
  while [[ "${#COMP_WORDS[@]}" != "${colon_idx}" ]] && [[ "${COMP_WORDS[$((colon_idx - 1))]}" == '-'* ]]
  do
    # If "-a" is among the provided arguments, consider the apps directory.
    if [ "${COMP_WORDS[$((colon_idx - 1))]}" == "-a" ]; then
      colon_idx=$((colon_idx + 2))
      num_options=$((num_options + 2))
    else
      colon_idx=$((colon_idx + 1))
      num_options=$((num_options + 1))
    fi
  done

  # When the command line is complete, COMP_WORDS array will have 5 elements
  # plus the options.
  # The script may return when this state is reached.
  if [ "${#COMP_WORDS[@]}" == $((5 + num_options)) ]; then
    return
  fi

  # If the last provided argument is "-a", do not execute autocompletion
  # algorithm.
  if [ "${COMP_WORDS[-2]}" == "-a" ]; then
    return
  fi

  local search_dir=""
  local search_pattern=""
  local reply_suffix=""

  if [ "${COMP_WORDS[${colon_idx}]}" == ':' ]; then
    local board_name=${COMP_WORDS[$((colon_idx - 1))]}
    local config_name=${COMP_WORDS[$((colon_idx + 1))]}

    search_dir="boards/*/*/${board_name}/configs/"
    search_pattern="${config_name}"
    reply_suffix=" "
  else
    search_dir="boards/*/*/"
    search_pattern="${COMP_WORDS[$((1 + num_options))]}"
    reply_suffix=":"
  fi

  local wordlist
  local suggestions
  wordlist=$(find ${search_dir} -maxdepth 1 -mindepth 1 -type d -exec basename {} \;) 2>/dev/null
  suggestions=($(compgen -W "${wordlist}" -- "${search_pattern}"))

  if [ "${#suggestions[@]}" == "1" ]; then
    # If there's only one match, complete with the appropriate reply suffix.
    COMPREPLY=("${suggestions[0]}${reply_suffix}")
  else
    # More than one suggestion resolved, respond with the suggestions intact.
    COMPREPLY=("${suggestions[@]}")
  fi
}

complete -o bashdefault -o default -o nospace -F _nuttx_configure_completion ./tools/configure.sh