#!/usr/bin/env bash
# .github/scripts/gh_api_retry.sh
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.
# The ASF licenses this file to you under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with
# the License.  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# Source this file, then call gh_api_retry with the arguments of "gh api".
# The GitHub API sometimes fails for a short time: a network error, a 5xx
# answer, a rate limit, or an error page instead of JSON.  gh_api_retry
# tries again after 5, 10 and 20 seconds.  It does not try again when the
# answer is HTTP 404 or 422, because the next try gives the same answer.
# It prints the output of the successful try and returns 0, or it prints
# the errors of the last try to stderr and returns 1.

gh_api_retry()
{
  local delay=5
  local err
  local out
  local try

  err=$(mktemp)
  for try in 1 2 3 4; do
    if out=$(gh api "$@" 2> "$err"); then
      rm -f "$err"
      printf '%s\n' "$out"
      return 0
    fi

    if grep -q 'HTTP 404\|HTTP 422' "$err" || [ "$try" -eq 4 ]; then
      break
    fi

    echo "gh api $1 failed (try $try of 4), trying again in ${delay} s:" \
      "$(head -c 200 "$err")" >&2
    sleep "$delay"
    delay=$((delay * 2))
  done

  cat "$err" >&2
  rm -f "$err"
  return 1
}
