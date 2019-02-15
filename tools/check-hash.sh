#!/usr/bin/env sh
############################################################################
# apps/tools/check-hash.sh
#
#   Copyright (C) 2019 Michał Łyszczek. All rights reserved.
#   Author: Michał Łyszczek <michal.lyszczek@bofc.pl>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

usage="Usage: $0 <hash-algo> <expected-hash> <file-to-check>"

if [ ${#} -ne 3 ]
then
    echo "ERROR: invalid number of arguments passed"
    echo ""
    echo ${usage}
    exit 1
fi

hash_algo=${1}
exp_hash=${2}
file_to_check=${3}

if [ ! -f "${file_to_check}" ]; then
    echo "ERROR: file '${file_to_check}' does not exist"
    echo ""
    echo ${usage}
    exit 1
fi

case "${hash_algo}" in
    sha1|sha224|sha256|sha384|sha512)
        # valid hash passed, continue
        ;;

    *)
        echo "ERROR: invalid hash '${hash_algo}' for file '${file_to_check}'"
        echo "supported hashes are:"
        echo "    sha1, sha224, sha256, sha384, sha512"
        echo ""
        echo ${usage}
        exit 1
esac

# Calculate hash value of passed file

calc_hash=$( ${hash_algo}sum "${file_to_check}" | cut -d' ' -f1 )

# Does it match expected hash?

if [ "${exp_hash}" == "${calc_hash}" ]; then
    # yes, they match, we're good
    exit 0
fi

# No, hashes don't match, print error message and remove corrupted file

echo "ERROR: file ${file_to_check} has invalid hash"
echo "got:      ${calc_hash}"
echo "expected: ${exp_hash}"
rm "${file_to_check}"
exit 1
