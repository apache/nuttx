#!/usr/bin/env bash
###########################################################################
#
#   Copyright 2017, 2018, 2019 David S. Alessio, All rights reserved.
#   Author: David S. Alessio <david.s.alessio@gmail.com>
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#   1. Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in
#      the documentation and/or other materials provided with the
#      distribution.
#   3. Neither the name NuttX nor the names of its contributors may be
#      used to endorse or promote products derived from this software
#      without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
#   OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
#
###########################################################################
#
#   This bash script queries the git repo for info/status
#   and generates a header file with version info #defs
#
#   Date  : 25-Aug-2019


###########################################################################
#
#   Global Vars
#

# full pathname to utils used
declare WHO=/usr/bin/whoami
declare GIT=/usr/bin/git
declare TR=/usr/bin/tr
declare WC=/usr/bin/wc

declare DATE=date
declare HOSTNAME=hostname



###########################################################################
#
#   define -- helper function to define vars with HEREDOC
#
define() { IFS='\n' read -r -d '' ${1} || true; }



###########################################################################
#
#   usage
#
usage() {
    define usage_str <<EOT

    Usage: $0 [option...]

       -f  <fname>      version info filename
       -t  <dir>        top directory
       -h               show usage
       -v               increase verbosity
EOT

    echo "$usage_str" >&2
    exit 1
}



###########################################################################
#
#   parse args
#
declare -i verbosity=0
declare TOP_DIR=

while getopts ":ht:v" opt; do
    case $opt in
        h) usage
            ;;

        t) TOP_DIR=$OPTARG
            ;;

        v) verbosity=$(($verbosity+1))
            ;;

        \?) echo "Invalid option: -$OPTARG" >&2
            usage
            ;;

        :) echo "Option -$OPTARG requires an argument." >&2
            usage
            ;;
    esac
done

shift $((OPTIND-1))



[[ $OPTIND -gt 1 ]] || usage

if [[ -z "$TOP_DIR" ]]; then
    echo "@@@"
    echo "@@@  TOP DIR not specified"
    echo "@@@  Exiting with Error!"
    echo "@@@"
    exit 1
fi



# bail out if .git directory doesn't exist
[[ -d $TOP_DIR/.git ]] || exit 0



###########################################################################
#
#   Git info vars
#
declare BUILD_DATE=`$DATE +"%a %d-%b-%Y %T %Z" | $TR -d '\n'`
declare BUILD_MACH=`$HOSTNAME | $TR -d '\n'`
declare BUILD_USER=`$WHO`
declare GIT_BRANCH=`$GIT rev-parse --abbrev-ref HEAD`
declare GIT_FW_DESCR=`$GIT describe --long --tags --dirty='+' --always`
declare GIT_SHORT_HASH=`$GIT rev-parse --short HEAD`

# Create the git revision string

declare GIT_REVISION_STR="$GIT_FW_DESCR, $GIT_BRANCH, $BUILD_MACH/$BUILD_USER, $BUILD_DATE"



###########################################################################
#
#   Dump vars to stderr
#
show_vars() {
    define vars <<EOT

              Branch : $GIT_BRANCH
                Hash : $GIT_SHORT_HASH

       Build Machine : $BUILD_MACH
          Build User : $BUILD_USER
          Build Date : $BUILD_DATE

          FW Version : $GIT_FW_DESCR

         Git Rev str : $GIT_REVISION_STR
EOT

    echo "$vars" >&2
}



if [[ verbosity -ge 1 ]]; then
    show_vars
fi



###########################################################################
#
#   update .version file with the following:
#

echo "CONFIG_GIT_REVISION_STR=\"$GIT_REVISION_STR\""
