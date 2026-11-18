rem boards/arm/lpc43xx/lpc4370-link2/scripts/flash.bat
rem
rem SPDX-License-Identifier: Apache-2.0
rem
rem Licensed to the Apache Software Foundation (ASF) under one or more
rem contributor license agreements.  See the NOTICE file distributed with
rem this work for additional information regarding copyright ownership.  The
rem ASF licenses this file to you under the Apache License, Version 2.0 (the
rem "License"); you may not use this file except in compliance with the
rem License.  You may obtain a copy of the License at
rem
rem   http://www.apache.org/licenses/LICENSE-2.0
rem
rem Unless required by applicable law or agreed to in writing, software
rem distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
rem WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
rem License for the specific language governing permissions and limitations
rem under the License.

set LPCScrypt=C:\NXP\LPCScrypt\bin

call %LPCScrypt%\..\scripts\boot_lpcscrypt.cmd

TIMEOUT /T 5

%LPCScrypt%\lpcscrypt.exe erase SPIFI
%LPCScrypt%\lpcscrypt.exe program ..\..\..\nuttx.bin SPIFI
