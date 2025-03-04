@echo off

rem tools/unlink.bat
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
rem

rem Verify that arguments were provided

set link=%1
if "%link%"=="" goto :MissingArgument

rem Check if something already exists at the link path

if exist "%link%" goto :LinkExists

rem It is not an error if the link does not exist
rem echo %link% does not exist
rem goto :ShowUsage

goto :End

rem %link% make be a symbolic link or it may be a copied director (with
rem a .fakelnk file in it).  It really does not matter which:  We do the
rem same thing in either case

:LinkExists

rmdir /q /s %link%
if errorlevel 1 (
  echo Failed to remove existing object at %link%
  goto :ShowUsage
)

goto :End

:MissingArgument

echo Missing Argument

:ShowUsage
echo USAGE: %0 ^<link^>
echo Where:
echo  ^<link^> is the linked (or copied) directory to be removed

:End
