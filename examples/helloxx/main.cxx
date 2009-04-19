//***************************************************************************
// examples/hello/main.c
//
//   Copyright (C) 2009 Gregory Nutt. All rights reserved.
//   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
// 3. Neither the name NuttX nor the names of its contributors may be
//    used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//***************************************************************************

//***************************************************************************
// Included Files
//***************************************************************************

#include <nuttx/config.h>
#include <nuttx/init.h>
#include <cstdio>
#include <debug.h>

//***************************************************************************
// Definitions
//***************************************************************************

//***************************************************************************
// Private Classes
//***************************************************************************

class CHelloWorld
{
  public:
    CHelloWorld(void) : mSecret(42) { lldbg("Constructor\n"); };
    ~CHelloWorld(void) { lldbg("Destructor\n"); };

    bool HelloWorld(void)
    {
        if (mSecret != 42)
          {
            printf("CONSTRUCTION FAILED!\n");
            return false;
          }
        else
          {
            printf("Hello, World!!\n");
            return true;
          }
    };

  private:
    int mSecret;
};

//***************************************************************************
// Private Data
//***************************************************************************

#ifndef CONFIG_EXAMPLE_HELLOXX_NOSTATICCONST 
static CHelloWorld g_HelloWorld;
#endif

//***************************************************************************
// Public Functions
//***************************************************************************

//***************************************************************************
// user_initialize
//***************************************************************************

#ifndef CONFIG_HAVE_WEAKFUNCTIONS
void user_initialize(void)
{
  // Stub that must be provided only if the toolchain does not support weak
  // functions.
}
#endif

//***************************************************************************
// user_start
//***************************************************************************

int user_start(int argc, char *argv[])
{
#ifndef CONFIG_EXAMPLE_HELLOXX_NOSTACKCONST
  CHelloWorld HelloWorld;
#endif
  CHelloWorld *pHelloWorld = new CHelloWorld;

  printf("Saying hello from the dynamically constructed instance\n");
  pHelloWorld->HelloWorld();

#ifndef CONFIG_EXAMPLE_HELLOXX_NOSTACKCONST
  printf("Saying hello from the statically constructed instance\n");
  HelloWorld.HelloWorld();
#endif

#ifndef CONFIG_EXAMPLE_HELLOXX_NOSTATICCONST
  printf("Saying hello from the statically constructed instance\n");
  g_HelloWorld.HelloWorld();
#endif

  delete pHelloWorld;
  return 0;
}

