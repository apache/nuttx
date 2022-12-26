/****************************************************************************
 * boards/sparc/s698pm/s698pm_dkit/scripts/linksparc.ld
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/*
 * The memory map looks like this:
 * +--------------------+ <- low memory
 * | .text              |
 * |        etext       |
 * |        ctor list   | the ctor and dtor lists are for
 * |        dtor list   | C++ support
 * |        _endtext    |
 * +--------------------+
 * | .data              | initialized data goes here
 * |        _sdata      |
 * |        _edata      |
 * +--------------------+
 * | .bss               |
 * |        __bss_start | start of bss, cleared by crt0
 * |        _end        | start of heap, used by sbrk()
 * +--------------------+
 * |    heap space      |
 * |        _ENDHEAP    |
 * |    stack space     | 
 * |        __stack     | top of stack
 * +--------------------+ <- high memory
 */

/*
 * User modifiable values:
 *
 * _CLOCK_SPEED               in Mhz (used to program the counter/timers)
 *
 * _PROM_SIZE                 size of PROM (permissible values are 128K, 256K,
 *                               512K, 1M, 2M, 4M, 8M and 16M)
 * _RAM_SIZE                  size of RAM (permissible values are 256K, 512K,
 *                               1M, 2M, 4M, 8M, 16M, and 32M)
 *
 */

/* Default values, can be overridden */

_PROM_SIZE = 16M;
_RAM_SIZE = 4M;

_RAM_START = 0x40000000;
_RAM_END = _RAM_START + _RAM_SIZE;

_PROM_START = 0x00000000;
_PROM_END = _PROM_START + _PROM_SIZE;

/*
 *  Alternate names without leading _.
 */

PROM_START = _PROM_START;
PROM_SIZE = _PROM_SIZE;
PROM_END = _PROM_END;

RAM_START = _RAM_START;
RAM_SIZE = _RAM_SIZE;
RAM_END = _RAM_END;

/*
 PROVIDE (__stack = _RAM_END);	//move to the file of stack.ld
*/
__stack = _RAM_END;

/* these are the maximum values */

MEMORY
{
  rom    (rx) : ORIGIN = 0x00000000, LENGTH = 16M
  ram  (rw!x) : ORIGIN = 0x40000000, LENGTH = 4M
}

OUTPUT_FORMAT("elf32-sparc")
OUTPUT_ARCH(sparc)
__DYNAMIC  =  0;
ENTRY(__start)
/*
 * stick everything in ram (of course)
 */
SECTIONS
{
  .text :
  {
    CREATE_OBJECT_SYMBOLS
    text_start = .;
    _text_start = .;
    *(.text)
    . = ALIGN (16);

    *(.eh_frame)
    . = ALIGN (16);
    
   /* *(.gnu.linkonce.t*) */

    /*
     * C++ constructors
     */
    __CTOR_LIST__ = .;
    LONG((__CTOR_END__ - __CTOR_LIST__) / 4 - 2)
    *(.ctors)
    LONG(0)
    __CTOR_END__ = .;
    __DTOR_LIST__ = .;
    LONG((__DTOR_END__ - __DTOR_LIST__) / 4 - 2)
    *(.dtors)
    LONG(0)
    __DTOR_END__ = .;

    _rodata_start = . ;
    *(.rodata*)
  /*  *(.gnu.linkonce.r*) */
    _erodata = ALIGN( 0x10 ) ;

    etext = ALIGN(0x10);
    _etext = .;
    *(.init)
    *(.fini)
    *(.lit)
    *(.shdata)
    . = ALIGN (16);
    _endtext = .;
  } >ram
  
  /*
  .dynamic	  : { *(.dynamic) 	} >ram
  .got		  : { *(.got) 		} >ram
  .plt   	  : { *(.plt) 		} >ram
  .hash		  : { *(.hash)		} >ram
  .dynrel	  : { *(.dynrel)	} >ram
  .dynsym	  : { *(.dynsym)	} >ram
  .dynstr	  : { *(.dynstr)	} >ram
  .hash		  : { *(.hash)		} >ram
  */
  
  .data :
  {
    data_start = .;
    _data_start = .;
    _sdata = . ;
    *(.data)
    /*
    *(.gnu.linkonce.d*)
    *(.gcc_except_table)
    */
    . = ALIGN(0x10);
    edata = .;
    _edata = .;
  } > ram
  .shbss :
  {
    *(.shbss)
  } 
  .bss :
  {
    __bss_start = ALIGN(0x10);
    _bss_start = .;
    bss_start = .;
    *(.bss)
    *(COMMON)
    end = .;
    _end = ALIGN(0x10);
    __end = ALIGN(0x10);
  }  > ram
  .noinit :
  {
    _snoinit = ABSOLUTE(.);
    *(.noinit*)
    _enoinit = ABSOLUTE(.);
  } > ram
  
  .jcr . (NOLOAD)  : { *(.jcr) 		}
  
  .stab . (NOLOAD) :
  {
    [ .stab ]
  }
  .stabstr . (NOLOAD) :
  {
    [ .stabstr ]
  }
  
}
