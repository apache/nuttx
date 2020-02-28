#!/usr/bin/env python
############################################################################
# tools/ide_exporter.py
#
#   Copyright (C) 2016 Kha Vo. All rights reserved.
#   Author: Kha Vo <canhkha@gmail.com>
#
#   Based on  convert_make2file_list.py and add_source_in_iar.py
#   Author: avyhovanec@yahoo.com
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

from __future__ import print_function
import os
import subprocess
import re
import sys
import argparse
from lxml import etree as ET
from copy import deepcopy

HELP = """
ide_exporter.pyis a tool for generation nuttx iar/keil workspace
usage: ide_exporter.py [-h] [-v] [-o OUT_DIR] [-d]
                       build_log {iar,uvision_gcc,uvision_armcc} template_dir

positional arguments:
  build_log             Log file from make V=1
  {iar,uvision_gcc,uvision_armcc}
                        The target IDE: iar, uvision_gcc, (uvision_armcc is
                        experimental)
  template_dir          Directory that contains IDEs template projects
                          template_nuttx.eww : iar template workspace
                          template_nuttx_main.ewp : iar template project
                          template_nuttx_lib.ewp : iar library project
                          or
                          template_nuttx.uvmpw : uVision template workspace
                          template_nuttx_main.uvproj : uVision template project
                          template_nuttx_lib.uvproj : uVision library project

optional arguments:
  -h, --help            show this help message and exit
  -v, --version         show program's version number and exit
  -o OUT_DIR, --output OUT_DIR
                        Output directory
  -d, --dump            Dump project structure tree
"""

IAR = 'iar'
UVISION_GCC = 'uvision_gcc'
UVISION_ARMCC = 'uvision_armcc'

COMPILE_PREFIX_LIST = ('CC: ', 'AS: ', 'CXX:')
LIB_PREFIX_LIST = ('AR: ')
LINK_PREFIX_LIST = ('LD: ')
MAKE_ENTER_DIR = 'Entering directory'
PREFIX_LEN = 4

IAR_EXT_REMAP = {r'gnu/(\w+)\.S$' : r'iar/\g<1>.S'}
ARMCC_EXT_REMAP = {r'gnu/(\w+)\.S$' : r'armcc/\g<1>.S',
                   r'(\w+)\.a$': r'\g<1>.lib'}
UVISION_GCC_EXT_REMAP = {}

# file ext to FileTye in uVision project
UVISION_FILE_TYPE_MAP = {'.c': '1', '.S' : '1', '.cxx' : '8', '.lib' : '4', '.a' : '4'}

# tags convention: tag[0] = root_tags, create if doesn't exist
#                  tag[1] = (sub_tag,) tag without text, create new
#                  tag[2] = (leaf_tag,) with text, create new
IAR_PRJ_SETTINGS = {'group_tags' : ('', ('group',), ('name', )),
                    'file_tags' :('', ('file',), ('name', )),
                    'rel_base' : '$PROJ_DIR$/',
                    'cleared_nodes' : ('group', 'file', ),
                    'include_pnodes' : (".//*[name='CCIncludePath2']", ".//*[name='AUserIncludes']"),
                    'include_tag' : 'state',
                    'output_path' : {'exe' : 'Obj', 'obj' : 'Obj', 'lst' : 'Lst'},
                    'ext_remap' : IAR_EXT_REMAP,}

IAR_WSP_SETTINGS = {'group_tags': ('',),
                    'file_tags':('', ('project', ), ('path',)),
                    'rel_base':'$WS_DIR$/',
                    'cleared_nodes': ('project', )}


UVISION_ARMCC_PRJ_SETTINGS = {'root_group':'',
                              'group_tags': ('.//Targets/Target/Groups', ('Group', ), ('GroupName', )),
                              'file_tags':('Files', ('File', ), ('FileName', 'FileType', 'FilePath', )),
                              'rel_base':'',
                              'cleared_nodes': ('.//Group', ),
                              'include_pnodes' : ".//VariousControls/IncludePath",
                              'output_path' : {'exe' : 'Obj', 'obj' : 'Obj', 'lst' : 'Lst'},
                              'ext_remap' : ARMCC_EXT_REMAP,
                              'uv_file_type' : UVISION_FILE_TYPE_MAP}

UVISION_GCC_PRJ_SETTINGS = {'root_group':'',
                            'group_tags': ('.//Targets/Target/Groups', ('Group', ), ('GroupName', )),
                            'file_tags':('Files', ('File', ), ('FileName', 'FileType', 'FilePath', )),
                            'rel_base':'',
                            'cleared_nodes': ('.//Group', ),
                            'include_pnodes' : ".//VariousControls/IncludePath",
                            'saved_tags' : ('.//FileOption', ),
                            'output_path' : {'exe' : 'Obj', 'obj' : 'Obj', 'lst' : 'Lst'},
                            'ext_remap' : UVISION_GCC_EXT_REMAP,
                            'uv_file_type' : UVISION_FILE_TYPE_MAP,
                            'c_misc' : ('.//Carm', '-fno-builtin -Wall -Wstrict-prototypes -Wshadow -Wundef -g'),
                            'cxx_misc' : ('.//Carm', '-fno-builtin -fno-exceptions -fcheck-new -fno-rtti -Wall -Wshadow -Wundef -g'),
                            'ld_misc' : ('.//LDarm', '--entry=__start -lgcc'),
                            'cxx_def' : ('.//Carm', ''),}

UVISION_WSP_SETTINGS = {'group_tags': ('',),
                        'file_tags':('', ('project', ), ('PathAndName', )),
                        'rel_base':'',
                        'cleared_nodes': ('project', )}


LIB_EXTS = ('.a', '.lib')
ASM_EXTS = ('.s', '.S')


def get_common_dir(dir_list):
    """ Get common parent directory of a given directory list
    """
    com_dir = dir_list[0]
    found = False
    while found == False:
        found = True
        com_dir = os.path.split(com_dir)[0]
        for directory in dir_list:
            if com_dir not in directory:
                found = False
                break

    if found:
        return com_dir
    else:
        return "/" #return root


class SourceInfo(object):
    """Source file information

    Attributes:
        src: source file
        include: List of including dir in compiled command
        flags: other compiled flags
    """
    def __init__(self, src, include=None, flags=''):
        self.include = []
        if include is not None:
            self.include = include
        self.src = src
        self.flags = flags
        self.include = include

    @staticmethod
    def get_common_src_dir(sinfo_list):
        """ Get Common directory from list of source code
        """
        source_list = [info.src for info in sinfo_list]
        com_dir = get_common_dir(source_list)
        return com_dir

    @staticmethod
    def get_including_set(sinfo_list):
        """ Get including set from list of source code
        """
        include_set = set()
        for sinfo in sinfo_list:
            for inc in sinfo.include:
                if inc != '':
                    include_set.add(inc)
        return include_set

class IdeProject(object):
    """Base IDE project class.

    make_src_nodes(self, source,  group = None, parent_node = None):
    make_include(self, sources, parent_node = None):
    make_output_dir(self, target):
    These functions need to override

    Attributes:
        root: root of its etree
        ewp_ET: Its etree
        settings: specific IDE setting
        proj_dir: base directory, files path are often relative from this
        rel_base: base directory symbol,
    """

    def __init__(self, proj, settings, out_dir=None, use_gcc=False):
        self.proj_dir = os.path.split(proj)[0]
        if (out_dir is not None) and os.path.exists(out_dir):
            self.proj_dir = out_dir

        self.root = None
        self.ewp_ET = None
        self.settings = {}
        self.rel_base = ''
        if settings is not None:
            self.settings = settings
        self.use_gcc = use_gcc

        self.rel_base = self.settings.get('rel_base', '')
        self.saved_nodes = {} #some inside nodes need to save before clear all sources
        try:
            #Read template project xml structure
            parser = ET.XMLParser(remove_blank_text=True) # use parser to make pretty print works
            self.ewp_ET = ET.parse(proj, parser)
            self.root = self.ewp_ET.getroot()

            # Save some template nodes before clear
            for tag in self.settings.get('saved_tags', []):
                n = self.root.find(tag)
                self.saved_nodes[tag] = deepcopy(n)

            self.clear_src_nodes() # Clear all source node in template file
        except Exception as e:
            print("ERR: {0}".format(str(e)))
            raise Exception("Can't init IdeProject object")

    def get_relpath(self, dest):
        """ Get relative path from its base directory
        """
        return self.rel_base + os.path.relpath(dest, self.proj_dir)

    def get_output_dir(self):
        """
        """
        out_paths = self.settings.get('output_path', {})
        return out_paths.get('exe', '')

    def get_obj_dir(self):
        """
        """
        out_paths = self.settings.get('output_path', {})
        return out_paths.get('obj', '')

    def get_lst_dir(self):
        """
        """
        out_paths = self.settings.get('output_path', {})
        return out_paths.get('lst', '')

    def write(self, ofile):
        """ Write etree to file
        """
        self.ewp_ET.write(ofile, pretty_print=True, xml_declaration=True, encoding='UTF-8')

    def remove_nodes(self, element, remove_list):
        """Delete nodes in list from the xlm tree

        Args:
            element: root node of etree
            remove_list: tuple of all node that need to remove

        Returns:
            Remove nodes from xlm tree
        Raises:
            None
        """
        try:
            for node in remove_list:
                p = element.find(node)
                while  p is not None:
                    c = p.getparent()
                    c.remove(p)
                    p = element.find(node)
        except Exception as e:
            print(str(e))

    def clear_src_nodes(self):
        """ Remove all predefined node in settings from its etree
        """
        self.remove_nodes(self.root, self.settings.get('cleared_nodes', []))

    def make_nodes(self, parent_node, tags, *args):
        """ Create node(s) by using tag convention
            Return most inner parent nodes
        """
        if parent_node is None:
            parent_node = self.root

        head = None
        root = None
        #print "Create tags: ", tags
        if len(tags) == 3:

            # Check root, create if not exist
            root_tag = tags[0]
            if root_tag != '':
                root = parent_node.find(root_tag)
                if root is None:
                    root = ET.SubElement(parent_node, root_tag.split('/')[-1])
            else:
                root = parent_node

            p_node = root

            # Create middle sub nodes
            sub_tags = tags[1]
            if len(sub_tags) > 0:
                head = ET.Element(sub_tags[0])
                p_node = head
                for tag in sub_tags[1:]:
                    p_node = ET.SubElement(p_node, tag)

            # Create leaf node with input text
            for src_tag, text in zip(tags[2], args):
                e = ET.SubElement(p_node, src_tag)
                e.text = text

            if head is not None:
                root.append(head)
        else:
            raise Exception('Wrong tag convention')
        return p_node

    def make_group(self, parent_node, *args):
        """ Create group of source/lib tags
            Tags info are get from settings
        Args:
            parent_node :
            *args : nodes' text
        return:
            Return group node
        """
        tags = self.settings.get('group_tags', [])
        return self.make_nodes(parent_node, tags, *args)

    def make_file(self, parent_node, *args):
        """ Create group of source/lib tags
            Tags info are get from settings
        """
        tags = self.settings.get('file_tags', [])
        return self.make_nodes(parent_node, tags, *args)

    def make_src_nodes(self, source, group=None, parent_node=None):
        """ Create xlm nodes for list of source file

        Args:
            sources: list of SourceInfo
            group : group name that contains all of these source
            parent_node : etree fake root node
        Returns:

        """
        pass

    def make_include(self, sources, parent_node=None):
        """ Create including nodes from source info for project

        Args:
            sources: list of SourceInfo
            parent_node: etree fake root node
        Returns:

        """
        pass

    def make_output_dir(self, target):
        """ Update output directory setting for project

        Args:
            target: project output target name
        Returns:

        """
        pass
    def add_misc(self, mtype, misc=''):
        pass
    def add_define(self, dtype, symbols):
        pass
    def set_link_libs(self, lib_dir, libs):
        pass
    def set_mcu(self, mcu):
        pass
    def set_core(self, core):
        pass

    @staticmethod
    def factory(objtype, xml_file, out_dir=None):
        """ Factory to create obj by derived type
        """
        return objtype(xml_file, out_dir=out_dir)

class IARWorkspace(IdeProject):
    """IAR workspace class.

    Depend on its settings only.
    Use default add node from base to add sub library project

    Attributes:
    """
    def __init__(self, proj, out_dir=None):
        super(IARWorkspace, self).__init__(proj, IAR_WSP_SETTINGS, out_dir)

class IARProject(IdeProject):
    """IAR project class.

    Add some specific logics to create source, include and output setting

    """
    def __init__(self, proj, settings=IAR_PRJ_SETTINGS, out_dir=None):
        super(IARProject, self).__init__(proj, settings, out_dir)

    def make_include(self, sources, parent_node=None):
        """ Create including nodes from source info for project
        IAR sample including nodes
            <option>
              <name>CCIncludePath2</name>
              <state>$PROJ_DIR$\nuttx\include\</state>
            </option>
        Args:
            sources: list of SourceInfo
            parent_node: etree fake root node
        Returns:

        """
        if parent_node is None:
            parent_node = self.root

        include_set = SourceInfo.get_including_set(sources)

        # Adding dir to user include node, tags is from setting
        include_nodes = self.settings['include_pnodes']
        for path in include_nodes:
            for p in parent_node.iterfind(path): # ex: ".//*[name='CCIncludePath2']"
                #print(n.tag, n.text)
                for inc in include_set:
                    state = ET.SubElement(p, self.settings['include_tag'])

                    # In cygwin, we need to convert windows path to relative
                    if sys.platform == 'cygwin':
                        inc = subprocess.check_output(['cygpath', '-u', inc])
                        inc = inc[:-1] #remove /n

                    state.text = self.get_relpath(inc)


    def make_src_nodes(self, sources, group=None, parent_node=None):
        """ Create nodes for list of source file

        Args:
            sources: list of SourceInfo
            group: group name that contains all of these source
            parent_node: etree fake root node
        Returns:

        """
        if parent_node is None:
            parent_node = self.root

        source_list = [info.src for info in sources]

        com_dir = get_common_dir(source_list)
        com_dir_name = os.path.split(com_dir)[1]

        if group is None:
            group = com_dir_name

        # Create group node to contain all source files
        group_node = self.make_group(parent_node, group)

        # Add source files to group as sub node
        for src in source_list:
            fname = self.get_relpath(src)  # make ref path from $PROJ_DIR$ to file

            ext_remap = self.settings.get('ext_remap', {})
            for ext, replacement in ext_remap.items():
                fname = re.sub(ext, replacement, fname)

            self.make_file(group_node, fname)

    def make_output_dir(self, target):
        """ Update output directory setting for IAR project

        Args:
            target: project's target name
        Returns:

        """
        sub_dir = '$PROJ_FNAME$'
        exe_path = self.get_output_dir()
        lst_path = self.get_output_dir()
        obj_path = self.get_output_dir()
        dirs = (exe_path, obj_path, lst_path)
        tags = ('.//*[name="ExePath"]', './/*[name="ObjPath"]', './/*[name="ListPath"]')

        for path, tag in zip(dirs, tags):
            if path != '':
                p = self.root.findall(tag)
                for n in p:
                    self.remove_nodes(n, ('state', ))
                    e = ET.SubElement(n, 'state')
                    e.text = sub_dir + '/' + path

class UVisionWorkspace(IdeProject):
    """uVision workspace class.

    Depend on its settings only.
    Use default add node from base to add sub library project

    Attributes:
    """
    def __init__(self, proj, out_dir=None):
        super(UVisionWorkspace, self).__init__(proj, UVISION_WSP_SETTINGS, out_dir)

class UVisionProject(IdeProject):
    """uVision project class.

    Add some specific logics to create source, include and output setting

    """
    def __init__(self, proj, settings=UVISION_ARMCC_PRJ_SETTINGS, out_dir=None, use_gcc=False):
        super(UVisionProject, self).__init__(proj, settings, out_dir, use_gcc)
        self.use_gcc = use_gcc

    def make_include(self, sources, parent_node=None):
        """ Create including nodes from source info for uVision project
        uVision sample including nodes:
            <VariousControls>
              <IncludePath>../../../../apps/examples/hello;../../../../apps/examples/nsh>
            </VariousControls>
        Args:
            sources: list of SourceInfo
            parent_node: etree fake root node
        Returns:

        """
        if parent_node is None:
            parent_node = self.root

        include_set = SourceInfo.get_including_set(sources)

        incs = []
        for inc in include_set:
            # In cygwin, we need to convert windows path to relative
            if sys.platform == 'cygwin':
                inc = subprocess.check_output(['cygpath', '-u', inc])
                inc = inc[:-1] #remove /n

            inc = self.get_relpath(inc)
            incs.append(inc)

        inc_text = ';'.join(incs)

        # Adding dir to user include node (both ASM & CC)
        for n in parent_node.iterfind(self.settings['include_pnodes']):
            n.text = inc_text


    def make_src_nodes(self, sources, group=None, parent_node=None):
        """ Create nodes for list of source file
        Sample uVision file:
          <Groups>
            <Group>
              <GroupName>board</GroupName>
              <Files>
                <File>
                  <FileName>stm32_boot.c</FileName>
                  <FileType>1</FileType>
                  <FilePath>../../../arch/arm/src/board/stm32_boot.c</FilePath>
                </File>
              </Files>
            </Group>
          </Groups>

        Args:
            sources: list of SourceInfo
            group: group name that contains all of these source
            parent_node: etree fake root node
        Returns:

        """
        if parent_node is None:
            parent_node = self.root

        source_list = [info.src for info in sources]

        com_dir = get_common_dir(source_list)
        com_dir_name = os.path.split(com_dir)[1]

        if group is None:
            group = com_dir_name

        # Create group node to contain all source files
        group_node = self.make_group(parent_node, group) # return <Group> node

        # Add source files to group as sub node
        for src in source_list:
            fname = self.get_relpath(src)
            ext = os.path.splitext(fname)[1]

            # get uVison FileType
            uv_file_type = self.settings.get('uv_file_type', {})
            ftype = uv_file_type.get(ext, '0')

            # Translate source to new format/location if need
            ext_remap = self.settings.get('ext_remap', {})
            for find, replacement in ext_remap.items():
                fname = re.sub(find, replacement, fname)

            name = os.path.split(fname)[1]
            file_node = self.make_file(group_node, name, ftype, fname)

            # Make exception for .S file (treat as C source with D__ASSEMBLY__)
            if (self.use_gcc) and (ext in ASM_EXTS):
                asm_opt_node = self.saved_nodes.get('.//FileOption')
                if asm_opt_node is not None:
                    file_node.append(deepcopy(asm_opt_node))

    def make_output_dir(self, target):
        """ Update output directory setting for IAR project

        Args:
            target: project's target name
        Returns:

        """

        exe_path = self.get_output_dir()
        if exe_path != '':
            p = self.root.find('.//OutputDirectory')
            if p is not None:
                p.text = '\\'.join(('.', target, exe_path, ''))

        lst_path = self.get_lst_dir()
        if lst_path != '':
            p = self.root.find('.//ListingPath')
            if p is not None:
                p.text = '\\'.join(('.', target, lst_path, ''))

        p = self.root.find('.//OutputName')
        if p is not None:
            p.text = re.sub(r'^lib(.*)$', r'\g<1>', target) # prevent liblibapps.a

    def add_misc(self, mtype, misc=''):
        misc_info = self.settings.get(mtype)
        if misc_info is not None:
            tag, default = misc_info
            if misc == '':
                misc = default
            n = self.root.find(tag)
            if n is not None:
                m = n.find('.//MiscControls')
                if m is not None:
                    m.text = (m.text or '') + ' ' + misc

    def add_define(self, dtype, symbols):
        def_info = self.settings.get(dtype)
        if def_info is not None:
            tag, default = def_info
            n = self.root.find(tag)
            if n is not None:
                m = n.find('.//Define')
                if m is not None:
                    m.text = (m.text or '') + ' ' + symbols

    def set_link_libs(self, libs, lib_dir='.\\lib'):
        if self.use_gcc:
            # need to add static lib in group so that linker does not throw errors
            # http://eli.thegreenplace.net/2013/07/09/library-order-in-static-linking
            mist_text = ' -Wl,--start-group'

            for sinfo in libs:
                lib = os.path.split(sinfo.src)[1]
                name, ext = os.path.splitext(lib)
                mist_text += ' -l' + name[3:] #remove lib in 'libAAA'

            mist_text += ' -Wl,--end-group'

            misc_info = self.settings.get('ld_misc')
            if misc_info is not None:
                tag, default = misc_info
                n = self.root.find(tag)
                if n is not None:
                    m = n.find('.//Misc')
                    if m is not None:
                        m.text += mist_text

                    m = n.find('.//IncludeDir')
                    if m is not None:
                        m.text = lib_dir

    def set_mcu(self, mcu):
        #TODO:
        pass
    def set_core(self, core):
        #TODO:
        pass

class UVisionARMCCProject(UVisionProject):
    """uVision for ARMCC project class.

    Add some specific logics to create source, include and output setting

    """
    def __init__(self, proj, out_dir=None):
        super(UVisionARMCCProject, self).__init__(proj, UVISION_ARMCC_PRJ_SETTINGS, out_dir)

class UVisionGCCProject(UVisionProject):
    """uVision for GCC project class.

    Add some specific logics to create source, include and output setting

    """
    def __init__(self, proj, settings=UVISION_GCC_PRJ_SETTINGS, out_dir=None, use_gcc=True):
        super(UVisionGCCProject, self).__init__(proj, settings, out_dir, use_gcc)


def get_project_structure(lines):
    """Get project structure from make log file.

    Loop through make log to figure the project structure

    Args:
        lines: A list of line from make log file (make V=1)

    Returns:
        A dict mapping library/target to its source list

        {
            'libc.a': [
                        {'src':/mynuttx/nuttx/libc/string/lib_strcat.c', 'include' : ['.', 'others include', ], 'flags': ''},
                        {'src':/mynuttx/nuttx/libc/string/lib_memcpy.c', 'include' : ['.', 'others include', ], 'flags': ''},

                      ],
        }
        Source list is in full path form
    Raises:
        An error occurred when can't parse lib/target name
    """

    group_dict = {}
    src_list = []
    make_path = ''
    src_path = ''
    ar_cmd = ''
    cc_cmd = ''

    for line in lines:

        _lp = line[:PREFIX_LEN]

        if _lp in COMPILE_PREFIX_LIST:
            src_path = os.path.join(make_path, line[PREFIX_LEN:].strip())
            cc_cmd = line.strip()

        elif _lp in LIB_PREFIX_LIST:
            ar_cmd = line.strip()

        elif _lp in LINK_PREFIX_LIST:
            match = re.search(_lp + r'(\w+)', line)
            if match:
                target = match.group(1)
                if target not in group_dict:
                    group_dict[target] = []

                for src in src_list:
                    group_dict[target].append(src)

        elif MAKE_ENTER_DIR in line:   # Get current make directory
            match = re.search(r"'(.+)'\n$", line)
            if match:
                make_path = match.group(1)
        elif cc_cmd != '': # Get include dirs and flags
            incs = [make_path]
            match = re.findall(r'(-I|-isystem) "(.+?)"', line)
            if match:
                incs += [p[1] for p in match]

            # TODO: parse and other compile flags

            src_info = SourceInfo(src_path, incs)
            src_list.append(src_info)

            cc_cmd = ''
            src_path = ''
        elif ar_cmd != '':    #put all compiled files to library source list
            match = re.search(r'(\w+?\.a)', line)
            if match:
                lib_name = match.group(1)       # get library name
                if lib_name not in group_dict:
                    group_dict[lib_name] = []  # create empty source info list

                lib_objs = re.findall(r'(\w+?)\.o', line)   # Get all obj name in libs (without ext)
                #print("OBJ in .a: ", lib_objs)
                remain_src_list = []
                for sinfo in src_list:
                    obj = os.path.basename(sinfo.src)
                    obj = os.path.splitext(obj)[0]      # Get the obj name  (without ext) from source file name

                    #print("OBJ from file: ", obj)
                    if obj in lib_objs:  # make sure the lib include this obj
                        group_dict[lib_name].append(sinfo)
                        #print('Put' + sinfo.src + "to lib: " + lib_name)
                    else:
                        remain_src_list.append(sinfo)
                        #print('Remain' + sinfo.src + " not in lib: " + lib_name)

                src_list = remain_src_list
                ar_cmd = ""

            else:
                raise AssertionError("Can't parse lib name ", line)
    return group_dict

def dump_project_struct(project_structure):
    """Dump project structure

    Print project structure

    Args:
        A dict mapping library/target to its source list

        {
            'libc.a': [
                        {'src':/mynuttx/nuttx/libc/string/lib_strcat.c', 'include' : ['.', 'others include', ], 'flags': ''},
                        {'src':/mynuttx/nuttx/libc/string/lib_memcpy.c', 'include' : ['.', 'others include', ], 'flags': ''},

                      ],
        }

    Returns:

    Raises:
        None
    """

    for lib, sinfo_list in project_structure.items():
        print(lib)
        for sinfo in sinfo_list:
            print('\t' + sinfo.src)


IAR_EXPORT = {'main': {'t' : IARProject, 'file' : 'template_nuttx_main.ewp'},
              'lib':  {'t' : IARProject, 'file' : 'template_nuttx_lib.ewp'},
              'workspace': {'t' : IARWorkspace, 'file' : 'template_nuttx.eww'}}

UVISION_ARMCC_EXPORT = {'main': {'t' : UVisionProject, 'file' : 'template_nuttx_main.uvproj'},
                        'lib':  {'t' : UVisionProject, 'file' : 'template_nuttx_lib.uvproj'},
                        'workspace': {'t' : UVisionWorkspace, 'file' : 'template_nuttx.uvmpw'}}

UVISION_GCC_EXPORT = {'main': {'t' : UVisionGCCProject, 'file' : 'template_nuttx_main.uvproj'},
                      'lib':  {'t' : UVisionGCCProject, 'file' : 'template_nuttx_lib.uvproj'},
                      'workspace': {'t' : UVisionWorkspace, 'file' : 'template_nuttx.uvmpw'}}

IDE_CONFIG_DICT = {IAR : IAR_EXPORT,
                   UVISION_GCC : UVISION_GCC_EXPORT,
                   UVISION_ARMCC : UVISION_ARMCC_EXPORT}


if __name__ == '__main__':

    parser = argparse.ArgumentParser(version='1.1')

    parser.add_argument('build_log',
                        help='Log file from make V=1',
                        type=argparse.FileType('rt'))

    parser.add_argument('ide',
                        choices=[IAR, UVISION_GCC, UVISION_ARMCC],
                        help="The target IDE: iar, uvision_gcc, (uvision_armcc is experimental)")

    parser.add_argument('template_dir',
                        help='Directory that contains IDEs template projects')

    parser.add_argument('-o', '--output',
                        action='store',
                        dest='out_dir',
                        help="Output directory")

    parser.add_argument('-d', '--dump',
                        action='store_true',
                        dest='dump', default=False,
                        help="Dump project structure tree")

    options = parser.parse_args()

    fmake_log = options.build_log

    # read log file
    lines = fmake_log.readlines()
    fmake_log.close()

    # get project structure
    project = get_project_structure(lines)

    if options.dump:
        dump_project_struct(project)

    templ_dir = options.template_dir
    if not os.path.exists(templ_dir):
        print(templ_dir + " does not exist")
        exit(1)

    prj_dir = templ_dir
    if options.out_dir is not None:
        prj_dir = os.path.abspath(options.out_dir)

    try:
        if not os.path.exists(prj_dir):
            os.makedirs(prj_dir)
    except Exception as e:
        print("ERR: {0}".format(str(e)))
        exit(1)


    ide_config = IDE_CONFIG_DICT[options.ide]
    xml_file = os.path.join(templ_dir, ide_config['workspace']['file'])
    ws_ext = os.path.splitext(xml_file)[1]
    ws = IdeProject.factory(ide_config['workspace']['t'], xml_file, prj_dir)



    target = {'libs':[], 'sources': []}

    # Create nuttx iar library projects
    for lib, group_src_list in project.items():
        lib_name, lib_ext = os.path.splitext(lib)
        if len(group_src_list) < 1:
            print(lib_name, group_src_list)
        elif lib_ext in LIB_EXTS:
            xml_file = os.path.join(templ_dir, ide_config['lib']['file'])
            lib_prj = IdeProject.factory(ide_config['lib']['t'], xml_file, prj_dir)

            #print lib_name, group_src_list
            lib_prj.make_src_nodes(group_src_list)
            lib_prj.make_include(group_src_list)
            lib_prj.make_output_dir(lib_name)
            if lib_name == 'libxx':
                lib_prj.add_misc('cxx_misc')
                lib_prj.add_define('cxx_def', 'CONFIG_WCHAR_BUILTIN')
            else:
                lib_prj.add_misc('c_misc')

            # save main xml project to file
            xml_ext = os.path.splitext(xml_file)[1]
            prj_fname = os.path.join(prj_dir, lib_name + xml_ext)
            lib_prj.write(prj_fname)
            print("Exported " + prj_fname)

            # Add library project to workspace
            ws.make_file(ws.root, ws.get_relpath(prj_fname))

            # Store output library file to ref from main project later
            exe_dir = lib_prj.get_output_dir()
            lib_fname = os.path.join(prj_dir, lib_name, exe_dir, lib)
            target['libs'].append(SourceInfo(lib_fname))
        else:
            # Save name and source list for main project
            target['name'] = lib
            target['sources'] = group_src_list


    # Create nuttx main project
    xml_file = os.path.join(templ_dir, ide_config['main']['file'])
    main_prj = IdeProject.factory(ide_config['main']['t'], xml_file, prj_dir)

    main_prj.make_src_nodes(target['sources'])
    main_prj.make_include(target['sources'])

    if main_prj.use_gcc:
        target['libs'].append(SourceInfo('libgcc.a')) # need add libgcc in ld
        main_prj.set_link_libs(target['libs'])
    else:
        main_prj.make_src_nodes(target['libs'], group='libs')

    main_prj.make_output_dir(target['name'])


    # save main xml project to file
    xml_ext = os.path.splitext(xml_file)[1]
    prj_fname = os.path.join(prj_dir, target['name'] + '_main' + xml_ext)
    main_prj.write(prj_fname)
    print("Exported " + prj_fname)

    # Add main project to workspace
    ws.make_file(ws.root, ws.get_relpath(prj_fname))

    # Write nuttx workspace
    ww_fname = os.path.join(prj_dir, 'nuttx' + ws_ext)
    ws.write(ww_fname)
    print("Exported " + ww_fname)
