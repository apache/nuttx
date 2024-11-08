#!/usr/bin/env python3
############################################################################
# tools/perfaddmmap.py
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
############################################################################

import argparse
import ctypes
import mmap
import os

import elftools.elf.elffile as elffile

PERF_ELF_FILENAME_MAX_LEN = 64


class Descriptor(object):
    """Descriptor of a structure in the perf.data"""

    CTYPE_MAP = {
        "u16": ctypes.c_uint16,
        "u32": ctypes.c_uint32,
        "u64": ctypes.c_uint64,
        "char_p": ctypes.c_char * PERF_ELF_FILENAME_MAX_LEN,
    }

    def __init__(self, fields):
        class TraceItem(ctypes.Structure):
            _fields_ = Descriptor.CtypesFields(fields)

            def __str__(self):
                return ", ".join(
                    "%s: %s" % (field, self.__getattribute__(field))
                    for field, _ in TraceItem._fields_
                )

        self.ctype = TraceItem

    def Read(self, trace, offset):
        return self.ctype.from_buffer(trace, offset)

    def GetCtype(self):
        return self.ctype()

    @staticmethod
    def CtypesFields(fields):
        return [(field, Descriptor.CTYPE_MAP[format]) for (field, format) in fields]


TRACE_HEADER_DESC = Descriptor(
    [
        ("magic", "u64"),
        ("size", "u64"),
        ("attr_size", "u64"),
        ("attrs_offset", "u64"),
        ("attrs_size", "u64"),
        ("data_offset", "u64"),
        ("data_size", "u64"),
        ("event_types_offset", "u64"),
        ("event_types_size", "u64"),
    ]
)

PERF_ATTR_SIZE_VER0 = 64
PERF_EVENT_ATTR_DESC = Descriptor(
    [
        ("type", "u32"),
        ("size", "u32"),
        ("config", "u64"),
        ("sample_period_or_freq", "u64"),
        ("sample_type", "u64"),
        ("read_format", "u64"),
        ("flags", "u64"),
        ("wakeup_events_or_watermark", "u32"),
        ("bt_type", "u32"),
        ("bp_addr", "u64"),
        ("bp_len", "u64"),
        ("branch_sample_type", "u64"),
    ]
)

PERF_EVENT_HEADER_DESC = Descriptor([("type", "u32"), ("misc", "u16"), ("size", "u16")])

PERF_MMAP_EVENT_BODY_DESC = Descriptor(
    [
        ("pid", "u32"),
        ("tid", "u32"),
        ("start", "u64"),
        ("len", "u64"),
        ("pgoff", "u64"),
        ("file", "char_p"),
    ]
)

# perf_event_attr.sample_type bits control the set of
# perf_sample_event fields.
PERF_SAMPLE_IP = 1 << 0
PERF_SAMPLE_TID = 1 << 1
PERF_SAMPLE_TIME = 1 << 2
PERF_SAMPLE_ADDR = 1 << 3
PERF_SAMPLE_READ = 1 << 4
PERF_SAMPLE_CALLCHAIN = 1 << 5
PERF_SAMPLE_ID = 1 << 6
PERF_SAMPLE_CPU = 1 << 7
PERF_SAMPLE_PERIOD = 1 << 8
PERF_SAMPLE_STREAM_ID = 1 << 9
PERF_SAMPLE_RAW = 1 << 10

PERF_SAMPLE_EVENT_BODY_FIELDS = [
    ("ip", "u64", PERF_SAMPLE_IP),
    ("pid", "u32", PERF_SAMPLE_TID),
    ("tid", "u32", PERF_SAMPLE_TID),
    ("time", "u64", PERF_SAMPLE_TIME),
    ("addr", "u64", PERF_SAMPLE_ADDR),
    ("id", "u64", PERF_SAMPLE_ID),
    ("stream_id", "u64", PERF_SAMPLE_STREAM_ID),
    ("cpu", "u32", PERF_SAMPLE_CPU),
    ("res", "u32", PERF_SAMPLE_CPU),
    ("period", "u64", PERF_SAMPLE_PERIOD),
]

PERF_RECORD_MMAP = 1
PERF_RECORD_SAMPLE = 9

PERF_DATA_HEADER_POS = 0x28


class TraceReader(object):
    _TRACE_HEADER_MAGIC = 3622385352885552464  # PERFILE2

    def __init__(self, trace_name):
        self.trace_file = open(trace_name, "r")
        self.trace = mmap.mmap(self.trace_file.fileno(), 0, mmap.MAP_PRIVATE)
        self.trace_header = TRACE_HEADER_DESC.Read(self.trace, 0)
        if self.trace_header.magic != TraceReader._TRACE_HEADER_MAGIC:
            print("Warning: unsupported trace header magic")
        self.offset = self.trace_header.data_offset
        self.limit = self.trace_header.data_offset + self.trace_header.data_size
        assert (
            self.limit <= self.trace.size()
        ), "Trace data limit exceeds trace file size"
        self.header_size = ctypes.sizeof(PERF_EVENT_HEADER_DESC.ctype)
        assert (
            self.trace_header.attrs_size != 0
        ), "No perf event attributes found in the trace"
        self.attrs = PERF_EVENT_ATTR_DESC.Read(
            self.trace, self.trace_header.attrs_offset
        )

        self.sample_event_body_desc = self._SampleEventBodyDesc()

    def ReadEventHeader(self):
        if self.offset >= self.limit:
            return None, 0
        offset = self.offset
        header = PERF_EVENT_HEADER_DESC.Read(self.trace, self.offset)
        self.offset += header.size
        return header, offset

    def ReadSample(self, header, offset):
        sample = self.sample_event_body_desc.Read(self.trace, offset + self.header_size)
        return sample

    def Dispose(self):
        del self.attrs
        del self.trace_header
        self.trace.close()
        self.trace_file.close()

    def _SampleEventBodyDesc(self):
        sample_type = self.attrs.sample_type
        fields = []
        for field, format, bit in PERF_SAMPLE_EVENT_BODY_FIELDS:
            if (bit & sample_type) != 0:
                fields += [(field, format)]
        return Descriptor(fields)


def checkElfLength(filename):
    if len(filename) > PERF_ELF_FILENAME_MAX_LEN:
        raise argparse.ArgumentTypeError(f"{filename} is too long")
    return filename


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Add the mmap info for perf.data file")
    parser.add_argument(
        "-f",
        "--trace",
        default="perf.data",
        help="perf trace file name [default: perf.data]",
    )
    parser.add_argument(
        "-e",
        "--elffile",
        required=True,
        action="append",
        type=checkElfLength,
        help="ELF file name list",
    )

    options = parser.parse_args()

    pids = []

    trace_reader = TraceReader(options.trace)
    while True:
        header, offset = trace_reader.ReadEventHeader()
        if not header:
            break
        if header.type == PERF_RECORD_MMAP:
            print("perf.data have exist mmap info")
            exit(0)
        elif header.type == PERF_RECORD_SAMPLE:
            sample = trace_reader.ReadSample(header, offset)
            if sample.pid not in pids:
                pids.append(sample.pid)
            del sample

    tempfile = "tmp.perf.data"
    with open(tempfile, "wb+") as file_write:
        mmap_len = 0
        file_write.write(trace_reader.trace[: trace_reader.trace_header.data_offset])
        file_write.seek(trace_reader.trace_header.data_offset)
        for pid in pids:
            for elf in options.elffile:
                try:
                    with open(elf, "rb") as elf_file:
                        readelf = elffile.ELFFile(elf_file)
                        for segment in readelf.iter_segments():
                            if (
                                segment.header["p_type"] == "PT_LOAD"
                                and segment.header["p_flags"] & 0x1
                            ):
                                header = PERF_EVENT_HEADER_DESC.GetCtype()
                                header.type = PERF_RECORD_MMAP
                                header.misc = 0x02

                                mmap_info = PERF_MMAP_EVENT_BODY_DESC.GetCtype()
                                mmap_info.pid = pid
                                mmap_info.tid = pid
                                mmap_info.start = segment.header["p_vaddr"]
                                mmap_info.len = segment.header["p_filesz"]
                                mmap_info.pgoff = segment.header["p_offset"]
                                mmap_info.file = elf.encode()
                                header.size = ctypes.sizeof(
                                    PERF_EVENT_HEADER_DESC.ctype
                                ) + ctypes.sizeof(PERF_MMAP_EVENT_BODY_DESC.ctype)
                                mmap_len += header.size
                                file_write.write(
                                    ctypes.string_at(
                                        ctypes.byref(header), ctypes.sizeof(header)
                                    )
                                )
                                file_write.write(
                                    ctypes.string_at(
                                        ctypes.byref(mmap_info),
                                        ctypes.sizeof(mmap_info),
                                    )
                                )
                        elf_file.close()
                except FileNotFoundError:
                    print(elf + " FileNotFoundError")

        file_write.write(trace_reader.trace[trace_reader.trace_header.data_offset :])
        file_write.seek(0)
        trace_reader.trace_header.data_size += mmap_len
        header_binary = ctypes.string_at(
            ctypes.byref(trace_reader.trace_header),
            ctypes.sizeof(trace_reader.trace_header),
        )
        file_write.write(header_binary)

    file_write.close()
    trace_reader.Dispose()
    os.remove(options.trace)
    os.rename(tempfile, options.trace)
