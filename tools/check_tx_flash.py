#!/usr/bin/env python3
"""Fail if the transmitter image reaches into the flight-log region.

The flight recorder owns the SAMD51 flash from FLOG_BASE (0x00040000,
transmitter/firmware/flight_log.cpp) for 256 KiB. The sketch is linked
after the 16 KiB UF2 bootloader, so it has 0x40000 - 0x4000 bytes; an
image that grows past FLOG_BASE would be overwritten by the first flight
log (or erase it). This reads the ELF's loadable segments - no toolchain
needed - and checks the highest flash byte any of them occupies.

Usage: python3 tools/check_tx_flash.py <build-path>/firmware.ino.elf
"""
import re
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FLASH_END_LIMIT = 0x00040000      # must match FLOG_BASE
RAM_BASE = 0x20000000             # SAMD51 SRAM: anything below is flash


def flog_base():
    src = (ROOT / "transmitter/firmware/flight_log.cpp").read_text()
    m = re.search(r"#define\s+FLOG_BASE\s+(0x[0-9A-Fa-f]+)u?", src)
    return int(m.group(1), 16) if m else None


def flash_end(elf_path):
    data = Path(elf_path).read_bytes()
    if data[:4] != b"\x7fELF" or data[4] != 1:
        raise SystemExit(f"{elf_path}: not a 32-bit ELF")
    endian = "<" if data[5] == 1 else ">"
    e_phoff, = struct.unpack_from(endian + "I", data, 28)
    e_phentsize, e_phnum = struct.unpack_from(endian + "HH", data, 42)
    end = 0
    for i in range(e_phnum):
        (p_type, _off, _vaddr, p_paddr, p_filesz, _memsz, _flags,
         _align) = struct.unpack_from(endian + "8I", data, e_phoff + i * e_phentsize)
        if p_type == 1 and p_filesz and p_paddr < RAM_BASE:   # PT_LOAD in flash
            end = max(end, p_paddr + p_filesz)
    return end


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    limit = flog_base() or FLASH_END_LIMIT
    if limit != FLASH_END_LIMIT:
        print(f"note: FLOG_BASE is 0x{limit:08X}, checking against it")
    end = flash_end(sys.argv[1])
    print(f"TX image ends at 0x{end:08X}; flight-log region starts at 0x{limit:08X} "
          f"({limit - end} bytes to spare)")
    if end > limit:
        print("::error::transmitter image overlaps the flight-log region")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
