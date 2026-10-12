#!/usr/bin/env python3
"""Disassemble named AArch64 ELF functions and annotate direct calls.

This is intentionally small and read-only. It is useful for Android .so files
where the host objdump cannot decode AArch64. Dependencies:

    python -m pip install capstone pyelftools

Example:
    PYTHONPATH=artifacts/python python tools/re_probe/elf_arm64_trace.py \
      artifacts/.../libcameraSdk.so 'ScannerMgr::getDepthStreamInfo'
"""

from __future__ import annotations

import argparse
import re
import subprocess
from pathlib import Path

from capstone import CS_ARCH_ARM64, CS_MODE_LITTLE_ENDIAN, Cs
from elftools.elf.elffile import ELFFile


def demangle(name: str) -> str:
    if not name.startswith("_Z"):
        return name
    result = subprocess.run(
        ["c++filt", name],
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() or name


def symbol_maps(elf: ELFFile) -> tuple[dict[int, str], list[tuple[str, int, int]]]:
    names: dict[int, str] = {}
    functions: list[tuple[str, int, int]] = []
    for section_name in (".dynsym", ".symtab"):
        section = elf.get_section_by_name(section_name)
        if section is None:
            continue
        for symbol in section.iter_symbols():
            value = int(symbol["st_value"])
            size = int(symbol["st_size"])
            if not value or symbol["st_info"]["type"] != "STT_FUNC":
                continue
            name = demangle(symbol.name)
            names.setdefault(value, name)
            if size:
                functions.append((name, value, size))

    # AArch64 PLT: 32-byte resolver header followed by 16-byte stubs in the
    # same order as .rela.plt. Annotating these turns "bl 0x..." into imports.
    plt = elf.get_section_by_name(".plt")
    rela = elf.get_section_by_name(".rela.plt")
    dynsym = elf.get_section_by_name(".dynsym")
    if plt is not None and rela is not None and dynsym is not None:
        base = int(plt["sh_addr"]) + 32
        for index, relocation in enumerate(rela.iter_relocations()):
            symbol = dynsym.get_symbol(relocation["r_info_sym"])
            names[base + index * 16] = f"{demangle(symbol.name)}@plt"
    return names, functions


def containing_section(elf: ELFFile, address: int, size: int):
    for section in elf.iter_sections():
        start = int(section["sh_addr"])
        end = start + int(section["sh_size"])
        if start <= address and address + size <= end:
            return section
    return None


def cstring_at(elf: ELFFile, address: int) -> str | None:
    section = containing_section(elf, address, 1)
    if section is None:
        return None
    offset = address - int(section["sh_addr"])
    data = section.data()[offset : offset + 256]
    raw = data.split(b"\0", 1)[0]
    if len(raw) < 2:
        return None
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        return None
    if not all(char.isprintable() or char in "\r\n\t" for char in text):
        return None
    return text


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path)
    parser.add_argument("symbol", help="case-insensitive regex matched after demangling")
    parser.add_argument(
        "--raw-address",
        type=lambda value: int(value, 0),
        help="disassemble this virtual address instead of matching a symbol",
    )
    parser.add_argument(
        "--raw-size",
        type=lambda value: int(value, 0),
        default=0x400,
        help="byte count for --raw-address (default: 0x400)",
    )
    parser.add_argument("--max-instructions", type=int, default=1200)
    parser.add_argument(
        "--from-offset",
        type=lambda value: int(value, 0),
        default=0,
        help="skip this many bytes from each matched function start",
    )
    parser.add_argument(
        "--byte-count",
        type=lambda value: int(value, 0),
        help="disassemble at most this many bytes after --from-offset",
    )
    args = parser.parse_args()

    pattern = re.compile(args.symbol, re.IGNORECASE)
    with args.elf.open("rb") as stream:
        elf = ELFFile(stream)
        if elf["e_machine"] != "EM_AARCH64":
            parser.error(f"{args.elf} is {elf['e_machine']}, not AArch64")
        address_names, functions = symbol_maps(elf)
        if args.raw_address is not None:
            matches = [("raw", args.raw_address, args.raw_size)]
        else:
            matches = sorted(
                (item for item in functions if pattern.search(item[0])),
                key=lambda item: item[1],
            )
        if not matches:
            parser.error(f"no function symbol matches {args.symbol!r}")

        disassembler = Cs(CS_ARCH_ARM64, CS_MODE_LITTLE_ENDIAN)
        for name, address, size in matches:
            section = containing_section(elf, address, size)
            if section is None:
                print(f"\n{name} @ 0x{address:x}: section not found")
                continue
            function_offset = address - int(section["sh_addr"])
            start = min(args.from_offset, size)
            count = size - start
            if args.byte_count is not None:
                count = min(count, args.byte_count)
            code = section.data()[
                function_offset + start : function_offset + start + count
            ]
            decode_address = address + start
            print(
                f"\n=== {name} @ 0x{address:x}, {size} bytes "
                f"(showing +0x{start:x}..+0x{start + count:x}) ==="
            )
            page_registers: dict[str, int] = {}
            for index, instruction in enumerate(
                disassembler.disasm(code, decode_address)
            ):
                if index >= args.max_instructions:
                    print("... instruction limit reached ...")
                    break
                annotation = ""
                if instruction.mnemonic in ("bl", "b"):
                    try:
                        target = int(instruction.op_str.lstrip("#"), 0)
                    except ValueError:
                        target = -1
                    target_name = address_names.get(target)
                    if target_name:
                        annotation = f"  ; {target_name}"
                elif instruction.mnemonic == "adrp":
                    match = re.fullmatch(r"(x\d+), #?(0x[0-9a-f]+)", instruction.op_str)
                    if match:
                        page_registers[match.group(1)] = int(match.group(2), 16)
                elif instruction.mnemonic == "add":
                    match = re.fullmatch(
                        r"(x\d+), (x\d+), #?(0x[0-9a-f]+)", instruction.op_str
                    )
                    if match and match.group(2) in page_registers:
                        target = page_registers[match.group(2)] + int(
                            match.group(3), 16
                        )
                        page_registers[match.group(1)] = target
                        text = cstring_at(elf, target)
                        if text is not None:
                            annotation = f"  ; {text!r}"
                print(
                    f"0x{instruction.address:08x}: "
                    f"{instruction.mnemonic:<8} {instruction.op_str}{annotation}"
                )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
