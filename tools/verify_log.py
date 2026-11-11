#!/usr/bin/env python3
"""Verify the hash chain of a Shipment Integrity Monitor log, offline.

Re-implements the STM32F411 hardware CRC unit (CRC-32, poly 0x04C11DB7,
init 0xFFFFFFFF, 32-bit words MSB-first, no reflection, no final XOR),
seeded with LOG_KEY at each SEALED line and never reset between lines.

Usage:  python tools/verify_log.py samples/sample_log.txt [--key 0xA5C3E1B7]
Exit code 0 = chain intact, 1 = edited or missing lines found.
"""
import argparse
import re
import sys

POLY = 0x04C11DB7


def crc_word(crc: int, word: int) -> int:
    crc ^= word
    for _ in range(32):
        crc = ((crc << 1) ^ POLY) & 0xFFFFFFFF if crc & 0x80000000 else (crc << 1) & 0xFFFFFFFF
    return crc


def crc_bytes(crc: int, data: bytes) -> int:
    data += b"\0" * (-len(data) % 4)            # last word zero-padded, as in Crc_Bytes()
    for i in range(0, len(data), 4):
        crc = crc_word(crc, int.from_bytes(data[i:i + 4], "little"))
    return crc


def verify(lines, key):
    crc, expected_seq = None, 0
    checked = bad = missing = 0
    first_bad = None
    for n, line in enumerate(lines, 1):
        line = line.rstrip("\r\n")
        if "EVENT=SEALED" in line:               # a new shipment restarts the chain
            crc, expected_seq = crc_word(0xFFFFFFFF, key), 0
        if crc is None or ",CHK=" not in line:
            continue
        text, chk = line.rsplit(",CHK=", 1)
        seq = int(re.search(r",SEQ=(\d+)$", text).group(1))
        if seq != expected_seq:
            missing += seq - expected_seq
        expected_seq = seq + 1
        crc = crc_bytes(crc, text.encode("ascii"))
        checked += 1
        if crc != int(chk, 16):
            bad += 1
            first_bad = first_bad or (n, seq, line)
    return checked, missing, bad, first_bad


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--key", default="0xA5C3E1B7", help="LOG_KEY from firmware/main.c")
    args = ap.parse_args()

    with open(args.log, encoding="ascii", errors="replace") as f:
        checked, missing, bad, first_bad = verify(f, int(args.key, 16))

    print(f"Lines checked : {checked}")
    print(f"Missing (SEQ) : {missing}")
    print(f"Mismatched    : {bad}")
    if first_bad:
        n, seq, line = first_bad
        print(f"First bad line: #{n} (SEQ {seq})\n  {line[:120]}")
    ok = checked > 0 and bad == 0 and missing == 0
    print("RESULT        :", "INTACT ✅" if ok else "ALTERED ❌")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
