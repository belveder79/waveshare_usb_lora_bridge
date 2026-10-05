#!/usr/bin/env python3
"""Diagnose "first open works, reopen doesn't" on the bridge's serial link.

Opens the port several times in a row under different strategies and PINGs
the firmware each time, printing the firmware's uptime and boot reset flags
(fw v2+). That separates the candidate causes:

  plain      open at 921600 with pyserial defaults (DTR/RTS asserted)
  toggle     set 115200 then 921600 before use -- forces the kernel to send
             a fresh SET_LINE_CODING to the CH343 (Linux cdc_acm only sends
             it when the termios speed actually changes)
  nodtr      open with DTR/RTS held deasserted, in case the CH343's modem
             lines are wired to the MCU's NRST/BOOT0
  nodtr+tog  both

Reading the output:
  - only the "toggle" variants work         -> CH343 loses its baud rate
                                               between opens; re-send line
                                               coding on every open
  - only the "nodtr" variants work          -> DTR/RTS reach the MCU
  - works with small uptime after each open -> opening the port resets the MCU

Before blaming the hardware, check that nothing else has the port open
(`sudo fuser -v /dev/ttyACM0`): termios is per tty, so any other process that
opens it -- e.g. a Meshtastic Python client auto-probing serial ports at
115200 -- silently changes the baud rate for everyone, and the bridge then
looks dead.

Usage:
    python3 tools/link_probe.py <serial-port> [rounds]
"""

import sys
import time

import serial

SOF = 0xAA


def crc8_maxim(data: bytes) -> int:
    crc = 0
    for b in data:
        for _ in range(8):
            mix = (crc ^ b) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            b >>= 1
    return crc


def open_port(port: str, toggle: bool, nodtr: bool) -> serial.Serial:
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200 if toggle else 921600
    s.timeout = 1.0
    if nodtr:
        s.dtr = False
        s.rts = False
    s.open()
    if toggle:
        s.baudrate = 921600
    time.sleep(0.05)
    s.reset_input_buffer()
    return s


def ping(s: serial.Serial) -> str:
    header = bytes([0x00, 0, 0])
    s.write(bytes([SOF]) + header + bytes([crc8_maxim(header)]))
    got = s.read(4)
    if len(got) < 4:
        return f"FAIL: got {len(got)}/4 header bytes ({got.hex() or 'nothing'})"
    if got[0] != SOF:
        return f"FAIL: bad SOF, got {got.hex()} + {s.read(16).hex()}"
    length = got[2] | (got[3] << 8)
    payload = s.read(length)
    crc = s.read(1)
    if len(payload) != length or len(crc) != 1:
        return f"FAIL: truncated frame {(got + payload + crc).hex()}"
    if crc8_maxim(got[1:] + payload) != crc[0]:
        return f"FAIL: CRC mismatch {(got + payload + crc).hex()}"
    if len(payload) >= 8:
        up = payload[6] | (payload[7] << 8)
        return f"ok  fw v{payload[4]}  reset flags 0x{payload[5]:02x}  uptime {up}s"
    return f"ok  {payload!r} (fw v1, no uptime)"


def main():
    if len(sys.argv) not in (2, 3):
        print(__doc__, file=sys.stderr)
        sys.exit(1)
    port = sys.argv[1]
    rounds = int(sys.argv[2]) if len(sys.argv) == 3 else 3

    for name, toggle, nodtr in (
        ("plain", False, False),
        ("toggle", True, False),
        ("nodtr", False, True),
        ("nodtr+tog", True, True),
    ):
        for i in range(rounds):
            try:
                s = open_port(port, toggle, nodtr)
                try:
                    result = ping(s)
                finally:
                    s.close()
            except serial.SerialException as e:
                result = f"FAIL: {e}"
            print(f"{name:10s} #{i + 1}: {result}")
            time.sleep(0.3)


if __name__ == "__main__":
    main()
