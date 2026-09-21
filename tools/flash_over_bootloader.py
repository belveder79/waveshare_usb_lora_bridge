#!/usr/bin/env python3
"""Flash a firmware .bin over USB serial into a device running Archie3d's
custom Waveshare USB-LoRa bootloader:
  https://github.com/Archie3d/waveshare-usb-lora-bootloader

Protocol (USART1, 115200 8N1, no flow control):
  Host sends, per 1024-byte page, starting at the app's flash address
  (0x08004000):
    [X_START][CRC16_LSB][CRC16_MSB][1024 bytes of data]
  Device replies:
    X_ACK   (0x30) - page written, advance to the next page
    X_NCRC  (0x50) + [CRC16_LSB][CRC16_MSB] - CRC mismatch, resend the page
    X_NACK  (0x40) - flash write failed
  After the last page, host sends X_END (0x20); device replies X_ACK once it
  has validated and jumped to the new app, or X_NACK if the image looks
  invalid (in which case it resets its write pointer and waits for the
  image to be resent from the first page).

  In practice the X_ACK for X_END is frequently lost: the bootloader's
  shutdown_and_jump_to_app() disables the USART right after queuing that
  byte, which can cut off the still-in-flight transmission before the wire
  fully clocks it out. The jump itself still happens. So a timeout at this
  exact step is treated as "probably OK" here, not a hard failure -- verify
  with the flashed firmware's own protocol (e.g. host_example.py) afterward.

  CRC16 is a custom, non-standard variant (poly x^16 + x^2 + x + 1, computed
  MSB-first over the raw byte, NOT the typical CRC16/CCITT table algorithm)
  -- see crc16() below, ported directly from the bootloader's src/crc16.c.

To enter programming mode: hold the module's push button while powering it
up (both LEDs light solid to confirm bootloader/programming mode).

Usage:
    python3 tools/flash_over_bootloader.py <serial-port> <firmware.bin>
"""

import sys
import time

import serial

BAUD = 115200
CHUNK_SIZE = 1024

X_START = 0x10
X_END = 0x20
X_ACK = 0x30
X_NACK = 0x40
X_NCRC = 0x50


def crc16(crc: int, data: bytes) -> int:
    for b in data:
        a = ((crc >> 8) ^ b) & 0xFFFF
        crc = ((a << 2) ^ (a << 1) ^ a ^ (crc << 8)) & 0xFFFF
    return crc


class FlashError(RuntimeError):
    pass


def program_chunk(ser: serial.Serial, chunk: bytes, retries: int = 3) -> None:
    assert len(chunk) == CHUNK_SIZE
    crc = crc16(0, chunk)

    for attempt in range(retries):
        ser.write(bytes([X_START]))
        ser.write(crc.to_bytes(2, "little"))
        ser.write(chunk)

        resp = ser.read(1)
        if len(resp) != 1:
            raise FlashError("timed out waiting for page response")

        if resp[0] == X_ACK:
            return
        if resp[0] == X_NCRC:
            device_crc = ser.read(2)
            if len(device_crc) != 2:
                raise FlashError("timed out reading NCRC payload")
            device_crc = int.from_bytes(device_crc, "little")
            print(
                f"  CRC mismatch (host {crc:#06x}, device saw {device_crc:#06x}),"
                f" retrying ({attempt + 1}/{retries})"
            )
            continue
        if resp[0] == X_NACK:
            raise FlashError("device reported flash write failure")
        raise FlashError(f"unexpected response byte {resp[0]:#04x}")

    raise FlashError("too many CRC retries on this page")


def flash(port: str, firmware_path: str) -> None:
    with open(firmware_path, "rb") as f:
        data = f.read()

    if len(data) % CHUNK_SIZE != 0:
        pad = CHUNK_SIZE - (len(data) % CHUNK_SIZE)
        data += b"\x00" * pad

    total_chunks = len(data) // CHUNK_SIZE
    print(f"{firmware_path}: {len(data)} bytes ({total_chunks} pages of {CHUNK_SIZE})")

    ser = serial.Serial(port, BAUD, timeout=5.0)
    try:
        time.sleep(0.2)  # let the port settle
        for i in range(total_chunks):
            chunk = data[i * CHUNK_SIZE : (i + 1) * CHUNK_SIZE]
            program_chunk(ser, chunk)
            print(f"  [{i + 1}/{total_chunks}] page written", end="\r")
        print()

        ser.write(bytes([X_END]))
        resp = ser.read(1)
        if len(resp) != 1:
            # See the module docstring: this specific ACK is often lost to a
            # race in the bootloader's own shutdown-then-jump code, even
            # though the jump succeeded. Only a real X_NACK is fatal.
            print(
                "no response to X_END (likely lost in the bootloader's shutdown "
                "race) - probably still OK; verify the new firmware responds"
            )
        elif resp[0] != X_ACK:
            raise FlashError(
                "device rejected the completed image (X_NACK) - firmware may be "
                "invalid, or entry point/stack pointer at the app start address "
                "don't look sane to the bootloader"
            )
        else:
            print("OK - device validated the image and jumped to the new firmware")
    finally:
        ser.close()


def main() -> None:
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <serial-port> <firmware.bin>", file=sys.stderr)
        sys.exit(1)

    try:
        flash(sys.argv[1], sys.argv[2])
    except (FlashError, OSError, serial.SerialException) as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
