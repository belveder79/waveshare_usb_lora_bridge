#!/usr/bin/env python3
"""Minimal host-side client for the Waveshare USB-LoRa plain SPI bridge.

Demonstrates the framed protocol implemented by waveshare_usb_lora_bridge.ino:
ping the firmware, pulse the SX1262 reset line, and read back the SX1262's
status byte via a raw SPI transfer.

Usage:
    python3 host_example.py /dev/ttyUSB0
"""

import sys
import time

import serial

SOF = 0xAA

CMD_PING = 0x00
CMD_SPI_XFER = 0x01
CMD_GPIO_WRITE = 0x02
CMD_GPIO_READ = 0x03
CMD_SET_CS = 0x04

GPIO_NRESET = 0
GPIO_RFSWITCH = 1
GPIO_LED_RX = 2
GPIO_LED_TX = 3
GPIO_BUSY = 4
GPIO_DIO1 = 5
GPIO_BUTTON = 6

STATUS_NAMES = {0: "OK", 1: "ERR_CRC", 2: "ERR_CMD", 3: "ERR_LEN", 4: "ERR_PIN"}


def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        b = byte
        for _ in range(8):
            mix = (crc ^ b) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            b >>= 1
    return crc & 0xFF


class BridgeError(RuntimeError):
    pass


class Bridge:
    def __init__(self, port: str, baud: int = 921600, timeout: float = 1.0):
        self._ser = serial.Serial(port, baud, timeout=timeout)

    def close(self):
        self._ser.close()

    def _send(self, cmd: int, payload: bytes = b""):
        header = bytes([cmd, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
        crc = crc8_maxim(header + payload)
        frame = bytes([SOF]) + header + payload + bytes([crc])
        self._ser.write(frame)

    def _recv(self):
        if self._ser.read(1) != bytes([SOF]):
            raise BridgeError("timed out waiting for start-of-frame")
        status = self._ser.read(1)[0]
        len_lo, len_hi = self._ser.read(2)
        length = len_lo | (len_hi << 8)
        payload = self._ser.read(length)
        crc = self._ser.read(1)[0]

        expected = crc8_maxim(bytes([status, len_lo, len_hi]) + payload)
        if crc != expected:
            raise BridgeError("CRC mismatch in response")
        if status != 0:
            raise BridgeError(f"device returned {STATUS_NAMES.get(status, status)}")
        return payload

    def ping(self) -> bytes:
        self._send(CMD_PING)
        return self._recv()

    def gpio_write(self, pin_id: int, value: int):
        self._send(CMD_GPIO_WRITE, bytes([pin_id, 1 if value else 0]))
        self._recv()

    def gpio_read(self, pin_id: int) -> int:
        self._send(CMD_GPIO_READ, bytes([pin_id]))
        return self._recv()[0]

    def spi_xfer(self, data: bytes) -> bytes:
        self._send(CMD_SPI_XFER, data)
        return self._recv()

    def reset_radio(self, pulse_ms: float = 5.0, settle_ms: float = 10.0):
        self.gpio_write(GPIO_NRESET, 0)
        time.sleep(pulse_ms / 1000.0)
        self.gpio_write(GPIO_NRESET, 1)
        time.sleep(settle_ms / 1000.0)


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <serial-port>", file=sys.stderr)
        sys.exit(1)

    bridge = Bridge(sys.argv[1])
    try:
        version = bridge.ping()
        print(f"ping ok: {version!r}")

        bridge.reset_radio()
        print("radio reset pulsed")

        for _ in range(100):
            if bridge.gpio_read(GPIO_BUSY) == 0:
                break
            time.sleep(0.001)
        else:
            print("warning: BUSY never went low after reset")

        # SX1262 GetStatus (opcode 0xC0): send opcode + NOP, second returned
        # byte is the status register.
        resp = bridge.spi_xfer(bytes([0xC0, 0x00]))
        print(f"SX1262 GetStatus raw reply: {resp.hex()}")
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
