# Waveshare USB-LoRa Node — plain USB↔SPI bridge

Arduino firmware for the Waveshare USB-to-LoRa module that turns it into a
"dumb" USB↔SPI converter: the host drives the on-board SX1262 LoRa radio
directly over raw SPI transactions and GPIO reads/writes, framed over the
module's USB virtual COM port. The firmware contains **no LoRa protocol
knowledge** — it just relays bytes. All SX1262 command sequences, register
maps, and BUSY/DIO1 timing belong on the host side (e.g. a Python driver, or
a port of a library like RadioLib that has been adapted to talk this
protocol instead of a local SPI bus).

## Background

The [Archie3d/waveshare-usb-lora](https://github.com/Archie3d/waveshare-usb-lora)
repository is a hardware teardown of this module, not firmware — it documents
that the stock firmware is read-protected but the board can be reprogrammed
over SWD. This project is new firmware written from scratch for that same
hardware, using the pinout Archie3d reverse-engineered.

## Hardware

| Part | Function |
|---|---|
| GD32F103C8T6 | MCU (STM32F103C8 register/pin-compatible clone) |
| SX1262 | LoRa transceiver |
| CH343 | USB↔UART bridge, wired to the MCU's USART1 |

Because USB↔serial conversion is already done in hardware by the CH343
chip, the MCU firmware only needs to talk UART (via `Serial1`) on one side
and SPI on the other — it never touches the STM32's USB peripheral.

### Pin map

| MCU pin | Function | Direction |
|---|---|---|
| PA9 / PA10 | USART1 TX/RX → CH343 → USB | — |
| PA4 | SX1262 `NRESET` | output |
| PB4 | RF switch enable | output |
| PA6 | LED (RXD), active low | output |
| PA7 | LED (TXD), active low | output |
| PB1 | SX1262 `BUSY` | input |
| PB0 | SX1262 `DIO1` | input |
| PA5 | User button, active low | input (pull-up) |
| PB12 | SPI2 `NSS` — used as manual CS, not hardware NSS | output |
| PB13 | SPI2 `SCK` | — |
| PB14 | SPI2 `MISO` | — |
| PB15 | SPI2 `MOSI` | — |

Pinout per Archie3d's reverse-engineering notes; double check against your
own unit/silkscreen before flashing, since Waveshare ships several PCB
revisions (433/868 MHz, XTAL/TCXO).

## Building / flashing

There are two ways to get this firmware onto the device, depending on
what's already on it.

### Option A: SWD (virgin device, no bootloader)

Connect an SWD probe (e.g. ST-Link) to the module's SWD pads — the USB port
on the module itself is only a serial link to the CH343, not a USB
DFU/bootloader interface, so you cannot flash over the USB cable this way.

**PlatformIO (recommended)** — this repo is a PlatformIO project
(`platformio.ini`, sketch in `src/`), targeting board id `bluepill_f103c8`
on the `ststm32` platform (GD32F103C8T6 is commonly flashed successfully
with this platform since it's register/pin compatible; if you hit issues,
look for a dedicated GD32F1-specific core instead):

```
pio run                 # build (env:bluepill_f103c8)
pio run -t upload       # build + flash over SWD (ST-Link probe required)
pio device monitor       # open the CH343 serial port at 921600 baud
```

Use the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode)
for VSCode, or the `pio` CLI directly.

**Arduino IDE (alternative):**

1. Install the STM32duino core ("STM32 MCU based boards" by STMicroelectronics)
   in the Arduino IDE Boards Manager.
2. Board: **Generic STM32F1 series** → Board part number **BluePill F103C8**.
3. Upload method: **STLink**.
4. Open `src/waveshare_usb_lora_bridge.ino` and upload.

Both build the same firmware, linked to run from `0x08000000`.

### Option B: USB serial (device already has Archie3d's custom bootloader)

If the device has [Archie3d's custom bootloader](https://github.com/Archie3d/waveshare-usb-lora-bootloader)
already installed at `0x08000000`–`0x08004000`, you can push new firmware
over the CH343's USB-serial link instead of using SWD:

1. Build the offset variant — this env links the firmware to run from
   `0x08004000` (the bootloader's app slot) instead of `0x08000000`:
   ```
   pio run -e bluepill_f103c8_bootloader
   ```
2. Power up the module while holding its push button to enter programming
   mode (both LEDs light solid to confirm).
3. Flash it with the included protocol client (implements the bootloader's
   page-based, CRC16-checked serial protocol at 115200 baud — see the file
   for details, and [Archie3d's own Go-based `wsprog` tool](https://github.com/Archie3d/waveshare-usb-lora-programmer)
   for a reference implementation):
   ```
   .venv/bin/python tools/flash_over_bootloader.py <serial-port> \
       .pio/build/bluepill_f103c8_bootloader/firmware.bin
   ```
   On success the bootloader validates the image and jumps straight to it.
   A "no response to X_END" message from the script is expected and can be
   ignored — the bootloader's shutdown-then-jump code has a race that
   frequently drops that specific ACK byte, even though the jump succeeds;
   only an explicit rejection (X_NACK) means the flash actually failed.

Do **not** flash the `bluepill_f103c8_bootloader` build over SWD, or the
plain `bluepill_f103c8` build over serial — the two are linked for
different base addresses and are not interchangeable.

**Why `setup()` starts with `__enable_irq()`:** the bootloader executes
`cpsid i` (disable all interrupts) immediately before jumping to the app,
and never re-enables them. Without explicitly re-enabling interrupts, the
jumped-to firmware silently hangs the first time it touches anything
interrupt-driven — `delay()`/`millis()` (SysTick) or `Serial1`'s RX buffer
(USART interrupt) — with no crash and no error, just total silence. This
call is a no-op on a normal SWD-flashed cold boot, where interrupts are
already enabled by the hardware reset itself.

## Wire protocol

All communication happens over the CH343 virtual COM port at **921600
baud, 8N1**.

### Frame format

Host → device:

```
[0xAA][CMD][LEN_LO][LEN_HI][PAYLOAD × LEN][CRC8]
```

Device → host:

```
[0xAA][STATUS][LEN_LO][LEN_HI][PAYLOAD × LEN][CRC8]
```

- `LEN` is little-endian, payload capped at 256 bytes.
- `CRC8` is Dallas/Maxim (poly `0x31`, init `0x00`), computed over
  `CMD/STATUS + LEN_LO + LEN_HI + PAYLOAD`.
- A CRC mismatch gets a `STATUS_ERR_CRC` reply with no payload; the device
  then resyncs on the next `0xAA`.

### Commands

| CMD | Name | Payload (host→device) | Response payload | Notes |
|---|---|---|---|---|
| `0x00` | `PING` | — | `"WSLB"` + fw version byte | liveness / version check |
| `0x01` | `SPI_XFER` | N bytes to clock out | N bytes read back | asserts CS low for the duration of the transfer, then deasserts |
| `0x02` | `GPIO_WRITE` | `[pin_id][value]` | — | pin_id: see table below |
| `0x03` | `GPIO_READ` | `[pin_id]` | `[value]` | pin_id: see table below |
| `0x04` | `SET_CS` | `[value]` | — | manual CS control, for holding CS across multiple `SPI_XFER` calls |

### GPIO pin IDs

| ID | Pin | Direction |
|---|---|---|
| `0` | NRESET | write |
| `1` | RF switch | write |
| `2` | LED RX | write |
| `3` | LED TX | write |
| `4` | BUSY | read |
| `5` | DIO1 | read |
| `6` | Button | read |

### Status codes

| Code | Meaning |
|---|---|
| `0x00` | OK |
| `0x01` | CRC error |
| `0x02` | Unknown command |
| `0x03` | Bad payload length |
| `0x04` | Unknown GPIO pin id |

### Typical host-side sequence

1. `GPIO_WRITE(NRESET, 0)`, wait a few ms, `GPIO_WRITE(NRESET, 1)` — reset the SX1262.
2. Poll `GPIO_READ(BUSY)` until it reads 0.
3. `SPI_XFER(...)` with the SX1262 command bytes (e.g. `SetStandby`, `SetPacketType`, etc. per the SX1262 datasheet).
4. Poll `GPIO_READ(BUSY)` after each command that triggers radio activity.
5. Poll or wait on `GPIO_READ(DIO1)` for RX/TX-done interrupts (this firmware doesn't wire DIO1 to a host-visible interrupt — it's simple polling only).

See `host_example.py` for a minimal working client.

## Files

- `platformio.ini` — PlatformIO project config (board, upload/monitor settings).
- `src/waveshare_usb_lora_bridge.ino` — the firmware.
- `host_example.py` — a small Python client demonstrating the protocol
  (ping the device, toggle reset, and read the SX1262's status byte).
- `tools/flash_over_bootloader.py` — flashes this firmware over USB serial
  into a device running Archie3d's custom bootloader (see "Building /
  flashing" above), instead of over SWD.
