// Waveshare USB-LoRa Node — plain USB<->SPI bridge firmware
//
// Target hardware: Waveshare USB-to-LoRa module
//   MCU:   GD32F103C8T6 (STM32F103C8 register/pin-compatible clone)
//   Radio: Semtech SX1262
//   USB:   CH343 USB<->UART bridge, wired to MCU USART1 (PA9=TX, PA10=RX)
//
// This firmware does NOT understand LoRa. It is a "dumb" bridge: the host
// sends framed commands over the CH343 virtual COM port, and the MCU
// executes them as raw SPI transfers or GPIO reads/writes against the
// SX1262. All radio protocol logic (register maps, opcodes, BUSY/DIO1
// timing) lives on the host side. See README.md for the wire protocol
// and pin map.
//
// Pin map (from hardware teardown, see README.md):
//   PA4  SX1262 NRESET   (output)
//   PB4  RF switch       (output)
//   PA6  LED RXD         (output, active low)
//   PA7  LED TXD         (output, active low)
//   PB1  SX1262 BUSY     (input)
//   PB0  SX1262 DIO1     (input)
//   PA5  User button     (input, active low, needs pull-up)
//   SPI2: PB12=NSS(manual CS) PB13=SCK PB14=MISO PB15=MOSI
//   USART1: PA9=TX PA10=RX -> CH343 -> USB (this is Serial1 on this core)

#include <Arduino.h>
#include <SPI.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

static const uint32_t HOST_BAUD = 921600;
static const uint32_t SPI_CLOCK_HZ = 8000000; // SX1262 supports up to ~18MHz; 8MHz is a safe default

// SPI2 pins (MOSI, MISO, SCK) — CS is handled manually, not via hardware NSS.
SPIClass SPI_2(PB15, PB14, PB13);

static const uint8_t PIN_NRESET = PA4;
static const uint8_t PIN_RFSWITCH = PB4;
static const uint8_t PIN_LED_RX = PA6;
static const uint8_t PIN_LED_TX = PA7;
static const uint8_t PIN_BUSY = PB1;
static const uint8_t PIN_DIO1 = PB0;
static const uint8_t PIN_BUTTON = PA5;
static const uint8_t PIN_CS = PB12;

// ---------------------------------------------------------------------------
// Wire protocol
//
// Host -> device frame:
//   [0xAA][CMD][LEN_LO][LEN_HI][PAYLOAD x LEN][CRC8]
// Device -> host frame:
//   [0xAA][STATUS][LEN_LO][LEN_HI][PAYLOAD x LEN][CRC8]
//
// CRC8 is Dallas/Maxim (poly 0x31, init 0x00) computed over
// CMD/STATUS + LEN_LO + LEN_HI + PAYLOAD.
// ---------------------------------------------------------------------------

static const uint8_t SOF = 0xAA;
static const size_t MAX_PAYLOAD = 256;

enum Command : uint8_t {
  CMD_PING = 0x00,
  CMD_SPI_XFER = 0x01,
  CMD_GPIO_WRITE = 0x02,
  CMD_GPIO_READ = 0x03,
  CMD_SET_CS = 0x04,
};

enum GpioId : uint8_t {
  GPIO_NRESET = 0,
  GPIO_RFSWITCH = 1,
  GPIO_LED_RX = 2,
  GPIO_LED_TX = 3,
  GPIO_BUSY = 4,
  GPIO_DIO1 = 5,
  GPIO_BUTTON = 6,
};

enum Status : uint8_t {
  STATUS_OK = 0x00,
  STATUS_ERR_CRC = 0x01,
  STATUS_ERR_CMD = 0x02,
  STATUS_ERR_LEN = 0x03,
  STATUS_ERR_PIN = 0x04,
};

static const uint8_t FW_VERSION = 1;

// ---------------------------------------------------------------------------
// CRC8/MAXIM
// ---------------------------------------------------------------------------

static uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      uint8_t mix = (crc ^ b) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      b >>= 1;
    }
  }
  return crc;
}

// ---------------------------------------------------------------------------
// Frame receive state machine
// ---------------------------------------------------------------------------

enum ParseState {
  WAIT_SOF,
  WAIT_CMD,
  WAIT_LEN_LO,
  WAIT_LEN_HI,
  WAIT_PAYLOAD,
  WAIT_CRC,
};

static ParseState parseState = WAIT_SOF;
static uint8_t rxCmd = 0;
static uint16_t rxLen = 0;
static uint16_t rxIndex = 0;
static uint8_t rxPayload[MAX_PAYLOAD];

static uint8_t txPayload[MAX_PAYLOAD];

static void sendResponse(uint8_t status, const uint8_t *payload, uint16_t len) {
  uint8_t header[3] = {status, (uint8_t)(len & 0xFF), (uint8_t)((len >> 8) & 0xFF)};
  uint8_t crcBuf[3 + MAX_PAYLOAD];
  memcpy(crcBuf, header, 3);
  if (len > 0) memcpy(crcBuf + 3, payload, len);
  uint8_t crc = crc8(crcBuf, 3 + len);

  Serial1.write(SOF);
  Serial1.write(header, 3);
  if (len > 0) Serial1.write(payload, len);
  Serial1.write(crc);
}

static void handleFrame(uint8_t cmd, const uint8_t *payload, uint16_t len) {
  switch (cmd) {
    case CMD_PING: {
      uint8_t resp[6] = {'W', 'S', 'L', 'B', FW_VERSION, 0};
      sendResponse(STATUS_OK, resp, 5);
      break;
    }

    case CMD_SPI_XFER: {
      if (len > MAX_PAYLOAD) {
        sendResponse(STATUS_ERR_LEN, nullptr, 0);
        break;
      }
      memcpy(txPayload, payload, len);
      digitalWrite(PIN_CS, LOW);
      for (uint16_t i = 0; i < len; ++i) {
        txPayload[i] = SPI_2.transfer(txPayload[i]);
      }
      digitalWrite(PIN_CS, HIGH);
      sendResponse(STATUS_OK, txPayload, len);
      break;
    }

    case CMD_SET_CS: {
      if (len != 1) {
        sendResponse(STATUS_ERR_LEN, nullptr, 0);
        break;
      }
      digitalWrite(PIN_CS, payload[0] ? HIGH : LOW);
      sendResponse(STATUS_OK, nullptr, 0);
      break;
    }

    case CMD_GPIO_WRITE: {
      if (len != 2) {
        sendResponse(STATUS_ERR_LEN, nullptr, 0);
        break;
      }
      uint8_t value = payload[1] ? HIGH : LOW;
      switch (payload[0]) {
        case GPIO_NRESET:   digitalWrite(PIN_NRESET, value); break;
        case GPIO_RFSWITCH: digitalWrite(PIN_RFSWITCH, value); break;
        case GPIO_LED_RX:   digitalWrite(PIN_LED_RX, value); break;
        case GPIO_LED_TX:   digitalWrite(PIN_LED_TX, value); break;
        default:
          sendResponse(STATUS_ERR_PIN, nullptr, 0);
          return;
      }
      sendResponse(STATUS_OK, nullptr, 0);
      break;
    }

    case CMD_GPIO_READ: {
      if (len != 1) {
        sendResponse(STATUS_ERR_LEN, nullptr, 0);
        break;
      }
      uint8_t value;
      switch (payload[0]) {
        case GPIO_BUSY:   value = digitalRead(PIN_BUSY); break;
        case GPIO_DIO1:   value = digitalRead(PIN_DIO1); break;
        case GPIO_BUTTON: value = digitalRead(PIN_BUTTON); break;
        default:
          sendResponse(STATUS_ERR_PIN, nullptr, 0);
          return;
      }
      sendResponse(STATUS_OK, &value, 1);
      break;
    }

    default:
      sendResponse(STATUS_ERR_CMD, nullptr, 0);
      break;
  }
}

static void resetParser() {
  parseState = WAIT_SOF;
  rxIndex = 0;
}

static void pollSerial() {
  while (Serial1.available() > 0) {
    uint8_t b = (uint8_t)Serial1.read();

    switch (parseState) {
      case WAIT_SOF:
        if (b == SOF) parseState = WAIT_CMD;
        break;

      case WAIT_CMD:
        rxCmd = b;
        parseState = WAIT_LEN_LO;
        break;

      case WAIT_LEN_LO:
        rxLen = b;
        parseState = WAIT_LEN_HI;
        break;

      case WAIT_LEN_HI:
        rxLen |= ((uint16_t)b) << 8;
        if (rxLen > MAX_PAYLOAD) {
          // Can't fit: bail out and resync on next SOF.
          resetParser();
          break;
        }
        rxIndex = 0;
        parseState = (rxLen == 0) ? WAIT_CRC : WAIT_PAYLOAD;
        break;

      case WAIT_PAYLOAD:
        rxPayload[rxIndex++] = b;
        if (rxIndex >= rxLen) parseState = WAIT_CRC;
        break;

      case WAIT_CRC: {
        uint8_t crcBuf[3 + MAX_PAYLOAD];
        crcBuf[0] = rxCmd;
        crcBuf[1] = (uint8_t)(rxLen & 0xFF);
        crcBuf[2] = (uint8_t)((rxLen >> 8) & 0xFF);
        memcpy(crcBuf + 3, rxPayload, rxLen);
        uint8_t expected = crc8(crcBuf, 3 + rxLen);

        if (b == expected) {
          handleFrame(rxCmd, rxPayload, rxLen);
        } else {
          sendResponse(STATUS_ERR_CRC, nullptr, 0);
        }
        resetParser();
        break;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  // A device flashed via Archie3d's custom USB-serial bootloader
  // (see README.md) arrives here with global interrupts disabled: that
  // bootloader executes `cpsid i` right before jumping to the app and never
  // re-enables them. Without this, SysTick (millis()/delay()) and the
  // UART RX interrupt used by Serial1 never fire, and everything below
  // silently hangs. This is a no-op on a normal SWD/cold-boot start, where
  // interrupts are already enabled.
  __enable_irq();

  pinMode(PIN_NRESET, OUTPUT);
  digitalWrite(PIN_NRESET, HIGH);

  pinMode(PIN_RFSWITCH, OUTPUT);
  digitalWrite(PIN_RFSWITCH, LOW);

  pinMode(PIN_LED_RX, OUTPUT);
  digitalWrite(PIN_LED_RX, HIGH); // active low -> off

  pinMode(PIN_LED_TX, OUTPUT);
  digitalWrite(PIN_LED_TX, HIGH); // active low -> off

  pinMode(PIN_BUSY, INPUT);
  pinMode(PIN_DIO1, INPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  SPI_2.begin();
  SPI_2.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));

  Serial1.begin(HOST_BAUD);
}

void loop() {
  pollSerial();
}
