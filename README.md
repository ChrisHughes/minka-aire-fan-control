# Minka-Aire Fan Controller

An ESP32-based IoT controller for **Minka Aire** ceiling fans (and compatible
300–348 MHz OOK-coded ceiling fan remotes). Reads ambient room temperature from
an on-board sensor and automatically adjusts fan speed via RF commands to keep
the room at your desired comfort band. Supports multiple fans with different
DIP-switch addresses, manual override with timed auto-resume, and a rotary
encoder UI with colour LCD.

---

## Aim

Commercial smart-fan retrofits are expensive, locked to vendor apps, or require
cloud accounts. This project replaces the included IR/RF remote with a small
desk-top controller that:

- Reads the room's **ambient temperature** via an I²C accelerometer/thermometer
- Transmits the same **303.875 MHz ASK/OOK** RF codes as the original Minka
  Aire remote
- Automatically ramps the fan between OFF → LOW → MED → HIGH based on
  user-configurable temperature thresholds with hysteresis
- Controls **multiple fans** in the same room (each with its own DIP switch
  address) from one device
- Offers **manual override** with pause-for-X-hours auto-resume
- Is fully self-contained — no WiFi, no cloud, no app

Hardware BOM is under $15 in single-unit quantities.

---

## Hardware

### Electronics

| Component | Role |
|---|---|
| **ESP32 Lolin D1 Mini** (or similar) | Microcontroller |
| **CC1101 sub-GHz radio module** (433 MHz variant, modified — see below) | RF TX at ~304 MHz |
| **ST7735 1.44" 128×128 color LCD** | Display (SPI) |
| **MXC6655XA** | 3-axis accel + temperature sensor (I²C) |
| **Rotary encoder** (with push switch) | Scroll / confirm input |
| Momentary push button | Back / cancel input |
| Passive resistor ladder on GPIO 36 | Reads encoder-push + back button via ADC |

All schematics and PCB artwork are in the `Minka-Aire/` directory (KiCad 9
project). A breadboarded prototype is functional; PCB fabrication is optional.

### CC1101 frequency modification

Common CC1101 breakout boards sold online are tuned for the 433 MHz ISM band.
The CC1101 silicon itself covers 300–348 MHz, but the external matching network
(balun + LC filter) attenuates signals outside the tuned band. To transmit
effectively at 303.875 MHz we **swapped the two balun capacitors** (C121 and
C131 on the reference design) from **3.9 pF to 8.2 pF**. See the comments in
`firmware/include/cc1101.h` for the full register configuration and the
discussion in the project history for the rationale.

There is also an undocumented-but-well-known **TEST0 register quirk**: writing
`TEST0 = 0x0B` (instead of the usual `0x09`) is required for the PLL to lock
below 348 MHz. Without it, the VCO bottoms out around ~310 MHz even though the
datasheet claims the full range.

### Pin mapping

See `firmware/include/pins.h` for authoritative values.

```
 ESP32                       Device
 ─────┬────────────────      ──────
  18  │ SPI SCK          →   LCD SCK   / CC1101 SCK  (shared)
  19  │ SPI MISO         →   CC1101 MISO            (shared)
  23  │ SPI MOSI         →   LCD MOSI  / CC1101 MOSI (shared)
   5  │ LCD CS
  25  │ LCD DC
  26  │ LCD RESET
  27  │ CC1101 CS
  16  │ CC1101 GDO0
  17  │ CC1101 GDO2
  21  │ I²C SDA          →   MXC6655XA SDA
  22  │ I²C SCL          →   MXC6655XA SCL
  32  │ Rotary encoder A (quadrature, interrupt-driven)
  33  │ Rotary encoder B
  36  │ ADC button ladder: encoder push + back button
```

---

## RF Protocol

Derived from SDR captures and cross-referenced with the FCC filings for the
**DL-4111T-01** Minka Aire remote (FCC ID 2A767-DL-4111T, see
`1589752.pdf` / `6134768.pdf`).

- **Carrier:** 303.875 MHz, ASK/OOK modulation
- **Frame:** 13-bit PWM, transmitted 15 times per button press
- **Bit timing:**
  - `0` bit → short carrier pulse (~350 µs) + long gap (~675 µs)
  - `1` bit → long carrier pulse (~700 µs) + long gap (~675 µs)
  - A short gap (~330 µs) is used when a `0` bit is followed by a `1` bit
- **Address:** upper 9 bits of the code, determined by the receiver's internal
  DIP switches. Upper 5 bits are fixed (`00100` for all DL-4111T units); lower
  4 bits are the user-selectable DIP bank.
- **Commands** (lower 4 bits): OFF=`1010`, LOW=`0010`, MED=`0100`, HIGH=`1000`
- **Inter-frame gap:** ~12 ms

The firmware transmits the OOK pattern by packing chips into the CC1101's TX
FIFO and letting the chip clock them out at ~2857 baud via a tight bit pattern
— no async-serial GDO bit-banging required.

**Dual-fan support:** when multiple fans are learned, the firmware transmits
each fan's unique code in sequence with a 50 ms gap between them. The order is
deliberately reversed (last-learned first) to work around a receiver quirk
where one fan's receiver would mis-latch on another's trailing OFF signal when
the two addresses differed by a single bit.

---

## Firmware

Built with [PlatformIO](https://platformio.org/) targeting the `lolin_d32`
board (ESP32). Source lives in `firmware/`.

### Build & flash

```bash
cd firmware
pio run -t upload       # compile + flash
pio device monitor      # serial console @ 115200
```

### Library dependencies (auto-installed by PlatformIO)

- `bodmer/TFT_eSPI` — ST7735 LCD driver with sprite buffer and DMA

All other drivers (CC1101, MXC6655XA, button reader, NVS settings store) are
bespoke and live in `firmware/include/`.

### Code organization

```
firmware/
├── platformio.ini              PlatformIO env + TFT_eSPI build flags
├── include/
│   ├── pins.h                  GPIO pin assignments
│   ├── buttons.h               ADC-ladder button + interrupt quadrature encoder
│   ├── cc1101.h                CC1101 SPI driver, OOK config, TX helpers
│   ├── mxc6655.h               I²C driver for accelerometer + temp sensor
│   └── settings.h              NVS-backed settings struct + auto-control logic
└── src/
    └── main.cpp                Screen definitions, UI state machine, loop
```

### User interface

- **Rotary encoder** (interrupt-driven quadrature on GPIO 32/33)
  - Rotate: scroll menus / adjust values
  - Push: confirm / enter
  - Long push: context-specific (delete, confirm reset)
- **Back button** (on the ADC ladder)
  - Short: return to previous screen
  - Long: context-specific (pause-for menu from main)

Screens are composed through a flicker-free off-screen sprite buffer
(`TFT_eSprite`, 128×128 × 16bpp = 32 KB) which is pushed to the display
atomically via DMA. Every frame is a full redraw into the sprite, then one
transfer to the panel — no visible partial updates.

### Settings (persisted to NVS)

- Auto vs Manual mode, last-commanded fan speed
- Temperature unit (°C / °F)
- Auto thresholds: engage-LOW, engage-MED, engage-HIGH temperatures + hysteresis
- Up to 8 learned remote addresses with user-editable names

Factory reset wipes NVS and reboots.

---

## SDR decoding (protocol reverse-engineering)

The `decode_sdr*.py` scripts are the tools used to decode the Minka Aire remote
from IQ captures taken with a USB SDR (RTL-SDR v3, GQRX, or similar). They
read a stereo-WAV IQ file, demodulate the AM envelope, threshold into binary,
and parse out the 13-bit codes per button press.

To reproduce from scratch:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy scipy
# Drop your IQ capture WAV into the project root, edit the path in decode_sdr3.py
python3 decode_sdr3.py
```

Output includes per-button decoded bit patterns, the extracted address (DIP
switch bits), and raw pulse timing statistics.

---

## Status

Fully functional. Controls two ceiling fans in the project author's living
room. Auto-mode engages nightly as the room warms up. Occasional visual
artifact on the bottom 2–3 rows of the LCD depending on ST7735 panel variant —
tracked in the code comments; easy to fine-tune with alternative GREENTAB
driver flags.

## License

MIT — see `LICENSE` if included; otherwise treat as open-source, attribution
appreciated.
