#pragma once

// Pin definitions for Minka-Aire Fan Controller
// Derived from KiCad schematic: Minka-Aire.kicad_sch
// ESP32 Lolin D1 Mini style board

// --- Shared SPI Bus ---
#define PIN_SPI_SCK   18
#define PIN_SPI_MISO  19
#define PIN_SPI_MOSI  23

// --- ST7735 LCD (J1) ---
#define PIN_TFT_CS    5
#define PIN_TFT_DC    25
#define PIN_TFT_RST   26
// BL (backlight) is tied to 3V3 via resistor on PCB — always on
// Display is 1.44" 128x128 ST7735

// --- CC1101 Radio (J2) ---
#define PIN_CC1101_CS   27
#define PIN_CC1101_GDO0 16
#define PIN_CC1101_GDO2 17

// --- MXC6655XA Accelerometer/Temp (U2) — I2C ---
#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define MXC6655_ADDR  0x15  // Default I2C address

// --- Buttons (resistor ladder on ADC) ---
// 3V3 → R3(1k) → node_A → R4(1k) → SW3 → R5(1k) → SW2 → R6(1k) → SW1
// node_A connects to GPIO 36 (ADC1_CH0)
// Pressing a button shorts that node to GND, creating distinct voltage levels:
//   No button:  ~3.3V  (ADC ~4095)
//   SW1 (btn1): ~2.48V (ADC ~3072)  — 3R below sense point
//   SW2 (btn2): ~2.20V (ADC ~2730)  — 2R below sense point
//   SW3 (btn3): ~1.65V (ADC ~2048)  — 1R below sense point
#define PIN_BTN_ADC   36

// --- Rotary encoder A/B on dedicated digital inputs ---
// (encoder push button + back button still go through the ADC ladder on GPIO 36)
#define PIN_ENC_A     32
#define PIN_ENC_B     33
