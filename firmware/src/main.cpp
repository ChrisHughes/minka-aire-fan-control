#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TFT_eSPI.h>
// FreeFonts are provided by TFT_eSPI (Fonts/GFXFF/*). Reference font structs
// directly; LOAD_GFXFF must be defined in platformio.ini.
#include "pins.h"
#include "buttons.h"
#include "cc1101.h"
#include "mxc6655.h"
#include "settings.h"

// --- RF Configuration ---
// Fan operates nominally at 303.875 MHz, but receiver tolerates the full band.
// 304.158 MHz gives slightly higher CC1101 TX gain on this module.
#define RF_TARGET_MHZ       304.158
#define RF_OFFSET_INITIAL   +0.03    // CC1101 crystal fine-tune
#define RF_OFFSET_STEP      0.01     // MHz per L/R press
float rfCrystalOffset = RF_OFFSET_INITIAL;  // Runtime tunable
#define RF_FREQUENCY        (RF_TARGET_MHZ + rfCrystalOffset)

// OOK Pulse timing (from SDR capture analysis)
// Encoding: 13-bit PWM, short ON (~350us) = 0, long ON (~700us) = 1
// OFF gap varies: ~675us normally, ~330us when 0-bit precedes 1-bit
#define PULSE_SHORT_US  350   // "0" bit ON duration
#define PULSE_LONG_US   700   // "1" bit ON duration
#define GAP_LONG_US     675   // Normal OFF gap
#define GAP_SHORT_US    330   // Shortened OFF gap (0 before 1)
#define FRAME_GAP_US    12000 // Inter-frame gap
#define TX_REPEATS      15    // Transmissions per button press
#define CODE_BITS       13    // Bits per code word

// Delay between successive fan transmissions (receivers need disambiguation)
#define INTER_FAN_DELAY_MS  50

// Aliases for backward-compat with existing UI code (will be removed when we
// move to the new main screen). FanSpeed enum is defined in settings.h.
#define CMD_COUNT     SPEED_COUNT
#define CMD_NAMES     SPEED_NAMES

// --- Menu System ---
enum Screen : uint8_t {
    SCR_MAIN = 0,       // Boot: fan control home (temp, speed, mode badge)
    SCR_SPEED_PICKER,   // Submenu: select manual fan speed
    SCR_PAUSE_PICKER,   // Submenu: select pause duration
    SCR_SETTINGS,       // Hub: access all sub-features
    SCR_UNITS,          // C/F toggle
    SCR_THRESHOLDS,     // Set auto-mode temp thresholds
    SCR_REMOTES,        // List learned remotes + launch learn flow
    SCR_LEARN,          // Guided remote learning
    SCR_FACTORY_RESET,  // Confirm factory reset
    SCR_SENSOR,         // Temperature + accelerometer details
    SCR_STATUS,         // Learned fan codes
    SCR_RFDIAG,
    SCR_BTNCAL,
    SCR_COUNT
};

static const char* SCR_TITLES[SCR_COUNT] = {
    "MAIN", "SPEED", "PAUSE", "SETTINGS",
    "UNITS", "THRESHOLDS", "REMOTES", "LEARN", "RESET?",
    "SENSOR", "STATUS", "RF DIAG", "BTNCAL"
};

// Settings menu items
static const Screen SETTINGS_MENU[] = {
    SCR_UNITS, SCR_THRESHOLDS, SCR_REMOTES,
    SCR_SENSOR, SCR_STATUS, SCR_RFDIAG, SCR_BTNCAL, SCR_FACTORY_RESET
};
static const uint8_t SETTINGS_MENU_COUNT =
    sizeof(SETTINGS_MENU) / sizeof(SETTINGS_MENU[0]);
static const char* SETTINGS_MENU_LABELS[SETTINGS_MENU_COUNT] = {
    "Units (C/F)", "Thresholds", "Remotes / Learn",
    "Sensor / Temp", "Code Status", "RF Diagnostics", "Button Cal", "Factory Reset"
};
uint8_t settingsCursor = 0;

// --- Runtime state ---
// Pause-for: if non-zero, we're in manual mode until millis() reaches this value,
// then auto-mode resumes automatically.
unsigned long pauseUntilMs = 0;
// Pause menu selection cursor (index into PAUSE_DURATIONS)
uint8_t pauseCursor = 0;
static const unsigned long PAUSE_DURATIONS_MIN[] = {0, 30, 60, 120, 240, 480};
static const char* PAUSE_LABELS[] = {
    "Cancel pause", "30 minutes", "1 hour", "2 hours", "4 hours", "8 hours"
};
static const uint8_t PAUSE_COUNT = sizeof(PAUSE_DURATIONS_MIN) / sizeof(PAUSE_DURATIONS_MIN[0]);

// Threshold setter state
uint8_t thresholdCursor = 0;  // 0=LOW, 1=MED, 2=HIGH
// Remote list cursor
uint8_t remoteCursor = 0;

// Debounced fan-speed commit state (set by encoder rotation on main screen,
// actually transmitted 500ms after last change so fast spins don't flood RF).
FanSpeed pendingSpeed = SPEED_OFF;
unsigned long pendingCommitMs = 0;

// Remote-learn screen state.
// Address layout: upper 5 bits fixed at 0b00100, lower 4 bits = DIP switches.
// learnDip[i] = state of DIP switch i+1 (true = ON = 1)
// learnCursor: 0..3 = select DIP switch to toggle, 4 = TEST, 5 = SAVE
bool learnDip[4] = {false, false, false, true};  // default: 0001 (matches Fan 1)
uint8_t learnCursor = 0;
unsigned long learnTestTime = 0;
const char* learnStatus = "";

// --- Globals ---
TFT_eSPI _lcd = TFT_eSPI();            // Physical display (use for init/rotation only)
TFT_eSprite tft = TFT_eSprite(&_lcd);  // All draw code renders here (off-screen, flicker-free)
ButtonReader buttons;
CC1101Radio radio;
MXC6655 sensor;
SettingsManager settings;

// Forward declaration — defined further down with the rest of TX code
void radioSend(uint16_t code);

// Helper: send a fan speed command to all learned fans, with delay between.
// Sends in REVERSE order (last-learned first) to avoid the 1-bit-difference
// receiver mis-latching issue we discovered earlier.
void sendToAllFans(FanSpeed speed) {
    if (settings.data.numFans == 0) {
        Serial.println("[FAN] No remotes learned — cannot send.");
        return;
    }
    for (int i = settings.data.numFans - 1; i >= 0; i--) {
        uint16_t code = settings.fanCode(i, speed);
        Serial.printf("[FAN] -> %s (slot %d, code 0x%04X)\n",
                      SPEED_NAMES[speed], i, code);
        radioSend(code);
        if (i > 0) delay(INTER_FAN_DELAY_MS);
    }
    settings.data.manualSpeed = speed;
    settings.save();  // Persist last-known state for next boot
    extern FanSpeed currentFanSpeed;
    currentFanSpeed = speed;  // Update UI state
}

Screen currentScreen = SCR_MAIN;
uint8_t controlCursor = 0;  // Which command is selected (0 to CMD_COUNT-1)
unsigned long lastTxTime = 0;
unsigned long lastTxCode = 0;
bool screenDirty = true;

// Encode a fan command as OOK bit pattern in a byte buffer.
// Each bit in the buffer = one "chip" of ~350us.
// Carrier ON = 1, Carrier OFF = 0.
// Fan code bit 0: 1 chip ON, 2 chips OFF  (short pulse, long gap)
// Fan code bit 1: 2 chips ON, 2 chips OFF  (long pulse, long gap)
// Fan code bit 0 before 1: 1 chip ON, 1 chip OFF (short pulse, short gap)
// Returns number of bytes written.
int encodeOOK(uint16_t code, uint8_t* buf, int bufSize) {
    // Build chip sequence: each "chip" is ~350us at our data rate
    uint8_t chips[128];
    int ci = 0;

    for (int i = CODE_BITS - 1; i >= 0; i--) {
        bool bit = (code >> i) & 1;
        bool nextBit = (i > 0) ? ((code >> (i - 1)) & 1) : 0;

        if (bit) {
            // Long pulse: 2 chips ON
            chips[ci++] = 1;
            chips[ci++] = 1;
            // Long gap: 2 chips OFF
            chips[ci++] = 0;
            chips[ci++] = 0;
        } else {
            // Short pulse: 1 chip ON
            chips[ci++] = 1;
            if (!bit && nextBit) {
                // Short gap before a 1-bit: 1 chip OFF
                chips[ci++] = 0;
            } else {
                // Long gap: 2 chips OFF
                chips[ci++] = 0;
                chips[ci++] = 0;
            }
        }
    }

    // Add inter-frame gap: ~12ms / 350us = ~34 chips of OFF
    for (int g = 0; g < 34 && ci < 120; g++) {
        chips[ci++] = 0;
    }

    // Pack chips into bytes (MSB first)
    int byteCount = (ci + 7) / 8;
    if (byteCount > bufSize) byteCount = bufSize;
    memset(buf, 0, byteCount);
    for (int j = 0; j < ci && j/8 < bufSize; j++) {
        if (chips[j]) {
            buf[j / 8] |= (0x80 >> (j % 8));
        }
    }

    Serial.printf("[TX] Encoded %d chips (%d bytes) for 0x%04X\n", ci, byteCount, code);
    return byteCount;
}

void radioSend(uint16_t code) {
    Serial.printf("[TX] Sending 0x%04X x%d at %.3f MHz\n",
                  code, TX_REPEATS, (float)RF_FREQUENCY);

    // Encode the OOK waveform
    uint8_t frameBuf[64];
    int frameLen = encodeOOK(code, frameBuf, sizeof(frameBuf));

    // Take exclusive control of SPI bus
    digitalWrite(PIN_TFT_CS, HIGH);
    SPI.end();
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

    // Helper lambdas for fast SPI (no end/begin overhead)
    auto strobe = [](uint8_t cmd) {
        digitalWrite(PIN_CC1101_CS, LOW);
        while (digitalRead(PIN_SPI_MISO)) {}
        SPI.transfer(cmd);
        digitalWrite(PIN_CC1101_CS, HIGH);
    };

    auto writeReg = [](uint8_t addr, uint8_t val) {
        digitalWrite(PIN_CC1101_CS, LOW);
        while (digitalRead(PIN_SPI_MISO)) {}
        SPI.transfer(addr);
        SPI.transfer(val);
        digitalWrite(PIN_CC1101_CS, HIGH);
    };

    auto writeFifo = [](uint8_t* data, int len) {
        digitalWrite(PIN_CC1101_CS, LOW);
        while (digitalRead(PIN_SPI_MISO)) {}
        SPI.transfer(0x7F);  // TX FIFO burst write
        for (int i = 0; i < len; i++) SPI.transfer(data[i]);
        digitalWrite(PIN_CC1101_CS, HIGH);
    };

    // Read FREQ registers BEFORE we touch anything
    auto readReg = [](uint8_t addr) -> uint8_t {
        digitalWrite(PIN_CC1101_CS, LOW);
        while (digitalRead(PIN_SPI_MISO)) {}
        SPI.transfer(addr | 0x80);
        uint8_t v = SPI.transfer(0);
        digitalWrite(PIN_CC1101_CS, HIGH);
        return v;
    };

    uint8_t f2 = readReg(0x0D), f1 = readReg(0x0E), f0 = readReg(0x0F);
    Serial.printf("[TX] FREQ before TX: %02X %02X %02X\n", f2, f1, f0);

    strobe(0x36);  // SIDLE
    delay(1);

    // Re-write FREQ registers (TFT SPI may have corrupted them)
    uint32_t fw = (uint32_t)((float)RF_FREQUENCY * 65536.0f / 26.0f);
    writeReg(0x0D, (fw >> 16) & 0xFF);  // FREQ2
    writeReg(0x0E, (fw >> 8) & 0xFF);   // FREQ1
    writeReg(0x0F, fw & 0xFF);          // FREQ0

    f2 = readReg(0x0D); f1 = readReg(0x0E); f0 = readReg(0x0F);
    Serial.printf("[TX] FREQ after rewrite: %02X %02X %02X (fw=0x%06lX)\n", f2, f1, f0, fw);

    // Also re-write key OOK config
    writeReg(0x12, 0x30);  // MDMCFG2: ASK/OOK, no sync
    writeReg(0x22, 0x11);  // FREND0: PA index 1

    // Re-write PA table
    digitalWrite(PIN_CC1101_CS, LOW);
    while (digitalRead(PIN_SPI_MISO)) {}
    SPI.transfer(0x3E | 0x40);  // PA table burst
    SPI.transfer(0x00);  // index 0: off
    SPI.transfer(0xC0);  // index 1: max power
    digitalWrite(PIN_CC1101_CS, HIGH);

    // FIFO TX mode
    writeReg(0x08, 0x00);  // PKTCTRL0: fixed length, FIFO, no CRC
    writeReg(0x06, frameLen);

    // Data rate ~2857 baud (350us/bit)
    writeReg(0x10, 0x86);  // MDMCFG4: DRATE_E=6
    writeReg(0x11, 0xCD);  // MDMCFG3: DRATE_M=205

    // Flush TX FIFO
    strobe(0x3B);  // SFTX

    for (int r = 0; r < TX_REPEATS; r++) {
        // Write frame to FIFO
        writeFifo(frameBuf, frameLen);

        // Start TX — CC1101 will transmit FIFO contents then return to IDLE
        strobe(0x35);  // STX

        // Wait for TX to complete (frame duration + margin)
        // Each byte = 8 chips * 350us = 2.8ms. Total ≈ frameLen * 2.8ms
        delay(frameLen * 3);

        // Flush FIFO for next frame
        strobe(0x36);  // SIDLE
        strobe(0x3B);  // SFTX
    }

    // Restore async serial mode for any future use
    writeReg(0x08, 0x30);  // PKTCTRL0: back to async serial

    strobe(0x36);  // SIDLE
    SPI.endTransaction();

    lastTxCode = code;
    lastTxTime = millis();
    Serial.println("[TX] Done.");
}

// ============================================================
// Display drawing
// ============================================================

// Draw a degree symbol (small circle) at the current cursor position.
// Font 1 doesn't include U+00B0, so we render it as a graphic primitive.
void drawDegreeSymbol(int x, int y, uint16_t color) {
    tft.drawCircle(x, y, 1, color);
}

// Small gear icon for the SETTINGS header (8x8 px, centered on cx,cy)
void drawGearIcon(int cx, int cy, uint16_t color) {
    // Body: two concentric shapes for a simple gear silhouette
    tft.fillCircle(cx, cy, 4, color);
    // Teeth (4 small rectangles at compass points)
    tft.fillRect(cx - 1, cy - 6, 2, 2, color);   // N
    tft.fillRect(cx - 1, cy + 4, 2, 2, color);   // S
    tft.fillRect(cx - 6, cy - 1, 2, 2, color);   // W
    tft.fillRect(cx + 4, cy - 1, 2, 2, color);   // E
    // Hole in the middle
    tft.fillCircle(cx, cy, 1, TFT_NAVY);
}

void drawHeader() {
    tft.fillRect(0, 0, 128, 14, TFT_NAVY);
    tft.setTextColor(TFT_WHITE, TFT_NAVY);
    tft.setTextSize(1);

    // Left-align the screen title with a bit of padding
    tft.setCursor(4, 3);
    const char* title = SCR_TITLES[currentScreen];
    tft.print(title);

    // Right-side: gear icon for all screens accessed via the settings hub
    bool isSettings = (currentScreen == SCR_SETTINGS ||
                       currentScreen == SCR_UNITS ||
                       currentScreen == SCR_THRESHOLDS ||
                       currentScreen == SCR_REMOTES ||
                       currentScreen == SCR_LEARN ||
                       currentScreen == SCR_FACTORY_RESET ||
                       currentScreen == SCR_SENSOR ||
                       currentScreen == SCR_STATUS ||
                       currentScreen == SCR_RFDIAG ||
                       currentScreen == SCR_BTNCAL);
    if (isSettings) {
        drawGearIcon(118, 7, TFT_WHITE);
    }
}

void drawFooter(const char* lb1, const char* lb2, const char* lb3) {
    int y = 118;
    tft.fillRect(0, y, 128, 10, TFT_DARKGREY);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
    tft.setTextSize(1);
    tft.setCursor(2, y + 1);
    tft.print(lb1);
    tft.setCursor(46, y + 1);
    tft.print(lb2);
    tft.setCursor(92, y + 1);
    tft.print(lb3);
}

// Placeholder fan icon drawn with primitives (center cx, cy; radius r; rotation in degrees)
// This is temporary — we'll swap in a pre-rendered bitmap during polish.
void drawFanIcon(int cx, int cy, int r, float angleDeg, uint16_t color) {
    // Hub
    tft.fillCircle(cx, cy, 3, color);
    // 3 blades, 120° apart
    for (int b = 0; b < 3; b++) {
        float a = (angleDeg + b * 120.0f) * 0.0174533f;  // deg → rad
        float c = cos(a), s = sin(a);
        // Each blade: an offset point + tangential width
        int tipX = cx + (int)(r * c);
        int tipY = cy + (int)(r * s);
        int midX = cx + (int)(r * 0.5f * c);
        int midY = cy + (int)(r * 0.5f * s);
        // Triangle blade
        int wX = (int)(r * 0.35f * -s);
        int wY = (int)(r * 0.35f * c);
        tft.fillTriangle(cx, cy, tipX + wX/2, tipY + wY/2, midX - wX, midY - wY, color);
    }
    // Outer ring
    tft.drawCircle(cx, cy, r + 2, TFT_DARKGREY);
}

// Runtime state for main screen
float lastRoomTempC = NAN;
FanSpeed currentFanSpeed = SPEED_OFF;
float fanAnimAngle = 0.0f;
unsigned long lastAnimUpdateMs = 0;

void drawMainScreen() {
    drawHeader();


    // Fallback: if we still don't have a temperature reading, try one now.
    // This covers cases where boot-time sensor.begin() reading didn't populate.
    if (isnan(lastRoomTempC) && sensor.isPresent()) {
        lastRoomTempC = sensor.readTemperatureC();
    }

    // --- AUTO/MANUAL badge (top-left) ---
    const char* modeLabel = settings.data.autoMode ? "AUTO" : "MANUAL";
    uint16_t badgeColor = settings.data.autoMode ? TFT_GREEN : TFT_ORANGE;
    int badgeW = settings.data.autoMode ? 32 : 44;
    tft.fillRoundRect(4, 18, badgeW, 12, 2, badgeColor);
    tft.setTextColor(TFT_BLACK, badgeColor);
    tft.setTextSize(1);
    tft.setCursor(settings.data.autoMode ? 10 : 8, 20);
    tft.print(modeLabel);

    // If a pause is active, show countdown "→ auto HH:MM"
    if (pauseUntilMs != 0) {
        unsigned long now = millis();
        long remainMs = (long)(pauseUntilMs - now);
        if (remainMs > 0) {
            int mins = remainMs / 60000;
            int hrs = mins / 60;
            mins %= 60;
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.setCursor(54, 20);
            tft.printf("%d:%02d", hrs, mins);
        }
    }

    // --- Fan icon (top right) ---
    drawFanIcon(96, 40, 18, fanAnimAngle,
                currentFanSpeed == SPEED_OFF ? TFT_DARKGREY : TFT_CYAN);

    // --- Current speed label (below fan, large) ---
    // Show pending speed when rotation is pending (preview before commit)
    FanSpeed shownSpeed = (pendingCommitMs != 0) ? pendingSpeed : currentFanSpeed;
    bool isPending = (pendingCommitMs != 0 && pendingSpeed != currentFanSpeed);
    tft.setTextSize(1);
    tft.setCursor(4, 40);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.print(isPending ? "Fan:  (commit...)" : "Fan:");
    tft.setTextSize(2);
    tft.setCursor(4, 52);
    uint16_t speedColor = shownSpeed == SPEED_OFF ? TFT_DARKGREY :
                          isPending ? TFT_YELLOW : TFT_WHITE;
    tft.setTextColor(speedColor, TFT_BLACK);
    tft.print(SPEED_NAMES[shownSpeed]);

    // --- Current temperature (big, center-bottom) ---
    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.setCursor(4, 78);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.print("Room temp:");
    // Use FreeSans font for a modern look
    tft.setFreeFont(&FreeSansBold18pt7b);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(4, 112);
    if (isnan(lastRoomTempC)) {
        tft.print("--");
    } else {
        tft.printf("%.0f", settings.displayTemp(lastRoomTempC));
    }
    // degree + unit (small, to the right). Font 1 has no ° glyph so we draw it.
    tft.setTextFont(1);
    tft.setTextSize(1);
    drawDegreeSymbol(58, 96, TFT_YELLOW);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(62, 95);
    tft.print(settings.tempUnitLabel());
}

void drawSettingsMenuScreen() {
    drawHeader();

    tft.setTextSize(1);

    tft.setCursor(4, 18);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.println("Select screen:");

    for (uint8_t i = 0; i < SETTINGS_MENU_COUNT; i++) {
        int y = 32 + i * 14;
        bool selected = (i == settingsCursor);
        if (selected) {
            tft.fillRect(0, y - 2, 128, 13, TFT_BLUE);
            tft.setTextColor(TFT_WHITE, TFT_BLUE);
        } else {
            tft.setTextColor(TFT_CYAN, TFT_BLACK);
        }
        tft.setCursor(8, y);
        tft.printf("%s %s", selected ? ">" : " ", SETTINGS_MENU_LABELS[i]);
    }

}

void drawSpeedPickerScreen() {
    drawHeader();

    tft.setTextSize(1);

    // Command list with cursor
    for (int i = 0; i < CMD_COUNT; i++) {
        int y = 22 + i * 18;
        bool selected = (i == controlCursor);

        if (selected) {
            tft.fillRect(0, y - 2, 128, 16, TFT_BLUE);
            tft.setTextColor(TFT_WHITE, TFT_BLUE);
        } else {
            tft.setTextColor(TFT_CYAN, TFT_BLACK);
        }
        tft.setCursor(8, y + 2);
        tft.setTextSize(1);
        tft.printf("%s %s", selected ? ">" : " ", CMD_NAMES[i]);
    }

    // Last TX info at bottom
    if (lastTxCode) {
        tft.setCursor(4, 104);
        unsigned long age = (millis() - lastTxTime) / 1000;
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.printf("TX: 0x%04lX %lus ago", lastTxCode, age);
    }

}

// ============================================================
// Pause picker — long-press MODE on main → opens this
// ============================================================
void drawPausePickerScreen() {
    drawHeader();

    tft.setTextSize(1);

    tft.setCursor(4, 18);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.println("Manual for...");

    for (int i = 0; i < PAUSE_COUNT; i++) {
        int y = 32 + i * 12;
        bool selected = (i == pauseCursor);
        if (selected) {
            tft.fillRect(0, y - 2, 128, 11, TFT_BLUE);
            tft.setTextColor(TFT_WHITE, TFT_BLUE);
        } else {
            tft.setTextColor(i == 0 ? TFT_RED : TFT_CYAN, TFT_BLACK);
        }
        tft.setCursor(8, y);
        tft.printf("%s %s", selected ? ">" : " ", PAUSE_LABELS[i]);
    }

}

// ============================================================
// Units screen — toggle C/F
// ============================================================
void drawUnitsScreen() {
    drawHeader();


    tft.setTextSize(1);
    tft.setCursor(4, 20);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.println("Temperature units:");

    // Two large options side by side
    bool isC = !settings.data.useFahrenheit;
    tft.fillRoundRect(8, 40, 52, 40, 5, isC ? TFT_GREEN : TFT_DARKGREY);
    tft.setTextColor(TFT_BLACK, isC ? TFT_GREEN : TFT_DARKGREY);
    tft.setTextSize(3);
    tft.setCursor(20, 50);
    tft.print("C");
    tft.setTextSize(1);
    tft.setCursor(14, 72);
    tft.print("Celsius");

    tft.fillRoundRect(68, 40, 52, 40, 5, !isC ? TFT_GREEN : TFT_DARKGREY);
    tft.setTextColor(TFT_BLACK, !isC ? TFT_GREEN : TFT_DARKGREY);
    tft.setTextSize(3);
    tft.setCursor(80, 50);
    tft.print("F");
    tft.setTextSize(1);
    tft.setCursor(72, 72);
    tft.print("Fahrenheit");

    tft.setTextSize(1);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(4, 95);
    tft.print("Sample: ");
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    if (!isnan(lastRoomTempC)) {
        int curX = tft.getCursorX();
        tft.printf("%.0f", settings.displayTemp(lastRoomTempC));
        int afterNum = tft.getCursorX();
        drawDegreeSymbol(afterNum + 1, 96, TFT_YELLOW);
        tft.setCursor(afterNum + 5, 95);
        tft.print(settings.tempUnitLabel());
    } else {
        tft.print("--");
    }
}

// ============================================================
// Threshold setter — visual bar + adjustable LOW/MED/HIGH setpoints
// ============================================================
void drawThresholdsScreen() {
    drawHeader();

    tft.setTextSize(1);

    // Convert thresholds to display units
    int8_t thresholds[3] = {
        settings.data.thresholdLow_C,
        settings.data.thresholdMed_C,
        settings.data.thresholdHigh_C
    };

    // Which setpoint is selected (highlighted)
    const char* labels[3] = {"LOW", "MED", "HIGH"};
    uint16_t colors[3] = {TFT_GREEN, TFT_YELLOW, TFT_RED};

    tft.setCursor(4, 18);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.println("Auto-mode thresholds:");

    // List of thresholds with cursor
    for (int i = 0; i < 3; i++) {
        int y = 32 + i * 14;
        bool selected = (i == thresholdCursor);
        if (selected) {
            tft.fillRect(0, y - 2, 128, 13, TFT_NAVY);
        }
        tft.setTextColor(colors[i], selected ? TFT_NAVY : TFT_BLACK);
        tft.setCursor(4, y);
        tft.printf("%s %s  %d",
                   selected ? ">" : " ",
                   labels[i],
                   (int)(settings.data.useFahrenheit ? thresholds[i] * 9.0f/5.0f + 32 : thresholds[i]));
        int afterNum = tft.getCursorX();
        drawDegreeSymbol(afterNum + 1, y + 1, colors[i]);
        tft.setCursor(afterNum + 5, y);
        tft.print(settings.data.useFahrenheit ? "F" : "C");
    }

    // Visual continuum bar: maps [min_C..max_C] → [x_left..x_right]
    int8_t minC = 15;  // 59°F
    int8_t maxC = 35;  // 95°F
    int barX = 4, barY = 78, barW = 120, barH = 8;
    tft.drawRect(barX, barY, barW, barH, TFT_DARKGREY);
    // Fill segments between thresholds
    auto xOf = [&](int c) {
        return barX + (int)((c - minC) / (float)(maxC - minC) * (barW - 2)) + 1;
    };
    int xL = xOf(thresholds[0]);
    int xM = xOf(thresholds[1]);
    int xH = xOf(thresholds[2]);
    tft.fillRect(barX+1, barY+1, xL - barX - 1, barH - 2, TFT_DARKGREY);      // OFF region
    tft.fillRect(xL, barY+1, xM - xL, barH - 2, TFT_GREEN);
    tft.fillRect(xM, barY+1, xH - xM, barH - 2, TFT_YELLOW);
    tft.fillRect(xH, barY+1, barX + barW - 1 - xH, barH - 2, TFT_RED);

    // Current temp indicator (if known)
    if (!isnan(lastRoomTempC)) {
        int xT = xOf((int)(lastRoomTempC + 0.5f));
        xT = max(barX, min(barX + barW - 1, xT));
        tft.drawFastVLine(xT, barY - 3, barH + 6, TFT_WHITE);
        tft.setCursor(barX, barY + barH + 2);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        int tY = barY + barH + 2;
        tft.printf("now: %.0f", settings.displayTemp(lastRoomTempC));
        int afterNum = tft.getCursorX();
        drawDegreeSymbol(afterNum + 1, tY + 1, TFT_WHITE);
        tft.setCursor(afterNum + 5, tY);
        tft.print(settings.tempUnitLabel());
    }

}

// ============================================================
// Remotes list — shows learned fans, select one to manage
// ============================================================
void drawRemotesScreen() {
    drawHeader();

    tft.setTextSize(1);

    tft.setCursor(4, 18);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.printf("%d/%d learned", settings.data.numFans, MAX_FANS);

    if (settings.data.numFans == 0) {
        tft.setCursor(4, 40);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.println("No remotes yet.");
        tft.setCursor(4, 54);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.println("Press LEARN to pair");
        tft.setCursor(4, 64);
        tft.println("your first remote.");
    } else {
        for (int i = 0; i < settings.data.numFans && i < 6; i++) {
            int y = 32 + i * 12;
            bool selected = (i == remoteCursor);
            if (selected) {
                tft.fillRect(0, y - 2, 128, 11, TFT_NAVY);
                tft.setTextColor(TFT_WHITE, TFT_NAVY);
            } else {
                tft.setTextColor(TFT_GREEN, TFT_BLACK);
            }
            tft.setCursor(4, y);
            tft.printf("%s %d %-9s 0x%03X",
                       selected ? ">" : " ",
                       i + 1,
                       settings.data.fans[i].name,
                       settings.data.fans[i].address);
        }
    }

}

// ============================================================
// Learn screen — user enters DIP switch settings from remote
// ============================================================
uint16_t learnComputeAddress() {
    // Upper 5 bits fixed at 00100, lower 4 bits from DIP state
    uint16_t addr = 0x040;  // 00100 0000
    for (int i = 0; i < 4; i++) {
        if (learnDip[i]) addr |= (1 << (3 - i));
    }
    return addr;
}

void drawLearnScreen() {
    drawHeader();

    tft.setTextSize(1);

    tft.setCursor(4, 18);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.println("Set DIP switches to");
    tft.setCursor(4, 28);
    tft.println("match your remote:");

    // DIP switch row
    for (int i = 0; i < 4; i++) {
        int x = 8 + i * 28;
        int y = 44;
        bool selected = (learnCursor == i);
        uint16_t borderColor = selected ? TFT_WHITE : TFT_DARKGREY;
        tft.drawRect(x, y, 22, 22, borderColor);
        // Fill background based on state
        tft.fillRect(x + 1, y + 1, 20, 20,
                     learnDip[i] ? TFT_GREEN : TFT_BLACK);
        // Label "1" "2" etc above
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.setCursor(x + 8, y - 10);
        tft.printf("%d", i + 1);
        // ON/off text inside
        tft.setTextColor(learnDip[i] ? TFT_BLACK : TFT_DARKGREY, learnDip[i] ? TFT_GREEN : TFT_BLACK);
        tft.setCursor(x + 3, y + 7);
        tft.print(learnDip[i] ? "ON " : "off");
    }

    // Computed address
    uint16_t addr = learnComputeAddress();
    tft.setCursor(4, 72);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.printf("Addr: 0x%03X", addr);

    // TEST / SAVE action buttons
    for (int i = 0; i < 2; i++) {
        int x = 8 + i * 56;
        int y = 86;
        bool selected = (learnCursor == 4 + i);
        uint16_t bg = selected ? TFT_BLUE : TFT_NAVY;
        tft.fillRoundRect(x, y, 52, 14, 3, bg);
        tft.setTextColor(TFT_WHITE, bg);
        tft.setCursor(x + (i == 0 ? 14 : 12), y + 4);
        tft.print(i == 0 ? "TEST" : "SAVE");
    }

    // Status line
    if (learnStatus[0] && millis() - learnTestTime < 2500) {
        tft.setCursor(4, 104);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.print(learnStatus);
    }

}

void handleLearnInput(Button btn) {
    const int CURSOR_POSITIONS = 6;  // 4 DIP + TEST + SAVE
    if (btn == BTN_BACK) {
        currentScreen = SCR_REMOTES;
        screenDirty = true;
    } else if (btn == BTN_ENC_CW) {
        learnCursor = (learnCursor + 1) % CURSOR_POSITIONS;
        screenDirty = true;
    } else if (btn == BTN_ENC_CCW) {
        learnCursor = (learnCursor + CURSOR_POSITIONS - 1) % CURSOR_POSITIONS;
        screenDirty = true;
    } else if (btn == BTN_ENC_PUSH) {
        if (learnCursor < 4) {
            // Toggle the DIP switch
            learnDip[learnCursor] = !learnDip[learnCursor];
        } else if (learnCursor == 4) {
            // TEST: send an OFF to the computed address
            uint16_t addr = learnComputeAddress();
            uint16_t code = (addr << 4) | SPEED_CMD_BITS[SPEED_OFF];
            Serial.printf("[LEARN] TEST send addr=0x%03X code=0x%04X\n", addr, code);
            radioSend(code);
            learnStatus = "TX sent — watch fan";
            learnTestTime = millis();
        } else {
            // SAVE: add as new fan
            uint16_t addr = learnComputeAddress();
            // Check if address already exists
            bool exists = false;
            for (int i = 0; i < settings.data.numFans; i++) {
                if (settings.data.fans[i].address == addr) { exists = true; break; }
            }
            if (exists) {
                learnStatus = "Already learned!";
                learnTestTime = millis();
            } else if (settings.data.numFans >= MAX_FANS) {
                learnStatus = "Max fans reached";
                learnTestTime = millis();
            } else {
                int slot = settings.addFan(addr, nullptr);
                Serial.printf("[LEARN] Saved addr=0x%03X as slot %d\n", addr, slot);
                learnStatus = "Saved!";
                learnTestTime = millis();
            }
        }
        screenDirty = true;
    }
}

// ============================================================
// Factory reset confirmation
// ============================================================
void drawFactoryResetScreen() {
    drawHeader();


    tft.setTextSize(1);
    tft.setCursor(4, 20);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("FACTORY RESET?");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(4, 40);
    tft.println("All settings, learned");
    tft.setCursor(4, 50);
    tft.println("remotes, and thresholds");
    tft.setCursor(4, 60);
    tft.println("will be erased.");

    tft.setCursor(4, 82);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.println("Hold OK for 2s to");
    tft.setCursor(4, 92);
    tft.println("confirm.");

}

void drawStatusScreen() {
    drawHeader();

    tft.setTextSize(1);

    tft.setCursor(4, 18);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.printf("Learned: %d/%d fans", settings.data.numFans, MAX_FANS);

    if (settings.data.numFans == 0) {
        tft.setCursor(4, 36);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.println("None learned yet.");
        tft.setCursor(4, 50);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.println("Use LEARN in settings");
        tft.setCursor(4, 60);
        tft.println("to pair a remote.");
    } else {
        // Header row
        tft.setCursor(4, 32);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.println("# Name       Addr OFF");
        for (int i = 0; i < settings.data.numFans; i++) {
            tft.setCursor(4, 44 + i * 10);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.printf("%d %-9s %03X %04X",
                       i + 1,
                       settings.data.fans[i].name,
                       settings.data.fans[i].address,
                       settings.fanCode(i, SPEED_OFF));
        }
    }

    tft.setCursor(4, 108);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.printf("RF: %.3f MHz", RF_FREQUENCY);

}

void drawSensorScreen() {
    drawHeader();

    tft.setTextSize(1);

    if (!sensor.isPresent()) {
        tft.setCursor(4, 40);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.println("MXC6655 not found");
        tft.setCursor(4, 56);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.println("Check I2C wiring:");
        tft.setCursor(4, 66);
        tft.printf("SDA=GPIO%d SCL=GPIO%d", PIN_I2C_SDA, PIN_I2C_SCL);
        return;
    }

    // Big temperature reading
    float tempC = sensor.readTemperatureC();
    float tempF = tempC * 9.0f / 5.0f + 32.0f;
    int8_t rawT = sensor.readTemperatureRaw();

    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setCursor(4, 18);
    tft.print("Room Temperature");

    tft.setTextSize(3);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(10, 32);
    tft.printf("%2.1f", tempC);
    tft.setTextSize(1);
    tft.setCursor(100, 32);
    tft.print("C");

    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, 60);
    tft.printf("%2.1f", tempF);
    tft.setTextSize(1);
    tft.setCursor(80, 64);
    tft.print("F");

    // Accelerometer readings
    float ax, ay, az;
    if (sensor.readAcceleration(ax, ay, az)) {
        tft.setTextSize(1);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.setCursor(4, 82);
        tft.printf("X:%+.2f Y:%+.2f", ax, ay);
        tft.setCursor(4, 92);
        tft.printf("Z:%+.2f g", az);
    }

    tft.setCursor(4, 104);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.printf("raw=0x%02X id=0x%02X", (uint8_t)rawT, sensor.deviceId());

}

void drawRfDiagScreen() {
    drawHeader();

    tft.setTextSize(1);

    tft.setCursor(4, 20);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.println("RF Diagnostics");
    tft.setCursor(4, 36);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.println("Press MID for");
    tft.setCursor(4, 46);
    tft.println("register dump");
    tft.setCursor(4, 56);
    tft.println("(output to serial)");

}

void drawBtnCalScreen() {
    drawHeader();

    int raw = analogRead(PIN_BTN_ADC);

    tft.setTextSize(1);
    tft.setCursor(4, 18);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.printf("ADC (GPIO%d):", PIN_BTN_ADC);

    // Big ADC value
    tft.setTextSize(3);
    tft.setCursor(4, 30);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("%4d", raw);

    // Bar graph
    tft.setTextSize(1);
    int barW = map(raw, 0, 4095, 0, 120);
    tft.fillRect(4, 60, 120, 6, TFT_DARKGREY);
    tft.fillRect(4, 60, barW, 6, TFT_CYAN);

    // ADC ladder now only carries encoder push + back
    const char* labels[] = {
        "ENC PUSH", "BACK"
    };
    int bandMins[] = {0,    1500};
    int bandMaxs[] = {300,  2200};

    tft.setCursor(4, 72);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.println("Band  Range    Detects");

    for (int i = 0; i < 2; i++) {
        int y = 82 + i * 10;
        bool hit = (raw >= bandMins[i] && raw < bandMaxs[i]);
        if (hit) {
            tft.fillRect(0, y - 1, 128, 10, TFT_GREEN);
            tft.setTextColor(TFT_BLACK, TFT_GREEN);
        } else {
            tft.setTextColor(TFT_CYAN, TFT_BLACK);
        }
        tft.setCursor(4, y);
        tft.printf("%-8s %4d-%4d %s",
                   labels[i], bandMins[i], bandMaxs[i],
                   hit ? "<-- YES" : "");
    }

    // Encoder diagnostics (accumulated raw quadrature steps since boot)
    static int32_t encTotal = 0;
    noInterrupts();
    int8_t d = g_encoderDelta;
    interrupts();
    // Don't consume the delta here — just peek. Total accumulated count is
    // informational; individual CW/CCW events get consumed by read().
    tft.setCursor(4, 105);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.printf("ENC delta pending: %+d", d);
}

void drawScreen() {
    if (!screenDirty) return;
    screenDirty = false;

    // Render to off-screen sprite first, then push atomically (zero flicker)
    tft.fillSprite(TFT_BLACK);

    switch (currentScreen) {
        case SCR_MAIN:           drawMainScreen(); break;
        case SCR_SETTINGS:       drawSettingsMenuScreen(); break;
        case SCR_SPEED_PICKER:   drawSpeedPickerScreen(); break;
        case SCR_PAUSE_PICKER:   drawPausePickerScreen(); break;
        case SCR_UNITS:          drawUnitsScreen(); break;
        case SCR_THRESHOLDS:     drawThresholdsScreen(); break;
        case SCR_REMOTES:        drawRemotesScreen(); break;
        case SCR_LEARN:          drawLearnScreen(); break;
        case SCR_FACTORY_RESET:  drawFactoryResetScreen(); break;
        case SCR_SENSOR:         drawSensorScreen(); break;
        case SCR_STATUS:         drawStatusScreen(); break;
        case SCR_RFDIAG:         drawRfDiagScreen(); break;
        case SCR_BTNCAL:         drawBtnCalScreen(); break;
        default: break;
    }

    tft.pushSprite(0, 0);  // Atomic push to display via DMA
}

// ============================================================
// Input handling per screen
// ============================================================

// Navigate back to the home menu (BTN1 on every subscreen)
void goMain() {
    currentScreen = SCR_MAIN;
    screenDirty = true;
}

void goSettings() {
    currentScreen = SCR_SETTINGS;
    screenDirty = true;
}

void handleMainInput(Button btn) {
    if (btn == BTN_ENC_CCW) {
        // Rotate up: step fan speed DOWN (OFF → ... backwards is CCW by convention)
        if (pendingCommitMs == 0) pendingSpeed = currentFanSpeed;
        if (pendingSpeed > SPEED_OFF) pendingSpeed = (FanSpeed)(pendingSpeed - 1);
        pendingCommitMs = millis() + 500;
        settings.data.autoMode = false;  // manual override
        screenDirty = true;
    } else if (btn == BTN_ENC_CW) {
        // Rotate down: step fan speed UP (LOW → MED → HIGH)
        if (pendingCommitMs == 0) pendingSpeed = currentFanSpeed;
        if (pendingSpeed < SPEED_HIGH) pendingSpeed = (FanSpeed)(pendingSpeed + 1);
        pendingCommitMs = millis() + 500;
        settings.data.autoMode = false;  // manual override
        screenDirty = true;
    } else if (btn == BTN_ENC_PUSH) {
        // Push: toggle auto/manual (cancels any active pause)
        settings.data.autoMode = !settings.data.autoMode;
        pauseUntilMs = 0;
        settings.save();
        Serial.printf("[MODE] %s\n", settings.data.autoMode ? "AUTO" : "MANUAL");
        screenDirty = true;
    } else if (btn == BTN_BACK) {
        // Back: open settings menu
        goSettings();
    } else if (btn == BTN_BACK_LONG) {
        // Long back: open pause-for submenu
        pauseCursor = 2;  // default to "1 hour"
        currentScreen = SCR_PAUSE_PICKER;
        screenDirty = true;
    }
}

void handlePausePickerInput(Button btn) {
    if (btn == BTN_BACK) {
        goMain();
    } else if (btn == BTN_ENC_CCW) {
        pauseCursor = (pauseCursor + PAUSE_COUNT - 1) % PAUSE_COUNT;
        screenDirty = true;
    } else if (btn == BTN_ENC_CW) {
        pauseCursor = (pauseCursor + 1) % PAUSE_COUNT;
        screenDirty = true;
    } else if (btn == BTN_ENC_PUSH) {
        if (pauseCursor == 0) {
            pauseUntilMs = 0;
            settings.data.autoMode = true;
            settings.save();
            Serial.println("[PAUSE] Cancelled — auto mode resumed");
        } else {
            unsigned long mins = PAUSE_DURATIONS_MIN[pauseCursor];
            pauseUntilMs = millis() + mins * 60UL * 1000UL;
            settings.data.autoMode = false;
            settings.save();
            Serial.printf("[PAUSE] Manual mode for %lu minutes\n", mins);
        }
        goMain();
    }
}

void handleUnitsInput(Button btn) {
    if (btn == BTN_BACK) {
        goSettings();
    } else if (btn == BTN_ENC_CCW || btn == BTN_ENC_CW) {
        // Rotate: toggle between C and F (preview)
        settings.data.useFahrenheit = !settings.data.useFahrenheit;
        Serial.printf("[UNITS] preview %s\n", settings.data.useFahrenheit ? "F" : "C");
        screenDirty = true;
    } else if (btn == BTN_ENC_PUSH) {
        // Push: save and return to settings
        settings.save();
        Serial.printf("[UNITS] saved %s\n", settings.data.useFahrenheit ? "F" : "C");
        goSettings();
    }
}

void handleThresholdsInput(Button btn) {
    int8_t* targets[3] = {
        &settings.data.thresholdLow_C,
        &settings.data.thresholdMed_C,
        &settings.data.thresholdHigh_C
    };
    int8_t* t = targets[thresholdCursor];
    int8_t prevMin = (thresholdCursor == 0) ? 0 : *targets[thresholdCursor - 1];
    int8_t nextMax = (thresholdCursor == 2) ? 40 : *targets[thresholdCursor + 1];

    if (btn == BTN_BACK) {
        settings.save();
        goSettings();
    } else if (btn == BTN_ENC_CCW) {
        if (*t > prevMin + 1 || (thresholdCursor == 0 && *t > 10)) {
            (*t)--;
            screenDirty = true;
        }
    } else if (btn == BTN_ENC_CW) {
        if (*t < nextMax - 1 || (thresholdCursor == 2 && *t < 40)) {
            (*t)++;
            screenDirty = true;
        }
    } else if (btn == BTN_ENC_PUSH) {
        thresholdCursor = (thresholdCursor + 1) % 3;
        screenDirty = true;
    }
}

void handleRemotesInput(Button btn) {
    if (btn == BTN_BACK) {
        goSettings();
    } else if (btn == BTN_ENC_CCW) {
        if (settings.data.numFans > 0) {
            remoteCursor = (remoteCursor + settings.data.numFans - 1) % settings.data.numFans;
            screenDirty = true;
        }
    } else if (btn == BTN_ENC_CW) {
        if (settings.data.numFans > 0) {
            remoteCursor = (remoteCursor + 1) % settings.data.numFans;
            screenDirty = true;
        }
    } else if (btn == BTN_ENC_PUSH) {
        // Push opens the learn flow to add a new remote
        currentScreen = SCR_LEARN;
        screenDirty = true;
    } else if (btn == BTN_ENC_PUSH_LONG) {
        // Long-press encoder deletes the currently selected remote
        if (settings.data.numFans > 0 && remoteCursor < settings.data.numFans) {
            Serial.printf("[REMOTES] Deleting %s\n",
                          settings.data.fans[remoteCursor].name);
            settings.removeFan(remoteCursor);
            if (remoteCursor >= settings.data.numFans && remoteCursor > 0) {
                remoteCursor--;
            }
            screenDirty = true;
        }
    }
}

void handleFactoryResetInput(Button btn) {
    if (btn == BTN_BACK) {
        goSettings();
    } else if (btn == BTN_ENC_PUSH_LONG) {
        // Held for 2s — perform reset
        settings.factoryReset();
        Serial.println("[RESET] Rebooting...");
        delay(500);
        ESP.restart();
    }
}

void handleSettingsMenuInput(Button btn) {
    if (btn == BTN_BACK) {
        goMain();
    } else if (btn == BTN_ENC_CCW) {
        settingsCursor = (settingsCursor + SETTINGS_MENU_COUNT - 1) % SETTINGS_MENU_COUNT;
        screenDirty = true;
    } else if (btn == BTN_ENC_CW) {
        settingsCursor = (settingsCursor + 1) % SETTINGS_MENU_COUNT;
        screenDirty = true;
    } else if (btn == BTN_ENC_PUSH) {
        currentScreen = SETTINGS_MENU[settingsCursor];
        screenDirty = true;
    }
}

void handleSpeedPickerInput(Button btn) {
    if (btn == BTN_BACK) {
        goMain();
    } else if (btn == BTN_ENC_CCW) {
        controlCursor = (controlCursor + CMD_COUNT - 1) % CMD_COUNT;
        screenDirty = true;
    } else if (btn == BTN_ENC_CW) {
        controlCursor = (controlCursor + 1) % CMD_COUNT;
        screenDirty = true;
    } else if (btn == BTN_ENC_PUSH) {
        sendToAllFans((FanSpeed)controlCursor);
        screenDirty = true;
    }
}

void handleStatusInput(Button btn) {
    if (btn == BTN_BACK) goSettings();
}

void handleSensorInput(Button btn) {
    if (btn == BTN_BACK) goSettings();
}

void handleRfDiagInput(Button btn) {
    if (btn == BTN_BACK) goSettings();
    else if (btn == BTN_ENC_PUSH) {
        radio.dumpRegisters();
        screenDirty = true;
    }
}

void handleBtnCalInput(Button btn) {
    if (btn == BTN_BACK) goSettings();
}

void handleInput(Button btn) {
    if (btn == BTN_NONE) return;
    Serial.printf("[BTN] Button %d on screen %s\n", btn, SCR_TITLES[currentScreen]);

    switch (currentScreen) {
        case SCR_MAIN:             handleMainInput(btn); break;
        case SCR_SETTINGS:         handleSettingsMenuInput(btn); break;
        case SCR_SPEED_PICKER:     handleSpeedPickerInput(btn); break;
        case SCR_PAUSE_PICKER:     handlePausePickerInput(btn); break;
        case SCR_UNITS:            handleUnitsInput(btn); break;
        case SCR_THRESHOLDS:       handleThresholdsInput(btn); break;
        case SCR_REMOTES:          handleRemotesInput(btn); break;
        case SCR_LEARN:            handleLearnInput(btn); break;
        case SCR_FACTORY_RESET:    handleFactoryResetInput(btn); break;
        case SCR_SENSOR:  handleSensorInput(btn); break;
        case SCR_STATUS:  handleStatusInput(btn); break;
        case SCR_RFDIAG:  handleRfDiagInput(btn); break;
        case SCR_BTNCAL:  handleBtnCalInput(btn); break;
        default: break;
    }
}

// ============================================================
// Setup & Loop
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("=== Minka-Aire Fan Controller ===");

    // --- Settings (NVS) ---
    settings.begin();
    if (settings.isFirstBoot() && settings.data.numFans == 0) {
        // Seed with the two fans we know about for development convenience.
        // Production builds will start empty and require LEARN flow.
        settings.addFan(0x041, "Living LR");  // DIP 0001
        settings.addFan(0x045, "Living RR");  // DIP 0101
        Serial.println("[SETTINGS] First boot: seeded 2 known fans");
    }

    // --- Buttons ---
    buttons.begin();

    // --- SPI + CC1101 Radio ---
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    bool cc1101_ok = radio.begin();
    Serial.printf("[CC1101] Detected: %s (v0x%02X)\n",
                  cc1101_ok ? "YES" : "NO", radio.version());

    // Always configure — even if begin() fails, try anyway
    Serial.printf("[CC1101] RF_FREQUENCY = %.3f MHz\n", (float)RF_FREQUENCY);
    radio.configureOOK(RF_FREQUENCY);
    radio.dumpRegisters();

    // --- MXC6655XA I2C Sensor ---
    if (sensor.begin()) {
        lastRoomTempC = sensor.readTemperatureC();
        float x, y, z;
        sensor.readAcceleration(x, y, z);
        Serial.printf("[MXC6655] OK — temp=%.1fC, accel=(%.2f, %.2f, %.2f)g\n",
                      lastRoomTempC, x, y, z);
    } else {
        Serial.println("[MXC6655] *** NOT DETECTED *** (check I2C wiring)");
    }

    // Restore last-commanded fan speed from NVS so the display shows truth
    currentFanSpeed = (FanSpeed)settings.data.manualSpeed;

    // --- TFT LCD ---
    _lcd.init();
    _lcd.setRotation(0);  // portrait 128x128
    _lcd.fillScreen(TFT_BLACK);

    // Allocate the off-screen sprite framebuffer (128x128 @ 16bpp = 32KB)
    tft.createSprite(128, 128);
    tft.setColorDepth(16);
    Serial.println("[TFT] ST7735 128x128 initialized");

    // --- Splash screen (renders to sprite, pushes each frame) ---
    for (int i = 0; i < 24; i++) {
        tft.fillSprite(TFT_BLACK);
        drawFanIcon(64, 50, 22, i * 15.0f, TFT_CYAN);
        tft.setFreeFont(&FreeSansBold9pt7b);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(10, 18);
        tft.print("MINKA-AIRE");
        tft.setFreeFont(&FreeSans9pt7b);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.setCursor(24, 95);
        tft.print("Controller");
        tft.setTextFont(1);
        tft.setCursor(32, 110);
        tft.print("v1.0");
        tft.pushSprite(0, 0);
        delay(40);
    }

    screenDirty = true;
    drawScreen();

    Serial.println("[READY] Fan control operational.");
}

void loop() {
    Button btn = buttons.read();
    handleInput(btn);

    unsigned long now = millis();

    // Debounced fan-speed commit from encoder rotation on main screen
    if (pendingCommitMs != 0 && (long)(now - pendingCommitMs) >= 0) {
        pendingCommitMs = 0;
        if (pendingSpeed != currentFanSpeed) {
            Serial.printf("[MAIN] Commit rotation → %s\n", SPEED_NAMES[pendingSpeed]);
            sendToAllFans(pendingSpeed);
            screenDirty = true;
        }
    }

    // Poll sensor every 2 seconds, regardless of screen (room temp updates live)
    static unsigned long lastSensorPoll = 0;
    static uint8_t currentRotation = 0;
    static uint8_t rotationDebounce = 0;
    // If sensor failed at boot, retry every 30s — maybe it was just a slow
    // power-up or flaky connection
    static unsigned long lastSensorRetry = 0;
    if (!sensor.isPresent() && now - lastSensorRetry > 30000) {
        lastSensorRetry = now;
        Serial.println("[MXC6655] Sensor absent — retrying detection...");
        if (sensor.retry()) {
            lastRoomTempC = sensor.readTemperatureC();
            screenDirty = true;
        }
    }

    if (sensor.isPresent() && now - lastSensorPoll > 2000) {
        lastSensorPoll = now;
        lastRoomTempC = sensor.readTemperatureC();

        // Auto-rotate screen: use accelerometer Y axis to detect upside-down
        float ax, ay, az;
        if (sensor.readAcceleration(ax, ay, az)) {
            uint8_t desiredRotation = (ay > 0.3f) ? 2 : (ay < -0.3f) ? 0 : currentRotation;
            if (desiredRotation != currentRotation) {
                // Debounce: require 3 consecutive agreeing readings before flipping
                if (++rotationDebounce >= 3) {
                    currentRotation = desiredRotation;
                    rotationDebounce = 0;
                    _lcd.setRotation(currentRotation);
                    _lcd.fillScreen(TFT_BLACK);
                    Serial.printf("[ROT] Screen rotation = %d\n", currentRotation);
                    screenDirty = true;
                }
            } else {
                rotationDebounce = 0;
            }
        }

        if (currentScreen == SCR_MAIN || currentScreen == SCR_SENSOR) {
            screenDirty = true;
        }
    }

    // --- Auto-mode timer expiry: resume auto when pause finishes ---
    if (pauseUntilMs != 0 && (long)(now - pauseUntilMs) >= 0) {
        Serial.println("[PAUSE] Timer expired, resuming auto mode");
        pauseUntilMs = 0;
        settings.data.autoMode = true;
        settings.save();
        if (currentScreen == SCR_MAIN) screenDirty = true;
    }

    // --- Auto control loop: check temp every 30s and adjust fan speed ---
    static unsigned long lastAutoCheck = 0;
    const unsigned long AUTO_CHECK_INTERVAL_MS = 30000;
    if (settings.data.autoMode &&
        !isnan(lastRoomTempC) &&
        sensor.isPresent() &&
        settings.data.numFans > 0 &&
        now - lastAutoCheck > AUTO_CHECK_INTERVAL_MS) {
        lastAutoCheck = now;
        FanSpeed target = settings.autoSpeedForTemp(lastRoomTempC, currentFanSpeed);
        if (target != currentFanSpeed) {
            Serial.printf("[AUTO] %.1fC: %s → %s\n",
                          lastRoomTempC,
                          SPEED_NAMES[currentFanSpeed], SPEED_NAMES[target]);
            sendToAllFans(target);
            if (currentScreen == SCR_MAIN) screenDirty = true;
        }
    }

    // Pause-timer countdown refresh on main (every minute tick)
    if (pauseUntilMs != 0 && currentScreen == SCR_MAIN) {
        static unsigned long lastPauseRefresh = 0;
        if (now - lastPauseRefresh > 30000) {
            lastPauseRefresh = now;
            screenDirty = true;
        }
    }

    // Fan animation: small angle steps at high frame rate for smooth rotation.
    // Rotation speed scales with fan speed; angle step is small (3°) so the
    // motion feels fluid rather than jumpy.
    if (currentFanSpeed != SPEED_OFF && currentScreen == SCR_MAIN) {
        unsigned long frameMs =
            currentFanSpeed == SPEED_LOW  ? 25 :
            currentFanSpeed == SPEED_MED  ? 12 : 6;
        if (now - lastAnimUpdateMs > frameMs) {
            lastAnimUpdateMs = now;
            fanAnimAngle += 3.0f;
            if (fanAnimAngle >= 360.0f) fanAnimAngle -= 360.0f;
            screenDirty = true;
        }
    }

    drawScreen();

    // Periodic refresh for live diagnostic screens
    static unsigned long lastRefresh = 0;
    if (currentScreen == SCR_BTNCAL && now - lastRefresh > 200) {
        lastRefresh = now;
        screenDirty = true;
    }
    if (currentScreen == SCR_SENSOR && now - lastRefresh > 500) {
        lastRefresh = now;
        screenDirty = true;
    }
}
