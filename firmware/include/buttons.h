#pragma once
#include <Arduino.h>
#include "pins.h"

// Button IDs (short press events).
// Physical inputs on the resistor ladder, ordered from closest to sense point
// (lowest ADC when pressed) to farthest:
//   BTN_1 = rotary encoder push (1R from sense) — confirm/enter
//   BTN_2 = dedicated back button (2R)           — back/cancel
//   BTN_3 = encoder phase A pulse (3R)           — rotate CCW (up/decrease)
//   BTN_4 = encoder phase B pulse (4R)           — rotate CW  (down/increase)
enum Button : uint8_t {
    BTN_NONE = 0,
    BTN_1, BTN_2, BTN_3, BTN_4,
    BTN_1_LONG, BTN_2_LONG, BTN_3_LONG, BTN_4_LONG,
};

// Semantic aliases — use these in handlers so intent is clear
#define BTN_ENC_PUSH       BTN_1
#define BTN_BACK           BTN_2
#define BTN_ENC_CCW        BTN_3   // rotate counter-clockwise = up/prev
#define BTN_ENC_CW         BTN_4   // rotate clockwise         = down/next
#define BTN_ENC_PUSH_LONG  BTN_1_LONG
#define BTN_BACK_LONG      BTN_2_LONG

inline bool isLongPress(Button b) {
    return b >= BTN_1_LONG && b <= BTN_4_LONG;
}
inline Button toShort(Button b) {
    return isLongPress(b) ? (Button)(b - 4) : b;
}
inline bool isEncoderRotation(Button b) {
    return b == BTN_ENC_CCW || b == BTN_ENC_CW;
}

// Resistor ladder on GPIO 36 (ADC1_CH0) with FOUR switches now:
//   3V3 → R(1k) → sense_node → R(1k) → R(1k) → R(1k) → R(1k)
//                                BTN1↓  BTN2↓   BTN3↓   BTN4↓
// Theoretical values (real ADC is nonlinear; BTNCAL screen shows actuals):
//   No press: ~4095
//   BTN_1 (1R): 3.3 * 1/2   = 1.65V  → ~2048
//   BTN_2 (2R): 3.3 * 2/3   = 2.20V  → ~2730
//   BTN_3 (3R): 3.3 * 3/4   = 2.48V  → ~3072
//   BTN_4 (4R): 3.3 * 4/5   = 2.64V  → ~3276

// Shorter debounce to keep up with rapid encoder rotation, longer long-press
// threshold since only BACK uses long-press now.
static const unsigned long BTN_DEBOUNCE_MS = 10;
static const unsigned long BTN_LONG_PRESS_MS = 700;
static const unsigned long BTN_DEBUG_INTERVAL_MS = 2000;

// Quadrature decoder state (interrupt-driven, modified in ISR context)
// Standard 16-entry transition table: index is (prev_state<<2 | curr_state),
// value is +1/-1/0 representing one "quadrature step". Most detent encoders
// emit 4 steps per physical click.
static const int8_t ENCODER_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};
volatile int8_t g_encoderDelta = 0;      // accumulates rotation steps
volatile uint8_t g_encoderState = 0;     // last (A,B) 2-bit state

void IRAM_ATTR encoderISR() {
    uint8_t s = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
    g_encoderDelta += ENCODER_TABLE[(g_encoderState << 2) | s];
    g_encoderState = s;
}

class ButtonReader {
public:
    void begin() {
        // Configure ADC for this specific pin
        analogReadResolution(12);
        analogSetPinAttenuation(PIN_BTN_ADC, ADC_11db);
        pinMode(PIN_BTN_ADC, INPUT);

        _lastButton = BTN_NONE;
        _lastChangeMs = 0;
        _lastDebugMs = 0;

        // Take a few throwaway reads to let ADC settle
        for (int i = 0; i < 5; i++) {
            analogRead(PIN_BTN_ADC);
            delay(10);
        }

        int baseline = analogRead(PIN_BTN_ADC);
        Serial.printf("[BTN] ADC baseline on GPIO %d: %d\n", PIN_BTN_ADC, baseline);

        // Rotary encoder A/B on dedicated digital inputs with internal pullups
        pinMode(PIN_ENC_A, INPUT_PULLUP);
        pinMode(PIN_ENC_B, INPUT_PULLUP);
        g_encoderState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
        g_encoderDelta = 0;
        attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);
        Serial.printf("[BTN] Encoder on GPIO %d/%d\n", PIN_ENC_A, PIN_ENC_B);
    }

    // Returns button events:
    //   - SHORT press fires on RELEASE (if held < LONG_PRESS_MS)
    //   - LONG press fires at LONG_PRESS_MS threshold while still held
    //     (subsequent release is suppressed)
    //   - Encoder rotation fires BTN_ENC_CW/CCW on each full detent (4 steps)
    Button read() {
        // Consume one detent worth of encoder rotation if accumulated.
        // Most detent encoders produce 4 quadrature steps per physical click.
        noInterrupts();
        int8_t d = g_encoderDelta;
        if (d >= 4)       { g_encoderDelta -= 4; interrupts(); return BTN_ENC_CW;  }
        else if (d <= -4) { g_encoderDelta += 4; interrupts(); return BTN_ENC_CCW; }
        interrupts();

        int raw = analogRead(PIN_BTN_ADC);
        Button cur = classify(raw);
        unsigned long now = millis();

        // Log when a new button is first detected (press edge) — shows the raw
        // ADC value which is critical for calibration
        if (cur != BTN_NONE && cur != _lastLoggedBtn) {
            Serial.printf("[BTN] ADC=%d -> B%d\n", raw, cur);
            _lastLoggedBtn = cur;
        } else if (cur == BTN_NONE) {
            _lastLoggedBtn = BTN_NONE;
        }

        // State change detected
        if (cur != _stableBtn) {
            // Debounce: require stability
            if (now - _lastEdgeMs > BTN_DEBOUNCE_MS) {
                // Transition from pressed → released?
                if (_stableBtn != BTN_NONE && cur == BTN_NONE) {
                    Button released = _stableBtn;
                    _stableBtn = BTN_NONE;
                    _pressStartMs = 0;
                    _lastEdgeMs = now;
                    // If long-press already fired, suppress the short event
                    if (_longFired) {
                        _longFired = false;
                        return BTN_NONE;
                    }
                    Serial.printf("[BTN] SHORT press: B%d\n", released);
                    return released;
                }
                // Transition from released → pressed
                if (_stableBtn == BTN_NONE && cur != BTN_NONE) {
                    _stableBtn = cur;
                    _pressStartMs = now;
                    _longFired = false;
                    _lastEdgeMs = now;
                    // Don't emit on press — wait for release or long-press timeout
                }
                // Transition directly between two different buttons (rare)
                if (_stableBtn != BTN_NONE && cur != BTN_NONE && cur != _stableBtn) {
                    _stableBtn = cur;
                    _pressStartMs = now;
                    _longFired = false;
                    _lastEdgeMs = now;
                }
            }
        }

        // Long-press detection while button is held
        if (_stableBtn != BTN_NONE && !_longFired &&
            (now - _pressStartMs) >= BTN_LONG_PRESS_MS) {
            _longFired = true;
            Button lp = (Button)((int)_stableBtn + 4);  // BTN_1 → BTN_1_LONG etc.
            Serial.printf("[BTN] LONG press: B%d\n", lp);
            return lp;
        }

        return BTN_NONE;
    }

    // Raw ADC value for calibration/debug
    int rawAdc() {
        return analogRead(PIN_BTN_ADC);
    }

private:
    Button _stableBtn = BTN_NONE;
    unsigned long _pressStartMs = 0;
    unsigned long _lastEdgeMs = 0;
    unsigned long _lastDebugMs = 0;
    bool _longFired = false;
    Button _lastLoggedBtn = BTN_NONE;

    // Legacy fields kept for API compat
    Button _lastButton;
    unsigned long _lastChangeMs;

    Button classify(int adc) {
        // Only push and back on the ADC ladder now. Encoder A/B rotation
        // is decoded via interrupts on GPIO 32/33 (see encoderISR).
        // If the old encoder resistors are still in the ladder, their ADC
        // bands are simply ignored here to prevent double-firing.
        //   Idle:        ~4095
        //   Dial push:   ~0     (direct short to GND)
        //   Back button: ~1900
        if (adc < 300)                    return BTN_ENC_PUSH;
        if (adc >= 1500 && adc < 2200)    return BTN_BACK;
        return BTN_NONE;
    }
};
