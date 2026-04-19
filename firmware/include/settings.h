#pragma once
#include <Arduino.h>
#include <Preferences.h>

// Maximum number of remotes/fans the controller can learn
#define MAX_FANS 8

// Fan speed identifiers
enum FanSpeed : uint8_t {
    SPEED_OFF  = 0,
    SPEED_LOW  = 1,
    SPEED_MED  = 2,
    SPEED_HIGH = 3,
    SPEED_COUNT
};

static const char* SPEED_NAMES[SPEED_COUNT] = {"OFF", "LOW", "MED", "HIGH"};

// Command bit patterns (lower 4 bits of the 13-bit code), derived from SDR capture.
// Full code = (address << 4) | SPEED_CMD_BITS[speed]
static const uint8_t SPEED_CMD_BITS[SPEED_COUNT] = {
    0b1010,  // OFF
    0b0010,  // LOW
    0b0100,  // MED
    0b1000,  // HIGH
};

// One learned fan: address bits + display label
struct LearnedFan {
    bool     valid;
    uint16_t address;          // 9-bit DIP-derived address (upper bits of code)
    char     name[12];         // user label, e.g. "Living LR"
};

// Persistent settings (saved to NVS)
struct Settings {
    uint8_t  version;          // schema version for future migrations
    bool     autoMode;         // true=auto control by temp, false=manual
    uint8_t  manualSpeed;      // last commanded speed (FanSpeed)
    bool     useFahrenheit;    // false=Celsius
    int8_t   thresholdLow_C;   // temp at which fan engages LOW
    int8_t   thresholdMed_C;   // ... MED
    int8_t   thresholdHigh_C;  // ... HIGH
    int8_t   hysteresisC;      // buffer when stepping down
    uint8_t  numFans;          // count of valid entries in fans[]
    LearnedFan fans[MAX_FANS];
};

// First-boot defaults
inline void settingsDefaults(Settings& s) {
    s.version          = 1;
    s.autoMode         = true;
    s.manualSpeed      = SPEED_OFF;
    s.useFahrenheit    = true;       // most US users
    s.thresholdLow_C   = 22;         // 72°F
    s.thresholdMed_C   = 25;         // 77°F
    s.thresholdHigh_C  = 28;         // 82°F
    s.hysteresisC      = 1;
    s.numFans          = 0;
    for (int i = 0; i < MAX_FANS; i++) {
        s.fans[i].valid = false;
        s.fans[i].address = 0;
        s.fans[i].name[0] = '\0';
    }
}

// Manages load/save of Settings via ESP32 NVS (Preferences API).
class SettingsManager {
public:
    Settings data;

    // Load from NVS; if missing or version mismatch, populate with defaults.
    bool begin() {
        _prefs.begin("minka", false);  // namespace "minka", read-write

        size_t sz = _prefs.getBytesLength(_key);
        if (sz != sizeof(Settings)) {
            Serial.printf("[SETTINGS] No saved data (got %u, expect %u). Using defaults.\n",
                          (unsigned)sz, (unsigned)sizeof(Settings));
            settingsDefaults(data);
            save();
            _firstBoot = true;
            return true;
        }

        _prefs.getBytes(_key, &data, sizeof(Settings));
        if (data.version != 1) {
            Serial.printf("[SETTINGS] Version mismatch (%d). Resetting.\n", data.version);
            settingsDefaults(data);
            save();
            _firstBoot = true;
            return true;
        }

        Serial.printf("[SETTINGS] Loaded: auto=%d manualSpd=%d unitF=%d numFans=%d\n",
                      data.autoMode, data.manualSpeed, data.useFahrenheit, data.numFans);
        _firstBoot = false;
        return true;
    }

    // Persist current settings to NVS
    void save() {
        _prefs.putBytes(_key, &data, sizeof(Settings));
    }

    // Wipe all settings, restore defaults, save
    void factoryReset() {
        _prefs.clear();
        settingsDefaults(data);
        save();
        Serial.println("[SETTINGS] Factory reset complete.");
    }

    bool isFirstBoot() { return _firstBoot; }

    // ---- Fan management ----

    // Returns -1 if no slot available
    int addFan(uint16_t address, const char* name = nullptr) {
        if (data.numFans >= MAX_FANS) return -1;
        // Check for duplicate
        for (int i = 0; i < data.numFans; i++) {
            if (data.fans[i].valid && data.fans[i].address == address) return i;
        }
        int slot = data.numFans;
        data.fans[slot].valid = true;
        data.fans[slot].address = address;
        if (name) {
            strncpy(data.fans[slot].name, name, sizeof(data.fans[slot].name) - 1);
            data.fans[slot].name[sizeof(data.fans[slot].name) - 1] = '\0';
        } else {
            snprintf(data.fans[slot].name, sizeof(data.fans[slot].name), "Fan %d", slot + 1);
        }
        data.numFans++;
        save();
        return slot;
    }

    void removeFan(uint8_t slot) {
        if (slot >= data.numFans) return;
        // Shift remaining fans down
        for (int i = slot; i < data.numFans - 1; i++) {
            data.fans[i] = data.fans[i + 1];
        }
        data.numFans--;
        data.fans[data.numFans].valid = false;
        save();
    }

    // Build the full 13-bit code for a given fan + speed
    uint16_t fanCode(uint8_t slot, FanSpeed speed) {
        if (slot >= data.numFans || !data.fans[slot].valid) return 0;
        return ((uint16_t)data.fans[slot].address << 4) | SPEED_CMD_BITS[(uint8_t)speed];
    }

    // ---- Temperature unit conversions ----

    float displayTemp(float celsius) {
        return data.useFahrenheit ? (celsius * 9.0f / 5.0f + 32.0f) : celsius;
    }

    const char* tempUnitLabel() {
        return data.useFahrenheit ? "F" : "C";
    }

    // ---- Auto-control logic with hysteresis ----

    // Given current temperature in Celsius, return what fan speed auto mode should set.
    // Hysteresis: must drop hysteresisC degrees BELOW the engage threshold to step down.
    FanSpeed autoSpeedForTemp(float currentC, FanSpeed currentSpeed) {
        // Stepping UP: any time we cross the engage threshold
        // Stepping DOWN: must drop below (threshold - hysteresis)
        int8_t hL = data.thresholdLow_C;
        int8_t hM = data.thresholdMed_C;
        int8_t hH = data.thresholdHigh_C;
        int8_t hys = data.hysteresisC;

        // Going up
        if (currentC >= hH) return SPEED_HIGH;
        if (currentC >= hM && currentSpeed < SPEED_MED) return SPEED_MED;
        if (currentC >= hL && currentSpeed < SPEED_LOW) return SPEED_LOW;

        // Going down with hysteresis
        if (currentSpeed == SPEED_HIGH && currentC < hH - hys) {
            // Drop only one step
            if (currentC >= hM) return SPEED_MED;
            if (currentC >= hL) return SPEED_LOW;
            return SPEED_OFF;
        }
        if (currentSpeed == SPEED_MED && currentC < hM - hys) {
            if (currentC >= hL) return SPEED_LOW;
            return SPEED_OFF;
        }
        if (currentSpeed == SPEED_LOW && currentC < hL - hys) {
            return SPEED_OFF;
        }

        return currentSpeed;  // no change
    }

private:
    Preferences _prefs;
    static constexpr const char* _key = "settings";
    bool _firstBoot = false;
};
