#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "pins.h"

// MXC6655XA 3-axis accelerometer + temperature sensor (MEMSIC)
// I2C address: 0x15

// Register map
#define MXC6655_REG_XOUT_U   0x03  // X-axis upper byte
#define MXC6655_REG_XOUT_L   0x04  // X-axis lower byte
#define MXC6655_REG_YOUT_U   0x05
#define MXC6655_REG_YOUT_L   0x06
#define MXC6655_REG_ZOUT_U   0x07
#define MXC6655_REG_ZOUT_L   0x08
#define MXC6655_REG_TOUT     0x09  // Temperature, 8-bit signed
#define MXC6655_REG_STATUS   0x02
#define MXC6655_REG_CONTROL  0x0D
#define MXC6655_REG_DEVICE_ID 0x0F

// Temperature: ~0.586°C/LSB, offset 0x00 corresponds to ~25°C
// (per MEMSIC AN-00MX-008)
#define MXC6655_TEMP_LSB_C   0.586f
#define MXC6655_TEMP_OFFSET_C 25.0f

// Accelerometer: ±2g range by default, 12-bit data (left-justified in 16 bits)
// LSB = 2g / 2048 = 0.000977 g/LSB (≈ 1 mg/LSB)
#define MXC6655_ACCEL_G_PER_LSB (2.0f / 2048.0f)

class MXC6655 {
public:
    // Manually bit-bang an I2C STOP condition to reset any slave stuck in the
    // middle of a transaction. SDA rises while SCL is high = STOP.
    void bitbangStop() {
        pinMode(PIN_I2C_SCL, OUTPUT);
        pinMode(PIN_I2C_SDA, OUTPUT);
        digitalWrite(PIN_I2C_SDA, LOW);
        digitalWrite(PIN_I2C_SCL, LOW);
        delayMicroseconds(10);
        digitalWrite(PIN_I2C_SCL, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_I2C_SDA, HIGH);  // STOP: SDA rising while SCL high
        delayMicroseconds(10);
        pinMode(PIN_I2C_SCL, INPUT);
        pinMode(PIN_I2C_SDA, INPUT);
    }

    // Clock SCL 9 times with SDA high to complete any pending byte transmission
    // and allow a slave to release the bus. Then issue STOP.
    void recoverBus() {
        Serial.println("[I2C] Bus recovery: clocking SCL 9x + STOP");
        pinMode(PIN_I2C_SDA, INPUT_PULLUP);   // SDA let float high
        pinMode(PIN_I2C_SCL, OUTPUT);
        for (int i = 0; i < 9; i++) {
            digitalWrite(PIN_I2C_SCL, HIGH);
            delayMicroseconds(10);
            digitalWrite(PIN_I2C_SCL, LOW);
            delayMicroseconds(10);
        }
        digitalWrite(PIN_I2C_SCL, HIGH);
        delayMicroseconds(10);
        bitbangStop();
        delay(5);
    }

    bool begin() {
        // Give the sensor a moment to stabilise after power-on (datasheet ~50ms)
        delay(30);

        // Check pin states before Wire takes over. If SDA is stuck low, clock
        // SCL to release any slave mid-transaction from a prior session.
        pinMode(PIN_I2C_SDA, INPUT);
        pinMode(PIN_I2C_SCL, INPUT);
        delay(5);
        if (digitalRead(PIN_I2C_SDA) == 0) {
            Serial.println("[I2C] SDA stuck low — recovering bus");
            recoverBus();
        }

        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
        Wire.setClock(100000);  // 100 kHz — conservative and reliable

        // Probe the sensor
        Wire.beginTransmission(MXC6655_ADDR);
        uint8_t err = Wire.endTransmission();
        if (err != 0) {
            Serial.printf("[MXC6655] Not detected at 0x%02X (err=%d)\n", MXC6655_ADDR, err);
            _present = false;
            return false;
        }

        _addr = MXC6655_ADDR;
        _devId = readReg(MXC6655_REG_DEVICE_ID);
        uint8_t status = readReg(MXC6655_REG_STATUS);
        Serial.printf("[MXC6655] OK: DEVICE_ID=0x%02X STATUS=0x%02X\n", _devId, status);
        _present = true;
        return true;
    }

    // Public: allow main loop to retry sensor detection after boot (e.g., user
    // pressed reset on sensor board, or power glitch)
    bool retry() {
        Serial.println("[MXC6655] Retry requested");
        return begin();
    }

    bool isPresent() { return _present; }
    uint8_t deviceId() { return _devId; }

    // Read temperature in degrees Celsius. Returns NAN if sensor not present.
    float readTemperatureC() {
        if (!_present) return NAN;
        int8_t raw = (int8_t)readReg(MXC6655_REG_TOUT);
        return MXC6655_TEMP_OFFSET_C + raw * MXC6655_TEMP_LSB_C;
    }

    // Raw temperature byte for diagnostics
    int8_t readTemperatureRaw() {
        return (int8_t)readReg(MXC6655_REG_TOUT);
    }

    // Read 3-axis acceleration in g. Returns true on success.
    bool readAcceleration(float& x, float& y, float& z) {
        uint8_t buf[6];
        if (!readBurst(MXC6655_REG_XOUT_U, buf, 6)) return false;

        // Data is left-justified 12-bit: upper 8 bits in U reg, top 4 bits of L reg
        // Assemble as signed 16-bit then shift right by 4
        int16_t rx = ((int16_t)buf[0] << 8 | buf[1]) >> 4;
        int16_t ry = ((int16_t)buf[2] << 8 | buf[3]) >> 4;
        int16_t rz = ((int16_t)buf[4] << 8 | buf[5]) >> 4;

        // Sign extend 12-bit values
        if (rx & 0x0800) rx |= 0xF000;
        if (ry & 0x0800) ry |= 0xF000;
        if (rz & 0x0800) rz |= 0xF000;

        x = rx * MXC6655_ACCEL_G_PER_LSB;
        y = ry * MXC6655_ACCEL_G_PER_LSB;
        z = rz * MXC6655_ACCEL_G_PER_LSB;
        return true;
    }

    // Determine screen orientation from gravity vector.
    // Returns: 0=portrait up, 1=landscape right, 2=portrait down, 3=landscape left
    uint8_t readOrientation() {
        float x, y, z;
        if (!readAcceleration(x, y, z)) return 0;
        if (fabs(x) > fabs(y)) {
            return (x > 0) ? 1 : 3;
        } else {
            return (y > 0) ? 0 : 2;
        }
    }

    uint8_t address() { return _addr; }

private:
    bool _present = false;
    uint8_t _devId = 0;
    uint8_t _addr = MXC6655_ADDR;

    // Write + STOP, then read (matches working MRAA reference, no repeated start)
    uint8_t readReg(uint8_t reg) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        if (Wire.endTransmission(true) != 0) return 0xFF;  // true = send STOP
        if (Wire.requestFrom(_addr, (uint8_t)1) != 1) return 0xFF;
        return Wire.read();
    }

    bool readBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        if (Wire.endTransmission(true) != 0) return false;  // STOP between write & read
        if (Wire.requestFrom(_addr, len) != len) return false;
        for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
        return true;
    }
};
