#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "pins.h"

// CC1101 strobe commands
#define CC1101_SRES    0x30
#define CC1101_SRX     0x34
#define CC1101_STX     0x35
#define CC1101_SIDLE   0x36

// CC1101 configuration registers
#define CC1101_IOCFG2   0x00
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
#define CC1101_PKTLEN   0x06
#define CC1101_PKTCTRL1 0x07
#define CC1101_PKTCTRL0 0x08
#define CC1101_FSCTRL1  0x0B
#define CC1101_FREQ2    0x0D
#define CC1101_FREQ1    0x0E
#define CC1101_FREQ0    0x0F
#define CC1101_MDMCFG4  0x10
#define CC1101_MDMCFG3  0x11
#define CC1101_MDMCFG2  0x12
#define CC1101_MDMCFG1  0x13
#define CC1101_MDMCFG0  0x14
#define CC1101_DEVIATN  0x15
#define CC1101_MCSM0    0x18
#define CC1101_FOCCFG   0x19
#define CC1101_AGCCTRL2 0x1B
#define CC1101_AGCCTRL1 0x1C
#define CC1101_AGCCTRL0 0x1D
#define CC1101_FREND1   0x21
#define CC1101_FREND0   0x22
#define CC1101_FSCAL3   0x23
#define CC1101_FSCAL2   0x24
#define CC1101_FSCAL1   0x25
#define CC1101_FSCAL0   0x26
#define CC1101_TEST2    0x2C
#define CC1101_TEST1    0x2D
#define CC1101_TEST0    0x2E
#define CC1101_PATABLE  0x3E

// Status registers (read with burst bit set)
#define CC1101_PARTNUM    0x30
#define CC1101_VERSION    0x31
#define CC1101_RSSI       0x34
#define CC1101_MARCSTATE  0x35
#define CC1101_PKTSTATUS  0x38

class CC1101Radio {
public:
    bool begin() {
        pinMode(PIN_CC1101_CS, OUTPUT);
        digitalWrite(PIN_CC1101_CS, HIGH);

        // Reset
        digitalWrite(PIN_CC1101_CS, HIGH);
        delayMicroseconds(30);
        digitalWrite(PIN_CC1101_CS, LOW);
        delayMicroseconds(30);
        digitalWrite(PIN_CC1101_CS, HIGH);
        delayMicroseconds(45);
        strobe(CC1101_SRES);
        delay(10);

        // Verify chip presence
        uint8_t partnum = readStatusReg(CC1101_PARTNUM);
        uint8_t version = readStatusReg(CC1101_VERSION);
        if (partnum != 0x00 || (version != 0x14 && version != 0x04)) {
            return false;
        }
        _version = version;
        return true;
    }

    // Configure for OOK TX at a given frequency.
    // ASK/OOK modulation so carrier turns on/off with GDO0.
    // In async serial TX mode, CC1101 reads data from GDO0 pin.
    void configureOOK(float mhz) {
        // Force IDLE and wait for confirmation before changing frequency
        strobe(CC1101_SIDLE);
        unsigned long idleStart = millis();
        while ((readStatusReg(CC1101_MARCSTATE) & 0x1F) != 0x01) {
            if (millis() - idleStart > 100) {
                Serial.println("[CC1101] WARNING: timeout waiting for IDLE");
                break;
            }
        }

        // Frequency
        uint32_t freqWord = (uint32_t)(mhz * 65536.0 / 26.0);
        writeReg(CC1101_FREQ2, (freqWord >> 16) & 0xFF);
        writeReg(CC1101_FREQ1, (freqWord >> 8) & 0xFF);
        writeReg(CC1101_FREQ0, freqWord & 0xFF);

        // GDO0 = high impedance (tri-state) so ESP32 can drive it for TX
        writeReg(CC1101_IOCFG0, 0x2E);
        // GDO2 = carrier sense (diagnostics)
        writeReg(CC1101_IOCFG2, 0x0E);

        // ASK/OOK modulation, no sync word — carrier on when data=1, off when data=0
        writeReg(CC1101_MDMCFG2, 0x30);  // ASK/OOK, no sync
        writeReg(CC1101_MDMCFG1, 0x22);
        writeReg(CC1101_MDMCFG0, 0xF8);

        // Data rate and RX BW (data rate affects TX clock in async mode)
        writeReg(CC1101_MDMCFG4, 0x17);  // Wide RX BW
        writeReg(CC1101_MDMCFG3, 0x83);

        writeReg(CC1101_FSCTRL1, 0x06);
        writeReg(CC1101_DEVIATN, 0x47);

        // Async serial mode: GDO0 is data input for TX, data output for RX
        writeReg(CC1101_PKTCTRL0, 0x30);  // Async serial
        writeReg(CC1101_PKTCTRL1, 0x00);
        writeReg(CC1101_PKTLEN,   0x00);

        // AGC settings
        writeReg(CC1101_AGCCTRL2, 0x07);
        writeReg(CC1101_AGCCTRL1, 0x00);
        writeReg(CC1101_AGCCTRL0, 0xB2); // 3-of-4, medium hysteresis

        // Frontend
        writeReg(CC1101_FREND1, 0x56);
        writeReg(CC1101_FREND0, 0x11);   // PA table index 1 for TX

        // Auto-calibrate on idle-to-rx/tx
        writeReg(CC1101_MCSM0, 0x18);
        writeReg(CC1101_FOCCFG, 0x16);

        // Frequency synthesizer calibration
        writeReg(CC1101_FSCAL3, 0xE9);
        writeReg(CC1101_FSCAL2, 0x2A);
        writeReg(CC1101_FSCAL1, 0x00);
        writeReg(CC1101_FSCAL0, 0x1F);

        writeReg(CC1101_TEST2, 0x81);
        writeReg(CC1101_TEST1, 0x35);
        // TEST0 = 0x0B enables PLL lock below 348 MHz (VCO_SEL_CAL_EN).
        // Without this, the VCO bottoms out near 310 MHz and refuses to go lower.
        writeReg(CC1101_TEST0, 0x0B);

        // PA table: index 0 = off, index 1 = max power
        uint8_t paTable[8] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        writeBurst(CC1101_PATABLE, paTable, 8);

        // Force a fresh frequency synthesizer calibration at the new frequency.
        // SCAL strobe calibrates the synth in IDLE without entering RX/TX.
        strobe(0x33);  // SCAL
        unsigned long calStart = millis();
        while ((readStatusReg(CC1101_MARCSTATE) & 0x1F) != 0x01) {
            if (millis() - calStart > 100) {
                Serial.println("[CC1101] WARNING: SCAL timeout");
                break;
            }
        }

        // Readback actual FSCAL values (tells us if VCO locked at this freq)
        uint8_t fscal3 = readConfigReg(CC1101_FSCAL3);
        uint8_t fscal2 = readConfigReg(CC1101_FSCAL2);
        uint8_t fscal1 = readConfigReg(CC1101_FSCAL1);
        uint8_t fscal0 = readConfigReg(CC1101_FSCAL0);

        uint32_t fw = (uint32_t)(mhz * 65536.0 / 26.0);
        Serial.printf("[CC1101] OOK @ %.3f MHz: fw=0x%06lX, FSCAL3/2/1/0=%02X/%02X/%02X/%02X\n",
                      mhz, fw, fscal3, fscal2, fscal1, fscal0);
    }

    void setRx() {
        strobe(CC1101_SIDLE);
        strobe(CC1101_SRX);
    }

    void setTx() {
        strobe(CC1101_SIDLE);
        strobe(CC1101_STX);
    }

    void setIdle() {
        strobe(CC1101_SIDLE);
    }

    uint8_t version() { return _version; }

    // Read RSSI in dBm
    int rssi() {
        uint8_t raw = readStatusReg(CC1101_RSSI);
        int rssi_dBm;
        if (raw >= 128) rssi_dBm = (int)(raw - 256) / 2 - 74;
        else            rssi_dBm = (int)(raw) / 2 - 74;
        return rssi_dBm;
    }

    // Read MARCSTATE (current radio state machine state)
    uint8_t marcState() {
        return readStatusReg(CC1101_MARCSTATE) & 0x1F;
    }

    // Read PKTSTATUS register (carrier sense bit, etc.)
    uint8_t pktStatus() {
        return readStatusReg(CC1101_PKTSTATUS);
    }

    // Read back a config register to verify it was written
    uint8_t readConfigReg(uint8_t addr) {
        return readReg(addr, 0x80);  // read single byte
    }

    // Dump key registers to serial for diagnostics
    void dumpRegisters() {
        Serial.println("[CC1101] Register dump:");
        Serial.printf("  IOCFG2   = 0x%02X\n", readConfigReg(CC1101_IOCFG2));
        Serial.printf("  IOCFG0   = 0x%02X\n", readConfigReg(CC1101_IOCFG0));
        Serial.printf("  PKTCTRL0 = 0x%02X\n", readConfigReg(CC1101_PKTCTRL0));
        Serial.printf("  PKTCTRL1 = 0x%02X\n", readConfigReg(CC1101_PKTCTRL1));
        Serial.printf("  FREQ2    = 0x%02X\n", readConfigReg(CC1101_FREQ2));
        Serial.printf("  FREQ1    = 0x%02X\n", readConfigReg(CC1101_FREQ1));
        Serial.printf("  FREQ0    = 0x%02X\n", readConfigReg(CC1101_FREQ0));
        Serial.printf("  MDMCFG4  = 0x%02X\n", readConfigReg(CC1101_MDMCFG4));
        Serial.printf("  MDMCFG3  = 0x%02X\n", readConfigReg(CC1101_MDMCFG3));
        Serial.printf("  MDMCFG2  = 0x%02X\n", readConfigReg(CC1101_MDMCFG2));
        Serial.printf("  MDMCFG1  = 0x%02X\n", readConfigReg(CC1101_MDMCFG1));
        Serial.printf("  AGCCTRL2 = 0x%02X\n", readConfigReg(CC1101_AGCCTRL2));
        Serial.printf("  AGCCTRL1 = 0x%02X\n", readConfigReg(CC1101_AGCCTRL1));
        Serial.printf("  AGCCTRL0 = 0x%02X\n", readConfigReg(CC1101_AGCCTRL0));
        Serial.printf("  FREND0   = 0x%02X\n", readConfigReg(CC1101_FREND0));
        Serial.printf("  MCSM0    = 0x%02X\n", readConfigReg(CC1101_MCSM0));
        Serial.printf("  MARCSTATE= 0x%02X\n", marcState());
        Serial.printf("  RSSI     = %d dBm\n", rssi());
        Serial.printf("  PKTSTATUS= 0x%02X\n", pktStatus());
    }

private:
    uint8_t _version = 0;

    // Per CC1101 datasheet section 10.1: after CS goes low,
    // wait for SO (MISO) to go low before sending header byte.
    void waitChipReady() {
        unsigned long start = millis();
        while (digitalRead(PIN_SPI_MISO)) {
            if (millis() - start > 100) {
                Serial.println("[CC1101] WARNING: chip not ready (MISO stuck high)");
                break;
            }
        }
    }

    void spiBegin() {
        // Deselect TFT to release the shared SPI bus
        digitalWrite(PIN_TFT_CS, HIGH);
        // Fully reclaim the bus: end any prior transaction, then restart
        // with our settings. TFT_eSPI can leave the peripheral in a
        // state that beginTransaction alone doesn't reset on ESP32.
        SPI.end();
        SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
        SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
        digitalWrite(PIN_CC1101_CS, LOW);
        waitChipReady();
    }

    void spiEnd() {
        digitalWrite(PIN_CC1101_CS, HIGH);
        SPI.endTransaction();
    }

    void strobe(uint8_t cmd) {
        spiBegin();
        SPI.transfer(cmd);
        spiEnd();
    }

    void writeReg(uint8_t addr, uint8_t val) {
        spiBegin();
        SPI.transfer(addr);
        SPI.transfer(val);
        spiEnd();
    }

    void writeBurst(uint8_t addr, uint8_t* data, uint8_t len) {
        spiBegin();
        SPI.transfer(addr | 0x40);  // burst write
        for (uint8_t i = 0; i < len; i++) {
            SPI.transfer(data[i]);
        }
        spiEnd();
    }

    uint8_t readReg(uint8_t addr, uint8_t mode) {
        spiBegin();
        SPI.transfer(addr | mode);
        uint8_t val = SPI.transfer(0x00);
        spiEnd();
        return val;
    }

    uint8_t readStatusReg(uint8_t addr) {
        return readReg(addr, 0xC0);  // read + burst/status
    }
};
