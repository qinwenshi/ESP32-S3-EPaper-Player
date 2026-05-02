#pragma once
// ES8311 audio codec initialization over I2C.
// Board: Waveshare ESP32-S3-ePaper-1.54 (V2)
// Register map based on official Espressif esp_codec_dev ES8311 driver.
// Config: slave mode, I2S standard, 16-bit, MCLK=256×fs from ESP32 pin IO14.

#include <Wire.h>

#define ES8311_I2C_ADDR 0x18

class ES8311 {
public:
    ES8311() {}

    bool begin(int sda, int scl) {
        Wire.begin(sda, scl);
        delay(10);
        return init();
    }

    // Call after audio starts if sample rate changes (44100 or 48000 Hz typical)
    void setSampleRate(uint32_t rate) { configSampleRate(rate); }

private:
    void writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(ES8311_I2C_ADDR);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    uint8_t readReg(uint8_t reg) {
        Wire.beginTransmission(ES8311_I2C_ADDR);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)ES8311_I2C_ADDR, (uint8_t)1);
        return Wire.available() ? Wire.read() : 0;
    }

    // Clock divider table: {mclk, rate, pre_div, pre_multi, adc_div, dac_div,
    //                        fs_mode, lrck_h, lrck_l, bclk_div, adc_osr, dac_osr}
    struct CoeffDiv {
        uint32_t mclk, rate;
        uint8_t pre_div, pre_multi, adc_div, dac_div, fs_mode,
                lrck_h, lrck_l, bclk_div, adc_osr, dac_osr;
    };
    static constexpr CoeffDiv coeff_table[] = {
        {11289600, 44100, 1,1,1,1,0,0x00,0xff,0x04,0x10,0x10},
        {12288000, 48000, 1,1,1,1,0,0x00,0xff,0x04,0x10,0x10},
        {12288000, 44100, 4,1,1,1,0,0x00,0xff,0x04,0x10,0x10}, // fallback
        {12288000, 32000, 3,2,1,1,0,0x00,0xff,0x04,0x10,0x10},
        {12288000, 16000, 3,1,1,1,0,0x00,0xff,0x04,0x10,0x20},
        {12288000,  8000, 6,1,1,1,0,0x00,0xff,0x04,0x10,0x20},
    };

    int findCoeff(uint32_t rate) {
        uint32_t mclk = rate * 256;
        for (int i = 0; i < (int)(sizeof(coeff_table)/sizeof(coeff_table[0])); i++) {
            if (coeff_table[i].rate == rate && coeff_table[i].mclk == mclk) return i;
        }
        // fallback: any matching rate
        for (int i = 0; i < (int)(sizeof(coeff_table)/sizeof(coeff_table[0])); i++) {
            if (coeff_table[i].rate == rate) return i;
        }
        return 0; // default to first entry
    }

    void configSampleRate(uint32_t rate) {
        int idx = findCoeff(rate);
        const CoeffDiv& c = coeff_table[idx];

        uint8_t reg02 = readReg(0x02) & 0x07;
        uint8_t pre_multi_bits = (c.pre_multi == 2) ? 1 : (c.pre_multi == 4) ? 2 : (c.pre_multi == 8) ? 3 : 0;
        reg02 |= (uint8_t)((c.pre_div - 1) << 5) | (pre_multi_bits << 3);
        writeReg(0x02, reg02);

        writeReg(0x05, (uint8_t)(((c.adc_div-1) << 4) | (c.dac_div-1)));

        uint8_t reg03 = readReg(0x03) & 0x80;
        reg03 |= (c.fs_mode << 6) | c.adc_osr;
        writeReg(0x03, reg03);

        uint8_t reg04 = readReg(0x04) & 0x80;
        reg04 |= c.dac_osr;
        writeReg(0x04, reg04);

        uint8_t reg07 = readReg(0x07) & 0xC0;
        reg07 |= c.lrck_h;
        writeReg(0x07, reg07);
        writeReg(0x08, c.lrck_l);

        // BCLK divider (slave mode: ES8311 receives BCLK from ESP32, this is input mode)
        uint8_t reg06 = readReg(0x06) & ~0x1F;
        reg06 |= (c.bclk_div < 19) ? (c.bclk_div - 1) : c.bclk_div;
        writeReg(0x06, reg06);
    }

    bool init() {
        // Noise immunity — write twice per official driver
        writeReg(0x44, 0x08);
        writeReg(0x44, 0x08);

        // Initial clock setup
        writeReg(0x01, 0x30);
        writeReg(0x02, 0x00);
        writeReg(0x03, 0x10);
        writeReg(0x16, 0x24);
        writeReg(0x04, 0x10);
        writeReg(0x05, 0x00);

        // System power
        writeReg(0x0B, 0x00);
        writeReg(0x0C, 0x00);
        writeReg(0x10, 0x1F);
        writeReg(0x11, 0x7F);

        // Soft reset + enable (slave mode: bit6=0)
        writeReg(0x00, 0x80);
        delay(5);

        // MCLK from external pin (IO14), not inverted
        writeReg(0x01, 0x3F);

        // SCLK not inverted
        uint8_t reg06 = readReg(0x06) & ~0x20;
        writeReg(0x06, reg06);

        writeReg(0x13, 0x10);
        writeReg(0x1B, 0x0A);
        writeReg(0x1C, 0x6A);

        // Internal reference (ADCL + DACR)
        writeReg(0x44, 0x58);

        // Configure clocks for default 44100 Hz
        configSampleRate(44100);

        // ── Serial digital port: 16-bit I2S standard ──────────────────────
        // REG09 (DAC SDP): bits[4:2]=0b011 for 16-bit, bits[1:0]=0 for I2S standard
        writeReg(0x09, 0x0C);
        writeReg(0x0A, 0x0C);

        // ── Enable DAC/ADC ─────────────────────────────────────────────────
        writeReg(0x17, 0xBF); // ADC digital volume
        writeReg(0x0E, 0x02); // Analog power up
        writeReg(0x12, 0x00); // Enable DAC
        writeReg(0x14, 0x1A); // Analog PGA: MIC input, no DMIC
        writeReg(0x0D, 0x01); // Power reference
        writeReg(0x15, 0x40); // ADC ramp rate
        writeReg(0x37, 0x08); // DAC ramp rate
        writeReg(0x45, 0x00); // GP control
        writeReg(0x32, 0xBF); // DAC digital volume = 0 dB

        return true;
    }
};
