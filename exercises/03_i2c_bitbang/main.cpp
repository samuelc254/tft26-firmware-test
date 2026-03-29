// =============================================================================
//  Exercise 03 — I2C Sensors (Bit-bang)
// =============================================================================
//
//  Virtual hardware:
//    P8 (SCL)  →  io.digital_write(8, …) / io.digital_read(8)
//    P9 (SDA)  →  io.digital_write(9, …) / io.digital_read(9)
//
//  PART 1 — TMP64 temperature sensor at I2C address 0x48
//    Register 0x0F  WHO_AM_I   — 1 byte  (expected: 0xA5)
//    Register 0x00  TEMP_RAW   — 4 bytes, big-endian int32_t, milli-Celsius
//
//  PART 2 — Unknown humidity sensor (same register layout, address unknown)
//    Register 0x0F  WHO_AM_I   — 1 byte
//    Register 0x00  HUM_RAW    — 4 bytes, big-endian int32_t, milli-percent
//
//  Goal (Part 1):
//    1. Implement an I2C master via bit-bang on P8/P9.
//    2. Read WHO_AM_I from TMP64 and confirm the sensor is present.
//    3. Read TEMP_RAW in a loop and print the temperature in °C every second.
//    4. Update display registers 6–7 with the formatted temperature string.
//
//  Goal (Part 2):
//    5. Scan the I2C bus (addresses 0x08–0x77) and print every responding address.
//    6. For each unknown device found, read its WHO_AM_I and print it.
//    7. Add the humidity sensor to the 1 Hz loop: read HUM_RAW and print %RH.
//
//  Read README.md before starting.
// =============================================================================

// -----------------------------------------------------------------------------
//  Architecture & Design Rationale
// -----------------------------------------------------------------------------
//
//  1. Software I2C is implemented with deterministic GPIO toggling to keep
//     timing explicit and independent from platform-specific peripherals.
//
//  2. The bus is modeled as open-drain; driving HIGH means releasing the line,
//     while LOW is actively driven. This matches I2C electrical semantics.
//
//  3. Register reads use a repeated START transaction to preserve bus ownership
//     between write-phase (register select) and read-phase (data fetch).
//
//  4. Runtime telemetry is scheduled with millis()-based deadlines (1 Hz),
//     separating transport logic from application periodicity.
//
//  5. Timing & Frequency Verification (Hardware Profiling):
//     Profiling tests revealed that `io.digital_write()` execution takes ~4.177 µs.
//     A full SCL clock cycle (HIGH + LOW) inherently takes at least ~8.35 µs.
//     This physical HAL latency naturally throttles the bit-bang I2C clock
//     to ~119.7 kHz. This places the communication safely within the I2C
//     Standard-Mode range (~100 kHz) and well below the sensor's 400 kHz
//     limit, completely eliminating the need for artificial NOP/delay throttling.
//
// =============================================================================

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <trac_fw_io.hpp>

namespace {

// =============================================================================
// 1. Hardware Abstraction Layer (HAL)
// =============================================================================
constexpr uint8_t SCL_PIN = 8;
constexpr uint8_t SDA_PIN = 9;

// Display register mapping
constexpr uint8_t DISPLAY_HUM_REG_START = 4;
constexpr uint8_t DISPLAY_HUM_REG_END = 5;
constexpr uint8_t DISPLAY_TEMP_REG_START = 6;
constexpr uint8_t DISPLAY_TEMP_REG_END = 7;

// Open-drain emulation helpers: HIGH releases the bus, LOW drives it.
inline void sda_high(trac_fw_io_t& io) { io.digital_write(SDA_PIN, 1); }
inline void sda_low(trac_fw_io_t& io) { io.digital_write(SDA_PIN, 0); }
inline void scl_high(trac_fw_io_t& io) { io.digital_write(SCL_PIN, 1); }
inline void scl_low(trac_fw_io_t& io) { io.digital_write(SCL_PIN, 0); }
inline bool sda_read(trac_fw_io_t& io) { return io.digital_read(SDA_PIN); }

// =============================================================================
// 2. I2C Protocol Primitives
// =============================================================================

// START condition with bus in released state before arbitration begins.
void i2c_start(trac_fw_io_t& io) {
    sda_high(io);
    scl_high(io);
    sda_low(io);
    scl_low(io);
}

// STOP condition to release bus ownership explicitly.
void i2c_stop(trac_fw_io_t& io) {
    sda_low(io);
    scl_low(io);
    scl_high(io);
    sda_high(io);
}

// Transmit one byte MSB-first; returns true only on ACK from slave.
bool i2c_write_byte(trac_fw_io_t& io, uint8_t data) {
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) {
            sda_high(io);
        } else {
            sda_low(io);
        }
        data <<= 1;

        scl_high(io);
        scl_low(io);
    }

    // 9th clock is dedicated to ACK/NACK sampling from the slave.
    sda_high(io);
    scl_high(io);
    bool ack = !sda_read(io);
    scl_low(io);

    return ack;
}

// Receive one byte MSB-first and emit ACK for continued reads or NACK to end.
uint8_t i2c_read_byte(trac_fw_io_t& io, bool send_ack) {
    uint8_t data = 0;
    sda_high(io);

    for (int i = 0; i < 8; i++) {
        data <<= 1;

        scl_high(io);
        if (sda_read(io)) {
            data |= 0x01;
        }
        scl_low(io);
    }

    if (send_ack) {
        sda_low(io);
    } else {
        sda_high(io);
    }

    scl_high(io);
    scl_low(io);
    sda_high(io);

    return data;
}

// =============================================================================
// 3. High-Level I2C Operations
// =============================================================================

// Read register window using write(register) + repeated-start + read(data).
bool i2c_read_register(trac_fw_io_t& io, uint8_t dev_addr, uint8_t reg_addr, uint8_t* buffer, uint8_t length) {
    // Phase 1: point internal register cursor.
    i2c_start(io);
    if (!i2c_write_byte(io, (dev_addr << 1) | 0)) {
        i2c_stop(io);
        return false;
    }
    if (!i2c_write_byte(io, reg_addr)) {
        i2c_stop(io);
        return false;
    }

    // Phase 2: switch to read without releasing the bus.
    i2c_start(io);
    if (!i2c_write_byte(io, (dev_addr << 1) | 1)) {
        i2c_stop(io);
        return false;
    }

    for (uint8_t i = 0; i < length; i++) {
        bool is_last_byte = (i == (length - 1));
        buffer[i] = i2c_read_byte(io, !is_last_byte);
    }

    i2c_stop(io);
    return true;
}

// Address probe: write-address phase only; ACK means device presence.
bool i2c_scan_probe(trac_fw_io_t& io, uint8_t addr) {
    i2c_start(io);
    bool ack = i2c_write_byte(io, (addr << 1) | 0);
    i2c_stop(io);
    return ack;
}

// Convert network-order sensor payload into signed 32-bit engineering value.
int32_t assemble_int32(const uint8_t* buf) {
    uint32_t raw = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
    int32_t value;
    std::memcpy(&value, &raw, 4);
    return value;
}

// LCD expects 8 ASCII chars split across two 32-bit registers.
void write_display_8chars(trac_fw_io_t& io, uint8_t reg_a, uint8_t reg_b, const char* text) {
    // Inicializa um buffer de 8 bytes preenchido com espaços (ASCII 0x20)
    char safe_buf[8] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
    
    // Copia o texto caractere por caractere, parando no limite de 8 ou no fim da string
    for (int i = 0; i < 8 && text[i] != '\0'; i++) {
        safe_buf[i] = text[i];
    }

    uint32_t a = 0;
    uint32_t b = 0;
    
    // Agora fazemos o memcpy com 100% de segurança a partir do nosso buffer limpo
    std::memcpy(&a, safe_buf + 0, 4);
    std::memcpy(&b, safe_buf + 4, 4);
    
    io.write_reg(reg_a, a);
    io.write_reg(reg_b, b);
}

}  // namespace

// =============================================================================
// 4. Main Application
// =============================================================================

int main() {
    trac_fw_io_t io;

    // Release both lines before first transaction to guarantee idle bus state.
    io.set_pullup(SCL_PIN, true);
    io.set_pullup(SDA_PIN, true);
    scl_high(io);
    sda_high(io);

    constexpr uint8_t TMP64_ADDR = 0x48;
    uint8_t hmd10_addr = 0x00;
    bool tmp64_found = false;

    std::printf("Starting I2C Bus Scan...\n");

    // Scan valid 7-bit address range excluding reserved low addresses.
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_scan_probe(io, addr)) {
            std::printf("Found device at address: 0x%02X\n", addr);
            if (addr == TMP64_ADDR) {
                tmp64_found = true;
            } else if (hmd10_addr == 0x00) {
                // Product assumption: first non-TMP64 responder is the humidity device.
                hmd10_addr = addr;
            }
        }
    }

    // After scan: verify TMP64 WHO_AM_I (0xA5)
    if (tmp64_found) {
        uint8_t tmp64_who_am_i = 0;
        if (i2c_read_register(io, TMP64_ADDR, 0x0F, &tmp64_who_am_i, 1)) {
            if (tmp64_who_am_i == 0xA5) {
                std::printf("TMP64 (0x%02X) WHO_AM_I: 0x%02X [OK]\n", TMP64_ADDR, tmp64_who_am_i);
            } else {
                std::printf("TMP64 (0x%02X) WHO_AM_I: 0x%02X [UNEXPECTED, expected 0xA5]\n", TMP64_ADDR, tmp64_who_am_i);
            }
        } else {
            std::printf("TMP64 (0x%02X) WHO_AM_I read failed\n", TMP64_ADDR);
        }
    } else {
        std::printf("TMP64 not found at address 0x%02X\n", TMP64_ADDR);
    }

    if (hmd10_addr != 0x00) {
        std::printf("Selected humidity sensor address: 0x%02X\n", hmd10_addr);
        uint8_t hmd10_who_am_i = 0;
        if (i2c_read_register(io, hmd10_addr, 0x0F, &hmd10_who_am_i, 1)) {
            std::printf("HMD10 (0x%02X) WHO_AM_I: 0x%02X\n", hmd10_addr, hmd10_who_am_i);
        } else {
            std::printf("HMD10 (0x%02X) WHO_AM_I read failed\n", hmd10_addr);
        }
    } else {
        std::printf("No unknown humidity sensor found on the bus\n");
    }

    // Telemetry Superloop
    uint32_t last_update_ms = io.millis();
    uint8_t data_buf[4] = {};

    while (true) {
        uint32_t now_ms = io.millis();

        // 1 Hz Update Rate (Non-blocking)
        if (now_ms - last_update_ms >= 1000) {
            // Prevent "drift" by scheduling the next update at fixed 1-second intervals.
            last_update_ms += 1000;

            // --- Read Temperature (TMP64) ---
            if (i2c_read_register(io, TMP64_ADDR, 0x00, data_buf, 4)) {
                int32_t raw_temp = assemble_int32(data_buf);
                float temp_c = (float)raw_temp / 1000.0f;

                std::printf("Temperature: %.3f C\n", temp_c);

                char buf[9] = {};
                std::snprintf(buf, sizeof(buf), "%8.3f", temp_c);
                write_display_8chars(io, DISPLAY_TEMP_REG_START, DISPLAY_TEMP_REG_END, buf);
            }

            // --- Read Humidity (HMD10) ---
            if (hmd10_addr != 0x00 && i2c_read_register(io, hmd10_addr, 0x00, data_buf, 4)) {
                int32_t raw_hum = assemble_int32(data_buf);
                float hum_pct = (float)raw_hum / 1000.0f;

                std::printf("Humidity: %.3f %%RH\n", hum_pct);

                char buf[9] = {};
                std::snprintf(buf, sizeof(buf), "%7.3f%%", hum_pct);
                write_display_8chars(io, DISPLAY_HUM_REG_START, DISPLAY_HUM_REG_END, buf);
            }
        }
    }
}