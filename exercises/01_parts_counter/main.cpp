// =============================================================================
//  Exercise 01 — Parts Counter
// =============================================================================
//
//  Virtual hardware:
//    SW 0        →  io.digital_read(0)        Inductive sensor input
//    Display     →  io.write_reg(6, …)        LCD debug (see README for format)
//                   io.write_reg(7, …)
//
//  Goal:
//    Count every part that passes the sensor and show the total on the display.
//
//  Read README.md before starting.
// =============================================================================

// -----------------------------------------------------------------------------
//  Architecture & Design Rationale
// -----------------------------------------------------------------------------
//
//  1. Empirical Telemetry Analysis & FSM Debounce:
//     The debounce strategy was not arbitrarily guessed. Telemetry data from
//     the simulator (edge timestamps) was extracted and plotted via Python/Pandas.
//     The data revealed two critical physical behaviors:
//       a) A part can occlude the sensor for a highly variable amount of time,
//          sometimes passing extremely fast (pulses as short as ~1 ms).
//       b) However, there is always a consistent physical gap (LOW state)
//          between parts.
//
//     Based on this, an asymmetric Finite State Machine (FSM) was implemented:
//     - MIN_VALID_PULSE_MS (1 ms): Validates the rising edge, keeping the system
//       responsive enough to catch very fast-moving parts.
//     - MIN_LOW_STABLE_MS (120 ms): Acts as a strict "Delay OFF" filter. Once a
//       part is counted, the sensor must remain physically clear (LOW) for at
//       least 120 ms. This completely rejects any mechanical bounce or vibration
//       that might otherwise cause a single part to be double-counted.
//
//  2. Interrupt-Driven & Non-Blocking Design:
//     To comply with constraints against busy-waiting (e.g., delay()), the sensor
//     is monitored via a hardware interrupt (InterruptMode::CHANGE).
//     The ISR acts as an O(1) state machine, capturing timestamps (io.millis())
//     to measure pulse widths without blocking the CPU.
//
//  3. Thread Safety & Memory Model:
//     Because the part count is mutated in the ISR context and read in the main()
//     superloop, std::atomic<uint32_t> is used to prevent race conditions and
//     tearing. std::memory_order_relaxed is intentionally applied since we only
//     need atomicity for the increment itself, avoiding unnecessary memory barrier
//     overhead. State variables are declared static within the lambda to strictly
//     encapsulate the FSM and prevent global scope pollution.
//
//  4. I/O Optimization:
//     The main loop handles the display updates. To avoid bus saturation,
//     the display is only updated when the count changes, OR periodically
//     (DISPLAY_UPDATE_INTERVAL_MS = 500 ms) to keep the simulated connection
//     alive and responsive.
// =============================================================================

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <trac_fw_io.hpp>

namespace {
// -------------------------------------------------------------------------
// Physical Parameters & Configuration
// -------------------------------------------------------------------------

// Time windows validated via telemetry data:
// Minimum HIGH pulse width to accept a part.
// Kept small enough for close/fast parts while filtering spikes.
constexpr uint32_t MIN_VALID_PULSE_MS = 1;

// Minimum time sensor must remain LOW after a valid count
// to prevent double-counting noisy or vibrating parts.
constexpr uint32_t MIN_LOW_STABLE_MS = 120;

// Update display at most every defined interval to keep the connection
// responsive while avoiding excessive I/O bus overhead.
constexpr uint32_t DISPLAY_UPDATE_INTERVAL_MS = 500;

constexpr uint8_t SENSOR_PORT = 0;
constexpr uint8_t DISPLAY_REG_START = 6;
constexpr uint8_t DISPLAY_REG_END = 7;

// Formats and outputs the count to the LCD display via the defined register interface.
void update_display(trac_fw_io_t& io, uint32_t count) {
    char buf[9] = {};
    std::snprintf(buf, sizeof(buf), "%8u", count);

    uint32_t r1, r2;
    std::memcpy(&r1, buf + 0, 4);
    std::memcpy(&r2, buf + 4, 4);

    io.write_reg(DISPLAY_REG_START, r1);
    io.write_reg(DISPLAY_REG_END, r2);
}
}  // anonymous namespace

int main() {
    trac_fw_io_t io;

    // ensure the port doesn't float
    io.set_pullup(SENSOR_PORT, true);

    std::atomic<uint32_t> count{0};
    uint32_t last_count = 0;
    uint32_t last_display_update = 0;

    // -------------------------------------------------------------------------
    // ISR Configuration (Isolated and Thread-Safe FSM)
    // -------------------------------------------------------------------------
    io.attach_interrupt(SENSOR_PORT, [&io, &count]() {
        // Static variables retain state across ISR calls.
        // Encapsulated here to avoid global scope pollution.
        static bool in_pulse = false;
        static uint32_t pulse_start_ms = 0;
        static uint32_t last_falling_edge_ms = 0;
        static bool waiting_for_low_stable = false;

        const bool sensor_state = io.digital_read(SENSOR_PORT);
        const uint32_t now_ms = io.millis();

        if (sensor_state) {
            // Rising edge: sensor goes HIGH (part present)
            if (waiting_for_low_stable) {
                const uint32_t low_stable_time_ms = now_ms - last_falling_edge_ms;
                
                if (low_stable_time_ms < MIN_LOW_STABLE_MS) {
                    // HIGH arrived before LOW was stable long enough (noise/bounce).
                    // Reset low-stable timer to force a new LOW stable window.
                    last_falling_edge_ms = now_ms;
                    return;
                }
                
                // LOW was stable long enough, re-arm for the next valid pulse.
                waiting_for_low_stable = false;
            }

            if (!in_pulse) {
                in_pulse = true;
                pulse_start_ms = now_ms;
            }
        } else {
            // Falling edge: sensor goes LOW (part absent)
            last_falling_edge_ms = now_ms;

            if (in_pulse) {
                // Validate pulse width and increment count
                const uint32_t pulse_width_ms = now_ms - pulse_start_ms;
                
                if (pulse_width_ms >= MIN_VALID_PULSE_MS) {
                    count.fetch_add(1, std::memory_order_relaxed);
                    waiting_for_low_stable = true; // Require LOW to stabilize
                }
                
                in_pulse = false;
            }
        } }, InterruptMode::CHANGE);

    // Ensure the display shows the initial count (0) immediately on startup.
    update_display(io, 0);

    while (true) {
        const uint32_t current_time = io.millis();
        const uint32_t current_count = count.load(std::memory_order_relaxed);

        const bool keep_display_on = (current_time - last_display_update) >= DISPLAY_UPDATE_INTERVAL_MS;

        if (keep_display_on || current_count != last_count) {
            update_display(io, current_count);
            last_count = current_count;
            last_display_update = current_time;
        }
    }
}