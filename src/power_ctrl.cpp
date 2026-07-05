//
// See power_ctrl.h.
//
// GP16 drives a transistor buffer to the eGPU PSU's PS_ON (see
// power_ctrl.h for why a buffer is needed — PS_ON idles at 5V, out of
// spec for this chip's GPIOs). Simple push-pull output, no Hi-Z dance
// needed here since the transistor does the isolating: HIGH turns the
// transistor on (pulls PS_ON low, PSU on), LOW turns it off (PS_ON floats
// back up via the PSU's own pull-up, but that 5V never reaches the pin).
//
// GP18 sits Hi-Z (input, pulls disabled) at rest. The ITX motherboard's own
// front-panel header pulls POWER_SW+ up internally, same as a real power
// button wired there instead of a Pico. To "press" it, GP18 is switched to
// an output driven low for kPulseUs, then switched back to Hi-Z — this
// sinks the line to ground for the pulse and never tries to source a
// voltage of its own, so there's no level/voltage mismatch to worry about.
//
// GP20 is a plain digital input reading the PSU's 3.3V main rail through a
// 1k series resistor. No pull needed beyond a defined default: the rail is
// actively driven to ~3.3V when the PC is on, and to ~0V (not floating)
// when it's off, so the Pico's own weak pull-down is only there for a sane
// reading before the rail has ever been sensed.
//
// Trigger: a rising edge of bt_is_connected(), or a debounced press of the
// case button on GP21, either one while the main rail reads absent (see
// power_ctrl.h for why that's the right condition). That starts a short
// sequence: assert PS_ON, wait kPsuStabilizeUs for the eGPU's PSU to come
// up, then pulse the motherboard's power button. Disconnect: once the main
// rail has read absent continuously for kOffConfirmUs — but only after
// it's been confirmed genuinely on at least once since the controller
// connected. That "seen on" requirement matters because right after our
// own button press, the rail is off by definition (that's why the press
// was allowed) and can take a while to come up — especially on this
// board, whose dying CMOS battery makes it power-cycle itself once or
// twice (each cycle ~1-2s off) before it actually boots. Without requiring
// "was on, then dropped," that whole boot-up window would look identical
// to a real shutdown and disconnect the controller instantly after every
// power-on.
//

#include "power_ctrl.h"

#include "hardware/gpio.h"
#include "pico/time.h"
#include "bt.h"

namespace {

constexpr uint kPinPsuOn = 16;         // -> transistor buffer -> eGPU PSU PS_ON
constexpr uint kPinPcPower = 18;       // -> ITX motherboard POWER_SW+
constexpr uint kPinMainRailSense = 20; // <- PSU 3.3V rail, via series resistor
constexpr uint kPinManualButton = 21;  // <- case power button, to GND, uses internal pull-up

constexpr uint64_t kPsuStabilizeUs = 1'000'000; // PS_ON asserted this long before the button pulse
constexpr uint64_t kPulseUs = 300'000;          // momentary power-button press length
constexpr uint64_t kButtonDebounceUs = 30'000;  // case button debounce

// How long the main rail has to read continuously absent before it's
// treated as a real shutdown/sleep rather than a boot-time power flicker.
constexpr uint64_t kOffConfirmUs = 3'000'000; // 3s

enum class SeqState { kIdle, kPsuStabilizing, kPulsing };

SeqState sequence_state = SeqState::kIdle;
uint64_t state_entered_us = 0;

bool prev_bt_connected = false;
bool prev_rail_on = false;
bool rail_seen_on_this_session = false; // armed only once the rail has actually come up since connecting
uint64_t rail_absent_since_us = 0;

bool button_stable_pressed = false; // debounced state
bool button_raw_prev = false;       // raw level from the last poll
uint64_t button_last_change_us = 0;

void psu_set(bool on) {
    gpio_put(kPinPsuOn, on ? 1 : 0); // drives the transistor buffer, never PS_ON directly
}

void pc_power_set(bool pressed) {
    if (pressed) {
        gpio_set_dir(kPinPcPower, GPIO_OUT);
        gpio_put(kPinPcPower, 0);
    } else {
        gpio_set_dir(kPinPcPower, GPIO_IN); // Hi-Z, released
    }
}

bool rail_on() {
    return gpio_get(kPinMainRailSense);
}

// Starts the PSU-then-button sequence, but only if nothing's already in
// progress and the PC currently reads as off. Shared by both trigger
// sources so they're gated identically.
void start_press_if_off() {
    if (sequence_state != SeqState::kIdle) return;
    if (rail_on()) return;
    psu_set(true);
    state_entered_us = time_us_64();
    sequence_state = SeqState::kPsuStabilizing;
}

// Debounced falling-edge (press) detector for the manual button. Returns
// true exactly once per physical press.
bool manual_button_pressed_edge(uint64_t now) {
    const bool raw_pressed = !gpio_get(kPinManualButton); // active low

    if (raw_pressed != button_raw_prev) {
        button_raw_prev = raw_pressed;
        button_last_change_us = now;
    }

    if ((now - button_last_change_us) >= kButtonDebounceUs
        && raw_pressed != button_stable_pressed) {
        button_stable_pressed = raw_pressed;
        return button_stable_pressed; // true only on the press transition
    }
    return false;
}

} // namespace

void power_ctrl_init() {
    gpio_init(kPinPsuOn);
    gpio_set_dir(kPinPsuOn, GPIO_OUT);
    gpio_put(kPinPsuOn, 0); // PS_ON transistor off at boot

    gpio_init(kPinPcPower);
    gpio_disable_pulls(kPinPcPower);
    gpio_set_dir(kPinPcPower, GPIO_IN); // start released

    gpio_init(kPinMainRailSense);
    gpio_set_dir(kPinMainRailSense, GPIO_IN);
    gpio_pull_down(kPinMainRailSense); // defined reading if ever disconnected

    gpio_init(kPinManualButton);
    gpio_set_dir(kPinManualButton, GPIO_IN);
    gpio_pull_up(kPinManualButton);

    sequence_state = SeqState::kIdle;
    prev_bt_connected = bt_is_connected(); // don't treat "already connected at boot" as a new edge
    prev_rail_on = rail_on();
    rail_seen_on_this_session = false;
    rail_absent_since_us = time_us_64();
    button_stable_pressed = false;
    button_raw_prev = !gpio_get(kPinManualButton);
    button_last_change_us = time_us_64();
}

void power_ctrl_task() {
    const uint64_t now = time_us_64();

    if (sequence_state == SeqState::kPsuStabilizing) {
        if ((now - state_entered_us) >= kPsuStabilizeUs) {
            pc_power_set(true);
            state_entered_us = now;
            sequence_state = SeqState::kPulsing;
        }
        return;
    }
    if (sequence_state == SeqState::kPulsing) {
        if ((now - state_entered_us) >= kPulseUs) {
            pc_power_set(false);
            sequence_state = SeqState::kIdle;
        }
        return;
    }

    const bool now_rail_on = rail_on();
    const bool now_bt_connected = bt_is_connected();

    if (now_bt_connected) {
        if (now_rail_on) {
            rail_seen_on_this_session = true; // system genuinely powered up
        } else if (prev_rail_on) {
            rail_absent_since_us = now; // rail just dropped — start timing
        }

        if (rail_seen_on_this_session && !now_rail_on &&
            (now - rail_absent_since_us) >= kOffConfirmUs) {
            bt_disconnect(); // was on, now confirmed off long enough: real shutdown/sleep
            rail_seen_on_this_session = false;
        }
    } else {
        rail_seen_on_this_session = false; // clean slate for the next connection
    }
    prev_rail_on = now_rail_on;

    if (now_bt_connected && !prev_bt_connected) {
        start_press_if_off();
    }
    prev_bt_connected = now_bt_connected;

    if (manual_button_pressed_edge(now)) {
        start_press_if_off();
    }
}
