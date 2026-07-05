//
// 1) PS_ON on GP16 asserted 1 second before the power-button press, for the
//    eGPU's own PSU. GP16 drives a small transistor buffer (see wiring
//    below) rather than the PS_ON line directly — PS_ON idles at 5V
//    (measured on this hardware), which is out of spec for this chip's
//    GPIOs, so the transistor isolates the Pico from ever seeing that 5V
//    at all, in either state.
// 2) Momentary power-button press on GP18, triggered either by a DS5
//    controller connecting over Bluetooth, or by a physical case button on
//    GP21 — both only while the PC currently reads as off. Without that
//    check, a controller reconnecting while the PC is already running
//    (e.g. after its own idle timeout) would press the button again, which
//    most OSes treat as a shutdown request.
// 3) Disconnects the DS5 controller once the PC's main rail drops, so it
//    doesn't sit paired to a machine that's off.
//
// "Is the PC on" is read directly from hardware: GP20 senses the PSU's
// 3.3V main rail (through a 1k series resistor). That rail is only live
// during real S0 operation — this board cuts it in S3 sleep too (a common
// "Deep Sleep" implementation for ErP compliance), so "rail present" means
// genuinely running, and "rail absent" covers both asleep and fully off.
// That's exactly the boundary wanted here: a press should be allowed in
// either of those states (it boots or wakes the machine) and blocked only
// while the PC is actually running. This replaced an earlier attempt based
// on TinyUSB's suspend/resume callbacks, which turned out not to reliably
// reflect this board's real shutdown/sleep behavior.
//
// Wiring:
//   - ITX motherboard front-panel POWER_SW+ -> Pico GP18
//   - ITX motherboard front-panel POWER_SW- -> Pico GND
//   - ATX PSU 5VSB+                          -> Pico VSYS (keeps the Pico
//     alive while the ITX board itself is off)
//   - ATX PSU COM                            -> Pico GND
//   - ATX PSU 3.3V (any orange wire)         -> 1k resistor -> Pico GP20
//   - Case power button, one leg             -> Pico GP21
//   - Case power button, other leg           -> Pico GND
//   - Pico GP16 -> resistor -> transistor gate/base
//     transistor drain/collector -> eGPU PSU PS_ON
//     transistor source/emitter  -> eGPU PSU COM (and Pico GND)
//   - Pico micro-USB                         -> ITX motherboard USB-A port
//
#ifndef DS5_BRIDGE_POWER_CTRL_H
#define DS5_BRIDGE_POWER_CTRL_H

// Call once at startup. Leaves GP18 released (Hi-Z).
void power_ctrl_init();

// Call once per main-loop iteration. Non-blocking.
void power_ctrl_task();

#endif //DS5_BRIDGE_POWER_CTRL_H
