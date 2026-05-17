//
// Created by awalol on 2026/3/5.
//

#ifndef DS5_BRIDGE_AUDIO_H
#define DS5_BRIDGE_AUDIO_H

#include <cstdint>

void audio_init();
void audio_loop();
void core1_entry();
void set_headset(bool state);

// Accessors used by the optional OLED add-on (diag + VU meter screens).
uint32_t audio_fifo_drops();
uint32_t opus_fifo_drops();
uint8_t  audio_peak_speaker();   // 0..255, decays on read
uint8_t  audio_peak_haptic();    // 0..255, decays on read

// Byte-flow counters for the Diagnostics screen + web emulator.
uint32_t audio_usb_frames();
uint32_t audio_bt_packets();

#endif //DS5_BRIDGE_AUDIO_H