#pragma once
// Interrupt latch for the BLTouch-style probe on the laser height probe head.
//
// The probe's Z-min output is a short HIGH pulse (milliseconds) when the pin is pushed up, and
// in its default mode the probe stows the pin itself. ESPHome's gpio binary_sensor reads the pin
// LEVEL in the main loop after the interrupt, so a pulse that ends before the loop runs (display
// redraw, Wi-Fi) is lost. This ISR watches both edges and latches a trigger that only an
// explicit clear resets, so a trigger cannot be missed.
//
// No external parts: the probe's white wire goes straight to the GPIO and the chip's internal
// pull-up (~45k) supplies the HIGH. That is only safe with the probe in OPEN-DRAIN output mode
// (BLTouch default; check a clone with a meter, see the README). A pulse must stay HIGH for at
// least MIN_PULSE_US to count, which rejects the microsecond spikes that stepper wiring induces
// (the job an RC filter would otherwise do).

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"

namespace laserprobe {

static const int64_t MIN_PULSE_US = 100;

static volatile uint32_t trigger_count = 0;
static volatile bool latched = false;
static volatile int64_t rise_us = -1;   // time of the last rising edge, -1 when LOW
static gpio_num_t probe_pin = GPIO_NUM_NC;

static inline void IRAM_ATTR accept() {
  trigger_count = trigger_count + 1;
  latched = true;
}

static void IRAM_ATTR on_edge(void *) {
  const int64_t now = esp_timer_get_time();
  if (gpio_get_level(probe_pin)) {
    rise_us = now;
  } else {
    const int64_t r = rise_us;
    rise_us = -1;
    if (r >= 0 && now - r >= MIN_PULSE_US) accept();
  }
}

inline void setup(int pin) {
  probe_pin = static_cast<gpio_num_t>(pin);
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << pin;
  cfg.mode = GPIO_MODE_INPUT;
  cfg.pull_up_en = GPIO_PULLUP_ENABLE;  // the only pull-up: no external resistor
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_ANYEDGE;
  gpio_config(&cfg);
  // ESP_ERR_INVALID_STATE means another component already installed the service: fine.
  gpio_install_isr_service(0);
  gpio_isr_handler_add(probe_pin, on_edge, nullptr);
}

// Called from the main loop: an output that rises and STAYS high (touch mode, alarm) never
// produces a falling edge, so accept it here once it has been high long enough.
inline void poll() {
  const int64_t r = rise_us;
  if (r >= 0 && gpio_get_level(probe_pin) && esp_timer_get_time() - r >= MIN_PULSE_US) {
    rise_us = -1;  // so the eventual falling edge doesn't count this pulse twice
    accept();
  }
}

// Open-drain test (button probe_od_check), run while the probe holds its output "triggered"
// (pin up + touch mode). With the pull-up, HIGH proves the wire. With the pull-DOWN, an
// open-drain output lets go and the pin reads LOW; an output that drives HIGH itself (5 V
// push-pull, or a pull-up inside the probe) still reads HIGH. Interrupts are off meanwhile so
// the pull change is not counted as a trigger.
inline void set_pull_down() {
  gpio_intr_disable(probe_pin);
  gpio_set_pull_mode(probe_pin, GPIO_PULLDOWN_ONLY);
}
inline void restore_pull_up() {
  gpio_set_pull_mode(probe_pin, GPIO_PULLUP_ONLY);
  rise_us = -1;
  gpio_intr_enable(probe_pin);
}

// Last HIST_MAX results (result_z as the plugin sets it), for the OLED's change and trend graph.
static const int HIST_MAX = 24;
static float hist[HIST_MAX];
static int hist_n = 0, hist_head = 0;
inline void push_result(float v) {
  hist[hist_head] = v;
  hist_head = (hist_head + 1) % HIST_MAX;
  if (hist_n < HIST_MAX) hist_n++;
}
inline float hist_at(int i) { return hist[(hist_head - hist_n + i + 2 * HIST_MAX) % HIST_MAX]; }  // 0 = oldest

inline void clear() { latched = false; }
inline bool is_latched() { poll(); return latched; }
inline uint32_t count() { return trigger_count; }
inline bool level() { return probe_pin != GPIO_NUM_NC && gpio_get_level(probe_pin) == 1; }

}  // namespace laserprobe
