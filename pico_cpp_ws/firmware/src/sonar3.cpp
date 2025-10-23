#include "sonar3.hpp"
#include "pico/time.h"

Sonar3::Sonar3(uint trig_L, uint echo_L,
               uint trig_C, uint echo_C,
               uint trig_R, uint echo_R,
               uint32_t timeout_us)
: tL_(trig_L), eL_(echo_L), tC_(trig_C), eC_(echo_C), tR_(trig_R), eR_(echo_R), timeout_us_(timeout_us) {
    gpio_init(tL_); gpio_set_dir(tL_, GPIO_OUT); gpio_put(tL_, 0);
    gpio_init(eL_); gpio_set_dir(eL_, GPIO_IN);
    gpio_init(tC_); gpio_set_dir(tC_, GPIO_OUT); gpio_put(tC_, 0);
    gpio_init(eC_); gpio_set_dir(eC_, GPIO_IN);
    gpio_init(tR_); gpio_set_dir(tR_, GPIO_OUT); gpio_put(tR_, 0);
    gpio_init(eR_); gpio_set_dir(eR_, GPIO_IN);
}

float Sonar3::measure_once(uint trig, uint echo, uint32_t timeout_us) {
    // Trigger 10us pulse
    gpio_put(trig, 0);
    sleep_us(2);
    gpio_put(trig, 1);
    sleep_us(10);
    gpio_put(trig, 0);
    // Wait for echo rising edge with timeout
    uint64_t start = time_us_64();
    while (gpio_get(echo) == 0) {
        if ((time_us_64() - start) > timeout_us) return -1.0f;
    }
    // Measure high time
    uint64_t t0 = time_us_64();
    while (gpio_get(echo) == 1) {
        if ((time_us_64() - t0) > timeout_us) return -1.0f;
    }
    uint64_t dur = time_us_64() - t0;
    // Convert to meters (343 m/s), divide by 2 for round-trip
    float dist = (dur / 1e6f) * 343.0f * 0.5f;
    return dist;
}

bool Sonar3::read_all(float &dL, float &dC, float &dR) {
    dL = measure_once(tL_, eL_, timeout_us_);
    sleep_us(200);
    dC = measure_once(tC_, eC_, timeout_us_);
    sleep_us(200);
    dR = measure_once(tR_, eR_, timeout_us_);
    return true;
}
