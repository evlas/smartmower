#include "encoder.hpp"
#include "pico/time.h"
#include "hardware/sync.h"

static SingleChannelEncoder* s_instances[30] = {nullptr};

SingleChannelEncoder::SingleChannelEncoder(uint gpio, uint32_t debounce_us)
: gpio_(gpio), count_(0), last_us_(0), debounce_us_(debounce_us) {
    gpio_init(gpio_);
    gpio_set_dir(gpio_, GPIO_IN);
    gpio_pull_up(gpio_);
    s_instances[gpio_] = this;
    gpio_set_irq_enabled_with_callback(gpio_, GPIO_IRQ_EDGE_RISE, true,
                                       [](uint gpio, uint32_t events){
                                           if (gpio < 30 && s_instances[gpio]) s_instances[gpio]->on_edge(gpio, events);
                                       });
}

void SingleChannelEncoder::on_edge(uint, uint32_t) {
    uint64_t now = time_us_64();
    if (now - last_us_ >= debounce_us_) {
        ++count_;
        last_us_ = now;
    }
}

int32_t SingleChannelEncoder::read_and_reset() {
    uint32_t irq = save_and_disable_interrupts();
    int32_t v = count_;
    count_ = 0;
    restore_interrupts(irq);
    return v;
}
