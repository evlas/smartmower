#pragma once
#include <cstdint>
#include "pico/stdlib.h"

/** \brief Encoder a singolo canale con debounce via timestamp hardware.
 *  Conta i fronti di salita sul GPIO usando IRQ e consente lettura atomica del conteggio.
 */
class SingleChannelEncoder {
public:
    /** \brief Crea un encoder su un dato pin.
     *  \param gpio GPIO da usare (es. 18/19)
     *  \param debounce_us filtro antirimbalzo in microsecondi (es. 200)
     */
    SingleChannelEncoder(uint gpio, uint32_t debounce_us = 200);
    /** \brief Restituisce i tick accumulati e azzera il contatore. */
    int32_t read_and_reset();

private:
    static void irq_trampoline(uint gpio, uint32_t events);
    void on_edge(uint gpio, uint32_t events);

    uint gpio_;
    volatile int32_t count_;
    volatile uint64_t last_us_;
    uint32_t debounce_us_;
};
