/**
 * @file encoder.hpp
 * @brief Interfaccia per encoder a singolo canale con debounce via timestamp (RP2040).
 *
 * @details Gestisce il conteggio dei fronti di salita tramite IRQ su un singolo GPIO
 * con semplice filtro antirimbalzo temporale. Fornisce una lettura atomica del contatore.
 */
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
    /** \brief Trampolino IRQ per instradare l'interrupt all'istanza corretta. */
    static void irq_trampoline(uint gpio, uint32_t events);
    /** \brief Gestore di bordo salita: applica debounce e incrementa contatore. */
    void on_edge(uint gpio, uint32_t events);

    /** \brief GPIO associato all'encoder. */
    uint gpio_;
    /** \brief Contatore tick (incrementato su edge validati). */
    volatile int32_t count_;
    /** \brief Timestamp ultimo fronte accettato (us). */
    volatile uint64_t last_us_;
    /** \brief Finestra di debounce in microsecondi. */
    uint32_t debounce_us_;
};
