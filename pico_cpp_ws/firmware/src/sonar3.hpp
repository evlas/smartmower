/**
 * @file sonar3.hpp
 * @brief Interfaccia per driver a 3 sensori ultrasuoni (L,C,R) con trigger/echo.
 */
#pragma once
#include <cstdint>
#include "pico/stdlib.h"

/** \brief Driver per 3 sonars ad ultrasuoni (L,C,R) con trigger/echo.
 *  Misura il tempo di eco in modo bloccante con timeout e converte in distanza (m).
 */
class Sonar3 {
public:
    /** \brief Crea un sonar triplo.
     *  \param trig_L GPIO trigger sinistra
     *  \param echo_L GPIO echo sinistra
     *  \param trig_C GPIO trigger centro
     *  \param echo_C GPIO echo centro
     *  \param trig_R GPIO trigger destra
     *  \param echo_R GPIO echo destra
     *  \param timeout_us Timeout misura singola in microsecondi (es. 30000)
     */
    Sonar3(uint trig_L, uint echo_L,
           uint trig_C, uint echo_C,
           uint trig_R, uint echo_R,
           uint32_t timeout_us = 30000);

    /** \brief Legge tutte le distanze (m). Restituisce false su errore generale. */
    bool read_all(float &dL, float &dC, float &dR);

private:
    /** \brief Misura singola distanza per una coppia trigger/echo.
     *  \return Distanza in metri o valore negativo su timeout/errore.
     */
    static float measure_once(uint trig, uint echo, uint32_t timeout_us);
    /** \brief GPIO trigger/echo per L, C, R. */
    uint tL_, eL_, tC_, eC_, tR_, eR_;
    /** \brief Timeout massimo per misura singola (us). */
    uint32_t timeout_us_;
};
