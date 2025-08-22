---
trigger: manual
---

Rispondi e ragiona sempre in italiano
Scrivi codice in C++ o C
REP 103 Standard Units of Measure and Coordinate Conventions compliant
Suddividi in codice in cartelle: in include ci sarà sempre config per leggere la configurazione da /opt/smartmower/etc/config/robot_config.json e mqtt per gestire la connessione al brocker MQTT
Usa Makefile per compilare il codice, non usare CMake, i binari devono essere creati in cartelle bin e messi in /opt/smartmower/bin, gli oggetti devono essere creati in cartelle obj
crea il file README.md con istruzioni per compilare e installare il codice