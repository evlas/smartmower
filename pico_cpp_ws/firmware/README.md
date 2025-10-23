# Pico Firmware C++ (IMU @ 100 Hz)

Questo è uno scheletro firmware in C++ per RP2040 (Raspberry Pi Pico) che:
- Inizializza I2C0 @400 kHz su SDA=GP16, SCL=GP17.
- Legge BNO055 all'indirizzo 0x28 con burst-read e converte i dati in:
  - Quaternione WXYZ (float32)
  - Accel XYZ in m/s^2 (float32)
  - Gyro XYZ in rad/s (float32)
- Invia frame IMU a 100 Hz su UART0 @230400 bps (TX=GP0, RX=GP1) usando protocollo COBS + CRC16-CCITT.
- Mantiene gli stessi `msg_id` del firmware MicroPython:
  - IMU: 0x01 (payload 10 float LE)

## Requisiti
- Pico SDK installato e variabile `PICO_SDK_PATH` impostata
- CMake >= 3.13, toolchain per RP2040

## Build
```bash
mkdir -p build && cd build
cmake ..
make -j
```
Produrrà `pico_firmware.uf2` nella cartella `build` (se il tuo SDK è configurato correttamente).

## Pin
- I2C0 SDA=GP16, SCL=GP17
- UART0 TX=GP0, RX=GP1 @ 230400

## Note
- Questo è uno scheletro pensato per partire subito; adatta i pin o le frequenze in `src/main.cpp` se usi cablaggi diversi.
- Il protocollo è compatibile con il nodo ROS `pico_control_hardware` esistente.
