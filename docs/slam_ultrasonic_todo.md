# Integrazione ultrasuoni nello SLAM – TODO

Obiettivo: definire e scegliere la strategia di utilizzo dei dati `ultrasonic` pubblicati dal bridge Pico su MQTT (`.../data`) all'interno del modulo SLAM (`src/slam/`).

- __Opzione A – Gating di sicurezza semplice (basso impatto)__
  - Leggere `ultrasonic.left|center|right` in `onDataMessage()`.
  - Parametro: `slam_config.ultrasonic.obstacle_threshold_m` (default es. 0.30 m).
  - Se una distanza < soglia:
    - evitare update (saltare `processIMU()` per quel ciclo) oppure
    - aumentare le covarianze del modello (Q/R) per ridurre la fiducia nell'aggiornamento.
  - Pro: implementazione rapida, robustezza contro outlier/meccanici.
  - Contro: non contribuisce direttamente alla stima di posa.
  - Criteri di accettazione:
    - Parametrizzazione da `robot_config.json`.
    - Log chiaro quando scatta il gating.

- __Opzione B – Misura per EKF come range-to-obstacle (medio)__
  - Aggiungere API `SensorFusion::processUltrasonic({left, center, right, timestamp})`.
  - Integrare un measurement model nell’EKF che usa distanza ostacoli attesi (richiede mappa o ipotesi ambientali).
  - Pro: informazione addizionale nella stima di stato.
  - Contro: serve modellazione, possibile drift se modello errato.
  - Criteri di accettazione:
    - Modello di misura documentato (equazioni, Jacobiani).
    - Test con dati registrati e confronto RMSE vs baseline.

- __Opzione C – Aggiornamento mappa/occupancy grid (medio–alto)__
  - Utilizzare `ultrasonic` per aggiornare un layer di occupancy (indipendente dalla posa o come supporto al planner/avoidance).
  - Pubblicare periodicamente la grid su MQTT (topic `slam/map` o nuovo dedicato es. `slam/occupancy`).
  - Pro: utile per navigazione/avoidance.
  - Contro: non migliora direttamente la posa senza loop con localizzazione.
  - Criteri di accettazione:
    - Parametri grid (risoluzione, size, decay) da `robot_config.json`.
    - Visualizzazione rapida (script o viewer).

## Note operative
- I dati `ultrasonic` sono disponibili in `.../data` come:
  ```json
  {
    "ultrasonic": { "left": 0.85, "center": 1.20, "right": 0.90 }
  }
  ```
- Pipeline attuale: `onDataMessage()` usa solo `accel`, `gyro`, `timestamp` → `sensor_fusion_->processIMU()`.

## Proposta
- Implementare prima __Opzione A__ (quick win), con flag runtime per attivazione/disattivazione da config.
- Pianificare B o C in base ai risultati e alle esigenze del planner.
