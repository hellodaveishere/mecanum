# 🧭 Documentazione Nav2 + SLAM Toolbox
Robot: piattaforma omnidirezionale (mecanum)  
Footprint: **25.5 × 15 cm**  
Base frame: **base_footprint**

Questo documento descrive in modo chiaro e tecnico tutti i moduli utilizzati nel file `nav2_params.yaml`, con note pratiche su cosa controllare, cosa modificare e come mantenere il sistema stabile.

---

# 📌 1. AMCL — Adaptive Monte Carlo Localization

AMCL è il modulo che permette al robot di localizzarsi all’interno di una mappa statica usando:

- LaserScan
- Odometria
- Modello di moto

### Funzioni principali
- Stima della posa tramite particelle
- Correzione continua della posizione
- Supporto per robot omnidirezionali (OmniMotionModel)

### Parametri critici
- `base_frame_id`: deve essere **base_footprint**
- `z_hit`, `z_rand`, `z_short`, `z_max`: influenzano la stabilità della localizzazione
- `max_particles`: più alto = più preciso ma più pesante

### Quando modificarlo
- Se la posa “salta”: abbassare `z_rand`
- Se la posa è lenta a correggersi: aumentare `max_particles`

---

# 📌 2. SLAM Toolbox — Mapping & Localization

SLAM Toolbox permette di:

- costruire mappe (modalità `mapping`)
- localizzarsi su mappe esistenti (modalità `localization`)
- chiudere loop (loop closure)
- ottimizzare la mappa con Ceres

### Parametri critici
- `mode`: mapping/localization
- `resolution`: deve essere identica alla global_costmap
- `minimum_travel_distance` e `minimum_travel_heading`: controllano la densità della mappa
- `do_loop_closing`: se attivo, migliora la mappa ma può deformarla se i parametri sono troppo permissivi

### Quando modificarlo
- Se la mappa è “vuota”: ridurre `minimum_travel_distance`
- Se la mappa si deforma: aumentare `loop_match_minimum_response_fine`

---

# 📌 3. Global Costmap

Rappresenta l’intera mappa dell’ambiente.  
Usata dal planner globale per generare percorsi.

### Plugin usati
- `static_layer`: mappa statica
- `obstacle_layer`: ostacoli dinamici
- `inflation_layer`: margine di sicurezza

### Parametri critici
- `resolution`: deve essere identica alla mappa
- `inflation_radius`: influenza quanto il robot evita gli ostacoli
- `track_unknown_space`: deve essere true per ambienti non completamente mappati

### Quando modificarlo
- Se il robot è troppo “timido”: ridurre `inflation_radius`
- Se il robot taglia gli angoli: aumentare `cost_scaling_factor`

---

# 📌 4. Local Costmap

Mappa locale centrata sul robot, aggiornata in tempo reale.  
Serve al controller per evitare ostacoli dinamici.

### Plugin usati
- `obstacle_layer`
- `denoise_layer`
- `inflation_layer`

### Parametri critici
- `global_frame = odom`
- `rolling_window = true`
- `width` / `height`: dimensione della finestra locale

### Quando modificarlo
- Se il robot non evita bene gli ostacoli: aumentare `update_frequency`
- Se il robot è troppo lento a reagire: ridurre `inflation_radius`

---

# 📌 5. Planner Server — Smac Hybrid (OMNI)

Genera il percorso globale dalla posizione corrente al goal.

### Perché Smac Hybrid?
- supporta robot omnidirezionali
- genera percorsi continui e realistici
- ottimo per ambienti indoor

### Parametri critici
- `motion_model_for_search = OMNI`
- `tolerance`: distanza accettabile dal goal
- `cost_travel_multiplier`: influenza la forma del percorso

### Quando modificarlo
- Se il robot non raggiunge il goal: aumentare `tolerance`
- Se i percorsi sono strani: riportare `cost_travel_multiplier` a 1.0

---

# 📌 6. Smoother Server

Rende il percorso più fluido e navigabile.

### Parametri critici
- `tolerance`: precisione della convergenza
- `do_refinement`: migliora la qualità del percorso

### Quando modificarlo
- Se il percorso è troppo “spigoloso”: ridurre `tolerance`
- Se il percorso è troppo lento da calcolare: aumentare `tolerance`

---

# 📌 7. Behavior Server

Gestisce i comportamenti di recovery:

- spin
- backup
- drive_on_heading
- wait
- assisted_teleop

### Parametri critici
- `max_rotational_vel` e `min_rotational_vel`
- `simulate_ahead_time`

### Quando modificarlo
- Se il robot ruota troppo lentamente: aumentare `max_rotational_vel`
- Se il robot ruota troppo velocemente: ridurre `rotational_acc_lim`

---

# 📌 8. Waypoint Follower

Permette al robot di seguire una lista di waypoint.

### Parametri critici
- `waypoint_pause_duration`
- `stop_on_failure`

### Quando modificarlo
- Se vuoi pause più brevi/lunghe ai waypoint: modifica `waypoint_pause_duration`

---

# 📌 9. Velocity Smoother

Filtra e limita le velocità inviate al robot.

### Parametri critici
- `max_velocity`: [vx, vy, wz]
- `max_accel` / `max_decel`
- `feedback`: OPEN_LOOP o CLOSED_LOOP

### Quando modificarlo
- Se il robot è troppo nervoso: ridurre `max_velocity`
- Se il robot è troppo lento: aumentare `max_accel`

---

# 📌 10. Collision Monitor

Modulo di sicurezza che monitora ostacoli vicini e modifica `cmd_vel`.

### Funzioni
- rallentamento
- stop
- approccio controllato agli ostacoli

### Parametri critici
- `min_height` / `max_height`: devono essere coerenti con l’altezza del lidar
- `time_before_collision`: quanto presto frena

### Quando modificarlo
- Se il robot frena troppo presto: ridurre `time_before_collision`
- Se il robot frena troppo tardi: aumentarlo

---

# 📌 11. Map Saver

Salva la mappa generata da SLAM Toolbox o map_server.

### Parametri critici
- `output_map_format`: png o pgm
- `map_subscribe_transient_local`: deve essere true

---

# 📌 12. Map Server

Pubblica la mappa statica per Nav2.

### Parametri critici
- `yaml_filename`: percorso della mappa
- `transient_local`: deve essere true

---

# 📌 13. Note finali

- Tutti i frame devono essere coerenti:  
  **map → odom → base_footprint → base_link**
- La risoluzione deve essere identica tra:
  - SLAM Toolbox
  - global_costmap
  - mappa salvata
- Il footprint deve essere identico in:
  - local_costmap
  - global_costmap
  - collision_monitor

---


