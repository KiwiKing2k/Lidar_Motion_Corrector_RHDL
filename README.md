# LiDAR Motion Correction Accelerator

Implementare hibrida hardware-software pentru corecția distorsiunilor de mișcare (motion de-skewing) a datelor LiDAR folosind date IMU.
Logica hardware este descrisă folosind RHDL (Rust Hardware Description Language).

## Cerințe

* Rust (Cargo) + RHDL, CSV, Serde, Nalgebra
* Python 3 (pentru vizualizare) + Matplotlib, Pandas, numpy
* GTKWave (pentru analiza formelor de undă)

## Instrucțiuni de Build și Rulare

### 0. Configurare date de intrare

Asigurați-vă că aveți datele de intrare necesare în format CSV în directorul `data/input/`. Acestea ar trebui să includă:
* Date brute LiDAR sub numele `raw_lidar.csv`
* Date IMU corespunzătoare sub numele `raw_imu.csv`

### 1. Procesare Completă (Host + FPGA Simulation)

Pentru a rula întregul pipeline software care încarcă datele brute (CSV), calculează traiectoria și simulează procesarea hardware cycle-accurate a fiecărui punct:

```bash
cargo run --bin host_software --release
```
### 2. (Opțional) Vizualizare Rezultate

După rularea simulării, puteți genera graficele rulând scriptul Python. Deschideți fișierul visualize_results.py și modificați 
variabilele START_TIME și END_TIME pentru a ajusta fereastra de timp afișată în grafice.

```bash
python3 visualize_results.py
```

## Validare și Testare (fpga_core)

Un punct critic al verificării sistemului este testul din `main` al modulului `fpga_core`. Acesta funcționează ca un **test_bench 
major**, validând logica de procesare pe hardware. Se poate rula folosind comanda:

```bash
cargo run --bin fpga_core
```

### Sursa Datelor de Test
Datele de intrare folosite pentru validare sunt consistente cu cele utilizate în **Lio-Sam**, provenind din seturile de date publice Google (Google Cartographer).

* **Dataset:** Folderul `Walking`
* **Dimensiune:** Aproximativ 12GB din aproximativ 50GB de date brute.
* **Context:** Înregistrarea este realizată cu o cameră LIDAR mobilă, fiind "plimbată" de un operator uman. Scenariul surprinde un mediu exterior, cel mai probabil un parc, oferind o traiectorie complexă pentru testarea algoritmilor de mapare și localizare.
