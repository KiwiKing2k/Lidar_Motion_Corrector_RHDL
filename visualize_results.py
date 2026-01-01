import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# --- CONFIGURARE ---
#valorile sunt alese din coloana 'timestamp_ns' din fisierul raw_lidar.csv
START_TIME = 1574367378706156049
END_TIME   = 1574367378806136034

RAW_FILE = 'data/raw_lidar.csv'
CORRECTED_FILE = 'data/corrected_cloud.csv'

def load_original_frame(filename, t_start, t_end):
    print(f"Caut frame-ul original in {filename}...")
    chunks = []
    found_start = False

    for chunk in pd.read_csv(filename, chunksize=100000):
        mask = (chunk['timestamp_ns'] >= t_start) & (chunk['timestamp_ns'] <= t_end)

        if mask.any():
            found_start = True
            chunks.append(chunk[mask])
        elif found_start:
            break

    if not chunks:
        print("EROARE: Nu am gasit puncte!")
        return None

    return pd.concat(chunks)

def calculate_stats(df_orig, df_corr):
    # Ne asiguram ca indexul este resetat pentru a putea face scaderea linie cu linie
    p_orig = df_orig[['x', 'y', 'z']].reset_index(drop=True).to_numpy()
    p_corr = df_corr[['x', 'y', 'z']].reset_index(drop=True).to_numpy()

    # Verificare dimensiuni
    if len(p_orig) != len(p_corr):
        min_len = min(len(p_orig), len(p_corr))
        print(f"ATENTIE: Numar diferit de puncte! Orig: {len(p_orig)}, Corr: {len(p_corr)}")
        print(f"Trunchiem la {min_len} pentru calcul.")
        p_orig = p_orig[:min_len]
        p_corr = p_corr[:min_len]

    # Calculam diferenta vectoriala (delta X, delta Y, delta Z)
    diff = p_corr - p_orig

    # Calculam Distanta Euclidiana (Norma vectorului) pentru fiecare punct
    # dist = sqrt((x2-x1)^2 + (y2-y1)^2 + ...)
    distances = np.linalg.norm(diff, axis=1)

    return distances

def plot_results(original, corrected, distances):
    fig = plt.figure(figsize=(18, 6))

    # Pas pentru viteza graficului
    step = 5

    # plot original
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.scatter(original['x'][::step], original['y'][::step], original['z'][::step],
                c=original['intensity'][::step], cmap='viridis', s=1)
    ax1.set_title('Original')
    ax1.set_xlabel('X')

    # plot corectat
    ax2 = fig.add_subplot(132, projection='3d')
    ax2.scatter(corrected['x'][::step], corrected['y'][::step], corrected['z'][::step],
                c=corrected['intensity'][::step], cmap='plasma', s=1)
    ax2.set_title('Corectat (FPGA)')
    ax2.set_xlabel('X')

    # plot histograma distante
    ax3 = fig.add_subplot(133)
    dist_cm = distances * 100

    ax3.hist(dist_cm, bins=50, color='skyblue', edgecolor='black')
    ax3.set_title('Distribuția Corecțiilor (Deplasare)')
    ax3.set_xlabel('Deplasare (centimetri)')
    ax3.set_ylabel('Număr de puncte')
    ax3.grid(axis='y', alpha=0.5)

    # statistici
    mean_val = np.mean(dist_cm)
    max_val = np.max(dist_cm)
    stats_text = f"Medie: {mean_val:.2f} cm\nMax: {max_val:.2f} cm"
    ax3.text(0.95, 0.95, stats_text, transform=ax3.transAxes,
             verticalalignment='top', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    print("--- Analiza Rezultate LiDAR ---")

    df_corr = pd.read_csv(CORRECTED_FILE)
    df_orig = load_original_frame(RAW_FILE, START_TIME, END_TIME)

    if df_orig is not None:
        deltas = calculate_stats(df_orig, df_corr)

        avg_m = np.mean(deltas)
        max_m = np.max(deltas)

        print("\n--- REZULTATE STATISTICE ---")
        print(f"Număr puncte analizate: {len(deltas)}")
        print(f"Deplasare Medie (Corecție): {avg_m:.4f} metri ({avg_m*100:.2f} cm)")
        print(f"Deplasare Maximă:           {max_m:.4f} metri ({max_m*100:.2f} cm)")
        print("----------------------------")

        if avg_m < 0.001:
            print("NOTA: Media este foarte mică. Verificați dacă transformările IMU nu sunt aproape de zero.")

        plot_results(df_orig, df_corr, deltas)