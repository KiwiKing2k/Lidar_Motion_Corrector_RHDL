mod data_loader;
mod lut_gen;

use std::error::Error;
use std::time::Instant;

fn main() -> Result<(), Box<dyn Error>> {
    println!("LiDAR Motion Correction: SINGLE FRAME MODE ");

    // 1. incărcăm IMU-ul primul (ca sa stim timpul de inceput)
    let start_load = Instant::now();
    let all_imu = data_loader::load_imu_data("data/raw_imu.csv")?;

    if all_imu.is_empty() {
        return Err("Fisierul IMU este gol!".into());
    }

    // cand a pornit IMU-ul
    let imu_start_time = all_imu.first().unwrap().timestamp_ns;
    println!("IMU Start Time: {}", imu_start_time);

    // 2. Incarcare scanarea LiDAR, dar cerem să înceapă DUPĂ imu_start_time
    // adaugam un mic buffer (+1ms) ca să fim siguri că avem date de interpolare în stanga
    let scan_points = data_loader::load_single_scan("data/raw_lidar.csv", imu_start_time + 1_000_000)
        .expect("Eroare la citirea/sincronizarea lidar data");

    // Aflăm intervalul scanării încărcate
    let t_start = scan_points.first().unwrap().timestamp_ns;
    let t_end = scan_points.last().unwrap().timestamp_ns;

    println!("Scanare LiDAR Validă: {} puncte", scan_points.len());
    println!("Interval Scanare: {} -> {}", t_start, t_end);
    println!("Timp Încărcare Total: {:.2?}", start_load.elapsed());

    // 3. filtram IMU-ul pentru acest interval cu margine de eroare
    let margin_ns = 5_000_000; // 5ms
    let relevant_imu: Vec<_> = all_imu.into_iter()
        .filter(|m| m.timestamp_ns >= (t_start - margin_ns) && m.timestamp_ns <= (t_end + margin_ns))
        .collect();

    println!("Date IMU relevante (pentru interpolare): {} măsurători", relevant_imu.len());

    if relevant_imu.len() < 2 {
        println!("Tot nu avem destule date IMU. CSV-urile au patit ceva.");
        return Ok(());
    }

    // 4. traiectorie + LUT
    let trajectory = lut_gen::calculate_trajectory(&relevant_imu);
    let pose_lut = lut_gen::generate_pose_lut(&trajectory);

    println!("Pose-LUT generat: {} intrări", pose_lut.len());

    // 5. testare Simulare
    let mut matched = 0;
    for point in &scan_points {
        if let Some(_pose) = lut_gen::interpolate_pose(&pose_lut, point.timestamp_ns) {
            matched += 1;
        }
    }

    println!("------------------------------------------------");
    println!("REZULTAT SINCRONIZARE:");
    println!("Puncte Corelate cu Succes: {} / {} ({:.2}%)",
             matched, scan_points.len(), (matched as f64 / scan_points.len() as f64) * 100.0);
    println!("------------------------------------------------");

    if matched > 0 {
        println!(" Putem trimite datele (Punct + Pose) la FPGA.");
    }

    Ok(())
}