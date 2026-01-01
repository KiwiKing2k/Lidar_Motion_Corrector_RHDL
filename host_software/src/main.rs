mod data_loader;
mod lut_gen;

use std::error::Error;
use std::time::Instant;
use std::fs::File;
use std::io::Write;

use rhdl::prelude::*;
use fpga_core::{LidarProcessor, LidarInput, LidarOutput};
use fpga_core::types::{Vector3 as FpgaVec3, Matrix3x3 as FpgaMat3x3};

// float -> fixed point (s32)
fn to_fix(val: f64) -> s32 {
    let scaled = (val * 65536.0) as i128; // 16.16 format
    s32::from(scaled.clamp(i32::MIN as i128, i32::MAX as i128))
}

// fixed point -> float
fn from_fix_to_float(val: s32) -> f64 {
    let raw = val.typed_bits().as_i64().unwrap();
    (raw as f64) / 65536.0
}
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

    // 5. testare + Simulare
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
    // aici vom colecta rezultatele
    let mut corrected_cloud = Vec::new();
    let mut matched_count = 0;

    let mut file = File::create("data/corrected_cloud.csv")?;
    writeln!(file, "x,y,z,intensity")?;

    let start_time = Instant::now();

    // pipeline
    for point in &scan_points {
        // gasim pose-ul interpolat
        if let Some(pose) = lut_gen::interpolate_pose(&pose_lut, point.timestamp_ns) {
            matched_count += 1;

            // convertim datele Host -> FPGA (Fixed Point)
            let p_in = FpgaVec3 {
                x: to_fix(point.x),
                y: to_fix(point.y),
                z: to_fix(point.z),
            };

            // convertim matricea de rotatie (pose.rotation este [[f64;3];3])
            let mut r_rows = [[to_fix(0.0); 3]; 3];
            let rot_matrix = pose.rotation.to_rotation_matrix();
            for i in 0..3 {
                for j in 0..3 {
                    r_rows[i][j] = to_fix(rot_matrix[(i, j)]);
                }
            }
            let rot_in = FpgaMat3x3 { rows: r_rows };

            let trans_in = FpgaVec3 {
                x: to_fix(pose.translation.x),
                y: to_fix(pose.translation.y),
                z: to_fix(pose.translation.z),
            };

            // construim intrare in fpga virtual
            let input_active = LidarInput {
                valid: true,
                point: p_in,
                rotation: rot_in,
                translation: trans_in,
            };

            // construim intrare in fpga virtual
            // necesar pentru ca FPGA-ul sa termine calculul fara sa piarda intrarea
            let input_hold = LidarInput {
                valid: false,
                point: p_in,
                rotation: rot_in,
                translation: trans_in,
            };

            // simulam ciclul hardware
            // replicam scenariul din testele 'fpga_core' (Idle -> Pulse -> Hold)
            let mut inputs = Vec::new();

            // t0-t1: Idle/Reset (2 cicluri pentru stabilizare)
            inputs.push(LidarInput::default());
            inputs.push(LidarInput::default());

            // t2: Impuls Date (Incarcare in registre)
            inputs.push(input_active);

            // t3-t17: Wait/Process (Mentinem datele pe fire in timp ce procesorul calculeaza)
            for _ in 0..15 {
                inputs.push(input_hold);
            }

            // rulam simularea pe acest stream
            // instantiem un procesor NOU per punct in simularea asta simpla
            let acc_hdware = LidarProcessor::default();

            // .with_reset(1) -> reset activ in primul ciclu
            let stream = inputs.into_iter().with_reset(1).clock_pos_edge(100);
            let vcd_iter = acc_hdware.run(stream).unwrap();

            // cautam output-ul valid
            for sample_state in vcd_iter {
                // sample.value este (ClockReset, Input, Output)
                // 2 este Output-ul procesorului
                let output = sample_state.value.2;

                if output.valid {
                    let out_vec = output.corrected_point;

                    // conversie inapoi FPGA -> Host
                    let fx = from_fix_to_float(out_vec.x);
                    let fy = from_fix_to_float(out_vec.y);
                    let fz = from_fix_to_float(out_vec.z);

                    // salvare
                    writeln!(file, "{},{},{},{}", fx, fy, fz, point.intensity)?;
                    corrected_cloud.push((fx, fy, fz));
                    break;
                }
            }
        }
    }

    println!("------------------------------------------------");
    println!("Procesare Finalizată în {:.2?}", start_time.elapsed());
    println!("Puncte Procesate: {} / {}", matched_count, scan_points.len());
    println!("Rezultat salvat în 'data/corrected_cloud.csv'");
    println!("------------------------------------------------");

    Ok(())
}