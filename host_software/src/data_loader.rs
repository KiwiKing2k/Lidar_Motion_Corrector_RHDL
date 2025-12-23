use serde::Deserialize;
use std::error::Error;
use std::fs::File;
use std::path::Path;

#[derive(Debug, Deserialize)]
pub struct ImuMeasurement {
    pub timestamp_ns: u64,
    pub ang_vel_x: f64,
    pub ang_vel_y: f64,
    pub ang_vel_z: f64,
    pub acc_x: f64,
    pub acc_y: f64,
    pub acc_z: f64,
}

#[derive(Debug, Deserialize)]
pub struct LidarPoint {
    pub timestamp_ns: u64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub intensity: u8,
}

pub fn load_imu_data(path: &str) -> Result<Vec<ImuMeasurement>, Box<dyn Error>> {
    let file_path = Path::new(path);
    let file = File::open(file_path)?;
    let mut rdr = csv::Reader::from_reader(file);
    let mut data = Vec::new();

    for result in rdr.deserialize() {
        data.push(result?);
    }
    Ok(data)
}


// CITIM doar o singură scanare (100ms) și se oprește.
pub fn load_single_scan(path: &str, min_timestamp: u64) -> Result<Vec<LidarPoint>, Box<dyn Error>> {
    let file_path = Path::new(path);
    if !file_path.exists() {
        return Err(format!("Nu gasesc fisierul: {}", path).into());
    }

    let file = File::open(file_path)?;
    let mut rdr = csv::Reader::from_reader(file);
    let mut scan_points = Vec::new();

    let mut start_ts = 0;
    const SCAN_DURATION_NS: u64 = 100_000_000; // 100ms

    println!("Cautam prima scanare valida dupa t={}", min_timestamp);

    for result in rdr.deserialize() {
        let point: LidarPoint = result?;

        // 1. Sărim la inceputul IMU-lui
        if point.timestamp_ns < min_timestamp {
            continue;
        }

        // 2. inceputul imu a fost gasit
        if start_ts == 0 {
            start_ts = point.timestamp_ns;
            println!("Sincronizare reusita! Incepem scanarea la t={}", start_ts);
        }

        // 3. Verificăm dacă a inceput o noua scanare
        if point.timestamp_ns > start_ts + SCAN_DURATION_NS {
            break;
        }

        scan_points.push(point);
    }

    // magie rust
    if scan_points.is_empty() {
        return Err("Nu s-au gasit puncte LiDAR după timestamp.".into());
    }

    scan_points.shrink_to_fit();
    Ok(scan_points)
}