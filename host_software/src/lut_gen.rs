use crate::data_loader::ImuMeasurement;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use std::collections::BTreeMap;

// Structura finala a POSE-ului pt fpga
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub timestamp_ns: u64,
    pub rotation: UnitQuaternion<f64>,
    pub translation: Vector3<f64>,
}

// 1. Calcularea traiectoriei ca vector continuu din datele IMU
pub fn calculate_trajectory(imu_data: &[ImuMeasurement]) -> Vec<Pose> {
    let mut trajectory = Vec::new();

    let mut position = Vector3::new(0.0, 0.0, 0.0);
    let mut velocity = Vector3::new(0.0, 0.0, 0.0);
    let mut rotation = UnitQuaternion::identity();

    // vectorul gravitational trebuie scos din masuratori
    let gravity = Vector3::new(0.0, 0.0, 9.81);

    if imu_data.is_empty() {
        return trajectory;
    }

    let mut prev_time = imu_data[0].timestamp_ns;

    for measurement in imu_data {
        let current_time = measurement.timestamp_ns;

        // calcul delta t (în secunde)
        let dt_ns = current_time.saturating_sub(prev_time);
        if dt_ns == 0 { continue; } // trecem peste duplicate
        let dt = dt_ns as f64 * 1e-9;

        // vectorul viteza unghiulara
        let ang_vel = Vector3::new(
            measurement.ang_vel_x,
            measurement.ang_vel_y,
            measurement.ang_vel_z
        );

        // Updatam rotatia curenta ca si quaternion
        let delta_rot = UnitQuaternion::from_scaled_axis(ang_vel * dt);
        rotation = rotation * delta_rot;


        // calculam acceleratia reala ca SI minus gravitatea pentru a trece in SNI
        let acc_local = Vector3::new(
            measurement.acc_x,
            measurement.acc_y,
            measurement.acc_z
        );
        let acc_global = rotation * acc_local - gravity;

        // v = v0 + a * dt
        velocity += acc_global * dt;
        // p = p0 + v * dt
        position += velocity * dt;

        // salvam starea noua
        trajectory.push(Pose {
            timestamp_ns: current_time,
            rotation,
            translation: position,
        });

        prev_time = current_time;
    }

    trajectory
}

// 2. traiectoria devine un B-tree (binary tree dar poate avea mai mult de 2 copii)
// are log n complexitate la search
pub fn generate_pose_lut(trajectory: &[Pose]) -> BTreeMap<u64, Pose> {
    let mut lut = BTreeMap::new();
    for pose in trajectory {
        lut.insert(pose.timestamp_ns, *pose);
    }
    lut
}

// 3. Interpolare rezultate
// Primeste un timestamp arbitrar (de la LiDAR) si returneaza Pose-ul interpolat (presupus continuu)
pub fn interpolate_pose(lut: &BTreeMap<u64, Pose>, query_ts: u64) -> Option<Pose> {
    // Cautăm vecinii
    let before = lut.range(..=query_ts).next_back();
    let after = lut.range(query_ts..).next();

    match (before, after) {
        (Some((_, p1)), Some((_, p2))) => {
            let t1 = p1.timestamp_ns;
            let t2 = p2.timestamp_ns;

            // daca timestamp-urile sunt identice, am gasit rezultatul neinterpolat
            if t2 == t1 { return Some(*p1); }

            // calculam factorul de interpolare 'alpha' (0.0 -> 1.0)
            let alpha = (query_ts - t1) as f64 / (t2 - t1) as f64;

            // SLERP pentru rotație (Spherical Linear Interpolation)
            let rot_interp = p1.rotation.slerp(&p2.rotation, alpha);

            // LERP pentru poziție (Linear Interpolation)
            let pos_interp = p1.translation.lerp(&p2.translation, alpha);

            return Some(Pose {
                timestamp_ns: query_ts,
                rotation: rot_interp,
                translation: pos_interp,
            })
        },
        // edgecases (început sau sfârșit de dataset)
        (Some((_, p1)), None) => Some(*p1),
        (None, Some((_, p2))) => Some(*p2),
        //iubim Option<>
        _ => None,
    }
}