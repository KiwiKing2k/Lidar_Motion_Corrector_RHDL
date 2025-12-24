use rhdl::prelude::*;
use rhdl_fpga::core::dff::DFF;
use crate::types::*;
use crate::alu::*;
use crate::control_unit::ControlSignals;

// structura engine (datapath), echivalentul 'cpu' dar fara control unit inauntru
// contine doar registrele de date (ca t1, t2 din cpu.rs)
#[derive(Synchronous, SynchronousDQ, Clone, Debug)]
pub struct Engine {
    // registre intrare
    pub point: DFF<Vector3>,
    pub rotation: DFF<Matrix3x3>,
    pub translation: DFF<Vector3>,

    // registru intermediar (scoate rezultatul rotatiei ie P_rot = R * P)
    pub temp_rotated: DFF<Vector3>,
}

impl Default for Engine {
    fn default() -> Self {
        Self {
            point: DFF::new(Vector3::default()),
            rotation: DFF::new(Matrix3x3::default()),
            translation: DFF::new(Vector3::default()),
            temp_rotated: DFF::new(Vector3::default()),
        }
    }
}

// interfata io
impl SynchronousIO for Engine {
    type I = (LidarInput, ControlSignals);
    type O = LidarOutput;
    type Kernel = engine_kernel;
}

#[kernel]
pub fn engine_kernel(
    _cr: ClockReset,
    input: (LidarInput, ControlSignals),
    q: Q
) -> (LidarOutput, D) {
    // despachetam intrarea
    let (data_in, cs) = input;

    // 1. definim logica combinationala
    // aici avem doua operatii distincte

    // calculam rotatia curenta: R * P
    let current_rotation_res = matrix_vector_mult(q.rotation, q.point);

    // calculam translatia finala: P_rot + T
    let final_res = vector_add(q.temp_rotated, q.translation);

    // 2. definim starea viitoare (d)
    // initializam d cu valorile curente (hold state)
    let mut next_point = q.point;
    let mut next_rotation = q.rotation;
    let mut next_translation = q.translation;
    let mut next_temp = q.temp_rotated;

    // logica de incarcare
    if cs.load_input {
        next_point = data_in.point;
        next_rotation = data_in.rotation;
        next_translation = data_in.translation;
    }

    // logica de salvare intermediara
    if cs.save_temp {
        next_temp = current_rotation_res;
    }

    // 3. construim iesirea
    let output = LidarOutput {
        valid: cs.output_valid,
        corrected_point: final_res,
    };

    // returnam iesirea si noua stare
    (output, D {
        point: next_point,
        rotation: next_rotation,
        translation: next_translation,
        temp_rotated: next_temp,
    })
}