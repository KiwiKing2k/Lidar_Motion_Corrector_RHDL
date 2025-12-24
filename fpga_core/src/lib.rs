use rhdl::prelude::*;

pub mod types;
pub mod alu;
pub mod control_unit;
pub mod engine;

// Importăm doar numele structurilor, NU și Q/D-ul lor intern.
pub use types::{LidarInput, LidarOutput, Vector3, Matrix3x3};
pub use control_unit::{ControlUnit, ControlSignals};
pub use engine::Engine;

#[derive(Synchronous, SynchronousDQ, Clone, Debug)]
pub struct LidarProcessor {
    cu: ControlUnit,
    engine: Engine,
}

impl Default for LidarProcessor {
    fn default() -> Self {
        Self {
            cu: ControlUnit::default(),
            engine: Engine::default(),
        }
    }
}

impl SynchronousIO for LidarProcessor {
    type I = LidarInput;
    type O = LidarOutput;
    type Kernel = top_kernel; // Acum va găsi tipul pentru că macro-ul de mai jos va rula corect
}

// kernelul este practic wiring
// Q-ul de aici este STRICT cel al LidarProcessor, nu cel importat din control_unit.
#[kernel]
pub fn top_kernel(_cr: ClockReset, input: LidarInput, q: Q) -> (LidarOutput, D) {

    // initializam structura de intrari (D) pentru componente
    let mut d = D::dont_care();

    // conectam Control Unit
    // Input-ul lui este doar semnalul valid
    d.cu = input.valid;

    // conectam engine
    // input-ul lui este (Date Lidar, Comenzi)
    // q.cu reprezintă iesirea curentă a unității de control (de tip ControlSignals)
    d.engine = (input, q.cu);

    // iesirea Sistemului
    let output = q.engine;

    (output, d)
}