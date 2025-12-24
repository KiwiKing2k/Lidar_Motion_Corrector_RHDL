use rhdl::prelude::*;
use rhdl_fpga::core::dff::DFF;

// semnalele de control care pleaca
#[derive(PartialEq, Debug, Digital, Default)]
pub struct ControlSignals {
    pub load_input: bool,   // permite scrierea datelor de intrare in registre
    pub save_temp: bool,    // permite salvarea rezultatului intermediar (rotatia)
    pub output_valid: bool, // semnalizeaza ca rezultatul final e gata
}

// starile automatului finit
#[derive(PartialEq, Debug, Digital, Default)]
pub enum State {
    #[default]
    Idle,       // asteapta semnalul valid de la host
    Load,       // incarca datele in registrele interne
    CalcRot,    // executa rotatia si salveaza in temp
    CalcTrans,  // executa translatia (combinational) si emite output
}

// componenta hardware cu registru de stare
#[derive(Synchronous, SynchronousDQ, Clone, Debug)]
pub struct ControlUnit {
    state: DFF<State>,
}

impl Default for ControlUnit {
    fn default() -> Self {
        Self {
            state: DFF::new(State::Idle),
        }
    }
}

// interfata io
impl SynchronousIO for ControlUnit {
    type I = bool;           // input: semnalul 'valid'
    type O = ControlSignals; // output: comenzile
    type Kernel = cu_kernel;
}

// logica de tranzitie
#[kernel]
pub fn cu_kernel(_cr: ClockReset, valid_in: bool, q: Q) -> (ControlSignals, D) {
    let mut cs = ControlSignals::default();
    let mut next_state = q.state;

    match q.state {
        State::Idle => {
            if valid_in {
                next_state = State::Load;
            }
        },
        State::Load => {
            cs.load_input = true;
            next_state = State::CalcRot;
        },
        State::CalcRot => {
            cs.save_temp = true;
            next_state = State::CalcTrans;
        },
        State::CalcTrans => {
            cs.output_valid = true;
            next_state = State::Idle;
        }
    }

    (cs, D { state: next_state })
}