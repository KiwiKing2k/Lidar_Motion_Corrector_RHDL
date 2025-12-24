use rhdl::prelude::*;
use fpga_core::{LidarProcessor, LidarInput, Vector3, Matrix3x3};

// magie pentru a afisa erorile din acest limbaj criptic
fn miette_report(err: RHDLError) -> String {
    let handler = miette::GraphicalReportHandler::new_themed(miette::GraphicalTheme::unicode_nocolor());
    let mut msg = String::new();
    handler.render_report(&mut msg, &err).unwrap();
    msg
}

// conversie Int -> Fixed Point (format 16.16)
fn tofixedpointvar(v: i32) -> s32 {
    s32::from(v as i128) << 16
}

fn run_simulation() -> Result<(), RHDLError> {
    // instantiem procesorul
    let uut = LidarProcessor::default();

    println!("--- Start Simulare Lidar Processor ---");

    // 2. Definim Scenario-ul de Test: Rotație 90 grade în jurul axei Z
    // Punct intrare: P(10, 0, 0)
    // Translație: T(0, 0, 0)
    // Matrice Rotație Z(90°):
    // [ 0 -1  0 ]
    // [ 1  0  0 ]
    // [ 0  0  1 ]
    // Așteptare matematică: P_out = (0, 10, 0)

    let point_in = Vector3 { x: tofixedpointvar(10), y: tofixedpointvar(0), z: tofixedpointvar(0) };
    let trans = Vector3::default(); // Translație zero

    let mut rows = [[tofixedpointvar(0); 3]; 3];
    rows[0][1] = tofixedpointvar(-1); // -sin(90)
    rows[1][0] = tofixedpointvar(1);  // sin(90)
    rows[2][2] = tofixedpointvar(1);  // 1
    let rot = Matrix3x3 { rows };

    // 3. Construim fluxul de intrări
    let mut inputs = Vec::new();

    // -- Etapa 1: Idle (Reset & Stabilizare) --
    // Trimitem bs pentru 2 cicluri
    for _ in 0..2 {
        inputs.push(LidarInput::default());
    }

    // -- Etapa 2: Impuls de Date (Valid = true) --
    // Trimitem datele reale timp de 1 ciclu de ceas
    inputs.push(LidarInput {
        valid: true,
        point: point_in,
        rotation: rot,
        translation: trans,
    });

    // -- Etapa 3: Procesare (Wait) --
    // Așteptăm ca automatul de stări să treacă prin Load -> CalcRot -> CalcTrans -> Output
    let hold_input = LidarInput {
        valid: false,
        point: point_in,
        rotation: rot,
        translation: trans
    };

    // Așteptăm 10 cicluri
    for _ in 0..10 {
        inputs.push(hold_input);
    }

    // 4. Executăm simularea
    // .with_reset(1) -> resetam la prima stare a automatului
    // .clock_pos_edge(100) -> perioada ceasului
    let stream = inputs.into_iter().with_reset(1).clock_pos_edge(100);

    // obiect VCD pentru colectarea semnalelor
    let vcd = uut.run(stream)?.collect::<Vcd>();

    // salvam rezultatele
    let filename = "lidar_test.vcd";
    println!("Se generează fișierul '{}'...", filename);
    vcd.dump_to_file(filename)?;

    println!("Succes! Verifică rezultatul în GTKWave.");
    println!("Ar trebui să vezi un semnal 'output_valid' activat spre final,");
    println!("iar 'corrected_point' să aibă valoarea aprox X=0, Y=655360 (10.0).");

    Ok(())
}

fn main() {
    // testam rotatia de 90 grade pe z: input (10, 0, 0) -> output (0, 10, 0)
    // in fixed-point, 10.0 = 655360, adica 0xA0000 in hexa
    // in gtkwave cautam secventa 'A0000' in semnalul 'temp_rotated' cand valid=1
    // in ierarhie, cautam top/engine/temp_rotated/dff unde dff reprezinta x,y,z
    // iar y trebuie sa aiba valoarea 0xA0000
    if let Err(e) = run_simulation() {
        println!("{}", miette_report(e));
    }
}