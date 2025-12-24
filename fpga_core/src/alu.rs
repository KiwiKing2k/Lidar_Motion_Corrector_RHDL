use rhdl::prelude::*;
use crate::types::*;

// inmultire cu shiftare pe literal
#[kernel]
pub fn fixed_mul(a: Fixed32, b: Fixed32) -> Fixed32 {
    // extindem la 64 biti
    let a_wide: s64 = a.resize();
    let b_wide: s64 = b.resize();

    // calculam produsul
    let prod = a_wide * b_wide;

    // shiftam cu 16 la stanga si taiem inapoi la marimea initiala
    (prod >> 16).resize()
}

// adunare vectoriala
#[kernel]
pub fn vector_add(v1: Vector3, v2: Vector3) -> Vector3 {
    Vector3 {
        x: v1.x + v2.x,
        y: v1.y + v2.y,
        z: v1.z + v2.z,
    }
}

// matrice * vector
#[kernel]
pub fn matrix_vector_mult(m: Matrix3x3, v: Vector3) -> Vector3 {
    // calculam x
    let x = fixed_mul(m.rows[0][0], v.x) +
        fixed_mul(m.rows[0][1], v.y) +
        fixed_mul(m.rows[0][2], v.z);

    // calculam y
    let y = fixed_mul(m.rows[1][0], v.x) +
        fixed_mul(m.rows[1][1], v.y) +
        fixed_mul(m.rows[1][2], v.z);

    // calculam z
    let z = fixed_mul(m.rows[2][0], v.x) +
        fixed_mul(m.rows[2][1], v.y) +
        fixed_mul(m.rows[2][2], v.z);

    Vector3 { x, y, z }
}