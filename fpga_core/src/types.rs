use rhdl::prelude::*;

// aliasul e pentru ca poate facem overflow idk
pub type Fixed32 = s32;

// pt impartire
pub const FRAC_BITS: u32 = 16;

// structurile de date
#[derive(PartialEq, Debug, Digital, Default)]
pub struct Vector3 {
    pub x: Fixed32,
    pub y: Fixed32,
    pub z: Fixed32,
}

#[derive(PartialEq, Debug, Digital, Default)]
pub struct Matrix3x3 {
    // array suportat nativ
    pub rows: [[Fixed32; 3]; 3],
}

// i/o interface

#[derive(PartialEq, Debug, Digital, Default)]
pub struct LidarInput {
    pub valid: bool,
    pub point: Vector3,
    pub rotation: Matrix3x3,
    pub translation: Vector3,
}

#[derive(PartialEq, Debug, Digital, Default)]
pub struct LidarOutput {
    // sunt date valide pentru output
    pub valid: bool,
    pub corrected_point: Vector3,
}