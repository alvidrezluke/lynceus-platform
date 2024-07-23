use thiserror::Error;

#[derive(Error, Debug)]
pub enum KinematicsError{
    #[error("Target position is not possible")]
    InvalidTargetPosition,
    #[error("Target orientation is not possible")]
    InvalidTargetOrientation,

}

#[derive(Error, Debug)]
pub enum MathError{
    #[error("Converting Decimal to Float 64 has failed")]
    InvalidFloatConversion,
    #[error("Error at angle")]
    InvalidAngle,
}