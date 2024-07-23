#![warn(missing_docs)]
mod maestro;
mod error;

pub use maestro::Maestro;
pub use maestro::MovingState;


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_maestro_error() {
        assert!(Maestro::new("COM0").is_err())
    }
}
