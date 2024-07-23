use thiserror::Error;

#[derive(Error, Debug)]
pub enum MaestroError {
    #[error("Unable to connect to Maestro!")]
    UnableToConnect,
    #[error("Lost connection to Maestro!")]
    UnableToSend,
    #[error("Invalid channel parameter passed! Valid parameters are 0-11")]
    InvalidChannel,
    #[error("Unable to receive date!")]
    UnableToReceive,
    #[error("Invalid moving state received from Maestro. Value should be 0 or 1")]
    InvalidMovingState,
    #[error("Input out of bounds")]
    OutOfBounds
}