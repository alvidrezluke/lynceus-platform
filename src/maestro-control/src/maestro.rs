use std::io::{Read, Write};
use std::time::Duration;
use serialport::SerialPort;
use crate::error::MaestroError;

/// Core of control program. Stores the serial port connection to pass to all other values.
///
/// # Example:
/// ```
/// use maestro_control::Maestro;
/// let mut maestro = Maestro::new("COM1").unwrap();
/// ```
pub struct Maestro {
    serial_port: Box<dyn SerialPort>
}

const BAUD_RATE: u32 = 9600;

impl Maestro {
    /// Opens the Maestro at the given serial port.
    ///
    /// `port` should be a valid serial port.
    ///
    /// Ports are opened in exclusive mode and are not released until the `Maestro` instance is dropped.
    /// # Errors
    /// - `UnableToConnect` if serial connection was unable to be established.
    pub fn new(port: &str) -> Result<Self, MaestroError> {
        let sp = serialport::new(port, BAUD_RATE).timeout(Duration::from_millis(10)).open();
        return if let Ok(serial_port) = sp {
            Ok(Maestro {
                serial_port,
            })
        } else {
            Err(MaestroError::UnableToConnect)
        }
    }

    /// Sets the acceleration of a single channel.
    ///
    /// `channel` should be a valid channel < 12.
    /// # Errors:
    /// - `InvalidChannel` if channel is out of range
    /// - `UnableToSend` if serial port was unable to send command to Maestro
    pub fn set_acceleration(&mut self, channel: u8, accel: u16) -> Result<(), MaestroError> {
        verify_channel_range(channel)?;
        let mut data: [u8; 4] = [0x89, channel, 0, 0];
        data[2..].copy_from_slice(&u16_to_u8(accel));
        self.send_command_no_response(&data)
    }

    /// Sets the speed of a single channel.
    ///
    /// `channel` should be a valid channel < 12.
    /// # Errors:
    /// - `InvalidChannel` if channel is out of range
    /// - `UnableToSend` if serial port was unable to send command to Maestro
    pub fn set_speed(&mut self, channel: u8, speed: u16) -> Result<(), MaestroError> {
        verify_channel_range(channel)?;
        let mut data: [u8; 4] = [0x87, channel, 0, 0];
        data[2..].copy_from_slice(&u16_to_u8(speed));
        self.send_command_no_response(&data)
    }

    /// Sets the position of a single channel.
    ///
    /// `channel` should be a valid channel < 12.
    /// # Errors:
    /// - `InvalidChannel` if channel is out of range
    /// - `UnableToSend` if serial port was unable to send command to Maestro
    pub fn set_position(&mut self, channel: u8, position: u16) -> Result<(), MaestroError> {
        verify_channel_range(channel)?;
        let mut data: [u8; 4] = [0x84, channel, 0, 0];
        data[2..].copy_from_slice(&u16_to_u8(position));
        self.send_command_no_response(&data)
    }

    /// Gets the position of a single channel.
    ///
    /// `channel` should be a valid channel < 12.
    /// # Errors:
    /// - `InvalidChannel` if channel is out of range
    /// - `UnableToSend` if serial port was unable to send command to Maestro
    /// - `UnableToReceive` if Maestro sends back invalid data
    pub fn get_position(&mut self, channel: u8) -> Result<i32, MaestroError> {
        verify_channel_range(channel)?;
        self.send_command(&[0x90, channel])
    }

    /// Set the accelerations of all channels in vector.
    ///
    /// `channels` should be a vector of valid channels < 12.
    /// # Errors:
    /// - `InvalidChannel` if channel is out of range
    /// - `UnableToSend` if serial port was unable to send command to Maestro
    pub fn set_accelerations(&mut self, channels: Vec<u8>, accels: Vec<u16>) -> Result<(), MaestroError> {
        for (channel, accel) in channels.into_iter().zip(accels.into_iter()) {
            self.set_acceleration(channel, accel)?;
        }
        Ok(())
    }

    /// Sets the speeds of all channels in vector.
    ///
    /// `channels` should be a vector of valid channels < 12.
    /// # Errors:
    /// - `InvalidChannel` if channel is out of range
    /// - `UnableToSend` if serial port was unable to send command to Maestro
    pub fn set_speeds(&mut self, channels: Vec<u8>, speeds: Vec<u16>) -> Result<(), MaestroError> {
        for (channel, speed) in channels.into_iter().zip(speeds.into_iter()) {
            self.set_speed(channel, speed)?;
        }
        Ok(())
    }

    /// Sets the positions of all channels in vector.
    ///
    /// `channels` should be a vector of valid channels < 12.
    /// # Errors:
    /// - `InvalidChannel` if channel is out of range
    /// - `UnableToSend` if serial port was unable to send command to Maestro
    pub fn set_positions(&mut self, channels: Vec<u8>, positions: Vec<u16>) -> Result<(), MaestroError> {
        for (channel, pos) in channels.into_iter().zip(positions.into_iter()) {
            self.set_position(channel, pos)?;
        }
        Ok(())
    }

    /// Gets the positions of all channels in vector.
    ///
    /// `channels` should be a vector of valid channels < 12.
    /// # Errors:
    /// - `InvalidChannel` if channel is out of range
    /// - `UnableToSend` if serial port was unable to send command to Maestro
    /// - `UnableToReceive` if Maestro sends back invalid data
    pub fn get_pos_motors(&mut self, channels: Vec<u8>) -> Result<Vec<i32>, MaestroError> {
        let mut motor_positions: Vec<i32> = Vec::with_capacity(channels.len());
        for channel in channels {
            motor_positions.push(self.get_position(channel)?);
        }
        Ok(motor_positions)
    }

    /// Check if any of the servos are currently moving.
    ///
    /// Returns `MovingState::ServosMoving` if any servos are currently moving, otherwise returning `MovingState::ServoStopped`.
    ///
    /// # Error
    /// - `UnableToSend` if serialport was unable to send the command to Maestro.
    pub fn get_moving_state(&mut self) -> Result<MovingState, MaestroError> {
        let res = self.send_command(&[0x93])?;
        return match res {
            0 => Ok(MovingState::ServosStopped),
            1 => Ok(MovingState::ServosMoving),
            _ => Err(MaestroError::InvalidMovingState)
        }
    }

    fn send_command_no_response(&mut self, data: &[u8]) -> Result<(), MaestroError> {
        let res = self.serial_port.write(data);
        if res.is_err() {
            return Err(MaestroError::UnableToSend);
        }
        Ok(())
    }

    fn send_command(&mut self, data: &[u8]) -> Result<i32, MaestroError> {
        let res = self.serial_port.write(data);
        if res.is_err() {
            return Err(MaestroError::UnableToSend);
        }
        let buf: &mut[u8; 2] = &mut [0; 2];
        let r = self.serial_port.read_exact(buf);
        if let Err(_) = r {
            return Err(MaestroError::UnableToReceive)
        }
        Ok(buf[0] as i32 + 256 * buf[1] as i32)
    }
}

/// Returned enum based on current servo status.
///
/// # Example:
/// ```
/// use maestro_control::{Maestro, MovingState};
///
/// let mut maestro = Maestro::new("COM1").unwrap();
/// match maestro.get_moving_state() {
///     Ok(MovingState::ServosMoving) => println!("Servos still moving!"),
///     Ok(MovingState::ServosStopped) => println!("All servos have stopped moving!"),
///     Err(e) => println!("{:?}", e),
/// }
/// ```
pub enum MovingState {
    /// Is returned if servos are still moving
    ServosMoving,
    /// Is returned if all servos have stopped moving
    ServosStopped
}

fn u16_to_u8(input: u16) -> [u8; 2] {
    [ (input & 0x7F) as u8, (input >> 7 & 0x7F) as u8 ]
}

const MAX_CHANNEL: u8 = 11;

fn verify_channel_range(channel: u8) -> Result<(), MaestroError> {
    return if channel > MAX_CHANNEL {
        Err(MaestroError::InvalidChannel)
    } else {
        Ok(())
    }
}