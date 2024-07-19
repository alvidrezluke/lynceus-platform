use std::io::{Read, Write};
use std::time::Duration;
use serialport::SerialPort;
use crate::error::MaestroError;

pub struct Maestro {
    serial_port: Box<dyn SerialPort>
}

const BAUDRATE: u32 = 9600;

impl Maestro {
    pub fn new(port: &str) -> Result<Self, MaestroError> {
        let sp = serialport::new(port, BAUDRATE).timeout(Duration::from_millis(10)).open();
        return if let Ok(serial_port) = sp {
            Ok(Maestro {
                serial_port,
            })
        } else {
            Err(MaestroError::UnableToConnect)
        }
    }

    pub fn set_accel(&mut self, channel: u8, accel: u16) -> Result<(), MaestroError> {
        verify_channel_range(channel)?;
        let mut data: [u8; 4] = [0x89, channel, 0, 0];
        data[2..].copy_from_slice(&u16_to_u8(accel));
        self.send_command_no_response(&data)?;
        Ok(())
    }
    pub fn set_speed(&mut self, channel: u8, speed: u16) -> Result<(), MaestroError> {
        verify_channel_range(channel)?;
        let mut data: [u8; 4] = [0x87, channel, 0, 0];
        data[2..].copy_from_slice(&u16_to_u8(speed));
        self.send_command_no_response(&data)?;
        Ok(())
    }
    pub fn set_pos(&mut self, channel: u8, pos: u16) -> Result<(), MaestroError> {
        verify_channel_range(channel)?;
        let mut data: [u8; 4] = [0x84, channel, 0, 0];
        data[2..].copy_from_slice(&u16_to_u8(pos));
        self.send_command_no_response(&data)?;
        Ok(())
    }

    pub fn get_pos(&mut self, channel: u8) -> Result<i32, MaestroError> {
        verify_channel_range(channel)?;
        self.send_command(&[0x90, channel])
    }

    pub fn get_moving_state(&mut self, channel: u8) -> Result<i32, MaestroError> {
        verify_channel_range(channel)?;
        self.send_command(&[0x93, channel])
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

fn u16_to_u8(input: u16) -> [u8; 2] {
    [ (input & 0x7F) as u8, (input >> 7 & 0x7F) as u8 ]
}

const MAXCHANNEL: u8 = 11;

fn verify_channel_range(channel: u8) -> Result<(), MaestroError> {
    return if channel > MAXCHANNEL {
        Err(MaestroError::InvalidChannel)
    } else {
        Ok(())
    }
}