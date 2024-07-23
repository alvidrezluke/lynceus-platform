use std::ops::Sub;
use ndarray::{arr2, Array, array, Array1, Array2, Ix1};
use libm::{acos, atan2};
use rust_decimal::{Decimal, MathematicalOps};
use rust_decimal::prelude::{FromPrimitive, ToPrimitive};
use rust_decimal_macros::dec;
use crate::errors::MathError;

/// (X, Y, Z)
pub type Point = Array<f64, Ix1>;

/// (Roll, Pitch, Yaw) In radians
pub type Orientation = Array<f64, Ix1>;

pub enum MotorId {
    Zero,
    One,
    Two,
    Three,
    Four,
    Five,
}

pub enum Direction{
    Left,
    Right,
}

pub struct Motor {
    position: Point,
    motor_id: MotorId,
    direction: Direction,
}

/*
    Motor Structure for Robot
*/
impl Motor{
    pub fn new(position: Point, direction: Direction, motor_id: MotorId) -> Self{
        Self{
            position,
            motor_id,
            direction,
        }
    }

    pub fn get_position(&self) -> &Point{
        &self.position
    }
    pub fn get_direction(&self) -> &Direction{
        &self.direction
    }
}

/*
    Platform Structure for Robot
*/
pub struct Platform{
    center: Point,
    arm_positions: [Point; 6],
}

/*
    Platform Structure for Robot
*/
pub struct Kinematics{
    top_leg_length: f64,
    bottom_leg_length: f64,
    motors:[Motor;6],
}

/// Inverse kinematics for modified Stewart platform (Movement)
impl Kinematics{
    pub fn new(top_leg_length:f64, bottom_leg_length:f64, motors:[Motor;6]) -> Self{
        Self{
            top_leg_length,
            bottom_leg_length,
            motors,
        }
    }

    /*
    /// Calculates the rotation matrix for platform
    /// Pre derived
     gamma: angle of roll
     beta : angle of pitch
     alpha: angle of yaw
    */
    pub fn calc_rot_matrix(&self, gamma: f64, beta: f64, alpha: f64) -> Array2<f64> {
        let gamma_dec = Decimal::from_f64(gamma).unwrap();
        let beta_dec =  Decimal::from_f64(beta).unwrap();
        let alpha_dec =  Decimal::from_f64(alpha).unwrap();

        arr2(&[
            [
                (alpha_dec.cos() * beta_dec.cos()).round_dp(4).to_f64().unwrap(),
                (alpha_dec.cos() * beta_dec.sin() * gamma_dec.sin() - gamma_dec.cos() * alpha_dec.sin()).round_dp(4).to_f64().unwrap(),
                (alpha_dec.sin() * gamma_dec.sin() + alpha_dec.cos() * gamma_dec.cos() * beta_dec.sin()).round_dp(4).to_f64().unwrap(),
            ],
            [
                (beta_dec.cos() * alpha_dec.sin()).round_dp(4).to_f64().unwrap(),
                (alpha_dec.cos() * gamma_dec.cos() + alpha_dec.sin() * beta_dec.sin() * gamma_dec.sin()).round_dp(4).to_f64().unwrap(),
                (gamma_dec.cos() * alpha_dec.sin() * beta_dec.sin() - alpha_dec.cos() * gamma_dec.sin()).round_dp(4).to_f64().unwrap(),
            ],
            [
                (-beta_dec.sin()).round_dp(4).to_f64().unwrap(),
                (beta_dec.cos() * gamma_dec.sin()).round_dp(4).to_f64().unwrap(),
                (beta_dec.cos() * gamma_dec.cos()).round_dp(4).to_f64().unwrap(),
            ],
        ])
    }

    /*
        end_pos: end effector desired position
        dir:     Direction of servo
        returns angle needed for servo
    */
    // Todo!():
    // Refactor for new structure
    // Write tests
    fn calc_servo_pos(&self, end_pos: &Point, dir: &Direction) -> f64 {
        // Fix
        // let ep: Array1<f64> = decimal_to_float_array(end_pos);
        let temp: Array1<f64> = array![end_pos[0], end_pos[1]];
        let phi: f64 = (l2_norm(end_pos.clone())
            + (self.bottom_leg_length.powf(2.0) - self.top_leg_length.powf(2.0)))
            / (2.0 * self.bottom_leg_length * l2_norm(temp));
        return match dir {
            Direction::Left => atan2(end_pos[2], end_pos[1]) + acos(phi % 1.0),
            Direction::Right => atan2(end_pos[2], end_pos[1]) - acos(phi % 1.0),
        };
    }

    /*
        /// Computes Inverse kinematics
        target: center of platform
        [x, y, z, roll, pitch, yaw]
        motors: vector of motors
        returns vector of angles

        c = centroid frame
        Given target_pos(x_c, y_c, z_c) and orientation()
        Inverse robot procedure:
        1) find the platform corners in platform frame (p_i^c)
        2) find the platform transformation relative to world frame (T_c^0)
        3) find platform corners in world frame (p_i^0 = T_c^0 p_i^c)
        4) find the Euclidean distance base to platform

        a_i: the vector from the world frame to the pos of motor
        S_i = p_c + R_c * b_i - a_i
*/
    /*
        Calculate the all angles needed for all motors
     */
    pub fn inverse_kinematics(&self, target_pos: Point, target_orientation: Orientation, platform:Platform) -> Vec<()> {
        println!("{:?}", target_orientation[0]);
        // get new orientation
        let platform_ore:Array2<f64> = self.calc_rot_matrix(target_orientation[0], target_orientation[1], target_orientation[2]);
        // get new corners
        // for every corner get the new target position of the end effector
        // calculate new servo position for motor
        // return new angles
        let distance = self.motors.iter().zip(platform.arm_positions.iter()).map(|(motor, dist)| {
            let dist = &target_pos + platform_ore.dot(dist) - &motor.position;
            // self.calc_servo_pos(&dist, motor.get_direction());
        }).collect::<Vec<_>>();
        println!("Angles: {:?}", distance);
        // for position in platform.arm_positions
        distance
    }

}

/*
/// Gets Euclidean distance of a vector
    x: vector
*/
fn l2_norm(x: Array1<f64>) -> f64 {
    x.dot(&x).sqrt()
}



