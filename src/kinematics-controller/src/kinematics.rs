use std::ops::Sub;
use ndarray::{arr2, array, Array1, Array2};
use libm::{acos, atan2};
use rust_decimal::Decimal;
use rust_decimal::prelude::ToPrimitive;
use crate::errors::MathError;

/// (X, Y, Z)
pub struct Point(Decimal, Decimal, Decimal);

/// (Roll, Pitch, Yaw) In radians
pub struct Orientation(Decimal, Decimal, Decimal);

pub enum MotorId {
    One,
    Two,
    Three,
    Four,
    Five,
    Six,
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
    pub fn new(position: Point, direction: Direction) -> Self{
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
    top_leg_length: Decimal,
    bottom_leg_length: Decimal,
    motors:[Motor;6],
}

impl Sub for Point {
    type Output = Point;

    fn sub(self, rhs: Self) -> Self::Output {
        Point(self.0 - rhs.0, self.1 - rhs.1, self.2 - rhs.2)
    }
}

/// Inverse kinematics for modified Stewart platform (Movement)
impl Kinematics{
    pub fn new(top_leg_length:Decimal, bottom_leg_length:Decimal, motors:[Motor;6]) -> Self{
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
    fn calc_rot_matrix(alpha: Decimal, beta: Decimal, gamma: Decimal) -> Array2<Decimal> {
        arr2(&[
            [
                (alpha.cos() * beta.cos()).round_dp(4),
                (alpha.cos() * beta.sin() * gamma.sin() - gamma.sin() * alpha.sin()).round_dp(4),
                (alpha.sin() * gamma.sin() + alpha.cos() * gamma.cos() * beta.sin()).round_dp(4),
            ],
            [
                (beta.cos() * alpha.sin()).round_dp(4),
                (alpha.cos() * gamma.cos() + alpha.sin() * beta.sin() * gamma.sin()).round_dp(4),
                (gamma.cos() * alpha.sin() * beta.sin() - alpha.cos() * gamma.sin()).round_dp(4),
            ],
            [
                (-beta.sin()).round_dp(4),
                (beta.cos() * gamma.sin()).round_dp(4),
                (beta.cos() * gamma.cos()).round_dp(4),
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
        let ep: Array1<f64> = array![
        end_pos.0.to_f64().unwrap(),
        end_pos.1.to_f64().unwrap(),
        end_pos.2.to_f64().unwrap()
    ];
        let temp: Array1<f64> = array![ep[0], ep[1]];
        let phi: f64 = (l2_norm(ep.clone())
            + (self.bottom_leg_length.powf(2.0) - self.top_leg_length.powf(2.0)))
            / (2.0 * self.bottom_leg_length * l2_norm(temp));
        return match dir {
            Direction::Left => atan2(ep[2], ep[1]) + acos(phi % 1.0),
            Direction::Right => atan2(ep[2], ep[1]) - acos(phi % 1.0),
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
    pub fn inverse_kinematics(&self, target_pos: Point, target_orientation: Orientation, platform:Platform) -> Array1<f64> {
        // get new orientation
        let platform_ore:Array2<Decimal> = Self::calc_rot_matrix(target_orientation.0, target_orientation.1, target_orientation.2);
        // get new corners
        // for every corner get the new target position of the end effector
        // calculate new servo position for motor
        // return new angles
        let angles:[Point; 6] = platform.arm_positions.iter().zip(&self.motors).map(|(dist, motor)| {
            // fix to multiply against rotation matrix
            let end_pos = target_pos - dist.copy();
            self.calc_servo_pos(end_pos, motor.get_direction());
        }).collect();
        angles
    }

}

/*
/// Gets Euclidean distance of a vector
    x: vector
*/
fn l2_norm(x: Array1<f64>) -> f64 {
    x.dot(&x).sqrt()
}

fn point_to_array(point: Point) -> Result<Array1<f64>, MathError>{
    array![
        point.0.to_f64().Err(MathError::InvalidFloatConversion),
        point.1.to_f64().Err(MathError::InvalidFloatConversion),
        point.2.to_f64().Err(MathError::InvalidFloatConversion),
    ];
}

// fn decimal_to_f64(num: Decimal) -> Result<f64, MathError>{
//     let result = num.to_f64();
//     if result.is_err{
//         return Err(MathError::InvalidFloatConversion);
//     }
//     Ok(result)
// }

