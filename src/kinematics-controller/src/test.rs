#[cfg(test)]
pub mod tests {
    use std::f64::consts::PI;
    use libm::sqrt;
    use ndarray::{arr1, arr2};
    use crate::kinematics::{Direction, Kinematics, Motor, MotorId, Point};

    #[test]
    fn rotation_matrix_test() {
        let deg_test_1 = arr2(&[[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]]);
        let deg_test_2 = arr2(&[[0.0, -0.7071, 0.7071],[0.5, 0.6124, 0.6124],[-0.8660, 0.3536, 0.3536]]);
        let deg_test_3 = arr2(&[[0.5780, -0.1730, 0.7975],[0.4939, 0.8521, -0.1730],[-0.6496, 0.4939, 0.5780]]);

        let test_motors = [
            Motor::new(arr1(&[28.3, -94.45, 10.0]), Direction::Right, MotorId::Zero),
            Motor::new(arr1( &[95.95, 22.72, 10.0]), Direction::Left, MotorId::One),
            Motor::new(arr1( &[67.65, 71.73, 10.0]), Direction::Right, MotorId::Two),
            Motor::new(arr1( &[-67.65, 71.73, 10.0]), Direction::Left, MotorId::Three),
            Motor::new(arr1( &[-95.95, 22.72, 10.0]), Direction::Right, MotorId::Four),
            Motor::new(arr1( &[-28.3, -94.45, 10.0]), Direction::Left, MotorId::Five),
        ];
        let test = Kinematics::new(119.0, 21.1, test_motors);
        assert_eq!(deg_test_1, test.calc_rot_matrix(0.0, 0.0, 0.0));
        assert_eq!(deg_test_2, test.calc_rot_matrix(PI/4.0, PI/3.0, PI/2.0));
        assert_eq!(deg_test_3, test.calc_rot_matrix(sqrt(2.0)/2.0, sqrt(2.0)/2.0, sqrt(2.0)/2.0));
    }

    #[test]
    fn calc_angle_test(){
        let test_motors = [
            Motor::new(arr1(&[28.3, -94.45, 10.0]), Direction::Right, MotorId::Zero),
            Motor::new(arr1( &[95.95, 22.72, 10.0]), Direction::Left, MotorId::One),
            Motor::new(arr1( &[67.65, 71.73, 10.0]), Direction::Right, MotorId::Two),
            Motor::new(arr1( &[-67.65, 71.73, 10.0]), Direction::Left, MotorId::Three),
            Motor::new(arr1( &[-95.95, 22.72, 10.0]), Direction::Right, MotorId::Four),
            Motor::new(arr1( &[-28.3, -94.45, 10.0]), Direction::Left, MotorId::Five),
        ];
    }
}