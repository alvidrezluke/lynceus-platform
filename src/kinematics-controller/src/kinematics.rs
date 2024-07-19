use rust_decimal::Decimal;
struct Point(Decimal, Decimal, Decimal);
pub struct Kinematics{
    top_leg_length: Decimal,
    bottom_leg_length: Decimal,
    motor_positions:[Point;6],
}

// impl Default for Kinematics {
//     fn default() -> Self {
//         Self::new();
//     }
// }

impl Kinematics{
    pub fn new(top_length:Decimal, bottom_length:Decimal, motor_pos:[Point;6]) -> Self{
        Self{
            top_leg_length: top_length,
            bottom_leg_length: bottom_length,
            motor_positions: motor_pos,
        }
    }
}


