use gilrs::{Gilrs, EventType, Axis};
use std::f64::consts::PI;

fn main() {
    let mut gilrs = Gilrs::new().unwrap();

    // Iterate over all connected gamepads
    for (_id, gamepad) in gilrs.gamepads() {
    //    println!("{} is {:?}", gamepad.name(), gamepad.power_info());
    }

    let mut last_left_joy_pos: [f64; 2] = [0.0, 0.0];

    loop {
        while let Some(event) =  gilrs.next_event() {
            //println!("{:?}", event);
            match event.event {
                EventType::AxisChanged(joy_axis, joy_value, joy_code) =>
                    match joy_axis {
                        Axis::LeftStickX => last_left_joy_pos[0] = joy_value as f64,
                        Axis::LeftStickY => last_left_joy_pos[1] = joy_value as f64,
                        _ => (),

                    }
                _ => (),
            }

            if last_left_joy_pos[0] > 1.0 { last_left_joy_pos[0] = 1.0};
            if last_left_joy_pos[0] < -1.0 { last_left_joy_pos[0] = -1.0};
            if last_left_joy_pos[1] > 1.0 { last_left_joy_pos[1] = 1.0};
            if last_left_joy_pos[1] < -1.0 { last_left_joy_pos[1] = -1.0};


            //println!("pos: {:?}", last_left_joy_pos);
            let atan2: f64 = last_left_joy_pos[0].atan2(last_left_joy_pos[1])+PI;
            //println!("atan2: {}", atan2);
            let mut dir :f64 = atan2 * 360.0/2.0*PI/10.0;
            dir = 360.0 - dir;
            if dir <= 180.0 {
                dir = 180.0 + dir;
            } else {
                dir = dir - 180.0;
            }
            let dir_u16 :u16 = dir as u16;

            let power_u8 :u8 = ( ((last_left_joy_pos[0].powi(2) + last_left_joy_pos[1].powi(2)).sqrt()) * 255.0 )as u8 ;

            println!("{} {}", dir_u16, power_u8);//0~360
        }
    }
}

